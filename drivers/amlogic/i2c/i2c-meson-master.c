/*
 * drivers/amlogic/i2c/i2c-meson-master.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/types.h>

/* Meson I2C register map */
#define REG_CTRL					0x00
#define REG_SLAVE_ADDR				0x04
#define REG_TOK_LIST0				0x08
#define REG_TOK_LIST1				0x0c
#define REG_TOK_WDATA0				0x10
#define REG_TOK_WDATA1				0x14
#define REG_TOK_RDATA0				0x18
#define REG_TOK_RDATA1				0x1c

/* Control register fields */
#define REG_CTRL_START				BIT(0)
#define REG_CTRL_ACK_IGNORE			BIT(1)
#define REG_CTRL_STATUS				BIT(2)
#define REG_CTRL_ERROR				BIT(3)
#define REG_CTRL_CLKDIV_SHIFT		12
#define REG_CTRL_CLKDIV_MASK		GENMASK(21, 12)
#define REG_CTRL_CLKDIVEXT_SHIFT	28
#define REG_CTRL_CLKDIVEXT_MASK		GENMASK(29, 28)
#define REG_FULL_MASK				GENMASK(31, 0)

#define I2C_TIMEOUT_MS				500
#define DEFAULT_FREQ				100000

enum {
	TOKEN_END = 0,
	TOKEN_START,
	TOKEN_SLAVE_ADDR_WRITE,
	TOKEN_SLAVE_ADDR_READ,
	TOKEN_DATA,
	TOKEN_DATA_LAST,
	TOKEN_STOP,
};

enum {
	STATE_IDLE,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP,
};

struct meson_i2c_data {
	unsigned char  delay_ajust;
	unsigned char div_factor;
};

/**
 * struct meson_i2c - Meson I2C device private data
 *
 * @adap:	I2C adapter instance
 * @dev:	Pointer to device structure
 * @regs:	Base address of the device memory mapped registers
 * @clk:	Pointer to clock structure
 * @irq:	IRQ number
 * @msg:	Pointer to the current I2C message
 * @state:	Current state in the driver state machine
 * @last:	Flag set for the last message in the transfer
 * @count:	Number of bytes to be sent/received in current transfer
 * @pos:	Current position in the send/receive buffer
 * @error:	Flag set when an error is received
 * @lock:	To avoid race conditions between irq handler and xfer code
 * @done:	Completion used to wait for transfer termination
 * @frequency:	Operating frequency of I2C bus clock
 * @tokens:	Sequence of tokens to be written to the device
 * @num_tokens:	Number of tokens
 * @retain_fast_mode: low frequency using high mode,for less than 100k
 */
struct meson_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	void __iomem		*regs;
	struct clk		*clk;
	int			irq;

	struct i2c_msg		*msg;
	int			state;
	bool			last;
	int			count;
	int			pos;
	int			error;

	spinlock_t		lock;
	struct completion	done;
	unsigned int		frequency;
	u32			tokens[2];
	int			num_tokens;

	int retain_fastmode;
	struct meson_i2c_data *data;
};

static void meson_i2c_set_mask(struct meson_i2c *i2c, int reg, u32 mask,
			       u32 val)
{
	u32 data;

	data = readl(i2c->regs + reg);
	data &= ~mask;
	data |= val & mask;
	writel(data, i2c->regs + reg);
}

static void meson_i2c_reset_tokens(struct meson_i2c *i2c)
{
	i2c->tokens[0] = 0;
	i2c->tokens[1] = 0;
	i2c->num_tokens = 0;
}

static void meson_i2c_add_token(struct meson_i2c *i2c, int token)
{
	if (i2c->num_tokens < 8)
		i2c->tokens[0] |= (token & 0xf) << (i2c->num_tokens * 4);
	else
		i2c->tokens[1] |= (token & 0xf) << ((i2c->num_tokens % 8) * 4);

	i2c->num_tokens++;
}

static void meson_i2c_write_tokens(struct meson_i2c *i2c)
{
	writel(i2c->tokens[0], i2c->regs + REG_TOK_LIST0);
	writel(i2c->tokens[1], i2c->regs + REG_TOK_LIST1);
}

/*
 * Count = clk/freq  = H + L
 * Duty  = H/(H + L) = 1/2	-- duty 50%
 * H = n + delay
 * L = 2m
 *
 * =>
 *
 * n = Count/2 - delay
 * m = Count/4
 * Standard Mode : 100k
 */
static void meson_i2c_set_clk_div_std(struct meson_i2c *i2c)
{
	unsigned long clk_rate = clk_get_rate(i2c->clk);
	unsigned int div_h, div_l;
	unsigned int div_temp;

	div_temp = DIV_ROUND_UP(clk_rate, i2c->frequency);
	div_h = DIV_ROUND_UP(div_temp, 2) - i2c->data->delay_ajust;
	div_l = DIV_ROUND_UP(div_temp, 4);

	/* clock divider has 12 bits */
	if (div_h >= (1 << 12)) {
		dev_err(i2c->dev, "requested bus frequency too low\n");
		div_h = (1 << 12) - 1;
	}

	if (div_l >= (1 << 12)) {
		dev_err(i2c->dev, "requested bus frequency too low\n");
		div_l = (1 << 12) - 1;
	}


	/*control reg:12-21 bits*/
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_CLKDIV_MASK,
			   (div_h & GENMASK(9, 0)) << REG_CTRL_CLKDIV_SHIFT);
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_CLKDIVEXT_MASK,
			   (div_h >> 10) << REG_CTRL_CLKDIVEXT_SHIFT);


	/* set SCL low delay */
	meson_i2c_set_mask(i2c, REG_SLAVE_ADDR, GENMASK(27, 16),
		(div_l << 16) & GENMASK(27, 16));

	/* enable to control SCL low time */
	meson_i2c_set_mask(i2c, REG_SLAVE_ADDR, BIT(28), BIT(28));

	dev_dbg(i2c->dev, "%s: clk %lu, freq %u, div_h %u, div_l %u\n",
		__func__, clk_rate, i2c->frequency, div_h, div_l);
}

/*
 * According to I2C-BUS Spec 2.1, in FAST-MODE, LOW period should be at
 * least 1.3uS, and HIGH period should be at lease 0.6. HIGH to LOW
 * ratio as 2 to 5 is more safe.
 * Duty  = H/(H + L) = 2/5	-- duty 40%%   H/L = 2/3
 * Fast Mode : 400k
 * High Mode : 3400k
 */
static void meson_i2c_set_clk_div_fast(struct meson_i2c *i2c)
{
	unsigned long clk_rate = clk_get_rate(i2c->clk);
	unsigned int div_h, div_l;
	unsigned int div_temp;

	div_temp = DIV_ROUND_UP(clk_rate * 2, i2c->frequency * 5);
	div_h = div_temp - i2c->data->delay_ajust;
	div_l = DIV_ROUND_UP(clk_rate * 3, i2c->frequency * 10);

	/* clock divider has 12 bits */
	if (div_h >= (1 << 12)) {
		dev_err(i2c->dev, "requested bus frequency too low\n");
		div_h = (1 << 12) - 1;
	}

	if (div_l >= (1 << 12)) {
		dev_err(i2c->dev, "requested bus frequency too low\n");
		div_l = (1 << 12) - 1;
	}

	/*control reg:12-21 bits*/
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_CLKDIV_MASK,
			   (div_h & GENMASK(9, 0)) << REG_CTRL_CLKDIV_SHIFT);
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_CLKDIVEXT_MASK,
			   (div_h >> 10) << REG_CTRL_CLKDIVEXT_SHIFT);


	/* set SCL low delay */
	meson_i2c_set_mask(i2c, REG_SLAVE_ADDR, GENMASK(27, 16),
		(div_l << 16) & GENMASK(27, 16));

	/* enable to control SCL low time */
	meson_i2c_set_mask(i2c, REG_SLAVE_ADDR, BIT(28), BIT(28));

	dev_dbg(i2c->dev, "%s: clk %lu, freq %u, div_h %u, div_l %u\n",
		__func__, clk_rate, i2c->frequency, div_h, div_l);
}

static void meson_i2c_set_clk_div(struct meson_i2c *i2c)
{
	if (i2c->frequency >= 400000)
		meson_i2c_set_clk_div_fast(i2c);
	else {
		if (!i2c->retain_fastmode)
			meson_i2c_set_clk_div_std(i2c);
		else
			meson_i2c_set_clk_div_fast(i2c);
	}
}

static void meson_i2c_get_data(struct meson_i2c *i2c, char *buf, int len)
{
	u32 rdata0, rdata1;
	int i;

	rdata0 = readl(i2c->regs + REG_TOK_RDATA0);
	rdata1 = readl(i2c->regs + REG_TOK_RDATA1);

	dev_dbg(i2c->dev, "%s: data %08x %08x len %d\n", __func__,
		rdata0, rdata1, len);

	for (i = 0; i < min_t(int, 4, len); i++)
		*buf++ = (rdata0 >> i * 8) & 0xff;

	for (i = 4; i < min_t(int, 8, len); i++)
		*buf++ = (rdata1 >> (i - 4) * 8) & 0xff;
}

static void meson_i2c_put_data(struct meson_i2c *i2c, char *buf, int len)
{
	u32 wdata0 = 0, wdata1 = 0;
	int i;

	for (i = 0; i < min_t(int, 4, len); i++)
		wdata0 |= *buf++ << (i * 8);

	for (i = 4; i < min_t(int, 8, len); i++)
		wdata1 |= *buf++ << ((i - 4) * 8);

	writel(wdata0, i2c->regs + REG_TOK_WDATA0);
	writel(wdata1, i2c->regs + REG_TOK_WDATA1);

	dev_dbg(i2c->dev, "%s: data %08x %08x len %d\n", __func__,
		wdata0, wdata1, len);
}

static void meson_i2c_prepare_xfer(struct meson_i2c *i2c)
{
	bool write = !(i2c->msg->flags & I2C_M_RD);
	int i;

	i2c->count = min_t(int, i2c->msg->len - i2c->pos, 8);

	for (i = 0; i < i2c->count - 1; i++)
		meson_i2c_add_token(i2c, TOKEN_DATA);

	if (i2c->count) {
		if (write || i2c->pos + i2c->count < i2c->msg->len)
			meson_i2c_add_token(i2c, TOKEN_DATA);
		else
			meson_i2c_add_token(i2c, TOKEN_DATA_LAST);
	}

	if (write)
		meson_i2c_put_data(i2c, i2c->msg->buf + i2c->pos, i2c->count);
}

static void meson_i2c_stop(struct meson_i2c *i2c)
{
	dev_dbg(i2c->dev, "%s: last %d\n", __func__, i2c->last);

	if (i2c->last) {
		i2c->state = STATE_STOP;
		meson_i2c_add_token(i2c, TOKEN_STOP);
	} else {
		i2c->state = STATE_IDLE;
		complete(&i2c->done);
	}
}

static irqreturn_t meson_i2c_irq(int irqno, void *dev_id)
{
	struct meson_i2c *i2c = dev_id;
	unsigned int ctrl;

	spin_lock(&i2c->lock);

	meson_i2c_reset_tokens(i2c);
	ctrl = readl(i2c->regs + REG_CTRL);

	dev_dbg(i2c->dev, "irq: state %d, pos %d, count %d, ctrl %08x\n",
		i2c->state, i2c->pos, i2c->count, ctrl);

	if (ctrl & REG_CTRL_ERROR && i2c->state != STATE_IDLE) {
		/*
		 * The bit is set when the IGNORE_NAK bit is cleared
		 * and the device didn't respond. In this case, the
		 * I2C controller automatically generates a STOP
		 * condition.
		 */
		dev_dbg(i2c->dev, "error bit set\n");
		i2c->error = -ENXIO;
		i2c->state = STATE_IDLE;
		complete(&i2c->done);
		goto out;
	}

	switch (i2c->state) {
	case STATE_READ:
		if (i2c->count > 0) {
			meson_i2c_get_data(i2c, i2c->msg->buf + i2c->pos,
					   i2c->count);
			i2c->pos += i2c->count;
		}

		if (i2c->pos >= i2c->msg->len) {
			meson_i2c_stop(i2c);
			break;
		}

		meson_i2c_prepare_xfer(i2c);
		break;
	case STATE_WRITE:
		i2c->pos += i2c->count;

		if (i2c->pos >= i2c->msg->len) {
			meson_i2c_stop(i2c);
			break;
		}

		meson_i2c_prepare_xfer(i2c);
		break;
	case STATE_STOP:
		i2c->state = STATE_IDLE;
		complete(&i2c->done);
		break;
	case STATE_IDLE:
		break;
	}

out:
	if (i2c->state != STATE_IDLE) {
		/* Restart the processing */
		meson_i2c_write_tokens(i2c);
		meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_START, 0);
		meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_START,
				   REG_CTRL_START);
	}

	spin_unlock(&i2c->lock);

	return IRQ_HANDLED;
}

static void meson_i2c_do_start(struct meson_i2c *i2c, struct i2c_msg *msg)
{
	int token;

	token = (msg->flags & I2C_M_RD) ? TOKEN_SLAVE_ADDR_READ :
		TOKEN_SLAVE_ADDR_WRITE;

	meson_i2c_set_mask(i2c, REG_SLAVE_ADDR, GENMASK(7, 0),
		(unsigned int)((msg->addr << 1) & GENMASK(7, 0)));

	meson_i2c_add_token(i2c, TOKEN_START);
	meson_i2c_add_token(i2c, token);
}

static int meson_i2c_xfer_msg(struct meson_i2c *i2c, struct i2c_msg *msg,
			      int last)
{
	unsigned long time_left, flags;
	int ret = 0;

	i2c->msg = msg;
	i2c->last = last;
	i2c->pos = 0;
	i2c->count = 0;
	i2c->error = 0;

	meson_i2c_reset_tokens(i2c);
	flags = (msg->flags & I2C_M_IGNORE_NAK) ? REG_CTRL_ACK_IGNORE : 0;
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_ACK_IGNORE, flags);

	if (!(msg->flags & I2C_M_NOSTART))
		meson_i2c_do_start(i2c, msg);

	i2c->state = (msg->flags & I2C_M_RD) ? STATE_READ : STATE_WRITE;
	meson_i2c_prepare_xfer(i2c);
	meson_i2c_write_tokens(i2c);
	reinit_completion(&i2c->done);

	/* Start the transfer */
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_START, REG_CTRL_START);

	time_left = msecs_to_jiffies(I2C_TIMEOUT_MS);
	time_left = wait_for_completion_timeout(&i2c->done, time_left);
	/*
	 * Protect access to i2c struct and registers from interrupt
	 * handlers triggered by a transfer terminated after the
	 * timeout period
	 */
	spin_lock_irqsave(&i2c->lock, flags);

	/* Abort any active operation */
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_START, 0);

	if (!time_left) {
		i2c->state = STATE_IDLE;
		ret = -ETIMEDOUT;
	}

	if (i2c->error)
		ret = i2c->error;

	spin_unlock_irqrestore(&i2c->lock, flags);

	return ret;
}

static int meson_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			  int num)
{
	struct meson_i2c *i2c = adap->algo_data;
	int i, ret = 0, count = 0;

	clk_enable(i2c->clk);
	meson_i2c_set_clk_div(i2c);

	for (i = 0; i < num; i++) {
		ret = meson_i2c_xfer_msg(i2c, msgs + i, i == num - 1);
		if (ret)
			break;
		count++;
	}

	clk_disable(i2c->clk);

	return ret ? ret : count;
}

static u32 meson_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm meson_i2c_algorithm = {
	.master_xfer	= meson_i2c_xfer,
	.functionality	= meson_i2c_func,
};

static ssize_t meson_i2c_speed_show(struct device *child,
			       struct device_attribute *attr, char *buf)
{
	struct meson_i2c *i2c =
		(struct meson_i2c *)dev_get_drvdata(child);
	return sprintf(buf, "%d\n", i2c->frequency);
}

static ssize_t meson_i2c_speed_store(struct device *child,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct meson_i2c *i2c =
		(struct meson_i2c *)dev_get_drvdata(child);
	int ret, val;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	i2c->frequency = val;

	return size;
}

static DEVICE_ATTR(speed, 0644, meson_i2c_speed_show,
						meson_i2c_speed_store);

int meson_i2c_speed_debug(struct device *dev)
{
	return sysfs_create_file(&dev->kobj, &dev_attr_speed.attr);
}

static int meson_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct meson_i2c *i2c;
	struct resource *mem;
	int ret = 0;
	int i;

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct meson_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				 &i2c->frequency))
		i2c->frequency = DEFAULT_FREQ;

	if (fwnode_property_present(&np->fwnode, "retain-fast-mode"))
		i2c->retain_fastmode = 1;

	i2c->dev = &pdev->dev;
	platform_set_drvdata(pdev, i2c);

	spin_lock_init(&i2c->lock);
	init_completion(&i2c->done);

	i2c->data = (struct meson_i2c_data *)
	of_device_get_match_data(&pdev->dev);

	i2c->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "can't get device clock\n");
		return PTR_ERR(i2c->clk);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(i2c->regs))
		return PTR_ERR(i2c->regs);

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0) {
		dev_err(&pdev->dev, "can't find IRQ\n");
		return i2c->irq;
	}

	ret = devm_request_irq(&pdev->dev, i2c->irq, meson_i2c_irq,
			       0, dev_name(&pdev->dev), i2c);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't request IRQ\n");
		return ret;
	}

	ret = clk_prepare(i2c->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't prepare clock\n");
		return ret;
	}

	strlcpy(i2c->adap.name, "Meson I2C adapter",
		sizeof(i2c->adap.name));
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &meson_i2c_algorithm;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = np;
	i2c->adap.algo_data = i2c;

	/*
	 * When the i2c controller has been used in U-boot, it may affect
	 * kernel i2c driver
	 */
	for (i = 0; i < 8; i++)
		meson_i2c_set_mask(i2c, REG_CTRL + 4*i, REG_FULL_MASK, 0);
	/*
	 * A transfer is triggered when START bit changes from 0 to 1.
	 * Ensure that the bit is set to 0 after probe
	 */
	meson_i2c_set_mask(i2c, REG_CTRL, REG_CTRL_START, 0);

	ret = meson_i2c_speed_debug(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "Creat speed debug sysfs failed\n");

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0) {
		clk_unprepare(i2c->clk);
		return ret;
	}

	return 0;
}

static int meson_i2c_remove(struct platform_device *pdev)
{
	struct meson_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
	clk_unprepare(i2c->clk);

	return 0;
}

static const struct meson_i2c_data i2c_meson6_data = {
	.div_factor = 4,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_meson8b_data = {
	.div_factor = 4,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_gx_data = {
	.div_factor = 4,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_txl_data = {
	.div_factor = 4,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_axg_data = {
	.div_factor = 3,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_txlx_data = {
	.div_factor = 3,
	.delay_ajust = 15,
};

static const struct meson_i2c_data i2c_g12a_data = {
	.div_factor = 3,
	.delay_ajust = 15,
};

/* for the stable i2c controller, div_factor=3*/
static const struct meson_i2c_data i2c_meson_data = {
	.div_factor = 3,
	.delay_ajust = 15,
};

static const struct of_device_id meson_i2c_match[] = {
	{ .compatible = "amlogic,meson6-i2c", .data = &i2c_meson6_data },
	{ .compatible = "amlogic,meson8b-i2c", .data = &i2c_meson8b_data },
	{ .compatible = "amlogic,meson-gx-i2c", .data = &i2c_gx_data },
	{ .compatible = "amlogic,meson-axg-i2c", .data = &i2c_axg_data },
	{ .compatible = "amlogic,meson-txl-i2c", .data = &i2c_txl_data },
	{ .compatible = "amlogic,meson-txlx-i2c", .data = &i2c_txlx_data },
	{ .compatible = "amlogic,meson-g12a-i2c", .data = &i2c_g12a_data },
	{ .compatible = "amlogic,meson-g12b-i2c", .data = &i2c_g12a_data },
	{ .compatible = "amlogic,meson-i2c", .data = &i2c_meson_data },
	{},
};
MODULE_DEVICE_TABLE(of, meson_i2c_match);

static struct platform_driver meson_i2c_driver = {
	.probe   = meson_i2c_probe,
	.remove  = meson_i2c_remove,
	.driver  = {
		.name  = "meson-i2c",
		.of_match_table = meson_i2c_match,
	},
};

//module_platform_driver(meson_i2c_driver);

static int __init meson_i2c_driver_init(void)
{
    return platform_driver_register(&meson_i2c_driver);
}

static void __exit meson_i2c_driver_exit(void)
{
    return platform_driver_unregister(&meson_i2c_driver);
}

subsys_initcall(meson_i2c_driver_init);
module_exit(meson_i2c_driver_exit);

MODULE_DESCRIPTION("Amlogic Meson I2C Bus driver");
MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_LICENSE("GPL v2");
