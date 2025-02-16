/*
 * Khadas MCU control driver
 *
 * Written by: Nick <nick@khadas.com>
 *
 * Copyright (c) 2019 Shenzhen Wesion Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/amlogic/pm.h>


/* Device registers */
#define MCU_BOOT_EN_WOL_REG             0x21
#define MCU_CMD_FAN_STATUS_CTRL_REG     0x88
#define MCU_USB_PCIE_SWITCH_REG         0x33 /* VIM3/VIM3L only */
#define MCU_PWR_OFF_CMD_REG             0x80
#define MCU_SHUTDOWN_NORMAL_REG         0x2c

#define MCU_FAN_TRIG_TEMP_LEVEL0        50	// 50 degree if not set
#define MCU_FAN_TRIG_TEMP_LEVEL1        60	// 60 degree if not set
#define MCU_FAN_TRIG_TEMP_LEVEL2        70	// 70 degree if not set
#define MCU_FAN_TRIG_MAXTEMP            80
#define MCU_FAN_LOOP_SECS               (30 * HZ)	// 30 seconds
#define MCU_FAN_TEST_LOOP_SECS          (5 * HZ)  // 5 seconds
#define MCU_FAN_LOOP_NODELAY_SECS       0
#define MCU_FAN_SPEED_OFF               0
#define MCU_FAN_SPEED_LOW               1
#define MCU_FAN_SPEED_MID               2
#define MCU_FAN_SPEED_HIGH              3

enum mcu_fan_mode {
	MCU_FAN_MODE_MANUAL = 0,
	MCU_FAN_MODE_AUTO,
};

enum mcu_fan_level {
	MCU_FAN_LEVEL_0 = 0,
	MCU_FAN_LEVEL_1,
	MCU_FAN_LEVEL_2,
	MCU_FAN_LEVEL_3,
};

enum mcu_fan_status {
	MCU_FAN_STATUS_DISABLE = 0,
	MCU_FAN_STATUS_ENABLE,
};

enum mcu_usb_pcie_switch_mode {
	MCU_USB_PCIE_SWITCH_MODE_USB3 = 0,
	MCU_USB_PCIE_SWITCH_MODE_PCIE
};

enum khadas_board_hwver {
	KHADAS_BOARD_HWVER_NONE = 0,
	KHADAS_BOARD_HWVER_V11,
	KHADAS_BOARD_HWVER_V12,
	KHADAS_BOARD_HWVER_V13,
	KHADAS_BOARD_HWVER_V14
};

enum khadas_board {
	KHADAS_BOARD_NONE,
	KHADAS_BOARD_VIM1,
	KHADAS_BOARD_VIM2,
	KHADAS_BOARD_VIM3
};

struct mcu_fan_data {
	struct platform_device *pdev;
	struct class *fan_class;
	struct delayed_work work;
	struct delayed_work fan_test_work;
	enum    mcu_fan_status enable;
	enum 	mcu_fan_mode mode;
	enum 	mcu_fan_level level;
	int	trig_temp_level0;
	int	trig_temp_level1;
	int	trig_temp_level2;
};

struct mcu_data {
	struct i2c_client *client;
	struct class *wol_class;
	struct class *usb_pcie_switch_class;
	struct class *mcu_class;
	int wol_enable;
	u8 usb_pcie_switch_mode;
	enum khadas_board board;
	enum khadas_board_hwver hwver;
	struct mcu_fan_data fan_data;
};

struct mcu_data *g_mcu_data;

extern void send_power_key(int state);
extern void realtek_enable_wol(int enable, bool is_shutdown);
void mcu_enable_wol(int enable, bool is_shutdown)
{
	realtek_enable_wol(enable, is_shutdown);
}

static int i2c_master_reg8_send(const struct i2c_client *client,
		const char reg, const char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = kzalloc(count + 1, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	memcpy(tx_buf+1, buf, count);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;
}

static int i2c_master_reg8_recv(const struct i2c_client *client,
		const char reg, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *)buf;

	ret = i2c_transfer(adap, msgs, 2);

	return (ret == 2) ? count : ret;
}

static int mcu_i2c_read_regs(struct i2c_client *client,
		u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret = i2c_master_reg8_recv(client, reg, buf, len);
	return ret;
}

static int mcu_i2c_write_regs(struct i2c_client *client,
		u8 reg, u8 const buf[], __u16 len)
{
	int ret;
	ret = i2c_master_reg8_send(client, reg, buf, (int)len);
	return ret;
}

static int is_mcu_fan_control_supported(void)
{
	// MCU FAN control is supported for:
	// 1. Khadas VIM1 V13 and later
	// 2. Khadas VIM2 V13 and later
	// 3. Khadas VIM3 V11 and later
	if (KHADAS_BOARD_VIM1 == g_mcu_data->board) {
		if (g_mcu_data->hwver >= KHADAS_BOARD_HWVER_V13)
			return 1;
		else
			return 0;
	} else if (KHADAS_BOARD_VIM2 == g_mcu_data->board) {
		if (g_mcu_data->hwver > KHADAS_BOARD_HWVER_V12)
			return 1;
		else
			return 0;
	} else if (KHADAS_BOARD_VIM3 == g_mcu_data->board) {
		if (g_mcu_data->hwver >= KHADAS_BOARD_HWVER_V11)
			return 1;
		else
			return 0;
	} else
		return 0;
}

static bool is_mcu_usb_pcie_switch_supported(void)
{
	// MCU USB PCIe switch is supported for:
	// 1. Khadas VIM3
	if (KHADAS_BOARD_VIM3 == g_mcu_data->board)
		return 1;
	else
		return 0;
}

static bool is_mcu_wol_supported(void)
{
	// WOL is supported for:
	// 1. Khadas VIM2
	// 2. Khadas VIM3
	if ((KHADAS_BOARD_VIM2 == g_mcu_data->board) ||
		(KHADAS_BOARD_VIM3 == g_mcu_data->board))
		return 1;
	else
		return 0;
}

static void mcu_fan_level_set(struct mcu_fan_data *fan_data, int level)
{
	if (is_mcu_fan_control_supported()) {
		int ret;
		u8 data[2] = {0};

		if(0 == level) {
			data[0] = MCU_FAN_SPEED_OFF;
		}else if(1 == level){
			data[0] = MCU_FAN_SPEED_LOW;
		}else if(2 == level){
			data[0] = MCU_FAN_SPEED_MID;
		}else if(3 == level){
			data[0] = MCU_FAN_SPEED_HIGH;
		}

		g_mcu_data->fan_data.level = data[0];

		ret = mcu_i2c_write_regs(g_mcu_data->client, MCU_CMD_FAN_STATUS_CTRL_REG, data, 1);
		if (ret < 0) {
			printk("write fan control err\n");
			return;
		}
	}
}

extern int get_cpu_temp(void);
extern int meson_get_temperature(void);
static void fan_work_func(struct work_struct *_work)
{
	if (is_mcu_fan_control_supported()) {
		int temp = -EINVAL;
		struct mcu_fan_data *fan_data = &g_mcu_data->fan_data;

		if ((KHADAS_BOARD_VIM1 == g_mcu_data->board) ||
			(KHADAS_BOARD_VIM2 == g_mcu_data->board))
			temp = get_cpu_temp();
		else if (KHADAS_BOARD_VIM3 == g_mcu_data->board)
			temp = meson_get_temperature();

		if(temp != -EINVAL){
			if(temp < fan_data->trig_temp_level0 ) {
				mcu_fan_level_set(fan_data, 0);
			}else if(temp < fan_data->trig_temp_level1 ) {
				mcu_fan_level_set(fan_data, 1);
			}else if(temp < fan_data->trig_temp_level2 ) {
				mcu_fan_level_set(fan_data, 2);
			}else{
				mcu_fan_level_set(fan_data, 3);
			}
		}

		schedule_delayed_work(&fan_data->work, MCU_FAN_LOOP_SECS);
	}
}

static void khadas_fan_set(struct mcu_fan_data  *fan_data)
{
	if (is_mcu_fan_control_supported()) {

		cancel_delayed_work(&fan_data->work);

		if (fan_data->enable == MCU_FAN_STATUS_DISABLE) {
			mcu_fan_level_set(fan_data, 0);
			return;
		}
		switch (fan_data->mode) {
			case MCU_FAN_MODE_MANUAL:
				switch(fan_data->level) {
					case MCU_FAN_LEVEL_0:
						mcu_fan_level_set(fan_data, 0);
						break;
					case MCU_FAN_LEVEL_1:
						mcu_fan_level_set(fan_data, 1);
						break;
					case MCU_FAN_LEVEL_2:
						mcu_fan_level_set(fan_data, 2);
						break;
					case MCU_FAN_LEVEL_3:
						mcu_fan_level_set(fan_data, 3);
						break;
					default:
						break;
				}
				break;

			case MCU_FAN_MODE_AUTO:
				// FIXME: achieve with a better way
				schedule_delayed_work(&fan_data->work, MCU_FAN_LOOP_NODELAY_SECS);
				break;

			default:
				break;
		}
	}
}

static ssize_t show_fan_enable(struct class *cls,
			 struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "Fan enable: %d\n", g_mcu_data->fan_data.enable);
}

static ssize_t store_fan_enable(struct class *cls, struct class_attribute *attr,
		       const char *buf, size_t count)
{
	int enable;

	if (kstrtoint(buf, 0, &enable))
		return -EINVAL;

	// 0: manual, 1: auto
	if( enable >= 0 && enable < 2 ){
		g_mcu_data->fan_data.enable = enable;
		khadas_fan_set(&g_mcu_data->fan_data);
	}

	return count;
}

static ssize_t show_fan_mode(struct class *cls,
			 struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "Fan mode: %d\n", g_mcu_data->fan_data.mode);
}

static ssize_t store_fan_mode(struct class *cls, struct class_attribute *attr,
		       const char *buf, size_t count)
{
	int mode;

	if (kstrtoint(buf, 0, &mode))
		return -EINVAL;

	// 0: manual, 1: auto
	if( mode >= 0 && mode < 2 ){
		g_mcu_data->fan_data.mode = mode;
		khadas_fan_set(&g_mcu_data->fan_data);
	}

	return count;
}

static ssize_t show_fan_level(struct class *cls,
			 struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "Fan level: %d\n", g_mcu_data->fan_data.level);
}

static ssize_t store_fan_level(struct class *cls, struct class_attribute *attr,
		       const char *buf, size_t count)
{
	int level;

	if (kstrtoint(buf, 0, &level))
		return -EINVAL;

	if( level >= 0 && level < 4){
		g_mcu_data->fan_data.level = level;
		khadas_fan_set(&g_mcu_data->fan_data);
	}

	return count;
}

static ssize_t show_fan_temp(struct class *cls,
			 struct class_attribute *attr, char *buf)
{
	int temp = -EINVAL;

	if ((KHADAS_BOARD_VIM1 == g_mcu_data->board) ||
		(KHADAS_BOARD_VIM2 == g_mcu_data->board))
		temp = get_cpu_temp();
	else if (KHADAS_BOARD_VIM3 == g_mcu_data->board)
		temp = meson_get_temperature();

	return sprintf(buf, "cpu_temp:%d\nFan trigger temperature: level0:%d level1:%d level2:%d\n", temp, g_mcu_data->fan_data.trig_temp_level0, g_mcu_data->fan_data.trig_temp_level1, g_mcu_data->fan_data.trig_temp_level2);
}

static ssize_t store_wol_enable(struct class *cls, struct class_attribute *attr,
		        const char *buf, size_t count)
{
	u8 reg[2];
	int ret;
	int enable;
	int state;

	if (kstrtoint(buf, 0, &enable))
		return -EINVAL;

	ret = mcu_i2c_read_regs(g_mcu_data->client, MCU_BOOT_EN_WOL_REG, reg, 1);
	if (ret < 0) {
		printk("write wol state err\n");
		return ret;
	}
	state = (int)reg[0];
	reg[0] = enable | (state & 0x02);
	ret = mcu_i2c_write_regs(g_mcu_data->client, MCU_BOOT_EN_WOL_REG, reg, 1);
	if (ret < 0) {
		printk("write wol state err\n");
		return ret;
	}

	g_mcu_data->wol_enable = reg[0];
	mcu_enable_wol(g_mcu_data->wol_enable, false);

	printk("write wol state: %d\n", g_mcu_data->wol_enable);
	return count;
}

static ssize_t show_wol_enable(struct class *cls,
		        struct class_attribute *attr, char *buf)
{
	int enable;
	enable = g_mcu_data->wol_enable & 0x01;
	return sprintf(buf, "%d\n", enable);
}

static ssize_t store_usb_pcie_switch_mode(struct class *cls, struct class_attribute *attr,
                const char *buf, size_t count)
{
	int ret;
	int mode;

	if (kstrtoint(buf, 0, &mode))
		return -EINVAL;

	if (0 != mode && 1 != mode)
		return -EINVAL;

	if ((mode < MCU_USB_PCIE_SWITCH_MODE_USB3) || (mode > MCU_USB_PCIE_SWITCH_MODE_PCIE))
		return -EINVAL;

	g_mcu_data->usb_pcie_switch_mode = (u8)mode;
	ret = mcu_i2c_write_regs(g_mcu_data->client, MCU_USB_PCIE_SWITCH_REG, &g_mcu_data->usb_pcie_switch_mode, 1);
	if (ret < 0) {
		printk("write USB PCIe switch error\n");

		return ret;
	}

	printk("Set USB PCIe Switch Mode: %s\n", g_mcu_data->usb_pcie_switch_mode ? "PCIe" : "USB3.0");

	return count;
}

static ssize_t show_usb_pcie_switch_mode(struct class *cls,
                struct class_attribute *attr, char *buf)
{
	printk("USB PCIe Switch Mode: %s\n", g_mcu_data->usb_pcie_switch_mode ? "PCIe" : "USB3.0");
	return sprintf(buf, "%d\n", g_mcu_data->usb_pcie_switch_mode);
}

static ssize_t store_mcu_poweroff(struct class *cls, struct class_attribute *attr,
                const char *buf, size_t count)
{
	int ret;
	int val;
	u8 reg;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (1 != val)
		return -EINVAL;

	reg = (u8)val;
	ret = mcu_i2c_write_regs(g_mcu_data->client, MCU_PWR_OFF_CMD_REG, &reg, 1);
	if (ret < 0) {
		printk("write poweroff cmd error\n");

		return ret;
	}

	return count;
}

static ssize_t store_mcu_rst(struct class *cls, struct class_attribute *attr,
            const char *buf, size_t count)
{
	u8 reg[2];
	int ret;
	int rst;

	if (kstrtoint(buf, 0, &rst))
		return -EINVAL;

	reg[0] = rst;
	ret = mcu_i2c_write_regs(g_mcu_data->client, MCU_SHUTDOWN_NORMAL_REG, reg, 1);
	if (ret < 0)
		printk("rst mcu err\n");

	return count;
}


static struct class_attribute wol_class_attrs[] = {
	__ATTR(enable, 0644, show_wol_enable, store_wol_enable),
};

static struct class_attribute fan_class_attrs[] = {
	__ATTR(enable, 0644, show_fan_enable, store_fan_enable),
	__ATTR(mode, 0644, show_fan_mode, store_fan_mode),
	__ATTR(level, 0644, show_fan_level, store_fan_level),
	__ATTR(temp, 0644, show_fan_temp, NULL),
};

static struct class_attribute mcu_class_attrs[] = {
	__ATTR(poweroff, 0644, NULL, store_mcu_poweroff),
	__ATTR(usb_pcie_switch_mode, 0644, show_usb_pcie_switch_mode, store_usb_pcie_switch_mode),
	__ATTR(rst, 0644, NULL, store_mcu_rst),
};

static void create_mcu_attrs(void)
{
	int i;
	printk("%s\n",__func__);
	if (is_mcu_wol_supported()) {
		g_mcu_data->wol_class = class_create(THIS_MODULE, "wol");
		if (IS_ERR(g_mcu_data->wol_class)) {
			pr_err("create wol_class debug class fail\n");
			return;
		}

		for (i = 0; i < ARRAY_SIZE(wol_class_attrs); i++) {
			if (class_create_file(g_mcu_data->wol_class, &wol_class_attrs[i]))
				pr_err("create wol attribute %s fail\n", wol_class_attrs[i].attr.name);
		}
	}

	g_mcu_data->mcu_class = class_create(THIS_MODULE, "mcu");
	if (IS_ERR(g_mcu_data->mcu_class)) {
		pr_err("create mcu_class debug class fail\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(mcu_class_attrs); i++) {
		if (strstr(mcu_class_attrs[i].attr.name, "usb_pcie_switch_mode")) {
			if (!is_mcu_usb_pcie_switch_supported())
				continue;
		}
		if (class_create_file(g_mcu_data->mcu_class, &mcu_class_attrs[i]))
			pr_err("create mcu attribute %s fail\n", mcu_class_attrs[i].attr.name);
	}

	if (is_mcu_fan_control_supported()) {
		g_mcu_data->fan_data.fan_class = class_create(THIS_MODULE, "fan");
		if (IS_ERR(g_mcu_data->fan_data.fan_class)) {
			pr_err("create fan_class debug class fail\n");
			return;
		}

		for (i=0; i<ARRAY_SIZE(fan_class_attrs); i++) {
			if (class_create_file(g_mcu_data->fan_data.fan_class, &fan_class_attrs[i]))
				pr_err("create fan attribute %s fail\n", fan_class_attrs[i].attr.name);
		}
	}
}

static int mcu_parse_dt(struct device *dev)
{
	int ret;
	const char *hwver = NULL;

	if (NULL == dev) return -EINVAL;

	// Get hardwere version
	ret = of_property_read_string(dev->of_node, "hwver", &hwver);
	if (ret < 0) {
		g_mcu_data->hwver = KHADAS_BOARD_HWVER_V12;
		g_mcu_data->board = KHADAS_BOARD_VIM2;
	} else {
		if (strstr(hwver, "VIM1"))
			g_mcu_data->board = KHADAS_BOARD_VIM1;
		else if (strstr(hwver, "VIM2"))
			g_mcu_data->board = KHADAS_BOARD_VIM2;
		else if (strstr(hwver, "VIM3"))
			g_mcu_data->board = KHADAS_BOARD_VIM3;
		else
			g_mcu_data->board = KHADAS_BOARD_NONE;

		if (KHADAS_BOARD_VIM1 == g_mcu_data->board) {
			if (0 == strcmp(hwver, "VIM1.V13")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V13;
			} else if (0 == strcmp(hwver, "VIM1.V14")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V14;
			} else {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_NONE;
			}
		} else if (KHADAS_BOARD_VIM2 == g_mcu_data->board) {
			if (0 == strcmp(hwver, "VIM2.V12")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V12;
			} else if (0 == strcmp(hwver, "VIM2.V13")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V13;
			} else if (0 == strcmp(hwver, "VIM2.V14")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V14;
			} else {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_NONE;
			}
		} else if (KHADAS_BOARD_VIM3 == g_mcu_data->board) {
			if (0 == strcmp(hwver, "VIM3.V11")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V11;
			} else if (0 == strcmp(hwver, "VIM3.V12")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V12;
			} else if (0 == strcmp(hwver, "VIM3.V13")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V13;
			} else if (0 == strcmp(hwver, "VIM3.V14")) {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_V14;
			} else {
				g_mcu_data->hwver = KHADAS_BOARD_HWVER_NONE;
			}
		}
	}

	ret = of_property_read_u32(dev->of_node, "fan,trig_temp_level0", &g_mcu_data->fan_data.trig_temp_level0);
	if (ret < 0)
		g_mcu_data->fan_data.trig_temp_level0 = MCU_FAN_TRIG_TEMP_LEVEL0;
	ret = of_property_read_u32(dev->of_node, "fan,trig_temp_level1", &g_mcu_data->fan_data.trig_temp_level1);
	if (ret < 0)
		g_mcu_data->fan_data.trig_temp_level1 = MCU_FAN_TRIG_TEMP_LEVEL1;
	ret = of_property_read_u32(dev->of_node, "fan,trig_temp_level2", &g_mcu_data->fan_data.trig_temp_level2);
	if (ret < 0)
		g_mcu_data->fan_data.trig_temp_level2 = MCU_FAN_TRIG_TEMP_LEVEL2;

	return ret;
}

int mcu_get_wol_status(void)
{
	if (g_mcu_data)
		return g_mcu_data->wol_enable;

	return 0;
}

static int mcu_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 reg[2];
	int ret;

	printk("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	g_mcu_data = kzalloc(sizeof(struct mcu_data), GFP_KERNEL);

	if (g_mcu_data == NULL)
		return -ENOMEM;

	mcu_parse_dt(&client->dev);

	printk("%s: board: %d, hwver: %d\n", __func__, (int)g_mcu_data->board, (int)g_mcu_data->hwver);

	g_mcu_data->client = client;
	if (is_mcu_wol_supported()) {
		ret = mcu_i2c_read_regs(client, MCU_BOOT_EN_WOL_REG, reg, 1);
		if (ret < 0)
			goto exit;
		g_mcu_data->wol_enable = (int)reg[0];
	}

	if (is_mcu_usb_pcie_switch_supported()) {
		// Get USB PCIe Switch status
		ret = mcu_i2c_read_regs(client, MCU_USB_PCIE_SWITCH_REG, reg, 1);
		if (ret < 0)
			goto exit;
		g_mcu_data->usb_pcie_switch_mode = (u8)reg[0];
	}

	if (is_mcu_fan_control_supported()) {
		g_mcu_data->fan_data.mode = MCU_FAN_MODE_AUTO;
		g_mcu_data->fan_data.level = MCU_FAN_LEVEL_0;
		g_mcu_data->fan_data.enable = MCU_FAN_STATUS_DISABLE;

		INIT_DELAYED_WORK(&g_mcu_data->fan_data.work, fan_work_func);
		mcu_fan_level_set(&g_mcu_data->fan_data, 0);
	}
	create_mcu_attrs();
	printk("%s,wol enable=%d\n",__func__ ,g_mcu_data->wol_enable);

	if (is_mcu_wol_supported()) {
	//	if (g_mcu_data->wol_enable == 3)
	//		mcu_enable_wol(g_mcu_data->wol_enable, false);

		reg[0] = 0x01;
		ret = mcu_i2c_write_regs(client, 0x87, reg, 1);
		if (ret < 0) {
			printk("write mcu err\n");
			goto  exit;
		}
	}
	return 0;
exit:
	return ret;
}


static int mcu_remove(struct i2c_client *client)
{
	return 0;
}

static void khadas_fan_shutdown(struct i2c_client *client)
{
	g_mcu_data->fan_data.enable = MCU_FAN_STATUS_DISABLE;
	khadas_fan_set(&g_mcu_data->fan_data);
}

#ifdef CONFIG_PM_SLEEP
static int khadas_fan_suspend(struct device *dev)
{
	cancel_delayed_work(&g_mcu_data->fan_data.work);
	mcu_fan_level_set(&g_mcu_data->fan_data, 0);

	return 0;
}

static int khadas_fan_resume(struct device *dev)
{
	khadas_fan_set(&g_mcu_data->fan_data);

	if (get_resume_method() == WOL_WAKEUP) {
		send_power_key(1);
		send_power_key(0);
	}
	return 0;
}

static const struct dev_pm_ops fan_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(khadas_fan_suspend, khadas_fan_resume)
};

#define FAN_PM_OPS (&(fan_dev_pm_ops))

#endif

static const struct i2c_device_id mcu_id[] = {
	{ "khadas-mcu", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcu_id);


static struct of_device_id mcu_dt_ids[] = {
	{ .compatible = "khadas-mcu" },
	{},
};
MODULE_DEVICE_TABLE(i2c, mcu_dt_ids);

struct i2c_driver mcu_driver = {
	.driver  = {
		.name   = "khadas-mcu",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(mcu_dt_ids),
#ifdef CONFIG_PM_SLEEP
		.pm = FAN_PM_OPS,
#endif
	},
	.probe		= mcu_probe,
	.remove 	= mcu_remove,
	.shutdown = khadas_fan_shutdown,
	.id_table	= mcu_id,
};
module_i2c_driver(mcu_driver);

MODULE_AUTHOR("Nick <nick@khadas.com>");
MODULE_DESCRIPTION("Khadas MCU control driver");
MODULE_LICENSE("GPL");
