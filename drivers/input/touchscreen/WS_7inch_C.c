/******************************************************************************
 * WS_7inch_C.c
WaveShare 7inch 1024*600
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define DRIVER_VERSION		"v2.1"
#define DRIVER_AUTHOR		"<2355742822@qq.com>"
#define DRIVER_DESC		"WaveShare 7inch 1024*600"

#define TS_POLL_DELAY	1	/* ms delay before the first sample */
#define TS_POLL_PERIOD	5	/* ms delay between samples */

//Waveshare_Rotate = 0   : display_rotate=0  
//Waveshare_Rotate = 90  : display_rotate=1  
//Waveshare_Rotate = 180 : display_rotate=2 
//Waveshare_Rotate = 270 : display_rotate=3
#define Waveshare_Rotate	0

/* device specifc data/functions */
struct usbtouch_usb;
struct usbtouch_device_info {
	int min_xc, max_xc;
	int min_yc, max_yc;
	int min_press, max_press;
	int rept_size;

	/*
	 * Always service the USB devices irq not just when the input device is
	 * open. This is useful when devices have a watchdog which prevents us
	 * from periodically polling the device. Leave this unset unless your
	 * touchscreen device requires it, as it does consume more of the USB
	 * bandwidth.
	 */
	bool irq_always;

	void (*process_pkt) (struct usbtouch_usb *usbtouch, unsigned char *pkt, int len);

	/*
	 * used to get the packet len. possible return values:
	 * > 0: packet len
	 * = 0: skip one byte
	 * < 0: -return value more bytes needed
	 */
	int  (*get_pkt_len) (unsigned char *pkt, int len);

	int  (*read_data)   (struct usbtouch_usb *usbtouch, unsigned char *pkt);
	int  (*alloc)       (struct usbtouch_usb *usbtouch);
	int  (*init)        (struct usbtouch_usb *usbtouch);
	void (*exit)	    (struct usbtouch_usb *usbtouch);
};
static  unsigned char touch_value[10];
static  int ret_touch;
/* a usbtouch device */
struct usbtouch_usb {
	unsigned char *data;
	dma_addr_t data_dma;
	int data_size;
	unsigned char *buffer;
	int buf_len;
    wait_queue_head_t	wait;
    struct work_struct work;
	struct urb *irq;
	struct usb_interface *interface;
	struct input_dev *input;
	struct usbtouch_device_info *type;
	char name[128];
	char phys[64];
	void *priv;

	unsigned int x, y;
	int touch, press;
};

/* device types */
enum {
        DEVTYPE_IGNORE = -1,
	WaveShare,
	
};

#define USB_DEVICE_HID_CLASS(vend, prod) \
	.match_flags = USB_DEVICE_ID_MATCH_INT_CLASS \
		| USB_DEVICE_ID_MATCH_DEVICE, \
	.idVendor = (vend), \
	.idProduct = (prod), \
	.bInterfaceClass = USB_INTERFACE_CLASS_HID

static const struct usb_device_id usbtouch_devices[] = {

	
	{USB_DEVICE_HID_CLASS(0x0eef, 0x0005), .driver_info = WaveShare},
	{}
};

/*****************************************************************************
 * WaveShare part
 */

#define WaveShare_PKT_TYPE_MASK		0xFE
#define WaveShare_PKT_TYPE_REPT		0x80
#define WaveShare_PKT_TYPE_DIAG		0x0A

static int WaveShare_init(struct usbtouch_usb *usbtouch)
{
    return 0;
}

static int WaveShare_read_data(struct usbtouch_usb *dev, unsigned char *pkt)
{
	if ((pkt[0] != 0x01)||(pkt[2] != 0))
		return 0;

#if (Waveshare_Rotate == 0)
	dev->x = (((pkt[5]) << 8) | (pkt[4]));
	dev->y = (((pkt[7]) << 8) | (pkt[6]));
#elif (Waveshare_Rotate == 90)
	dev->x = (((pkt[7]) << 8) | (pkt[6]));
	dev->y = 1024 - (((pkt[5]) << 8) | (pkt[4]));
#elif (Waveshare_Rotate == 180)
	dev->x = 1024 - (((pkt[5]) << 8) | (pkt[4]));
	dev->y = 600  - (((pkt[7]) << 8) | (pkt[6]));
#elif (Waveshare_Rotate == 270)

	dev->x = 600 - (((pkt[7]) << 8) | (pkt[6])); 
	dev->y = (((pkt[5]) << 8) | (pkt[4]));
#endif	
	
	dev->touch = pkt[1]&(0x01) ;

	return 1;
}

static int WaveShare_get_pkt_len(unsigned char *buf, int len)
{
	switch (buf[0] & WaveShare_PKT_TYPE_MASK) {
	case WaveShare_PKT_TYPE_REPT:
		return 5;

	case WaveShare_PKT_TYPE_DIAG:
		if (len < 2)
			return -1;

		return buf[1] + 2;
	}

	return 0;
}

/*****************************************************************************
 * General Touch Part
 */

/*****************************************************************************
 * the different device descriptors
 */
static void usbtouch_process_pkt(struct usbtouch_usb *usbtouch,
                                 unsigned char *pkt, int len);

static struct usbtouch_device_info usbtouch_dev_info[] = {
#if ((Waveshare_Rotate == 0) || (Waveshare_Rotate == 180))
	[WaveShare] = {
		.min_xc		= 0,
		.max_xc		= 1024,
		.min_yc		= 0,
		.max_yc		= 600,
		.rept_size	= 11,
		.process_pkt	= usbtouch_process_pkt,
		.get_pkt_len	= WaveShare_get_pkt_len,
		.read_data	= WaveShare_read_data,
		.init		= WaveShare_init,
	},
#elif ((Waveshare_Rotate == 90) || (Waveshare_Rotate == 270))
	[WaveShare] = {
		.min_xc		= 0,
		.max_xc		= 600,
		.min_yc		= 0,
		.max_yc		= 1024,
		.rept_size	= 11,
		.process_pkt	= usbtouch_process_pkt,
		.get_pkt_len	= WaveShare_get_pkt_len,
		.read_data	= WaveShare_read_data,
		.init		= WaveShare_init,
	},
#endif
};


/*****************************************************************************
 * Generic Part
 */
static void usbtouch_process_pkt(struct usbtouch_usb *usbtouch,
                                 unsigned char *pkt, int len)
{
	struct usbtouch_device_info *type = usbtouch->type;

	if (!type->read_data(usbtouch, pkt))
		return;
            
	input_report_key(usbtouch->input, BTN_TOUCH, usbtouch->touch);
    if(usbtouch->touch!=0)
    {
		input_report_abs(usbtouch->input, ABS_X, usbtouch->x);
		input_report_abs(usbtouch->input, ABS_Y, usbtouch->y);
    }      
    input_sync(usbtouch->input);
}

static void usbtouch_irq(struct urb *urb)
{
	struct usbtouch_usb *usbtouch = urb->context;
	struct device *dev = &usbtouch->interface->dev;
         
	int retval;

	switch (urb->status) 
	{
		case 0:
			/* success */
			break;
		case -ETIME:
			/* this urb is timing out */
			dev_dbg(dev,
				"%s - urb timed out - was the device unplugged?\n",
				__func__);
			return;
		case -ECONNRESET:
		case -ENOENT:
		case -ESHUTDOWN:
		case -EPIPE:
			/* this urb is terminated, clean up */
			dev_dbg(dev, "%s - urb shutting down with status: %d\n",
				__func__, urb->status);
			return;
		default:
			dev_dbg(dev, "%s - nonzero urb status received: %d\n",
				__func__, urb->status);
			goto exit;
	}
      usbtouch->type->process_pkt(usbtouch, usbtouch->data, 7);
	//schedule_work(&usbtouch->work);

exit:
	usb_mark_last_busy(interface_to_usbdev(usbtouch->interface));
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - usb_submit_urb failed with result: %d\n",
			__func__, retval);

}

static int usbtouch_open(struct input_dev *input)
{
	struct usbtouch_usb *usbtouch = input_get_drvdata(input);
	int r;

	usbtouch->irq->dev = interface_to_usbdev(usbtouch->interface);

	r = usb_autopm_get_interface(usbtouch->interface) ? -EIO : 0;
	if (r < 0)
		goto out;

	if (!usbtouch->type->irq_always) 
	{
		if (usb_submit_urb(usbtouch->irq, GFP_KERNEL)) 
		{
			r = -EIO;
			goto out_put;
		}
	}

	usbtouch->interface->needs_remote_wakeup = 1;
out_put:
	usb_autopm_put_interface(usbtouch->interface);
out:
	return r;
}

static void usbtouch_close(struct input_dev *input)
{
	struct usbtouch_usb *usbtouch = input_get_drvdata(input);
	int r;

	if (!usbtouch->type->irq_always)
		usb_kill_urb(usbtouch->irq);
	r = usb_autopm_get_interface(usbtouch->interface);
	usbtouch->interface->needs_remote_wakeup = 0;
	if (!r)
		usb_autopm_put_interface(usbtouch->interface);
}

static int usbtouch_suspend
(struct usb_interface *intf, pm_message_t message)
{
	struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);

	usb_kill_urb(usbtouch->irq);

	return 0;
}

static int usbtouch_resume(struct usb_interface *intf)
{
	struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);
	struct input_dev *input = usbtouch->input;
	int result = 0;

	mutex_lock(&input->mutex);
	if (input->users || usbtouch->type->irq_always)
		result = usb_submit_urb(usbtouch->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return result;
}

static int usbtouch_reset_resume(struct usb_interface *intf)
{
	struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);
	struct input_dev *input = usbtouch->input;
	int err = 0;

	/* reinit the device */
	if (usbtouch->type->init) 
	{
		err = usbtouch->type->init(usbtouch);
		if (err) {
			dev_dbg(&intf->dev,
				"%s - type->init() failed, err: %d\n",
				__func__, err);
			return err;
		}
	}

	/* restart IO if needed */
	mutex_lock(&input->mutex);
	if (input->users)
		err = usb_submit_urb(usbtouch->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return err;
}

static void usbtouch_free_buffers(struct usb_device *udev,
				  struct usbtouch_usb *usbtouch)
{
	usb_free_coherent(udev, usbtouch->data_size,
			  usbtouch->data, usbtouch->data_dma);
	kfree(usbtouch->buffer);
}

static struct usb_endpoint_descriptor *
usbtouch_get_input_endpoint(struct usb_host_interface *interface)
{
	int i;

	for (i = 0; i < interface->desc.bNumEndpoints; i++)
		if (usb_endpoint_dir_in(&interface->endpoint[i].desc))
			return &interface->endpoint[i].desc;

	return NULL;
}
//////////////////////////////////////////////////////////////////
static void usbtouch_work_func(struct work_struct *work)
{
	struct usbtouch_usb *usbtouch =
		container_of(work, struct usbtouch_usb, work);

	struct usb_device *udev = interface_to_usbdev(usbtouch->interface);
	msleep(TS_POLL_DELAY);

	ret_touch = usb_control_msg(udev, usb_rcvctrlpipe (udev, 0),
	                       0x01,
	                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                      0, 0, touch_value, 7, USB_CTRL_SET_TIMEOUT);
	while ((ret_touch>=0)&&(touch_value[1]!=0)) 
	{
		ret_touch = usb_control_msg(udev, usb_rcvctrlpipe (udev, 0),
	                       0x01,
	                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                      0, 0, touch_value, 7, USB_CTRL_SET_TIMEOUT);
		usbtouch->type->process_pkt(usbtouch, touch_value, 7);


		wait_event_timeout(usbtouch->wait, (touch_value[1]!=0),
				   msecs_to_jiffies(TS_POLL_PERIOD));
         if(touch_value[1]==0)break;
	}
	
}
////////////////////////////////////////////////////////////////////////////
static int usbtouch_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usbtouch_usb *usbtouch;
	struct input_dev *input_dev;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usbtouch_device_info *type;
	int err = -ENOMEM;

	/* some devices are ignored */
	if (id->driver_info == DEVTYPE_IGNORE)
		return -ENODEV;

	endpoint = usbtouch_get_input_endpoint(intf->cur_altsetting);
	if (!endpoint)
		return -ENXIO;

	usbtouch = kzalloc(sizeof(struct usbtouch_usb), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!usbtouch || !input_dev)
		goto out_free;

	type = &usbtouch_dev_info[id->driver_info];
	usbtouch->type = type;
	if (!type->process_pkt)
		type->process_pkt = usbtouch_process_pkt;

	usbtouch->data_size = type->rept_size;
	if (type->get_pkt_len) 
	{
		/*
		 * When dealing with variable-length packets we should
		 * not request more than wMaxPacketSize bytes at once
		 * as we do not know if there is more data coming or
		 * we filled exactly wMaxPacketSize bytes and there is
		 * nothing else.
		 */
		usbtouch->data_size = min(usbtouch->data_size,
					  usb_endpoint_maxp(endpoint));
	}

	usbtouch->data = usb_alloc_coherent(udev, usbtouch->data_size,
					    GFP_KERNEL, &usbtouch->data_dma);
	if (!usbtouch->data)
		goto out_free;

	if (type->get_pkt_len) 
	{
		usbtouch->buffer = kmalloc(type->rept_size, GFP_KERNEL);
		if (!usbtouch->buffer)
			goto out_free_buffers;
	}

	usbtouch->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!usbtouch->irq) 
	{
		dev_dbg(&intf->dev,
			"%s - usb_alloc_urb failed: usbtouch->irq\n", __func__);
		goto out_free_buffers;
	}

	usbtouch->interface = intf;
	usbtouch->input = input_dev;

	if (udev->manufacturer)
		strlcpy(usbtouch->name, udev->manufacturer, sizeof(usbtouch->name));

	if (udev->product) 
	{
		if (udev->manufacturer)
			strlcat(usbtouch->name, " ", sizeof(usbtouch->name));
		strlcat(usbtouch->name, udev->product, sizeof(usbtouch->name));
	}

	if (!strlen(usbtouch->name))
		snprintf(usbtouch->name, sizeof(usbtouch->name),
			"USB Touchscreen %04x:%04x",
			 le16_to_cpu(udev->descriptor.idVendor),
			 le16_to_cpu(udev->descriptor.idProduct));

	usb_make_path(udev, usbtouch->phys, sizeof(usbtouch->phys));
	strlcat(usbtouch->phys, "/input0", sizeof(usbtouch->phys));

	input_dev->name = usbtouch->name;
	input_dev->phys = usbtouch->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->dev.parent = &intf->dev;

	input_set_drvdata(input_dev, usbtouch);

	input_dev->open = usbtouch_open;
	input_dev->close = usbtouch_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, type->min_xc, type->max_xc, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, type->min_yc, type->max_yc, 0, 0);
	if (type->max_press)
		input_set_abs_params(input_dev, ABS_PRESSURE, type->min_press,
		                     type->max_press, 0, 0);

	if (usb_endpoint_type(endpoint) == USB_ENDPOINT_XFER_INT)
		usb_fill_int_urb(usbtouch->irq, udev,
			 usb_rcvintpipe(udev, endpoint->bEndpointAddress),
			 usbtouch->data, usbtouch->data_size,
			 usbtouch_irq, usbtouch, endpoint->bInterval);
	else
		usb_fill_bulk_urb(usbtouch->irq, udev,
			 usb_rcvbulkpipe(udev, endpoint->bEndpointAddress),
			 usbtouch->data, usbtouch->data_size,
			 usbtouch_irq, usbtouch);

	usbtouch->irq->dev = udev;
	usbtouch->irq->transfer_dma = usbtouch->data_dma;
	usbtouch->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
       init_waitqueue_head(&usbtouch->wait);
       INIT_WORK(&usbtouch->work, usbtouch_work_func);
	/* device specific allocations */
	if (type->alloc) 
	{
		err = type->alloc(usbtouch);
		if (err) 
		{
			dev_dbg(&intf->dev,
				"%s - type->alloc() failed, err: %d\n",
				__func__, err);
			goto out_free_urb;
		}
	}

	/* device specific initialisation*/
	if (type->init) 
	{
		err = type->init(usbtouch);
		if (err) 
		{
			dev_dbg(&intf->dev,
				"%s - type->init() failed, err: %d\n",
				__func__, err);
			goto out_do_exit;
		}
	}

	err = input_register_device(usbtouch->input);
	if (err) 
	{
		dev_dbg(&intf->dev,
			"%s - input_register_device failed, err: %d\n",
			__func__, err);
		goto out_do_exit;
	}

	usb_set_intfdata(intf, usbtouch);

	if (usbtouch->type->irq_always) 
	{
		/* this can't fail */
		usb_autopm_get_interface(intf);
		err = usb_submit_urb(usbtouch->irq, GFP_KERNEL);
		if (err) 
		{
			usb_autopm_put_interface(intf);
			dev_err(&intf->dev,
				"%s - usb_submit_urb failed with result: %d\n",
				__func__, err);
			goto out_unregister_input;
		}
	}

	return 0;

out_unregister_input:
	input_unregister_device(input_dev);
	input_dev = NULL;
out_do_exit:
	if (type->exit)
		type->exit(usbtouch);
out_free_urb:
	usb_free_urb(usbtouch->irq);
out_free_buffers:
	usbtouch_free_buffers(udev, usbtouch);
out_free:
	input_free_device(input_dev);
	kfree(usbtouch);
	return err;
}

static void usbtouch_disconnect(struct usb_interface *intf)
{
	struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);

	if (!usbtouch)
		return;

	dev_dbg(&intf->dev,
		"%s - usbtouch is initialized, cleaning up\n", __func__);

	usb_set_intfdata(intf, NULL);
	/* this will stop IO via close */
	input_unregister_device(usbtouch->input);
	usb_free_urb(usbtouch->irq);
	if (usbtouch->type->exit)
		usbtouch->type->exit(usbtouch);
	usbtouch_free_buffers(interface_to_usbdev(intf), usbtouch);
	kfree(usbtouch);
}

MODULE_DEVICE_TABLE(usb, usbtouch_devices);

static struct usb_driver usbtouch_driver = 
{
	.name		= "WS_usbtouchscreen",
	.probe		= usbtouch_probe,
	.disconnect	= usbtouch_disconnect,
	.suspend	= usbtouch_suspend,
	.resume		= usbtouch_resume,
	.reset_resume	= usbtouch_reset_resume,
	.id_table	= usbtouch_devices,
	.supports_autosuspend = 1,
};

module_usb_driver(usbtouch_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

MODULE_ALIAS("WS_7inch_C");
MODULE_ALIAS("WS_usbtouchscreen");
MODULE_ALIAS("mtouchusb");
