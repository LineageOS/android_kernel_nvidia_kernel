/*
 * Copyright (C) 2026 Thomas Makin
 * Copyright (C) 2026 The LineageOS Project
 * SPDX-License-Identifier: GPL-2.0
 *
 * Driver for MStar scaler over UART, used in Google Jamboard
 *
 * Authors:
 * - Bruno Figueiredo <djbruno@gmail.com>
 * - Thomas Makin <halorocker89@gmail.com>
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

/* Backlight constants */
#define MSTAR_MAX_BRIGHTNESS	100

/* Serial constants */
#define MSTAR_BAUD		9600
#define FRAME_LEN		10
#define RESP_MAX_LEN		32
#define RESP_MIN_LEN		8
#define MSTAR_RESP_TIMEOUT_MS	500

static const u8 MAGIC[5] = { 0x6E, 0x51, 0x86, 0x00, 0xF5 };

/* Command type */
static const u8 REQ_GET = 0x01;
static const u8 REQ_SET = 0x03;

/* Packet type */
static const u8 TYPE_COMMAND		= 0xDD;
static const u8 TYPE_NOTIFICATION	= 0xDC;

/* Opcodes */
static const u8 OPCODE_NOTIF_INPUT	= 0x82;
static const u8 OPCODE_GET_BRIGHTNESS	= 0x83;
static const u8 OPCODE_SET_BRIGHTNESS	= 0x85;
static const u8 OPCODE_GET_FIRMWARE	= 0x86;
static const u8 OPCODE_INPUT		= 0xE2;
static const u8 OPCODE_SPEAKER		= 0xE4;

/* Unknown param bytes */
static const u8 PARAM_GET			= 0x01;
static const u8 PARAM_SET_BRIGHT_INPUT	= 0xFE;
static const u8 PARAM_SET_SPEAKER		= 0xFF;

static const u8 RESP_MAGIC[2]	= { 0xC2, 0x3D };
static const u8 RESP_STATUS_OK	= 0x01;

/* Input source IDs */
enum mstar_input_state {
	INPUT_ANDROID = 0,
	INPUT_DP      = 1,
	INPUT_HDMI1   = 2, /* side */
	INPUT_HDMI2   = 3, /* rear */
};

static const char * const mstar_input_names[] = {
	[INPUT_ANDROID] = "android",
	[INPUT_DP]      = "dp",
	[INPUT_HDMI1]   = "hdmi1",
	[INPUT_HDMI2]   = "hdmi2",
};

/* used for cmd and notif */
struct mstar_uart_cmd {
	u8 magic_and_type[5];
	u8 pkt_type;
	u8 opcode;
	u8 param;
	u8 data;
	u8 crc; /* xor */
} __packed;

struct mstar_uart_resp {
	u8 magic[2];
	u8 status;
	u8 cmd;
	u8 unk1;
	u8 unk2;
	u8 len;
	u8 data[]; /* arbitrary len */
	/* u8 crc; trailing byte after data[] */
} __packed;

struct mstar_device {
	struct serdev_device *sdev;
	struct backlight_device *bdev;

	/* state */
	enum mstar_input_state input_state;
	int brightness;

	/* response receive buffer */
	u8 resp_buf[RESP_MAX_LEN];
	size_t resp_len;
	u8 resp_expected_cmd;
	bool resp_pending;
	struct completion resp_comp;
	struct mutex resp_lock; /* serializes cmd/response pairs */

	/* notification reassembly */
	u8 notif_buf[FRAME_LEN];
	size_t notif_len;

	/* misc */
	spinlock_t lock;
};

static u8 xor_checksum(const u8 *buf, int len)
{
	u8 cs = 0;
	int i;

	for (i = 0; i < len; i++)
		cs ^= buf[i];
	return cs;
}

static int mstar_serial_send(struct mstar_device *mdev, const u8 *data,
			     size_t len)
{
	int ret;

	ret = serdev_device_write(mdev->sdev, data, len,
				  msecs_to_jiffies(MSTAR_RESP_TIMEOUT_MS));
	if (ret < 0) {
		dev_err(&mdev->sdev->dev,
			"Failed to send serial data; ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int mstar_serial_send_cmd(struct mstar_device *mdev,
				struct mstar_uart_resp *resp, size_t resp_sz,
				u8 opcode, u8 param, u8 data)
{
	struct mstar_uart_cmd pkt;
	struct device *dev = &mdev->sdev->dev;
	u8 magic_and_type[5];
	int ret;

	memcpy(magic_and_type, MAGIC, sizeof(magic_and_type));

	/* only expect response from getter */
	if (resp)
		magic_and_type[3] = REQ_GET;
	else
		magic_and_type[3] = REQ_SET;

	memset(&pkt, 0, sizeof(pkt));
	memcpy(&pkt.magic_and_type, magic_and_type, sizeof(magic_and_type));
	pkt.pkt_type = TYPE_COMMAND;
	pkt.opcode = opcode;
	pkt.param = param;
	pkt.data = data;
	pkt.crc = xor_checksum((u8 *)&pkt, sizeof(pkt) - 1);

	dev_dbg(dev, "send_cmd: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		pkt.magic_and_type[0], pkt.magic_and_type[1],
		pkt.magic_and_type[2], pkt.magic_and_type[3],
		pkt.magic_and_type[4], pkt.pkt_type, pkt.opcode,
		pkt.param, pkt.data, pkt.crc);

	mutex_lock(&mdev->resp_lock);

	if (resp) {
		reinit_completion(&mdev->resp_comp);
		mdev->resp_expected_cmd = opcode;
		mdev->resp_pending = true;
		mdev->resp_len = 0;
	}

	ret = mstar_serial_send(mdev, (u8 *)&pkt, sizeof(pkt));
	if (ret) {
		dev_err(dev, "Failed to send packet\n");
		goto out;
	}

	if (!resp)
		goto out;

	if (!wait_for_completion_timeout(&mdev->resp_comp,
			msecs_to_jiffies(MSTAR_RESP_TIMEOUT_MS))) {
		dev_err(dev, "Timeout waiting for response to opcode 0x%02x\n",
			opcode);
		ret = -ETIMEDOUT;
		goto out;
	}

	if (mdev->resp_len < RESP_MIN_LEN) {
		dev_err(dev, "Short response (%zu bytes)\n", mdev->resp_len);
		ret = -EIO;
		goto out;
	}

	if (resp_sz < mdev->resp_len)
		ret = -EOVERFLOW;
	else
		ret = 0;
	memcpy(resp, mdev->resp_buf, min(mdev->resp_len, resp_sz));

out:
	mdev->resp_pending = false;
	mutex_unlock(&mdev->resp_lock);
	return ret;
}

static int mstar_uart_get_brightness(struct mstar_device *mdev, u8 *val)
{
	struct device *dev = &mdev->sdev->dev;
	struct mstar_uart_resp resp;
	int ret;

	ret = mstar_serial_send_cmd(mdev, &resp, sizeof(resp),
				    OPCODE_GET_BRIGHTNESS, PARAM_GET, 0x00);
	if (ret)
		return ret;

	if (resp.len < 1)
		return -EIO;

	*val = resp.data[0];
	dev_info(dev, "get_brightness: %u%%\n", *val);
	return 0;
}

static int mstar_uart_set_brightness(struct mstar_device *mdev, u8 val)
{
	dev_info(&mdev->sdev->dev, "set_brightness: %u%%\n", val);
	return mstar_serial_send_cmd(mdev, NULL, 0,
				     OPCODE_SET_BRIGHTNESS,
				     PARAM_SET_BRIGHT_INPUT, val);
}

static int mstar_uart_get_input(struct mstar_device *mdev, u8 *val)
{
	struct device *dev = &mdev->sdev->dev;
	struct mstar_uart_resp resp;
	int ret;

	ret = mstar_serial_send_cmd(mdev, &resp, sizeof(resp),
				    OPCODE_INPUT, PARAM_GET, 0x00);
	if (ret)
		return ret;

	if (resp.len < 1)
		return -EIO;

	*val = resp.data[0];
	dev_info(dev, "get_input: %u\n", *val);
	return 0;
}

static int mstar_uart_set_input(struct mstar_device *mdev,
				enum mstar_input_state state)
{
	dev_info(&mdev->sdev->dev, "set_input: %s (%u)\n",
		 mstar_input_names[state], (u8)state);
	return mstar_serial_send_cmd(mdev, NULL, 0,
				     OPCODE_INPUT, PARAM_SET_BRIGHT_INPUT,
				     (u8)state);
}

static int mstar_uart_get_speaker(struct mstar_device *mdev, u8 *val)
{
	struct device *dev = &mdev->sdev->dev;
	struct mstar_uart_resp resp;
	int ret;

	ret = mstar_serial_send_cmd(mdev, &resp, sizeof(resp),
				    OPCODE_SPEAKER, PARAM_GET, 0x00);
	if (ret)
		return ret;

	if (resp.len < 1)
		return -EIO;

	*val = resp.data[0];
	dev_info(dev, "get_speaker: %s\n", *val ? "on" : "off");
	return 0;
}

static int mstar_uart_set_speaker(struct mstar_device *mdev, bool on)
{
	dev_info(&mdev->sdev->dev, "set_speaker: %s\n", on ? "on" : "off");
	return mstar_serial_send_cmd(mdev, NULL, 0,
				     OPCODE_SPEAKER, PARAM_SET_SPEAKER,
				     on ? 0x01 : 0x00);
}

static void mstar_handle_notification(struct mstar_device *mdev,
				      const u8 *buf, size_t len)
{
	struct device *dev = &mdev->sdev->dev;
	enum mstar_input_state src;

	if (len < FRAME_LEN)
		return;

	if (buf[5] != TYPE_NOTIFICATION || buf[6] != OPCODE_NOTIF_INPUT)
		return;

	if (xor_checksum(buf, FRAME_LEN - 1) != buf[FRAME_LEN - 1]) {
		dev_warn(dev, "Notification checksum mismatch\n");
		return;
	}

	src = (enum mstar_input_state)buf[8];
	if (src >= ARRAY_SIZE(mstar_input_names)) {
		dev_warn(dev, "Unknown input source %u\n", src);
		return;
	}

	mdev->input_state = src;
	dev_info(dev, "Input source changed: %s (%u)\n",
		 mstar_input_names[src], src);
	sysfs_notify(&dev->kobj, NULL, "input");
}

static int mstar_scaler_serial_receive_buf(struct serdev_device *serdev,
					   const unsigned char *buf,
					   size_t count)
{
	struct mstar_device *mdev = serdev_device_get_drvdata(serdev);
	size_t i;

	if (!mdev)
		return 0;

	for (i = 0; i < count; i++) {
		u8 b = buf[i];

		/* Response frames start with RESP_MAGIC[0] and are handled
		 * only when a GET command is outstanding. */
		if (mdev->resp_pending) {
			if (mdev->resp_len == 0 && b != RESP_MAGIC[0])
				continue;

			if (mdev->resp_len < RESP_MAX_LEN)
				mdev->resp_buf[mdev->resp_len++] = b;

			if (mdev->resp_len >= RESP_MIN_LEN) {
				struct mstar_uart_resp *r =
					(struct mstar_uart_resp *)mdev->resp_buf;
				size_t total = sizeof(*r) + r->len + 1;

				if (mdev->resp_len >= total) {
					complete(&mdev->resp_comp);
					mdev->resp_pending = false;
				}
			}
			continue;
		}

		/* Otherwise, treat as async notification reassembly */
		if (mdev->notif_len == 0) {
			if (b != MAGIC[0])
				continue;
			mdev->notif_buf[mdev->notif_len++] = b;
		} else {
			mdev->notif_buf[mdev->notif_len++] = b;
			if (mdev->notif_len >= FRAME_LEN) {
				mstar_handle_notification(mdev,
					mdev->notif_buf, mdev->notif_len);
				mdev->notif_len = 0;
			}
		}
	}

	return count;
}

static struct serdev_device_ops mstar_scaler_serdev_ops = {
	.receive_buf = mstar_scaler_serial_receive_buf,
};

static ssize_t mstar_input_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mstar_device *mdev = dev_get_drvdata(dev);

	if (mdev->input_state >= ARRAY_SIZE(mstar_input_names))
		return -EINVAL;

	return sprintf(buf, "%s\n", mstar_input_names[mdev->input_state]);
}

static ssize_t mstar_input_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mstar_device *mdev = dev_get_drvdata(dev);
	int sel;
	int ret;

	sel = sysfs_match_string(mstar_input_names, buf);
	if (sel < 0)
		return sel;

	ret = mstar_uart_set_input(mdev, (enum mstar_input_state)sel);
	if (ret)
		return ret;

	mdev->input_state = sel;
	return count;
}

static DEVICE_ATTR(input, S_IWUSR | S_IRUGO, mstar_input_show, mstar_input_store);

static int mstar_scaler_bl_update_status(struct backlight_device *bl)
{
	struct mstar_device *mdev = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (brightness == mdev->brightness)
		return 0;

	ret = mstar_uart_set_brightness(mdev, (u8)brightness);
	if (ret) {
		dev_err(&mdev->sdev->dev,
			"Failed to set brightness %d: %d\n",
			brightness, ret);
		return ret;
	}

	mdev->brightness = brightness;
	return 0;
}

static int mstar_scaler_bl_get_brightness(struct backlight_device *bl)
{
	struct mstar_device *mdev = bl_get_data(bl);

	return mdev->brightness;
}

static const struct backlight_ops mstar_scaler_bl_ops = {
	.update_status = mstar_scaler_bl_update_status,
	.get_brightness = mstar_scaler_bl_get_brightness,
};

static int mstar_scaler_probe(struct serdev_device *serdev)
{
	struct backlight_properties props;
	struct backlight_device *bdev;
	struct mstar_device *mdev;
	struct device *dev = &serdev->dev;
	u8 val;
	int ret;

	dev_info(dev, "mstar_scaler_probe\n");

	mdev = devm_kzalloc(dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->sdev = serdev;
	serdev_device_set_drvdata(serdev, mdev);

	spin_lock_init(&mdev->lock);
	mutex_init(&mdev->resp_lock);
	init_completion(&mdev->resp_comp);

	serdev_device_set_client_ops(serdev, &mstar_scaler_serdev_ops);
	serdev_device_set_baudrate(serdev, MSTAR_BAUD);
	serdev_device_set_flow_control(serdev, false);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(dev, "Unable to open serdev device\n");
		return ret;
	}

	/* Give the scaler a moment to settle after port open */
	msleep(500);

	ret = mstar_uart_get_input(mdev, &val);
	if (ret) {
		dev_warn(dev, "Could not query input source: %d\n", ret);
		mdev->input_state = INPUT_ANDROID;
	} else if (val < ARRAY_SIZE(mstar_input_names)) {
		mdev->input_state = (enum mstar_input_state)val;
		dev_info(dev, "Input source: %s (%u)\n",
			 mstar_input_names[mdev->input_state], val);
	}

	ret = mstar_uart_get_brightness(mdev, &val);
	if (ret) {
		dev_warn(dev, "Could not query brightness: %d\n", ret);
		mdev->brightness = MSTAR_MAX_BRIGHTNESS;
	} else {
		mdev->brightness = val;
		dev_info(dev, "Brightness: %u%%\n", val);
	}

	ret = mstar_uart_get_speaker(mdev, &val);
	if (ret) {
		dev_warn(dev, "Could not query speaker state: %d\n", ret);
	} else if (val == 0) {
		dev_info(dev, "Speaker was OFF, enabling\n");
		mstar_uart_set_speaker(mdev, true);
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MSTAR_MAX_BRIGHTNESS;
	props.brightness = mdev->brightness;

	bdev = devm_backlight_device_register(dev, "mstar_scaler", dev,
					mdev, &mstar_scaler_bl_ops, &props);
	if (IS_ERR(bdev)) {
		dev_err(dev, "failed to register backlight\n");
		ret = PTR_ERR(bdev);
		goto err_close;
	}

	mdev->bdev = bdev;

	ret = device_create_file(dev, &dev_attr_input);
	if (ret) {
		dev_err(dev, "failed to create input sysfs entry\n");
		goto err_close;
	}

	dev_info(dev, "scaler backlight registered (max=%d)\n",
		 MSTAR_MAX_BRIGHTNESS);
	return 0;

err_close:
	serdev_device_close(serdev);
	return ret;
}

static void mstar_scaler_remove(struct serdev_device *serdev)
{
	device_remove_file(&serdev->dev, &dev_attr_input);
	serdev_device_close(serdev);
}

static const struct of_device_id mstar_scaler_of_match[] = {
	{ .compatible = "mstar,scaler-serial" },
	{ },
};
MODULE_DEVICE_TABLE(of, mstar_scaler_of_match);

static struct serdev_device_driver mstar_scaler_driver = {
	.driver = {
		.name = "scaler-serial",
		.of_match_table = mstar_scaler_of_match,
	},
	.probe = mstar_scaler_probe,
	.remove = mstar_scaler_remove,
};
module_serdev_device_driver(mstar_scaler_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Makin <halorocker89@gmail.com>");
MODULE_DESCRIPTION("Driver for MStar display scaler over UART");
