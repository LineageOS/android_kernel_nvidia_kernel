/*
 * Copyright (C) 2024 The LineageOS Project
 * SPDX-License-Identifier: GPL-2.0
 *
 * scaler_backlight - Virtual backlight driver for Jamboard MStar scaler
 *
 * Creates a /sys/class/backlight/scaler/ node that the Android Lights HAL
 * auto-discovers. The actual brightness is controlled by the scalerd userspace
 * daemon, which watches this sysfs node via inotify and sends UART commands
 * to the MStar MST9U23T1 scaler IC.
 *
 * This driver only stores the brightness value — it does not communicate
 * with hardware directly. The UART protocol lives entirely in userspace.
 */

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SCALER_MAX_BRIGHTNESS 255

struct scaler_bl_data {
	int brightness;
};

static int scaler_bl_update_status(struct backlight_device *bl)
{
	struct scaler_bl_data *data = bl_get_data(bl);

	data->brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		data->brightness = 0;

	return 0;
}

static int scaler_bl_get_brightness(struct backlight_device *bl)
{
	struct scaler_bl_data *data = bl_get_data(bl);

	return data->brightness;
}

static const struct backlight_ops scaler_bl_ops = {
	.update_status = scaler_bl_update_status,
	.get_brightness = scaler_bl_get_brightness,
};

static int scaler_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct scaler_bl_data *data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = SCALER_MAX_BRIGHTNESS;
	props.brightness = SCALER_MAX_BRIGHTNESS;

	bl = devm_backlight_device_register(&pdev->dev, "scaler",
					    &pdev->dev, data,
					    &scaler_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	data->brightness = props.brightness;
	platform_set_drvdata(pdev, bl);

	dev_info(&pdev->dev, "scaler backlight registered (max=%d)\n",
		 SCALER_MAX_BRIGHTNESS);
	return 0;
}

static const struct of_device_id scaler_bl_of_match[] = {
	{ .compatible = "jamboard,scaler-backlight" },
	{ },
};
MODULE_DEVICE_TABLE(of, scaler_bl_of_match);

static struct platform_driver scaler_bl_driver = {
	.driver = {
		.name = "scaler-backlight",
		.of_match_table = scaler_bl_of_match,
	},
	.probe = scaler_bl_probe,
};
module_platform_driver(scaler_bl_driver);

MODULE_DESCRIPTION("Jamboard MStar scaler virtual backlight driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("LineageOS");
