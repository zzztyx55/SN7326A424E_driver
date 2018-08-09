/*
 * MStar MSG21XX touchscreen driver
 *
 * Copyright (c) 2006-2012 MStar Semiconductor, Inc.
 *
 * Copyright (C) 2012 Bruce Ding <bruce.ding@mstarsemi.com>
 *
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>
#include <linux/irqchip/mt-eic.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/rtpm_prio.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

/* Constant Value & Variable Definition*/
#define SN7326_REG_KEYFIFO	0x10
#define SN7326_REG_CONFIG	0x08
#define SN7326_MAX_KEY_ROWS	8
#define SN7326_MAX_KEY_COLS	8
#define SN7326_MAX_KEY_NUM	(SN7326_MAX_KEY_ROWS * SN7326_MAX_KEY_COLS)
#define SN7326_ROW_SHIFT	3

#define SN7326_I2C_VTG_MIN_UV	1800000
#define SN7326_I2C_VTG_MAX_UV	1800000

#define SN7326_KEYLED_VTG_MIN_UV	1800000
#define SN7326_KEYLED_VTG_MAX_UV	1800000

/*
 * Note.
 * Please do not change the below setting.
 */

#define PINCTRL_STATE_ACTIVE	"pmx_key_int_active"
#define PINCTRL_STATE_SUSPEND	"pmx_key_int_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_key_release"

static struct i2c_client *sn7326_client= NULL;;
static struct input_dev *sn7326_input_dev= NULL;

static struct work_struct sn7326_work;
static struct workqueue_struct *sn7326_workqueue;


struct sn7326_platform_data {
	//const char *name;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
};

struct sn7326_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct sn7326_platform_data *pdata;
	struct regulator *vcc_i2c;
	struct regulator *vcc_keyled;
	bool suspended;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};



#if defined(CONFIG_HAS_EARLYSUSPEND)
static int sn7326_early_suspend(struct early_suspend *h);
static int sn7326_late_resume(struct early_suspend *h);
#endif

/* Function Definition */

static int sn7326_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
			__func__, reg, val, ret);
	return ret;
}

static int sn7326_read_reg(struct i2c_client *client, int reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, err %d\n",
			__func__, reg, ret);
	return ret;
}

#if 1
static struct pinctrl_state *sn7326_eint_as_int, *sn7326_rst_output0,*sn7326_rst_output1;
static struct pinctrl *sn7326_pinctrl1 = NULL;

static void sn7326_gpio_as_int(void)
{
	pinctrl_select_state(sn7326_pinctrl1, sn7326_eint_as_int);
}

static void sn7326_gpio_rest_high(void)
{
	pinctrl_select_state(sn7326_pinctrl1, sn7326_rst_output1);
}

static void sn7326_gpio_rest_low(void)
{
	pinctrl_select_state(sn7326_pinctrl1, sn7326_rst_output0);
}

#endif 
/* read data through I2C then report data to input
sub-system when interrupt occurred  */

#define MATRIX_SCAN_CODE(row, col, row_shift)	(((row) << (row_shift)) + (col))

static const u16 key_define[SN7326_MAX_KEY_NUM] = {
/*   col0      col1      col2      col3      col4      col5      col6      col7 */
  	BTN_DPAD_UP,    KEY_MENU,    KEY_F6,    KEY_F5,    KEY_ENTER,    KEY_1,    KEY_2,    KEY_3,  // row0
  	KEY_2/***/,	  KEY_3,	KEY_BACK,	  KEY_1,	KEY_6,	  KEY_0,	KEY_BACKSPACE,	  KEY_DOWN,  // row1
  	KEY_0,	  KEY_NUMERIC_POUND,	KEY_F7,	  KEY_NUMERIC_STAR,/*#*/	KEY_SPACE,	  KEY_VOLUMEUP,	KEY_VOLUMEDOWN,	  KEY_F1/*side key*/,  // row2
	KEY_8,	  KEY_9,	KEY_BACKSPACE,	  KEY_7,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,  // row3
	KEY_5,	  KEY_6,	KEY_ENTER,	  KEY_4,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,  // row4
	KEY_F2,	  KEY_F3,	KEY_F4,	  KEY_F1,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,  // row5
	BTN_DPAD_LEFT,	  BTN_DPAD_DOWN,	BTN_DPAD_RIGHT,	  KEY_HOMEPAGE,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,  // row6
	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,	KEY_1,	  KEY_1,  // row7
};

static irqreturn_t sn7326_interrupt(int irq, void *dev_id)
{
    queue_work(sn7326_workqueue, &sn7326_work);
	return IRQ_HANDLED;
}

#if 0
#else
static int sn7326_resume(struct device *dev)
{
	//int retval;
	struct sn7326_data *key_data = dev_get_drvdata(dev);
	//printk(KERN_ERR"sn7326: exit suspend ! \n");


	if (!key_data->suspended) {
		dev_err(dev, "sn7326 already in resume\n");
		return 0;
	}
	#if 0
	retval = sn7326_power_on(key_data, true);
	if (retval) {
		dev_err(dev, "sn7326 power on failed");
		return retval;
	}
	#endif
	key_data->suspended = false;

	return 0;
}
static int sn7326_suspend(struct device *dev)
{
	//int retval;
	struct sn7326_data *key_data = dev_get_drvdata(dev);
	//printk(KERN_ERR"sn7326: enter suspend ! \n");
	
	if (key_data->suspended) {
		dev_err(dev, "sn7326 already in suspend\n");
		return 0;
	}
	#if 0
	retval = sn7326_power_on(key_data, false);
	if (retval) {
		dev_err(dev, "sn7326 power off failed");
		return retval;
	}
	#endif
	key_data->suspended = true;
	
	return 0;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static int sn7326_late_resume(struct early_suspend *h)
{
	struct sn7326_data *key_data;

	key_data = container_of(h, struct sn7326_data, early_suspend);
	sn7326_resume(&key_data->client->dev);
	return;
}

static int sn7326_early_suspend(struct early_suspend *h)
{
	struct sn7326_data *key_data;

	key_data = container_of(h, struct sn7326_data, early_suspend);
	sn7326_suspend(&key_data->client->dev);
	return;
}
#endif


#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops sn7326_dev_pm_ops = {
	.suspend = sn7326_suspend,
	.resume = sn7326_resume,
};
#else
static const struct dev_pm_ops sn7326_dev_pm_ops = {
};
#endif


static int sn7326_debug_suspend_set(void *_data, u64 val)
{
	struct sn7326_data *data = _data;


	if (val)
		sn7326_suspend(&data->client->dev);
	else
		sn7326_resume(&data->client->dev);


	return 0;
}

static int sn7326_debug_suspend_get(void *_data, u64 *val)
{
	struct sn7326_data *data = _data;

	*val = data->suspended;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, sn7326_debug_suspend_get,
			sn7326_debug_suspend_set, "%lld\n");



static struct of_device_id sn7326_dev_of_match[] = {
	{ .compatible = "mediatek,sn7326_key", },
	{ },
};

int sn7326_irq_step(struct i2c_client *cli)
{
	int ints[2] = {0, 0};
	int ret = 0;
	struct device_node *node = NULL;
	int sn7326_irq;
	
	//node = of_find_matching_node(node, touch_of_match);
	node = of_find_matching_node(node, sn7326_dev_of_match);
	if (!node)
	{
		printk("can't find compatible node\n");
		return -1;
	}
    of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
    gpio_set_debounce(ints[0], ints[1]);

    sn7326_irq = irq_of_parse_and_map(node, 0);
    //printk("sn7326_irq_step sn7326_irq=%d ,mt_gpio_to_irq(14)=%d\n",sn7326_irq,mt_gpio_to_irq(14));
    ret = request_irq(sn7326_irq, sn7326_interrupt,IRQF_TRIGGER_FALLING, "SN7326_KEY-eint", NULL);
    //ret = request_threaded_irq(sn7326_irq, NULL,sn7326_interrupt,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"sn7326_irq_gpio", NULL);
    if (ret)
    {
        printk("EINT IRQ LINE NOT AVAILABLE\n");
        return -1;
    }

	cli->irq = sn7326_irq;

	return 0;
}

static void sn7326_work_callback(struct work_struct *work)
{
	int val, row, col, press;

	val = sn7326_read_reg(sn7326_client, SN7326_REG_CONFIG);
	//printk("sn7326_event_handler: read SN7326_REG_CONFIG = %x\n", val);
	val = 0;

	do{
        val = sn7326_read_reg(sn7326_client, SN7326_REG_KEYFIFO);
        //printk("sn7326: read SN7326_REG_KEYFIFO = %x\n", val);
    } while(val & 0x80) ;

    row = val & 0x7;
    col = (val >> 3) & 0x7;
    press = (val & 0x40) ? 1 : 0;

    //printk(KERN_ERR"sn7326: key[%d:%d]-> [%d] %s\n", row, col, key_define[row*8+col], press ? "press" : "release");

    input_event(sn7326_input_dev, EV_KEY, key_define[row*8+col], press);
    input_sync(sn7326_input_dev);
}



/* probe function is used for matching and initializing input device */
static int sn7326_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {

	int ret = 0, i;
	struct input_dev *input_dev;
	struct sn7326_data *key_data;
	struct sn7326_platform_data *pdata;

	//printk("sn7326 enter probe !!\n");
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct sn7326_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	#if 0
		ret = sn7326_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			return ret;
		}
	#endif 
	} else
		pdata = client->dev.platform_data;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}
	//printk("sn7326 enter probe yy!!\n");
	key_data = devm_kzalloc(&client->dev,
			sizeof(struct sn7326_data), GFP_KERNEL);
	if (!key_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	sn7326_client = client;
	key_data->client = client;

	/* allocate an input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "input device allocation failed\n");
		goto err_input_allocate_dev;
	}
	//printk("sn7326 enter probe xx!!\n");
	input_dev->name = "sn7326"; //client->name;
	input_dev->phys = "I2C";
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;

	key_data->input_dev = input_dev;
	key_data->client = client;
	key_data->pdata = pdata;
	sn7326_input_dev = input_dev;

	input_set_drvdata(input_dev, key_data);
	i2c_set_clientdata(client, key_data);


	sn7326_gpio_as_int();
	//printk("sn7326 enter probe 11!!\n");
	msleep(5);
	sn7326_gpio_rest_low();
	msleep(20);
	sn7326_gpio_rest_high();
	msleep(10);

	sn7326_write_reg(sn7326_client, SN7326_REG_CONFIG,0x10);
	do{
		ret = sn7326_read_reg(key_data->client, SN7326_REG_CONFIG);
		if(ret < 0)
			break;
	}while (ret & 0x80);
	
	//printk("sn7326: read SN7326_REG_CONFIG = %x\n", ret);
	
	ret = sn7326_read_reg(key_data->client, SN7326_REG_KEYFIFO);
	//printk("sn7326: read SN7326_REG_KEYFIFO = %x\n", ret);
	

	/* set the supported event type for input device */
	//set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	for(i=0; i<SN7326_MAX_KEY_NUM; i++)
		input_set_capability(input_dev, EV_KEY, key_define[i]);

	/* register the input device to input sub-system */
	ret = input_register_device(input_dev);
	if (ret < 0) {
		dev_err(&client->dev,
			"Unable to register ms-touchscreen input device\n");
		goto err_input_reg_dev;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
		key_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		key_data->early_suspend.suspend = sn7326_early_suspend;
		key_data->early_suspend.resume = sn7326_late_resume;
		register_early_suspend(&key_data->early_suspend);
#endif
	
    sn7326_workqueue = create_singlethread_workqueue("sn7326");
    INIT_WORK(&sn7326_work, sn7326_work_callback);

	ret = sn7326_irq_step(client);
	//disable_irq(client->irq);
	enable_irq(client->irq);

	
	return 0;

	free_irq(client->irq, key_data);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&key_data->early_suspend);
#endif
err_input_reg_dev:
	input_free_device(input_dev);
	input_dev = NULL;
err_input_allocate_dev:
#if 0
exit_gpio_config:
	if (key_data->key_pinctrl) {
		if (IS_ERR_OR_NULL(key_data->pinctrl_state_release)) {
			devm_pinctrl_put(key_data->key_pinctrl);
			key_data->key_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(key_data->key_pinctrl,
					key_data->pinctrl_state_release);
			if (ret < 0)
				dev_err(&key_data->client->dev,
					"Cannot get release pinctrl state\n");
		}
	}
#endif 
	//sn7326_power_on(key_data, false);
//exit_deinit_power:
	//sn7326_power_init(key_data, false);
	input_free_device(input_dev);

	return ret;
}

/* remove function is triggered when the input device is removed
from input sub-system */
static int touch_driver_remove(struct i2c_client *client)
{
	int retval = 0;
	struct sn7326_data *key_data = i2c_get_clientdata(client);

	free_irq(key_data->client->irq, key_data);
	gpio_free(key_data->pdata->irq_gpio);
	gpio_free(key_data->pdata->reset_gpio);

	if (key_data->key_pinctrl) {
		if (IS_ERR_OR_NULL(key_data->pinctrl_state_release)) {
			devm_pinctrl_put(key_data->key_pinctrl);
			key_data->key_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(key_data->key_pinctrl,
					key_data->pinctrl_state_release);
			if (retval < 0)
				dev_err(&key_data->client->dev,
					"Cannot get release pinctrl state\n");
		}
	}

	input_unregister_device(key_data->input_dev);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&key_data->early_suspend);
#endif

	return retval;
}

static int mt_s7326_pinctrl_init(struct platform_device *pdev)
{
	int retval = 0;
	//printk("mt_s7326_pinctrl_init \n");

	sn7326_pinctrl1 = devm_pinctrl_get(&pdev->dev);
	
	if (IS_ERR(sn7326_pinctrl1)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		retval = PTR_ERR(sn7326_pinctrl1);
		return retval;
	}
	sn7326_eint_as_int = pinctrl_lookup_state(sn7326_pinctrl1, "sn7326_eint_int");
	if (IS_ERR(sn7326_eint_as_int)) 
	{
		retval = PTR_ERR(sn7326_eint_as_int);
		printk("sn7326 Cannot find pinctrl sn7326_eint_as_int!\n");
		return retval;
	}
	sn7326_rst_output1 = pinctrl_lookup_state(sn7326_pinctrl1, "sn7326_rst_high");
	if (IS_ERR(sn7326_rst_output1)) 
	{
		retval = PTR_ERR(sn7326_rst_output1);
		printk("sn7326 Cannot find pinctrl sn7326_rst_output1!\n");
		return retval;
	}

	sn7326_rst_output0 = pinctrl_lookup_state(sn7326_pinctrl1, "sn7326_rst_low");
	if (IS_ERR(sn7326_rst_output0)) 
	{
		retval = PTR_ERR(sn7326_rst_output0);
		printk("sn7326 Cannot find pinctrl sn7326_rst_output0!\n");
		return retval;
	}
	
	return 0;
}

static struct of_device_id sn7326_match_table[] = {
	{ .compatible = "mediatek,i2c_sn7326_key", },
	{ },
};
static const struct i2c_device_id keypad_device_id[] = {
	{"sn7326", 0},
	{}, /* should not omitted */
};
  
MODULE_DEVICE_TABLE(i2c, keypad_device_id);

static struct i2c_driver keypad_device_driver = {
	.driver = {
		.name = "SI-EN-sn7326",
		.owner = THIS_MODULE,
		.of_match_table = sn7326_match_table,
#if CONFIG_PM
		.pm = &sn7326_dev_pm_ops,
#endif
	},
	.probe = sn7326_probe,
	.remove = touch_driver_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = sn7326_early_suspend,
	.resume = sn7326_late_resume,
#endif
	.id_table = keypad_device_id,
};

static int mt_sn7326_probe(struct platform_device *pdev)
{
	int ret = 0;
	//printk("mt_sn7326_probe \n");
	
	ret = mt_s7326_pinctrl_init(pdev);
	
	i2c_add_driver(&keypad_device_driver);

	return ret;
}
static int mt_sn7326_remove(struct platform_device *pdev)
{
	//printk("mt_sn7326_remove \n");
	return 0 ;
}


static struct platform_driver s7326_platform_driver = {
	.probe = mt_sn7326_probe,
	.remove = mt_sn7326_remove,
	.driver = {
		   .name = "SI-EN-sn7326",
		   .owner = THIS_MODULE,
		   .of_match_table = sn7326_dev_of_match,
		   },
};


static int s7326_init(void)
{

	platform_driver_register(&s7326_platform_driver);
	
	return 0;
}
//module_i2c_driver(keypad_device_driver);
module_init( s7326_init );
//module_exit( ns7326_exit );
MODULE_AUTHOR("SI-EN sn7326 Inc.");
MODULE_LICENSE("GPL v2");
