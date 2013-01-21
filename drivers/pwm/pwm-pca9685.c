/* drivers/pwm/pwm-pca9685.c
 * 
 * Copyright (C) 2013 Koen Kooi
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 * 
 * This program is distributed in the hope that is will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABLILITY of FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Genernal Public License for more details.
 *
 */


#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pwm.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>


#define PCA9685_REG_PSC			0xFE
#define PCA9685_REG_LED_ON(led)		0x6 + (led >> 2)
#define PCA9685_REG_LED_OFF(led)	0x7 + (led >> 2)

#define NUM_PWM_CHANNEL			16	/* PCA9685 channels */

struct pca9685_pwm_chip {
	struct pwm_chip chip;
	unsigned long period_cycles[NUM_PWM_CHANNEL];
	enum pwm_polarity polarity[NUM_PWM_CHANNEL];
};

struct pca9685_device {
	struct i2c_client *client;
	/* TODO */
};

static int pca9685_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	pr_info("PWM config");
	return 0;
}

static int pca9685_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
		enum pwm_polarity polarity)
{
	pr_info("PWM set polarity");
	return 0;
}

static int pca9685_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_info("PWM enable");
	return 0;
}

static void pca9685_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_info("PWM disable");
}

static void pca9685_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_info("PWM free");
	if (test_bit(PWMF_ENABLED, &pwm->flags)) {
		dev_warn(chip->dev, "Removing PWM device without disabling\n");
	}
}

static const struct pwm_ops pca9685_pwm_ops = {
	.free		= pca9685_pwm_free,
	.config		= pca9685_pwm_config,
	.set_polarity	= pca9685_pwm_set_polarity,
	.enable		= pca9685_pwm_enable,
	.disable	= pca9685_pwm_disable,
	.owner		= THIS_MODULE,
};

/* TODO */

static int pca9685_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct pca9685_device *dev;
	struct pca9685_pwm_chip *pc;
	int ret;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK)) {
		printk(KERN_ERR "%s: needed i2c functionality is not supported\n", __func__);
		return -ENODEV;
	}
	
	dev = kzalloc(sizeof(struct pca9685_device), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	dev->client = client;
	i2c_set_clientdata(client, dev);

	pc = kzalloc(sizeof(struct pca9685_pwm_chip), GFP_KERNEL);
	if (pc == NULL) {
		printk(KERN_ERR "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	pc->chip.dev = &client->dev;
	pc->chip.ops = &pca9685_pwm_ops;
	pc->chip.base = -1;
        pc->chip.npwm = 16;

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
	pr_err("pwm-pca9685: pwmchip_add() failed: %d\n", ret);
		return ret;
        }

	return 0;
}

static int pca9685_remove(struct i2c_client *client)
{
	struct pca9685_client *dev = i2c_get_clientdata(client);

	/* TODO: do something */

	kfree(dev);
	return 0;
}

static const struct i2c_device_id pca9685_id[] = {
	{ "pwm-pca9685", 0 },
	{ }
};

static struct i2c_driver pca9685_driver = {
	.probe    = pca9685_probe,
	.remove   = pca9685_remove,
	.id_table = pca9685_id,
	.driver   = {
		.name = "pca9685",
	},
};

static int __init pca9685_init_driver(void)
{
	return i2c_add_driver(&pca9685_driver);
}

static void __exit pca9685_exit_driver(void)
{
	i2c_del_driver(&pca9685_driver);
}

module_init(pca9685_init_driver);
module_exit(pca9685_exit_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("PCA9685 I2C 16 channel PWM driver");
MODULE_LICENSE("GPL");
