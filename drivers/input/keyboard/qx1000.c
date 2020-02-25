/*
 *  FxTec Pro1 QX1000 (aka. IDEA t5) keyboard driver.
 *
 *  Based on original code in aw9523b.[ch]:
 *    Copyright (c) 2019 FX Technology Limited
 *    Copyright (c) 2019 IDEA International
 *
 *  Original code modified by:
 *    Copyright (c) 2019 mat <netman69@github>
 *
 *  Entirely rewritten by:
 *    Copyright (c) 2020 Tom Marshall <tdm.code@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 *  The QX1000 keyboard is comprised of the following:
 *
 *  - An aw9523b controller that drives most of the kyes.
 *  - Six gpios that drive the remainder of the keys.
 *
 *  Note that there are two each of the shift, ctrl, and alt keys, but
 *  each pair are driven by a single gpio.  Thus, left and right cannot
 *  be distinguished for these keys.
 *
 *  The QX1000 has two variants, QWERTY and QWERTZ.  The hardware is
 *  identical -- the only difference is the keycap labels.  The driver
 *  exports a sysfs file so that userspace may determine the actual
 *  variant (by asking the user) and set the variant at runtime.
 *
 *  Due to the compact size, several of the keys on a normal full size
 *  keyboard are missing.  The keyboard has yellow labels on some keys
 *  and a pair of "function" keys with yellow arrows to access them.
 *  The driver does not report these keys directly.  It keeps the state
 *  internally and emits the proper keycodes (including modifiers) when
 *  other keys are pressed and released.  Thus, the keyboard layout has
 *  two tables: one is used when the function keys are not pressed, and
 *  the other when one or both of them are pressed.
 *
 *  The aw9523b datasheet may be found here:
 *  https://pdf1.alldatasheet.com/datasheet-pdf/view/1148542/AWINIC/AW9523B/+0148775XvUpOKG+GcGDCDL+/datasheet.pdf
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif
#include <linux/device.h>


#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/syscore_ops.h>

/*
 * These keys are reserved for stylus devices.  If they are included in
 * the input device capabilities, Android will assume that the device
 * is a stylus, not a keyboard.  So we must avoid them.
 */
#define KEY_STYLUS_MIN		BTN_DIGI
#define KEY_STYLUS_MAX		BTN_WHEEL

#define AWINIC_NAME		"aw9523b"

/* aw9523b register addresses */
#define REG_INPUT_PORT0		0x00
#define REG_INPUT_PORT1		0x01
#define REG_OUTPUT_PORT0	0x02
#define REG_OUTPUT_PORT1	0x03
#define REG_CONFIG_PORT0	0x04
#define REG_CONFIG_PORT1	0x05
#define REG_INT_PORT0		0x06
#define REG_INT_PORT1		0x07
#define REG_IC_ID		0x10
#define REG_CTL			0x11
#define REG_SOFT_RESET		0x7f

#define AW9523_NR_PORTS         8
#define AW9523_NR_KEYS          (AW9523_NR_PORTS * BITS_PER_BYTE)

#define AW9523_CHIP_ID		0x23

#define AW9523_MIN_POLL_MS	10
#define AW9523_MAX_POLL_MS	1000
#define AW9523_DEFAULT_POLL_MS	40

#define OUT_LOW_VALUE		0

#define KF_SHIFT		0x8000
#define KF_CTRL			0x4000
#define KF_ALT			0x2000
#define KF_FN			0x1000	/* Not used in key array */

#define KEY_FLAGS(key) ((key) & 0xf000)
#define KEY_VALUE(key) ((key) & 0x0fff)

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;

	struct timer_list release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct pinctrl *key_pinctrl;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_button_data data[0];
};

static struct device *global_dev;
static struct syscore_ops gpio_keys_syscore_pm_ops;

static void gpio_keys_syscore_resume(void);

struct aw9523b_data {
	struct i2c_client	*client;
	spinlock_t		irq_lock;
	bool			irq_is_disabled;
	int			gpio_rst;
	int			gpio_caps_led;
	int			gpio_irq;
	struct work_struct	irq_work;
	struct delayed_work	poll_work;

	struct notifier_block   fb_client;
	bool			fb_blanked;
};

static struct input_dev *aw9523b_input_dev;
static struct i2c_client *g_client = NULL;

static u16 g_physical_modifiers = 0;
static u16 g_logical_modifiers = 0;

static unsigned int g_poll_interval;

enum keylayout {
	LAYOUT_NONE,
	LAYOUT_QWERTY,
	LAYOUT_QWERTZ,
	LAYOUT_MAX
};
static const char *keylayout_name(enum keylayout layout)
{
	const char *name;

	switch (layout) {
	case LAYOUT_NONE:
		name = "none";
		break;
	case LAYOUT_QWERTY:
		name = "qwerty";
		break;
	case LAYOUT_QWERTZ:
		name = "qwertz";
		break;
	default:
		name = "error";
	}

	return name;
}
static enum keylayout keylayout_enum(const char *name)
{
	if (!strcmp(name, "qwerty"))
		return LAYOUT_QWERTY;
	if (!strcmp(name, "qwertz"))
		return LAYOUT_QWERTZ;
	return LAYOUT_NONE;
}

static enum keylayout g_layout;
static bool g_layout_modified = false;
static u16 key_array[AW9523_NR_KEYS];
static u16 key_fn_array[AW9523_NR_KEYS];

static const u16 qwerty_keys[AW9523_NR_KEYS] = {
	/* 0..7 */
	KEY_RESERVED,	KEY_H,		KEY_B,		KEY_7,
	KEY_UP,		KEY_ENTER,	KEY_Y,		KEY_COMMA,
	/* 8..15 */
	KEY_3,		KEY_S,		KEY_Z,		KEY_M,
	KEY_I,		KEY_9,		KEY_W,		KEY_J,
	/* 16..23 */
	KEY_LEFT,	KEY_G,		KEY_V,		KEY_6,
	KEY_RIGHT,	KEY_DELETE,	KEY_T,		KEY_DOT,
	/* 24..31 */
	KEY_RIGHTALT,	KEY_A,		KEY_RIGHTBRACE,	KEY_RESERVED,
	KEY_P,		KEY_MINUS,	KEY_Q,		KEY_L,
	/* 32..39 */
	KEY_BACKSPACE,	KEY_D,		KEY_X,		KEY_RESERVED,
	KEY_SEMICOLON,	KEY_EQUAL,	KEY_E,		KEY_APOSTROPHE,
	/* 40..47 */
	KEY_CAPSLOCK,	KEY_BACKSLASH,	KEY_LEFTBRACE,	KEY_DOWN,
	KEY_O,		KEY_0,		KEY_GRAVE,	KEY_K,
	/* 48..55 */
	KEY_SPACE,	KEY_F,		KEY_C,		KEY_N,
	KEY_U,		KEY_8,		KEY_R,		KEY_5,
	/* 56..63 */
	KEY_ESC,	KEY_1,		KEY_RESERVED,	KEY_RESERVED,
	KEY_2,		KEY_4,		KEY_TAB,	KEY_RESERVED,
};
static const u16 qwerty_fn_keys[AW9523_NR_KEYS] = {
	/* 0..7 */
	KEY_RESERVED,			KEY_H,				KEY_B,				KEY_7 | KF_SHIFT,
	KEY_PAGEUP,			KEY_ENTER,			KEY_Y,				KEY_COMMA | KF_SHIFT,
	/* 8..15 */
	KEY_3 | KF_SHIFT,		KEY_S,				KEY_Z,				KEY_M,
	KEY_I,				KEY_9 | KF_SHIFT,		KEY_W,				KEY_J,
	/* 16..23 */
	KEY_HOME,			KEY_G,				KEY_V,				KEY_6 | KF_SHIFT,
	KEY_END,			KEY_INSERT,			KEY_T,				KEY_DOT | KF_SHIFT,
	/* 24..31 */
	KEY_RIGHTALT,			KEY_A,				KEY_RIGHTBRACE | KF_SHIFT,	KEY_RESERVED,
	KEY_SLASH,			KEY_MINUS | KF_SHIFT,		KEY_Q,				KEY_SLASH | KF_SHIFT,
	/* 32..39 */
	KEY_BACKSPACE,			KEY_D,				KEY_X,				KEY_RESERVED,
	KEY_SEMICOLON | KF_SHIFT,	KEY_EQUAL | KF_SHIFT,		KEY_E,				KEY_APOSTROPHE | KF_SHIFT,
	/* 40..47 */
	KEY_CAPSLOCK,			KEY_BACKSLASH | KF_SHIFT,	KEY_LEFTBRACE | KF_SHIFT,	KEY_PAGEDOWN,
	KEY_O,				KEY_0 | KF_SHIFT,		KEY_GRAVE | KF_SHIFT,		KEY_K,
	/* 48..55 */
	KEY_WWW,			KEY_F,				KEY_C,				KEY_N,
	KEY_U,				KEY_8 | KF_SHIFT,		KEY_R,				KEY_5 | KF_SHIFT,
	/* 56..63 */
	KEY_BACK,			KEY_1 | KF_SHIFT,		KEY_RESERVED,			KEY_RESERVED,
	KEY_2 | KF_SHIFT,		KEY_4 | KF_SHIFT,		KEY_TAB,			KEY_RESERVED,
};

static const u16 qwertz_keys[AW9523_NR_KEYS] = {
	/* 0..7 */
	KEY_RESERVED,	KEY_J,		KEY_N,		KEY_7,
	KEY_UP,		KEY_ENTER,	KEY_U,		KEY_DOT,
	/* 8..15 */
	KEY_3,		KEY_D,		KEY_X,		KEY_COMMA,
	KEY_O,		KEY_9,		KEY_E,		KEY_K,
	/* 16..23 */
	KEY_LEFT,	KEY_H,		KEY_B,		KEY_6,
	KEY_RIGHT,	KEY_DELETE,	KEY_Y,		KEY_SLASH,
	/* 24..31 */
	KEY_RIGHTALT,	KEY_S,		KEY_Z,		KEY_RESERVED,
	KEY_LEFTBRACE,	KEY_MINUS,	KEY_W,		KEY_SEMICOLON,
	/* 32..39 */
	KEY_BACKSPACE,	KEY_F,		KEY_C,		KEY_RESERVED,
	KEY_RIGHTBRACE,	KEY_EQUAL,	KEY_R,		KEY_APOSTROPHE,
	/* 40..47 */
	KEY_CAPSLOCK,	KEY_A,		KEY_GRAVE,	KEY_DOWN,
	KEY_P,		KEY_0,		KEY_Q,		KEY_L,
	/* 48..55 */
	KEY_SPACE,	KEY_G,		KEY_V,		KEY_M,
	KEY_I,		KEY_8,		KEY_T,		KEY_5,
	/* 56..63 */
	KEY_ESC,	KEY_1,		KEY_RESERVED,	KEY_RESERVED,
	KEY_2,		KEY_4,		KEY_TAB,	KEY_RESERVED,
};
static const u16 qwertz_fn_keys[AW9523_NR_KEYS] = {
	/* 0..7 */
	KEY_RESERVED,			KEY_J,				KEY_N,				KEY_7 | KF_SHIFT,
	KEY_PAGEUP,			KEY_ENTER,			KEY_U,				KEY_DOT | KF_SHIFT,
	/* 8..15 */
	KEY_3 | KF_SHIFT,		KEY_D,				KEY_X,				KEY_COMMA,
	KEY_O,				KEY_9 | KF_SHIFT,		KEY_E,				KEY_K,
	/* 16..23 */
	KEY_HOME,			KEY_H,				KEY_B,				KEY_6 | KF_SHIFT,
	KEY_END,			KEY_INSERT,			KEY_Y,				KEY_SLASH | KF_SHIFT,
	/* 24..31 */
	KEY_RIGHTALT,			KEY_S,				KEY_Z,				KEY_RESERVED,
	KEY_LEFTBRACE | KF_SHIFT,	KEY_MINUS | KF_SHIFT,		KEY_W,				KEY_SEMICOLON | KF_SHIFT,
	/* 32..39 */
	KEY_BACKSPACE,			KEY_F,				KEY_C,				KEY_RESERVED,
	KEY_RIGHTBRACE | KF_SHIFT,	KEY_EQUAL | KF_SHIFT,		KEY_R,				KEY_APOSTROPHE | KF_SHIFT,
	/* 40..47 */
	KEY_CAPSLOCK,			KEY_A,				KEY_GRAVE | KF_SHIFT,		KEY_PAGEDOWN,
	KEY_P,				KEY_0 | KF_SHIFT,		KEY_Q,				KEY_L,
	/* 48..55 */
	KEY_WWW,			KEY_G,				KEY_V,				KEY_M,
	KEY_I,				KEY_8 | KF_SHIFT,		KEY_T,				KEY_5 | KF_SHIFT,
	/* 56..63 */
	KEY_BACK,			KEY_1 | KF_SHIFT,		KEY_RESERVED,			KEY_RESERVED,
	KEY_2 | KF_SHIFT,		KEY_4 | KF_SHIFT,		KEY_TAB,			KEY_RESERVED,
};

static int aw9523b_i2c_read(struct i2c_client *client, char *writebuf,
               int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	}
	else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
    }
    return ret;
}

static int aw9523b_i2c_write(struct i2c_client *client, char *writebuf,
                int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int aw9523b_write_reg(u8 addr, const u8 val)
{
	u8 buf[2] = { 0 };

	buf[0] = addr;
	buf[1] = val;

	return aw9523b_i2c_write(g_client, buf, sizeof(buf));
}



static int aw9523b_read_reg(u8 addr, u8 *val)
{
	int ret;
	ret = aw9523b_i2c_read(g_client, &addr, 1, val, 1);
	if (ret < 0)
		dev_err(&g_client->dev, "%s:i2c read error.\n", __func__);
	return ret;
}

static int aw9523b_hw_reset(struct aw9523b_data *data)
{
	int ret = 0;

	ret = gpio_direction_output(data->gpio_rst, 1);
	if (ret) {
		dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
	}
	udelay(50);

	ret = aw9523b_write_reg(REG_SOFT_RESET, 0x00);
	if (ret < 0) {
		dev_err(&data->client->dev,"*****can not communicate with aw9523b\n");
		return -EINVAL;
	}

	ret = gpio_direction_output(data->gpio_rst, 0);
	if (ret) {
		dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
	}
	udelay(250);

	ret = gpio_direction_output(data->gpio_rst, 1);
	if (ret) {
		dev_err(&data->client->dev,"set_direction for pdata->gpio_rst failed\n");
		return -EINVAL;
	}
	udelay(50); 

	return 0;
}


static u8 aw9523b_chip_id(void)
{ 
	u8 chip_id;

	aw9523b_read_reg(REG_IC_ID, &chip_id);

	return chip_id;
}

static void aw9523b_config_P1_output(void)
{    
	aw9523b_write_reg(REG_CONFIG_PORT1, 0x00);
}

static void aw9523b_config_P0_input(void)
{
	aw9523b_write_reg(REG_CONFIG_PORT0, 0xff);
}

static void aw9523b_enable_P0_interrupt(void)
{
	aw9523b_write_reg(REG_INT_PORT0, 0x00);
}

static void aw9523b_enable_P0_interrupt_mask(u8 mask)
{
	aw9523b_write_reg(REG_INT_PORT0, mask);
}

static void aw9523b_disable_P0_interrupt(void)
{
	aw9523b_write_reg(REG_INT_PORT0, 0xff);
}

static void aw9523b_disable_P1_interrupt(void)
{
	aw9523b_write_reg(REG_INT_PORT1, 0xff);
}

static u8 aw9523b_get_P0_value(void)
{
	u8 value = 0;
	aw9523b_read_reg(REG_INPUT_PORT0, &value);
	return value;
}


static u8 aw9523b_get_P1_value(void) 
{    
	u8 value = 0;
	aw9523b_read_reg(REG_INPUT_PORT1, &value);
	return value;
}

static void aw9523b_set_P1_value(u8 data)
{
	aw9523b_write_reg(REG_OUTPUT_PORT1, data);
}

void aw9523b_irq_disable(struct aw9523b_data *data)
{
	unsigned long irqflags;

	spin_lock_irqsave(&data->irq_lock, irqflags);
	if (!data->irq_is_disabled) {
		data->irq_is_disabled = true;
		disable_irq_nosync(data->client->irq);
	}
	spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

void aw9523b_irq_enable(struct aw9523b_data *data)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&data->irq_lock, irqflags);
	if (data->irq_is_disabled) {
		enable_irq(data->client->irq);
		data->irq_is_disabled = false;
	}
	spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

  
static void aw9523b_read_keyboard_state(u8* keyboard_state)
{
	int port;

	for (port = 0; port < AW9523_NR_PORTS; ++port) {
		aw9523b_write_reg(REG_CONFIG_PORT1, ~(1 << port));
		keyboard_state[port] = ~aw9523b_get_P0_value();
	}
	aw9523b_write_reg(REG_CONFIG_PORT1, 0);
}

static bool aw9523b_key_state(const u8* keyboard_state, int key_nr)
{
	int idx = key_nr / BITS_PER_BYTE;
	int bit = key_nr % BITS_PER_BYTE;

	return (keyboard_state[idx] >> bit) & 0x01;
}

static void aw9523b_deghost(const u8* old_state, u8* new_state)
{
	unsigned int row1, row2;

	for (row1 = 0; row1 < AW9523_NR_PORTS; ++row1) {
		if (hweight8(new_state[row1]) < 2)
			continue;
		for (row2 = 0; row2 < AW9523_NR_PORTS; ++row2) {
			if (row2 == row1)
				continue;
			if (!(new_state[row2] & new_state[row1]))
				continue;
			new_state[row1] &= old_state[row1];
			new_state[row2] &= old_state[row2];
		}
	}
}

static void aw9523b_check_keys(struct aw9523b_data *pdata, u8* keyboard_state)
{
	static u8 capslock_led_enable = 0;
	static u8 prev_keyboard_state[AW9523_NR_PORTS] = { 0 };
	static u16 pressed[AW9523_NR_KEYS] = { 0 }; /* Elements are keycodes */

	int key_nr;
	u16 keycode;

	if (!memcmp(prev_keyboard_state, keyboard_state, sizeof(prev_keyboard_state)))
	    return;

	aw9523b_deghost(prev_keyboard_state, keyboard_state);
	memcpy(prev_keyboard_state, keyboard_state, sizeof(prev_keyboard_state));

	for (key_nr = 0; key_nr < AW9523_NR_KEYS; ++key_nr) {
		bool key_state = aw9523b_key_state(keyboard_state, key_nr);
		if (key_state && !pressed[key_nr]) {
			u16 force_flags;
			if (pdata->fb_blanked) {
				printk(KERN_INFO "aw9523b: wakeup\n");
				keycode = KEY_WAKEUP;
				force_flags = 0;
			}
			else if (g_physical_modifiers & KF_FN) {
				keycode = KEY_VALUE(key_fn_array[key_nr]);
				force_flags = KEY_FLAGS(key_fn_array[key_nr]);
			}
			else {
				keycode = key_array[key_nr];
				force_flags = 0;
			}
			printk(KERN_INFO "aw9523b: key press: key_nr=%d keycode=%04hx ff=%04hx gpm=%04hx glm=%04hx\n",
			    key_nr, keycode, force_flags, g_physical_modifiers, g_logical_modifiers);
			if (keycode == KEY_RESERVED) {
				printk(KERN_ERR "aw9523b: pressed dead key\n");
				continue;
			}
			pressed[key_nr] = keycode;
			if ((force_flags & KF_SHIFT) && !(g_logical_modifiers & KF_SHIFT)) {
				printk(KERN_INFO "aw9523b: press logical shift\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTSHIFT, 1);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers |= KF_SHIFT;
			}
			if ((force_flags & KF_CTRL) && !(g_logical_modifiers & KF_CTRL)) {
				printk(KERN_INFO "aw9523b: press logical ctrl\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTCTRL, 1);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers |= KF_CTRL;
			}
			if ((force_flags & KF_ALT) && !(g_logical_modifiers & KF_ALT)) {
				printk(KERN_INFO "aw9523b: press logical alt\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTALT, 1);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers |= KF_ALT;
			}
			input_report_key(aw9523b_input_dev, keycode, 1);
			input_sync(aw9523b_input_dev);
			if (keycode == KEY_CAPSLOCK) {
				if (capslock_led_enable == 0) {
					gpio_direction_output(pdata->gpio_caps_led, 1);
				}
				++capslock_led_enable;
			}
		}
		else if (!key_state && pressed[key_nr]) {
			keycode = pressed[key_nr];
			printk(KERN_INFO "aw9523b: key release: key_nr=%d keycode=%04hx gpm=%04hx glm=%04hx\n",
			    key_nr, keycode, g_physical_modifiers, g_logical_modifiers);
			if (keycode == KEY_RESERVED) {
				printk(KERN_ERR "aw9523b: released dead key\n");
				continue;
			}
			pressed[key_nr] = 0;
			input_report_key(aw9523b_input_dev, keycode, 0);
			input_sync(aw9523b_input_dev);
			if ((g_logical_modifiers & KF_ALT) && !(g_physical_modifiers & KF_ALT)) {
				printk(KERN_INFO "aw9523b: release logical alt\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTALT, 0);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers &= ~KF_ALT;
			}
			if ((g_logical_modifiers & KF_CTRL) && !(g_physical_modifiers & KF_CTRL)) {
				printk(KERN_INFO "aw9523b: release logical ctrl\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTCTRL, 0);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers &= ~KF_CTRL;
			}
			if ((g_logical_modifiers & KF_SHIFT) && !(g_physical_modifiers & KF_SHIFT)) {
				printk(KERN_INFO "aw9523b: release logical shift\n");
				input_report_key(aw9523b_input_dev, KEY_LEFTSHIFT, 0);
				input_sync(aw9523b_input_dev);
				g_logical_modifiers &= ~KF_SHIFT;
			}
			if (keycode == KEY_CAPSLOCK) {
				if (capslock_led_enable >= 2) {
					gpio_direction_output(pdata->gpio_caps_led, 0);
					capslock_led_enable = 0;
				}
			}
		}
	}
}

static void aw9523b_irq_work(struct work_struct *work)
{
	struct aw9523b_data *pdata = container_of(work, struct aw9523b_data, irq_work);
	u8 keyboard_state[AW9523_NR_PORTS];
	int idx;
	u8 p0_mask;

	aw9523b_disable_P0_interrupt();
	aw9523b_read_keyboard_state(keyboard_state);
	aw9523b_check_keys(pdata, keyboard_state);
	p0_mask = 0x00;
	for (idx = 0; idx < AW9523_NR_PORTS; ++idx) {
		if (keyboard_state[idx]) {
			p0_mask = 0xff; /* |= (1 << idx); */
		}
	}
	aw9523b_config_P0_input();
	aw9523b_enable_P0_interrupt_mask(p0_mask);
	if (p0_mask != 0x00) {
		printk(KERN_INFO "%s: enter polling mode: p0_mask=%02x\n", __func__, p0_mask);
		schedule_delayed_work(&pdata->poll_work, msecs_to_jiffies(g_poll_interval));
	}
	aw9523b_config_P1_output();
	aw9523b_disable_P1_interrupt();
	aw9523b_set_P1_value(OUT_LOW_VALUE);
	aw9523b_irq_enable(pdata);
}

static void aw9523b_poll_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct aw9523b_data *pdata = container_of(dwork, struct aw9523b_data, poll_work);
	u8 keyboard_state[AW9523_NR_PORTS];
	int idx;
	u8 p0_mask;
	unsigned int poll_delay = g_poll_interval;

	aw9523b_read_keyboard_state(keyboard_state);
	aw9523b_check_keys(pdata, keyboard_state);
	p0_mask = 0x00;
	for (idx = 0; idx < AW9523_NR_PORTS; ++idx) {
		if (keyboard_state[idx]) {
			p0_mask = 0xff; /* |= (1 << idx); */
		}
	}
	aw9523b_enable_P0_interrupt_mask(p0_mask);
	if (p0_mask == 0x00) {
		u8 val = ~aw9523b_get_P0_value();
		if (val) {
			/*
			 * A key was pressed between reading the keyboard
			 * state and enabling interrupts.  Reschedule ourself
			 * immediately and disable interrupts.
			 */
			poll_delay = 0;
			p0_mask = 0xff;
			aw9523b_enable_P0_interrupt_mask(p0_mask);
		}
	}
	if (p0_mask != 0x00) {
		schedule_delayed_work(&pdata->poll_work, msecs_to_jiffies(poll_delay));
	}
	else {
		printk(KERN_INFO "%s: enter irq mode\n", __func__);
	}
}

static irqreturn_t aw9523b_irq_handler(int irq, void *dev_id)
{
	struct aw9523b_data *pdata = dev_id;

	printk(KERN_INFO "%s: enter\n", __func__);
	aw9523b_irq_disable(pdata);
	schedule_work(&pdata->irq_work);

	return IRQ_HANDLED;
}

/* sysfs */

#ifdef DEBUG
static ssize_t aw9523b_show_regs(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	static const struct { u8 addr; const char *name; } reg_vec[] = {
		{ REG_INPUT_PORT0,  "Input_Port0" },
		{ REG_INPUT_PORT1,  "Input_Port1" },
		{ REG_OUTPUT_PORT0, "Output_Port0" },
		{ REG_OUTPUT_PORT1, "Output_Port1" },
		{ REG_CONFIG_PORT0, "Config_Port0" },
		{ REG_CONFIG_PORT1, "Config_Port1" },
		{ REG_INT_PORT0,    "Int_Port0" },
		{ REG_INT_PORT1,    "Int_Port1" },
		{ REG_IC_ID,        "ID" },
		{ REG_CTL,          "GCR" }
	};
	char *ptr = buf;
	u8 val;
	int n;

	for (n = 0; n < ARRAY_SIZE(reg_vec); ++n) {
	    aw9523b_read_reg(reg_vec[n].addr, &val);
	    ptr += sprintf(ptr, "%s=0x%02x\n", reg_vec[n].name, val);
	}
	BUG_ON((ptr - buf) >= PAGE_SIZE);

	return (ptr - buf);
}
static DEVICE_ATTR(regs, (S_IRUGO | S_IWUSR | S_IWGRP),
            aw9523b_show_regs,
            NULL);

static ssize_t aw9523b_show_keyboard_state(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *ptr = buf;
	u8 keyboard_state[AW9523_NR_PORTS];
	int n;

	aw9523b_read_keyboard_state(keyboard_state);
	for (n = 0; n < AW9523_NR_PORTS; ++n) {
		ptr += sprintf(ptr, "%02x ", keyboard_state[n]);
	}
	*(ptr - 1) = '\n';
	BUG_ON((ptr - buf) >= PAGE_SIZE);

	return (ptr - buf);
}
static DEVICE_ATTR(keyboard_state, (S_IRUGO | S_IWUSR | S_IWGRP),
            aw9523b_show_keyboard_state,
            NULL);
#endif

static ssize_t aw9523b_show_layout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *ptr = buf;

	ptr += sprintf(ptr, "%s", keylayout_name(g_layout));
	if (g_layout_modified) {
		ptr += sprintf(ptr, " (modified)");
	}
	ptr += sprintf(ptr, "\n");

	return (ptr - buf);
}

static ssize_t aw9523b_store_layout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	char namebuf[80];
	const char *eol;

	if (count >= sizeof(namebuf)) {
		return -EINVAL;
	}
	memset(namebuf, 0, sizeof(namebuf));
	eol = memchr(buf, '\n', count);
	if (eol) {
		memcpy(namebuf, buf, eol - buf);
	}
	else {
		memcpy(namebuf, buf, count);
	}
	g_layout = keylayout_enum(namebuf);
	g_layout_modified = false;
	switch (g_layout) {
	case LAYOUT_QWERTY:
		memcpy(key_array, qwerty_keys, sizeof(key_array));
		memcpy(key_fn_array, qwerty_fn_keys, sizeof(key_fn_array));
		break;
	case LAYOUT_QWERTZ:
		memcpy(key_array, qwertz_keys, sizeof(key_array));
		memcpy(key_fn_array, qwertz_fn_keys, sizeof(key_fn_array));
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t aw9523b_show_keymap(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *ptr = buf;
	char *end = buf + PAGE_SIZE;
	int n;

	for (n = 0; n < AW9523_NR_KEYS; ++n) {
		ptr += snprintf(ptr, (end - ptr), "%d:%04hx:%04hx\n",
				n, key_array[n], key_fn_array[n]);
	}

	return (ptr - buf);
}

static ssize_t aw9523b_store_keymap(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	char keybuf[80];
	const char *end = buf + count;
	const char *eol;
	int key_idx;
	u16 key_val, key_fn_val;

	while (buf < end) {
		memset(keybuf, 0, sizeof(keybuf));
		eol = memchr(buf, '\n', end - buf);
		if (eol) {
			if (eol - buf >= sizeof(keybuf)) {
				return -EINVAL;
			}
			memcpy(keybuf, buf, eol - buf);
			buf = eol + 1;
		}
		else {
			memcpy(keybuf, buf, end - buf);
			buf = end;
		}
		if (sscanf(keybuf, "%d:%hx:%hx", &key_idx, &key_val, &key_fn_val) != 3) {
			return -EINVAL;
		}
		if (key_idx < 0 || key_idx >= AW9523_NR_KEYS) {
			return -EINVAL;
		}
		key_array[key_idx] = key_val;
		key_fn_array[key_idx] = key_fn_val;
		g_layout_modified = true;
	}

	return count;
}

static ssize_t aw9523b_show_poll_interval(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *ptr = buf;
	char *end = buf + PAGE_SIZE;

	ptr += snprintf(ptr, (end - ptr), "%u\n", g_poll_interval);

	return (ptr - buf);
}

static ssize_t aw9523b_store_poll_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val) != 0)
		return -EINVAL;
	if (val < AW9523_MIN_POLL_MS || val > AW9523_MAX_POLL_MS)
		return -EINVAL;

	g_poll_interval = val;

	return count;
}

static DEVICE_ATTR(layout, (S_IRUGO | S_IWUSR | S_IWGRP),
	aw9523b_show_layout,
	aw9523b_store_layout);

static DEVICE_ATTR(keymap, (S_IRUGO | S_IWUSR | S_IWGRP),
	aw9523b_show_keymap,
	aw9523b_store_keymap);

static DEVICE_ATTR(poll_interval, (S_IRUGO | S_IWUSR | S_IWGRP),
	aw9523b_show_poll_interval,
	aw9523b_store_poll_interval);

static struct attribute *aw9523b_attrs[] = {
#ifdef DEBUG
	&dev_attr_regs.attr,
	&dev_attr_keyboard_state.attr,
#endif
	&dev_attr_layout.attr,
	&dev_attr_keymap.attr,
	&dev_attr_poll_interval.attr,
	NULL
};

static const struct attribute_group aw9523b_attr_group = {
	.attrs = aw9523b_attrs,
};

/* framebuffer */

static int qx1000_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct aw9523b_data *pdata = container_of(self, struct aw9523b_data, fb_client);
	int *blank;

	if (event == FB_EVENT_BLANK) {
		blank = evdata->data;
		pdata->fb_blanked = (*blank != FB_BLANK_UNBLANK);
	}

	return 0;
}

static int register_aw9523b_input_dev(struct device *pdev)
{
	int key;

	aw9523b_input_dev = input_allocate_device();
	if (!aw9523b_input_dev)
		return -ENOMEM;

	aw9523b_input_dev->name = "Builtin Keyboard";
	aw9523b_input_dev->id.bustype = BUS_HOST;
	aw9523b_input_dev->id.vendor = 0x0;
	aw9523b_input_dev->id.product = 0x0;
	aw9523b_input_dev->id.version = 0x0;

	__set_bit(EV_KEY, aw9523b_input_dev->evbit);

	/* We can potentially generate all keys due to remapping */
	for (key = 1; key < 0xff; ++key) {
		if (key >= KEY_STYLUS_MIN && key < KEY_STYLUS_MAX)
			continue;
		input_set_capability(aw9523b_input_dev, EV_KEY, key);
	}

	aw9523b_input_dev->dev.parent = pdev;

	return 0;
}

static int aw9523b_power_ctl(struct aw9523b_data *data, bool on)
{
	return 0;
}

static int aw9523b_power_init(struct aw9523b_data *data)
{
	return 0;
}

static int aw9523b_power_deinit(struct aw9523b_data *data)
{
        return 0;
}

#ifdef CONFIG_OF
static int aw9523b_parse_dt(struct device *dev,
            struct aw9523b_data *pdata)
{
	int err = 0;
	struct device_node *np = dev->of_node;

	pdata->gpio_rst = of_get_named_gpio(np, "awinic,reset-gpio", 0);
	if (gpio_is_valid(pdata->gpio_rst)) {
		err = gpio_request(pdata->gpio_rst, "aw9523b_reset_gpio");
		if (err) {
			dev_err(&pdata->client->dev, "pdata->gpio_rst gpio request failed");
			return -ENODEV;
		}
		err = gpio_direction_output(pdata->gpio_rst, 1);
		if (err) {
			dev_err(&pdata->client->dev,
				"set_direction for pdata->gpio_rst failed\n");
			return -ENODEV;
		}
	}
	else {
		dev_err(dev, "pdata->gpio_rst is error\n");
		return pdata->gpio_rst;
	}
	pdata->gpio_caps_led = of_get_named_gpio(np, "awinic,caps-gpio", 0);
	if (gpio_is_valid(pdata->gpio_caps_led)) {
		err = gpio_request(pdata->gpio_caps_led, "aw9523b_gpio_caps_led");
		if (err) {
			dev_err(&pdata->client->dev, "pdata->gpio_caps_led gpio request failed");
			return -ENODEV;
		}
	}
	else {
		dev_err(dev, "pdata->gpio_caps_led is error\n");
		return pdata->gpio_caps_led;
	}

	pdata->gpio_irq = of_get_named_gpio(np, "awinic,irq-gpio", 0);
	if (gpio_is_valid(pdata->gpio_irq)) {
		err = gpio_request(pdata->gpio_irq, "aw9523b_irq_gpio");
		if (err) {
			dev_err(&pdata->client->dev, "pdata->gpio_rst gpio request failed");
			return -ENODEV;
		}
		err = gpio_direction_input(pdata->gpio_irq);
		if (err) {
			dev_err(&pdata->client->dev,
				"set_direction for pdata->gpio_irq failed\n");
			return -ENODEV;
		}
	}
	else {
		dev_err(dev, "pdata->gpio_irq is error\n");
		return pdata->gpio_irq;
	}

	return 0;
}
#else
static int aw9523b_parse_dt(struct device *dev,
            struct aw9523b_data *pdata)
{
	return -EINVAL;
}
#endif

static int aw9523b_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
	int err = 0;
	struct aw9523b_data *pdata;

	printk("%s begin\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		err = -EPERM;
		goto exit;
	}
	pdata = kzalloc(sizeof(struct aw9523b_data), GFP_KERNEL);
	if (!pdata) {
		err = -ENOMEM;
		goto exit;
	}
    
	if (client->dev.of_node) {
		err = aw9523b_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto pdata_free_exit;
		}
	}

	spin_lock_init(&pdata->irq_lock);   
	g_client = client;
	i2c_set_clientdata(client, pdata);
	pdata->client = client;

	/* Set initial keylayout */
	g_layout = LAYOUT_QWERTY;
	memcpy(key_array, qwerty_keys, sizeof(key_array));
	memcpy(key_fn_array, qwerty_fn_keys, sizeof(key_fn_array));

	g_poll_interval = AW9523_DEFAULT_POLL_MS;

	err = aw9523b_power_init(pdata);
	if (err) {
		dev_err(&client->dev, "Failed to get aw9523b regulators\n");
		err = -EINVAL;
		goto free_input_dev;
	}
	err = aw9523b_power_ctl(pdata, true);
	if (err) {
		dev_err(&client->dev, "Failed to enable aw9523b power\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}

	err = aw9523b_hw_reset(pdata);
	if (err) {
		dev_err(&client->dev, "aw9523b failed to write\n");
		goto deinit_power_exit;
	}
	if (err) {
		dev_err(&client->dev, "aw9523b failed to reset\n");
	}

	if (aw9523b_chip_id() != AW9523_CHIP_ID) {
		dev_err(&client->dev, "aw9523b failed to read\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}
	
	err = register_aw9523b_input_dev(&client->dev);
	if (err) {
		dev_err(&client->dev, "Failed to register aw9523b input device\n");
		goto free_i2c_clientdata_exit;
	}

	err = sysfs_create_group(&client->dev.kobj, &aw9523b_attr_group);
	if (err) {
		dev_err(&client->dev, "aw9523b failed to register sysfs\n");
	}

	pdata->fb_client.notifier_call = qx1000_fb_notifier_callback;
	err = fb_register_client(&pdata->fb_client);
	if (err) {
		dev_err(&client->dev, "aw9523b failed to register framebuffer client\n");
	}

	/* Configure the chip */
	aw9523b_config_P0_input();
	aw9523b_enable_P0_interrupt();
	aw9523b_config_P1_output();
	aw9523b_disable_P1_interrupt();
	aw9523b_set_P1_value(OUT_LOW_VALUE);
	aw9523b_get_P0_value();
	aw9523b_get_P1_value();

	INIT_WORK(&pdata->irq_work, aw9523b_irq_work);
	INIT_DELAYED_WORK(&pdata->poll_work, aw9523b_poll_work);
	pdata->irq_is_disabled = false;
	err = request_irq(client->irq,
			  aw9523b_irq_handler,
			  IRQ_TYPE_LEVEL_LOW,
			  client->name, pdata);
	if (err) {
		dev_err(&client->dev, "failed to request irq\n");
		goto deinit_power_exit;
	}
	aw9523b_irq_enable(pdata);
	dev_info(&client->dev, "probe successful\n");

	return 0;

deinit_power_exit:
	aw9523b_power_deinit(pdata);
free_input_dev:
	input_free_device(aw9523b_input_dev);
pdata_free_exit:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
free_i2c_clientdata_exit:
	i2c_set_clientdata(client, NULL);    
	kfree(pdata);
exit:
	return err;
}


static int aw9523b_remove(struct i2c_client *client)
{
	struct aw9523b_data *data = i2c_get_clientdata(client);
    
	cancel_delayed_work_sync(&data->poll_work);

	aw9523b_power_deinit(data);
	i2c_set_clientdata(client, NULL);

	kfree(data);

	return 0;
}

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	int state;
	u16 mask = 0;
	u16 keycode = button->code;
	bool report = true;

	state = (__gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
 printk(KERN_INFO "aw9523b: gpio_keys_gpio_report_event: desc=%s code=%u state=%d\n",
     (button->desc ? button->desc : "(null)"), button->code, state);
	if (state < 0) {
		dev_err(input->dev.parent, "failed to get gpio state\n");
		return;
	}
	if (button->type != EV_KEY) {
		dev_err(input->dev.parent, "button is not a key\n");
		return;
	}
	if (button->code == KEY_FN) {
		mask = KF_FN;
		keycode = KEY_FN;
		report = false;
	}
	if (button->code == KEY_LEFTALT || button->code == KEY_RIGHTALT) {
		mask = KF_ALT;
		keycode = KEY_LEFTALT;
	}
	if (button->code == KEY_LEFTCTRL || button->code == KEY_RIGHTCTRL) {
		mask = KF_CTRL;
		keycode = KEY_LEFTCTRL;
	}
	if (button->code == KEY_LEFTSHIFT || button->code == KEY_RIGHTSHIFT) {
		mask = KF_SHIFT;
		keycode = KEY_LEFTSHIFT;
	}

	if (mask) {
		if (state) {
			g_physical_modifiers |= mask;
			if (report && !(g_logical_modifiers & mask)) {
				input_report_key(input, keycode, 1);
				input_sync(input);
				g_logical_modifiers |= mask;
			}
		}
		else {
			g_physical_modifiers &= ~mask;
			if (report && (g_logical_modifiers & mask)) {
				input_report_key(input, keycode, 0);
				input_sync(input);
				g_logical_modifiers &= ~mask;
			}
		}
	}
	else {
		input_report_key(input, keycode, state);
		input_sync(input);
	}
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);
 
	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);

	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}

static void gpio_keys_quiesce_key(void *data)
{
	struct gpio_button_data *bdata = data;

	if (gpio_is_valid(bdata->button->gpio))
		cancel_delayed_work_sync(&bdata->work);
	else
		del_timer_sync(&bdata->release_timer);
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

printk(KERN_INFO "aw9523b: gpio_keys_setup_key: desc=%s gpio=%d irq=%d\n", (button->desc ? button->desc : "(null)"), button->gpio, button->irq);
	if (!gpio_is_valid(button->gpio)) {
		printk(KERN_ERR "aw9523b: gpio_keys_setup_key: gpio is not valid\n");
		return -EINVAL;
	}

	error = devm_gpio_request_one(&pdev->dev, button->gpio,
				      GPIOF_IN, desc);
	if (error < 0) {
		dev_err(dev, "Failed to request GPIO %d, error %d\n",
			button->gpio, error);
		return error;
	}

	if (button->debounce_interval) {
		error = gpio_set_debounce(button->gpio,
				button->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0)
			bdata->software_debounce =
					button->debounce_interval;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		return error;
	}
	bdata->irq = irq;

	INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(&pdev->dev, gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(&pdev->dev,
			"failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(&pdev->dev, bdata->irq,
					     gpio_keys_gpio_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_pinctrl_configure(struct gpio_keys_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"idea_gpio_key_active");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"idea_gpio_key_suspend");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get gpiokey pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		dev_err(&ddata->input->dev,
				"cannot set ts pinctrl active state\n");
		return retval;
	}

	return 0;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);
	pdata->name = of_get_property(node, "input-name", NULL);
	pdata->use_syscore = of_property_read_bool(node, "use-syscore");

	i = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;

		button = &pdata->buttons[i++];

		button->gpio = of_get_gpio_flags(pp, 0, &flags);
		if (button->gpio < 0) {
			error = button->gpio;
			if (error != -ENOENT) {
				if (error != -EPROBE_DEFER)
					dev_err(dev,
						"Failed to get gpio flags, error: %d\n",
						error);
				return ERR_PTR(error);
			}
		} else {
			button->active_low = flags & OF_GPIO_ACTIVE_LOW;
		}

		button->irq = irq_of_parse_and_map(pp, 0);

		if (!gpio_is_valid(button->gpio) && !button->irq) {
			dev_err(dev, "Found button without gpios or irqs\n");
			return ERR_PTR(-EINVAL);
		}

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = of_property_read_bool(pp, "wakeup-source") ||
				 /* legacy name */
				 of_property_read_bool(pp, "gpio-key,wakeup");

		button->can_disable = !!of_get_property(pp, "linux,can-disable", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					&button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "idea-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;
	struct pinctrl_state *set_state;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	input = aw9523b_input_dev;//devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	global_dev = dev;
	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	//input->id.bustype = BUS_HOST;
	input->id.vendor = 0x181d;
	input->id.product = 0x5018;
	input->id.version = 0x0002;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	/* Get pinctrl if target uses pinctrl */
	ddata->key_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "cannot set ts pinctrl active state\n");
			return error;
		}
	}

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto err_setup_key;

		if (button->wakeup)
			wakeup = 1;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_remove_group;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	if (pdata->use_syscore)
		gpio_keys_syscore_pm_ops.resume = gpio_keys_syscore_resume;

	register_syscore_ops(&gpio_keys_syscore_pm_ops);

	return 0;

err_remove_group:
err_setup_key:
	if (ddata->key_pinctrl) {
		set_state =
		pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_gpio_key_suspend");
		if (IS_ERR(set_state))
			dev_err(dev, "cannot get gpiokey pinctrl sleep state\n");
		else
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	unregister_syscore_ops(&gpio_keys_syscore_pm_ops);

	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void gpio_keys_syscore_resume(void)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(global_dev);
	struct input_dev *input = ddata->input;
	struct gpio_button_data *bdata = NULL;
	int error = 0;
	int i;

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(global_dev, "failed to put the pin in resume state\n");
			return;
		}
	}

	if (device_may_wakeup(global_dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return;

	gpio_keys_report_state(ddata);
}

static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i, ret;

	if (ddata->key_pinctrl) {
		ret = gpio_keys_pinctrl_configure(ddata, false);
		if (ret) {
			dev_err(dev, "failed to put the pin in suspend state\n");
			return ret;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (ddata->pdata->use_syscore == true) {
		dev_dbg(global_dev, "Using syscore resume, no need of this resume.\n");
		return 0;
	}

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "failed to put the pin in resume state\n");
			return error;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	//gpio_keys_report_state(ddata);
	return 0;
}

#else

static void gpio_keys_syscore_resume(void){}

static int gpio_keys_suspend(struct device *dev)
{
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "idea-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	printk(KERN_INFO "aw9523b: gpio_keys_init called\n");
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);


static const struct i2c_device_id aw9523b_id[] = {
	{ AWINIC_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, aw9523b_id);

static const struct of_device_id aw9523b_of_match[] = {
	{ .compatible = "awinic,aw9523b", },
	{ },
};

static struct i2c_driver aw9523b_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= AWINIC_NAME,
		.of_match_table	= aw9523b_of_match,
	},
	.id_table	= aw9523b_id,
	.probe		= aw9523b_probe,
	.remove		= aw9523b_remove,
};

static int __init aw9523b_init(void)
{
	return i2c_add_driver(&aw9523b_driver);
}

static void __exit aw9523b_exit(void)
{
	i2c_del_driver(&aw9523b_driver);
}

MODULE_AUTHOR("Awinic Technology <contact@awinic.com>");
MODULE_DESCRIPTION("aw9523b keyboard gpio driver");
MODULE_LICENSE("GPL v2");

module_init(aw9523b_init);
module_exit(aw9523b_exit);

