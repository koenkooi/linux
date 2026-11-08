// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ Micro Keyboard and HP iPAQ Foldable Keyboard
 *
 * Both keyboards plug into the sync connector of an iPAQ and send to its
 * serial port at 4800 baud, 8N1; the host never sends anything back. A key
 * event is two bytes: the key code, with bit 7 set on release, then the
 * bitwise complement of the key code without bit 7. A pair with code 0x64
 * or 0x38 is not a key but identifies the keyboard model, and code 0x75
 * restarts the keyboard. Each physical key is reported under one key code;
 * the Fn layers are left to the keymap of the session.
 *
 * Attach it to the tty with the serport line discipline (N_MOUSE) and
 * SPIOCSTYPE SERIO_IPAQ_KBD.
 */

#include <linux/cleanup.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/serio.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define DRIVER_DESC	"HP iPAQ serial keyboard driver"

#define IPAQ_KBD_CODE_MASK	0x7f
#define IPAQ_KBD_RELEASE	0x80
#define IPAQ_KBD_NUM_CODES	128

/*
 * The HP WinCE drivers hand over between the Micro and the Foldable driver
 * when 0x38 arrives, and a later HP driver picks one of two translation
 * tables for 0x64 or 0x38, so 0x38 is taken to be the Foldable and 0x64 the
 * Micro. The WinCE drivers also suggest that 0x75 announces the Micro, but
 * they disagree with each other on it; it is handled as a restart, as the
 * handhelds.org 2.4 driver did, and does not change the table.
 */
#define IPAQ_KBD_ID_MICRO	0x64
#define IPAQ_KBD_ID_FOLDABLE	0x38
#define IPAQ_KBD_RESTART	0x75

enum ipaq_kbd_variant {
	IPAQ_KBD_MICRO,
	IPAQ_KBD_FOLDABLE,
	IPAQ_KBD_NUM_VARIANTS,
};

/* scancode = variant << 7 | key code, as seen by EVIOCGKEYCODE and MSC_SCAN */
#define IPAQ_KBD_SCAN(variant, code)	(((variant) << 7) | (code))
#define IPAQ_KBD_KEYMAP_SIZE	(IPAQ_KBD_NUM_VARIANTS * IPAQ_KBD_NUM_CODES)

/*
 * The Micro has two Fn keys, an orange one on the left and a blue one on
 * the right, and the Foldable has one; they select the second and third
 * legends of the other keys. Each physical key is reported under one key
 * code whatever Fn does, and the layers are left to the keymap of the
 * session. The Fn keys therefore need key codes that an X11 server can see
 * (X11 key codes stop at 255, evdev code 247, which rules out KEY_FN) and
 * that no XKB layout binds, so that they do nothing until a layout for
 * this keyboard gives them a meaning: xkeyboard-config leaves KEY_MACRO and
 * KEY_ISO unbound.
 */
#define IPAQ_KBD_KEY_FN_ORANGE	KEY_MACRO
#define IPAQ_KBD_KEY_FN_BLUE	KEY_ISO

enum ipaq_kbd_event_type {
	IPAQ_KBD_EV_NONE,
	IPAQ_KBD_EV_KEY,
	IPAQ_KBD_EV_IDENTITY,
	IPAQ_KBD_EV_RESTART,
	IPAQ_KBD_EV_RESYNC,
};

struct ipaq_kbd_decoder {
	u8 first;
	bool have_first;
};

struct ipaq_kbd_event {
	enum ipaq_kbd_event_type type;
	u8 code;
	bool release;
	enum ipaq_kbd_variant variant;
};

static void ipaq_kbd_decoder_reset(struct ipaq_kbd_decoder *dec)
{
	dec->have_first = false;
	dec->first = 0;
}

/*
 * Feed one received byte. A pair whose second byte is not the complement
 * of the first yields IPAQ_KBD_EV_RESYNC and keeps the second byte as the
 * possible start of the next pair, so a lost byte costs one event and the
 * decoder can never stay out of step.
 */
static enum ipaq_kbd_event_type ipaq_kbd_decode(struct ipaq_kbd_decoder *dec,
						u8 byte,
						struct ipaq_kbd_event *ev)
{
	u8 code;

	ev->type = IPAQ_KBD_EV_NONE;

	if (!dec->have_first) {
		dec->first = byte;
		dec->have_first = true;
		return ev->type;
	}

	code = dec->first & IPAQ_KBD_CODE_MASK;
	if ((code ^ byte) != 0xff) {
		dec->first = byte;
		ev->type = IPAQ_KBD_EV_RESYNC;
		return ev->type;
	}

	ev->code = code;
	ev->release = dec->first & IPAQ_KBD_RELEASE;
	ipaq_kbd_decoder_reset(dec);

	switch (code) {
	case IPAQ_KBD_ID_MICRO:
		ev->type = IPAQ_KBD_EV_IDENTITY;
		ev->variant = IPAQ_KBD_MICRO;
		break;
	case IPAQ_KBD_ID_FOLDABLE:
		ev->type = IPAQ_KBD_EV_IDENTITY;
		ev->variant = IPAQ_KBD_FOLDABLE;
		break;
	case IPAQ_KBD_RESTART:
		ev->type = IPAQ_KBD_EV_RESTART;
		break;
	default:
		ev->type = IPAQ_KBD_EV_KEY;
		break;
	}

	return ev->type;
}

#define MICRO(code)		IPAQ_KBD_SCAN(IPAQ_KBD_MICRO, code)
#define FOLDABLE(code)		IPAQ_KBD_SCAN(IPAQ_KBD_FOLDABLE, code)

/*
 * One key code per physical key, named after the key's main legend; what
 * Fn selects is up to the keymap of the session. The Micro's codes were
 * measured on a real keyboard and do not follow the WinCE KeybdG350 tables
 * (which put Q at 0x15; the wire sends 0x35), so the WinCE-derived Micro
 * entries are gone. The Foldable codes come from the base translation
 * table of the HP WinCE driver (KeybdG750 1.67) and, where it has them,
 * the handhelds.org 2.4 h3600_microkbd.c; entries both agree on are not
 * marked. "inferred" marks a Linux key chosen from what the WinCE driver
 * does with the key, not from a legend or a measurement.
 *
 * A code missing here is reported as KEY_UNKNOWN with its scancode, and
 * EVIOCSKEYCODE corrects the table at run time.
 */
static const unsigned short ipaq_kbd_keymap[IPAQ_KBD_KEYMAP_SIZE] = {
	/* The Micro's 37 keys, measured on a real keyboard (two guided captures agree) */
	[MICRO(0x35)] = KEY_Q,
	[MICRO(0x13)] = KEY_W,
	[MICRO(0x70)] = KEY_E,
	[MICRO(0x2d)] = KEY_R,
	[MICRO(0x23)] = KEY_T,
	[MICRO(0x4d)] = KEY_Y,
	[MICRO(0x34)] = KEY_U,
	[MICRO(0x5d)] = KEY_I,
	[MICRO(0x49)] = KEY_O,
	[MICRO(0x32)] = KEY_P,
	[MICRO(0x58)] = KEY_A,
	[MICRO(0x1b)] = KEY_S,
	[MICRO(0x1c)] = KEY_D,
	[MICRO(0x11)] = KEY_F,
	[MICRO(0x24)] = KEY_G,
	[MICRO(0x54)] = KEY_H,
	[MICRO(0x42)] = KEY_J,
	[MICRO(0x4b)] = KEY_K,
	[MICRO(0x4a)] = KEY_L,
	[MICRO(0x26)] = KEY_BACKSPACE,
	[MICRO(0x02)] = IPAQ_KBD_KEY_FN_ORANGE,
	[MICRO(0x2c)] = KEY_Z,
	[MICRO(0x27)] = KEY_X,
	[MICRO(0x3c)] = KEY_C,
	[MICRO(0x44)] = KEY_V,
	[MICRO(0x73)] = KEY_B,
	[MICRO(0x2b)] = KEY_N,
	[MICRO(0x4c)] = KEY_M,
	[MICRO(0x76)] = IPAQ_KBD_KEY_FN_BLUE,
	[MICRO(0x79)] = KEY_ENTER,
	[MICRO(0x12)] = KEY_LEFTSHIFT,
	[MICRO(0x41)] = KEY_LEFT,
	[MICRO(0x7d)] = KEY_RIGHT,
	[MICRO(0x3b)] = KEY_SPACE,
	[MICRO(0x7c)] = KEY_DOWN,
	[MICRO(0x31)] = KEY_UP,
	[MICRO(0x45)] = KEY_LEFTCTRL,

	[FOLDABLE(0x02)] = IPAQ_KBD_KEY_FN_ORANGE,
	/* WinCE: the Windows key; 2.4: F11 */
	[FOLDABLE(0x07)] = KEY_LEFTMETA,
	[FOLDABLE(0x0d)] = KEY_TAB,
	/* WinCE: grave; 2.4: apostrophe, which 0x52 already is */
	[FOLDABLE(0x0e)] = KEY_GRAVE,
	[FOLDABLE(0x11)] = KEY_LEFTALT,
	[FOLDABLE(0x12)] = KEY_LEFTSHIFT,
	[FOLDABLE(0x14)] = KEY_LEFTCTRL,
	[FOLDABLE(0x15)] = KEY_Q,
	[FOLDABLE(0x16)] = KEY_1,
	[FOLDABLE(0x1a)] = KEY_Z,
	[FOLDABLE(0x1b)] = KEY_S,
	[FOLDABLE(0x1c)] = KEY_A,
	[FOLDABLE(0x1d)] = KEY_W,
	[FOLDABLE(0x1e)] = KEY_2,
	[FOLDABLE(0x21)] = KEY_C,
	[FOLDABLE(0x22)] = KEY_X,
	[FOLDABLE(0x23)] = KEY_D,
	[FOLDABLE(0x24)] = KEY_E,
	[FOLDABLE(0x25)] = KEY_4,
	[FOLDABLE(0x26)] = KEY_3,
	[FOLDABLE(0x28)] = KEY_UP,
	[FOLDABLE(0x2a)] = KEY_V,
	[FOLDABLE(0x2b)] = KEY_F,
	[FOLDABLE(0x2c)] = KEY_T,
	[FOLDABLE(0x2d)] = KEY_R,
	[FOLDABLE(0x2e)] = KEY_5,
	[FOLDABLE(0x2f)] = KEY_RIGHT,
	[FOLDABLE(0x31)] = KEY_N,
	[FOLDABLE(0x32)] = KEY_B,
	[FOLDABLE(0x33)] = KEY_H,
	[FOLDABLE(0x34)] = KEY_G,
	[FOLDABLE(0x35)] = KEY_Y,
	[FOLDABLE(0x36)] = KEY_6,
	[FOLDABLE(0x3a)] = KEY_M,
	[FOLDABLE(0x3b)] = KEY_J,
	[FOLDABLE(0x3c)] = KEY_U,
	[FOLDABLE(0x3d)] = KEY_7,
	[FOLDABLE(0x3e)] = KEY_8,
	[FOLDABLE(0x41)] = KEY_COMMA,
	[FOLDABLE(0x42)] = KEY_K,
	[FOLDABLE(0x43)] = KEY_I,
	[FOLDABLE(0x44)] = KEY_O,
	[FOLDABLE(0x45)] = KEY_0,
	[FOLDABLE(0x46)] = KEY_9,
	[FOLDABLE(0x49)] = KEY_DOT,
	[FOLDABLE(0x4a)] = KEY_SLASH,
	[FOLDABLE(0x4b)] = KEY_L,
	[FOLDABLE(0x4c)] = KEY_SEMICOLON,
	[FOLDABLE(0x4d)] = KEY_P,
	[FOLDABLE(0x4e)] = KEY_MINUS,
	[FOLDABLE(0x52)] = KEY_APOSTROPHE,
	[FOLDABLE(0x54)] = KEY_LEFTBRACE,
	[FOLDABLE(0x55)] = KEY_EQUAL,
	[FOLDABLE(0x58)] = KEY_CAPSLOCK,
	[FOLDABLE(0x59)] = KEY_RIGHTSHIFT,
	[FOLDABLE(0x5a)] = KEY_ENTER,
	[FOLDABLE(0x5b)] = KEY_RIGHTBRACE,
	[FOLDABLE(0x5c)] = KEY_SPACE,
	[FOLDABLE(0x5d)] = KEY_BACKSLASH,
	[FOLDABLE(0x5e)] = KEY_LEFT,
	[FOLDABLE(0x60)] = KEY_DOWN,
	[FOLDABLE(0x66)] = KEY_DELETE,
	/* inferred: WinCE brings the Today screen to the front */
	[FOLDABLE(0x70)] = KEY_HOMEPAGE,
	[FOLDABLE(0x71)] = KEY_BACKSPACE,
	[FOLDABLE(0x73)] = KEY_RIGHTCTRL,
};

#undef MICRO
#undef FOLDABLE

struct ipaq_kbd {
	struct input_dev *input;
	struct serio *serio;
	/* serializes the decoder and key state against teardown */
	spinlock_t lock;
	struct ipaq_kbd_decoder dec;
	enum ipaq_kbd_variant variant;
	/*
	 * The key code and scancode each held key went down with, so that
	 * its release matches the press even when the table or the keymap
	 * changed while it was held.
	 */
	unsigned short down_key[IPAQ_KBD_NUM_CODES];
	unsigned short down_scan[IPAQ_KBD_NUM_CODES];
	unsigned short keymap[IPAQ_KBD_KEYMAP_SIZE];
	char phys[40];
};

static bool ipaq_kbd_is_frame_code(unsigned int code)
{
	return code == IPAQ_KBD_ID_MICRO || code == IPAQ_KBD_ID_FOLDABLE ||
	       code == IPAQ_KBD_RESTART;
}

/*
 * Every code that can reach the key path gets a key code, so that a key
 * missing from the table still produces an event a capture can see.
 * KEY_UNKNOWN is what sparse-keymap reports in that case, and no XKB
 * layout binds it.
 */
static void ipaq_kbd_init_keymap(unsigned short *keymap)
{
	unsigned int i;

	memcpy(keymap, ipaq_kbd_keymap, sizeof(ipaq_kbd_keymap));
	for (i = 0; i < IPAQ_KBD_KEYMAP_SIZE; i++)
		if (keymap[i] == KEY_RESERVED &&
		    !ipaq_kbd_is_frame_code(i & IPAQ_KBD_CODE_MASK))
			keymap[i] = KEY_UNKNOWN;
}

/*
 * Several codes can share a key code (KEY_UNKNOWN, or a keymap changed
 * through EVIOCSKEYCODE); the key is down while any of them holds it.
 */
static bool ipaq_kbd_key_held(struct ipaq_kbd *kbd, unsigned short key)
{
	unsigned int i;

	for (i = 0; i < IPAQ_KBD_NUM_CODES; i++)
		if (kbd->down_key[i] == key)
			return true;

	return false;
}

static void ipaq_kbd_release(struct ipaq_kbd *kbd, u8 code)
{
	unsigned short key = kbd->down_key[code];

	kbd->down_key[code] = KEY_RESERVED;
	input_event(kbd->input, EV_MSC, MSC_SCAN, kbd->down_scan[code]);
	if (!ipaq_kbd_key_held(kbd, key))
		input_report_key(kbd->input, key, 0);
}

static void ipaq_kbd_release_all(struct ipaq_kbd *kbd)
{
	bool any = false;
	unsigned int i;

	for (i = 0; i < IPAQ_KBD_NUM_CODES; i++) {
		if (!kbd->down_key[i])
			continue;
		ipaq_kbd_release(kbd, i);
		any = true;
	}

	if (any)
		input_sync(kbd->input);
}

static void ipaq_kbd_key(struct ipaq_kbd *kbd, u8 code, bool release)
{
	unsigned int scan;
	unsigned short key;

	if (release) {
		if (!kbd->down_key[code])
			return;
		ipaq_kbd_release(kbd, code);
		input_sync(kbd->input);
		return;
	}

	/* the input core does autorepeat; a second press is a duplicate */
	if (kbd->down_key[code])
		return;

	scan = IPAQ_KBD_SCAN(kbd->variant, code);
	key = kbd->keymap[scan];

	/*
	 * Esc is printed in orange on the Micro's Ctrl key. Without any Esc
	 * the device would not be classified as a keyboard by userspace
	 * (udev's input_id wants KEY_ESC .. KEY_S), so this one Fn combination
	 * is translated here. down_key[] records the key that was reported, so
	 * the release matches whatever Fn does in between.
	 */
	if (kbd->variant == IPAQ_KBD_MICRO && key == KEY_LEFTCTRL &&
	    ipaq_kbd_key_held(kbd, IPAQ_KBD_KEY_FN_ORANGE))
		key = KEY_ESC;

	input_event(kbd->input, EV_MSC, MSC_SCAN, scan);
	if (key != KEY_RESERVED) {
		if (!ipaq_kbd_key_held(kbd, key))
			input_report_key(kbd->input, key, 1);
		kbd->down_key[code] = key;
		kbd->down_scan[code] = scan;
	}
	input_sync(kbd->input);
}

static void ipaq_kbd_process_byte(struct ipaq_kbd *kbd, u8 data,
				  unsigned int flags)
{
	struct ipaq_kbd_event ev;

	/* a byte with a framing or parity error cannot be trusted as data */
	if (flags & (SERIO_FRAME | SERIO_PARITY)) {
		ipaq_kbd_decoder_reset(&kbd->dec);
		ipaq_kbd_release_all(kbd);
		return;
	}

	switch (ipaq_kbd_decode(&kbd->dec, data, &ev)) {
	case IPAQ_KBD_EV_NONE:
		break;
	case IPAQ_KBD_EV_KEY:
		ipaq_kbd_key(kbd, ev.code, ev.release);
		break;
	case IPAQ_KBD_EV_IDENTITY:
		ipaq_kbd_release_all(kbd);
		kbd->variant = ev.variant;
		break;
	case IPAQ_KBD_EV_RESTART:
		ipaq_kbd_release_all(kbd);
		break;
	case IPAQ_KBD_EV_RESYNC:
		/* a lost byte may have been a release */
		ipaq_kbd_release_all(kbd);
		break;
	}
}

static irqreturn_t ipaq_kbd_interrupt(struct serio *serio, unsigned char data,
				      unsigned int flags)
{
	struct ipaq_kbd *kbd = serio_get_drvdata(serio);

	guard(spinlock_irqsave)(&kbd->lock);
	ipaq_kbd_process_byte(kbd, data, flags);

	return IRQ_HANDLED;
}

static int ipaq_kbd_connect(struct serio *serio, struct serio_driver *drv)
{
	struct input_dev *input;
	struct ipaq_kbd *kbd;
	unsigned int i;
	int err;

	kbd = kzalloc_obj(*kbd);
	input = input_allocate_device();
	if (!kbd || !input) {
		err = -ENOMEM;
		goto err_free;
	}

	kbd->serio = serio;
	kbd->input = input;
	kbd->variant = IPAQ_KBD_MICRO;
	spin_lock_init(&kbd->lock);
	ipaq_kbd_decoder_reset(&kbd->dec);
	ipaq_kbd_init_keymap(kbd->keymap);
	snprintf(kbd->phys, sizeof(kbd->phys), "%s/input0", serio->phys);

	input->name = "HP iPAQ Serial Keyboard";
	input->phys = kbd->phys;
	input->id.bustype = BUS_RS232;
	input->id.vendor = SERIO_IPAQ_KBD;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input->dev.parent = &serio->dev;

	input->keycode = kbd->keymap;
	input->keycodesize = sizeof(kbd->keymap[0]);
	input->keycodemax = ARRAY_SIZE(kbd->keymap);

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_REP, input->evbit);
	input_set_capability(input, EV_MSC, MSC_SCAN);
	for (i = 0; i < ARRAY_SIZE(kbd->keymap); i++)
		__set_bit(kbd->keymap[i], input->keybit);
	__set_bit(KEY_ESC, input->keybit);
	__clear_bit(KEY_RESERVED, input->keybit);

	serio_set_drvdata(serio, kbd);

	err = serio_open(serio, drv);
	if (err)
		goto err_clear_drvdata;

	err = input_register_device(input);
	if (err)
		goto err_close;

	return 0;

err_close:
	serio_close(serio);
err_clear_drvdata:
	serio_set_drvdata(serio, NULL);
err_free:
	input_free_device(input);
	kfree(kbd);
	return err;
}

static void ipaq_kbd_disconnect(struct serio *serio)
{
	struct ipaq_kbd *kbd = serio_get_drvdata(serio);

	serio_close(serio);

	scoped_guard(spinlock_irqsave, &kbd->lock)
		ipaq_kbd_release_all(kbd);

	serio_set_drvdata(serio, NULL);
	input_unregister_device(kbd->input);
	kfree(kbd);
}

static const struct serio_device_id ipaq_kbd_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_IPAQ_KBD,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ }
};
MODULE_DEVICE_TABLE(serio, ipaq_kbd_serio_ids);

static struct serio_driver ipaq_kbd_drv = {
	.driver		= {
		.name	= "ipaq-serial-kbd",
	},
	.description	= DRIVER_DESC,
	.id_table	= ipaq_kbd_serio_ids,
	.interrupt	= ipaq_kbd_interrupt,
	.connect	= ipaq_kbd_connect,
	.disconnect	= ipaq_kbd_disconnect,
};
module_serio_driver(ipaq_kbd_drv);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_KEYBOARD_IPAQ_SERIAL_KUNIT_TEST)
#include "ipaq-serial-kbd-test.c"
#endif
