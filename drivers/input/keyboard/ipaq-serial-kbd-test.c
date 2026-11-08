// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the HP iPAQ serial keyboard decoder
 */

#include <kunit/test.h>

static enum ipaq_kbd_event_type feed(struct ipaq_kbd_decoder *dec,
				     const u8 *bytes, size_t len,
				     struct ipaq_kbd_event *ev)
{
	enum ipaq_kbd_event_type type = IPAQ_KBD_EV_NONE;
	size_t i;

	for (i = 0; i < len; i++)
		type = ipaq_kbd_decode(dec, bytes[i], ev);

	return type;
}

static void ipaq_kbd_test_valid_pairs(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;
	static const u8 codes[] = { 0x15, 0x1c, 0x5a, 0x02, 0x66, 0x00, 0x7f };
	size_t i;

	ipaq_kbd_decoder_reset(&dec);
	for (i = 0; i < ARRAY_SIZE(codes); i++) {
		KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, codes[i], &ev),
				IPAQ_KBD_EV_NONE);
		KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, ~codes[i], &ev),
				IPAQ_KBD_EV_KEY);
		KUNIT_EXPECT_EQ(test, ev.code, codes[i]);
		KUNIT_EXPECT_FALSE(test, ev.release);
	}
}

static void ipaq_kbd_test_release(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;
	static const u8 press_release[] = { 0x15, 0xea, 0x95, 0xea };

	ipaq_kbd_decoder_reset(&dec);
	KUNIT_EXPECT_EQ(test, feed(&dec, press_release, 2, &ev), IPAQ_KBD_EV_KEY);
	KUNIT_EXPECT_FALSE(test, ev.release);
	KUNIT_EXPECT_EQ(test, feed(&dec, press_release + 2, 2, &ev),
			IPAQ_KBD_EV_KEY);
	KUNIT_EXPECT_EQ(test, ev.code, 0x15);
	KUNIT_EXPECT_TRUE(test, ev.release);
}

static void ipaq_kbd_test_resync(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;

	ipaq_kbd_decoder_reset(&dec);

	/* bad complement, then the next pair must still decode */
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0x15, &ev), IPAQ_KBD_EV_NONE);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0xeb, &ev), IPAQ_KBD_EV_RESYNC);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0x1d, &ev), IPAQ_KBD_EV_RESYNC);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0xe2, &ev), IPAQ_KBD_EV_KEY);
	KUNIT_EXPECT_EQ(test, ev.code, 0x1d);

	/* a lost first byte: the stray complement must not swallow the pair */
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0xea, &ev), IPAQ_KBD_EV_NONE);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0x1c, &ev), IPAQ_KBD_EV_RESYNC);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0xe3, &ev), IPAQ_KBD_EV_KEY);
	KUNIT_EXPECT_EQ(test, ev.code, 0x1c);
	KUNIT_EXPECT_FALSE(test, ev.release);
}

static void ipaq_kbd_test_special_frames(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;
	static const u8 micro[] = { 0x64, 0x9b };
	static const u8 foldable[] = { 0x38, 0xc7 };
	static const u8 restart[] = { 0x75, 0x8a };

	ipaq_kbd_decoder_reset(&dec);
	KUNIT_EXPECT_EQ(test, feed(&dec, foldable, 2, &ev), IPAQ_KBD_EV_IDENTITY);
	KUNIT_EXPECT_EQ(test, ev.variant, IPAQ_KBD_FOLDABLE);
	KUNIT_EXPECT_EQ(test, feed(&dec, micro, 2, &ev), IPAQ_KBD_EV_IDENTITY);
	KUNIT_EXPECT_EQ(test, ev.variant, IPAQ_KBD_MICRO);
	KUNIT_EXPECT_EQ(test, feed(&dec, restart, 2, &ev), IPAQ_KBD_EV_RESTART);
}

static void ipaq_kbd_test_partial_pair(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;
	static const u8 bytes[] = { 0x15, 0xea, 0x1c };

	ipaq_kbd_decoder_reset(&dec);
	KUNIT_EXPECT_EQ(test, feed(&dec, bytes, 3, &ev), IPAQ_KBD_EV_NONE);
	KUNIT_EXPECT_TRUE(test, dec.have_first);
	KUNIT_EXPECT_EQ(test, dec.first, 0x1c);

	ipaq_kbd_decoder_reset(&dec);
	KUNIT_EXPECT_FALSE(test, dec.have_first);
	KUNIT_EXPECT_EQ(test, ipaq_kbd_decode(&dec, 0xe3, &ev), IPAQ_KBD_EV_NONE);
}

static u32 ipaq_kbd_test_lcg(u32 *state)
{
	*state = *state * 1664525 + 1013904223;
	return *state >> 24;
}

static void ipaq_kbd_test_garbage_flood(struct kunit *test)
{
	struct ipaq_kbd_decoder dec;
	struct ipaq_kbd_event ev;
	u32 seed = 0x1badb002;
	unsigned int i, round;

	ipaq_kbd_decoder_reset(&dec);
	for (round = 0; round < 64; round++) {
		for (i = 0; i < 4096; i++)
			ipaq_kbd_decode(&dec, ipaq_kbd_test_lcg(&seed), &ev);

		/*
		 * A press byte has bit 7 clear and so can never complete a
		 * pair, whatever the flood left behind.
		 */
		ipaq_kbd_decode(&dec, 0x1c, &ev);
		KUNIT_ASSERT_EQ(test, ipaq_kbd_decode(&dec, 0xe3, &ev),
				IPAQ_KBD_EV_KEY);
		KUNIT_EXPECT_EQ(test, ev.code, 0x1c);
		KUNIT_EXPECT_FALSE(test, ev.release);
	}
}

static void ipaq_kbd_test_free_input(void *input)
{
	input_free_device(input);
}

static struct ipaq_kbd *ipaq_kbd_test_alloc(struct kunit *test)
{
	struct input_dev *input;
	struct ipaq_kbd *kbd;
	unsigned int i;

	kbd = kunit_kzalloc(test, sizeof(*kbd), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, kbd);
	input = input_allocate_device();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, input);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
							ipaq_kbd_test_free_input,
							input), 0);

	kbd->input = input;
	kbd->variant = IPAQ_KBD_MICRO;
	spin_lock_init(&kbd->lock);
	ipaq_kbd_decoder_reset(&kbd->dec);
	ipaq_kbd_init_keymap(kbd->keymap);

	__set_bit(EV_KEY, input->evbit);
	input_set_capability(input, EV_MSC, MSC_SCAN);
	for (i = 0; i < ARRAY_SIZE(kbd->keymap); i++)
		__set_bit(kbd->keymap[i], input->keybit);
	__set_bit(KEY_ESC, input->keybit);
	__clear_bit(KEY_RESERVED, input->keybit);

	return kbd;
}

static void ipaq_kbd_test_feed_kbd(struct ipaq_kbd *kbd, const u8 *bytes,
				   size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		ipaq_kbd_process_byte(kbd, bytes[i], 0);
}

static void ipaq_kbd_test_identity_clears_keys(struct kunit *test)
{
	struct ipaq_kbd *kbd = ipaq_kbd_test_alloc(test);
	static const u8 press_a_shift[] = { 0x58, 0xa7, 0x12, 0xed };
	static const u8 foldable[] = { 0x38, 0xc7 };
	static const u8 press_1[] = { 0x16, 0xe9 };

	ipaq_kbd_test_feed_kbd(kbd, press_a_shift, sizeof(press_a_shift));
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_A, kbd->input->key));
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_LEFTSHIFT, kbd->input->key));

	ipaq_kbd_test_feed_kbd(kbd, foldable, sizeof(foldable));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_A, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_LEFTSHIFT, kbd->input->key));
	KUNIT_EXPECT_EQ(test, kbd->variant, IPAQ_KBD_FOLDABLE);

	/* 0x16 is the 1 key on the Foldable and nothing on the Micro */
	ipaq_kbd_test_feed_kbd(kbd, press_1, sizeof(press_1));
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_1, kbd->input->key));
}

static void ipaq_kbd_test_fn_is_a_plain_key(struct kunit *test)
{
	struct ipaq_kbd *kbd = ipaq_kbd_test_alloc(test);
	static const u8 fn_down[] = { 0x02, 0xfd };
	static const u8 fn_up[] = { 0x82, 0xfd };
	static const u8 q_down[] = { 0x35, 0xca };
	static const u8 q_up[] = { 0xb5, 0xca };

	/* Fn+Q is Fn and Q; the layer is the keymap's business */
	ipaq_kbd_test_feed_kbd(kbd, fn_down, 2);
	ipaq_kbd_test_feed_kbd(kbd, q_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(IPAQ_KBD_KEY_FN_ORANGE, kbd->input->key));
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_Q, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_1, kbd->input->key));

	ipaq_kbd_test_feed_kbd(kbd, fn_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(IPAQ_KBD_KEY_FN_ORANGE, kbd->input->key));
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_Q, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, q_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_Q, kbd->input->key));

	/* the two Fn key codes are distinct and visible to X11 */
	KUNIT_EXPECT_NE(test, IPAQ_KBD_KEY_FN_ORANGE, IPAQ_KBD_KEY_FN_BLUE);
	KUNIT_EXPECT_LT(test, IPAQ_KBD_KEY_FN_ORANGE, 248);
	KUNIT_EXPECT_LT(test, IPAQ_KBD_KEY_FN_BLUE, 248);
}

static void ipaq_kbd_test_fn_ctrl_is_esc(struct kunit *test)
{
	struct ipaq_kbd *kbd = ipaq_kbd_test_alloc(test);
	static const u8 fn_down[] = { 0x02, 0xfd };
	static const u8 fn_up[] = { 0x82, 0xfd };
	static const u8 ctrl_down[] = { 0x45, 0xba };
	static const u8 ctrl_up[] = { 0xc5, 0xba };

	/* Ctrl alone is Ctrl */
	ipaq_kbd_test_feed_kbd(kbd, ctrl_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_LEFTCTRL, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_ESC, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, ctrl_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_LEFTCTRL, kbd->input->key));

	/* orange Fn held: the same key is Esc, and its release matches */
	ipaq_kbd_test_feed_kbd(kbd, fn_down, 2);
	ipaq_kbd_test_feed_kbd(kbd, ctrl_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_ESC, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_LEFTCTRL, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, fn_up, 2);
	ipaq_kbd_test_feed_kbd(kbd, ctrl_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_ESC, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_LEFTCTRL, kbd->input->key));

	/* the other order: Ctrl is let go first, Fn after */
	ipaq_kbd_test_feed_kbd(kbd, fn_down, 2);
	ipaq_kbd_test_feed_kbd(kbd, ctrl_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_ESC, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, ctrl_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_ESC, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, fn_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_LEFTCTRL, kbd->input->key));
}

static void ipaq_kbd_test_unknown_codes(struct kunit *test)
{
	struct ipaq_kbd *kbd = ipaq_kbd_test_alloc(test);
	static const u8 x01_down[] = { 0x01, 0xfe };
	static const u8 x01_up[] = { 0x81, 0xfe };
	static const u8 x03_down[] = { 0x03, 0xfc };
	static const u8 x03_up[] = { 0x83, 0xfc };
	unsigned int c;

	for (c = 0; c < IPAQ_KBD_KEYMAP_SIZE; c++) {
		if (ipaq_kbd_is_frame_code(c & IPAQ_KBD_CODE_MASK))
			KUNIT_EXPECT_EQ(test, kbd->keymap[c], KEY_RESERVED);
		else
			KUNIT_EXPECT_NE(test, kbd->keymap[c], KEY_RESERVED);
	}

	/* two unknown codes share KEY_UNKNOWN: up only when both are up */
	ipaq_kbd_test_feed_kbd(kbd, x01_down, 2);
	ipaq_kbd_test_feed_kbd(kbd, x03_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_UNKNOWN, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, x01_up, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_UNKNOWN, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, x03_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_UNKNOWN, kbd->input->key));

	/* a code moved to the blue Fn key code at run time */
	kbd->keymap[0x01] = IPAQ_KBD_KEY_FN_BLUE;
	__set_bit(IPAQ_KBD_KEY_FN_BLUE, kbd->input->keybit);
	ipaq_kbd_test_feed_kbd(kbd, x01_down, 2);
	KUNIT_EXPECT_TRUE(test, test_bit(IPAQ_KBD_KEY_FN_BLUE, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_UNKNOWN, kbd->input->key));
	ipaq_kbd_test_feed_kbd(kbd, x01_up, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(IPAQ_KBD_KEY_FN_BLUE, kbd->input->key));
}

static void ipaq_kbd_test_errors_release_keys(struct kunit *test)
{
	struct ipaq_kbd *kbd = ipaq_kbd_test_alloc(test);
	static const u8 press_a[] = { 0x58, 0xa7 };
	static const u8 restart[] = { 0x75, 0x8a };
	unsigned int i;

	ipaq_kbd_test_feed_kbd(kbd, press_a, 2);
	ipaq_kbd_process_byte(kbd, 0x1c, SERIO_FRAME);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_A, kbd->input->key));
	KUNIT_EXPECT_FALSE(test, kbd->dec.have_first);

	ipaq_kbd_test_feed_kbd(kbd, press_a, 2);
	ipaq_kbd_test_feed_kbd(kbd, restart, 2);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_A, kbd->input->key));
	KUNIT_EXPECT_EQ(test, kbd->variant, IPAQ_KBD_MICRO);

	ipaq_kbd_test_feed_kbd(kbd, press_a, 2);
	ipaq_kbd_process_byte(kbd, 0x1c, 0);
	ipaq_kbd_process_byte(kbd, 0x00, 0);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_A, kbd->input->key));

	for (i = 0; i < IPAQ_KBD_NUM_CODES; i++)
		KUNIT_EXPECT_EQ(test, kbd->down_key[i], KEY_RESERVED);
}

static struct kunit_case ipaq_kbd_test_cases[] = {
	KUNIT_CASE(ipaq_kbd_test_valid_pairs),
	KUNIT_CASE(ipaq_kbd_test_release),
	KUNIT_CASE(ipaq_kbd_test_resync),
	KUNIT_CASE(ipaq_kbd_test_special_frames),
	KUNIT_CASE(ipaq_kbd_test_partial_pair),
	KUNIT_CASE(ipaq_kbd_test_garbage_flood),
	KUNIT_CASE(ipaq_kbd_test_identity_clears_keys),
	KUNIT_CASE(ipaq_kbd_test_fn_is_a_plain_key),
	KUNIT_CASE(ipaq_kbd_test_fn_ctrl_is_esc),
	KUNIT_CASE(ipaq_kbd_test_unknown_codes),
	KUNIT_CASE(ipaq_kbd_test_errors_release_keys),
	{ }
};

static struct kunit_suite ipaq_kbd_test_suite = {
	.name = "ipaq-serial-kbd",
	.test_cases = ipaq_kbd_test_cases,
};

kunit_test_suite(ipaq_kbd_test_suite);
