// SPDX-License-Identifier: GPL-2.0-only
/*
 * iPAQ H3xxx/h5xxx expansion-pack ("sleeve") bus core
 *
 * Based on the Compaq driver of the same name for the 2.4/2.6 handheld
 * kernels (Copyright 2000 Compaq Computer Corporation, Author: Jamey
 * Hicks). The API shape is kept: the ipaq_sleeve_ops backend vtable, the
 * id_table match rules, the ipaq_sleeve_register() and presence-interrupt
 * driven enumeration from a work item, the ID read from EEPROM offsets 6
 * and 8 and the 250 ms settle after powering the sleeve. Differences:
 *
 *  - A real struct bus_type (ipaq_sleeve_bus_type) with a match callback,
 *    in the same shape as spi_bus_type: the bus probe and remove bridge to
 *    ipaq_sleeve_driver::probe() and remove(), which take a properly typed
 *    struct ipaq_sleeve_device.
 *
 *  - Only one sleeve can be active, as the hardware has one connector. It
 *    is a NULL-when-empty active_sleeve pointer to a heap-allocated
 *    struct ipaq_sleeve_device freed by its release() callback, which the
 *    driver core requires.
 *
 *  - No /proc/bus/sleeve files and no private debug macro; dynamic debug
 *    covers the latter. Each sleeve device carries read-only sysfs
 *    attributes for its identity and every decoded field of its ID EEPROM
 *    record, and the bus has one knob, identity_override, see
 *    Documentation/ABI/testing/sysfs-bus-ipaq-sleeve.
 *
 *  - The ID EEPROM is read past the vendor and device IDs: the description,
 *    the fixed fields and the 0xbb section are parsed into struct
 *    ipaq_sleeve_record by ipaq_sleeve_parse_record(), following the layout
 *    of the vendor's option-pack manager. The original read only offsets 6
 *    and 8, and never pulsed OPT_RESET; the reset width of a v2 or v3
 *    record now does, see ipaq_sleeve_reset_pulse().
 *
 *  - msleep() instead of mdelay() for the 250 ms EEPROM settle, as it runs
 *    from a work item, and cancel_delayed_work_sync() on this driver's own
 *    work item instead of flushing the system workqueue. The 100 ms
 *    option-detect debounce is the delayed work's own restartable timer, so
 *    a bounce burst pushes the sample out instead of queueing a second,
 *    back-to-back run.
 *
 *  - ipaq_sleeve_eject() does not call the driver's remove() a second time
 *    by hand after device_unregister() has already run it through the bus.
 *
 *  - A driver with a NULL id_table matches every sleeve, as the
 *    documentation of that field always said, instead of crashing in the
 *    match function.
 *
 *  - A slot-consumer notifier chain (IPAQ_SLEEVE_EVENT_ADD and _REMOVE,
 *    ipaq_sleeve_register_notifier()) and a feature table
 *    (ipaq_sleeve_features()) are new. The bus core also asserts and
 *    clears IPAQ_EGPIO_OPT_ON, the main power of the jacket, around
 *    insert and eject. The original had each sleeve driver assert it from
 *    its own probe(), which only works while every sleeve type has exactly
 *    one driver doing everything. A sleeve can carry an ipaq_sleeve_driver
 *    unrelated to its CF or PC Card slot, for example a Navman GPS and CF
 *    sleeve where the GPS driver binds and the CF socket is a sibling
 *    consumer of the same sleeve, so slot instantiation is a bus level
 *    event keyed on ipaq_sleeve_features() and not on which driver bound.
 */

#include <linux/array_size.h>
#include <linux/cleanup.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/ipaq-sleeve.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/string_choices.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* Set once by ipaq_sleeve_register(), cleared by ipaq_sleeve_unregister().
 * @sleeve_ops/@sleeve_dev are only ever mutated under
 * ipaq_sleeve_ops_lock; they are read without it from the work item and
 * from the ipaq_sleeve_write_egpio()/read_egpio()/egpio_irq() helpers,
 * which is safe because a backend is expected to unregister only after
 * quiescing its own presence IRQ and calling cancel_delayed_work_sync()
 * (done for it, below, inside ipaq_sleeve_unregister()) -- the ops lock
 * exists to make double-registration safe, not to serialise every access. Both
 * registration and unregistration also hold ipaq_sleeve_lock, because the
 * identity_override store path re-enumerates under that lock alone and is
 * not covered by the cancel_delayed_work_sync().
 */
static DEFINE_MUTEX(ipaq_sleeve_ops_lock);
static struct ipaq_sleeve_ops *sleeve_ops;
static void *sleeve_dev;

static struct ipaq_sleeve_device *active_sleeve;

/*
 * Slot-consumer notifier chain, see the IPAQ_SLEEVE_EVENT_* comment in
 * include/linux/ipaq-sleeve.h. ipaq_sleeve_lock guards both active_sleeve
 * and the register-time replay, so a consumer that registers concurrently
 * with an insert or eject sees exactly one of "replayed the sleeve that was
 * already there" or "will be notified by the in-flight insert once it
 * finishes", never both and never neither.
 */
static BLOCKING_NOTIFIER_HEAD(ipaq_sleeve_notifier_list);
static DEFINE_MUTEX(ipaq_sleeve_lock);

/*
 * Operator-forced identity (identity_override, below). A zero vendor means
 * that no override is in effect: SLEEVE_NO_VENDOR is never a forceable
 * value, since it is exactly the ID that ipaq_sleeve_insert() refuses to
 * treat as an identity. Both are read and written only under
 * ipaq_sleeve_lock, which ipaq_sleeve_insert() also runs under, so an
 * insertion never sees a half-updated pair.
 */
static unsigned short ipaq_sleeve_forced_vendor;
static unsigned short ipaq_sleeve_forced_device;

/*
 * Every sleeve type the bus knows, keyed by the vendor:device pair its ID
 * EEPROM carries. It is the single authority for how many CF or PC Card
 * slots a sleeve has and what else it carries, and for the names that an
 * operator may force through identity_override. It is keyed on the IDs read
 * from the sleeve, so a sleeve with no bound driver at all, for example an
 * unrecognised jacket or one whose only driver is unrelated to CF/PCMCIA,
 * still gets an accurate slot count.
 *
 * The names must be unique, must contain neither ':' nor whitespace, and
 * must never be "none": ipaq_sleeve_parse_identity() relies on that to tell
 * a name from a raw VVVV:DDDD pair and from the token that clears the
 * override.
 *
 * The GPRS pack and the Symbol WLAN scanner declare no CF or PCMCIA bit, so
 * they get no socket. An all-ones ID is a blank ID EEPROM and would get one
 * PC Card socket, but ipaq_sleeve_insert() rejects it on the byte 0 header
 * check, so that row is reachable only through identity_override. An
 * all-zeros ID is deliberately not in the table, see ipaq_sleeve_insert()
 * for why that case has to be loud. The Mercury BackPAQ has no row, because
 * its features are not known; it can still be forced by its raw ID.
 */
struct ipaq_sleeve_type {
	const char *name;
	unsigned short vendor, device;
	unsigned int features;
};

static const struct ipaq_sleeve_type ipaq_sleeve_types[] = {
	{ "cf", COMPAQ_VENDOR_ID, SINGLE_COMPACTFLASH_SLEEVE,
	  SLEEVE_HAS_CF(1) },
	{ "cf_plus", COMPAQ_VENDOR_ID, SINGLE_CF_PLUS_SLEEVE,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT },
	{ "pcmcia", COMPAQ_VENDOR_ID, SINGLE_PCMCIA_SLEEVE,
	  SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_FIXED_BATTERY },
	{ "pcmcia_plus", COMPAQ_VENDOR_ID, SINGLE_PCMCIA_PLUS_SLEEVE,
	  SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT },
	{ "dual_pcmcia", COMPAQ_VENDOR_ID, DUAL_PCMCIA_SLEEVE,
	  SLEEVE_HAS_PCMCIA(2) | SLEEVE_HAS_FIXED_BATTERY },
	{ "bluetooth_cf", COMPAQ_VENDOR_ID, BLUETOOTH_EXPANSION_PACK,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_BLUETOOTH },
	{ "gprs", COMPAQ_VENDOR_ID, GPRS_EXPANSION_PACK,
	  SLEEVE_HAS_FIXED_BATTERY | SLEEVE_HAS_GPRS },
	{ "nexian_dual_cf", NEXIAN_VENDOR, NEXIAN_DUAL_CF_SLEEVE,
	  SLEEVE_HAS_CF(2) | SLEEVE_HAS_REMOVABLE_BATTERY },
	{ "nexian_camera_cf", NEXIAN_VENDOR, NEXIAN_CAMERA_CF_SLEEVE,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_CAMERA },
	{ "symbol_wlan_scanner", SYMBOL_VENDOR, SYMBOL_WLAN_SCANNER_SLEEVE,
	  SLEEVE_HAS_BARCODE_READER | SLEEVE_HAS_802_11B | SLEEVE_HAS_FIXED_BATTERY },
	{ "pitech_memplug", PITECH_VENDOR, MEMPLUG_SLEEVE,
	  SLEEVE_HAS_CF(2) },
	{ "pitech_memplug3", PITECH_VENDOR, MEMPLUG_SLEEVE3,
	  SLEEVE_HAS_CF(2) },
	{ "navman_gps_cf", TALON_VENDOR, NAVMAN_GPS_SLEEVE,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS },
	{ "navman_gps_cf_alt", TALON_VENDOR, NAVMAN_GPS_SLEEVE_ALT,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS },
	{ "pcmcia_blank_eeprom", 0xffff, 0xffff,
	  SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_FIXED_BATTERY },
};

static const struct ipaq_sleeve_type *
ipaq_sleeve_type_by_id(unsigned short vendor, unsigned short device)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipaq_sleeve_types); i++) {
		const struct ipaq_sleeve_type *t = &ipaq_sleeve_types[i];

		if (t->vendor == vendor && t->device == device)
			return t;
	}

	return NULL;
}

/* @buf is raw sysfs input, so match with sysfs_streq() to ignore a trailing
 * newline.
 */
static const struct ipaq_sleeve_type *ipaq_sleeve_type_by_name(const char *buf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipaq_sleeve_types); i++) {
		if (sysfs_streq(buf, ipaq_sleeve_types[i].name))
			return &ipaq_sleeve_types[i];
	}

	return NULL;
}

unsigned int ipaq_sleeve_features(const struct ipaq_sleeve_device *sdev)
{
	const struct ipaq_sleeve_type *t;

	if (!sdev->id_valid)
		return 0;

	t = ipaq_sleeve_type_by_id(sdev->vendor, sdev->device);

	return t ? t->features : 0;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_features);

/*
 * ipaq_sleeve_register_notifier: subscribe @nb to IPAQ_SLEEVE_EVENT_*.
 * Replays the current sleeve, if any, as an immediate ADD callback
 * before returning, so a consumer registering after the real
 * enumeration already ran (the common case: this runs from a work
 * item, a consumer's own late_initcall has no guaranteed ordering
 * against it) still sees it.
 */
int ipaq_sleeve_register_notifier(struct notifier_block *nb)
{
	int ret;

	guard(mutex)(&ipaq_sleeve_lock);

	ret = blocking_notifier_chain_register(&ipaq_sleeve_notifier_list, nb);
	if (ret)
		return ret;

	if (active_sleeve)
		nb->notifier_call(nb, IPAQ_SLEEVE_EVENT_ADD, active_sleeve);

	return 0;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_register_notifier);

int ipaq_sleeve_unregister_notifier(struct notifier_block *nb)
{
	guard(mutex)(&ipaq_sleeve_lock);

	return blocking_notifier_chain_unregister(&ipaq_sleeve_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_unregister_notifier);

void ipaq_sleeve_write_egpio(int nr, int state)
{
	if (WARN_ON(!sleeve_ops))
		return;
	sleeve_ops->control_egpio(sleeve_dev, nr, state);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_write_egpio);

int ipaq_sleeve_read_egpio(int nr)
{
	if (WARN_ON(!sleeve_ops))
		return -ENODEV;
	return sleeve_ops->read_egpio(sleeve_dev, nr);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_read_egpio);

/*
 * Optional: a backend without an edge-reconfigurable card-detect source
 * does not provide the callback, and doing nothing is correct for it.
 */
int ipaq_sleeve_rearm_cd_edge(int nr, bool card_present)
{
	if (WARN_ON(!sleeve_ops))
		return -ENODEV;
	if (!sleeve_ops->rearm_cd_edge)
		return 0;
	return sleeve_ops->rearm_cd_edge(sleeve_dev, nr, card_present);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_rearm_cd_edge);

int ipaq_sleeve_egpio_irq(int nr)
{
	if (WARN_ON(!sleeve_ops))
		return -ENODEV;
	return sleeve_ops->egpio_irq(sleeve_dev, nr);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_egpio_irq);

/*
 * ipaq_sleeve_match_device: search a concrete id_table for an entry
 * matching @dev's vendor/device, honouring the SLEEVE_ANY_ID wildcard.
 * @ids must be non-NULL (see ipaq_sleeve_bus_match(), which owns the
 * "NULL id_table matches everything" policy). Sentinel-terminated by a
 * {0,} entry (vendor == 0).
 */
static const struct ipaq_sleeve_device_id *
ipaq_sleeve_match_device(const struct ipaq_sleeve_device_id *ids,
			 const struct ipaq_sleeve_device *dev)
{
	for (; ids->vendor; ids++) {
		if ((ids->vendor == SLEEVE_ANY_ID || ids->vendor == dev->vendor) &&
		    (ids->device == SLEEVE_ANY_ID || ids->device == dev->device))
			return ids;
	}
	return NULL;
}

static void ipaq_sleeve_device_release(struct device *dev)
{
	kfree(to_ipaq_sleeve_device(dev));
}

/*
 * ID EEPROM record layout past the 10-byte identity. The description
 * starts at byte 10 and ends in a NUL at offset n; everything after it
 * is at a fixed offset from n + 1, packed, little-endian. v1 records
 * carry no socket types and no OPT_ON byte, so their section marker,
 * 0xBB byte and 0xBB section sit five bytes earlier than in v2/v3.
 */
#define IPAQ_SLEEVE_EEPROM_SIZE		512	/* 25LC040 */
#define IPAQ_SLEEVE_DESC_OFFSET		10
#define IPAQ_SLEEVE_READ_CHUNK		32

#define IPAQ_SLEEVE_TAIL_TYPE		0
#define IPAQ_SLEEVE_TAIL_INT_ENABLE	1
#define IPAQ_SLEEVE_TAIL_EXT_BATTERY	2
#define IPAQ_SLEEVE_TAIL_INIT_POWER	3
#define IPAQ_SLEEVE_TAIL_SUSP_POWER	4
#define IPAQ_SLEEVE_TAIL_RESET_WIDTH	5
#define IPAQ_SLEEVE_TAIL_STABILIZE	7
#define IPAQ_SLEEVE_TAIL_UNKNOWN_1A	9
#define IPAQ_SLEEVE_TAIL_UNKNOWN_1C	11
#define IPAQ_SLEEVE_TAIL_SOCKET0	13	/* v2/v3 */
#define IPAQ_SLEEVE_TAIL_SOCKET1	15	/* v2/v3 */
#define IPAQ_SLEEVE_TAIL_OPT_ON		17	/* v2/v3 */
#define IPAQ_SLEEVE_TAIL_MARKER_V1	13
#define IPAQ_SLEEVE_TAIL_MARKER_V2	18

#define IPAQ_SLEEVE_MARKER		0x0f0f0f0f
#define IPAQ_SLEEVE_BB			0xbb
/* 0xBB byte, then four u32 words */
#define IPAQ_SLEEVE_BB_SECTION_LEN	(1 + 4 * 4)

#define IPAQ_SLEEVE_TAIL_LEN_V2		(IPAQ_SLEEVE_TAIL_MARKER_V2 + 4 + \
					 IPAQ_SLEEVE_BB_SECTION_LEN)

/* Memory window size from which WinCE registers the pack as a disk. */
#define IPAQ_SLEEVE_MEMORY_MIN		1024

static_assert(IPAQ_SLEEVE_DESC_OFFSET + IPAQ_SLEEVE_DESC_MAX +
	      IPAQ_SLEEVE_TAIL_LEN_V2 <= IPAQ_SLEEVE_EEPROM_SIZE);

static int ipaq_sleeve_eeprom_read(u8 *raw, unsigned int off, unsigned int len)
{
	return sleeve_ops->spi_read(sleeve_dev, off, raw + off, len);
}

/*
 * ipaq_sleeve_parse_record: read and check the record that follows the
 * identity in the ID EEPROM. @raw is an IPAQ_SLEEVE_EEPROM_SIZE buffer
 * holding bytes 0-9 already, header checked. Returns NULL with @rec
 * complete, or a reason the record was not parsed.
 *
 * WinCE stops at the first marker or 0xBB mismatch and treats the pack
 * as unreadable; here that only leaves the record unparsed. WinCE also
 * copies the description into a 256-byte buffer without a bound, so the
 * bound and the sanitising below are this driver's own.
 */
static const char *ipaq_sleeve_parse_record(u8 *raw,
					    struct ipaq_sleeve_record *rec)
{
	unsigned int off, len, n, tail, marker_off;
	const u8 *nul = NULL, *t;
	int ret;

	rec->length = get_unaligned_le32(&raw[1]);
	rec->version = raw[5];
	if (rec->version < 1 || rec->version > 3)
		return "unknown record version";

	for (off = IPAQ_SLEEVE_DESC_OFFSET;
	     !nul && off < IPAQ_SLEEVE_DESC_OFFSET + IPAQ_SLEEVE_DESC_MAX;
	     off += len) {
		len = min_t(unsigned int, IPAQ_SLEEVE_READ_CHUNK,
			    IPAQ_SLEEVE_DESC_OFFSET + IPAQ_SLEEVE_DESC_MAX - off);
		ret = ipaq_sleeve_eeprom_read(raw, off, len);
		if (ret)
			return "description read failed";
		nul = memchr(&raw[off], 0, len);
	}
	if (!nul)
		return "description not terminated within 256 bytes";

	n = nul - raw;
	for (off = IPAQ_SLEEVE_DESC_OFFSET; off < n; off++)
		rec->description[off - IPAQ_SLEEVE_DESC_OFFSET] =
			isascii(raw[off]) && isprint(raw[off]) ? raw[off] : '?';
	rec->description[n - IPAQ_SLEEVE_DESC_OFFSET] = '\0';

	marker_off = rec->version == 1 ? IPAQ_SLEEVE_TAIL_MARKER_V1 :
					 IPAQ_SLEEVE_TAIL_MARKER_V2;
	tail = n + 1;
	len = marker_off + 4 + IPAQ_SLEEVE_BB_SECTION_LEN;
	ret = ipaq_sleeve_eeprom_read(raw, tail, len);
	if (ret)
		return "fixed fields read failed";

	t = &raw[tail];
	if (get_unaligned_le32(&t[marker_off]) != IPAQ_SLEEVE_MARKER)
		return "section marker mismatch";
	if (t[marker_off + 4] != IPAQ_SLEEVE_BB)
		return "0xBB byte mismatch";

	rec->type = t[IPAQ_SLEEVE_TAIL_TYPE];
	rec->interrupt_enable = t[IPAQ_SLEEVE_TAIL_INT_ENABLE];
	rec->extended_battery = t[IPAQ_SLEEVE_TAIL_EXT_BATTERY];
	rec->initial_power_state = t[IPAQ_SLEEVE_TAIL_INIT_POWER];
	rec->suspend_power_state = t[IPAQ_SLEEVE_TAIL_SUSP_POWER];
	rec->reset_width_ms = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_RESET_WIDTH]);
	rec->stabilize_ms = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_STABILIZE]);
	rec->unknown_1a = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_UNKNOWN_1A]);
	rec->unknown_1c = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_UNKNOWN_1C]);
	if (rec->version >= 2) {
		rec->socket_type[0] = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_SOCKET0]);
		rec->socket_type[1] = get_unaligned_le16(&t[IPAQ_SLEEVE_TAIL_SOCKET1]);
		rec->opt_on = t[IPAQ_SLEEVE_TAIL_OPT_ON];
	}
	for (off = 0; off < ARRAY_SIZE(rec->bb_word); off++)
		rec->bb_word[off] = get_unaligned_le32(&t[marker_off + 5 + 4 * off]);
	rec->end = tail + len;

	return NULL;
}

/* WinCE's own end - start + 1; a window that ends before it starts is none. */
static u64 ipaq_sleeve_record_window(const struct ipaq_sleeve_record *rec)
{
	u32 start = rec->bb_word[2], end = rec->bb_word[3];

	return end >= start ? (u64)end - start + 1 : 0;
}

/*
 * Only a meaning for bytes 1-4 is inferred, so the one check possible is
 * that a length reaches at least as far as the fields just parsed and no
 * further than the part holding them.
 */
static void ipaq_sleeve_check_record_length(const struct ipaq_sleeve_record *rec)
{
	if (rec->length < rec->end || rec->length > IPAQ_SLEEVE_EEPROM_SIZE)
		pr_warn("ipaq-sleeve: ID EEPROM bytes 1-4 read %u, not a record length covering the %u bytes parsed\n",
			rec->length, rec->end);
}

/*
 * The record's reset width and stabilisation wait are u16 millisecond
 * counts with no bound in the WinCE reader; the pulse runs under
 * ipaq_sleeve_lock, so a corrupt record must not stall enumeration for
 * a minute.
 */
#define IPAQ_SLEEVE_RESET_MAX_MS	1000

static unsigned int ipaq_sleeve_reset_clamp(const char *what, u16 ms)
{
	if (ms <= IPAQ_SLEEVE_RESET_MAX_MS)
		return ms;

	pr_warn("ipaq-sleeve: ID EEPROM %s of %u ms clamped to %u ms\n",
		what, ms, IPAQ_SLEEVE_RESET_MAX_MS);
	return IPAQ_SLEEVE_RESET_MAX_MS;
}

/*
 * ipaq_sleeve_reset_pulse: pulse OPT_RESET for the width a v2/v3 record
 * asks for, then wait the stabilisation time it names, as the WinCE
 * option-pack manager does. Only for a record that parsed in full and
 * describes the pack on the connector (ipaq_sleeve_usable_record()), so
 * a failed read or a forced identity never resets the jacket. v1 records
 * get no pulse, as on WinCE.
 *
 * Runs after OPT_ON, the order of the h38xx v1.20, h3950 and h5550
 * managers; the h3870's v2.15 manager pulses before it.
 *
 * Caller holds ipaq_sleeve_lock.
 */
static void ipaq_sleeve_reset_pulse(const struct ipaq_sleeve_device *sdev)
{
	const struct ipaq_sleeve_record *rec = ipaq_sleeve_usable_record(sdev);
	unsigned int width, stabilize;

	if (!rec || rec->version < 2 || !rec->reset_width_ms)
		return;

	width = ipaq_sleeve_reset_clamp("reset width", rec->reset_width_ms);
	stabilize = ipaq_sleeve_reset_clamp("stabilisation wait",
					    rec->stabilize_ms);

	pr_debug("ipaq-sleeve: OPT_RESET pulse %u ms, then %u ms\n",
		 width, stabilize);

	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_RESET, 1);
	msleep(width);
	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_RESET, 0);
	msleep(stabilize);
}

/*
 * ipaq_sleeve_eject: notify slot consumers, unregister the active
 * sleeve, if any, and power its jacket and NVRAM rails down.
 * device_unregister() runs the bound driver's remove() through the
 * bus's own dispatch (ipaq_sleeve_bus_remove()) as part of tearing the
 * device down -- see the file comment above for why that is not also
 * done by hand here.
 *
 * Order is the reverse of ipaq_sleeve_insert(): notify REMOVE first (a
 * slot consumer like the CF socket needs the sleeve device to still
 * exist and its driver, if any, to still be bound while it tears its
 * own platform_device down), then device_unregister(), then power off,
 * the jacket rail before the NVRAM and ID EEPROM rail.
 *
 * Caller holds ipaq_sleeve_lock.
 */
static void ipaq_sleeve_eject(void)
{
	struct ipaq_sleeve_device *sdev = active_sleeve;

	if (!sdev)
		return;

	active_sleeve = NULL;

	blocking_notifier_call_chain(&ipaq_sleeve_notifier_list,
				     IPAQ_SLEEVE_EVENT_REMOVE, sdev);

	device_unregister(&sdev->dev);

	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_ON, 0);
	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_NVRAM_ON, 0);
}

/*
 * ipaq_sleeve_insert: power the sleeve's NVRAM rail, wait for it to
 * settle, read its ID EEPROM identity and the record behind it, apply
 * the operator's identity_override if one is set, register the sleeve
 * on the bus so any matching ipaq_sleeve_driver can bind, then power the
 * jacket's main rail and notify slot consumers (e.g. the CF socket).
 * EEPROM failure with no override leaves the sleeve explicitly
 * unidentified: ID-matched drivers cannot bind, but a board-specific slot
 * consumer may still use physical detect lines. Runs from
 * ipaq_sleeve_task_handler(), i.e. process context, never from the
 * presence IRQ itself.
 *
 * Order matters and is deliberate: NVRAM rail -> settle -> EEPROM read
 * -> device_register() -> OPT_ON -> settle -> the record's OPT_RESET
 * pulse, if it asks for one -> notify. device_register()
 * before OPT_ON so a sleeve-specific driver (e.g. the Navman GPS UART)
 * finishes its own probe on an unpowered-but-identified sleeve first;
 * notify last so a slot consumer only ever sees a sleeve whose jacket
 * rail is already up and whose own driver, if any, has already bound.
 * Caller holds ipaq_sleeve_lock.
 */
static void ipaq_sleeve_insert(void)
{
	struct ipaq_sleeve_device *sdev;
	unsigned short eeprom_vendor = 0, eeprom_device = 0;
	unsigned short vendor, device;
	const char *record_err;
	bool id_valid = false;
	u8 *eeprom;
	int ret;

	/*
	 * No current caller gets here with active_sleeve set:
	 * ipaq_sleeve_reenumerate() ejects right before calling in, and
	 * ipaq_sleeve_task_handler() does not call in while a sleeve is
	 * registered. If a future caller does, eject before powering the rail
	 * below, not after, so there is exactly one rail-on and it always gets
	 * the full settle; ejecting after would power-cycle the rail and read
	 * the EEPROM unsettled. Kept as a defensive invariant check rather
	 * than deleted.
	 */
	if (active_sleeve) {
		pr_err("ipaq-sleeve: insert without a matching remove, ejecting stale sleeve\n");
		ipaq_sleeve_eject();
	}

	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_NVRAM_ON, 1);
	/*
	 * The ID EEPROM is read 250 ms after its NVRAM rail is powered. The
	 * vendor driver waits only a few milliseconds, so this is
	 * conservative.
	 */
	msleep(250);

	sdev = kzalloc_obj(*sdev, GFP_KERNEL);
	eeprom = kzalloc(IPAQ_SLEEVE_EEPROM_SIZE, GFP_KERNEL);
	if (!sdev || !eeprom) {
		kfree(eeprom);
		kfree(sdev);
		sleeve_ops->control_egpio(sleeve_dev,
					  IPAQ_EGPIO_OPT_NVRAM_ON, 0);
		return;
	}

	/*
	 * Read the header through the device ID (bytes 0 to 9) in one
	 * transaction. The vendor software rejects anything but 0xaa in byte
	 * 0 before it looks at the vendor ID (offset 6) or the device ID
	 * (offset 8), both 2 bytes. Without that check, a result of 0000:0000
	 * cannot be told apart between a dead or mis-clocked bus and a jacket
	 * without an ID EEPROM.
	 */
	ret = sleeve_ops->spi_read(sleeve_dev, 0, eeprom, IPAQ_SLEEVE_DESC_OFFSET);
	if (ret) {
		pr_err("ipaq-sleeve: ID EEPROM read failed: %d\n", ret);
		record_err = "identity read failed";
	} else if (eeprom[0] != 0xaa) {
		record_err = "header byte is not 0xaa";
		pr_err("ipaq-sleeve: ID EEPROM byte 0 is 0x%02x, expected 0xaa: bus dead or mis-clocked, or no ID EEPROM on this jacket\n",
		       eeprom[0]);
	} else {
		eeprom_vendor = eeprom[6] | (eeprom[7] << 8);
		eeprom_device = eeprom[8] | (eeprom[9] << 8);

		if (!eeprom_vendor || !eeprom_device)
			pr_err("ipaq-sleeve: ID EEPROM returned invalid jacket %04x:%04x\n",
			       eeprom_vendor, eeprom_device);
		else
			id_valid = true;

		/*
		 * The identity above stands whatever the rest of the record
		 * holds: a pack whose markers do not check out keeps the
		 * vendor:device it announced, where WinCE would reject it as
		 * unreadable.
		 */
		record_err = ipaq_sleeve_parse_record(eeprom, &sdev->record);
		sdev->record_valid = !record_err;
		if (record_err)
			memset(&sdev->record, 0, sizeof(sdev->record));
	}
	kfree(eeprom);

	if (sdev->record_valid) {
		const struct ipaq_sleeve_record *rec = &sdev->record;

		pr_debug("ipaq-sleeve: ID EEPROM record v%u \"%s\": type 0x%02x, interrupt enable 0x%02x, extended battery %s, reset %u ms + %u ms, OPT_ON %s, memory window %llu bytes, length %u for %u bytes parsed\n",
			 rec->version, rec->description, rec->type,
			 rec->interrupt_enable,
			 str_yes_no(ipaq_sleeve_record_ext_battery(rec)),
			 rec->reset_width_ms, rec->stabilize_ms,
			 rec->version == 1 ? "n/a (v1)" :
				 str_yes_no(rec->opt_on),
			 ipaq_sleeve_record_window(rec), rec->length,
			 rec->end);
		ipaq_sleeve_check_record_length(rec);
	} else {
		pr_debug("ipaq-sleeve: ID EEPROM record not parsed: %s\n",
			 record_err);
	}

	vendor = id_valid ? eeprom_vendor : SLEEVE_NO_VENDOR;
	device = id_valid ? eeprom_device : SLEEVE_NO_DEVICE;

	/*
	 * Applied over a good EEPROM as readily as over a dead one, so that
	 * one jacket can be made to present as another. The warning names
	 * both the forced identity and what the EEPROM returned.
	 */
	if (ipaq_sleeve_forced_vendor) {
		const struct ipaq_sleeve_type *t =
			ipaq_sleeve_type_by_id(ipaq_sleeve_forced_vendor,
					       ipaq_sleeve_forced_device);

		pr_warn("ipaq-sleeve: identity forced to %s (%04x:%04x) by identity_override; ID EEPROM %s %04x:%04x\n",
			t ? t->name : "unnamed type",
			ipaq_sleeve_forced_vendor, ipaq_sleeve_forced_device,
			ret ? "read failed, returned" : "read",
			eeprom_vendor, eeprom_device);

		vendor = ipaq_sleeve_forced_vendor;
		device = ipaq_sleeve_forced_device;
		id_valid = true;
		sdev->identity_forced = true;
	}

	sdev->vendor = vendor;
	sdev->device = device;
	sdev->id_valid = id_valid;
	sdev->dev.bus = &ipaq_sleeve_bus_type;
	sdev->dev.release = ipaq_sleeve_device_release;
	if (id_valid)
		dev_set_name(&sdev->dev, "%04x:%04x", vendor, device);
	else
		dev_set_name(&sdev->dev, "unidentified");

	if (device_register(&sdev->dev)) {
		pr_err("ipaq-sleeve: could not register sleeve device\n");
		put_device(&sdev->dev);
		sleeve_ops->control_egpio(sleeve_dev,
					  IPAQ_EGPIO_OPT_NVRAM_ON, 0);
		return;
	}

	active_sleeve = sdev;

	if (id_valid) {
		const struct ipaq_sleeve_type *t =
			ipaq_sleeve_type_by_id(vendor, device);

		pr_debug("ipaq-sleeve: jacket %04x:%04x (%s), features 0x%08x\n",
			 vendor, device, t ? t->name : "unknown type",
			 ipaq_sleeve_features(sdev));
	} else {
		pr_info("ipaq-sleeve: unidentified jacket present; write a type to /sys/bus/ipaq-sleeve/identity_override to force one\n");
	}

	/*
	 * Jacket main rail. It is switched on here, once, after
	 * device_register() so that a sleeve-specific driver, such as the
	 * Navman GPS UART, has already run its probe(). That is earlier than
	 * waiting for a slot consumer and also covers a jacket that no
	 * ipaq_sleeve_driver binds to, which the h39xx needs: its card detect
	 * line does not read card presence until this rail is up.
	 */
	sleeve_ops->control_egpio(sleeve_dev, IPAQ_EGPIO_OPT_ON, 1);
	/* Same settle time as for the NVRAM rail above. */
	msleep(250);

	ipaq_sleeve_reset_pulse(sdev);

	blocking_notifier_call_chain(&ipaq_sleeve_notifier_list,
				     IPAQ_SLEEVE_EVENT_ADD, sdev);
}

static int ipaq_sleeve_bus_match(struct device *dev, const struct device_driver *drv)
{
	struct ipaq_sleeve_device *sdev = to_ipaq_sleeve_device(dev);
	const struct ipaq_sleeve_driver *sdrv = to_ipaq_sleeve_driver(drv);

	if (!sdev->id_valid)
		return 0;

	if (!sdrv->id_table)
		return 1;

	return !!ipaq_sleeve_match_device(sdrv->id_table, sdev);
}

static int ipaq_sleeve_bus_probe(struct device *dev)
{
	const struct ipaq_sleeve_driver *sdrv = to_ipaq_sleeve_driver(dev->driver);
	struct ipaq_sleeve_device *sdev = to_ipaq_sleeve_device(dev);

	if (!sdrv->probe)
		return 0;
	return sdrv->probe(sdev);
}

static void ipaq_sleeve_bus_remove(struct device *dev)
{
	const struct ipaq_sleeve_driver *sdrv = to_ipaq_sleeve_driver(dev->driver);
	struct ipaq_sleeve_device *sdev = to_ipaq_sleeve_device(dev);

	if (sdrv->remove)
		sdrv->remove(sdev);
}

/*
 * The legacy bus suspend/resume callbacks run for every device on the bus
 * during a system suspend, not only ones with a driver bound, since the PM
 * core calls them ahead of looking at dev->driver. A sleeve that is
 * unidentified, or only a CF socket consumer, has no ipaq_sleeve_driver, so
 * dev->driver is NULL here in the common case.
 */
static int ipaq_sleeve_bus_suspend(struct device *dev, pm_message_t state)
{
	const struct ipaq_sleeve_driver *sdrv;

	if (!dev->driver)
		return 0;

	sdrv = to_ipaq_sleeve_driver(dev->driver);
	if (sdrv->suspend)
		return sdrv->suspend(to_ipaq_sleeve_device(dev), state);
	return 0;
}

static int ipaq_sleeve_bus_resume(struct device *dev)
{
	const struct ipaq_sleeve_driver *sdrv;

	if (!dev->driver)
		return 0;

	sdrv = to_ipaq_sleeve_driver(dev->driver);
	if (sdrv->resume)
		return sdrv->resume(to_ipaq_sleeve_device(dev));
	return 0;
}

/*
 * A software re-plug: tear down whatever is enumerated, then enumerate
 * whatever the detect line says is on the connector now. This is how a
 * changed identity_override reaches every consumer at once, because the bus
 * match, ipaq_sleeve_features(), dev_name() and the slot notifier all take
 * their identity from ipaq_sleeve_insert().
 *
 * Caller holds ipaq_sleeve_lock.
 */
static void ipaq_sleeve_reenumerate(void)
{
	int opt_det = 0;

	if (!sleeve_ops)
		return;

	ipaq_sleeve_eject();

	sleeve_ops->get_option_detect(sleeve_dev, &opt_det);
	if (opt_det)
		ipaq_sleeve_insert();
}

/*
 * Parse an identity_override write: "none", one of the names in
 * ipaq_sleeve_types[], or a raw "VVVV:DDDD" hexadecimal pair, which reaches
 * IDs that have no row yet. No name contains ':', so trying the names first
 * is unambiguous.
 *
 * Both halves must be non-zero: zero is the "no override" sentinel and the
 * value that ipaq_sleeve_insert() refuses to accept as an identity.
 */
static int ipaq_sleeve_parse_identity(const char *buf, unsigned short *vendor,
				      unsigned short *device)
{
	const struct ipaq_sleeve_type *t;
	unsigned short v, d;
	char trail;
	int n;

	if (sysfs_streq(buf, "none")) {
		*vendor = SLEEVE_NO_VENDOR;
		*device = SLEEVE_NO_DEVICE;
		return 0;
	}

	t = ipaq_sleeve_type_by_name(buf);
	if (t) {
		*vendor = t->vendor;
		*device = t->device;
		return 0;
	}

	n = sscanf(buf, "%4hx:%4hx%c", &v, &d, &trail);
	if (n < 2 || (n > 2 && trail != '\n'))
		return -EINVAL;
	if (!v || !d)
		return -EINVAL;

	*vendor = v;
	*device = d;

	return 0;
}

/* Shared by the sysfs attribute and the module parameter, which are two
 * ways to set the same state.
 */
static int ipaq_sleeve_apply_identity(const char *buf)
{
	unsigned short vendor, device;
	int ret;

	ret = ipaq_sleeve_parse_identity(buf, &vendor, &device);
	if (ret)
		return ret;

	guard(mutex)(&ipaq_sleeve_lock);

	ipaq_sleeve_forced_vendor = vendor;
	ipaq_sleeve_forced_device = device;
	ipaq_sleeve_reenumerate();

	return 0;
}

/*
 * Like led_trigger_format(), the bracket marks whatever currently governs
 * the device, not merely that an override was written. With no override
 * forced, that is the identity detected for active_sleeve (0:0, the same
 * as SLEEVE_NO_VENDOR/DEVICE, when a jacket is absent or present but
 * unidentified), so "none" is bracketed only when nothing is identified
 * at all.
 */
static ssize_t identity_override_show(const struct bus_type *bus, char *buf)
{
	const struct ipaq_sleeve_type *effective;
	unsigned short vendor, device;
	unsigned int i;
	int len = 0;

	scoped_guard(mutex, &ipaq_sleeve_lock) {
		if (ipaq_sleeve_forced_vendor) {
			vendor = ipaq_sleeve_forced_vendor;
			device = ipaq_sleeve_forced_device;
		} else if (active_sleeve) {
			vendor = active_sleeve->vendor;
			device = active_sleeve->device;
		} else {
			vendor = SLEEVE_NO_VENDOR;
			device = SLEEVE_NO_DEVICE;
		}
	}

	effective = vendor ? ipaq_sleeve_type_by_id(vendor, device) : NULL;

	len += sysfs_emit_at(buf, len, "%s", vendor ? "none" : "[none]");

	for (i = 0; i < ARRAY_SIZE(ipaq_sleeve_types); i++) {
		const struct ipaq_sleeve_type *t = &ipaq_sleeve_types[i];

		len += sysfs_emit_at(buf, len, " %s%s%s",
				     t == effective ? "[" : "", t->name,
				     t == effective ? "]" : "");
	}

	/*
	 * An effective identity with no row of its own, a forced raw pair or
	 * a jacket the table does not know, is shown as the bracketed pair.
	 */
	if (vendor && !effective)
		len += sysfs_emit_at(buf, len, " [%04x:%04x]", vendor, device);

	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t identity_override_store(const struct bus_type *bus,
				       const char *buf, size_t count)
{
	int ret = ipaq_sleeve_apply_identity(buf);

	return ret ? ret : count;
}
static BUS_ATTR_RW(identity_override);

static struct attribute *ipaq_sleeve_bus_attrs[] = {
	&bus_attr_identity_override.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ipaq_sleeve_bus);

/*
 * ipaq_sleeve.identity_override= seeds the same state from the kernel command
 * line, for a board whose console cannot be typed at. It is read-only in
 * sysfs, so that /sys/bus/ipaq-sleeve/identity_override stays the one
 * writable path to the variable; reading the module parameter returns the
 * value in effect now, not the boot string.
 *
 * On the command line this only stores the pair, because parameters are
 * parsed before any initcall, so no backend has registered yet and
 * ipaq_sleeve_reenumerate() returns immediately.
 */
static int ipaq_sleeve_identity_override_set(const char *val,
					     const struct kernel_param *kp)
{
	return ipaq_sleeve_apply_identity(val);
}

static int ipaq_sleeve_identity_override_get(char *buf,
					     const struct kernel_param *kp)
{
	const struct ipaq_sleeve_type *t;
	unsigned short vendor, device;

	scoped_guard(mutex, &ipaq_sleeve_lock) {
		vendor = ipaq_sleeve_forced_vendor;
		device = ipaq_sleeve_forced_device;
	}

	if (!vendor)
		return sysfs_emit(buf, "none\n");

	t = ipaq_sleeve_type_by_id(vendor, device);
	if (t)
		return sysfs_emit(buf, "%s\n", t->name);

	return sysfs_emit(buf, "%04x:%04x\n", vendor, device);
}

static const struct kernel_param_ops ipaq_sleeve_identity_override_ops = {
	.set = ipaq_sleeve_identity_override_set,
	.get = ipaq_sleeve_identity_override_get,
};
module_param_cb(identity_override, &ipaq_sleeve_identity_override_ops, NULL, 0444);
MODULE_PARM_DESC(identity_override,
		 "force the sleeve identity (type name or VVVV:DDDD); same values as /sys/bus/ipaq-sleeve/identity_override");

/*
 * Per-sleeve attributes. Everything they read is set before
 * device_register() and never changes afterwards (struct
 * ipaq_sleeve_device), so no lock is taken.
 */
static ssize_t id_eeprom_valid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", to_ipaq_sleeve_device(dev)->record_valid);
}
static DEVICE_ATTR_RO(id_eeprom_valid);

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "0x%04x\n", to_ipaq_sleeve_device(dev)->vendor);
}
static DEVICE_ATTR_RO(vendor);

static ssize_t device_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "0x%04x\n", to_ipaq_sleeve_device(dev)->device);
}
static DEVICE_ATTR_RO(device);

static ssize_t type_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	const struct ipaq_sleeve_device *sdev = to_ipaq_sleeve_device(dev);
	const struct ipaq_sleeve_type *t = NULL;

	if (sdev->id_valid)
		t = ipaq_sleeve_type_by_id(sdev->vendor, sdev->device);

	return sysfs_emit(buf, "%s\n", t ? t->name : "unknown");
}
static DEVICE_ATTR_RO(type);

#define IPAQ_SLEEVE_RECORD_ATTR(_name, _fmt, _val)			\
static ssize_t _name##_show(struct device *dev,				\
			    struct device_attribute *attr, char *buf)	\
{									\
	const struct ipaq_sleeve_record *rec =				\
		&to_ipaq_sleeve_device(dev)->record;			\
									\
	return sysfs_emit(buf, _fmt "\n", _val);			\
}									\
static DEVICE_ATTR_RO(_name)

IPAQ_SLEEVE_RECORD_ATTR(record_length_raw, "%u", rec->length);
IPAQ_SLEEVE_RECORD_ATTR(record_version, "%u", rec->version);
IPAQ_SLEEVE_RECORD_ATTR(record_format, "v%u", rec->version);
IPAQ_SLEEVE_RECORD_ATTR(description, "%s", rec->description);
IPAQ_SLEEVE_RECORD_ATTR(type_raw, "0x%02x", rec->type);
IPAQ_SLEEVE_RECORD_ATTR(interrupt_enable_raw, "0x%02x", rec->interrupt_enable);
IPAQ_SLEEVE_RECORD_ATTR(extended_battery, "%d",
			ipaq_sleeve_record_ext_battery(rec));
IPAQ_SLEEVE_RECORD_ATTR(initial_power_state_raw, "0x%02x",
			rec->initial_power_state);
IPAQ_SLEEVE_RECORD_ATTR(suspend_power_state_raw, "0x%02x",
			rec->suspend_power_state);
IPAQ_SLEEVE_RECORD_ATTR(reset_width_ms, "%u", rec->reset_width_ms);
IPAQ_SLEEVE_RECORD_ATTR(stabilize_ms, "%u", rec->stabilize_ms);
IPAQ_SLEEVE_RECORD_ATTR(unknown_1a_raw, "0x%04x", rec->unknown_1a);
IPAQ_SLEEVE_RECORD_ATTR(unknown_1c_raw, "0x%04x", rec->unknown_1c);
IPAQ_SLEEVE_RECORD_ATTR(opt_on_requested, "%d", !!rec->opt_on);
IPAQ_SLEEVE_RECORD_ATTR(memory_window_bytes, "%llu",
			ipaq_sleeve_record_window(rec));
IPAQ_SLEEVE_RECORD_ATTR(memory_capable, "%d",
			ipaq_sleeve_record_window(rec) >= IPAQ_SLEEVE_MEMORY_MIN);
IPAQ_SLEEVE_RECORD_ATTR(bb_word1_raw, "0x%08x", rec->bb_word[0]);
IPAQ_SLEEVE_RECORD_ATTR(bb_word2_raw, "0x%08x", rec->bb_word[1]);

/*
 * The bridge type the h38xx WinCE PC Card driver acts on: below 2 it
 * resets the socket directly, 2 means a LinkUp L1110 bridge.
 */
static ssize_t ipaq_sleeve_socket_type_show(struct device *dev, char *buf,
					    unsigned int socket)
{
	u16 type = to_ipaq_sleeve_device(dev)->record.socket_type[socket];

	if (type < 2)
		return sysfs_emit(buf, "direct\n");
	if (type == 2)
		return sysfs_emit(buf, "l1110-bridge\n");
	return sysfs_emit(buf, "unknown (%u)\n", type);
}

static ssize_t socket0_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return ipaq_sleeve_socket_type_show(dev, buf, 0);
}
static DEVICE_ATTR_RO(socket0_type);

static ssize_t socket1_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return ipaq_sleeve_socket_type_show(dev, buf, 1);
}
static DEVICE_ATTR_RO(socket1_type);

static struct attribute *ipaq_sleeve_dev_attrs[] = {
	&dev_attr_id_eeprom_valid.attr,
	&dev_attr_vendor.attr,
	&dev_attr_device.attr,
	&dev_attr_type.attr,
	&dev_attr_record_length_raw.attr,
	&dev_attr_record_version.attr,
	&dev_attr_record_format.attr,
	&dev_attr_description.attr,
	&dev_attr_type_raw.attr,
	&dev_attr_interrupt_enable_raw.attr,
	&dev_attr_extended_battery.attr,
	&dev_attr_initial_power_state_raw.attr,
	&dev_attr_suspend_power_state_raw.attr,
	&dev_attr_reset_width_ms.attr,
	&dev_attr_stabilize_ms.attr,
	&dev_attr_unknown_1a_raw.attr,
	&dev_attr_unknown_1c_raw.attr,
	&dev_attr_socket0_type.attr,
	&dev_attr_socket1_type.attr,
	&dev_attr_opt_on_requested.attr,
	&dev_attr_memory_window_bytes.attr,
	&dev_attr_memory_capable.attr,
	&dev_attr_bb_word1_raw.attr,
	&dev_attr_bb_word2_raw.attr,
	NULL,
};

/*
 * Record fields exist only for a record that parsed in full, and the
 * socket types and OPT_ON byte only in v2/v3 layouts; a file that is
 * absent cannot be misread as a field value of zero.
 */
static umode_t ipaq_sleeve_dev_attr_visible(struct kobject *kobj,
					    struct attribute *attr, int n)
{
	const struct ipaq_sleeve_device *sdev =
		to_ipaq_sleeve_device(kobj_to_dev(kobj));

	if (attr == &dev_attr_id_eeprom_valid.attr ||
	    attr == &dev_attr_vendor.attr ||
	    attr == &dev_attr_device.attr ||
	    attr == &dev_attr_type.attr)
		return attr->mode;

	if (!sdev->record_valid)
		return 0;

	if ((attr == &dev_attr_socket0_type.attr ||
	     attr == &dev_attr_socket1_type.attr ||
	     attr == &dev_attr_opt_on_requested.attr) &&
	    sdev->record.version < 2)
		return 0;

	return attr->mode;
}

static const struct attribute_group ipaq_sleeve_dev_group = {
	.attrs		= ipaq_sleeve_dev_attrs,
	.is_visible	= ipaq_sleeve_dev_attr_visible,
};
__ATTRIBUTE_GROUPS(ipaq_sleeve_dev);

const struct bus_type ipaq_sleeve_bus_type = {
	.name		= "ipaq-sleeve",
	.match		= ipaq_sleeve_bus_match,
	.probe		= ipaq_sleeve_bus_probe,
	.remove		= ipaq_sleeve_bus_remove,
	.suspend	= ipaq_sleeve_bus_suspend,
	.resume		= ipaq_sleeve_bus_resume,
	.bus_groups	= ipaq_sleeve_bus_groups,
	.dev_groups	= ipaq_sleeve_dev_groups,
};
EXPORT_SYMBOL_GPL(ipaq_sleeve_bus_type);

int ipaq_sleeve_driver_register(struct ipaq_sleeve_driver *drv)
{
	drv->driver.bus = &ipaq_sleeve_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_driver_register);

void ipaq_sleeve_driver_unregister(struct ipaq_sleeve_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_driver_unregister);

static void ipaq_sleeve_task_handler(struct work_struct *work)
{
	int opt_det = 0;

	/*
	 * Sample the detect line under the lock: the identity_override store
	 * path can hold it for several hundred milliseconds, and a value read
	 * before waiting for it would be stale by the time it is used. It
	 * also covers a presence IRQ that re-schedules this work after
	 * ipaq_sleeve_unregister() has returned from cancel_delayed_work_sync().
	 */
	guard(mutex)(&ipaq_sleeve_lock);

	if (!sleeve_ops)
		return;

	sleeve_ops->get_option_detect(sleeve_dev, &opt_det);

	/*
	 * Act only on a change: insert when a jacket is present and none is
	 * registered, eject when none is present. Present with a sleeve
	 * already registered changes nothing -- most often a backend's resume
	 * hook re-checking a jacket that never moved; calling
	 * ipaq_sleeve_insert() there would run its eject-then-reinsert, which
	 * on h3800/h39xx power-cycles a mounted CF root. The cost: presence
	 * alone cannot tell a jacket swapped while the system slept from the
	 * same one, so a swapped jacket keeps the previous identity until
	 * presence next reads absent. A bound ipaq_sleeve_driver's own
	 * ->resume() runs independently, through ipaq_sleeve_bus_resume().
	 */
	if (opt_det) {
		if (!active_sleeve)
			ipaq_sleeve_insert();
	} else {
		ipaq_sleeve_eject();
	}
}

/*
 * A delayed work, not a plain one with a sleep in the handler: a bounce edge
 * arriving while the previous run was already past the sleep would queue a
 * second, back-to-back run instead of pushing the sample out. Every edge
 * restarts the 100 ms timer, so only the last edge of a bounce burst
 * produces a sample.
 */
static DECLARE_DELAYED_WORK(ipaq_sleeve_task, ipaq_sleeve_task_handler);

irqreturn_t ipaq_sleeve_presence_interrupt(int irq, void *dev_id)
{
	mod_delayed_work(system_wq, &ipaq_sleeve_task, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_presence_interrupt);

/*
 * ipaq_sleeve_register: install @ops/@dev as the active bus transport
 * and kick off initial enumeration. Called once by a backend driver's
 * probe (e.g. a SAMCOP EPS driver), analogous to how the backend later
 * wires ipaq_sleeve_presence_interrupt() to its own option-detect IRQ.
 */
int ipaq_sleeve_register(struct ipaq_sleeve_ops *ops, void *dev)
{
	/*
	 * The identity_override store path calls ipaq_sleeve_insert() from its
	 * own context. Hold ipaq_sleeve_lock across publishing the ops and
	 * clearing the rails, so that a store sees either no backend yet or a
	 * powered-down connector, and never reads the ID EEPROM while the
	 * rails are being cleared. The lock order is ipaq_sleeve_lock, then
	 * ipaq_sleeve_ops_lock.
	 */
	scoped_guard(mutex, &ipaq_sleeve_lock) {
		scoped_guard(mutex, &ipaq_sleeve_ops_lock) {
			if (sleeve_ops)
				return -EBUSY;
			sleeve_ops = ops;
			sleeve_dev = dev;
		}

		ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_RESET);
		ipaq_sleeve_clear_egpio(IPAQ_EGPIO_CARD_RESET);
		ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_ON);
		ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);
	}

	mod_delayed_work(system_wq, &ipaq_sleeve_task, msecs_to_jiffies(100));

	return 0;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_register);

/*
 * The teardown is one critical section: the identity_override store path is
 * not cancel_delayed_work_sync()'d, so it can be in ipaq_sleeve_insert()
 * driving the egpio operations while a backend unloads. Holding
 * ipaq_sleeve_lock from the eject through clearing the ops makes the "is
 * there a backend" test of the store authoritative.
 */
int ipaq_sleeve_unregister(void)
{
	if (!sleeve_ops)
		return -ENODEV;

	cancel_delayed_work_sync(&ipaq_sleeve_task);

	guard(mutex)(&ipaq_sleeve_lock);

	ipaq_sleeve_eject();

	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_CARD_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_ON);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);

	scoped_guard(mutex, &ipaq_sleeve_ops_lock) {
		sleeve_ops = NULL;
		sleeve_dev = NULL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_unregister);

static int __init ipaq_sleeve_init(void)
{
	return bus_register(&ipaq_sleeve_bus_type);
}
module_init(ipaq_sleeve_init);

static void __exit ipaq_sleeve_exit(void)
{
	bus_unregister(&ipaq_sleeve_bus_type);
}
module_exit(ipaq_sleeve_exit);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("iPAQ H3xxx/h5xxx expansion pack (sleeve) bus");
MODULE_LICENSE("GPL");
