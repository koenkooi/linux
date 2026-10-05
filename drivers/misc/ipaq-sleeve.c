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
 *    covers the latter.
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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/ipaq-sleeve.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* Set once by ipaq_sleeve_register(), cleared by ipaq_sleeve_unregister().
 * @sleeve_ops/@sleeve_dev are only ever mutated under
 * ipaq_sleeve_ops_lock; they are read without it from the work item and
 * from the ipaq_sleeve_write_egpio()/read_egpio()/egpio_irq() helpers,
 * which is safe because a backend is expected to unregister only after
 * quiescing its own presence IRQ and calling cancel_delayed_work_sync()
 * (done for it, below, inside ipaq_sleeve_unregister()) -- the lock exists
 * to make double-registration safe, not to serialise every access.
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
 * Feature lookup table: the single authority for how many CF or PC Card
 * slots a sleeve has, and what else it carries. It is keyed on the IDs
 * read from the sleeve, so a sleeve with no bound driver at all, for example
 * an unrecognised jacket or one whose only driver is unrelated to
 * CF/PCMCIA, still gets an accurate slot count.
 *
 * The GPRS pack and the Symbol WLAN scanner declare no CF or PCMCIA bit, so
 * they get no socket. An all-ones ID is a blank ID EEPROM, and such a
 * jacket gets one PC Card socket. An all-zeros ID is deliberately not in
 * the table, see ipaq_sleeve_insert() for why that case has to be loud.
 */
struct ipaq_sleeve_feature {
	unsigned short vendor, device;
	unsigned int features;
};

static const struct ipaq_sleeve_feature ipaq_sleeve_feature_table[] = {
	{ COMPAQ_VENDOR_ID, SINGLE_COMPACTFLASH_SLEEVE, SLEEVE_HAS_CF(1) },
	{ COMPAQ_VENDOR_ID, SINGLE_CF_PLUS_SLEEVE,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT },
	{ COMPAQ_VENDOR_ID, SINGLE_PCMCIA_SLEEVE,
	  SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_FIXED_BATTERY },
	{ COMPAQ_VENDOR_ID, SINGLE_PCMCIA_PLUS_SLEEVE,
	  SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_REMOVABLE_BATTERY | SLEEVE_HAS_EBAT },
	{ COMPAQ_VENDOR_ID, DUAL_PCMCIA_SLEEVE,
	  SLEEVE_HAS_PCMCIA(2) | SLEEVE_HAS_FIXED_BATTERY },
	{ COMPAQ_VENDOR_ID, BLUETOOTH_EXPANSION_PACK,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_BLUETOOTH },
	{ COMPAQ_VENDOR_ID, GPRS_EXPANSION_PACK,
	  SLEEVE_HAS_FIXED_BATTERY | SLEEVE_HAS_GPRS },
	{ NEXIAN_VENDOR, NEXIAN_DUAL_CF_SLEEVE,
	  SLEEVE_HAS_CF(2) | SLEEVE_HAS_REMOVABLE_BATTERY },
	{ NEXIAN_VENDOR, NEXIAN_CAMERA_CF_SLEEVE,
	  SLEEVE_HAS_CF(1) | SLEEVE_HAS_CAMERA },
	{ SYMBOL_VENDOR, SYMBOL_WLAN_SCANNER_SLEEVE,
	  SLEEVE_HAS_BARCODE_READER | SLEEVE_HAS_802_11B | SLEEVE_HAS_FIXED_BATTERY },
	{ PITECH_VENDOR, MEMPLUG_SLEEVE, SLEEVE_HAS_CF(2) },
	{ PITECH_VENDOR, MEMPLUG_SLEEVE3, SLEEVE_HAS_CF(2) },
	{ TALON_VENDOR, NAVMAN_GPS_SLEEVE, SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS },
	{ TALON_VENDOR, NAVMAN_GPS_SLEEVE_ALT, SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS },
	{ 0xffff, 0xffff, SLEEVE_HAS_PCMCIA(1) | SLEEVE_HAS_FIXED_BATTERY },
};

unsigned int ipaq_sleeve_features(const struct ipaq_sleeve_device *sdev)
{
	unsigned int i;

	if (!sdev->id_valid)
		return 0;

	for (i = 0; i < ARRAY_SIZE(ipaq_sleeve_feature_table); i++) {
		const struct ipaq_sleeve_feature *f = &ipaq_sleeve_feature_table[i];

		if (f->vendor == sdev->vendor && f->device == sdev->device)
			return f->features;
	}

	return 0;
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
 * settle, read its ID EEPROM, register it on the bus so any matching
 * ipaq_sleeve_driver can bind, then power the jacket's main rail and
 * notify slot consumers (e.g. the CF socket). EEPROM failure leaves
 * the sleeve explicitly unidentified: ID-matched drivers cannot bind,
 * but a board-specific slot consumer may still use physical detect
 * lines. Runs from ipaq_sleeve_task_handler(), i.e. process context,
 * never from the presence IRQ itself.
 *
 * Order matters and is deliberate: NVRAM rail -> settle -> EEPROM read
 * -> device_register() -> OPT_ON -> settle -> notify. device_register()
 * before OPT_ON so a sleeve-specific driver (e.g. the Navman GPS UART)
 * finishes its own probe on an unpowered-but-identified sleeve first;
 * notify last so a slot consumer only ever sees a sleeve whose jacket
 * rail is already up and whose own driver, if any, has already bound.
 * Caller holds ipaq_sleeve_lock.
 */
static void ipaq_sleeve_insert(void)
{
	struct ipaq_sleeve_device *sdev;
	unsigned short vendor = 0, device = 0;
	unsigned char eeprom[10] = { 0 };
	bool id_valid = false;
	int ret;

	/*
	 * A stale active_sleeve means the last remove was missed, most likely
	 * contact bounce re-triggering the task before the unregister of a
	 * real removal ran. Eject it before powering the rail, not after:
	 * ejecting powers the rail off again, and a second power-on without
	 * a settle would make the EEPROM read below see a jacket that has
	 * been powered for 0 ms instead of 250.
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

	/*
	 * Read the header through the device ID (bytes 0 to 9) in one
	 * transaction. The vendor software rejects anything but 0xaa in byte
	 * 0 before it looks at the vendor ID (offset 6) or the device ID
	 * (offset 8), both 2 bytes. Without that check, a result of 0000:0000
	 * cannot be told apart between a dead or mis-clocked bus and a jacket
	 * without an ID EEPROM.
	 */
	ret = sleeve_ops->spi_read(sleeve_dev, 0, eeprom, sizeof(eeprom));
	if (ret) {
		pr_err("ipaq-sleeve: ID EEPROM read failed: %d; continuing without sleeve identity\n",
		       ret);
	} else if (eeprom[0] != 0xaa) {
		pr_err("ipaq-sleeve: ID EEPROM byte 0 is 0x%02x, expected 0xaa: bus dead or mis-clocked, or no ID EEPROM on this jacket; continuing without sleeve identity\n",
		       eeprom[0]);
	} else {
		vendor = eeprom[6] | (eeprom[7] << 8);
		device = eeprom[8] | (eeprom[9] << 8);

		if (!vendor || !device) {
			pr_err("ipaq-sleeve: ID EEPROM returned invalid jacket %04x:%04x; continuing without sleeve identity\n",
			       vendor, device);
			vendor = 0;
			device = 0;
		} else {
			id_valid = true;
		}
	}

	sdev = kzalloc_obj(*sdev, GFP_KERNEL);
	if (!sdev) {
		sleeve_ops->control_egpio(sleeve_dev,
					  IPAQ_EGPIO_OPT_NVRAM_ON, 0);
		return;
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

	if (id_valid)
		pr_debug("ipaq-sleeve: jacket %04x:%04x, features 0x%08x\n",
			 vendor, device, ipaq_sleeve_features(sdev));
	else
		pr_info("ipaq-sleeve: unidentified jacket present\n");

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

const struct bus_type ipaq_sleeve_bus_type = {
	.name		= "ipaq-sleeve",
	.match		= ipaq_sleeve_bus_match,
	.probe		= ipaq_sleeve_bus_probe,
	.remove		= ipaq_sleeve_bus_remove,
	.suspend	= ipaq_sleeve_bus_suspend,
	.resume		= ipaq_sleeve_bus_resume,
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

	sleeve_ops->get_option_detect(sleeve_dev, &opt_det);

	guard(mutex)(&ipaq_sleeve_lock);

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
	mutex_lock(&ipaq_sleeve_ops_lock);
	if (sleeve_ops) {
		mutex_unlock(&ipaq_sleeve_ops_lock);
		return -EBUSY;
	}
	sleeve_ops = ops;
	sleeve_dev = dev;
	mutex_unlock(&ipaq_sleeve_ops_lock);

	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_CARD_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_ON);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);

	mod_delayed_work(system_wq, &ipaq_sleeve_task, msecs_to_jiffies(100));

	return 0;
}
EXPORT_SYMBOL_GPL(ipaq_sleeve_register);

int ipaq_sleeve_unregister(void)
{
	if (!sleeve_ops)
		return -ENODEV;

	cancel_delayed_work_sync(&ipaq_sleeve_task);

	scoped_guard(mutex, &ipaq_sleeve_lock)
		ipaq_sleeve_eject();

	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_CARD_RESET);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_ON);
	ipaq_sleeve_clear_egpio(IPAQ_EGPIO_OPT_NVRAM_ON);

	mutex_lock(&ipaq_sleeve_ops_lock);
	sleeve_ops = NULL;
	sleeve_dev = NULL;
	mutex_unlock(&ipaq_sleeve_ops_lock);

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
