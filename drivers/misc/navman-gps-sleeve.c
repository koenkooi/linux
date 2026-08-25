// SPDX-License-Identifier: GPL-2.0-only
/*
 * Navman GPS 3000/3400/3450 "Smart sleeve" (Talon/Navman GPS/CF sleeve)
 * expansion pack, HP iPAQ h39xx (ASIC2) and h5xxx (SAMCOP).
 *
 * A snap-on expansion pack combining a built-in GPS receiver, reachable as
 * a memory-mapped 16550-class serial port in the PC Card address space of
 * the host, with an integrated CompactFlash slot. Only the GPS half lives
 * here. The CF slot is instantiated independently, by the PCMCIA socket
 * driver of the board, as a sibling consumer of the same sleeve bus event
 * that this file consumes, see IPAQ_SLEEVE_EVENT_ADD in
 * include/linux/ipaq-sleeve.h.
 *
 * Based on the driver of the same purpose in the 2.4 handheld kernels. Its
 * h3900 branch was complete, its h5400 branch was a bare
 *
 *	#elif defined(CONFIG_ARCH_H5400)
 *		#warning need to set up serial irq for navman sleeve for h5400
 *
 * because on the h5400 the sleeve interrupt is not a raw PXA GPIO edge line
 * but a source inside the EPS interrupt controller of the SAMCOP. Both
 * boards are handled here through one abstraction, so that neither needs a
 * board string in this file.
 *
 * What differs from the historical driver:
 *
 *  - The sleeve interrupt is resolved through the egpio_irq() operation of
 *    the sleeve bus as IPAQ_EGPIO_OPT_INT, and each backend maps that
 *    identifier onto its own hardware. That operation is the only part that
 *    is per machine. The asymmetry is real: on the h39xx OPT_INT is a
 *    different physical line from the PC Card interrupt, whereas the SAMCOP
 *    EPS has no separate option interrupt source, so on the h5xxx the two
 *    are necessarily the same line. That is why the port is registered with
 *    UPF_SHARE_IRQ.
 *
 *  - register_serial() and setup_serial(), both long removed, become
 *    serial8250_register_8250_port() against a struct uart_8250_port, as
 *    drivers/tty/serial/8250/serial_cs.c does.
 *
 *  - The UART sits at the start of the I/O window of PC Card socket 1,
 *    the jacket's second socket, with byte register spacing and a
 *    3.6864 MHz clock, see NAVMAN_GPS_UART_PHYS. The jacket's CF slot is
 *    socket 0. The jacket drives socket 1's card-detect line itself, so
 *    Card Services reports a "card" in socket 1 whenever the jacket is
 *    attached (measured on an h3970: the attribute window reads a
 *    constant 0x0001 pattern, not a CIS). This driver is not a Card
 *    Services client: there is no CIS to parse, so pcmcia_request_io()
 *    and friends have nothing to act on. A UART registered inside a
 *    socket's window before that socket exists makes
 *    soc_pcmcia_add_one()'s request_resource() over the whole window
 *    fail and the CF socket never register, so this driver orders itself
 *    after the socket driver on both delivery paths of the sleeve event
 *    -- see navman_sleeve_nb and the initcall comment.
 *
 *  - All hardware work runs from the sleeve bus's ADD/REMOVE notifier
 *    rather than from .probe(). Two reasons, either sufficient.
 *    ipaq_sleeve_insert() (drivers/misc/ipaq-sleeve.c) deliberately
 *    asserts the jacket's main rail, IPAQ_EGPIO_OPT_ON, *after*
 *    device_register() and settles 250 ms before notifying, so a bound
 *    driver's .probe() runs against an unpowered jacket -- touching the
 *    UART there reads a dead bus. And a jacket whose ID EEPROM cannot
 *    be read (earlier SPI transports returned 0000:0000) matches no
 *    driver unless the operator forces an identity (the sleeve bus's
 *    identity_override, Documentation/ABI/testing/sysfs-bus-ipaq-sleeve),
 *    so .probe() is never called. The notifier fires either way, which
 *    makes it the only route that works in both cases.
 *
 *  - The driver does not assert or clear IPAQ_EGPIO_OPT_ON. The bus core
 *    owns that line, and a driver clearing it on teardown would race the
 *    eject sequence of the core.
 *
 *  - No GPS or NMEA protocol handling: gpsd talks to the resulting tty.
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ipaq-sleeve.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/types.h>
#include <pcmcia/ss.h>

/*
 * The UART of the jacket: the I/O base of PC Card socket 1, register offset 0,
 * byte register spacing.
 *
 * On both the PXA2xx and the SA-1110 the PC Card space is two 256 MB socket
 * windows at 0x20000000 and 0x30000000, each starting with its I/O partition,
 * so socket 1's I/O base is 0x30000000 on every iPAQ that takes this jacket.
 * It is the address of the UART in the handheld kernels' driver for this
 * jacket, and was measured on an h3970 with MECR.NOS set, so that socket 1 is
 * selected: a 16550 answers there. The whole 4 KiB I/O partition repeats one
 * 8-byte register file; IER, IIR, LCR and MCR read 0x00, 0x01, 0x00, 0x00, LSR
 * 0x60, MSR 0x20, stable over repeated reads, and the scratch register stores
 * and returns 0x5a, 0xa5, 0x00 and 0xff.
 *
 * On the PXA2xx the factory firmware programs MECR = 3 (CIT | NOS, two
 * sockets) unconditionally. pxa2xx_configure_sockets() sets NOS only when the
 * board registers two sockets; with it clear, socket 1 reads as an undriven
 * bus.
 */
#define NAVMAN_GPS_UART_PHYS		0x30000000UL
#define NAVMAN_GPS_UART_SIZE		8

/*
 * The UART's input clock. The handheld kernels' driver gave the port a
 * baud_base of 230400, which is 3686400 / 16. On the h3970 the receiver's
 * GGA, RMC and GSA sentences come out valid at 57600 baud 8N1 with divisor 4,
 * which is this clock.
 */
#define NAVMAN_GPS_UART_CLOCK		3686400

/* All three guarded by navman_lock; navman_line < 0 means "not attached". */
static DEFINE_MUTEX(navman_lock);
static void __iomem *navman_membase;
static int navman_line = -1;

static int navman_pccard_count_cb(struct device *dev, void *data)
{
	unsigned int *sockets = data;

	(*sockets)++;

	return 0;
}

/*
 * Ask the bus whether anything answers at @membase, with the same existence
 * test that autoconfig() in 8250_port.c runs before it identifies a port:
 * write IER, read it back, twice, with values chosen so that both a floating
 * bus (reads 0xff, so the first read-back is non-zero) and a dead one (reads
 * 0x00, so the second read-back is not UART_IER_ALL_INTR) fail.
 *
 * It is duplicated here because serial8250_get_port(), which would allow to
 * inspect port.type after registering, is private to the 8250 directory.
 * Testing first means that a jacket whose UART does not answer, for example
 * on a board that leaves socket 1 deselected, leaves no phantom ttyS behind.
 * autoconfig() would write the same two bytes to the same register once the
 * port was registered, and the window is the jacket's own socket, which holds
 * no card.
 */
static bool navman_uart_present(void __iomem *membase)
{
	u8 saved, scratch, scratch2;

	saved = readb(membase + UART_IER);

	writeb(0, membase + UART_IER);
	scratch = readb(membase + UART_IER) & UART_IER_ALL_INTR;

	writeb(UART_IER_ALL_INTR, membase + UART_IER);
	scratch2 = readb(membase + UART_IER) & UART_IER_ALL_INTR;

	writeb(saved, membase + UART_IER);

	return scratch == 0 && scratch2 == UART_IER_ALL_INTR;
}

/*
 * serial8250_config_port() returns before it sets up->fcr when its
 * request_mem_region() fails, which it does here, see navman_attach(), so the
 * FIFO would stay off. NMEA arrives in 80-byte bursts at 57600 baud, a byte
 * every 174 us, and with one interrupt per byte the port overran 16 to 22
 * times in 30 s. Set the FCR that config_port() gives a 16550A.
 */
static int navman_uart_startup(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	up->fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10;

	return serial8250_do_startup(port);
}

/*
 * Caller holds navman_lock and has established that nothing is attached.
 * Returns nothing: a failure here only means that this jacket gets no GPS
 * tty, which the notifier cannot act on, and every case is logged where it is
 * detected.
 */
static void navman_attach(struct device *dev)
{
	struct uart_8250_port uart = { };
	unsigned int sockets = 0;
	void __iomem *membase;
	int ret, irq, line;

	ret = class_for_each_device(&pcmcia_socket_class, NULL, &sockets,
				    navman_pccard_count_cb);
	if (ret) {
		dev_err(dev, "cannot enumerate PC Card sockets: %d\n", ret);
		return;
	}

	membase = ioremap(NAVMAN_GPS_UART_PHYS, NAVMAN_GPS_UART_SIZE);
	if (!membase) {
		dev_err(dev, "cannot map GPS UART at %#lx\n",
			NAVMAN_GPS_UART_PHYS);
		return;
	}

	if (!navman_uart_present(membase)) {
		dev_info(dev, "no 16550 answered at %#lx\n",
			 NAVMAN_GPS_UART_PHYS);
		iounmap(membase);
		return;
	}

	/*
	 * With port.irq == 0 the 8250 core falls back to polling with a
	 * timer, which still gives a usable tty.
	 *
	 * Both backends hand out a dedicated, edge triggered SoC GPIO for
	 * this line. PXA GPIO has no level mode, while the INT output of a
	 * real 16550 stays asserted until it is acknowledged, so an edge
	 * only interrupt servicing a level style source can lose events. It
	 * is not known which way this line behaves.
	 */
	irq = ipaq_sleeve_egpio_irq(IPAQ_EGPIO_OPT_INT);
	if (irq < 0) {
		dev_info(dev,
			 "no sleeve interrupt from this backend (%d); running the GPS UART polled\n",
			 irq);
		irq = 0;
	}

	uart.port.iotype = UPIO_MEM;
	uart.port.mapbase = NAVMAN_GPS_UART_PHYS;
	uart.port.membase = membase;
	uart.port.uartclk = NAVMAN_GPS_UART_CLOCK;
	uart.port.irq = irq;
	uart.port.startup = navman_uart_startup;
	/*
	 * UPF_SKIP_TEST skips only the MCR loopback test of autoconfig(), not
	 * the existence test above it. serial_cs.c sets it for the same
	 * reason. The loopback test drives MCR lines that run through the
	 * buffer chip of the sleeve on a PC Card, and a false negative there
	 * costs a cold boot to recover from, where a false positive costs only
	 * a tty that emits nothing.
	 *
	 * UPF_FIXED_TYPE, together with port.type below, is safe to set
	 * because navman_uart_present() above already did the discrimination
	 * autoconfig() would have. It is also load-bearing inside a
	 * registered socket's I/O partition: serial8250_config_port() starts
	 * with its own request_mem_region() over this range, which the
	 * socket's IORESOURCE_BUSY reservation rejects, and returns early.
	 * With UPF_FIXED_TYPE our port.type survives that
	 * (uart_configure_port() only resets it to PORT_UNKNOWN when the flag
	 * is clear), serial8250_set_defaults() supplies the FIFO size and
	 * capabilities for the fixed type, and navman_uart_startup() the FCR.
	 * What is lost is the "serial" line in /proc/iomem.
	 *
	 * PORT_16550A and not PORT_NS16550A: the latter is a specific National
	 * Semiconductor high speed variant for which 8250_port.c forces bank
	 * switched register access and a different clock derivation, a
	 * hardware claim that nothing here supports. Nothing identifies this
	 * chip, so the generic type without special capabilities is used.
	 */
	uart.port.type = PORT_16550A;
	uart.port.flags = UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_FIXED_TYPE;
	uart.port.dev = dev;

	line = serial8250_register_8250_port(&uart);
	if (line < 0) {
		dev_err(dev, "cannot register GPS UART at %#lx: %d\n",
			NAVMAN_GPS_UART_PHYS, line);
		iounmap(membase);
		return;
	}

	navman_membase = membase;
	navman_line = line;

	dev_info(dev, "GPS UART at %#lx on ttyS%d\n", NAVMAN_GPS_UART_PHYS,
		 line);

	/*
	 * With no socket registered, the 8250 core now holds a "serial"
	 * region directly under iomem_resource, inside the window that a
	 * socket driver claims whole with request_resource() when it
	 * registers (soc_pcmcia_add_one()). That later claim conflicts and
	 * the CF socket never registers. The ordering below makes this
	 * unreachable for a built-in socket driver; it remains reachable by
	 * loading a modular one after this module.
	 */
	if (!sockets)
		dev_warn(dev,
			 "GPS UART registered inside PC Card socket 1's window while no PC Card socket is registered; a socket driver loaded from now on will fail to register until this module is unloaded -- load it first\n");
}

/* Caller holds navman_lock and has established that something is attached. */
static void navman_detach(void)
{
	serial8250_unregister_port(navman_line);
	iounmap(navman_membase);
	navman_membase = NULL;
	navman_line = -1;
}

/*
 * Log only on purpose, see the header comment on why the jacket is still
 * unpowered when .probe() of a bound driver runs. Binding by ID is still
 * worth doing, because it is what tells the notifier below that this jacket
 * really is a Navman and not merely an unidentified one.
 */
static int navman_sleeve_probe(struct ipaq_sleeve_device *sdev)
{
	dev_info(&sdev->dev,
		 "Navman GPS/CF sleeve identified; GPS UART attaches once the jacket rail is up\n");
	return 0;
}

static const struct ipaq_sleeve_device_id navman_sleeve_id_table[] = {
	{ TALON_VENDOR, NAVMAN_GPS_SLEEVE },
	{ TALON_VENDOR, NAVMAN_GPS_SLEEVE_ALT },
	{ 0, }
};

static struct ipaq_sleeve_driver navman_sleeve_driver = {
	.driver = {
		.name = "navman-gps-sleeve",
	},
	.id_table = navman_sleeve_id_table,
	.features = SLEEVE_HAS_CF(1) | SLEEVE_HAS_GPS,
	.probe = navman_sleeve_probe,
};

/*
 * The bus core fires this chain unconditionally, for an identified jacket or
 * not, and only once the main rail of the jacket is up and settled. This
 * driver needs both properties and .probe() has neither.
 */
static int navman_sleeve_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct ipaq_sleeve_device *sdev = data;

	guard(mutex)(&navman_lock);

	switch (action) {
	case IPAQ_SLEEVE_EVENT_ADD:
		if (navman_line >= 0)
			break;

		/*
		 * Bound by ID, the one of the jacket or one forced with the
		 * identity_override of the bus, and by nothing else. A jacket
		 * that the bus could not identify matches no driver, so this
		 * one test covers both: anything else on the connector is not
		 * the jacket of this driver.
		 */
		if (sdev->dev.driver != &navman_sleeve_driver.driver)
			break;

		navman_attach(&sdev->dev);
		break;

	case IPAQ_SLEEVE_EVENT_REMOVE:
		if (navman_line < 0)
			break;

		navman_detach();
		break;
	}

	return NOTIFY_OK;
}

/*
 * A sleeve event reaches this driver on one of two paths, and the PCMCIA
 * socket driver of the board must go first on both, see "Ordering" in the
 * header comment. This block covers the live path. An ADD delivered by
 * blocking_notifier_call_chain() runs its callbacks in descending priority,
 * and the socket drivers register at the default 0, so a lower priority puts
 * this callback after theirs whichever of the two registered first. Their
 * callback registers the "pxa2xx-pcmcia" platform device, which probes
 * synchronously, so by the time this one runs the window of the socket is
 * claimed and its present flag exists. The other path, the replay of an
 * already enumerated jacket by ipaq_sleeve_register_notifier(), is ordered
 * by who registers first, see the initcall comment below.
 */
static struct notifier_block navman_sleeve_nb = {
	.notifier_call = navman_sleeve_notify,
	.priority = -1,
};

static int __init navman_sleeve_init(void)
{
	int ret;

	/*
	 * Driver first: ipaq_sleeve_register_notifier() replays an already
	 * present jacket as an immediate ADD, and that replay must be able to
	 * see whether this driver bound to it by ID.
	 */
	ret = ipaq_sleeve_driver_register(&navman_sleeve_driver);
	if (ret)
		return ret;

	ret = ipaq_sleeve_register_notifier(&navman_sleeve_nb);
	if (ret)
		ipaq_sleeve_driver_unregister(&navman_sleeve_driver);

	return ret;
}

/*
 * late_initcall_sync and not module_init, for the replay path: when this
 * driver is built in and the jacket has already enumerated by the time this
 * runs, ipaq_sleeve_register_notifier() delivers the ADD right here,
 * synchronously, and the socket driver must have registered its socket
 * before that. The socket drivers register from late_initcall, and every
 * late_initcall runs before any late_initcall_sync regardless of link order,
 * which matters because drivers/misc/ links before drivers/pcmcia/ and an
 * equal level would run this first. Nothing this depends on comes later: the
 * bus core, the 8250 core and the sleeve backends are device_initcall. As a
 * module this is module_init() anyway.
 */
late_initcall_sync(navman_sleeve_init);

static void __exit navman_sleeve_exit(void)
{
	ipaq_sleeve_unregister_notifier(&navman_sleeve_nb);
	ipaq_sleeve_driver_unregister(&navman_sleeve_driver);

	guard(mutex)(&navman_lock);

	if (navman_line >= 0)
		navman_detach();
}
module_exit(navman_sleeve_exit);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Navman GPS/CF sleeve driver (HP iPAQ h39xx/h5xxx)");
MODULE_LICENSE("GPL");
