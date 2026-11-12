// SPDX-License-Identifier: GPL-2.0-only
/*
 * SAMCOP EPS (expansion-pack status) backend for the generic iPAQ
 * H3xxx/h5xxx sleeve bus (drivers/misc/ipaq-sleeve.c), HP iPAQ
 * h51xx/h54xx/h55xx.
 *
 * Modelled on Erik Hovland's 2005 driver of the same purpose,
 * drivers/misc/samcop_sleeve.c (pre-mainline h5400 tree); the register
 * facts and control flow below are transcribed from that source and
 * cross-checked against include/linux/mfd/samcop.h. Concrete changes and
 * why:
 *
 *  - struct ipaq_sleeve_ops against a real struct platform_device
 *    probed as a drivers/mfd/samcop.c mfd_cell ("samcop-sleeve"),
 *    instead of the original's own hand-registered platform_driver
 *    against a machine-specific h5400_eps_device. Resource lookup
 *    (mem window, hwirq) comes from that cell's DEFINE_RES_MEM()/
 *    DEFINE_RES_IRQ() in drivers/mfd/samcop.c, already translated to a
 *    real Linux IRQ through SAMCOP's own irq_domain by the mfd core.
 *
 *  - The ID EEPROM read (samcop_eps_spi_read(), historical lines
 *    ~116-148) goes through the real spi_controller/spi_sync() API against
 *    the ssp1 SPI controller (pxa25x.dtsi's "mrvl,pxa25x-ssp" node), not
 *    the original's own hand-rolled SSCR0/SSDR busy-wait/FIFO code.
 *
 *    Getting there is not a plain of_find_spi_controller_by_node() on
 *    ssp1, though: drivers/soc/pxa/ssp.c's own pxa_ssp_driver matches the
 *    identical "mrvl,pxa25x-ssp" compatible string and registers earlier
 *    (arch_initcall vs spi-pxa2xx-platform.c's subsys_initcall); its
 *    probe() only steps aside in spi-pxa2xx-platform.c's favour when the
 *    SSP node has a real DT child (added for hx4700's ssp2/touchscreen),
 *    and ssp1 here has none -- the sleeve's ID EEPROM has no fixed
 *    OF-compatible identity of its own (whichever sleeve is plugged in
 *    varies, and is discovered only by reading it), so there is nothing to
 *    declare as a child. This node's "samsung,sleeve-port" property (a
 *    phandle to &ssp1) is instead resolved directly and handed to
 *    pxa_ssp_request_of() + pxa2xx_spi_probe() (both exported entry
 *    points, include/linux/spi/pxa2xx_spi.h) -- the shape
 *    sound/soc/pxa/pxa-ssp.c uses via the "port" phandle of its
 *    "mrvl,pxa-ssp-dai" nodes -- so this driver never competes with
 *    pxa_ssp_driver for the raw node, and the resulting spi_controller is
 *    looked up afterwards by this node's own of_node (which
 *    spi_register_controller() gives it via device_set_node() in
 *    drivers/spi/spi.c, since this driver passes its own device as
 *    pxa2xx_spi_probe()'s parent), not ssp1's.
 *
 *    The chip select is GPIO24, muxed as a plain GPIO rather than the
 *    SSP1_SFRM alternate function, and resolved through this node's
 *    "cs-gpios" property (the SPI core reads it from the controller's
 *    firmware node, which is this node), not driven by hand the way the
 *    original's spi_set_cs() did. The spi_device itself is allocated by
 *    hand at probe with spi_alloc_device()/spi_add_device(), since the
 *    EEPROM has no fixed identity for a DT child node to name either.
 *    __spi_add_device() still resolves spi->cs_gpiod from the controller's
 *    cs-gpios array by chip_select index, so CS handling stays entirely
 *    the SPI core's job.
 *
 *  - OPT_ON/OPT_RESET (SAMCOP GPA bits 17/18, historical
 *    samcop_set_gpio_a() calls at lines ~217/226) are gpiod consumers
 *    against SAMCOP's own gpio_chip, resolved through the "opt-on-gpios"
 *    and "opt-reset-gpios" properties of this node, instead of the
 *    original's direct GPA register poke. OPT_NVRAM_ON (a plain PXA GPIO)
 *    is likewise a gpiod consumer ("nvram-gpios"). OPT_ON is optional: the
 *    h5000 DT leaves it out, because a driven GPA17 leaves the jacket's CF
 *    slot undriven (see samcop_sleeve_probe()).
 *
 *  - CARD_RESET writes SAMCOP_PCMCIA_CC directly (a real EPS register,
 *    unlike the three GPIO-backed lines above) -- unchanged in
 *    substance from the historical driver's own
 *    samcop_eps_write_register(eps, _SAMCOP_PCMCIA_CC, ...).
 *
 *  - IPAQ_EGPIO_PCMCIA_CD0_N/CD1_N/IRQ0/IRQ1 read_egpio()/egpio_irq()
 *    are implemented from the historical driver's own
 *    samcop_eps_read_egpio()/samcop_eps_egpio_irq() (lines 150-206):
 *    one SAMCOP_PCMCIA_EPS read masked with the matching
 *    SAMCOP_PCMCIA_EPS_{CD0_N,CD1_N,IRQ0_N,IRQ1_N} bit for the level,
 *    and a stored Linux virq for the IRQ. These four lines are the only
 *    path a PCMCIA socket driver has to a jacket's card-detect and
 *    card-interrupt state on this board: they are EPS register bits, not
 *    board GPIOs, and SAMCOP's own gpio_chip covers GPA-GPE only, so
 *    there is no gpiod to hand a socket driver instead. Consumed by
 *    the h5000 PCMCIA socket driver; see include/linux/ipaq-sleeve.h's
 *    own enum comment for the raw-bit read convention (this backend
 *    returns the raw active-low bit; the consumer inverts).
 *
 *    The historical egpio_irq() gated its lookup table on
 *    machine_is_h5400() and returned -EINVAL otherwise; that gate is
 *    dropped, since this file only builds under MFD_SAMCOP, which
 *    `depends on MACH_H5000` -- the SAMCOP this backend is a cell of
 *    shipped on no other machine. The four hwirqs themselves are
 *    SAMCOP_IRQ_EPS_CD0/_CD1/_IRQ0/_IRQ1 (include/linux/mfd/samcop.h),
 *    real individual sources of SAMCOP's own EPS demux tier -- not one
 *    aggregated PCMCIA interrupt -- delivered here already translated to
 *    Linux virqs by the MFD core, as DEFINE_RES_IRQ() entries 1-4 of
 *    drivers/mfd/samcop.c's samcop_sleeve_resources[].
 *
 *  - IPAQ_EGPIO_OPT_INT is the "opt-int-gpios" gpiod (a PXA GPIO), not an
 *    EPS register bit, and not a shared card IRQ. The board wiring matches
 *    h39xx's identical connector signal, so it is resolved the same way
 *    as drivers/misc/asic2-sleeve.c does: devm_gpiod_get_optional() then
 *    gpiod_to_irq(), non-fatal on every failure except -EPROBE_DEFER,
 *    since an unrouted or unresolvable line must not take the rest of the
 *    sleeve backend down with it.
 *
 * Power-up sequencing: the 250 ms OPT_NVRAM settle before the ID
 * EEPROM is read is already implemented generically by the bus core
 * itself (drivers/misc/ipaq-sleeve.c's ipaq_sleeve_insert(), which
 * calls control_egpio(IPAQ_EGPIO_OPT_NVRAM_ON, 1) then msleep(250)
 * before ever calling spi_read()) -- this driver does not duplicate
 * that delay. OPT_ON is no longer a per-sleeve-driver decision
 * either: drivers/misc/ipaq-sleeve.c's own ipaq_sleeve_insert() asserts
 * IPAQ_EGPIO_OPT_ON unconditionally after device_register() and waits
 * another 250 ms, so every jacket gets its main rail powered whether or
 * not any ipaq_sleeve_driver binds to it -- see that function's own
 * comment for why (h39xx's CD0_N does not read card presence at all
 * until that rail is up). OPT_RESET is likewise the bus core's alone:
 * it holds the line deasserted, pulsing it after OPT_ON only when the
 * pack's parsed ID EEPROM record asks for a reset width, and no
 * ipaq_sleeve_driver or slot consumer touches it.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ipaq-sleeve.h>
#include <linux/irq.h>
#include <linux/mfd/samcop.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/string.h>

/*
 * 25LC040 READ: opcode 3 with address bit A8 in opcode bit 3, one
 * address byte, then @len read bytes. The historical driver
 * (samcop_sleeve.c's samcop_eps_spi_read(), ~lines 116-148) sent a fixed
 * opcode and only the low address byte, which silently reads the lower
 * half of the array for any address from 0x100 up.
 */
#define SAMCOP_SLEEVE_EEPROM_READ(addr)	(0x03 | (((addr) & 0x100) >> 5))
#define SAMCOP_SLEEVE_EEPROM_SIZE	512

/*
 * Match the WinCE sleeve driver's SSCR0 = 0x1f87, i.e. SCR = 31.
 *
 * PXA25x SSP1 runs from the fixed 3.6864 MHz oscillator, and its bit
 * rate is 3686400 / (2 * (SCR + 1)) -- the factor of two is specific to
 * the PXA25x SSP port, and is what spi-pxa2xx's ssp_get_clk_div()
 * computes for PXA25x_SSP. SCR 31 is therefore 3686400 / 64 = 57600 Hz,
 * and 57600 is the exact rate that ssp_get_clk_div() turns back into
 * SCR = DIV_ROUND_UP(3686400, 2 * 57600) - 1 = 32 - 1 = 31.
 */
#define SAMCOP_SLEEVE_SPI_HZ		57600

struct samcop_sleeve {
	struct device		*dev;
	void __iomem		*regs;
	struct spi_device	*spi;
	/* pxa2xx_spi_probe() keeps this pointer for the controller's life. */
	struct pxa2xx_spi_controller spi_info;
	struct gpio_desc	*nvram_gpio;
	struct gpio_desc	*opt_on_gpio;
	struct gpio_desc	*opt_reset_gpio;
	struct gpio_desc	*opt_int_gpio;
	int			irq_odet;
	int			irq_cd0;
	int			irq_cd1;
	int			irq_irq0;
	int			irq_irq1;
	int			irq_opt_int;
	int			irq_bat_fault;

	/*
	 * Guards read-modify-write of SAMCOP_PCMCIA_IM, whose per-source
	 * 2-bit edge fields are written independently per socket -- see
	 * samcop_sleeve_rearm_cd_edge(). raw_spinlock_t, matching
	 * drivers/mfd/samcop.c's own chip-wide convention: the rearm is
	 * reachable from the PCMCIA socket path, which can run with
	 * interrupts disabled.
	 */
	raw_spinlock_t		im_lock;
};

/*
 * @reg is one of the shared, chip-absolute SAMCOP_PCMCIA_* register
 * macros (include/linux/mfd/samcop.h); this cell's own ioremap only
 * covers the PCMCIA/EPS sub-window (SAMCOP_PCMCIA_BASE..+SAMCOP_PCMCIA_SIZE),
 * so the byte offset into @eps->regs is @reg relative to
 * SAMCOP_PCMCIA_BASE, not @reg itself. 32-bit access matches every
 * other register in this MFD -- see drivers/mfd/samcop.c's own header
 * comment.
 */
static inline u32 samcop_sleeve_read(struct samcop_sleeve *eps, u32 reg)
{
	return readl_relaxed(eps->regs + (reg - SAMCOP_PCMCIA_BASE));
}

static inline void samcop_sleeve_write(struct samcop_sleeve *eps, u32 reg, u32 val)
{
	writel_relaxed(val, eps->regs + (reg - SAMCOP_PCMCIA_BASE));
}

/*
 * ipaq_sleeve_ops::spi_read -- read @len bytes from the sleeve's ID
 * EEPROM at @address into @data over ssp1, CS held asserted across the
 * whole transaction (one spi_message, two transfers, no cs_change
 * between them) -- reproduces the historical driver's own "CS low
 * across cmd+addr+data, high after" shape without touching the CS line
 * by hand; see this file's header comment.
 *
 * Returns 0 on success, matching asic2_sleeve_spi_read()'s documented
 * convention for this vtable slot (spi_write_then_read()'s own "0 on
 * success" contract) -- not the byte count. ipaq_sleeve_insert()'s
 * `if (!ret)`/`if (ret)` checks (drivers/misc/ipaq-sleeve.c) depend on
 * that: returning len here made the first (vendor-ID) read's own
 * success look like a failure to that caller, since len is 2 and
 * therefore truthy, which skipped the device-ID read entirely and
 * printed "ID EEPROM read failed: 2" on every successful read.
 */
static int samcop_sleeve_spi_read(void *dev, unsigned short address,
				  unsigned char *data, unsigned short len)
{
	struct samcop_sleeve *eps = dev;
	struct spi_transfer xfers[2] = { };
	struct spi_message msg;
	u8 cmd[2];

	if (!data || address >= SAMCOP_SLEEVE_EEPROM_SIZE)
		return -EINVAL;

	cmd[0] = SAMCOP_SLEEVE_EEPROM_READ(address);
	cmd[1] = address & 0xff;

	xfers[0].tx_buf = cmd;
	xfers[0].len = sizeof(cmd);
	xfers[1].rx_buf = data;
	xfers[1].len = len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	return spi_sync(eps->spi, &msg);
}

/*
 * ipaq_sleeve_ops::get_option_detect -- historical
 * samcop_eps_get_option_detect(), ~lines 234-249, transcribed exactly:
 * a direct register read with no timing subtlety. ODET0_N/ODET1_N are
 * active-low; both lines clear together means a sleeve is present.
 */
static int samcop_sleeve_get_option_detect(void *dev, int *result)
{
	struct samcop_sleeve *eps = dev;
	u32 raw, val;

	raw = samcop_sleeve_read(eps, SAMCOP_PCMCIA_EPS);
	val = raw & (SAMCOP_PCMCIA_EPS_ODET0_N | SAMCOP_PCMCIA_EPS_ODET1_N);

	if (result)
		*result = (val == 0);

	return 0;
}

/*
 * ipaq_sleeve_ops::control_egpio -- the four lines enum ipaq_sleeve_egpio
 * (include/linux/ipaq-sleeve.h) carries. OPT_NVRAM_ON/OPT_ON/OPT_RESET
 * are gpiod consumers (see this file's header comment); CARD_RESET is
 * the one real EPS register write here, SAMCOP_PCMCIA_CC_RESET,
 * unchanged from the historical driver's own samcop_eps_control_egpio().
 */
static int samcop_sleeve_control_egpio(void *dev, int egpio, int state)
{
	struct samcop_sleeve *eps = dev;

	switch (egpio) {
	case IPAQ_EGPIO_OPT_NVRAM_ON:
		gpiod_set_value_cansleep(eps->nvram_gpio, state);
		break;
	case IPAQ_EGPIO_OPT_ON:
		if (eps->opt_on_gpio)
			gpiod_set_value_cansleep(eps->opt_on_gpio, state);
		break;
	case IPAQ_EGPIO_OPT_RESET:
		gpiod_set_value_cansleep(eps->opt_reset_gpio, state);
		break;
	case IPAQ_EGPIO_CARD_RESET:
		samcop_sleeve_write(eps, SAMCOP_PCMCIA_CC,
				    state ? SAMCOP_PCMCIA_CC_RESET : 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * ipaq_sleeve_ops::read_egpio -- historical samcop_eps_read_egpio(),
 * lines 150-166. The four register/mask pairs are transcribed
 * bit-for-bit; the default arm is not, and deliberately so: the original
 * printk()ed "unknown ipaq_egpio_type" and returned 0, which is
 * indistinguishable from a genuine low reading of a line that is low. This
 * returns -EINVAL, so a caller asking for something this backend does not
 * have gets an error rather than a plausible answer.
 *
 * Returns the RAW active-low EPS bit (non-zero == line high), not a
 * logical "asserted" value; the consumer inverts. See
 * include/linux/ipaq-sleeve.h's own enum comment.
 *
 * The four OPT_ON/OPT_NVRAM_ON/OPT_RESET/CARD_RESET members of the enum
 * are deliberately absent from this switch: three are gpiod outputs and
 * the fourth is a write-only control register, none of them readable
 * status. That matches the historical driver, whose switch carried
 * exactly these four PCMCIA cases and printed an "unknown
 * ipaq_egpio_type" for everything else.
 *
 * IPAQ_EGPIO_OPT_BAT_FAULT is the exception to the raw-bit rule, as the
 * enum's comment requires: it returns 1 for an asserted fault. EPS bit 6,
 * BATT_FLT, is read as active-high. That is inferred, not observed: it
 * is the one EPS status bit whose name carries no _N, and the same
 * connector signal reads high for a fault on h38xx, where the WinCE
 * keyboard driver's own test is "GPLR bit 22 set".
 */
static int samcop_sleeve_read_egpio(void *dev, int nr)
{
	struct samcop_sleeve *eps = dev;
	u32 eps_raw, mask;

	eps_raw = samcop_sleeve_read(eps, SAMCOP_PCMCIA_EPS);

	switch (nr) {
	case IPAQ_EGPIO_OPT_BAT_FAULT:
		return !!(eps_raw & SAMCOP_PCMCIA_EPS_BATT_FLT);
	case IPAQ_EGPIO_PCMCIA_CD0_N:
		mask = SAMCOP_PCMCIA_EPS_CD0_N;
		break;
	case IPAQ_EGPIO_PCMCIA_CD1_N:
		mask = SAMCOP_PCMCIA_EPS_CD1_N;
		break;
	case IPAQ_EGPIO_PCMCIA_IRQ0:
		mask = SAMCOP_PCMCIA_EPS_IRQ0_N;
		break;
	case IPAQ_EGPIO_PCMCIA_IRQ1:
		mask = SAMCOP_PCMCIA_EPS_IRQ1_N;
		break;
	default:
		return -EINVAL;
	}

	return eps_raw & mask;
}

/*
 * ipaq_sleeve_ops::egpio_irq -- historical samcop_eps_egpio_irq(),
 * lines 168-206. That version indexed a table of raw SAMCOP hwirq
 * offsets and added the chip's irq_base by hand; here the MFD core has
 * already translated each hwirq through SAMCOP's own irq_domain, so
 * these are finished Linux virqs straight out of platform_get_irq().
 * The machine_is_h5400() gate is dropped -- see this file's header
 * comment.
 */
static int samcop_sleeve_egpio_irq(void *dev, int nr)
{
	struct samcop_sleeve *eps = dev;

	switch (nr) {
	case IPAQ_EGPIO_PCMCIA_CD0_N:
		return eps->irq_cd0;
	case IPAQ_EGPIO_PCMCIA_CD1_N:
		return eps->irq_cd1;
	case IPAQ_EGPIO_PCMCIA_IRQ0:
		return eps->irq_irq0;
	case IPAQ_EGPIO_PCMCIA_IRQ1:
		return eps->irq_irq1;
	/*
	 * EPS itself has no option-interrupt source -- SAMCOP_PCMCIA_EPS
	 * carries only CD0/CD1, IRQ0/IRQ1, ODET and BATT_FAULT -- but the
	 * *board*, like h39xx, routes a dedicated OPT_INT to its own SoC
	 * pin (a PXA GPIO) rather than reusing an EPS source. Resolved as
	 * the "opt-int" gpiod in probe, the way
	 * drivers/misc/asic2-sleeve.c's asic2_sleeve_egpio_irq() does it;
	 * handing out irq_irq0 instead would share a card IRQ with whatever
	 * PCMCIA device is in the socket.
	 */
	case IPAQ_EGPIO_OPT_INT:
		return eps->irq_opt_int;
	default:
		return -EINVAL;
	}
}

/*
 * ipaq_sleeve_ops::rearm_cd_edge -- CD0/CD1 need a dynamic edge, not just
 * the correct initial one probe() programs: SAMCOP_PCMCIA_IM packs a 2-bit
 * edge-select field per EPS source (1=falling/2=rising/3=both, see
 * SAMCOP_PCMCIA_IM_DEFAULT's own comment), CD0 at bits[1:0], CD1 at
 * bits[3:2] -- the same source ordering as SAMCOP_PCMCIA_EPS's own
 * CD0_N/CD1_N/IRQ0_N/IRQ1_N bit layout. A real WinCE decompile+verify pass
 * over the factory pcmcia.dll disassembly-confirmed the stock driver
 * reprograms this same field to the OPPOSITE edge on every
 * card-request/release call:
 * card present -> arm for removal (rising), card absent -> arm for
 * insertion (falling). Without this, probe()'s one-time
 * SAMCOP_PCMCIA_IM_DEFAULT write leaves both fields armed for insertion
 * only (0x25a5 decodes to falling-only for both CD0 and CD1), so an
 * eject after the first insertion never generates an interrupt again --
 * confirmed real, not just suspected, by that decompile pass.
 *
 * CD0's field and CD1's field are different bits of the same 32-bit
 * register, and each socket re-arms its own on every poll of its own state,
 * so the read-modify-write below is taken under eps->im_lock: without it
 * two sockets rearming concurrently is the textbook lost update, and the
 * update that gets lost is the edge programming whose absence this function
 * exists to prevent in the first place.
 */
static int samcop_sleeve_rearm_cd_edge(void *dev, int nr, bool card_present)
{
	struct samcop_sleeve *eps = dev;
	unsigned long flags;
	unsigned int shift;
	u32 im;

	switch (nr) {
	case IPAQ_EGPIO_PCMCIA_CD0_N:
		shift = 0;
		break;
	case IPAQ_EGPIO_PCMCIA_CD1_N:
		shift = 2;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&eps->im_lock, flags);
	im = samcop_sleeve_read(eps, SAMCOP_PCMCIA_IM);
	im &= ~(0x3 << shift);
	im |= (card_present ? 0x2 : 0x1) << shift;
	samcop_sleeve_write(eps, SAMCOP_PCMCIA_IM, im);
	raw_spin_unlock_irqrestore(&eps->im_lock, flags);

	return 0;
}

static struct ipaq_sleeve_ops samcop_sleeve_ops = {
	.spi_read		= samcop_sleeve_spi_read,
	.get_option_detect	= samcop_sleeve_get_option_detect,
	.control_egpio		= samcop_sleeve_control_egpio,
	.read_egpio		= samcop_sleeve_read_egpio,
	.egpio_irq		= samcop_sleeve_egpio_irq,
	.rearm_cd_edge		= samcop_sleeve_rearm_cd_edge,
};

static void samcop_sleeve_ssp_free(void *ssp)
{
	pxa_ssp_free(ssp);
}

static void samcop_sleeve_spi_remove(void *data)
{
	struct device *dev = data;

	pm_runtime_get_sync(dev);
	pxa2xx_spi_remove(dev);
	pm_runtime_put_noidle(dev);
}

static void samcop_sleeve_spi_unregister(void *spi)
{
	spi_unregister_device(spi);
}

/*
 * Borrow ssp1 through this node's own "samsung,sleeve-port" phandle and
 * register a real spi_controller against it, then allocate and add the
 * ID-EEPROM spi_device by hand -- see this file's header comment for the
 * full reasoning (why a plain of_find_spi_controller_by_node() on ssp1
 * cannot work here, and why the port phandle + pxa_ssp_request_of() +
 * pxa2xx_spi_probe() is the same shape sound/soc/pxa/pxa-ssp.c already
 * uses, not a new pattern).
 *
 * Returns -EPROBE_DEFER if ssp1's own drivers/soc/pxa/ssp.c backend has
 * not registered the raw ssp_device yet -- pxa_ssp_request_of() cannot
 * distinguish "not yet" from "never will", so every failure here is
 * treated as transient and left to the driver core to retry, matching
 * spi-pxa2xx-platform.c's own identical inability to tell the two
 * apart in pxa2xx_spi_ssp_request().
 *
 * Every step below and in samcop_sleeve_probe() is undone by a devres
 * action, so teardown runs in exact reverse order on both a failed probe
 * and unbind. That keeps the controller registered until the SPI device
 * is gone and unregisters it before devres drops the controller memory
 * pxa2xx_spi_probe() allocated. It also means this driver never sets its
 * own drvdata: pxa2xx_spi_probe() stores its driver_data there, and
 * pxa2xx_spi_remove() reads it back.
 */
static int samcop_sleeve_add_spi_device(struct samcop_sleeve *eps)
{
	struct device_node *ssp_np;
	struct spi_controller *ctlr;
	struct ssp_device *ssp;
	struct spi_device *spi;
	int ret;

	ssp_np = of_parse_phandle(eps->dev->of_node, "samsung,sleeve-port", 0);
	if (!ssp_np)
		return dev_err_probe(eps->dev, -ENODEV,
				     "no 'samsung,sleeve-port' phandle\n");

	ssp = pxa_ssp_request_of(ssp_np, "samcop-sleeve");
	of_node_put(ssp_np);
	if (!ssp)
		return dev_err_probe(eps->dev, -EPROBE_DEFER,
				     "ssp1 not registered yet\n");

	ret = devm_add_action_or_reset(eps->dev, samcop_sleeve_ssp_free, ssp);
	if (ret)
		return ret;

	/*
	 * pxa2xx_spi_probe() itself never touches runtime PM -- in the
	 * native spi-pxa2xx-platform.c path that is pxa2xx_spi_platform_probe()'s
	 * job, done to its own &pdev->dev before calling into the shared
	 * core. eps->dev is that same "own device" here, so it needs the
	 * identical sequence, or __spi_setup()'s auto_runtime_pm path
	 * (unconditionally armed by pxa2xx_spi_probe() via
	 * controller->set_cs/auto_runtime_pm) calls
	 * pm_runtime_resume_and_get() against a device whose
	 * power.disable_depth is still the default 1, and
	 * rpm_resume() returns -EACCES for that, which shows up as
	 * "spi_master spi0: Failed to power device: -13" and "can't setup
	 * spi0.0, status -13".
	 */
	pm_runtime_set_autosuspend_delay(eps->dev, 50);
	pm_runtime_use_autosuspend(eps->dev);
	pm_runtime_set_active(eps->dev);
	ret = devm_pm_runtime_enable(eps->dev);
	if (ret)
		return ret;

	eps->spi_info.num_chipselect = 1;
	ret = pxa2xx_spi_probe(eps->dev, ssp, &eps->spi_info);
	if (ret)
		return dev_err_probe(eps->dev, ret,
				     "failed to register ssp1 as a spi controller\n");

	ret = devm_add_action_or_reset(eps->dev, samcop_sleeve_spi_remove,
				       eps->dev);
	if (ret)
		return ret;

	/*
	 * pxa2xx_spi_probe() was handed eps->dev as the controller's own
	 * parent, so spi_register_controller() (drivers/spi/spi.c's
	 * device_set_node(&ctlr->dev, dev_fwnode(dev))) gave the new
	 * spi_controller this node's own of_node, not ssp1's -- that is
	 * what this lookup finds.
	 */
	ctlr = of_find_spi_controller_by_node(eps->dev->of_node);
	if (!ctlr)
		return dev_err_probe(eps->dev, -ENODEV,
				     "spi controller vanished after registering\n");

	spi = spi_alloc_device(ctlr);
	put_device(&ctlr->dev);
	if (!spi)
		return dev_err_probe(eps->dev, -ENOMEM,
				     "failed to allocate spi device\n");

	/*
	 * Single, fixed chip select (GPIO24 via this node's own cs-gpios) --
	 * see this file's header comment on why __spi_add_device() still
	 * resolves spi->cs_gpiod from that array for a hand-allocated
	 * device with no DT child node of its own.
	 */
	spi_set_chipselect(spi, 0, 0);
	spi->cs_index_mask = BIT(0);
	spi->max_speed_hz = SAMCOP_SLEEVE_SPI_HZ;
	/*
	 * CPOL=0, CPHA=1: WinCE BattMgr.dll programs SSCR1 = 0x10 (SPH set,
	 * SPO clear), matching the historical Linux drivers' SSCR1_SPH.
	 */
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	strscpy(spi->modalias, "samcop-sleeve-eeprom", sizeof(spi->modalias));

	ret = spi_add_device(spi);
	if (ret) {
		spi_dev_put(spi);
		return dev_err_probe(eps->dev, ret, "failed to add spi device\n");
	}

	eps->spi = spi;

	return devm_add_action_or_reset(eps->dev, samcop_sleeve_spi_unregister,
					spi);
}

static void samcop_sleeve_unregister(void *data)
{
	ipaq_sleeve_unregister();
}

static int samcop_sleeve_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samcop_sleeve *eps;
	struct resource *mem;
	unsigned long flags;
	int ret;

	eps = devm_kzalloc(dev, sizeof(*eps), GFP_KERNEL);
	if (!eps)
		return -ENOMEM;

	eps->dev = dev;
	raw_spin_lock_init(&eps->im_lock);

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_ioremap_resource() -- the latter's exclusive
	 * devm_request_mem_region() always loses to drivers/mfd/samcop.c's
	 * own devm_ioremap_resource() of the entire SAMCOP MMIO window
	 * with -EBUSY. Same pattern and same reasoning as
	 * drivers/iio/adc/samcop-adc.c's own probe(); see that driver's
	 * header comment for the full explanation.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	eps->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!eps->regs)
		return -ENOMEM;

	/*
	 * Resource index order is fixed by drivers/mfd/samcop.c's own
	 * samcop_sleeve_resources[]: ODET first (this driver's own presence
	 * interrupt, index 0 since before the CD/IRQ lines existed), then
	 * CD0/CD1/IRQ0/IRQ1 for the socket consumer. All five are separate
	 * hwirqs of SAMCOP's EPS demux tier, already virq-translated by the
	 * MFD core.
	 */
	eps->irq_odet = platform_get_irq(pdev, 0);
	if (eps->irq_odet < 0)
		return dev_err_probe(dev, eps->irq_odet, "failed to get odet irq\n");

	eps->irq_cd0 = platform_get_irq(pdev, 1);
	if (eps->irq_cd0 < 0)
		return dev_err_probe(dev, eps->irq_cd0, "failed to get cd0 irq\n");

	eps->irq_cd1 = platform_get_irq(pdev, 2);
	if (eps->irq_cd1 < 0)
		return dev_err_probe(dev, eps->irq_cd1, "failed to get cd1 irq\n");

	eps->irq_irq0 = platform_get_irq(pdev, 3);
	if (eps->irq_irq0 < 0)
		return dev_err_probe(dev, eps->irq_irq0, "failed to get irq0 irq\n");

	eps->irq_irq1 = platform_get_irq(pdev, 4);
	if (eps->irq_irq1 < 0)
		return dev_err_probe(dev, eps->irq_irq1, "failed to get irq1 irq\n");

	eps->nvram_gpio = devm_gpiod_get(dev, "nvram", GPIOD_OUT_LOW);
	if (IS_ERR(eps->nvram_gpio))
		return dev_err_probe(dev, PTR_ERR(eps->nvram_gpio),
				     "failed to get nvram gpio\n");

	/*
	 * Optional on purpose: requesting GPA17 makes it an output, and on the
	 * h5550 a driven OPT_ON, low or high, leaves the jacket's CF slot
	 * undriven. The pin has to stay in its reset-time mode 2, as under the
	 * factory firmware, so the h5000 DT does not list it.
	 */
	eps->opt_on_gpio = devm_gpiod_get_optional(dev, "opt-on", GPIOD_OUT_LOW);
	if (IS_ERR(eps->opt_on_gpio))
		return dev_err_probe(dev, PTR_ERR(eps->opt_on_gpio),
				     "failed to get opt-on gpio\n");

	eps->opt_reset_gpio = devm_gpiod_get(dev, "opt-reset", GPIOD_OUT_LOW);
	if (IS_ERR(eps->opt_reset_gpio))
		return dev_err_probe(dev, PTR_ERR(eps->opt_reset_gpio),
				     "failed to get opt-reset gpio\n");

	/*
	 * OPT_INT (a PXA GPIO) -- the board's real
	 * sleeve-device interrupt pin; see samcop_sleeve_egpio_irq()'s own
	 * comment for why this replaces sharing irq_irq0. Optional in
	 * exactly the sense drivers/misc/asic2-sleeve.c's own probe()
	 * treats its identical "opt-int" line: nothing below this point --
	 * the SPI EEPROM device, the EPS interrupt-mask programming, the
	 * bus registration -- may fail because this one non-essential line
	 * could not be resolved. -EPROBE_DEFER is still propagated so the
	 * core retries the whole probe; every other failure (absent
	 * property, no irq behind the gpiod, a rejected trigger type)
	 * degrades to "no OPT_INT source" instead, matching
	 * asic2_sleeve_egpio_irq()'s own -ENXIO-on-absence contract that
	 * IPAQ_EGPIO_OPT_INT consumers (drivers/misc/navman-gps-sleeve.c)
	 * already have to handle for boards that don't wire this pin.
	 */
	eps->opt_int_gpio = devm_gpiod_get_optional(dev, "opt-int", GPIOD_IN);
	if (IS_ERR(eps->opt_int_gpio)) {
		ret = PTR_ERR(eps->opt_int_gpio);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev,
			 "failed to get opt-int gpio (%d); continuing with no OPT_INT source\n",
			 ret);
		eps->opt_int_gpio = NULL;
	}

	eps->irq_opt_int = -ENXIO;
	if (eps->opt_int_gpio) {
		int opt_int_irq = gpiod_to_irq(eps->opt_int_gpio);

		if (opt_int_irq < 0) {
			dev_warn(dev,
				 "opt-int gpio has no irq (%d); continuing with no OPT_INT source\n",
				 opt_int_irq);
			goto opt_int_done;
		}

		/*
		 * Rising edge, the same convention the h39xx port uses for the
		 * identical connector signal. PXA GPIO interrupts are
		 * edge-only silicon (pxa_gpio_irq_type() implements no level
		 * modes), so no level type is available to choose instead,
		 * and an irq derived from a gpiod carries no trigger type
		 * until something sets one.
		 */
		ret = irq_set_irq_type(opt_int_irq, IRQ_TYPE_EDGE_RISING);
		if (ret) {
			dev_warn(dev,
				 "cannot set opt-int irq trigger (%d); continuing with no OPT_INT source\n",
				 ret);
			goto opt_int_done;
		}

		eps->irq_opt_int = opt_int_irq;
	}
opt_int_done:

	ret = samcop_sleeve_add_spi_device(eps);
	if (ret)
		return ret;

	/*
	 * Edge-type config for the seven EPS sources, programmed verbatim --
	 * see include/linux/mfd/samcop.h's SAMCOP_PCMCIA_IM_DEFAULT comment
	 * for where this value deviates from the silicon reset default. IC
	 * (per-line IRQ enable) is
	 * not touched here: SAMCOP's own EPS irq_chip (drivers/mfd/samcop.c)
	 * unmasks it automatically the moment request_irq() below is called
	 * on the translated virq, the same way every other hwirq in this
	 * domain works.
	 *
	 * Under eps->im_lock even though no socket exists yet to contend
	 * with: every writer of this register holding the lock is easier to
	 * keep true than an argument about which writers are ordered.
	 */
	raw_spin_lock_irqsave(&eps->im_lock, flags);
	samcop_sleeve_write(eps, SAMCOP_PCMCIA_IM, SAMCOP_PCMCIA_IM_DEFAULT);
	raw_spin_unlock_irqrestore(&eps->im_lock, flags);

	ret = ipaq_sleeve_register(&samcop_sleeve_ops, eps);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register sleeve bus backend\n");

	ret = devm_add_action_or_reset(dev, samcop_sleeve_unregister, NULL);
	if (ret)
		return ret;

	/*
	 * Requested after the bus registration so that devres frees it
	 * before ipaq_sleeve_unregister(): a presence IRQ firing during that
	 * function's cancel_delayed_work_sync()/eject() would otherwise
	 * schedule new work on an already-unregistered backend. Same
	 * ordering the historical driver's own samcop_eps_cleanup() used
	 * (free_irq() before ipaq_sleeve_unregister()).
	 */
	ret = devm_request_irq(dev, eps->irq_odet,
			       ipaq_sleeve_presence_interrupt, 0,
			       "samcop-sleeve-odet", NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request option-detect irq\n");

	/*
	 * EPS BATT_FLT, SAMCOP_IRQ_EPS_BATT_FAULT. Its edge field in
	 * SAMCOP_PCMCIA_IM is programmed for both edges by
	 * SAMCOP_PCMCIA_IM_DEFAULT above, so the bus core sees a fault and
	 * its clearing. Optional in the same sense as the ASIC2 backend's
	 * bat-fault line: without it the insertion-time check still reads
	 * the EPS bit, and only a fault on an already enumerated jacket goes
	 * unseen. The hwirq has no cell resource, so it is mapped through
	 * the parent SAMCOP's own domain.
	 */
	eps->irq_bat_fault = samcop_hwirq_to_irq(dev->parent,
						 SAMCOP_IRQ_EPS_BATT_FAULT);
	if (eps->irq_bat_fault >= 0) {
		ret = devm_request_irq(dev, eps->irq_bat_fault,
				       ipaq_sleeve_bat_fault_interrupt, 0,
				       "samcop-sleeve-bat-fault", NULL);
		if (ret) {
			dev_warn(dev,
				 "failed to request battery fault irq (%d); battery fault checked at insertion only\n",
				 ret);
			eps->irq_bat_fault = -ENXIO;
		}
	} else {
		dev_warn(dev,
			 "no battery fault irq (%d); battery fault checked at insertion only\n",
			 eps->irq_bat_fault);
	}

	return 0;
}

static struct platform_driver samcop_sleeve_driver = {
	.driver = {
		.name = "samcop-sleeve",
	},
	.probe	= samcop_sleeve_probe,
};
module_platform_driver(samcop_sleeve_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("SAMCOP EPS expansion-pack (sleeve) backend (HP iPAQ h5xxx)");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SPI_PXA2xx");
