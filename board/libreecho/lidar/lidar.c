// SPDX-License-Identifier: GPL-2.0+
/*
 * Amazon Echo Plus (2nd generation), "lidar": board hook.
 *
 * U-Boot is chainloaded by the stock Amazon LK.  LK leaves the MediaTek
 * TOPRGU watchdog running; nothing in this image may write storage, so the
 * only job here is to stop the inherited watchdog and bring up the USB
 * gadget that fastboot uses.
 */

#include <dm.h>
#include <init.h>
#include <time.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/printk.h>

#include "crumbs.h"
#include "lidar_usb.h"

#define MTK_MUSB_BASE		0x11100000
#define MTK_USB_PHY_BASE	0x11110800
#define MTK_TOPCKGEN_BASE	0x10000000

static void lidar_usb_dump(const char *why)
{
	void __iomem *m = (void __iomem *)MTK_MUSB_BASE;
	void __iomem *p = (void __iomem *)MTK_USB_PHY_BASE;
	void __iomem *t = (void __iomem *)MTK_TOPCKGEN_BASE;

	u8 index = readb(m + 0x0e);
	u16 csr0;

	/*
	 * MUSB_INTRUSB/INTRTX/INTRRX are read-to-clear.  Never read them
	 * here: a diagnostic read steals the reset or ep0 interrupt from
	 * the real ISR.  INTRUSBE, INTRTXE and INTRRXE are enable
	 * registers and are safe.
	 */
	writeb(0, m + 0x0e);
	csr0 = readw(m + 0x12);
	writeb(index, m + 0x0e);

	lidar_crumb_printf("@LIDAR USB %s FADDR=%02x POWER=%02x INTRUSBE=%02x DEVCTL=%02x TXE=%04x RXE=%04x CSR0=%04x L1INTM=%08x PHY1a=%02x 20=%02x 21=%02x 22=%02x 68=%02x 6a=%02x 6c=%02x 6d=%02x 6e=%02x TOP20=%08x TOP24=%08x TOP3c=%08x",
			   why, readb(m + 0x00), readb(m + 0x01),
			   readb(m + 0x0b), readb(m + 0x60),
			   readw(m + 0x06), readw(m + 0x08), csr0,
			   readl(m + 0xa4),
			   readb(p + 0x1a), readb(p + 0x20), readb(p + 0x21),
			   readb(p + 0x22), readb(p + 0x68), readb(p + 0x6a),
			   readb(p + 0x6c), readb(p + 0x6d), readb(p + 0x6e),
			   readl(t + 0x20), readl(t + 0x24), readl(t + 0x3c));
}

static int setup_seen;
static int fallback_done;
static int mac_logged;
static ulong registered_at;

static void lidar_phy_dump_words(const char *why)
{
	void __iomem *p = (void __iomem *)MTK_USB_PHY_BASE;
	u32 off;

	for (off = 0; off < 0x80; off += 0x20)
		lidar_crumb_printf("@LIDAR PHY %s %02x: %08x %08x %08x %08x %08x %08x %08x %08x",
				   why, off, readl(p + off), readl(p + off + 4),
				   readl(p + off + 8), readl(p + off + 12),
				   readl(p + off + 16), readl(p + off + 20),
				   readl(p + off + 24), readl(p + off + 28));
}

static void lidar_mac_dump(void)
{
	void __iomem *m = (void __iomem *)MTK_MUSB_BASE;
	u32 off;

	lidar_crumb_printf("@LIDAR MAC 60: %08x %08x %08x %08x %08x %08x %08x %08x",
			   readl(m + 0x60), readl(m + 0x64), readl(m + 0x68),
			   readl(m + 0x6c), readl(m + 0x70), readl(m + 0x74),
			   readl(m + 0x78), readl(m + 0x7c));
	lidar_crumb_printf("@LIDAR MAC a0: %08x %08x %08x",
			   readl(m + 0xa0), readl(m + 0xa4), readl(m + 0xa8));
	for (off = 0x600; off < 0x700; off += 0x20)
		lidar_crumb_printf("@LIDAR MAC %03x: %08x %08x %08x %08x %08x %08x %08x %08x",
				   off, readl(m + off), readl(m + off + 4),
				   readl(m + off + 8), readl(m + off + 12),
				   readl(m + off + 16), readl(m + off + 20),
				   readl(m + off + 24), readl(m + off + 28));
}

static void lidar_usb_reconnect(void)
{
	void __iomem *m = (void __iomem *)MTK_MUSB_BASE;
	u8 power = readb(m + 0x01);

	/*
	 * LK already enumerated as its own address (FADDR=0x23 was seen at
	 * handoff) and left the D+ pullup on.  U-Boot resets the address to
	 * zero, but the host keeps talking to the old address unless it sees
	 * a disconnect.  Drop the pullup long enough for the host to notice,
	 * then attach again with a fresh HS chirp.
	 */
	writeb(power & ~0x40, m + 0x01);
	mdelay(250);
	power = readb(m + 0x01);
	writeb((power | 0x40 | 0x20) & ~0x10, m + 0x01);
	writeb(0, m + 0x00);
	lidar_crumb_value("USB_RECONNECT_POWER", power);
}

/*
 * Called from cmd/fastboot.c once g_dnl_register() has bound and connected
 * the gadget.  The fastboot loop then calls board_fastboot_poll() through
 * the MUSB interrupt entry point.
 */
void board_fastboot_registered(void)
{
	registered_at = get_timer(0);
	lidar_usb_reconnect();
	lidar_usb_dump("REG");
	lidar_phy_dump_words("REG");
	if (!mac_logged) {
		mac_logged = 1;
		lidar_mac_dump();
	}
}

#define MTK_TOPRGU_BASE		0x10007000
#define MTK_WDT_MODE		0x00
#define MTK_WDT_MODE_KEY	(0x22 << 24)
#define MTK_WDT_MODE_EN		BIT(0)
#define MTK_WDT_RST		0x08
#define MTK_WDT_RESTART		0x1971

#if !defined(LIDAR_PHY_BISECT) || !LIDAR_PHY_BISECT
struct lidar_phy_reg {
	u8 off;
	u32 val;
};

/*
 * Kaeru LK's live PHY state while it enumerates at HS on this unit:
 * dumps/20261117-lidar-kaeru-usb-regs/peek.txt.
 */
static const struct lidar_phy_reg lidar_lk_phy[] = {
	{ 0x00, 0x0000144e }, { 0x04, 0x009c5500 }, { 0x08, 0x00000000 },
	{ 0x0c, 0x00000000 }, { 0x10, 0x000a4414 }, { 0x14, 0x00801000 },
	{ 0x18, 0x00100482 }, { 0x1c, 0xc0040000 }, { 0x20, 0x0004001a },
	{ 0x60, 0x00000402 }, { 0x64, 0x00000001 }, { 0x68, 0x02000000 },
	{ 0x6c, 0x00003e2e },
};

static void lidar_usb_lk_golden(void)
{
	void __iomem *p = (void __iomem *)MTK_USB_PHY_BASE;
	unsigned int i;

	lidar_crumb("USB_STAGE_PRE_GOLDEN");
	lidar_phy_dump_words("PRE_GOLD");
	for (i = 0; i < ARRAY_SIZE(lidar_lk_phy); i++)
		writel(lidar_lk_phy[i].val, p + lidar_lk_phy[i].off);
	lidar_usb_reconnect();
	lidar_crumb("USB_STAGE lkgolden");
	lidar_phy_dump_words("GOLD");
}
#endif

/* Cap the log: a wedged host retries SETUP forever and would fill the ring. */
void board_musb_setup(const void *pkt)
{
	const u8 *p = pkt;
	static int logged;

	setup_seen = 1;
	if (logged++ >= 16)
		return;
	lidar_crumb_printf("@LIDAR USB SETUP bm=%02x req=%02x val=%04x idx=%04x len=%04x",
			   p[0], p[1], p[2] | (p[3] << 8),
			   p[4] | (p[5] << 8), p[6] | (p[7] << 8));
}

void board_fastboot_poll(const struct lidar_usb_stats *st)
{
	static ulong last;
	static int wdt_started;
	ulong now = get_timer(0);

	/*
	 * LK's watchdog is left armed (~65 s).  Only start servicing it once
	 * a SETUP has actually arrived, so a failed bring-up still resets
	 * warm and preserves the crumb log for Kaeru.
	 */
	if (setup_seen) {
		if (!wdt_started) {
			wdt_started = 1;
			lidar_crumb("WDT_KICK_START");
		}
		writel(MTK_WDT_RESTART, (void __iomem *)MTK_TOPRGU_BASE +
		       MTK_WDT_RST);
	}

	if (!setup_seen && !fallback_done && registered_at &&
	    now - registered_at >= 20000) {
		fallback_done = 1;
#if defined(LIDAR_PHY_BISECT) && LIDAR_PHY_BISECT
		/*
		 * Bisect build: exactly one fallback, from the Linux PHY
		 * values to LK's.  The golden-table write is compiled out of
		 * this build so a slow host cannot be mistaken for a PHY
		 * that needs LK's whole table.
		 */
		lidar_usb_phy_lk_fallback();
		lidar_usb_reconnect();
#else
		lidar_usb_lk_golden();
#endif
	}

	if (now - last < 1000)
		return;
	last = now;
	lidar_crumb_printf("@LIDAR USB POLL usb=%u(tx=%u rx=%u) rst=%u susp=%u res=%u con=%u dis=%u sof=%u sess=%u verr=%u l1raw=%u(last=%08x) l1act=%u(last=%08x)",
			   st->usb_events, st->tx_events, st->rx_events,
			   st->usb_reset, st->usb_suspend, st->usb_resume,
			   st->usb_connect, st->usb_disconnect, st->usb_sof,
			   st->usb_sessreq, st->usb_vbuserr,
			   st->l1_raw_events, st->l1_last_raw,
			   st->l1_active_events, st->l1_last_active);
	lidar_usb_dump("POLLREG");
}

/* First ep0 interrupts: csr/len/state prove whether SETUP reached the core. */
void board_musb_ep0(u16 csr, u16 len, u8 state, u16 int_tx)
{
	static int logged;

	if (logged++ >= 32)
		return;
	lidar_crumb_printf("@LIDAR USB EP0 csr=%04x len=%u state=%u int_tx=%04x",
			   csr, len, state, int_tx);
}

/*
 * MTK_WDT_MODE is key protected: the write must carry the key in the top
 * byte.  LK arms the watchdog before the jump, and it expires about 65 s
 * later.
 */
static void lidar_stop_lk_wdt(void)
{
	void __iomem *base = (void __iomem *)MTK_TOPRGU_BASE;
	u32 mode = readl(base + MTK_WDT_MODE);

	if (mode & MTK_WDT_MODE_EN) {
		writel((mode & ~MTK_WDT_MODE_EN) | MTK_WDT_MODE_KEY,
		       base + MTK_WDT_MODE);
		pr_info("lidar: disabled LK watchdog (mode %#x)\n", mode);
		lidar_crumb_value("WDT_OFF", mode);
	} else {
		lidar_crumb_value("WDT_ALREADY_OFF", mode);
	}
}

int board_early_init_f(void)
{
	lidar_crumb("EARLY_INIT");

	/*
	 * Diagnostic builds pass -DLIDAR_KEEP_LK_WDT=1 so a hang before USB
	 * comes up still ends in a warm reset, which keeps DRAM and the crumb
	 * log for Kaeru to fetch.
	 */
	if (!IS_ENABLED(LIDAR_KEEP_LK_WDT))
		lidar_stop_lk_wdt();

	return 0;
}

int board_init(void)
{
	lidar_crumb("BOARD_INIT");
	return 0;
}

int board_late_init(void)
{
	struct udevice *dev;
	int ret;

	lidar_crumb("LATE_INIT");
	/* PHY bytes as LK left them, before U-Boot's MUSB probe writes any. */
	lidar_usb_dump("PRE");
	lidar_phy_dump_words("PRE");
	if (IS_ENABLED(CONFIG_USB_GADGET)) {
		ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
		if (ret) {
			pr_err("lidar: USB gadget unavailable (%d)\n", ret);
			lidar_crumb_value("USB_FAIL", ret);
		} else {
			lidar_crumb("USB_OK");
			if (IS_ENABLED(LIDAR_KEEP_LK_WDT) &&
			    !IS_ENABLED(LIDAR_WDT_NEVER_STOP))
				lidar_stop_lk_wdt();
		}
	}

	return 0;
}
