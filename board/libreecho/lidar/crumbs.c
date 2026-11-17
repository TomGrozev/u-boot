// SPDX-License-Identifier: GPL-2.0+
/*
 * "lidar" RAM breadcrumb log.
 *
 * The stock LK watchdog resets the board 65 s after hand-off, which wipes
 * ordinary console output.  DRAM survives a warm reset and Kaeru can read
 * 0x43810000..+0x60000 back with "fastboot fetch ramoops", so every stage of
 * this U-Boot appends a plain-ASCII line there.
 *
 * Layout, little endian:
 *   0x00 "LUBT" | u32 write_off | u32 marker_seq | u32 version=1
 *   0x10 .. write_off: ASCII, markers look like
 *        "@LIDAR <tag> v=0x<value> pc=0x<pc>"
 *
 * Writes are direct to DRAM and the range is cleaned from the dcache, so the
 * log is valid both before the MMU is on and after it is enabled.
 */

#include <command.h>
#include <cpu_func.h>
#include <asm/system.h>
#include <linux/types.h>
#include <vsprintf.h>

#include "crumbs.h"

#define CRUMB_BASE		0x43810000UL
#define CRUMB_SIZE		0x60000UL
#define CRUMB_HDR_SIZE		16UL
#define CRUMB_MAGIC		"LUBT"
#define CRUMB_VER		1

struct crumb_hdr {
	char magic[4];
	u32 off;
	u32 seq;
	u32 ver;
};

static struct crumb_hdr *crumb_hdr(void)
{
	return (struct crumb_hdr *)CRUMB_BASE;
}

static void crumb_sync(void)
{
	struct crumb_hdr *h = crumb_hdr();
	ulong off = h->off;
	ulong start = CRUMB_BASE & ~63UL;
	ulong end;

	if (off > CRUMB_SIZE)
		off = CRUMB_SIZE;
	end = (CRUMB_BASE + off + 63) & ~63UL;

	asm volatile("dsb sy" ::: "memory");
	flush_dcache_range(start, end);
	asm volatile("dsb sy" ::: "memory");
}

/*
 * Called from pre-relocation code, so it must not depend on .bss being zero:
 * everything is derived from the header in DRAM.
 */
static void crumb_init(void)
{
	struct crumb_hdr *h = crumb_hdr();
	int valid = h->magic[0] == 'L' && h->magic[1] == 'U' &&
		     h->magic[2] == 'B' && h->magic[3] == 'T' &&
		     h->off >= CRUMB_HDR_SIZE && h->off <= CRUMB_SIZE &&
		     h->ver == CRUMB_VER;

	if (valid)
		return;

	h->magic[0] = 'L';
	h->magic[1] = 'U';
	h->magic[2] = 'B';
	h->magic[3] = 'T';
	h->off = CRUMB_HDR_SIZE;
	h->seq = 0;
	h->ver = CRUMB_VER;
	crumb_sync();
}

static void crumb_write(const char *s, ulong len)
{
	struct crumb_hdr *h = crumb_hdr();
	ulong off;

	crumb_init();
	off = h->off;
	while (len-- && off < CRUMB_SIZE) {
		*(volatile char *)(CRUMB_BASE + off) = *s++;
		off++;
		h->off = off;
	}
}

static void crumb_puts(const char *s)
{
	const char *p = s;

	while (*p)
		p++;
	crumb_write(s, p - s);
}

static void crumb_hex64(char *dst, u64 val)
{
	int i;

	dst[0] = '0';
	dst[1] = 'x';
	for (i = 15; i >= 0; i--) {
		u32 nibble = (val >> (i * 4)) & 0xf;

		dst[2 + (15 - i)] = nibble > 9 ? 'a' + nibble - 10 : '0' + nibble;
	}
	dst[18] = '\0';
}

void lidar_crumb_value(const char *tag, u64 val)
{
	struct crumb_hdr *h = crumb_hdr();
	u64 pc = (u64)(ulong)__builtin_return_address(0);
	char hex[19];

	crumb_init();
	h->seq++;
	crumb_puts("\n@LIDAR ");
	crumb_puts(tag);
	crumb_puts(" v=");
	crumb_hex64(hex, val);
	crumb_puts(hex);
	crumb_puts(" pc=");
	crumb_hex64(hex, pc);
	crumb_puts(hex);
	crumb_puts("\n");
	crumb_sync();
}

void lidar_crumb(const char *tag)
{
	lidar_crumb_value(tag, 0);
}

void lidar_crumb_printf(const char *fmt, ...)
{
	struct crumb_hdr *h = crumb_hdr();
	char buf[256];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n < 0)
		return;
	if (n > (int)sizeof(buf) - 2)
		n = sizeof(buf) - 2;

	crumb_init();
	h->seq++;
	buf[n++] = '\n';
	{
		ulong old = h->off;

		crumb_write(buf, n);
		/* Only the header and the new line need cleaning: earlier
		 * lines were synced when they were written.  Flushing the
		 * whole growing ring in the fastboot loop made each dump
		 * take about a second and stalled the ISR.
		 */
		asm volatile("dsb sy" ::: "memory");
		flush_dcache_range(CRUMB_BASE & ~63UL,
				   (CRUMB_BASE + CRUMB_HDR_SIZE + 63) & ~63UL);
		flush_dcache_range((CRUMB_BASE + old) & ~63UL,
				   (CRUMB_BASE + h->off + 63) & ~63UL);
		asm volatile("dsb sy" ::: "memory");
	}
}

/* Every console character is mirrored, newline-terminated lines are synced. */
void board_console_mirror(char c)
{
	crumb_write(&c, 1);
	if (c == '\n')
		crumb_sync();
}

/* Direct debug-UART output bypasses console.c; capture it too. */
void board_debug_uart_mirror(int ch)
{
	char c = ch;

	board_console_mirror(c);
}

/*
 * The earliest marker is written by save_boot_params in lowlevel.S, before
 * U-Boot has a stack or MMU.
 */

static int do_lidar_mark(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	if (argc != 2)
		return CMD_RET_USAGE;
	lidar_crumb(argv[1]);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	lidar_mark, 2, 0, do_lidar_mark,
	"append a marker to the RAM breadcrumb log", "<tag>"
);
