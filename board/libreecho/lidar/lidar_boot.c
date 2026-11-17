// SPDX-License-Identifier: GPL-2.0+
/*
 * RAM boot of an Amazon-LK Android boot image for "lidar".
 *
 * fastboot downloads an image built for the stock LK (Android boot image v0,
 * kernel field = 512-byte MediaTek KERNEL header + gzip(arm64 Image) +
 * appended DTBs, see experiments/phase-04/pack_boot.py).  U-Boot cannot use
 * bootm on that layout: the kernel field does not start with a gzip or uImage
 * header.  This command applies the same interpretation LK applies, then
 * hands the pieces to booti.  Nothing is written to storage.
 */

#include <command.h>
#include <env.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <gzip.h>
#include <mapmem.h>
#include <malloc.h>
#include <asm/global_data.h>
#include <android_image.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/log2.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <vsprintf.h>

DECLARE_GLOBAL_DATA_PTR;

#define LIDAR_MTK_MAGIC		0x58881688
#define LIDAR_MTK_HDR_SIZE	512
#define LIDAR_TAGS_ADDR		0x4e000000UL
#define LIDAR_DTB_MAX_SIZE	0x80000UL
#define LIDAR_MAX_DTBS		32
#define LIDAR_MAX_IMAGE_SIZE	SZ_64M

struct lidar_dtb {
	ulong addr;
	u32 size;
};

static bool lidar_fdt_at(ulong addr, ulong end)
{
	const struct fdt_header *h = (const struct fdt_header *)addr;
	u32 size;

	if (end - addr < sizeof(*h))
		return false;
	if (fdt32_to_cpu(h->magic) != FDT_MAGIC)
		return false;
	size = fdt32_to_cpu(h->totalsize);
	if (size < sizeof(*h) || size > LIDAR_DTB_MAX_SIZE)
		return false;
	return addr + size == end;
}

/*
 * The DTBs follow the gzip stream with no padding.  gunzip() does not report
 * how many compressed bytes it consumed, so find the chain from the end: each
 * DTB ends exactly where the next one (or the kernel field) ends.
 */
static int lidar_collect_dtbs(ulong start, ulong end, struct lidar_dtb *dtbs)
{
	int n = 0;

	while (n < LIDAR_MAX_DTBS && end > start + sizeof(struct fdt_header)) {
		ulong lo = end > LIDAR_DTB_MAX_SIZE ?
			   max(start, end - LIDAR_DTB_MAX_SIZE) : start;
		ulong p;
		bool found = false;

		for (p = end - sizeof(struct fdt_header); p >= lo; p--) {
			if (lidar_fdt_at(p, end)) {
				found = true;
				break;
			}
			if (p == 0)
				break;
		}
		if (!found)
			break;
		dtbs[n].addr = p;
		dtbs[n].size = fdt32_to_cpu(
			((const struct fdt_header *)p)->totalsize);
		n++;
		end = p;
	}

	return n;
}

static const char *lidar_root_compatible(const void *fdt)
{
	int node = fdt_path_offset(fdt, "/");
	int len;
	const char *compat;

	if (node < 0)
		return NULL;
	compat = fdt_getprop(fdt, node, "compatible", &len);
	if (!compat || len <= 0)
		return NULL;

	return compat;
}

/*
 * libfdt refuses trees that are not 8-byte aligned (FDT_ERR_ALIGNMENT), and
 * appended DTBs follow an odd-length gzip stream, so read each candidate from
 * an aligned copy.  No silent fallback to an arbitrary DTB: a stock tree boots
 * far enough to look like a driver failure.
 */
static const struct lidar_dtb *lidar_pick_dtb(struct lidar_dtb *dtbs, int n)
{
	const struct lidar_dtb *fallback = NULL;
	int i;

	for (i = 0; i < n; i++) {
		void *fdt = memalign(8, dtbs[i].size);
		const char *compat;
		bool pvt, lidar;

		if (!fdt)
			return NULL;
		memcpy(fdt, (const void *)dtbs[i].addr, dtbs[i].size);
		compat = lidar_root_compatible(fdt);
		pvt = compat && !strcmp(compat, "amzn,lidar-pvt");
		lidar = compat && !strncmp(compat, "amzn,lidar", 10);
		free(fdt);

		if (pvt)
			return &dtbs[i];
		if (lidar && !fallback)
			fallback = &dtbs[i];
	}

	return fallback;
}

static int lidar_boot_image(ulong addr)
{
	struct andr_boot_img_hdr_v0 *hdr;
	const struct lidar_dtb *dtb;
	struct lidar_dtb dtbs[LIDAR_MAX_DTBS];
	ulong field, field_end, kaddr, tags;
	ulong comp_len;
	u32 page, ksize, rsize, rdest;
	char cmdline[ANDR_BOOT_ARGS_SIZE + 1];
	char cmd[160];
	int n, ret;

	if (addr < CONFIG_FASTBOOT_BUF_ADDR ||
	    addr >= CONFIG_FASTBOOT_BUF_ADDR + CONFIG_FASTBOOT_BUF_SIZE) {
		pr_err("lidar: image address %#lx is outside the fastboot buffer\n",
		       addr);
		return -EINVAL;
	}

	hdr = map_sysmem(addr, sizeof(*hdr));
	if (memcmp(hdr->magic, ANDR_BOOT_MAGIC, ANDR_BOOT_MAGIC_SIZE)) {
		pr_err("lidar: not an Android boot image\n");
		ret = -EINVAL;
		goto out_hdr;
	}

	page = hdr->page_size;
	if (page < 2048 || page > SZ_64K || !is_power_of_2(page)) {
		pr_err("lidar: bad boot image page size %#x\n", page);
		ret = -EINVAL;
		goto out_hdr;
	}

	ksize = hdr->kernel_size;
	rsize = hdr->ramdisk_size;
	rdest = hdr->ramdisk_addr;
	kaddr = hdr->kernel_addr;
	tags = hdr->tags_addr ? hdr->tags_addr : LIDAR_TAGS_ADDR;

	field = addr + page;
	field_end = field + ksize;
	if (!ksize || field_end > CONFIG_FASTBOOT_BUF_ADDR +
				   CONFIG_FASTBOOT_BUF_SIZE ||
	    field_end > gd->ram_top) {
		pr_err("lidar: bad kernel size %#x\n", ksize);
		ret = -EINVAL;
		goto out_hdr;
	}

	/* The 512-byte MediaTek header covers only the gzip stream. */
	if (*(u32 *)field == LIDAR_MTK_MAGIC) {
		if (ksize <= LIDAR_MTK_HDR_SIZE) {
			pr_err("lidar: truncated MediaTek kernel header\n");
			ret = -EINVAL;
			goto out_hdr;
		}
		field += LIDAR_MTK_HDR_SIZE;
		ksize -= LIDAR_MTK_HDR_SIZE;
	}

	if (kaddr < gd->ram_base || kaddr >= gd->ram_top ||
	    rdest >= gd->ram_top || tags >= gd->ram_top) {
		pr_err("lidar: load address outside DRAM\n");
		ret = -EINVAL;
		goto out_hdr;
	}

	comp_len = LIDAR_MAX_IMAGE_SIZE;
	if (kaddr + comp_len > gd->ram_top)
		comp_len = gd->ram_top - kaddr;
	if (rsize && rdest > kaddr && rdest - kaddr < comp_len)
		comp_len = rdest - kaddr;

	if (ksize >= 2 && ((u8 *)field)[0] == 0x1f && ((u8 *)field)[1] == 0x8b) {
		ulong out_len = ksize;

		ret = gunzip((void *)kaddr, (int)comp_len, (u8 *)field,
			     &out_len);
		if (ret) {
			pr_err("lidar: gunzip failed (%d)\n", ret);
			ret = -EINVAL;
			goto out_hdr;
		}
		comp_len = out_len;
	} else {
		if (ksize > comp_len) {
			pr_err("lidar: kernel does not fit at %#lx\n", kaddr);
			ret = -EINVAL;
			goto out_hdr;
		}
		memmove((void *)kaddr, (void *)field, ksize);
		comp_len = ksize;
	}

	if (comp_len < 0x40 || memcmp((void *)(kaddr + 0x38), "ARM\x64", 4)) {
		pr_err("lidar: payload is not an arm64 Image\n");
		ret = -EINVAL;
		goto out_hdr;
	}

	n = lidar_collect_dtbs(field, field_end, dtbs);
	if (!n) {
		pr_err("lidar: no appended DTB in boot image\n");
		ret = -EINVAL;
		goto out_hdr;
	}
	dtb = lidar_pick_dtb(dtbs, n);
	if (!dtb) {
		pr_err("lidar: no amzn,lidar DTB in boot image\n");
		ret = -EINVAL;
		goto out_hdr;
	}
	if (tags + dtb->size > gd->ram_top ||
	    (dtb->size && tags < kaddr + comp_len &&
	     tags + dtb->size > kaddr)) {
		pr_err("lidar: DTB does not fit at %#lx\n", tags);
		ret = -EINVAL;
		goto out_hdr;
	}
	memmove((void *)tags, (void *)dtb->addr, dtb->size);

	if (rsize) {
		ulong rsrc = addr + page + ALIGN((ulong)hdr->kernel_size, page);

		if (rsrc + rsize > CONFIG_FASTBOOT_BUF_ADDR +
				   CONFIG_FASTBOOT_BUF_SIZE ||
		    rdest + rsize > gd->ram_top ||
		    (rdest < kaddr + comp_len && rdest + rsize > kaddr) ||
		    (rdest < tags + dtb->size && rdest + rsize > tags)) {
			pr_err("lidar: ramdisk does not fit at %#x\n", rdest);
			ret = -EINVAL;
			goto out_hdr;
		}
		memmove((void *)(ulong)rdest, (void *)rsrc, rsize);
	}

	memcpy(cmdline, hdr->cmdline, ANDR_BOOT_ARGS_SIZE);
	cmdline[ANDR_BOOT_ARGS_SIZE] = '\0';
	if (cmdline[0])
		env_set("bootargs", cmdline);

	if (rsize)
		snprintf(cmd, sizeof(cmd), "booti 0x%lx 0x%x:0x%x 0x%lx",
			 kaddr, rdest, rsize, tags);
	else
		snprintf(cmd, sizeof(cmd), "booti 0x%lx - 0x%lx", kaddr, tags);

	printf("lidar: booti kernel %#lx (%#lx bytes), dtb %#lx (%#x bytes)\n",
	       kaddr, comp_len, tags, dtb->size);

	unmap_sysmem(hdr);
	return run_command(cmd, 0);

out_hdr:
	unmap_sysmem(hdr);
	return ret;
}

static int do_lidar_bootimg(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	ulong addr;

	if (argc > 2)
		return CMD_RET_USAGE;

	addr = argc > 1 ? hextoul(argv[1], NULL) : (ulong)fastboot_buf_addr;
	if (!addr)
		addr = CONFIG_FASTBOOT_BUF_ADDR;

	return lidar_boot_image(addr) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	lidar_bootimg, 2, 0, do_lidar_bootimg,
	"boot an Amazon-LK Android boot image from RAM",
	"[addr] - parse the v0 boot image at addr (default: fastboot buffer),\n"
	"         decompress its arm64 Image, select the appended DTB, booti"
);
