/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Earliest "lidar" breadcrumb.  LK branches here with x0 = DTB, before
 * U-Boot has a stack, MMU or caches.  This routine must therefore use only
 * x9-x13 (x0 is preserved for the C entry) and write one cache line.
 */

#include <linux/linkage.h>

#define CRUMB_BASE	0x43810000

ENTRY(save_boot_params)
	ldr	x9, =CRUMB_BASE
	ldr	x10, =0x5442554c		/* "LUBT", little endian */
	str	w10, [x9]
	mov	w10, #1
	str	w10, [x9, #8]			/* marker_seq */
	str	w10, [x9, #12]			/* version */
	adr	x11, 1f
	mov	x12, #16			/* skip header */
2:	ldrb	w13, [x11], #1
	cbz	w13, 3f
	strb	w13, [x9, x12]
	add	x12, x12, #1
	b	2b
3:	str	w12, [x9, #4]			/* write_off */
	dc	civac, x9
	dsb	sy
	b	save_boot_params_ret

1:	.asciz	"\n@LIDAR SAVE_BOOT_PARAMS\n"
ENDPROC(save_boot_params)
