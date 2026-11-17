/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * second-stage U-Boot for the Amazon Echo Plus (2nd generation)
 */

#ifndef __CONFIG_LIDAR_H
#define __CONFIG_LIDAR_H

/*
 * fastboot "boot" hands the downloaded Android boot image to our own parser.
 *
 * lidar_pstore describes the Linux ramoops layout (fs/pstore/ram.c: dmesg
 * zones first, then console, ftrace, pmsg).  The host runs
 * "fastboot oem run:run lidar_pstore", then "fastboot oem run:pstore display
 * console" and reads the result back with "fastboot oem console".
 */
#define CFG_EXTRA_ENV_SETTINGS \
	"fastboot_bootcmd=lidar_bootimg\0" \
	"bootargs=\0" \
	"lidar_pstore=pstore set 0x60000000 0x100000 0x20000 0x80000 0 0x20000 0\0"

#endif
