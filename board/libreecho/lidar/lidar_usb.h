/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Diagnostic counters shared between the MTK MUSB glue and the Lidar board
 * code.  The weak hook and this struct exist only for bring-up; neither is
 * meant for upstream.
 */
#ifndef __LIDAR_USB_H__
#define __LIDAR_USB_H__

#include <linux/types.h>

struct lidar_usb_stats {
	u32 usb_events;
	u32 tx_events;
	u32 rx_events;
	u32 l1_raw_events;
	u32 l1_active_events;
	u32 usb_reset;
	u32 usb_suspend;
	u32 usb_resume;
	u32 usb_connect;
	u32 usb_disconnect;
	u32 usb_sof;
	u32 usb_sessreq;
	u32 usb_vbuserr;
	u32 l1_last_raw;
	u32 l1_last_active;
};

void board_fastboot_poll(const struct lidar_usb_stats *st)
	__attribute__((weak));

#if defined(LIDAR_PHY_BISECT) && LIDAR_PHY_BISECT
/* PHY bisect build only: switch the PHY to the Kaeru LK tuning once. */
void lidar_usb_phy_lk_fallback(void);
#endif

#endif
