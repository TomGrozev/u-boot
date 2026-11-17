/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __LIDAR_CRUMBS_H
#define __LIDAR_CRUMBS_H

void lidar_crumb(const char *tag);
void lidar_crumb_value(const char *tag, unsigned long long val);
void lidar_crumb_printf(const char *fmt, ...);

#endif
