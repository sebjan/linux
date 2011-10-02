/*
 * OMAP Temperature sensor header file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy <at> ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __ARCH_ARM_PLAT_OMAP_INCLUDE_PLAT_TEMPERATURE_SENSOR_H
#define __ARCH_ARM_PLAT_OMAP_INCLUDE_PLAT_TEMPERATURE_SENSOR_H

/* Offsets from the base of temperature sensor registers */

#define OMAP4460_TEMP_SENSOR_CTRL_OFFSET	0x00
#define OMAP4460_BGAP_CTRL_OFFSET		0x4c
#define OMAP4460_BGAP_COUNTER_OFFSET		0x50
#define OMAP4460_BGAP_THRESHOLD_OFFSET		0x54
#define OMAP4460_BGAP_TSHUT_OFFSET		0x58
#define OMAP4460_BGAP_STATUS_OFFSET		0x5c
#define OMAP4460_FUSE_OPP_BGAP			-0xcc

/*
 * The register offsets and but fields might change across
 * OMAP versions hence populating them in this structure.
 */
struct omap_temp_sensor_registers {
	u32	temp_sensor_ctrl;
	u32	bgap_tempsoff_mask;
	u32	bgap_soc_mask;
	u32	bgap_eocz_mask;
	u32	bgap_dtemp_mask;

	u32	bgap_mask_ctrl;
	u32	mask_hot_mask;
	u32	mask_cold_mask;

	u32	bgap_mode_ctrl;
	u32	mode_ctrl_mask;

	u32	bgap_counter;
	u32	counter_mask;

	u32	bgap_threshold;
	u32	threshold_thot_mask;
	u32	threshold_tcold_mask;

	u32	thsut_threshold;
	u32	tshut_hot_mask;
	u32	tshut_cold_mask;

	u32	bgap_status;
	u32	status_clean_stop_mask;
	u32	status_bgap_alert_mask;
	u32	status_hot_mask;
	u32	status_cold_mask;

	u32	bgap_efuse;
};

/* @name: Name of the domain of the temperature sensor */
struct omap_temp_sensor_dev_attr {
	const char *name;
};

extern struct omap_temp_sensor_registers omap_mpu_temp_sensor_registers;

/*
 * omap_temp_sensor platform data
 * @registers - pointer to register set and thier bit fields information
 * @min_freq - The minimum frequency for temp sensor to be operational
 * @max_freq - The maximum frequency at which temp sensor is operational
 */
struct omap_temp_sensor_pdata {
	struct omap_temp_sensor_registers *registers;
	u32 min_freq;
	u32 max_freq;
};

#endif
