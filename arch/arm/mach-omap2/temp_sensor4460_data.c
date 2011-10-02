/*
 * OMAP4460 on die Temperature sensor data file
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

#include <linux/slab.h>
#include "control.h"
#include <plat/temperature_sensor.h>

/*
 * OMAP4460 has one instance of thermal sensor for MPU
 * need to describe the individual bit fields
 */
struct omap_temp_sensor_registers omap_mpu_temp_sensor_registers = {
	.temp_sensor_ctrl		= OMAP4460_TEMP_SENSOR_CTRL_OFFSET,
	.bgap_tempsoff_mask		= OMAP4460_BGAP_TEMPSOFF_MASK,
	.bgap_soc_mask			= OMAP4460_BGAP_TEMP_SENSOR_SOC_MASK,
	.bgap_eocz_mask			= OMAP4460_BGAP_TEMP_SENSOR_EOCZ_MASK,
	.bgap_dtemp_mask		= OMAP4460_BGAP_TEMP_SENSOR_DTEMP_MASK,

	.bgap_mask_ctrl			= OMAP4460_BGAP_CTRL_OFFSET,
	.mask_hot_mask			= OMAP4460_MASK_HOT_MASK,
	.mask_cold_mask			= OMAP4460_MASK_COLD_MASK,

	.bgap_mode_ctrl			= OMAP4460_BGAP_CTRL_OFFSET,
	.mode_ctrl_mask			= OMAP4460_SINGLE_MODE_MASK,

	.bgap_counter			= OMAP4460_BGAP_COUNTER_OFFSET,
	.counter_mask			= OMAP4460_COUNTER_MASK,

	.bgap_threshold			= OMAP4460_BGAP_THRESHOLD_OFFSET,
	.threshold_thot_mask		= OMAP4460_T_HOT_MASK,
	.threshold_tcold_mask		= OMAP4460_T_COLD_MASK,

	.thsut_threshold		= OMAP4460_BGAP_TSHUT_OFFSET,
	.tshut_hot_mask			= OMAP4460_TSHUT_HOT_MASK,
	.tshut_cold_mask		= OMAP4460_TSHUT_COLD_MASK,

	.bgap_status			= OMAP4460_BGAP_STATUS_OFFSET,
	.status_clean_stop_mask		= OMAP4460_CLEAN_STOP_MASK,
	.status_bgap_alert_mask		= OMAP4460_BGAP_ALERT_MASK,
	.status_hot_mask		= OMAP4460_HOT_FLAG_MASK,
	.status_cold_mask		= OMAP4460_COLD_FLAG_MASK,

	.bgap_efuse			= OMAP4460_FUSE_OPP_BGAP,
};
