/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CUST_PE_H_
#define _CUST_PE_H_

/* Pump Express support (fast charging) */
#define TA_START_BATTERY_SOC	1
#define TA_STOP_BATTERY_SOC	85
//CEI comment start
#if 0 //MTK ORG
#define TA_AC_12V_INPUT_CURRENT CHARGE_CURRENT_3200_00_MA
#define TA_AC_9V_INPUT_CURRENT	CHARGE_CURRENT_3200_00_MA
#define TA_AC_7V_INPUT_CURRENT	CHARGE_CURRENT_3200_00_MA
#define TA_AC_CHARGING_CURRENT	CHARGE_CURRENT_3200_00_MA
#else
#define TA_AC_12V_INPUT_CURRENT CHARGE_CURRENT_1250_00_MA
#define TA_AC_9V_INPUT_CURRENT	CHARGE_CURRENT_1500_00_MA
#define TA_AC_7V_INPUT_CURRENT	CHARGE_CURRENT_1600_00_MA
#define TA_AC_CHARGING_CURRENT	CHARGE_CURRENT_1856_00_MA // change from CHARGE_CURRENT_2300_00_MA to CHARGE_CURRENT_1856_00_MA#endif
#endif

#define TA_9V_SUPPORT
#define TA_12V_SUPPORT

/* Ichg threshold for leaving PE+/PE+20 */
#define PEP20_ICHG_LEAVE_THRESHOLD 1000 /* mA */
#define PEP_ICHG_LEAVE_THRESHOLD 1000 /* mA */

/* For PE+20, VBUS V.S. VBAT table, according to BQ25896 */
#define VBAT3400_VBUS CHR_VOLT_08_000000_V
#define VBAT3500_VBUS CHR_VOLT_08_500000_V
#define VBAT3600_VBUS CHR_VOLT_08_500000_V
#define VBAT3700_VBUS CHR_VOLT_09_000000_V
#define VBAT3800_VBUS CHR_VOLT_09_000000_V
#define VBAT3900_VBUS CHR_VOLT_09_000000_V
#define VBAT4000_VBUS CHR_VOLT_09_500000_V
#define VBAT4100_VBUS CHR_VOLT_09_500000_V
#define VBAT4200_VBUS CHR_VOLT_10_000000_V
#define VBAT4300_VBUS CHR_VOLT_10_000000_V

#endif /* _CUST_PE_H_ */
