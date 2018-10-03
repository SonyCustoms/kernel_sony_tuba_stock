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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    switch_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
 * Revision:   1.0
 * Modtime:   11 Aug 2005 10:28:16
 * Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter_hal.h>
#include <mach/mt_battery_meter.h>
#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt-plat/mt_boot.h>

#include "mtk_pep_intf.h"
#include "mtk_pep20_intf.h"

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <mach/mt_diso.h>
#endif


/* ============================================================ // */
/* define */
/* ============================================================ // */
/* cut off to full */
#define POST_CHARGING_TIME (30*60)	/* 30mins */
//CEI comment start//
//BAT_status_Full feature
#define FULL_CHECK_TIMES 6 //Doesn't modify because bat thread time is 10 seconds in MT6755
//CEI comment end//

#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define STATUS_FAIL -2


/* ============================================================ // */
/* global variable */
/* ============================================================ // */
unsigned int g_bcct_flag = 0;
unsigned int g_bcct_value = 0;
/*input-output curent distinction*/
unsigned int g_bcct_input_flag = 0;
unsigned int g_bcct_input_value = 0;
unsigned int g_full_check_count = 0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
unsigned int g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited;
#if defined(CONFIG_MTK_HAFG_20)
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
//CEI comment start//
//BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_340000_V;
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_304000_V;
//CEI comment end//
#else
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_200000_V;
#endif
unsigned int get_cv_voltage(void)
{
	return g_cv_voltage;
}
#ifdef CUSTOM_SOFT_CHARGE_30
kal_bool g_sc30_low_cv_voltage = KAL_FALSE;
void set_sc30_low_cv_voltage(kal_bool low_cv)
{
	g_sc30_low_cv_voltage = low_cv;
}
#endif
#endif

//CEI comment start//
extern int board_type_with_hw_id(void);
extern int battery_id_invalid;
extern int g_fg_battery_id;
extern int id_voltage_read;
extern int g_battery_id_voltage[];
//CEI comment end

DEFINE_MUTEX(g_ichg_aicr_access_mutex);
DEFINE_MUTEX(g_aicr_access_mutex);
DEFINE_MUTEX(g_ichg_access_mutex);
unsigned int g_aicr_upper_bound;
static bool g_enable_dynamic_cv = true;

 /* ///////////////////////////////////////////////////////////////////////////////////////// */
 /* // JEITA */
 /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
int g_temp_status = TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag = KAL_TRUE;
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n",
			    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}


unsigned int get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

int mtk_get_dynamic_cv(unsigned int *cv)
{
	int ret = 0;
#ifdef CONFIG_MTK_BIF_SUPPORT
	u32 _cv;
	u32 vbat_bif = 0, vbat_auxadc = 0, vbat = 0;
	u32 retry_cnt = 0;
	u32 ircmp_volt_clamp = 0, ircmp_resistor = 0;

	if (!g_enable_dynamic_cv) {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_340000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;
		goto _out;
	}

	do {
		ret = battery_charging_control(CHARGING_CMD_GET_BIF_VBAT,
			&vbat_bif);
		vbat_auxadc = battery_meter_get_battery_voltage(KAL_TRUE);

		if (ret >= 0 && vbat_bif != 0 && vbat_bif < vbat_auxadc) {
			vbat = vbat_bif;
			battery_log(BAT_LOG_CRTI,
				"%s: use BIF vbat = %dmV, dV to auxadc = %dmV\n",
				__func__, vbat, vbat_auxadc - vbat_bif);
			break;
		}

		retry_cnt++;
	} while (retry_cnt < 5);

	if (retry_cnt == 5) {
		ret = 0;
		vbat = vbat_auxadc;
		battery_log(BAT_LOG_CRTI,
			"%s: use AUXADC vbat = %dmV, since BIF vbat = %d\n",
			__func__, vbat_auxadc, vbat_bif);
	}

	/* Adjust CV according to the obtained vbat */
	if (vbat >= 3400 && vbat < 4300) {
		_cv = 4550;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);
	} else {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_340000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;

		/* Turn on IR compensation */
		ircmp_volt_clamp = 200;
		ircmp_resistor = 80;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);

		/* Disable dynamic CV */
		g_enable_dynamic_cv = false;
	}

_out:
	*cv = _cv;
	battery_log(BAT_LOG_CRTI, "%s: CV = %dmV, enable dynamic cv = %d\n",
		__func__, _cv, g_enable_dynamic_cv);
#else
	ret = -ENOTSUPP;
#endif
	return ret;
}

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
//CEI comment start//
//JEITA enable
BATTERY_VOLTAGE_ENUM global_cv_voltage = BATTERY_VOLT_04_200000_V;
//CEI comment end//

static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	if (g_temp_status == TEMP_ABOVE_POS_60) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_60) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
		if (batt_cust_data.high_battery_voltage_support)
//CEI comment start//
//JEITA enable: check
			cv_voltage = BATTERY_VOLT_04_304000_V;
//CEI comment end//
		else
			cv_voltage = JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_0_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_NEG_10) {
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;
	PMU_STATUS jeita_status = PMU_STATUS_OK;

	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery Over high Temperature(%d) !!\n\r",
			    TEMP_POS_60_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_60;

		//CEI comment start//
		//JEITA enable
		//return PMU_STATUS_FAIL;
		jeita_status = PMU_STATUS_FAIL;
		//CEI comment end//
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD) {	/* control 45c to normal behavior */
		if ((g_temp_status == TEMP_ABOVE_POS_60)
		    && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_POS_60_THRES_MINUS_X_DEGREE, TEMP_POS_60_THRESHOLD);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_60;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_60)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_0_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
				    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_0_THRESHOLD) {
		if ((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD);
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_0_THRES_PLUS_X_DEGREE);
				//CEI comment start//
				//JEITA enable
				//return PMU_STATUS_FAIL;
				jeita_status = PMU_STATUS_FAIL;
				//CEI comment end//
			}
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_0_TO_POS_10;
		}
	} else if (BMT_status.temperature >= TEMP_NEG_10_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			g_temp_status = TEMP_NEG_10_TO_POS_0;
		}
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery below low Temperature(%d) !!\n\r",
			    TEMP_NEG_10_THRESHOLD);
		g_temp_status = TEMP_BELOW_NEG_10;

		jeita_status = PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (g_temp_status != TEMP_POS_10_TO_POS_45) {
		cv_voltage = select_jeita_cv();

//CEI comment start//
//JEITA enable, Alien_battery
	if(battery_id_invalid)
	{
		if (g_temp_status >= TEMP_POS_45_TO_POS_60)
			cv_voltage = BATTERY_VOLT_04_000000_V;
		else
			cv_voltage = BATTERY_VOLT_04_200000_V;

		battery_log(BAT_LOG_CRTI,
			"[BATTERY]LE(K)=> [ALIEN] do_jeita_state_machine(), cv_voltage=%d, g_temp_status=%d\n", cv_voltage, g_temp_status);
	}

	battery_log(BAT_LOG_CRTI,
	    "[BATTERY]LE(K)=> [JEITA] battery_id_invalid=%d, g_temp_status=%d, BMT_status.temp=%d, jeita_status=%s\n", battery_id_invalid, g_temp_status, BMT_status.temperature, (jeita_status == PMU_STATUS_OK)? "OK":"NOK");

	battery_log(BAT_LOG_CRTI,
	    "[BATTERY]LE(K)=> [JEITA] SET_CV_VOLTAGE=%d, BATT_ID=%d, battery_id_invalid=%d, id_voltage_read=%d, g_battery_id_voltage=%d %d %d\n", cv_voltage, g_fg_battery_id, battery_id_invalid, id_voltage_read, g_battery_id_voltage[0], g_battery_id_voltage[1], g_battery_id_voltage[2]);

	global_cv_voltage = cv_voltage;
//CEI comment end//

		battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE,
			&cv_voltage);
#if defined(CONFIG_MTK_HAFG_20)
		g_cv_voltage = cv_voltage;
#endif
	}

	return jeita_status;
}


static void set_jeita_charging_current(void)
{
#ifdef CONFIG_USB_IF
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif
//CEI comment start//
//JEITA enable
#if 0
	if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		g_temp_CC_value = CHARGE_CURRENT_350_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\r\n",
			    g_temp_CC_value);
#else
	if((g_temp_status <= TEMP_NEG_10_TO_POS_0) || (g_temp_status == TEMP_ABOVE_POS_60))
    {
        g_temp_CC_value = CHARGE_CURRENT_0_00_MA; //for lowest/highest temp
        battery_log(BAT_LOG_CRTI, "[BATTERY] LE(K)=> [JEITA] set charging current : %d(%d) (case A) \r\n", g_temp_CC_value, g_temp_status);
    }
	else if(( (g_temp_status == TEMP_POS_0_TO_POS_10) || (g_temp_status == TEMP_POS_45_TO_POS_60) ) && (BMT_status.charger_type == STANDARD_CHARGER))
    {
        g_temp_CC_value = CHARGE_CURRENT_675_00_MA; //for lower/higher temp
        battery_log(BAT_LOG_CRTI, "[BATTERY] LE(K)=> [JEITA] set charging current : %d(%d) (case B) \r\n", g_temp_CC_value, g_temp_status);
    }
#endif
//CEI comment end//
}


//CEI comment start//
signed int check_if_jeita_current_limit(void)
{
	int jeita_cur_limit  = -1;

	if((g_temp_status <= TEMP_NEG_10_TO_POS_0) || (g_temp_status == TEMP_ABOVE_POS_60))
	{
		jeita_cur_limit = CHARGE_CURRENT_0_00_MA;
	}
	else if(( (g_temp_status == TEMP_POS_0_TO_POS_10) || (g_temp_status == TEMP_POS_45_TO_POS_60) ) && (BMT_status.charger_type == STANDARD_CHARGER))
		jeita_cur_limit = CHARGE_CURRENT_675_00_MA;
	else
		jeita_cur_limit = CHARGE_CURRENT_1856_00_MA;

	return jeita_cur_limit;
}
//CEI comment end//

#endif /* CONFIG_MTK_JEITA_STANDARD_SUPPORT */

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;

		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

//CEI comments start//
struct chrlimt_table {
        int vchr_thres[4];
        int chrlimt_current[5][3];
};

static struct chrlimt_table chrlimit =
{
{2500, 5500, 7500, 9500},
{
{1001, 501, 251}, //vchr is less than 2.5V
{1000, 500, 250}, //vchr is 5V
{867, 467, 351}, //vchr is 7V
{733, 434, 352}, //vchr is 9V
{600, 400, 353}, //vchr is 12V
},
};

struct bcct_log_table{
	int bcct_value[17];
	unsigned int bcct_count[17];
};

struct bcct_log_table bcct_log =
{
{1001, 1000, 867, 733, 600, 501, 500, 467, 434, 400, 353, 352, 351, 350, 251, 250, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int bcct_level = 0;

int get_chrlimt_bcct_level(int chr_current)
{
        int ret = 0;

        switch (chr_current)
        {
        case 250:
		ret = 3;
                break;
        case 500:
		ret = 2;
                break;
        case 1000:
		ret = 1;
                break;
        }
        return ret;
}

void set_bcct_log(int bcct_now)
{
	int index = 0;

	for(index=0; index<(sizeof(bcct_log.bcct_value)/sizeof(int)); index++)
	{
		if(bcct_now == bcct_log.bcct_value[index])
		{
			if(bcct_log.bcct_count[index] < 94608000)
				bcct_log.bcct_count[index]++;
			break;
		}
	}
}

int bcct_ever = 0;
//CEI comments end//

/*BQ25896 is the first switch chrager separating input and charge current
*/
unsigned int set_chr_input_current_limit(int current_limit)
{
#ifdef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	u32 power_path_enable = 1;
//CEI comments start//
//	CHR_CURRENT_ENUM chr_type_aicr = 0; /* 10uA */
//	CHR_CURRENT_ENUM chr_type_ichg = 0;
//CEI comment end//

	mutex_lock(&g_aicr_access_mutex);
	if (current_limit != -1) {
//CEI comments start//
		bcct_level = get_chrlimt_bcct_level(current_limit);
		g_bcct_input_flag = 1;

		if(bcct_ever < 47304000)
			bcct_ever++;

#if 0 //MTK-ORG
		if (current_limit < 100) {
			/* limit < 100, turn off power path */
			current_limit = 0;
			power_path_enable = 0;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		} else {
			/* Enable power path if it is disabled previously */
			if (g_bcct_input_value == 0) {
				power_path_enable = 1;
				battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
					&power_path_enable);
			}

			switch (BMT_status.charger_type) {
			case STANDARD_HOST:
				chr_type_aicr = batt_cust_data.usb_charger_current;
				break;
			case NONSTANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.non_std_ac_charger_current;
				break;
			case STANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.ac_charger_input_current;
				mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				break;
			case CHARGING_HOST:
				chr_type_aicr = batt_cust_data.charging_host_charger_current;
				break;
			case APPLE_2_1A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_2_1a_charger_current;
				break;
			case APPLE_1_0A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_1_0a_charger_current;
				break;
			case APPLE_0_5A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_0_5a_charger_current;
				break;
			default:
				chr_type_aicr = CHARGE_CURRENT_500_00_MA;
				break;
			}
			chr_type_aicr /= 100;
			if (current_limit > chr_type_aicr)
				current_limit = chr_type_aicr;
		}

		g_bcct_input_value = current_limit;
#endif
                battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] LE(K)=> set_chr_input_current_limit (%d)\r\n", current_limit);
//CEI comments end//
	} else {
		/* Enable power path if it is disabled previously */
		if (g_bcct_input_value == 0) {
			power_path_enable = 1;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		}

		/* Change to default current setting */
		g_bcct_input_flag = 0;
//CEI comments start//
		bcct_level = 0;
//CEI comments end//
	}

//CEI comment start//
	battery_log(BAT_LOG_CRTI,
		"[BATTERY][IUSB] LE(K)=> set_chr_input_current_limit (%d), bcct_level=%d\n", current_limit, bcct_level);
//CEI comment end//

	mutex_unlock(&g_aicr_access_mutex);
	return g_bcct_input_flag;
#else
	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_chr_input_current_limit _NOT_ supported\n");
	return 0;
#endif
}

static void mtk_select_ichg_aicr(void);
unsigned int set_bat_charging_current_limit(int current_limit)
{
	CHR_CURRENT_ENUM chr_type_ichg = 0;
	CHR_CURRENT_ENUM chr_type_aicr = 0;

	mutex_lock(&g_ichg_access_mutex);
	if (current_limit != -1) {
		g_bcct_flag = 1;
		switch (BMT_status.charger_type) {
		case STANDARD_HOST:
			chr_type_ichg = batt_cust_data.usb_charger_current;
			break;
		case NONSTANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.non_std_ac_charger_current;
			break;
		case STANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.ac_charger_current;
			mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			break;
		case CHARGING_HOST:
			chr_type_ichg = batt_cust_data.charging_host_charger_current;
			break;
		case APPLE_2_1A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_2_1a_charger_current;
			break;
		case APPLE_1_0A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_1_0a_charger_current;
			break;
		case APPLE_0_5A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_0_5a_charger_current;
			break;
		default:
			chr_type_ichg = CHARGE_CURRENT_500_00_MA;
			break;
		}
		chr_type_ichg /= 100;
		if (current_limit > chr_type_ichg)
			current_limit = chr_type_ichg;
		g_bcct_value = current_limit;
	} else /* change to default current setting */
		g_bcct_flag = 0;

	mtk_select_ichg_aicr();

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_bat_charging_current_limit (%d)\r\n",
		current_limit);

	mutex_unlock(&g_ichg_access_mutex);
	return g_bcct_flag;
}

int mtk_chr_reset_aicr_upper_bound(void)
{
	g_aicr_upper_bound = 0;
	return 0;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_SET_BOOST_CURRENT_LIMIT,
		&current_limit);

	return ret;
}

int set_chr_enable_otg(unsigned int enable)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_ENABLE_OTG, &enable);

	return ret;
}

int mtk_chr_get_tchr(int *min_temp, int *max_temp)
{
	int ret = 0;
	int temp[2] = {0, 0};

	ret = battery_charging_control(CHARGING_CMD_GET_CHARGER_TEMPERATURE, temp);
	if (ret < 0)
		return ret;

	*min_temp = temp[0];
	*max_temp = temp[1];

	return ret;
}

int mtk_chr_get_soc(unsigned int *soc)
{
	if (BMT_status.SOC < 0) {
		*soc = 0;
		return -ENOTSUPP;
	}

	*soc = BMT_status.SOC;

	return 0;
}

int mtk_chr_get_ui_soc(unsigned int *ui_soc)
{
	/* UI_SOC2 is the one that shows on UI */
	if (BMT_status.UI_SOC2 < 0) {
		*ui_soc = 0;
		return -ENOTSUPP;
	}

	*ui_soc = BMT_status.UI_SOC2;

	return 0;
}

int mtk_chr_get_vbat(unsigned int *vbat)
{
	if (BMT_status.bat_vol < 0) {
		*vbat = 0;
		return -ENOTSUPP;
	}

	*vbat = BMT_status.bat_vol;

	return 0;
}

int mtk_chr_get_ibat(unsigned int *ibat)
{
	*ibat = BMT_status.IBattery / 10;
	return 0;
}

int mtk_chr_get_vbus(unsigned int *vbus)
{
	if (BMT_status.charger_vol < 0) {
		*vbus = 0;
		return -ENOTSUPP;
	}
	*vbus = BMT_status.charger_vol;

	return 0;
}

int mtk_chr_get_aicr(unsigned int *aicr)
{
	int ret = 0;
	u32 _aicr = 0; /* 10uA */

	ret = battery_charging_control(CHARGING_CMD_GET_INPUT_CURRENT, &_aicr);
	*aicr = _aicr / 100;

	return ret;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	*exist = (BMT_status.charger_exist ? 1 : 0);
	return 0;
}

static unsigned int charging_full_check(void)
{
	unsigned int status;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if (status == KAL_TRUE) {
		g_full_check_count++;
		if (g_full_check_count >= FULL_CHECK_TIMES)
			return KAL_TRUE;
		else
			return KAL_FALSE;
	}

	g_full_check_count = 0;
	return status;
}

static bool mtk_is_pep_series_connect(void)
{
	if (mtk_pep20_get_is_connect() ||
	    mtk_pep_get_is_connect()) {
		return true;
	}

	return false;
}

static int mtk_check_aicr_upper_bound(void)
{
	u32 aicr_upper_bound = 0; /* 10uA */

	if (mtk_is_pep_series_connect())
		return -EPERM;

	/* Check AICR upper bound gererated by AICL */
	aicr_upper_bound = g_aicr_upper_bound * 100;
	if (g_temp_input_CC_value > aicr_upper_bound && aicr_upper_bound > 0)
		g_temp_input_CC_value = aicr_upper_bound;

	return 0;
}

void select_charging_current(void)
{
	if (g_ftm_battery_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\r\n",
			    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = batt_cust_data.ac_charger_current;

			battery_log(BAT_LOG_CRTI, "[BATTERY] set_ac_current \r\n");
		}
	} else {
		if (BMT_status.charger_type == STANDARD_HOST) {
#ifdef CONFIG_USB_IF
			{
				g_temp_input_CC_value = CHARGE_CURRENT_MAX;
				if (g_usb_state == USB_SUSPEND)
					g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
				else if (g_usb_state == USB_UNCONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;
				else if (g_usb_state == USB_CONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_configured;
				else
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;

				g_temp_input_CC_value = g_temp_CC_value;

				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
					    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = batt_cust_data.usb_charger_current;
				//CEI comment start//
				//Ibat is not Iusb, it can exceed 500mA, for sync to charging_hw_init CON04 ICHG setting
				#if 0
				g_temp_CC_value = batt_cust_data.usb_charger_current;
				#else
				g_temp_CC_value = USB_IBAT_CURRENT;
				#endif
				//CEI comment end//
			}
#endif
		} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.non_std_ac_charger_current;
			g_temp_CC_value = batt_cust_data.non_std_ac_charger_current;

		} else if (BMT_status.charger_type == STANDARD_CHARGER) {
			if (batt_cust_data.ac_charger_input_current != 0)
				g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
			else
				g_temp_input_CC_value = batt_cust_data.ac_charger_current;

			g_temp_CC_value = batt_cust_data.ac_charger_current;
			mtk_pep_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);
			mtk_pep20_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);

		} else if (BMT_status.charger_type == CHARGING_HOST) {
			g_temp_input_CC_value = batt_cust_data.charging_host_charger_current;
			g_temp_CC_value = batt_cust_data.charging_host_charger_current;
		} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_2_1a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_2_1a_charger_current;
		} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_1_0a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_1_0a_charger_current;
		} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_0_5a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_0_5a_charger_current;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;
			g_temp_CC_value = batt_cust_data.ac_charger_current;
		}
#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		set_jeita_charging_current();
#endif
	}

	mtk_check_aicr_upper_bound();
}

void select_charging_current_bcct(void)
{
/*BQ25896 is the first switch chrager separating input and charge current
* any switch charger can use this compile option which may be generalized
* to be CONFIG_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
*/
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	    (BMT_status.charger_type == NONSTANDARD_CHARGER)) {
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		   (BMT_status.charger_type == CHARGING_HOST)) {
		g_temp_input_CC_value = CHARGE_CURRENT_MAX;

		/* --------------------------------------------------- */
		/* set IOCHARGE */
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		/* --------------------------------------------------- */

	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}
#else
	if (g_bcct_flag == 1)
		g_temp_CC_value = g_bcct_value * 100;
	if (g_bcct_input_flag == 1)
		g_temp_input_CC_value = g_bcct_input_value * 100;
#endif

	mtk_check_aicr_upper_bound();
}

#ifdef CONFIG_CHARGER_QNS
extern int battery_get_qns_current(void);
#endif

//CEI comment start//
int adaptive_chrlimt_current(int vchr, int bcct_level_value)
{
        int index = 0;
        int ret = 0;

        for(index=0; index<(sizeof(chrlimit.vchr_thres)/sizeof(int)); index++)
        {
                if(vchr < chrlimit.vchr_thres[index])
                        break;
        }
	if ((bcct_level_value > 0) && (bcct_level_value <= 3))
		ret = chrlimit.chrlimt_current[index][bcct_level_value-1];
        return ret;
}

extern int TM_enable;
//CEI comment end//

static void mtk_select_ichg_aicr(void)
{
	kal_bool enable_charger = KAL_TRUE;

#ifdef CONFIG_CHARGER_QNS
	int qns_current = battery_get_qns_current() * 100;
#endif

//CEI comment start//
	int bcct_now = 0;
//CEI comment end//

	mutex_lock(&g_ichg_aicr_access_mutex);

	/* Set Ichg, AICR */
	if (get_usb_current_unlimited()) {
		if (batt_cust_data.ac_charger_input_current != 0)
			g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		else
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;

		g_temp_CC_value = batt_cust_data.ac_charger_current;
		battery_log(BAT_LOG_FULL,
			"USB_CURRENT_UNLIMITED, use batt_cust_data.ac_charger_current\n");
	}
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	else if (g_bcct_flag == 1) {
		select_charging_current_bcct();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current_bcct !\n");
	} else {
		select_charging_current();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current !\n");
	}
#else
//CEI comment start//
//	else if (g_bcct_flag == 1 || g_bcct_input_flag == 1) {
	else if ((g_bcct_flag == 1 || g_bcct_input_flag == 1) && (TM_enable)) {

	int bcct_level_value = bcct_level;
	int chr_vol = BMT_status.charger_vol;
	int chr_current_limt = 0;

	select_charging_current();
//	select_charging_current_bcct(); //MTK ORG
//CEI add: S//
	chr_current_limt = adaptive_chrlimt_current(chr_vol, bcct_level_value);
	if (chr_current_limt != 0)
	{
		if ((BMT_status.charger_type == STANDARD_HOST) ||
			(BMT_status.charger_type == NONSTANDARD_CHARGER))
		{
			if(chr_current_limt > CHARGE_CURRENT_500_00_MA / 100)
			{
				g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
				g_bcct_input_value = CHARGE_CURRENT_500_00_MA / 100;
			}
			else
			{
				g_temp_input_CC_value = chr_current_limt *100;
				g_bcct_input_value = chr_current_limt;
			}
		} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
			(BMT_status.charger_type == CHARGING_HOST))
		{
			if ((chr_current_limt < 350) && (chr_vol > 5500))
			{
				g_temp_input_CC_value = CHARGE_CURRENT_350_00_MA;
				g_bcct_input_value = CHARGE_CURRENT_350_00_MA / 100;
			}
			else
			{
				g_temp_input_CC_value = chr_current_limt *100;
				g_bcct_input_value = chr_current_limt;
			}
		}
		else
		{
			if(chr_current_limt > CHARGE_CURRENT_500_00_MA / 100)
			{
				g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
				g_bcct_input_value = CHARGE_CURRENT_500_00_MA / 100;
			}
			else
			{
				g_temp_input_CC_value = chr_current_limt *100;
				g_bcct_input_value = chr_current_limt;
			}
		}
		bcct_now = g_bcct_input_value;
		battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] LE(K)=> select_charging_curret_bcct! bcct_level_value=%d, chr_vol=%d, chr_current_limt=%d, bcct_now=%d\n", bcct_level_value, chr_vol, chr_current_limt, bcct_now);
	}
	else
	{
		bcct_now = 0;
		battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] LE(K)=> select_charging_curret! SPECIAL !g_temp_input_CC_value=%d\n", g_temp_input_CC_value);
	}
//CEI add: E//

		battery_log(BAT_LOG_CRTI,
			"[BATTERY][IUSB] LE(K)=> select_charging_curret_bcct !\n");
//CEI comment end//
	} else {
//CEI comment start//
		bcct_now = 0;
//CEI comment end//
		select_charging_current();
//CEI comment start//
		battery_log(BAT_LOG_CRTI,
			"[BATTERY][IUSB] LE(K)=> select_charging_curret !\n");
//CEI comment end//
	}
#endif

#ifdef CONFIG_CHARGER_QNS
	if  ((qns_current != 0) && (qns_current < g_temp_CC_value)) {
		battery_log(BAT_LOG_CRTI, "LE(K)=> qns_current (%d) is less than current setting (%d), use %dmA\r\n", qns_current/100, g_temp_CC_value/100, qns_current/100);
		g_temp_CC_value = qns_current;
	}
#endif

//CEI comment start//
//Alien_battery support
		if( (g_temp_CC_value > CHARGE_CURRENT_500_00_MA) && (battery_id_invalid == 1))
		{
			battery_log(BAT_LOG_CRTI, "LE(K)=> [ALIEN] battery_id_invalid, mtk_select_ichg_aicr org ichg=%dmA, new ichg=%d\r\n", g_temp_CC_value/100, CHARGE_CURRENT_500_00_MA/100);
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}
//CEI comment end//

	battery_charging_control(CHARGING_CMD_SET_CURRENT,
		&g_temp_CC_value);

//CEI comment start//
//Move from above to here
	battery_log(BAT_LOG_CRTI,
		    "[BATTERY][IUSB] LE(K)=> Default CC mode charging : %d, input current = %d, qns=%d, g_bcct_flag=%d, g_bcct_input_flag=%d, bcct_ever=%d, bcct_now=%d, Vchr=%d, bcct_level=%d, TM_enable=%d\r\n",
		    g_temp_CC_value, g_temp_input_CC_value, qns_current, g_bcct_flag, g_bcct_input_flag, bcct_ever, bcct_now, BMT_status.charger_vol, bcct_level, TM_enable);

	set_bcct_log(bcct_now);
	battery_log(BAT_LOG_CRTI,
		    "[BATTERY][IUSB] LE(K)=> %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u\n",
		    bcct_log.bcct_value[0], bcct_log.bcct_count[0], bcct_log.bcct_value[1], bcct_log.bcct_count[1], bcct_log.bcct_value[2], bcct_log.bcct_count[2], bcct_log.bcct_value[3], bcct_log.bcct_count[3], bcct_log.bcct_value[4], bcct_log.bcct_count[4],
		    bcct_log.bcct_value[5], bcct_log.bcct_count[5], bcct_log.bcct_value[6], bcct_log.bcct_count[6], bcct_log.bcct_value[7], bcct_log.bcct_count[7], bcct_log.bcct_value[8], bcct_log.bcct_count[8]);
	battery_log(BAT_LOG_CRTI,
		    "[BATTERY][IUSB] LE(K)=> %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, normal=%u\n\n",
		    bcct_log.bcct_value[9], bcct_log.bcct_count[9], bcct_log.bcct_value[10], bcct_log.bcct_count[10], bcct_log.bcct_value[11], bcct_log.bcct_count[11], bcct_log.bcct_value[12], bcct_log.bcct_count[12], bcct_log.bcct_value[13], bcct_log.bcct_count[13],
		    bcct_log.bcct_value[14], bcct_log.bcct_count[14], bcct_log.bcct_value[15], bcct_log.bcct_count[15], bcct_log.bcct_count[16]);

//	g_temp_input_CC_value = CHARGE_CURRENT_200_00_MA;

	battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
		&g_temp_input_CC_value);
//CEI comment end//

	/* For thermal, they need to enable charger immediately */
	if (g_temp_CC_value > 0 && g_temp_input_CC_value > 0)
		battery_charging_control(CHARGING_CMD_ENABLE, &enable_charger);

	/* If AICR < 300mA, stop PE+/PE+20 */
	if (g_temp_input_CC_value < CHARGE_CURRENT_300_00_MA) {
		if (mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(false);
			if (mtk_pep20_get_is_connect())
				mtk_pep20_reset_ta_vchr();
		}
		if (mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(false);
			if (mtk_pep_get_is_connect())
				mtk_pep_reset_ta_vchr();
		}
	} else if (g_bcct_input_flag == 0 && g_bcct_flag == 0) {
		if (!mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(true);
			mtk_pep20_set_to_check_chr_type(true);
		}
		if (!mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(true);
			mtk_pep_set_to_check_chr_type(true);
		}
	}

	mutex_unlock(&g_ichg_aicr_access_mutex);
}

static void mtk_select_cv(void)
{
	int ret = 0;
	u32 dynamic_cv = 0;
	BATTERY_VOLTAGE_ENUM cv_voltage;

#ifdef CONFIG_MTK_JEITA_STANDARD_SUPPORT
	/* If temperautre is abnormal, return not permitted */
	if (g_temp_status != TEMP_POS_10_TO_POS_45)
		return;
#endif

	if (batt_cust_data.high_battery_voltage_support)
//CEI comment start//
		cv_voltage = BATTERY_VOLT_04_304000_V; // MTK ORG=BATTERY_VOLT_04_340000_V
//CEI comment end//
	else
		cv_voltage = BATTERY_VOLT_04_200000_V;

	ret = mtk_get_dynamic_cv(&dynamic_cv);
	if (ret == 0) {
		cv_voltage = dynamic_cv * 1000;
		battery_log(BAT_LOG_FULL, "%s: set dynamic cv = %dmV\n",
			__func__, dynamic_cv);
	}

#if defined(CONFIG_MTK_HAFG_20) && defined(CUSTOM_SOFT_CHARGE_30)
	if (g_sc30_low_cv_voltage && (cv_voltage > BATTERY_VOLT_04_300000_V))
		cv_voltage = BATTERY_VOLT_04_300000_V;

	battery_log(BAT_LOG_CRTI, "[SC30] low_cv=%d cv_voltage=%d\n",
		g_sc30_low_cv_voltage, cv_voltage);
#endif

//CEI comment start//
//Alien_battery
	if(battery_id_invalid)
	{
		//It will never goes to here because the condition (g_temp_status != TEMP_POS_10_TO_POS_45); return at the begining
		if (g_temp_status >= TEMP_POS_45_TO_POS_60)
			cv_voltage = BATTERY_VOLT_04_000000_V;
		else
			cv_voltage = BATTERY_VOLT_04_200000_V;

		battery_log(BAT_LOG_CRTI,
			"[BATTERY]LE(K)=> [ALIEN] mtk_select_cv(), cv_voltage=%d, g_temp_status=%d\n", cv_voltage, g_temp_status);
	}

	battery_log(BAT_LOG_CRTI,
		    "[BATTERY]LE(K)=> %s SET_CV_VOLTAGE=%d, BATT_ID=%d, battery_id_invalid=%d, id_voltage_read=%d g_battery_id_voltage=%d %d %d\n", __func__, cv_voltage,
		    g_fg_battery_id, battery_id_invalid, id_voltage_read, g_battery_id_voltage[0], g_battery_id_voltage[1], g_battery_id_voltage[2]);
//CEI comment end//
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);

#if defined(CONFIG_MTK_HAFG_20)
	g_cv_voltage = cv_voltage;
#endif
}

static void pchr_turn_on_charging(void)
{
	u32 charging_enable = KAL_TRUE;

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	if (BMT_status.charger_exist)
		charging_enable = KAL_TRUE;
	else
		charging_enable = KAL_FALSE;
#endif

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] Charger Error, turn OFF charging !\n");
		charging_enable = KAL_FALSE;
	} else if ((g_platform_boot_mode == META_BOOT) ||
		   (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/* HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);
		battery_log(BAT_LOG_FULL, "charging_hw_init\n");

		/* PE+/PE+20 algorithm */
		mtk_pep20_start_algorithm();
		mtk_pep_start_algorithm();

		/* Select ICHG/AICR */
		mtk_select_ichg_aicr();

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA ||
		    g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;
			battery_log(BAT_LOG_CRTI,
				"[BATTERY] charging current is set 0mA, turn off charging !\r\n");
		} else /* Set CV Voltage */
			mtk_select_cv();

	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL,
		"[BATTERY] pchr_turn_on_charging(), enable =%d !\r\n",
		charging_enable);
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
#ifdef CONFIG_MTK_BIF_SUPPORT
	int ret = 0;
	bool bif_exist = false;
#endif
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n\r",
		    BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

#ifdef CONFIG_MTK_BIF_SUPPORT
	/* If defined BIF but not BIF's battery, stop charging */
	ret = battery_charging_control(CHARGING_CMD_GET_BIF_IS_EXIST,
		&bif_exist);
	if (!bif_exist) {
		battery_log(BAT_LOG_CRTI,
			"%s: define BIF but no BIF battery, disable charging\n",
			__func__);
		BMT_status.bat_charging_state = CHR_ERROR;
		return PMU_STATUS_OK;
	}
#endif

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();
#if defined(CONFIG_MTK_HAFG_20)
	if (BMT_status.UI_SOC2 == 100 && charging_full_check()) {
#else
	if (BMT_status.UI_SOC == 100) {
#endif
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	/* If it is not disabled by throttling,
	 * enable PE+/PE+20, if it is disabled
	 */
	if (g_bcct_input_flag && g_bcct_input_value < 300)
		return PMU_STATUS_OK;

	if (!mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(true);
		mtk_pep20_set_to_check_chr_type(true);
	}
	if (!mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(true);
		mtk_pep_set_to_check_chr_type(true);
	}

	return PMU_STATUS_OK;
}

//CEI comment start//
//99%<->100% reqirement
extern int chg_ever_full;
//CEI comment end//
PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%d on %d!!\n",
		    BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();

	if (charging_full_check() == KAL_TRUE) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
//CEI comment start//
//99%<->100% reqirement
		chg_ever_full = 1;
//CEI comment end//
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryFullAction(void)
{
	unsigned int led_en = false;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n\r");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	battery_log(BAT_LOG_FULL, "Turn off PWRSTAT LED\n");
	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*
	 * If CV is set to lower value by JEITA,
	 * Reset CV to normal value if temperture is in normal zone
	 */
	mtk_select_cv();

	if (charging_full_check() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n\r");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
#ifndef CONFIG_MTK_HAFG_20
		battery_meter_reset();
#endif
		mtk_pep20_set_to_check_chr_type(true);
		mtk_pep_set_to_check_chr_type(true);
		g_enable_dynamic_cv = true;
	}


	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n\r");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_log(BAT_LOG_CRTI, "[BATTERY] Exit Hold mode and Enter CC mode !!\n\r");
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n\r");

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	if ((g_temp_status == TEMP_ABOVE_POS_60) || (g_temp_status == TEMP_BELOW_NEG_10))
		temp_error_recovery_chr_flag = KAL_FALSE;

	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60)
	    && (g_temp_status != TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#endif

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	/* Disable PE+/PE+20 */
	if (mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(false);
		if (mtk_pep20_get_is_connect())
			mtk_pep20_reset_ta_vchr();
	}
	if (mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(false);
		if (mtk_pep_get_is_connect())
			mtk_pep_reset_ta_vchr();
	}

	return PMU_STATUS_OK;
}

//CEI comment start//
//Add more charing related logs
int fgr = -1;
int f_rname = 0;
extern void fgr_get(int fgr);
extern int get_FG_Current(void);
extern int data_log_level;
extern int fg_w_avg_delta;
extern int get_car_tune_value(void);

int calc_fgr(void)
{
	int ret_car = 117;
	int w_avg_delta = fg_w_avg_delta;

	if ( (w_avg_delta >= 7) && (w_avg_delta < 10) )
		ret_car = 110;
	if ( (w_avg_delta >= 10) && (w_avg_delta < 13) )
		ret_car = 107;
	else if ( (w_avg_delta >= 13) && (w_avg_delta < 16) )
		ret_car = 104;
	else if ( w_avg_delta >= 16 )
		ret_car = 101;

	return ret_car;
}

//=========//
#define SWITCHING_THR (7)
#define SAMPLE_NUM (180)
#define DATA_FILTER_MAX (1950)
#define DATA_FILTER_MIN (550)
#define BQ_SCALE (50)
#define BQ_CASE_NUM ((DATA_FILTER_MAX / BQ_SCALE) + 1)
#define FILTER_VAL (30)
#define VALID_SAMPLE_NUM (80)
#define VALID_BQ_CASE_SAMPLE_NUM (6)
#define IGN_SAMPLE_NUM (5)

int bq_sample[SAMPLE_NUM] = {0};
int fg_sample[SAMPLE_NUM] = {0};
int sample_idx = 0;

int fg_w_avg_delta = 0;
int fg_avg_delta = 0;
int fg_check_status = 0;
int fg_n_case = 0;
int fg_a_case = 0;

int calculate_delta(void)
{
	int bq_case_sample[SAMPLE_NUM] = {0};
	int fg_delta[BQ_CASE_NUM] = {0};
	int fg_delta_idx[BQ_CASE_NUM] = {0};
	int bq_case_i = 0;
	int bq_i = 0, fg_i = 0;
	int sort_i = 0, sort_j = 0, sort_tmp_val = 0;
	int avg_delta_idx = 0;
	int valid_sample = 0;
	int fg_check_state = 0;
	int ign_sample = 0;

	battery_log(BAT_LOG_FULL, "CHIHI(K)=> calculate_delta() enter\n");

	sample_idx = 0;
	fg_avg_delta = 0;
	fg_w_avg_delta = 0;

	for (bq_case_i = 0; bq_case_i < BQ_CASE_NUM; bq_case_i ++)
	{
		fg_delta[bq_case_i] = 0;
		fg_delta_idx[bq_case_i]=0;
		for (bq_i = 0; bq_i < SAMPLE_NUM; bq_i ++)
		{
			if ((bq_sample[bq_i] / BQ_SCALE) == bq_case_i)
			{
				bq_case_sample[fg_delta_idx[bq_case_i]] = fg_sample[bq_i];
				fg_delta_idx[bq_case_i] ++;
			}
		}

		if (fg_delta_idx[bq_case_i] < 1)
		{
			continue;
		}

		for (sort_i = 0; sort_i < (fg_delta_idx[bq_case_i] - 1); sort_i ++)
		{
			for(sort_j = (sort_i + 1); sort_j < (fg_delta_idx[bq_case_i]); sort_j++)
			{
				if (bq_case_sample[sort_j] < bq_case_sample[sort_i])
				{
					sort_tmp_val = bq_case_sample[sort_i];
					bq_case_sample[sort_i] = bq_case_sample[sort_j];
					bq_case_sample[sort_j] = sort_tmp_val;
				}
			}
		}

		for( fg_i = fg_delta_idx[bq_case_i]; fg_i < (((fg_delta_idx[bq_case_i] + 9) / 10) * 10); fg_i ++)
		{
			bq_case_sample[fg_i] = -1;
		}
		for( fg_i = 0; fg_i < ((fg_delta_idx[bq_case_i] + 9) / 10); fg_i ++)
		{
			battery_log(BAT_LOG_FULL, "CHIHI(K)=> %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d \n",
				bq_case_sample[fg_i * 10], bq_case_sample[fg_i * 10 + 1], bq_case_sample[fg_i * 10 + 2], bq_case_sample[fg_i * 10 + 3], bq_case_sample[fg_i * 10 + 4],
				bq_case_sample[fg_i * 10 + 5], bq_case_sample[fg_i * 10 + 6], bq_case_sample[fg_i * 10 + 7], bq_case_sample[fg_i * 10 + 8], bq_case_sample[fg_i * 10 + 9]);
		}

		if (fg_delta_idx[bq_case_i] >= VALID_BQ_CASE_SAMPLE_NUM)
		{
			ign_sample = ((fg_delta_idx[bq_case_i] * IGN_SAMPLE_NUM) / 100);
			for( fg_i = ign_sample; fg_i < (fg_delta_idx[bq_case_i] - ign_sample); fg_i ++)
			{
				fg_delta[bq_case_i] += (bq_case_sample[fg_i] - (BQ_SCALE * bq_case_i));
				valid_sample ++;
			}
			fg_delta[bq_case_i] = (((fg_delta[bq_case_i] * 100) / (fg_delta_idx[bq_case_i] - (ign_sample * 2))) / (BQ_SCALE * bq_case_i));
			fg_avg_delta += fg_delta[bq_case_i];
			fg_w_avg_delta += (fg_delta[bq_case_i] * (fg_delta_idx[bq_case_i] - (ign_sample * 2)));
			avg_delta_idx ++;
			battery_log(BAT_LOG_FULL, "CHIHI(K)=> bq:%d sample=%d ign_sample:%d fg_delta:%d fg_w_delta:%d (delta:%d, w_elta:%d)\n", (int)(BQ_SCALE * bq_case_i), fg_delta_idx[bq_case_i], ign_sample, fg_delta[bq_case_i], (fg_delta[bq_case_i] * (fg_delta_idx[bq_case_i] - (ign_sample * 2))), fg_avg_delta, fg_w_avg_delta);
		}
		else
		{
			fg_delta[bq_case_i] = 0;
			battery_log(BAT_LOG_FULL, "CHIHI(K)=> bq:%d sample=%d Ignore! \n", (int)(BQ_SCALE * bq_case_i), fg_delta_idx[bq_case_i]);
		}
	}

	battery_log(BAT_LOG_FULL, "CHIHI(K)=> avg_delta:%d (delta:%d index:%d) \n", (fg_avg_delta / avg_delta_idx), fg_avg_delta, avg_delta_idx);
	battery_log(BAT_LOG_FULL, "CHIHI(K)=> w_avg_delta:%d (w_delta:%d valid_sample:%d) \n", (fg_w_avg_delta / valid_sample), fg_w_avg_delta, valid_sample);
	if( valid_sample >= VALID_SAMPLE_NUM && avg_delta_idx > 0 )
	{
		fg_avg_delta = fg_avg_delta / avg_delta_idx;
		fg_w_avg_delta = fg_w_avg_delta / valid_sample;
		if( fg_w_avg_delta >= SWITCHING_THR )
		{
			fg_check_state = 2;
			fg_a_case ++;
			battery_log(BAT_LOG_FULL, "CHIHI(K)=> Average(W) Delta:%d [a] (n:%d, a:%d) delta:%d\n", fg_w_avg_delta, fg_n_case, fg_a_case, fg_avg_delta);
		}
		else
		{
			fg_check_state = 1;
			fg_n_case ++;
			battery_log(BAT_LOG_FULL, "CHIHI(K)=> Average(W) Delta:%d [n] (n:%d, a:%d) delta:%d\n", fg_w_avg_delta, fg_n_case, fg_a_case, fg_avg_delta);
		}
	}
	else
	{
		battery_log(BAT_LOG_FULL, "CHIHI(K)=> Ignore!  \n");
	}

	battery_log(BAT_LOG_FULL, "CHIHI(K)=> calculate_delta() leave\n");
	return (fg_check_state);

}

int delta_checking(int bq, int fg)
{
	int s_data = 0, l_data = 0;
	int fg_check_state = 0;

	battery_log(BAT_LOG_FULL, "CHIH(K)=> delta_checking() enter: bq=%d, fg=%d\n", bq, fg);

	if (fg >= 0 || bq <= 0)
	{
		battery_log(BAT_LOG_FULL, "CHIH(K)=> delta_checking() discharging\n");
		return (fg_check_state);
	}

	if (bq > DATA_FILTER_MAX || bq < DATA_FILTER_MIN )
	{
		battery_log(BAT_LOG_FULL, "CHIH(K)=> delta_checking() Invalid data! (bq > %d or bq < %d)\n", (int)((DATA_FILTER_MAX)-(BQ_SCALE)), (int)(DATA_FILTER_MIN));
		return (fg_check_state);
	}

	fg = abs(fg);
	if (bq < fg)
	{
		s_data = bq;
		l_data = fg;
	}
	else
	{
		s_data = fg;
		l_data = bq;
	}
	if ((s_data * (100 + FILTER_VAL)) < (l_data * 100))
	{
		battery_log(BAT_LOG_FULL, "CHIH(K)=> delta_checking() Invalid data! (delta > %d \n", (int)(FILTER_VAL));
		return (fg_check_state);
	}

	bq_sample[sample_idx] = bq;
	fg_sample[sample_idx] = fg;
	sample_idx ++;
	battery_log(BAT_LOG_FULL, "CHIH(K)=> delta_checking() valid data! bq=%d, fg=%d sample_idx:%d\n", bq, fg, sample_idx);

	if (sample_idx >= SAMPLE_NUM)
	{
		fg_check_state = calculate_delta();
	}

	return (fg_check_state);

}

extern int get_fcr(void);
extern int cat_rtc_info(void);
extern int echo_rtc_info(int val);
extern int car_value_rtc;
extern int car_value_rtc_post;

void dump_chg_data(void)
{
	unsigned int chg_data = 0;
	int f_data_signed = 0;
	static int cat_rtc_value = 0;

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER_GET_DATA, &chg_data);

	if ((data_log_level != 0) && (fgr == -1))
	{
		if(car_value_rtc != 0)
		{
			//Could be marked later: S
			//f_data_signed = get_FG_Current()/10;
			//battery_log(BAT_LOG_CRTI, "LE(K)=> APTB(A) fcr=%d, cat_rtc_value=%d, value_rtc=%d, val rtc post=%d, bq=%d, fg=%d\n", get_fcr(), cat_rtc_value, car_value_rtc, car_value_rtc_post, chg_data, f_data_signed);
			//Could be marked later: E
		}
		else
		{
			f_data_signed = get_FG_Current()/10;

			fg_check_status = delta_checking(chg_data, f_data_signed);
			battery_log(BAT_LOG_FULL, "LE(K)=> APTB(B) data_log_level=%d, fg_check_status=%d, f_rname=%d, fgr=%d, fcr=%d, cat_rtc_value=%d, value_rtc=%d, val rtc post=%d\n", data_log_level, fg_check_status, f_rname, fgr, get_fcr(), cat_rtc_value, car_value_rtc, car_value_rtc_post);

			if ( (fg_check_status != 0) && ((data_log_level == 2) || (data_log_level == 3)) )
			{
				if (fg_check_status == 1)
				{
					if(fgr == -1)
					{
						f_rname = 1;
						fgr = get_car_tune_value();
						if(data_log_level == 3)
						{
							fgr_get(fgr);
							if( (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) ||
								(g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) )
							{
								echo_rtc_info(fgr);
								cat_rtc_value=cat_rtc_info();
							}
						}
					}
				}
				else if (fg_check_status == 2)
				{
					if(fgr == -1)
					{
						f_rname = 2;
						fgr = calc_fgr();
						if(data_log_level == 3)
						{
							fgr_get(fgr);
							if( (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) ||
								(g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) )
							{
								echo_rtc_info(fgr);
								cat_rtc_value=cat_rtc_info();
							}
						}
					}
				}
				battery_log(BAT_LOG_FULL, "LE(K)=> APTB: fgr=%d, zfg_w_avg_delta=%d, cat_rtc_value=%d\n", fgr, fg_w_avg_delta, cat_rtc_value);
			}
		}
	}
}
//CEI comment end//

void mt_battery_charging_algorithm(void)
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

	/* Generate AICR upper bound by AICL */
	if (!mtk_is_pep_series_connect()) {
		battery_charging_control(CHARGING_CMD_RUN_AICL,
			&g_aicr_upper_bound);
	}

	mtk_pep20_check_charger();
	mtk_pep_check_charger();
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE:
		BAT_PreChargeModeAction();
		break;

	case CHR_CC:
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_BATFULL:
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:
		BAT_BatteryHoldAction();
		break;

	case CHR_ERROR:
		BAT_BatteryStatusFailAction();
		break;
	}

//CEI comment start//
	dump_chg_data();
//CEI comment end//
}
