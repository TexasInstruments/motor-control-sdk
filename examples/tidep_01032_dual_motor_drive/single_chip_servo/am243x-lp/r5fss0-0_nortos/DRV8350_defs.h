/*
 *  Copyright (C) 2004-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DRV8350_DEFS_H
#define _DRV8350_DEFS_H

#include <inttypes.h>

/*****************************************************************************/
// Register Addresses
/*****************************************************************************/
//DRV8305 Register Addresses
#define DRV8350_FAULT_STATUS_1_ADDR	           	   0x00
#define DRV8350_VGS_STATUS_2_ADDR 			       0x01
#define DRV8350_DRIVER_CONTROL_ADDR		           0x02
#define DRV8350_GATE_DRIVE_HS_ADDR		           0x03
#define DRV8350_GATE_DRIVE_LS_ADDR 				   0x04
#define DRV8350_OCP_CONTROL_ADDR   				   0x05
#define DRV8350_CSA_CONTROL_ADDR               	   0x06
#define DRV8350_CAL_MODE_ADDR                 	   0x07

#define DRV8350_READ                               0x8000
#define DRV8350_WRITE                              0x0000
#define DRV8350_ADDR_SHIFT                         11


//=============================================================================

// DRV8305 Warning and Watchdog Register bit definitions:
struct  DRV8350_FAULT_STATUS_1_REG_BITS
{                                   // bit    	description
	   uint16_t VDS_LC:1;				// 0	Over temp warning
	   uint16_t VDS_HC:1;				// 1	temp > 135degC
	   uint16_t VDS_LB:1;				// 2	temp > 125degC
	   uint16_t VDS_HB:1;				// 3	temp > 105degC
	   uint16_t VDS_LA:1;				// 4	Charge pump UV flag warning
	   uint16_t VDS_HA:1;				// 5	Real time status of VDS sensor
	   uint16_t OTSD:1;				// 6	PVDD ov flag
	   uint16_t UVLO:1;				// 7	PVDD uv flag
	   uint16_t GDF:1;				// 8	temp > 175degC
	   uint16_t VDS_OCP:1;			// 9
	   uint16_t FAULT:1;				// 10	Latched fault
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               		all;
   struct DRV8350_FAULT_STATUS_1_REG_BITS	bit;
} DRV8350_FAULT_STATUS_1_REG;
// =============================
struct DRV8350_VGS_STATUS_2_REG_BITS
{                                   // bit    	description
	   uint16_t VGS_LC:1;				// 0	Over temp warning
	   uint16_t VGS_HC:1;				// 1	temp > 135degC
	   uint16_t VGS_LB:1;				// 2	temp > 125degC
	   uint16_t VGS_HB:1;				// 3	temp > 105degC
	   uint16_t VGS_LA:1;				// 4	Charge pump UV flag warning
	   uint16_t VGS_HA:1;				// 5	Real time status of VDS sensor
	   uint16_t OTSD:1;				// 6	VDS monitor fault - high side FET C
	   uint16_t OTW:1;				// 7	VDS monitor fault - low side FET B
	   uint16_t SC_OC:1;				// 8	VDS monitor fault - high side FET B
	   uint16_t SB_OC:1;				// 9	VDS monitor fault - low side FET A
	   uint16_t SA_OC:1;				// 10	VDS monitor fault - high side FET A
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               			        all;
   struct DRV8350_VGS_STATUS_2_REG_BITS	bit;
} DRV8350_VGS_STATUS_2_REG;
// =============================
struct DRV8350_DRIVER_CONTROL_REG_BITS
{                                   // bit    	description
	   uint16_t CLR_FLT:1;			// 0	charge pump high side OV ABS fault
	   uint16_t BRAKE:1;		    	// 1	charge pump high side OV fault
	   uint16_t COAST:1;		    	// 2	charge pump high side UV fault
	   uint16_t PWM1_DIR:1;			// 3
	   uint16_t PWM1_COM:1;			// 4    charge pump low side gate driver fault
	   uint16_t PWM_MODE:2;			// 5:6	AVDD uv fault
	   uint16_t OTW_REP:1;			// 7
	   uint16_t DIS_GDF:1;			// 8	Over temp fault
	   uint16_t DIS_GDUV:1;			// 9	Watchdog fault
	   uint16_t OCP_ACT:1;			// 10	PVDD uvder uv 2 fault
	   uint16_t Reserved3:5;			// 15-11
};

typedef union  {
   uint16_t               			    all;
   struct DRV8350_DRIVER_CONTROL_REG_BITS	bit;
} DRV8350_DRIVER_CONTROL_REG;

enum {
	DRV8350_OCP_disable_phase = 0,
	DRV8350_OCP_disable_three_phases = 1
};

enum {
	DRV8350_VCP_VGLS_enable = 0,
	DRV8350_VCP_VGLS_disable = 1
};

enum {
	DRV8350_gate_drive_fault_enable = 0,
	DRV8350_gate_drive_fault_disable = 1
};

enum {
	DRV8350_OTW_rep_disable = 0,
	DRV8350_OTW_rep_enable = 1
};

enum {
	DRV8350_PWM_mode_6 = 0,
	DRV8350_PWM_mode_3 = 1,
	DRV8350_PWM_mode_1 = 2
};

enum {
	DRV8350_1PWM_mode_sync = 0,
	DRV8350_1PWM_mode_async = 1
};

enum {
	DRV8350_coast_disable = 0,
	DRV8350_coast_enable = 1
};

enum {
	DRV8350_brake_disable = 0,
	DRV8350_brake_enable = 1
};

enum {
	DRV8350_ClrFaults_No = 0,
	DRV8350_ClrFaults_Yes = 1
};

// =============================

struct DRV8350_GATE_DRIVE_HS_REG_BITS
{                                   // bit    	description
	   uint16_t IDRIVEN_HS:4;			// 3:0
	   uint16_t IDRIVEP_HS:4;			// 4:7     VGS monitor fault low side FET C
	   uint16_t LOCK:3;				// 8:10     VGS monitor fault high side FET C
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               			        all;
   struct DRV8350_GATE_DRIVE_HS_REG_BITS	bit;
} DRV8350_GATE_DRIVE_HS_REG;
// =============================



struct DRV8350_GATE_DRIVE_LS_REG_BITS
{                                   // bit    	description
	   uint16_t IDRIVEN_LS:4;			// 3:0	 high side gate driver peak source current
	   uint16_t IDRIVEP_LS:4;			// 7:4	 high side gate driver peak sink current
	   uint16_t TDRIVE:2;				// 9:8   high side gate driver peak source time
	   uint16_t CBC:1;           		// 10
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               			             all;
   struct DRV8350_GATE_DRIVE_LS_REG_BITS	 bit;
} DRV8350_GATE_DRIVE_LS_REG;

enum {
	DRV8350_lock_enable = 6,
	DRV8350_lock_disable = 3
};

enum {
	DRV8350_OCP_ClrFaults_Cycle_by_Cycle_No = 0,
	DRV8350_OCP_ClrFaults_Cycle_by_Cycle_Yes = 1
};

enum {
	DRV8350_tdrive_500nS = 0,
	DRV8350_tdrive_1000nS = 1,
	DRV8350_tdrive_2000nS = 2,
	DRV8350_tdrive_4000nS = 3
};

enum {
	DRV8350_idriveN_100mA = 0,
	DRV8350_idriveN_100mA1 = 1,
	DRV8350_idriveN_200mA = 2,
	DRV8350_idriveN_300mA = 3,
	DRV8350_idriveN_600mA = 4,
	DRV8350_idriveN_700mA = 5,
	DRV8350_idriveN_800mA = 6,
	DRV8350_idriveN_900mA = 7,
	DRV8350_idriveN_1100mA = 8,
	DRV8350_idriveN_1200mA = 9,
	DRV8350_idriveN_1300mA = 10,
	DRV8350_idriveN_1400mA = 11,
	DRV8350_idriveN_1700mA = 12,
	DRV8350_idriveN_1800mA = 13,
	DRV8350_idriveN_1900mA = 14,
	DRV8350_idriveN_2000mA = 15
};

enum {
	DRV8350_idriveP_50mA = 0,
	DRV8350_idriveP_50mA1 = 1,
	DRV8350_idriveP_100mA = 2,
	DRV8350_idriveP_150mA = 3,
	DRV8350_idriveP_300mA = 4,
	DRV8350_idriveP_350mA = 5,
	DRV8350_idriveP_400mA = 6,
	DRV8350_idriveP_450mA = 7,
	DRV8350_idriveP_550mA = 8,
	DRV8350_idriveP_600mA = 9,
	DRV8350_idriveP_650mA = 10,
	DRV8350_idriveP_700mA = 11,
	DRV8350_idriveP_850mA = 12,
	DRV8350_idriveP_900mA = 13,
	DRV8350_idriveP_950mA = 14,
	DRV8350_idriveP_1000mA = 15
};

// =============================
struct DRV8350_OCP_CONTROL_REG_BITS
{                                   // bit    	description
	   uint16_t VDS_LVL:4;			// 3:0	 low side gate driver peak source current
	   uint16_t OCP_DEG:2;			// 5:4	 low side gate driver peak sink current
	   uint16_t OCP_MODE:2;			// 7:6   low side gate driver peak source time
	   uint16_t DEAD_TIME:2;          // 9:8
	   uint16_t TRETRY:2;          	// 10
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               			              all;
   struct DRV8350_OCP_CONTROL_REG_BITS	  bit;
} DRV8350_OCP_CONTROL_REG;

enum {
	DRV8350_OCP_retry_time_8ms = 0,
	DRV8350_OCP_retry_time_50us = 1
};

enum {
	DRV8350_deadTime_50nS = 0,
	DRV8350_deadTime_100nS = 1,
	DRV8350_deadTime_200nS = 2,
	DRV8350_deadTime_400nS = 3,
};

enum {
	DRV8350_OCP_Latch_Fault = 0,
	DRV8350_OCP_RETRY_Fault = 1,
	DRV8350_OCP_report_no_action = 2,
	DRV8350_OCP_no_report_no_action = 3
};

enum {
	DRV8350_deglitch_1us = 0,
	DRV8350_deglitch_2us = 1,
	DRV8350_deglitch_4us = 2,
	DRV8350_deglitch_8us = 3
};

enum {
	DRV8350_VDS_LVL_60mV = 0,
	DRV8350_VDS_LVL_70mV = 1,
	DRV8350_VDS_LVL_80mV = 2,
	DRV8350_VDS_LVL_90mV = 3,
	DRV8350_VDS_LVL_100mV = 4,
	DRV8350_VDS_LVL_200mV = 5,
	DRV8350_VDS_LVL_300mV = 6,
	DRV8350_VDS_LVL_400mV = 7,
	DRV8350_VDS_LVL_500mV = 8,
	DRV8350_VDS_LVL_600mV = 9,
	DRV8350_VDS_LVL_700mV = 10,
	DRV8350_VDS_LVL_800mV = 11,
	DRV8350_VDS_LVL_900mV = 12,
	DRV8350_VDS_LVL_1000mV = 13,
	DRV8350_VDS_LVL_1500mV = 14,
	DRV8350_VDS_LVL_2000mV = 15
};

// =============================
struct DRV8350_CSA_CONTROL_REG_BITS
{                                   // bit    	description
	   uint16_t SEN_LVL:2;			// 1:0    VDS sense deglitch
	   uint16_t CSA_CAL_C:1;			// 2	  VDS sense blanking
   	   uint16_t CSA_CAL_B:1;			// 3	  VDS sense blanking
   	   uint16_t CSA_CAL_A:1;			// 4	  VDS sense blanking
	   uint16_t DIS_SEN:1;			// 5	  Dead time
	   uint16_t CSA_GAIN:2;			// 7:6	  PWM mode
	   uint16_t LS_REF:1;				// 8	  Rectification control
	   uint16_t VREF_DIF:1;			// 9	  Rectification control
	   uint16_t CSA_FET:1;			// 10
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               			    all;
   struct DRV8350_CSA_CONTROL_REG_BITS	    bit;
} DRV8350_CSA_CONTROL_REG;

enum {
	DRV8350_amp_pos_SPx = 0,
	DRV8350_amp_pos_SHx = 1
};

enum {
	DRV8350_vref_uni = 0,
	DRV8350_vref_div_2 = 1
};

enum {
	DRV8350_VDS_measured_SHxtoSPx = 0,
	DRV8350_VDS_measured_SHxtoSNx = 1
};

enum {
	DRV8350_gain_CS_5 = 0,
	DRV8350_gain_CS_10 = 1,
	DRV8350_gain_CS_20 = 2,
	DRV8350_gain_CS_40 = 3
};

enum {
	DRV8350_enable_SnsOcp = 0,
	DRV8350_disable_SnsOcp = 1
};

enum {
	DRV8350_Normal_operation_amp_A = 0,
	DRV8350_Calibrate_amp_A = 1
};
enum {
	DRV8350_Normal_operation_amp_B = 0,
	DRV8350_Calibrate_amp_B = 1
};
enum {
	DRV8350_Normal_operation_amp_C = 0,
	DRV8350_Calibrate_amp_C = 1
};

enum {
	DRV8350_Snslvl_250mV = 0,
	DRV8350_Snslvl_500mV = 1,
	DRV8350_Snslvl_750mV = 2,
	DRV8350_Snslvl_1000mV = 3
};

// =============================
struct DRV8350_CAL_MODE_REG_BITS
{                                   // bit    	description
	   uint16_t CAL_MODE:1;			// 0	  set charge pump uv threshold level
	   uint16_t Reserved1:10;			// 10-1
	   uint16_t Reserved2:5;			// 15-11
};

typedef union  {
   uint16_t               		    all;
   struct DRV8350_CAL_MODE_REG_BITS	bit;
} DRV8350_CAL_MODE_REG;



enum {
	DRV8350_Cal_manual = 0,
	DRV8350_Cal_auto = 1
};
//=============================================================================
typedef struct  {
	DRV8350_FAULT_STATUS_1_REG          status1_fault;
	DRV8350_VGS_STATUS_2_REG            status2_vgs;
	DRV8350_DRIVER_CONTROL_REG          driver_control;
	DRV8350_GATE_DRIVE_HS_REG           gate_drive_hs;
	DRV8350_GATE_DRIVE_LS_REG           gate_drive_ls;
	DRV8350_OCP_CONTROL_REG    			ocp_control;
	DRV8350_CSA_CONTROL_REG    			csa_control;
	DRV8350_CAL_MODE_REG                cal_mode;

	// DRV8350 variables
	uint16_t readCntrl2,
	       readCntrl3,
		   readCntrl4,
	       readCntrl5,
		   readCntrl6,
	       readCntrl7,

		   DRV_fault;
} DRV8350_Vars;

#define  DRV8350_DEFAULTS  {               \
		0,     /*  Rsvd0              */   \
		0,     /*  status1_wwd        */   \
		0,     /*  status2_ov_vds     */   \
		0,     /*  status3_IC         */   \
		0,     /*  status4_gd_Vgs     */   \
		0,     /*  cntr5_hs_gd        */   \
		0,     /*  cntr6_ls_gd        */   \
		0,     /*  cntr7_gd           */   \
                                           \
		0,     /*  readCntrl2         */   \
		0,     /*  readCntrl3         */   \
		0,     /*  readCntrl4         */   \
		0,     /*  readCntrl5         */   \
		0,     /*  readCntrl6         */   \
		0,     /*  readCntrl7         */   \
		0,     /*  DRV_fault          */   \
		&SpiaRegs,						   \
}

#endif
