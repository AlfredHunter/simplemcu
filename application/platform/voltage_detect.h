/* 
*  Author: Adam Huang
*  Date:2016/6/8
*/
#ifndef __VOLTAGE_DETECT_H
#define __VOLTAGE_DETECT_H

#include <stdint.h>
#include "Common.h"
#include "Debug.h"
#include "Mico.h"
#define  VOLTAGE_DEBUG

#define VBAT_POWER_ON_LEVEL          2400//x10mV
#define VBAT_LOW_POWER_LEVEL         2500//x10mV
#define VBAT_POWER_OFF_LEVEL         2400//x10mV
#define VBAT_FULL_POWER_LEVEL        2800//x10mV

//#define SCOTT_FOR_TEST
#ifndef SCOTT_FOR_TEST
#define   CURRENT_THRESHOLD_H_PRINTER_MA          2500
#define   CURRENT_THRESHOLD_L_PRINTER_MA          0
#define   CURRENT_THRESHOLD_H_DLP_MA              2500
#define   CURRENT_THRESHOLD_L_DLP_MA              0
#define   CURRENT_THRESHOLD_H_2_1_PA_MA           3000
#define   CURRENT_THRESHOLD_L_2_1_PA_MA           0
#define   CURRENT_THRESHOLD_H_PAD_MA              2500
#define   CURRENT_THRESHOLD_L_PAD_MA              0
#define   CURRENT_THRESHOLD_H_5V_RESERVE1_MA      1500
#define   CURRENT_THRESHOLD_L_5V_RESERVE1_MA      0
#define   CURRENT_THRESHOLD_H_X86_MA              3000
#define   CURRENT_THRESHOLD_L_X86_MA              0
#define   CURRENT_THRESHOLD_H_DH_5V_MA            200
#define   CURRENT_THRESHOLD_L_DH_5V_MA            0
#define   CURRENT_THRESHOLD_H_SENSOR_MA           200
#define   CURRENT_THRESHOLD_L_SENSOR_MA           0
#define   CURRENT_THRESHOLD_H_5V_LED_MA           3500
#define   CURRENT_THRESHOLD_L_5V_LED_MA           0
#define   CURRENT_THRESHOLD_H_MOTOR_5V_MA         100
#define   CURRENT_THRESHOLD_L_MOTOR_5V_MA         0
#define   CURRENT_THRESHOLD_H_MOTOR_MA            2000
#define   CURRENT_THRESHOLD_L_MOTOR_MA            0
#define   CURRENT_THRESHOLD_H_DH_12V_MA           2500
#define   CURRENT_THRESHOLD_L_DH_12V_MA           0
#define   CURRENT_THRESHOLD_H_VBAT_MA             30000
#define   CURRENT_THRESHOLD_L_VBAT_MA             0
#define   CURRENT_THRESHOLD_H_12V_RESEVE2_MA      10000
#define   CURRENT_THRESHOLD_L_12V_RESEVE2_MA      0
#define   CURRENT_THRESHOLD_H_24V_RESEVE_MA       10000
#define   CURRENT_THRESHOLD_L_24V_RESEVE_MA       0
#else
#define   CURRENT_THRESHOLD_PRINTER_MA          10000
#define   CURRENT_THRESHOLD_DLP_MA              1600
#define   CURRENT_THRESHOLD_2_1_PA_MA           5000
#define   CURRENT_THRESHOLD_PAD_MA              1000
#define   CURRENT_THRESHOLD_5V_RESERVE1_MA      5000
#define   CURRENT_THRESHOLD_X86_MA              1500
#define   CURRENT_THRESHOLD_DH_5V_MA            100
#define   CURRENT_THRESHOLD_SENSOR_MA           1500
#define   CURRENT_THRESHOLD_5V_LED_MA           1200
#define   CURRENT_THRESHOLD_MOTOR_5V_MA         2000
#define   CURRENT_THRESHOLD_MOTOR_MA            10000
#define   CURRENT_THRESHOLD_DH_12V_MA           100
#define   CURRENT_THRESHOLD_VBAT_MA             15000
#define   CURRENT_THRESHOLD_12V_RESEVE2_MA      10000
#define   CURRENT_THRESHOLD_24V_RESEVE_MA       10000
#endif


struct convert_adc_data {
  const mico_adc_t      adc_type;
  const uint8_t         convert_type;
#define                         CONVERT_TYPE_VOLTAGE            0x01
#define                         CONVERT_TYPE_CURRENTS           0x02
#define                         CONVERT_TYPE_TEMP               0x03
  const uint8_t         isNeedDelay;
  uint8_t               err_flag;
  const uint8_t         err_expire_count;
  uint8_t               err_count;
  const float           convert_factor;  
  const uint16_t        threshold_low;
  const uint16_t        threshold_high;
  uint16_t              err_value;
};

typedef struct _VoltageData_t {
  uint16_t              _5V_reserve1_currents;
  uint16_t              _12V_reserve2_currents;
  uint16_t              bat_voltage;
  uint16_t              sys_all_currents;
  uint16_t              dh_12V_currents;
  uint16_t              dh_5V_currents;
  uint16_t              sensor_currents;
  uint16_t              dlp_currents;
  uint16_t              motor_5v_currents;
  uint16_t              _24V_reserve1_currents;
  uint16_t              _2_1_pa_currents;
  uint16_t              pad_currents;
  uint16_t              printer_currents;
  uint16_t              x86_currents;
  uint16_t              motor_currents;
  uint16_t              _5V_led_currents;
  uint16_t              bat_nv_currents;
  uint16_t              _12V_nv_currents;
  uint16_t              router_currents;
  uint16_t              dyp_currents;
  uint16_t              ir_led_currents;
  uint16_t              charger_currents;
  uint16_t              charge_currents;
  uint16_t              _24V_voltage;
  uint16_t              _12V_voltage;
  uint16_t              _5V_voltage;
  uint16_t              _5V_currents;
  uint16_t              _12V_currents;
  uint16_t              _24V_currents;
  uint16_t              ambient_temperature;
  uint16_t              _5V_regulator_temp;
  uint16_t              _12V_regulator_temp;
  uint16_t              _24V_regulator_temp;
} voltageData_t;

typedef struct _voltageConvert_t {
  uint8_t              bat_voltage;
#if 0
  uint8_t              _5V_reserve1;
  uint8_t              _12V_reserve2;
  uint8_t              _24V_all;
  uint8_t              dh_12V;
  uint8_t              dh_5V;
  uint8_t              sensor;
  uint8_t              dlp;
  uint8_t              motor_5v;
  uint8_t              _24V_reserve1;
  uint8_t              _2_1_pa;
  uint8_t              pad;
  uint8_t              printer;
  uint8_t              x86;
  uint8_t              motor;
  uint8_t              _5V_led;   
#endif
 uint8_t                faultBitTemp[4];
#define         AMBIENT_TEMP_FAULT_BIT                  ((uint32_t)0x00000001)
#define         SYS_CURRENTS_FAULT_BIT                  ((uint32_t)0x00000002)
#define         BAT_MOTOR_CURRENTS_FAULT_BIT            ((uint32_t)0x00000004)
#define         MOTOR_CURRENTS_FAULT_BIT                ((uint32_t)0x00000008)
#define         DH_5V_CURRENTS_FAULT_BIT                ((uint32_t)0x00000010)
#define         SENSOR_CURRENTS_FAULT_BIT               ((uint32_t)0x00000020)
#define         DLP_CURRENTS_FAULT_BIT                  ((uint32_t)0x00000040)
#define         LEDS_CURRENTS_FAULT_BIT                 ((uint32_t)0x00000080)
#define         PAD_CURRENTS_FAULT_BIT                  ((uint32_t)0x00000100)
#define         X86_CURRENTS_FAULT_BIT                  ((uint32_t)0x00000200)
#define         PA_2_1_CURRENTS_FAULT_BIT               ((uint32_t)0x00000400)
#define         DH_12V_CURRENTS_FAULT_BIT               ((uint32_t)0x00000800)
#define         PRINTER_CURRENTS_FAULT_BIT              ((uint32_t)0x00001000)
#define         RES_5V_CURRENTS_FAULT_BIT               ((uint32_t)0x00002000)
#define         RES_12V_CURRENTS_FAULT_BIT              ((uint32_t)0x00004000)
#define         RES_24V_CURRENTS_FAULT_BIT              ((uint32_t)0x00008000)
#define         BAT_NV_CURRENTS_FAULT_BIT               ((uint32_t)0x00010000)
#define         _12V_NV_CURRENTS_FAULT_BIT              ((uint32_t)0x00020000)
#define         ROUTER_CURRENTS_FAULT_BIT               ((uint32_t)0x00040000)
#define         DYP_CURRENTS_FAULT_BIT                  ((uint32_t)0x00080000)
#define         IRLED_CURRENTS_FAULT_BIT                ((uint32_t)0x00100000)
#define         CHARGE_CURRENTS_FAULT_BIT               ((uint32_t)0x00200000)
#define         BATIN_CURRENTS_FAULT_BIT                ((uint32_t)0x00400000)
#define         _24V_VOLTAGE_FAULT_BIT                  ((uint32_t)0x00800000)
#define         _12V_VOLTAGE_FAULT_BIT                  ((uint32_t)0x01000000)
#define         _5V_VOLTAGE_FAULT_BIT                   ((uint32_t)0x02000000)
#define         _5V_CURRENTS_FAULT_BIT                  ((uint32_t)0x04000000)
#define         _12V_CURRENTS_FAULT_BIT                 ((uint32_t)0x08000000)
#define         _24V_CURRENTS_FAULT_BIT                 ((uint32_t)0x10000000)
#define         _5V_REGULATOR_TEMP_FAULT_BIT            ((uint32_t)0x20000000)
#define         _12V_REGULATOR_TEMP_FAULT_BIT           ((uint32_t)0x40000000)
#define         _24V_REGULATOR_TEMP_FAULT_BIT           ((uint32_t)0x80000000)
} voltageConvertData_t;

typedef struct _errCurSaveToFlash_t {
  uint32_t              errTime;
  uint16_t              batVoltage;
  uint16_t              totalCur;
  uint8_t               errChannel;
  uint16_t              errCur;
  uint8_t               errTimes;
  uint8_t               checksum;
} errCurSaveToFlash_t;

typedef struct _voltageDebug_t {
  uint8_t               printType;
#define                         PRINT_NO        0x00
#define                         PRINT_ONCE      0x01
#define                         PRINT_PEROID    0x02
#ifdef  VOLTAGE_DEBUG
  uint8_t               printMaxType;
#define                         RESET_MAX_BUF   0x03
#endif
  uint16_t              peroid;
  uint32_t              startTime; 
  uint8_t               uploadRate;
  uint8_t               isNeedUpload;
  uint32_t              uploadFlagTime;
} voltageDebug_t;

extern voltageData_t *voltageConvert;
extern voltageConvertData_t *voltageConvertData;
extern voltageDebug_t voltageDebug;

OSStatus VolDetect_Init( void );
void VolDetect_Tick( void );

#endif

