/* 
*  Author: Adam Huang
*  Date:2016/6/8
*/
#include "voltage_detect.h"
#include <stdlib.h>
#include "stm32f1xx_powerboard.h"
#include "app_platform.h"
#include "serial_leds.h"
#include "protocol.h"
#include "multi_channel_detection.h"

#define vol_detect_log(M, ...) custom_log("VolDetect", M, ##__VA_ARGS__)
#define vol_detect_log_trace() custom_log_trace("VolDetect")

static voltageData_t ramRawVoltageData, ramVoltageConvert, ramFaultTime;
voltageData_t *rawVoltageData = NULL;//&ramRawVoltageData;
voltageData_t *voltageConvert = NULL;//&ramVoltageConvert;
voltageData_t *faultTime = NULL;//&ramFaultTime;

#ifdef VOLTAGE_DEBUG
static voltageData_t ramTempMaxVoltageData;
static voltageData_t *tempMaxVoltageData = NULL;
#endif

voltageConvertData_t *voltageConvertData;
voltageDebug_t voltageDebug;

static void computeVoltage( void );

typedef enum {
  CURRENTS_5V_RES = 0,
  CURRENTS_12V_RES,
  CURRENTS_BAT_NV,
  CURRENTS_12V_NV,
  CURRENTS_ROUTER,
  CURRENTS_DYP,
  CURRENTS_SENSOR,
  CURRENTS_DLP,
  CURRENTS_MOTOR,
  CURRENTS_24V_RES,
  CURRENTS_2_1_PA,
  CURRENTS_PAD,
  CURRENTS_PRINTER,
  CURRENTS_X86,
  CURRENTS_IRLED,
  CURRENTS_LEDS,
  CURRENTS_CHARGE,
  CURRENTS_BATIN,
  CURRENTS_VBUS,
  CURRENTS_BAT_MOTOR,
  TEMP_24V_TS,
  TEMP_12V_TS,
  TEMP_5V_TS,
  TEMP_AIR_TS,
  CURRENTS_24V_ALL,
  CURRENTS_12V_ALL,
  CURRENTS_5V_ALL,
  VOLTAGE_24V,
  VOLTAGE_12V,
  VOLTAGE_5V,
  VOLTAGE_BAT,
} adc_channel_t ;

const struct convert_adc_data convert_data[] = {
  [CURRENTS_5V_RES] = 
  {
    .adc_type           = MICO_ADC_5V_RES1,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_12V_RES] = 
  {
    .adc_type           = MICO_ADC_12V_RES2,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_BAT_NV] = 
  {
    .adc_type           = MICO_ADC_BAT_NV,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_ROUTER] = 
  {
    .adc_type           = MICO_ADC_ROUTER,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_DYP] = 
  {
    .adc_type           = MICO_ADC_DYP,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_SENSOR] = 
  {
    .adc_type           = MICO_ADC_SENSOR,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_DLP] = 
  {
    .adc_type           = MICO_ADC_DLP,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_MOTOR] = 
  {
    .adc_type           = MICO_ADC_MOTOR,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_24V_RES] = 
  {
    .adc_type           = MICO_ADC_24V_RES1,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_2_1_PA] = 
  {
    .adc_type           = MICO_ADC_2_1_PA,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_PAD] = 
  {
    .adc_type           = MICO_ADC_PAD,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_PRINTER] = 
  {
    .adc_type           = MICO_ADC_PRINTER,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_X86] = 
  {
    .adc_type           = MICO_ADC_X86,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_IRLED] = 
  {
    .adc_type           = MICO_ADC_IRLED,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_LEDS] = 
  {
    .adc_type           = MICO_ADC_LEDS,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_CHARGE] = 
  {
    .adc_type           = MICO_ADC_CHARGE,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 50,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_BATIN] = 
  {
    .adc_type           = MICO_ADC_BATIN,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 50,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_VBUS] = 
  {
    .adc_type           = MICO_ADC_VBUS,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 50,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_BAT_MOTOR] = 
  {
    .adc_type           = MICO_ADC_BAT_MOTOR,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 0,
    .err_expire_count   = 1,
    .convert_factor     = 50,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [TEMP_24V_TS] = 
  {
    .adc_type           = MICO_ADC_24V_TS,
    .convert_type       = CONVERT_TYPE_TEMP,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 1,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [TEMP_AIR_TS] = 
  {
    .adc_type           = MICO_ADC_AIR_TS,
    .convert_type       = CONVERT_TYPE_TEMP,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 1,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_24V_ALL] = 
  {
    .adc_type           = MICO_ADC_24V_ALL,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_12V_ALL] = 
  {
    .adc_type           = MICO_ADC_12V_ALL,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [CURRENTS_5V_ALL] = 
  {
    .adc_type           = MICO_ADC_5V_ALL,
    .convert_type       = CONVERT_TYPE_CURRENTS,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [VOLTAGE_24V] = 
  {
    .adc_type           = MICO_ADC_VDET_24V,
    .convert_type       = CONVERT_TYPE_VOLTAGE,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [VOLTAGE_12V] = 
  {
    .adc_type           = MICO_ADC_VDET_12V,
    .convert_type       = CONVERT_TYPE_VOLTAGE,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [VOLTAGE_5V] = 
  {
    .adc_type           = MICO_ADC_VDET_5V,
    .convert_type       = CONVERT_TYPE_VOLTAGE,
    .isNeedDelay        = 2,
    .err_expire_count   = 1,
    .convert_factor     = 5,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
  [VOLTAGE_BAT] = 
  {
    .adc_type           = MICO_ADC_VDET_BAT,
    .convert_type       = CONVERT_TYPE_VOLTAGE,
    .isNeedDelay        = 1,
    .err_expire_count   = 1,
    .convert_factor     = 11,
    .threshold_low      = 0,
    .threshold_high     = 0xFFFF,
  },
};

OSStatus VolDetect_Init( void )
{
  OSStatus err = kNoErr;  
   
  rawVoltageData = &ramRawVoltageData;
  memset( rawVoltageData, 0x0, sizeof(voltageData_t) );

  voltageConvert = &ramVoltageConvert;
  memset( voltageConvert, 0x0, sizeof(voltageData_t) );

  faultTime = &ramFaultTime;
  memset( faultTime, 0x0, sizeof(voltageData_t) );
#ifdef VOLTAGE_DEBUG
  tempMaxVoltageData = &ramTempMaxVoltageData;
  memset( tempMaxVoltageData, 0x0, sizeof(voltageData_t) );
  if( tempMaxVoltageData )
  {
    tempMaxVoltageData->bat_voltage = 10000;//set temp max voltage is 100v
  }
#endif
  voltageConvertData = (voltageConvertData_t *)malloc( sizeof(voltageConvertData_t) );
  require_action( voltageConvertData, exit, err = kNoMemoryErr );
  memset( voltageConvertData, 0x0, sizeof(voltageConvertData_t) );
  if( voltageConvertData )
  {
  }
  analog_multiplexer_init();
  err = multi_channel_adc_dma_init();
  require_noerr_quiet( err, exit );
exit:
  if( kNoErr != err )
  {
    vol_detect_log("voltage detection init error!");
  }
  else
    vol_detect_log("voltage detection init success!");
  return err;
}

static uint16_t processChannelsData( adc_channel_t type )
{
  uint16_t      readData;
  
  struct convert_adc_data *pConvertAdcData = (struct convert_adc_data *)&convert_data[type];
  
  if( pConvertAdcData->isNeedDelay )
  {
    select_multi_channel( pConvertAdcData->adc_type );
    HAL_Delay(1);//10ms
  }
  
  readData = (uint16_t)(read_channel_values_mV( pConvertAdcData->adc_type ) * pConvertAdcData->convert_factor );
  
  if( readData > pConvertAdcData->threshold_high || readData < pConvertAdcData->threshold_low )
  {
    pConvertAdcData->err_count ++;
    if( pConvertAdcData->err_count > pConvertAdcData->err_expire_count )
    {
      pConvertAdcData->err_count = 0;
      pConvertAdcData->err_flag = 1;
      pConvertAdcData->err_value = readData;
    } 
  }
  else
  {
    if( pConvertAdcData->err_count || pConvertAdcData->err_flag )
    {
       pConvertAdcData->err_count = 0;
       pConvertAdcData->err_flag = 0;
    }
  }
  return readData;
}

static uint8_t  sample_index = 0;
static void computeVoltage( void )
{  
  voltageConvert->bat_voltage = processChannelsData( VOLTAGE_BAT );
  voltageConvertData->bat_voltage = voltageConvert->bat_voltage/100;
  if( voltageConvertData->bat_voltage > 30 )
  {
     boardStatus->vBatLevel = (uint8_t)(voltageConvert->bat_voltage/100.0 + 170);
  }
  else if( voltageConvertData->bat_voltage > 20 )
  {
     boardStatus->vBatLevel = (uint8_t)(10*(voltageConvert->bat_voltage/100.0) - 100);
  }
  else
  {
     boardStatus->vBatLevel = (uint8_t)(5*(voltageConvert->bat_voltage/100.0));
  }
  
  voltageConvert->sys_all_currents = processChannelsData( CURRENTS_VBUS );
  voltageConvert->sensor_currents = processChannelsData( CURRENTS_SENSOR );
  voltageConvert->dlp_currents = processChannelsData( CURRENTS_DLP );
  voltageConvert->motor_5v_currents = processChannelsData( CURRENTS_MOTOR );
  voltageConvert->_2_1_pa_currents = processChannelsData( CURRENTS_2_1_PA );
  voltageConvert->pad_currents = processChannelsData( CURRENTS_PAD );
  voltageConvert->printer_currents = processChannelsData( CURRENTS_PRINTER );
  voltageConvert->x86_currents = processChannelsData( CURRENTS_X86 );
  voltageConvert->motor_currents = processChannelsData( CURRENTS_BAT_MOTOR );
  voltageConvert->_5V_led_currents = processChannelsData( CURRENTS_LEDS );
  voltageConvert->bat_nv_currents = processChannelsData( CURRENTS_BAT_NV );
  voltageConvert->_12V_nv_currents = processChannelsData( CURRENTS_12V_NV );    
  voltageConvert->router_currents = processChannelsData( CURRENTS_ROUTER );
  voltageConvert->dyp_currents = processChannelsData( CURRENTS_DYP );
  voltageConvert->ir_led_currents = processChannelsData( CURRENTS_IRLED );
  voltageConvert->charger_currents = processChannelsData( CURRENTS_CHARGE );
  voltageConvert->charge_currents = processChannelsData( CURRENTS_BATIN );
  voltageConvert->charger_currents = processChannelsData( CURRENTS_CHARGE );
  switch( sample_index )
  {
  case VOLTAGE_24V:
    voltageConvert->_24V_voltage = processChannelsData( VOLTAGE_24V );
    sample_index = VOLTAGE_12V;
  break;
  case VOLTAGE_12V:
    voltageConvert->_12V_voltage = processChannelsData( VOLTAGE_12V );
    sample_index = CURRENTS_5V_ALL;
  break;
  case CURRENTS_5V_ALL:
    voltageConvert->_5V_currents = processChannelsData( CURRENTS_5V_ALL );
    sample_index = CURRENTS_12V_ALL;
  break;
  case CURRENTS_12V_ALL:
    voltageConvert->_12V_currents = processChannelsData( CURRENTS_12V_ALL );
    sample_index = CURRENTS_24V_ALL;
  break;
  case CURRENTS_24V_ALL:
    voltageConvert->_24V_currents = processChannelsData( CURRENTS_24V_ALL );
    sample_index = TEMP_AIR_TS;
  break;
  case TEMP_AIR_TS:
    voltageConvert->ambient_temperature = processChannelsData( TEMP_AIR_TS );
    sample_index = TEMP_5V_TS;
  break;
  case TEMP_5V_TS:
    voltageConvert->_5V_regulator_temp = processChannelsData( TEMP_5V_TS );
    sample_index = TEMP_12V_TS;
  break;
  case TEMP_12V_TS:
    voltageConvert->_12V_regulator_temp = processChannelsData( TEMP_12V_TS );
    sample_index = TEMP_24V_TS;
  break;
  case TEMP_24V_TS:
    voltageConvert->_24V_regulator_temp = processChannelsData( TEMP_24V_TS );
    sample_index = VOLTAGE_24V;
  break;
  }
}

static uint32_t lowVoltageStartTime = 0;

void VolDetect_Tick( void )
{
    computeVoltage();
    if( ( YES == voltageDebug.isNeedUpload ) && IS_SEND_RATE_DATA( voltageDebug.uploadRate ) )
    {     
      if( voltageDebug.uploadRate == SEND_RATE_SINGLE )
      {
        uploadCurrentInformation( voltageConvert );
        voltageDebug.isNeedUpload = NO;
      }
      else
      {
        if( (os_get_time() - voltageDebug.uploadFlagTime) >= sendRateToTime(voltageDebug.uploadRate) )
        {
          voltageDebug.uploadFlagTime = os_get_time();
          uploadCurrentInformation( voltageConvert );
        }
      }
    }
#if 1
    if( PRINT_ONCE == voltageDebug.printType || PRINT_PEROID == voltageDebug.printType )
    {
      if( voltageDebug.printType == PRINT_ONCE )
      {
          voltageDebug.printType = PRINT_NO;
      }
      if( PRINT_PEROID == voltageDebug.printType )
      {
        if( os_get_time() - voltageDebug.startTime < voltageDebug.peroid/SYSTICK_PERIOD )
        {
          return;
        }       
      }
      printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
             voltageConvert->bat_voltage,\
             voltageConvert->sys_all_currents,\
             voltageConvert->dh_12V_currents,\
             voltageConvert->dh_5V_currents,\
             voltageConvert->sensor_currents,\
             voltageConvert->dlp_currents,\
             voltageConvert->motor_5v_currents,\
             voltageConvert->_2_1_pa_currents,\
             voltageConvert->pad_currents,\
             voltageConvert->printer_currents,\
             voltageConvert->x86_currents,\
             voltageConvert->motor_currents,\
             voltageConvert->_5V_led_currents,\
             voltageConvert->_5V_reserve1_currents,\
             voltageConvert->_12V_reserve2_currents,\
             voltageConvert->_24V_reserve1_currents  );
      if( PRINT_PEROID == voltageDebug.printType )
      {
        voltageDebug.startTime = os_get_time();
        //vol_detect_log("print peroid = %d ms",voltageDebug.peroid );
      }
    }
#ifdef  VOLTAGE_DEBUG
    if( PRINT_ONCE == voltageDebug.printMaxType || RESET_MAX_BUF == voltageDebug.printMaxType )
    {
      if( voltageDebug.printMaxType == PRINT_ONCE )
      {
          voltageDebug.printMaxType = PRINT_NO;
      }
      if( RESET_MAX_BUF == voltageDebug.printMaxType )
      {
         memset(tempMaxVoltageData, 0x0, sizeof(voltageData_t));
        if( tempMaxVoltageData )
        {
          tempMaxVoltageData->bat_voltage = 10000;//set temp max voltage is 100v
        }
        voltageDebug.printMaxType = PRINT_NO;
      }
      vol_detect_log("min vbat = %.2f V", tempMaxVoltageData->bat_voltage/100.0);
      vol_detect_log("max sys_all = %d mA", tempMaxVoltageData->sys_all_currents);
      vol_detect_log("max dh_12V = %d mA", tempMaxVoltageData->dh_12V_currents);
      vol_detect_log("max dh_5v = %d mA", tempMaxVoltageData->dh_5V_currents);
      vol_detect_log("max sensor = %d mA", tempMaxVoltageData->sensor_currents);
      vol_detect_log("max dlp = %d mA", tempMaxVoltageData->dlp_currents);
      vol_detect_log("max motor_5v = %d mA", tempMaxVoltageData->motor_5v_currents);
      vol_detect_log("max _2_1_pa = %d mA", tempMaxVoltageData->_2_1_pa_currents);
      vol_detect_log("max pad = %d mA", tempMaxVoltageData->pad_currents);
      vol_detect_log("max printer = %d mA", tempMaxVoltageData->printer_currents);
      vol_detect_log("max x86  = %d mA", tempMaxVoltageData->x86_currents);
      vol_detect_log("max motor = %d mA", tempMaxVoltageData->motor_currents);
      vol_detect_log("max _5V_led = %d mA", tempMaxVoltageData->_5V_led_currents);
      vol_detect_log("max _5V_reserve1 = %d mA", tempMaxVoltageData->_5V_reserve1_currents);
      vol_detect_log("max _12V_reserve2 = %d mA", tempMaxVoltageData->_12V_reserve2_currents);
      vol_detect_log("max _24V_reserve1 = %d mA", tempMaxVoltageData->_24V_reserve1_currents);
    }
#endif //#ifdef  VOLTAGE_DEBUG
#endif // #if 1
    if( SWITCH_ON == switch_user->switchOnOff )
    {
      if( voltageConvert->bat_voltage < VBAT_LOW_POWER_LEVEL )
      {
        boardStatus->sysStatus |= STATE_IS_LOW_POWER;
      }
      else
      {
        boardStatus->sysStatus &= ~STATE_IS_LOW_POWER;
      }
         
      if( voltageConvert->bat_voltage < VBAT_POWER_OFF_LEVEL )
      {
        if( lowVoltageStartTime == 0)
        {
          lowVoltageStartTime = os_get_time();
        }
        if( os_get_time() - lowVoltageStartTime >= 5000/SYSTICK_PERIOD )
        {
          PowerOffDevices();
          lowVoltageStartTime = 0;
        }
      }
      else
      {
        if( lowVoltageStartTime != 0 )
        {
          lowVoltageStartTime = 0;
        }
      }
    }
}

#if defined ( MOTOR_CURRENTS_TEST )
#define ENABLE_INTERRUPTS   __asm("CPSIE i")  /**< Enable interrupts  */
#define DISABLE_INTERRUPTS  __asm("CPSID i")  /**< Disable interrupts  */
__IO static uint32_t errCount = 0;
uint16_t tempErrs, tempErrsMax;
uint16_t errCache[60];
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  //vol_detect_log("occur ADC watchdog"); 
  
  DISABLE_INTERRUPTS;
  errCount = 0;
  tempErrsMax = 0;
  while( averageADCxConvertedValues(8) > 170 )
  {
    tempErrs = averageADCxConvertedValues(8);
    if( tempErrs > tempErrsMax )
    {
      tempErrsMax = tempErrs;
    }
    while( tempErrs == averageADCxConvertedValues(8) ){;}
    printf("%d\t",\
    (uint16_t)((tempErrs/4096.0*3.3)/0.0498*0.73*1000));
    errCount ++;
  }
  printf("%d:%d\r\n",errCount,\
    (uint16_t)((tempErrsMax/4096.0*3.3)/0.0498*0.73*1000));
  ENABLE_INTERRUPTS;
}
#endif
/*********************END OF FILE**************/
