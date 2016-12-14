/**
 ******************************************************************************
 * @file    board_init.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    26-Nov-2016
 * @brief   
 ******************************************************************************
*/
#include "board_init.h"
#include "platform.h"
#include "mico_platform.h"

extern const platform_gpio_t            platform_gpio_pins[];
extern const platform_adc_t             platform_adc_peripherals[];
const board_module_enable_t board_module_enable = 
{
  .pin_motor                    = &platform_gpio_pins[MICO_GPIO_MOTOR_EN],
  .pin_sensor                   = &platform_gpio_pins[MICO_GPIO_SENSOR_EN],
  .pin_leds                     = &platform_gpio_pins[MICO_GPIO_LEDS_EN],
  .pin_5v                       = &platform_gpio_pins[MICO_GPIO_5V_EN],
  .pin_12v                      = &platform_gpio_pins[MICO_GPIO_12V_EN],
  .pin_24v                      = &platform_gpio_pins[MICO_GPIO_24V_EN],
};

void board_init_gpios( void )
{
  const board_module_enable_t *module_enable = &board_module_enable;
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  platform_gpio_init( module_enable->pin_5v, &pin_config );
  platform_gpio_init( module_enable->pin_12v, &pin_config );
  platform_gpio_init( module_enable->pin_24v, &pin_config );
}


void board_deinit_gpios( void )
{
  const board_module_enable_t *module_enable = &board_module_enable;
  platform_gpio_deinit( module_enable->pin_5v );
  platform_gpio_deinit( module_enable->pin_12v );
  platform_gpio_deinit( module_enable->pin_24v );
}


void board_gpios_init( void )
{
  platform_pin_config_t pin_config;
  
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_IRLED_PWM, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_5V_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_12V_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_24V_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_MOTOR_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SENSOR_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_LEDS_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_5V_RES_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PAD_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ROUTER_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_2_1_PA_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_DYP_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_X86_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_NV_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_DLP_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_12V_RES_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PRINTER_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_24V_RES_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_BAT_NV_EN, &pin_config );
  
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWR_NV, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWR_DLP, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWR_PAD, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWR_X86, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWR_RES, &pin_config );

  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_IRLED_PWM );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_5V_EN );  
  MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_12V_EN );
  MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_24V_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_MOTOR_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SENSOR_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_LEDS_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_5V_RES_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PAD_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_ROUTER_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_2_1_PA_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_DYP_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_X86_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_NV_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_DLP_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_12V_RES_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PRINTER_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_24V_RES_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_BAT_NV_EN );
  
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PWR_NV );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PWR_DLP );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PWR_PAD );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PWR_X86 );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_PWR_RES );
  
}

void board_adc_dma_init( void *buffer, uint32_t length )
{
  MicoAdcStreamInitializeEarly( MICO_ADC_5V_RES1,16 );
  MicoAdcStreamAddChannel( MICO_ADC_5V_RES1, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_12V_RES2, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_BAT_NV, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_12V_NV, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_ROUTER, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_DYP, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_SENSOR, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_DLP, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_MOTOR, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_24V_RES1, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_2_1_PA, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_PAD, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_PRINTER, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_X86, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_IRLED, 235 );
  MicoAdcStreamAddChannel( MICO_ADC_LEDS, 235 );
  MicoAdcStreamInitializeLate( MICO_ADC_LEDS, buffer, length );
}
