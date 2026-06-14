/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI197V1
 * - NUCLEO_F411RE + STEVAL-MKI197V1
 * - DISCOVERY_SPC584B + STEVAL-MKI197V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F411RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsox_reg.h"
#include "main.h"
#include "math.h"

#include "arm_math.h"
#include "math_helper.h"

#if defined(NUCLEO_F411RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          10 	//ms
#define NUM_TAPS  			  53 	//definiamo il numero di taps come nell'offline_test.m
#define BLOCKSIZE			  104
#define WINDOW_SIZE 		  40 	//definiamo la window_size per il calcolo dell'Rms
#define THRESHOLD			  65 	//definiamo la threshold per l'accensione del led
/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static float32_t firStateF32[BLOCKSIZE + NUM_TAPS - 1];
float32_t outputF32[104];				//vettore d'uscita dal filtro FIR
float massimo_decimo_meridio[104]; 		//massimo dei valori delle accelerazioni sui tre assi
float w_avg;
float Rms[104];
float accRms=0;
uint32_t blocksize=BLOCKSIZE;
float32_t x_in[104],y_in[104],z_in[104];		//vettori in cui salvo le acquisizioni delle accelerazioni
int i=0;
uint8_t charToRead=0;

//COEFFICIENTI FILTRO FIR
const float32_t firCoeffs32[NUM_TAPS] = {
		3.48E-18,-0.000376274,-0.000821011,-0.001346422,-0.001905514,-0.002375337,-0.002562671,-0.002235727,-0.001179218,0.000736147,0.003483801,0.006827522,0.01029927,0.013219783,0.014765728,0.014078279,0.010399121,0.003212854,-0.007628788,-0.021824744,-0.038602992,-0.056768917,-0.07482175,-0.091125417,-0.104110264,-0.112475936,0.8844602,-0.112475936,-0.104110264,-0.091125417,-0.07482175,-0.056768917,-0.038602992,-0.021824744,-0.007628788,0.003212854,0.010399121,0.014078279,0.014765728,0.013219783,0.01029927,0.006827522,0.003483801,0.000736147,-0.001179218,-0.002235727,-0.002562671,-0.002375337,-0.001905514,-0.001346422,-0.000821011,-0.000376274,3.48E-18
};

int __io_putchar(int ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */

	while(!(USART2->SR & USART_SR_TC));

	LL_USART_TransmitData8(USART2, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (LL_USART_IsActiveFlag_TC(USART2) == 0)
	{}

	return ch;
}



/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/

void lsm6dsox_read_data_polling(void)
{

  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = 0;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_104Hz);		//settaggio della frequenza di acquisizione del sensore a 104Hz
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_104Hz);
  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dsox_xl_hp_path_on_out_set(&dev_ctx, LSM6DSOX_LP_ODR_DIV_100);
  lsm6dsox_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  arm_fir_instance_f32 S;																//istanziamento struttura per il filtro fir
  arm_fir_init_f32(&S, NUM_TAPS,(float32_t *)&firCoeffs32[0],&firStateF32[0],blocksize);//settaggio del filtro fir

  LL_USART_EnableIT_RXNE(USART2);														//enable interrupt per USART
  LL_USART_Enable(USART2);
  /* Read samples in polling mode (no int) */
  while (1) {
    uint8_t reg;
    /* Read output only if new xl value is available */
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);



    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

      acceleration_mg[0] =lsm6dsox_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =lsm6dsox_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =lsm6dsox_from_fs2_to_mg(data_raw_acceleration[2]);

      x_in[i]=acceleration_mg[0];											//salvo i 104 valori delle accelerazioni in tre vettori
      y_in[i]=acceleration_mg[1];
      z_in[i]=acceleration_mg[2];
      i++;

if( x_in[i]>y_in[i]) massimo_decimo_meridio[i]=x_in[i];			//calcolo il massimo tra le tre coordinate
 if(y_in[i]>x_in[i]) massimo_decimo_meridio[i]=y_in[i];
else massimo_decimo_meridio[i]=z_in[i];


          if(i==104){			//quando arrivo a 104 valori filtro il vettore



         		  arm_fir_f32(&S, massimo_decimo_meridio, outputF32, blocksize);	 //filtro il massimo delle coordinate


         		 for(int s=0; s<104-WINDOW_SIZE;s++){				//calcolo l'Rms del segnale filtrato
         		          	 for(int k=s; k<WINDOW_SIZE+s;k++){
         		          		 accRms+=sqrt(outputF32[k]*outputF32[k]);
         		          	 }
         		          	 Rms[s]  =  accRms/WINDOW_SIZE;
         		          	 if(Rms[s]>=THRESHOLD) {				//applico la THRESHOLD per l'accensione del led verde
         		          		 LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
         		          	  	 accRms = 0;
         		          	 }
         		          	  else if(Rms[s]<THRESHOLD){
         		          		 LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
         		          		 accRms = 0;
         		          	  }

         		           }
         		i=0;		//in questo modo posso sovrascrivere il vettore

          }


         sprintf((char *)tx_buffer, "%4.2f\r\t %4.2f\r\n", outputF32[i], Rms[i]);
          					 tx_com(tx_buffer, strlen((char const *)tx_buffer));



     /* sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));*/
    }

    lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &reg);







  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSOX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
#endif
  i2c_write(LSM6DSOX_I2C_ADD_L, reg, bufp, len);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSOX_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  i2c_read(LSM6DSOX_I2C_ADD_L, reg, bufp, len);
  return 0;
}

/*
 * @brief  platform specific outputs on terminal (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
  print_uart(tx_buffer, len);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
  delay_ms(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
