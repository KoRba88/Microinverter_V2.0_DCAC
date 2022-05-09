/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HVDC_ADC1_IN10_Pin GPIO_PIN_0
#define HVDC_ADC1_IN10_GPIO_Port GPIOF
#define VMON_VSUPPLY_Pin GPIO_PIN_1
#define VMON_VSUPPLY_GPIO_Port GPIOF
#define PGOOD_DCDC_BOOST_Pin GPIO_PIN_10
#define PGOOD_DCDC_BOOST_GPIO_Port GPIOG
#define LF_MOS_COMP3_INP_Pin GPIO_PIN_0
#define LF_MOS_COMP3_INP_GPIO_Port GPIOA
#define HF_MOS_COMP1_INP_Pin GPIO_PIN_1
#define HF_MOS_COMP1_INP_GPIO_Port GPIOA
#define HVDC_OPAMP1_VOUT_Pin GPIO_PIN_2
#define HVDC_OPAMP1_VOUT_GPIO_Port GPIOA
#define HVDC_OPAMP1_VINN_Pin GPIO_PIN_3
#define HVDC_OPAMP1_VINN_GPIO_Port GPIOA
#define TMON_NTC_Pin GPIO_PIN_4
#define TMON_NTC_GPIO_Port GPIOA
#define I_HVDC_ADC2_IN13_Pin GPIO_PIN_5
#define I_HVDC_ADC2_IN13_GPIO_Port GPIOA
#define I_HVDC_OPAMP2_OUT_Pin GPIO_PIN_6
#define I_HVDC_OPAMP2_OUT_GPIO_Port GPIOA
#define HVDC_OPAMP1_VINP_Pin GPIO_PIN_7
#define HVDC_OPAMP1_VINP_GPIO_Port GPIOA
#define I_HVDC_OPAMP2_COMP4_Pin GPIO_PIN_0
#define I_HVDC_OPAMP2_COMP4_GPIO_Port GPIOB
#define DRV_HF_MOS_BOTT_Pin GPIO_PIN_8
#define DRV_HF_MOS_BOTT_GPIO_Port GPIOA
#define OUT_OC_BOTT_Pin GPIO_PIN_9
#define OUT_OC_BOTT_GPIO_Port GPIOA
#define DRV_LF_MOS_BOTT_Pin GPIO_PIN_10
#define DRV_LF_MOS_BOTT_GPIO_Port GPIOA
#define TRIG_ADC_BOTT_Pin GPIO_PIN_11
#define TRIG_ADC_BOTT_GPIO_Port GPIOA
#define TWO_LEVEL_OC_Pin GPIO_PIN_12
#define TWO_LEVEL_OC_GPIO_Port GPIOA
#define OC_IN_Pin GPIO_PIN_15
#define OC_IN_GPIO_Port GPIOA
#define TOP_LF_CHP_TIM16_CH1N_Pin GPIO_PIN_6
#define TOP_LF_CHP_TIM16_CH1N_GPIO_Port GPIOB
#define OV_OUT_BOTT_Pin GPIO_PIN_7
#define OV_OUT_BOTT_GPIO_Port GPIOB
#define TOP_HF_CHP_TIM16_CH1_Pin GPIO_PIN_8
#define TOP_HF_CHP_TIM16_CH1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BB(reg) ((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(reg) - PERIPH_BASE) * 32U))

#define DMA1_CLEAR_HTIF1 (BB(DMA1->IFCR)[2])
#define DMA1_CLEAR_CTCIF1 (BB(DMA1->IFCR)[1])
#define DMA1_CLEAR_GIF1 (BB(DMA1->IFCR)[0])
#define ADC_BUFFER_SIZE 1
#define ADC_CODE 4095
#define VREF 3290
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF75A8))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF75CA))

#define SPI_RX_BUFFER_SIZE 3
#define SPI_TX_BUFFER_SIZE 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
