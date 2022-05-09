/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum
{
  MOSFET_OVERCURRENT,
  SYSTEM_OK,
  MCU_OVERTEMPERATURE,
  MOSFET_OVERTEMPERATURE,
  VIN_OV,
  VIN_UV,
  BUS_OVERCURRENT,
  BUS_OVERVOLTAGE,
  SYSTEM_INIT,
  SPI_TRANSFER_ERROR
};



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp4;

DAC_HandleTypeDef hdac3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint32_t ADC12_BUFFER[ADC_BUFFER_SIZE] = {0};
uint32_t ADC2_BUFFER[ADC_BUFFER_SIZE] = {0};

uint16_t ADC_DRIVERS_MID_VOLTAGE = 0X7FE;
uint16_t IAC_OPAMP_MID_VOLTAGE = 0X7FE;
uint16_t OVERCURRENT_MOS = 0X199; //330 mV = 16.5 A
uint16_t OVERCURRENT_BUS =  0XFFF; //720 mV TESTY// 0XF2B;// 3106 mV = 2 A
#define OC_FILTER 5;

uint16_t I_HVDC_ADC_VAL = 0;
uint16_t V_HVDC_ADC_VAL = 0;

uint16_t VIN_MON = 0;
uint16_t VBAT_VAL = 0;
uint16_t NTC_VAL = 0;
uint16_t INT_TEMP_VAL = 0;


uint8_t aqusitionDMA1_1 = 0;
uint8_t *ptr_aqusitionDMA1_1;
uint8_t frequency_factor = 1;

uint16_t ii = 0;

uint16_t SPI_TxBuffer[SPI_TX_BUFFER_SIZE] = {0};

uint16_t SPI_RxBuffer[SPI_RX_BUFFER_SIZE] = {0};

uint16_t *ptr_SPI_TxBuffer;

uint16_t *ptr_SPI_RxBuffer;

uint8_t OC_PROT_ON = 0;

__IO uint16_t SystemState = SYSTEM_INIT;

__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t ubReceptionComplete = 0;

uint16_t Status_Val1 = 0;
uint16_t Status_Val2 = 0;
uint16_t Status_Val3 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP3_Init(void);
static void MX_COMP4_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t GetLSB(uint32_t intValue);
uint16_t GetMSB(uint32_t intValue);
void spi_w16( SPI_TypeDef *SPIx, uint16_t dat );
void     Activate_SPI(void);
void SPI3_TransferError_Callback(void);
void DMA1_TransmitComplete_Callback(void);
void DMA1_ReceiveComplete_Callback(void);
void SPIx_Transfer2(SPI_TypeDef *SPIx, uint16_t *outp, uint16_t *inp, uint16_t count);
void delay_us(uint32_t delay_us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void TIM1_GPIO(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP3_Init();
  MX_COMP4_Init();
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);


  //DAC3 CHANNEL 1 - COMP1 & COMP3 input  (MOSFET SHOOT THROUGH OVERCURRENT) //
  DAC3 ->DHR12R1 = (uint32_t)(OVERCURRENT_MOS); //DAC3 CHANNEL 1 SET VALUE
  DAC3->CR |= DAC_CR_EN1; //ENABLE DAC1 CHANNEL 1
  while(!(DAC3->SR & DAC_SR_DAC1RDY)); //WAIT UNTIL DAC3 IS READY
  DAC3 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

  //DAC3 CHANNEL 2 - COMP4 input (HVDC BUS OVERCURRENT)
  DAC3 ->DHR12R2 = (uint32_t)(OVERCURRENT_BUS); //DAC3 CHANNEL 2 SET VALUE // 70% max val
  DAC3->CR |= DAC_CR_EN2; //ENABLE DAC3 CHANNEL 2
  while(!(DAC3->SR & DAC_SR_DAC2RDY)); //WAIT UNTIL DAC3 CHANNEL 2 IS READY
  DAC3 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;


  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC12_BUFFER, ADC_BUFFER_SIZE);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  //HAL_GPIO_WritePin(TWO_LEVEL_OC_GPIO_Port, TWO_LEVEL_OC_Pin, SET);




  //HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // HF HIGH SIDE AND NEG VOL MOGSFET CHARGE PUMPS

  //HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1); // LF HIGH SIDE MOSFET CHARGE PUMP

   ptr_aqusitionDMA1_1 = &aqusitionDMA1_1;

   ptr_SPI_TxBuffer = &SPI_TxBuffer[0];
   ptr_SPI_RxBuffer = &SPI_RxBuffer[0];



   //TIM1_GPIO();
   //GPIOA->BRR = (1<<9);
   //GPIOA->BSRR = (1<<12);

   //MX_TIM1_Init();

/*
   for(int i=0;i<200;i++)
   {

	   //delay_us(100000);
	   GPIOA->BSRR = (1<<9);
       GPIOA->BSRR = (1<<12);
	   //delay_us(100000);

	   GPIOA->BRR = (1<<9);
	   GPIOA->BRR = (1<<12);
   }
*/
   //HAL_GPIO_WritePin(OC_OUT_BOTT_GPIO_Port, OC_OUT_BOTT_Pin, RESET);

   HAL_COMP_Start(&hcomp1);
   HAL_COMP_Start(&hcomp3);
   HAL_COMP_Start(&hcomp4);


   //HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_2);
   //HAL_TIMEx_OnePulseN_Start_IT(&htim1, TIM_CHANNEL_2);


   //HAL_Delay(10);

   //TIM1->CR1|=(TIM_CR1_CEN);

   if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC12_BUFFER, ADC_BUFFER_SIZE) != HAL_OK)
   {
     Error_Handler();
   }
   // Enable the SPI3 peripheral.
   SPI3->CR1 |=  ( SPI_CR1_SPE );

   //TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);

   //uint32_t tmp;
   //tmp = TIM_CCER_CC1NE << (TIM_CHANNEL_2 & 0x1FU); /* 0x1FU = 31 bits max shift */
   /* Reset the CCxNE Bit */
   //TIM1->CCER &=  ~tmp;
   /* Set or reset the CCxNE Bit */
   //TIM1->CCER |= (uint32_t)(TIM_CCxN_ENABLE << (TIM_CHANNEL_2 & 0x1FU)); /* 0x1FU = 31 bits max shift */
   //TIM1->BDTR |= TIM_BDTR_MOE; //ENABLE ALL OUTPUTS


   HAL_GPIO_WritePin(OV_OUT_BOTT_GPIO_Port, OV_OUT_BOTT_Pin, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if ((*ptr_aqusitionDMA1_1) == 1)
	  {

		  if(OC_PROT_ON == 0)
		  {
			   HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out To Two Level MOSFET Driver (Low default)
			   HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2); // Second stage overcurrent and Boost/LDO Enable (High default)
			   //HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_2);
			   //HAL_TIMEx_OnePulseN_Start_IT(&htim1, TIM_CHANNEL_2);

		          //TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);

		          //uint32_t tmp;
		          //tmp = TIM_CCER_CC1NE << (TIM_CHANNEL_2 & 0x1FU); /* 0x1FU = 31 bits max shift */
		          /* Reset the CCxNE Bit */
		          //TIM1->CCER &=  ~tmp;
		          /* Set or reset the CCxNE Bit */
		          //TIM1->CCER |= (uint32_t)(TIM_CCxN_ENABLE << (TIM_CHANNEL_2 & 0x1FU)); /* 0x1FU = 31 bits max shift */
		          //TIM1->BDTR |= TIM_BDTR_MOE; //ENABLE ALL OUTPUTS

			   //HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out To Two Level MOSFET Driver (Low default)
			   //HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2); // Second stage overcurrent and Boost/LDO Enable (High default)




			   OC_PROT_ON = 1;
		  }

		  	HAL_GPIO_TogglePin(TOP_HF_CHP_TIM16_CH1_GPIO_Port, TOP_HF_CHP_TIM16_CH1_Pin);
			if (frequency_factor == 1) // divide transmit PWM Frequency by 2 - So 50 kHz / 2  = 25 kHz up to day
			{
			  //HAL_GPIO_WritePin(OV_OUT_BOTT_GPIO_Port, OV_OUT_BOTT_Pin, SET);
			  I_HVDC_ADC_VAL = GetMSB(ADC12_BUFFER[0])*VREF/ADC_CODE;
			  V_HVDC_ADC_VAL = GetLSB(ADC12_BUFFER[0])*VREF/ADC_CODE;


			  //spi_w16( SPI3, I_HVDC_ADC_VAL );
			  //spi_w16( SPI3, V_HVDC_ADC_VAL );
			  //spi_w16( SPI3, INT_TEMP_VAL );

			  //SPIx_Transfer(SPI3,I_HVDC_ADC_VAL,SPI_RxBuffer[0]);
			  //SPIx_Transfer(SPI3,I_HVDC_ADC_VAL,SPI_RxBuffer[0]);
			  //SPIx_Transfer(SPI3,I_HVDC_ADC_VAL,SPI_RxBuffer[0]);

			  //SPI_TxBuffer[0] = VIN_MON;
			  SPI_TxBuffer[0] = SPI_RxBuffer[0];
			  SPI_TxBuffer[1] = SPI_RxBuffer[1];
			  SPI_TxBuffer[2] = SPI_RxBuffer[2];
			  //SPI_TxBuffer[3] = VBAT_VAL;

			  SPIx_Transfer2(SPI3,ptr_SPI_TxBuffer,ptr_SPI_RxBuffer,SPI_TX_BUFFER_SIZE);



			  //HAL_GPIO_WritePin(OV_OUT_BOTT_GPIO_Port, OV_OUT_BOTT_Pin, RESET);

			  HAL_ADCEx_InjectedStart(&hadc2);
			  HAL_ADCEx_InjectedStart(&hadc1);


			  Status_Val1 = SPI_RxBuffer[0];
			  Status_Val2 = SPI_RxBuffer[1];
			  Status_Val3 = SPI_RxBuffer[2];
			  frequency_factor++;
			}

			else
			{
				//HAL_GPIO_WritePin(OV_OUT_BOTT_GPIO_Port, OV_OUT_BOTT_Pin, RESET);
				frequency_factor = 1;
				HAL_ADCEx_InjectedPollForConversion(&hadc2, 1);
				HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);

				INT_TEMP_VAL = __LL_ADC_CALC_TEMPERATURE(VREF,(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)),LL_ADC_RESOLUTION_12B);
				VIN_MON = (HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2))*VREF/ADC_CODE*2.83;
				VBAT_VAL = (HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2))*VREF/ADC_CODE*3;;
				NTC_VAL = (HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1))*VREF/ADC_CODE;

				/*
				  if(Status_Val1 == 0xA || Status_Val2 == 0x1 || Status_Val3 == 0x15) // GRID INSERTION
				  {
					  if(OC_PROT_ON == 1)
					  {

					     COMP1 ->CSR |= COMP_CSR_EN;//Glitch test
					     COMP3 ->CSR |= COMP_CSR_EN;//Glitch test

						HAL_GPIO_WritePin(TOP_LF_CHP_TIM16_CH1N_GPIO_Port, TOP_LF_CHP_TIM16_CH1N_Pin,SET);
						OC_PROT_ON++;
					  }
				  }
				  */
			}


			 /* if (ii < 500)
			  {

				  IAC_ADC_VAL_BUFF[ii]  = IAC_ADC_VAL;
				  ii++;

			  }
			  */
			  //HAL_GPIO_TogglePin(STATUS1_GPIO_Port, STATUS1_Pin);




/*

*/



			*ptr_aqusitionDMA1_1 = 0;

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EXT_IT11;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGINTERL_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VBAT;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */
  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI3 GPIO Configuration
  PB3   ------> SPI3_SCK
  PB4   ------> SPI3_MISO
  PB5   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI3);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP3;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 150;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = (uint32_t)OC_FILTER;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  //TIM1->AF1 &= ~TIM1_AF1_BKINE; //DISABLE
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 169999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 6799;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOP_LF_CHP_TIM16_CH1N_Pin|OV_OUT_BOTT_Pin|TOP_HF_CHP_TIM16_CH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PGOOD_DCDC_BOOST_Pin */
  GPIO_InitStruct.Pin = PGOOD_DCDC_BOOST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PGOOD_DCDC_BOOST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_HF_MOS_BOTT_Pin DRV_LF_MOS_BOTT_Pin */
  GPIO_InitStruct.Pin = DRV_HF_MOS_BOTT_Pin|DRV_LF_MOS_BOTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_ADC_BOTT_Pin */
  GPIO_InitStruct.Pin = TRIG_ADC_BOTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRIG_ADC_BOTT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOP_LF_CHP_TIM16_CH1N_Pin OV_OUT_BOTT_Pin */
  GPIO_InitStruct.Pin = TOP_LF_CHP_TIM16_CH1N_Pin|OV_OUT_BOTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOP_HF_CHP_TIM16_CH1_Pin */
  GPIO_InitStruct.Pin = TOP_HF_CHP_TIM16_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOP_HF_CHP_TIM16_CH1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t GetLSB(uint32_t intValue)
{
	return (intValue & 0x0000FFFF);
}

uint16_t GetMSB(uint32_t intValue)
{
    return ((intValue & 0xFFFF0000)) >> 16;
}

void spi_w16( SPI_TypeDef *SPIx, uint16_t dat ) {
  // Wait for TXE 'transmit buffer empty' bit to be set.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  // Send the bytes.
  *( uint16_t* )&( SPIx->DR ) = dat;
}

void SPIx_Transfer(SPI_TypeDef *SPIx, uint16_t outp, uint16_t inp) {

        while(!(SPIx->SR & SPI_SR_TXE)){};

        *(volatile uint16_t *)&SPIx->DR = outp;

        while(!(SPIx->SR & SPI_SR_RXNE)){};

        inp = *(volatile uint16_t *)&SPIx->DR;

}

void SPIx_Transfer2(SPI_TypeDef *SPIx, uint16_t *outp, uint16_t *inp, uint16_t count) {
    while(count--) {
        while(!(SPIx->SR & SPI_SR_TXE))
            ;
        *(volatile uint16_t *)&SPIx->DR = *outp++;
        while(!(SPIx->SR & SPI_SR_RXNE))
            ;
        *inp++ = *(volatile uint16_t *)&SPIx->DR;
    }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  //HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out To Two Level MOSFET Driver (Low default)
  //HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_2); // Second stage overcurrent and Boost/LDO Enable (High default)

  //HAL_Delay(3000);

  //HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out To Two Level MOSFET Driver (Low default)
  //HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2); // Second stage overcurrent and Boost/LDO Enable (High default)
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIMEx_BreakCallback could be implemented in the user file
   */
}

void Activate_SPI(void)
{
  /* Enable SPI1 */
  LL_SPI_Enable(SPI3);

  /* Enable DMA Channels */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void DMA1_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
  ubReceptionComplete = 1;
}

/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1_TransmitComplete_Callback(void)
{
  /* DMA Tx transfer completed */
  ubTransmissionComplete = 1;
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI3_TransferError_Callback(void)
{
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  /* Set LED2 to Blinking mode to indicate error occurs */
  //LED_Blinking(LED_BLINK_ERROR);
}


void TIM1_GPIO(void)
{

	  GPIO_InitTypeDef GPIO_InitStruct2 = {0};

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, OUT_OC_BOTT_Pin|TWO_LEVEL_OC_Pin, GPIO_PIN_RESET);


	  /*Configure GPIO pins : TOP_LF_CHP_TIM16_CH1N_Pin OV_OUT_BOTT_Pin TOP_HF_CHP_TIM16_CH1_Pin */
	  GPIO_InitStruct2.Pin = OUT_OC_BOTT_Pin|TWO_LEVEL_OC_Pin;
	  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct2.Pull = GPIO_PULLDOWN;
	  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	SystemState = SPI_TRANSFER_ERROR;
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

