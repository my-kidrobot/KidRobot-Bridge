/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_ch1_up;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_pwm(uint32_t, uint8_t) ;

static void ws2812b_set_color(int num, uint8_t r, uint8_t g, uint8_t b) ;
static void ws2812b_update(void) ;


#define WS2812B_NUMBER 6

struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} ws2812b_data_color[WS2812B_NUMBER]; // 24 bit per 1 led
uint8_t ws2812b_sending_flag = 0;

uint8_t reg_offset = 0;
uint8_t reg_data[30];
uint8_t i2c_state = 0;

HAL_StatusTypeDef I2C_Slave_isr(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) {
	if (hi2c->Instance->ISR & (I2C_ISR_ADDR)) {
		/* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
	}
	
	if (hi2c->Instance->ISR & (I2C_ISR_RXNE)) {
		uint8_t data = hi2c->Instance->RXDR;
		if (i2c_state == 0) {
			reg_offset = data;
			i2c_state = 1;
		} else {
		  reg_data[reg_offset++] = hi2c->Instance->RXDR;
		}
	}
	
	if (hi2c->Instance->ISR & (I2C_ISR_TXIS)) {
		hi2c->Instance->TXDR = reg_data[reg_offset++];
	}
	
	if (reg_offset >= sizeof reg_data) {
		reg_offset = 0;
	}
	
	if (hi2c->Instance->ISR & (I2C_ISR_STOPF)) {
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
		
		// flush the transmit data register I2C_TXDR
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
		
		i2c_state = 0;
	}
	
	return HAL_OK;
}

// microsec
uint64_t microsec = 0;

// ADC state
static uint8_t i_loop = 0;

uint16_t adc_value[2] = { 0, 0 };

// IR
uint64_t ir_code = 0;
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
  MX_I2C1_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  // ====|| I2C Slave setup with interrupt ||====
  
  /* Enable Address Acknowledge */
  hi2c1.Instance->CR2 &= ~I2C_CR2_NACK;
	
  // Set isr function
  hi2c1.XferISR = I2C_Slave_isr;
  
  // Enable I2C interrup
  __HAL_I2C_ENABLE_IT(&hi2c1, I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_TXI | I2C_IT_RXI);
  
  // ====|| END ||====
	
	// Start timer
	HAL_TIM_Base_Start_IT(&htim3);
	
	// Reset WS2812B
	memset(ws2812b_data_color, 0, sizeof ws2812b_data_color);
	ws2812b_update();
	
	// Clean reg
	memset(reg_data, 0, sizeof reg_data);
	
	// Start ADC DMA
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_value, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Use to check MCU running
		i_loop++;
		
		
		// CH A
		HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port, MOTOR1_A_Pin, reg_data[0] & 0x80 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port, MOTOR1_B_Pin, reg_data[0] & 0x80 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		set_pwm(TIM_CHANNEL_1, reg_data[0] & 0x7F);
			
		// CH B
		HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port, MOTOR2_A_Pin, reg_data[1] & 0x80 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port, MOTOR2_B_Pin, reg_data[1] & 0x80 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		set_pwm(TIM_CHANNEL_2, reg_data[1] & 0x7F);
		
		// Write flag is set ?
		if ((ws2812b_sending_flag == 0) & (reg_data[14] & 0x01)) {
			ws2812b_set_color(0, reg_data[2], reg_data[3], reg_data[4]);
			ws2812b_set_color(1, reg_data[5], reg_data[6], reg_data[7]);
			ws2812b_set_color(2, reg_data[8], reg_data[9], reg_data[10]);
			ws2812b_set_color(3, reg_data[11], reg_data[12], reg_data[13]);
			ws2812b_update();			
		} else if ((ws2812b_sending_flag == 1) & (reg_data[14] & 0x01)) {
		  // Set flag to low
			reg_data[14] &= ~(0x01);
		}
		
		// HC-SR04p
		static uint8_t hc_sr04_state = 0;
		static uint64_t start_microsec = 0, end_microsec = 0, start_wait_time = 0;
		if (reg_data[15] & 0x80) { // if trig flag is set
			if (hc_sr04_state == 0) {
				// Start trig
				HAL_GPIO_WritePin(HC_SR04p_Trig_GPIO_Port, HC_SR04p_Trig_Pin, 1);
				
				start_microsec = microsec;
				hc_sr04_state = 1;
			} else if (hc_sr04_state == 1) {
				if (microsec > (start_microsec + 10)) {
					// Set Trig to LOW
					HAL_GPIO_WritePin(HC_SR04p_Trig_GPIO_Port, HC_SR04p_Trig_Pin, 0);
					
					start_wait_time = microsec;
					hc_sr04_state = 2;
				}
			} else if (hc_sr04_state == 2) {
				if (HAL_GPIO_ReadPin(HC_SR04p_Echo_GPIO_Port, HC_SR04p_Echo_Pin) == 1) {
					start_microsec = microsec;
					hc_sr04_state = 3;
				} else {
					if (microsec > (start_wait_time + 100000)) { // over 100mS ?
						start_microsec = 0;
						end_microsec = 0;
						hc_sr04_state = 4;
					}
				}
			} else if (hc_sr04_state == 3) {
				if (HAL_GPIO_ReadPin(HC_SR04p_Echo_GPIO_Port, HC_SR04p_Echo_Pin) == 0) {
					end_microsec = microsec;
					hc_sr04_state = 4;
				} else {
					if (microsec > (start_microsec + 100000)) { // over 100mS ?
						start_microsec = 0;
						end_microsec = 0;
						hc_sr04_state = 4;
					}
				}
			} else if (hc_sr04_state == 4) {
				uint16_t distance = (float)(end_microsec - start_microsec) * 0.0173681 * 10.0;
				
				// Update in register
			  reg_data[15] = (reg_data[15]&0xF0)|(distance>>8);
				reg_data[16] = distance;
			
			  reg_data[15] &= ~(0x80);
				
				hc_sr04_state = 0;
			}
		}
		
		/*
		// LINE track update
		reg_data[17] = HAL_GPIO_ReadPin(LINE_TRACK_LEFT_GPIO_Port, LINE_TRACK_LEFT_Pin);
		reg_data[18] = HAL_GPIO_ReadPin(LINE_TRACK_RIGHT_GPIO_Port, LINE_TRACK_RIGHT_Pin);
		*/
		
		// LINE track update
		reg_data[17] = adc_value[0] * 255.0 / 4095.0;
		reg_data[18] = adc_value[1] * 255.0 / 4095.0;
		
		
		// IR
		static uint64_t wait_time = 0;
		if ((HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 0) && (microsec - wait_time) > 100) {
			uint8_t err = 0;
			uint64_t start_time;
			
			// Start ... LOW
			start_time = microsec;
			while(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 0) {
				if ((microsec - start_time) > 5000) { // over 5mS
					err = 1;
					break;
				}
			}
			
			if (err) {
				wait_time = microsec;
				continue;
			}
			
			ir_code = 0;
			
			// Start ... HIGH
			start_time = microsec;
			while(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 1) {
				if ((microsec - start_time) > 5000) { // over 5mS
					err = 1;
					break;
				}
			}
			
			if (err) {
				wait_time = microsec;
				continue;
			}
			
			// loop data 48 bit
			for (int i=0;i<48;i++) {
				start_time = microsec;
			  while(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 0) {
				  if ((microsec - start_time) > 3000) { // over 3mS
					  err = 1;
					  break;
				  }
			  }
			
			  if (err) {
				  wait_time = microsec;
				  break;
			  }
				
				start_time = microsec;
			  while(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 1) {
				  if ((microsec - start_time) > 5000) { // over 5mS
					  err = 1;
					  break;
				  }
		  	}
			
			  if (err) {
				  HAL_Delay(100); // delay 100mS
				  break;
			  }
				
				if ((microsec - start_time) > 1000) { // if over 1000uS
					ir_code |= (1UL << (47-i));   // write 1
				} else {
					ir_code &= ~(1UL << (47-i));  // write 0
				}
		  }
			
			if (err) {
				continue;
			}
			
			// Wait LOW of END
			start_time = microsec;
			while(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == 0) {
				if ((microsec - start_time) > 5000) { // over 5mS
					err = 1;
					break;
				}
			}
			
			if (err) {
				wait_time = microsec;
				continue;
			}
			
			switch(ir_code) {
				case 0x1002c2d: 
					reg_data[20] = 'F';
				  break;
				
				case 0x100acad: 
					reg_data[20] = 'B';
				  break;
				
				case 0x1008485: 
					reg_data[20] = 'L';
				  break;
				
				case 0x1000405: 
					reg_data[20] = 'R';
				  break;
				
				default:
					reg_data[20] = 0;
				
			}
			wait_time = microsec;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2010091A;
  hi2c1.Init.OwnAddress1 = 202;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 960-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 480;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 480;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 60;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 60;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC_SR04p_Trig_Pin|MOTOR2_A_Pin|MOTOR2_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR1_A_Pin|MOTOR1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HC_SR04p_Trig_Pin MOTOR2_A_Pin MOTOR2_B_Pin */
  GPIO_InitStruct.Pin = HC_SR04p_Trig_Pin|MOTOR2_A_Pin|MOTOR2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HC_SR04p_Echo_Pin */
  GPIO_InitStruct.Pin = HC_SR04p_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HC_SR04p_Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR1_A_Pin MOTOR1_B_Pin */
  GPIO_InitStruct.Pin = MOTOR1_A_Pin|MOTOR1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Pin */
  GPIO_InitStruct.Pin = IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void set_pwm(uint32_t Channel, uint8_t value) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, Channel);
	HAL_TIM_PWM_Start(&htim1, Channel);
}

static void ws2812b_set_color(int num, uint8_t r, uint8_t g, uint8_t b) {
	ws2812b_data_color[num].r = r;
	ws2812b_data_color[num].g = g;
	ws2812b_data_color[num].b = b;
}

static void ws2812b_update(void) {
	uint32_t bit0_period = 13;
	uint32_t bit1_period = 40;

	uint32_t tData[50 + (24 * WS2812B_NUMBER)];
	memset(tData, 0, 50);
	
	for (int color_inx=0;color_inx<WS2812B_NUMBER;color_inx++) {
		uint8_t r = ws2812b_data_color[color_inx].r;
		uint8_t g = ws2812b_data_color[color_inx].g;
		uint8_t b = ws2812b_data_color[color_inx].b;
		
		int offset = 50 + (color_inx * 24);
	
		tData[offset + 0] = (g & (1<<7)) ? bit1_period : bit0_period;
		tData[offset + 1] = (g & (1<<6)) ? bit1_period : bit0_period;
		tData[offset + 2] = (g & (1<<5)) ? bit1_period : bit0_period;
		tData[offset + 3] = (g & (1<<4)) ? bit1_period : bit0_period;
		tData[offset + 4] = (g & (1<<3)) ? bit1_period : bit0_period;
		tData[offset + 5] = (g & (1<<2)) ? bit1_period : bit0_period;
		tData[offset + 6] = (g & (1<<1)) ? bit1_period : bit0_period;
		tData[offset + 7] = (g & (1<<0)) ? bit1_period : bit0_period;
		
		tData[offset + 8] = (r & (1<<7)) ? bit1_period : bit0_period;
		tData[offset + 9] = (r & (1<<6)) ? bit1_period : bit0_period;
		tData[offset + 10] = (r & (1<<5)) ? bit1_period : bit0_period;
		tData[offset + 11] = (r & (1<<4)) ? bit1_period : bit0_period;
		tData[offset + 12] = (r & (1<<3)) ? bit1_period : bit0_period;
		tData[offset + 13] = (r & (1<<2)) ? bit1_period : bit0_period;
		tData[offset + 14] = (r & (1<<1)) ? bit1_period : bit0_period;
		tData[offset + 15] = (r & (1<<0)) ? bit1_period : bit0_period;
		
		tData[offset + 16] = (b & (1<<7)) ? bit1_period : bit0_period;
		tData[offset + 17] = (b & (1<<6)) ? bit1_period : bit0_period;
		tData[offset + 18] = (b & (1<<5)) ? bit1_period : bit0_period;
		tData[offset + 19] = (b & (1<<4)) ? bit1_period : bit0_period;
		tData[offset + 20] = (b & (1<<3)) ? bit1_period : bit0_period;
		tData[offset + 21] = (b & (1<<2)) ? bit1_period : bit0_period;
		tData[offset + 22] = (b & (1<<1)) ? bit1_period : bit0_period;
		tData[offset + 23] = (b & (1<<0)) ? bit1_period : bit0_period;
	}
	
	TIM_OC_InitTypeDef sConfig;
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	//sConfig.Pulse        = 0;
	HAL_TIM_PWM_ConfigChannel(&htim17, &sConfig, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, tData, sizeof tData);
	
	ws2812b_sending_flag = 1;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	/*
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	*/
	
	htim17.Instance->CCR1 = 0;
	
	ws2812b_sending_flag = 0;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
