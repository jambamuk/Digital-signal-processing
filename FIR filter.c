/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
//#include "arm_common_tables.h"
//#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define __FPU_PRESENT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
//sampled values (input)
float32_t sampleValues[128];
//sampled values copy(input)
float32_t sampleValuesCopy[128];
//filtered values
float32_t filteredValue[128];
//filtered values- copy
float32_t filteredValueCopy[128];
//output values from fft on sampled values
float32_t fftOutput[64];
//output of fft on filtered values
float32_t fftOutputFiltered[64];

//int counter = 0;
int flagSimulate = 1;
//flag for displaying fft or time domain graph
int isFFT = 0;
//Lookup table for Hamming window coefficients (precomputed with Python). Reduces computation time.
float32_t hammingCoefficients[] = {0.00016105781876405852, 0.00031994869975521544, 0.0004699875926568673, 0.000600869735108455, 0.0006976873976794068,
								   0.0007412869735641744, 0.0007102116412965183, 0.0005841716004881761, 0.0003486557448675082, -6.280296790960751e-19,
								   -0.00044998770918983067, -0.0009708328229796027, -0.0015125468832298183, -0.0020078113805769024, -0.0023774978648667083,
								   -0.0025393475168049087, -0.0024191094937598494, -0.0019629451053162156, -0.0011495203016289486, 1.5185279134756748e-18,
								   0.0014158259747368783, 0.0029787912097362975, 0.0045259777779578176, 0.005863020462973077, 0.006782568147748727,
								   0.007087526928096838, 0.006616934173502753, 0.005271721614091009, 0.0030373098278770228, -2.637438245619114e-18,
								   -0.003645482075329859, -0.007599196755782551, -0.011470836502328635, -0.014805871584587387, -0.017121383267832174,
								   -0.017948777860227322, -0.01687960533647347, -0.0136100852374606, -0.00797976038386098, 3.5489904338781336e-18,
								   0.01013115045388205, 0.02203008770192949, 0.03514556389945731, 0.048792581313126575, 0.062199834408475096, 0.0745668380084052,
								   0.08512580368927543, 0.09320272380525213, 0.09827206734676573, 0.1, 0.09827206734676573, 0.09320272380525213, 0.08512580368927543,
								   0.0745668380084052, 0.062199834408475096, 0.048792581313126575, 0.03514556389945731, 0.02203008770192949, 0.01013115045388205,
								   3.5489904338781336e-18, -0.00797976038386098, -0.0136100852374606, -0.01687960533647347, -0.017948777860227322, -0.017121383267832174,
								   -0.014805871584587387, -0.011470836502328635, -0.007599196755782551, -0.003645482075329859, -2.637438245619114e-18, 0.0030373098278770228,
								   0.005271721614091009, 0.006616934173502753, 0.007087526928096838, 0.006782568147748727, 0.005863020462973077, 0.0045259777779578176,
								   0.0029787912097362975, 0.0014158259747368783, 1.5185279134756748e-18, -0.0011495203016289486, -0.0019629451053162156,
								   -0.0024191094937598494, -0.0025393475168049087, -0.0023774978648667083, -0.0020078113805769024, -0.0015125468832298183,
								   -0.0009708328229796027, -0.00044998770918983067, -6.280296790960751e-19, 0.0003486557448675082, 0.0005841716004881761,
								   0.0007102116412965183, 0.0007412869735641744, 0.0006976873976794068, 0.000600869735108455, 0.0004699875926568673, 0.00031994869975521544,
								   0.00016105781876405852};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */
void displayGraphSetup();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void FIR_Filter();
void simulateSineWave();
float32_t AWGNGenerator();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Interrupt for USER pushbutton.
 * Changes modes for the display, i.e. fft <--> time domain
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

//	if (isFFT == 1) {
//		isFFT = 0; //displays time graph
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
//	} else {
//		isFFT = 1; //displays fft graph
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
//	}
	if (isFFT == 0)
		isFFT = 1;//sine wave of sampled
	else if (isFFT == 1)
		isFFT = 2; //fft of filtered
	else if (isFFT == 2)
		isFFT = 3; //sine wave of filtered
	else
		isFFT = 0;//fft of sampled
}

/**
 * Sets up the graph on the LCD
 */
void displayGraphSetup() {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"Amplitude", CENTER_MODE);
	if (isFFT == 0) {
		BSP_LCD_DisplayChar(0, 50, 'F');
		BSP_LCD_DisplayChar(0, 70, 'R');
		BSP_LCD_DisplayChar(0, 90, 'E');
		BSP_LCD_DisplayChar(0, 110, 'Q');
		BSP_LCD_DisplayChar(0, 130, 'U');
		BSP_LCD_DisplayChar(0, 150, 'E');
		BSP_LCD_DisplayChar(0, 170, 'N');
		BSP_LCD_DisplayChar(0, 190, 'C');
		BSP_LCD_DisplayChar(0, 210, 'Y');
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*)"(Hz)", LEFT_MODE);
	} else if (isFFT == 1) {
		BSP_LCD_DisplayChar(0, 50, 'T');
		BSP_LCD_DisplayChar(0, 70, 'I');
		BSP_LCD_DisplayChar(0, 90, 'M');
		BSP_LCD_DisplayChar(0, 110, 'E');
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"(s)", LEFT_MODE);
	} else if (isFFT == 2) {
		BSP_LCD_DisplayChar(0, 50, 'F');
		BSP_LCD_DisplayChar(0, 70, 'R');
		BSP_LCD_DisplayChar(0, 90, 'E');
		BSP_LCD_DisplayChar(0, 110, 'Q');
		BSP_LCD_DisplayChar(0, 130, 'U');
		BSP_LCD_DisplayChar(0, 150, 'E');
		BSP_LCD_DisplayChar(0, 170, 'N');
		BSP_LCD_DisplayChar(0, 190, 'C');
		BSP_LCD_DisplayChar(0, 210, 'Y');
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*)"(Hz)", LEFT_MODE);
		BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"Filtered", LEFT_MODE);
	} else {
		BSP_LCD_DisplayChar(0, 50, 'T');
		BSP_LCD_DisplayChar(0, 70, 'I');
		BSP_LCD_DisplayChar(0, 90, 'M');
		BSP_LCD_DisplayChar(0, 110, 'E');
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"(s)", LEFT_MODE);
		BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"Filtered", LEFT_MODE);
	}

	BSP_LCD_DrawHLine(25, 30, 200);
	BSP_LCD_DrawVLine(25, 30, 256);

}

/**
 * Generates Additive White Gaussian Noise
 */
float32_t AWGNGenerator()
{
	int temp = 1;
	float32_t val;
	while (temp > 0) {
		val = (float32_t)rand()/RAND_MAX;
		if (val == 0)
			temp = 1;
		else
			temp = -1;
	}
	float32_t val2 = cosf(2*M_PI) * rand() / RAND_MAX;
	return (sqrtf(-2 * log(val)) * val2);
}

/**
 * Simulates the sine wave at frequency = 1.5 kHz and sampling rate of 3 kHz
 */
void simulateSineWave()
{
	float32_t noise = AWGNGenerator();
	static int i= 0;

	if (flagSimulate == 1) {
		for(int k = 0; k < 128; ++k)
			sampleValuesCopy[k] = sampleValues[k] = (float32_t) sin((2 * M_PI * k * 1500)/3000) + noise;
//			sampleValuesCopy[k] = sampleValues[k] = (float32_t) sin((2 * M_PI * k/3000 * 1500)) + noise;
		flagSimulate = 0;
	} else {
		for (int k = 127; k > 0; --k) {
			sampleValues[k] = sampleValues[k-1];
			sampleValuesCopy[k] = sampleValues[k];
		}
		sampleValuesCopy[0] = sampleValues[0] = (float32_t) sin((2 * M_PI * i * 1500)/3000) + noise;
		++i;
		if (i >= 128)
			i = 0;
	}

}

/**
 * FIR filter
 * Implements the Hamming window function with precomputed coefficients.
 * Convolves input samples with coefficients.
 */
void FIR_Filter()
{
	float32_t buff[99];
	for (int i = 0; i < 99; ++i) buff[i] = 0;
	for (int i = 0; i < 128; ++i) {
		float32_t firSum = 0.0f;
	    for (int k = 0; k < 98; ++k)
	    	buff[98-k] = buff[97-k];
	    buff[0] = sampleValues[i];

	    for (int k = 0; k < 99; ++k)
	    	firSum += hammingCoefficients[k] * buff[k];
	    filteredValueCopy[i] = filteredValue[i] = firSum;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//seed random number generator
	srand(time(0));
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
  MX_ADC1_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLUE);

  //max value of fftOutput
  float32_t maxVal, maxValFilter;
  //index of max value of fftOutput
  uint32_t indexMax, indexMaxFilter;
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //sets up graph for fft/time domain
	  displayGraphSetup();

	  //simulate the sine wave:
	  simulateSineWave();

	  //filters the samples:
	  FIR_Filter();

	  //perform fft on unfiltered:
	  arm_cfft_f32(&arm_cfft_sR_f32_len64, sampleValuesCopy, 0, 1);
	  arm_cmplx_mag_f32(sampleValuesCopy, fftOutput, 64);
	  fftOutput[0] = 0;
	  arm_max_f32(fftOutput, 64, &maxVal, &indexMax);

	  //perform fft on filtered:
	  arm_cfft_f32(&arm_cfft_sR_f32_len64, filteredValueCopy, 0, 1);
	  arm_cmplx_mag_f32(filteredValueCopy, fftOutputFiltered, 64);
	  arm_max_f32(fftOutputFiltered, 64, &maxValFilter, &indexMaxFilter);


	  uint16_t y_pos = 30;
	  if (isFFT == 0) {
		  for (size_t i=0; i < 64; ++i) {
			  if (fftOutput[i] >= 0) {
				  BSP_LCD_FillRect(25, y_pos, fftOutput[i]/maxVal*200, 3);
				  if (i % 10 == 0)
					  BSP_LCD_DrawHLine(20, y_pos, 5);
			  }
			  y_pos += 4;
		  }
	  } else if (isFFT == 1){
		  float32_t maxNumTime = sampleValues[0];
		  float32_t minNumTime = sampleValues[0];
		  for (size_t i = 1; i < 128; ++i) {
			  if (sampleValues[i] > maxNumTime)
				  maxNumTime = sampleValues[i];
			  if (sampleValues[i] < minNumTime)
				  minNumTime = sampleValues[i];
		  }

		  for (size_t i = 0; i < 127; i+=2) {
			  float32_t temp1;
			  float32_t temp2;
			  if (sampleValues[i] >= 0)
				  temp1 = sampleValues[i]/maxNumTime * 125 + 100;
			  else if (sampleValues[i] < 0)
				  temp1 = 150 - (sampleValues[i]/minNumTime * 125);

			  if (sampleValues[i+2] >= 0)
				  temp2 = sampleValues[i+2]/maxNumTime * 125 + 100;
			  else if (sampleValues[i+2] < 0)
				  temp2 = 150 - (sampleValues[i+2]/minNumTime * 125);

			  BSP_LCD_DrawLine(temp1, y_pos, temp2, y_pos+4);
			  y_pos += 4;
		  }
	  } else if (isFFT == 2){
		  for (size_t i=0; i < 64; ++i) {
			  if (fftOutput[i] >= 0) {
		  		BSP_LCD_FillRect(25, y_pos, fftOutputFiltered[i]/maxValFilter*200, 3);
		  		if (i % 10 == 0)
		  			BSP_LCD_DrawHLine(20, y_pos, 5);
			  }
		  			  y_pos += 4;
		  }

	  } else {
		  float32_t maxNumTime = filteredValue[0];
		  float32_t minNumTime = filteredValue[0];
		  for (size_t i = 1; i < 128; ++i) {
			  if (filteredValue[i] > maxNumTime)
				  maxNumTime = filteredValue[i];
			  if (filteredValue[i] < minNumTime)
				  minNumTime = filteredValue[i];
		  }

		  for (size_t i = 0; i < 127; ++i) {
			  float32_t temp1;
			  float32_t temp2;
			  if (filteredValue[i] >= 0)
				  temp1 = filteredValue[i]/maxNumTime * 125 + 100;
			  else if (filteredValue[i] < 0)
				  temp1 = 150 - (filteredValue[i]/minNumTime * 125);

			  if (filteredValue[i+1] >= 0)
				  temp2 = filteredValue[i+1]/maxNumTime * 125 + 100;
			  else if (filteredValue[i+1] < 0)
				  temp2 = 150 - (filteredValue[i+1]/minNumTime * 125);

			  BSP_LCD_DrawLine(temp1, y_pos, temp2, y_pos+2);
			  y_pos += 2;
		  }
	  }


	  HAL_Delay(500);
	  BSP_LCD_Clear(LCD_COLOR_BLUE);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
//
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
//
  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
//
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 334;
  hltdc.Init.AccumulatedActiveH = 245;
  hltdc.Init.TotalWidth = 340;
  hltdc.Init.TotalHeigh = 247;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_HS_ID_Pin OTG_HS_DM_Pin OTG_HS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_HS_ID_Pin|OTG_HS_DM_Pin|OTG_HS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_HS_Pin */
  GPIO_InitStruct.Pin = VBUS_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
