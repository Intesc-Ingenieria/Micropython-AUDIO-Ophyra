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
#include <ST7735_Driver.h>
#include "pdm_filter.h"
#define BUFF_SIZE 1024
//#include <arm_math.h>
//#include "core_cm4.h"
#include "arm_math.h"
#define TEST_LENGTH_SAMPLES  16
#define BLOCK_SIZE            1024
#define NUM_TAPS              129


//include "0410.arm_math.h"
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
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t conta = 0,indice = 0, dato;
uint16_t segundo[64];
uint16_t segundo8[BUFF_SIZE/2];
uint16_t filtrada[16], filtrada2[16];
uint8_t buffer[128];
float fsegundo[1024];
float ffiltrada[16],ffiltrada2[16];

int i = 0,j=0,k=0,m=0;
float firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
float filtro[129] = {
  1.246786212e-07,-6.392520504e-07,-3.496861837e-06,-3.749720463e-06,3.717402251e-06,
   6.24457698e-06,-7.993840882e-06,-1.053305004e-05,1.676874126e-05,1.510273614e-05,
  -3.295551869e-05,-1.696207801e-05,5.957375834e-05,1.024715675e-05,-9.835393575e-05,
  1.442727898e-05,0.0001474380551,-6.962037151e-05,-0.0001983891998,0.0001690689824,
  0.0002332162403,-0.000323457527,-0.0002225039643,0.0005339680938, 0.000126236293,
  -0.000784247648,0.0001012585708, 0.001032550586,-0.000501205679,-0.001206626883,
   0.001095243148, 0.001204437111,-0.001866966253,-0.0009035709663, 0.002743585734,
  0.0001810341055,-0.003582561854, 0.001057010959, 0.004168597516,-0.002838655608,
  -0.004225619603, 0.005088475067, 0.003446045797,-0.007595965173, -0.00153611647,
   0.009997447953,-0.001728131319, -0.01177604217, 0.006443469319,  0.01227827743,
   -0.01253225841,  -0.0107352389,  0.01971703954, 0.006255940069, -0.02752585709,
   0.002287781332,  0.03533204272, -0.01672390662, -0.04242540523,  0.04134583846,
    0.04810500145, -0.09195075184, -0.05177829042,   0.3134599328,   0.5530491471,
     0.3134599328, -0.05177829042, -0.09195075184,  0.04810500145,  0.04134583846,
   -0.04242540523, -0.01672390662,  0.03533204272, 0.002287781332, -0.02752585709,
   0.006255940069,  0.01971703954,  -0.0107352389, -0.01253225841,  0.01227827743,
   0.006443469319, -0.01177604217,-0.001728131319, 0.009997447953, -0.00153611647,
  -0.007595965173, 0.003446045797, 0.005088475067,-0.004225619603,-0.002838655608,
   0.004168597516, 0.001057010959,-0.003582561854,0.0001810341055, 0.002743585734,
  -0.0009035709663,-0.001866966253, 0.001204437111, 0.001095243148,-0.001206626883,
  -0.000501205679, 0.001032550586,0.0001012585708,-0.000784247648, 0.000126236293,
  0.0005339680938,-0.0002225039643,-0.000323457527,0.0002332162403,0.0001690689824,
  -0.0001983891998,-6.962037151e-05,0.0001474380551,1.442727898e-05,-9.835393575e-05,
  1.024715675e-05,5.957375834e-05,-1.696207801e-05,-3.295551869e-05,1.510273614e-05,
  1.676874126e-05,-1.053305004e-05,-7.993840882e-06, 6.24457698e-06,3.717402251e-06,
  -3.749720463e-06,-3.496861837e-06,-6.392520504e-07,1.246786212e-07
};
//PDMFilter_InitStruct Filter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void decimador(uint16_t* , uint16_t* );
void bintofloat(uint16_t* , float* );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ST7735_tenErrCode pfSPI_WRITE (U8* pu8Data, U32 u32Size)
{
	HAL_SPI_Transmit(&hspi1,(uint8_t *)pu8Data, u32Size, 0);
}
ST7735_tenErrCode pfTFT_CS_GPIO(U8 u8State)
{

	if (0 == u8State)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	}
	else if (1 == u8State)
	{
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
}
ST7735_tenErrCode pfTFT_RS_GPIO(U8 u8State)
{
	if (0 == u8State)
	{
		HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_RESET);
	}
	else if (1 == u8State)
	{

		HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_SET);
	}
}
ST7735_tenErrCode pfTFT_RESET_GPIO(U8 u8state)
{
	if (0 == u8state)
	{
		HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
	}
	else if (1 == u8state)
	{
	    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
	}

}
ST7735_tenErrCode pfDELAY_MS(U32 u32Delay)
{
	HAL_Delay(u32Delay);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	ST7735_tstBSPDriver ST7735_BSP;
	ST7735_tstStrDesc ST7735_STR;

	//u_int8_t uartbuf[1];
	//uint16_t s[128];
	//uint16_t dato[2048];



	//float fsegundo[16384],fbuf;
	//char internalFilter[34];
	CHAR str[64];





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
  MX_DAC_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  hi2s2.Instance->I2SPR = 0x002A;
  ST7735_BSP.DELAY_MS = pfDELAY_MS;
  ST7735_BSP.SPI_WRITE = pfSPI_WRITE;
  ST7735_BSP.TFT_CS_GPIO = pfTFT_CS_GPIO;
  ST7735_BSP.TFT_RESET_GPIO = pfTFT_RESET_GPIO;
  ST7735_BSP.TFT_RS_GPIO = pfTFT_RS_GPIO;
  ST7735_enRegisterBSP(&ST7735_BSP);
  ST7735_enInit();
  ST7735_enFillDisplay(0xFFFFFFFF);

  ST7735_STR.pchString= "HOLA";
  ST7735_STR.u32StrColour = 0;
  ST7735_STR.u8XCursor=100;
  ST7735_STR.u8Ycursor=100;

  //HAL_Delay(2000);
  //ST7735_enPrintStr(&ST7735_STR);
  //arm_fir_instance_f32 S;

  arm_fir_decimate_instance_f32 S;
  /*
  float clear[613];
  float l[10124];
  */
  /*
  //__CRC_CLK_ENABLE();
  Filter.Fs = 16000;
  Filter.LP_HZ = 10000;
  Filter.HP_HZ = 5;
  Filter.Out_MicChannels = 1;
  Filter.In_MicChannels = 1;
  //Filter.InternalFilter = internalFilter;
  PDM_Filter_Init(&Filter);
  */


  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Init(&huart3);
  //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  //arm_fir_decimate_init_f32(&S,NUM_TAPS,64,filtro,firStateF32,BLOCK_SIZE);
  //arm_fir_init_f32(&S,NUM_TAPS,filtro,firStateF32,16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //for(k=0;k<1024;k++)
	  //{
	  HAL_I2S_Receive(&hi2s2,segundo,64,100);
/*	  HAL_UART_Receive(&huart3,buffer,1,100);
	  if(buffer[0]==0x30)
		  while(1)
		  {
		  HAL_I2S_Receive(&hi2s2,segundo,64,100);
		  k=0;
		  for(i=0;i<64;i++)
		  {
			  buffer[k]=(uint8_t)segundo[i];
			  k++;
			  buffer[k]=(uint8_t)(segundo[i]>>8);
			  k++;
		  }
		  HAL_UART_Transmit_IT(&huart3,buffer,128);
		  }*/
		  //segundo[k]=s[0];
		  //HAL_Delay(1);
	  //}

//	  HAL_I2S_Receive(&hi2s2,segundo,64,100);
	  //for(i=0; i<64;i++)
	  //{
		//  for(j=0;j<16;j++)
		 // {
		//	 k=((segundo[i]>>j)&0x0001);
			// HAL_GPIO_WritePin(SALIDA_GPIO_Port,SALIDA_Pin,k);
		  //}
	  //}
	  bintofloat(segundo,fsegundo);
	  //decimador(segundo,filtrada);
	  //for(i=0;i<16;i++)
	 //	  ffiltrada[i]=filtrada[i]*1.0f;
	  //arm_fir_f32(&S, ffiltrada, ffiltrada2, 16);
	  /*for(i=0;i < BLOCK_SIZE + NUM_TAPS - 1;i++)
		  firStateF32[i]=0;

	  arm_fir_decimate_f32(&S,fsegundo,ffiltrada2,BLOCK_SIZE);
*/


	  /*k=0;
	  for(i=0;i<1024;i++)
	  {
		  for(j=0;j<16;j++)
		  {
			  fsegundo[k]=(((segundo[i]>>j)&0x0001)*512.0f)+1536+256;
			  k++;
		  }
	  }*/

	//  fil.M = 64;
	//  fil.numTaps = 101;
	//  fil.pCoeffs = filtro;
	//  fil.pState = l;		//steps (orden del filtro) [101 +(1024/2)]
	  //arm_fir_decimate_f32(&fil ,segundo, clear, 512);
	//  arm_fir_decimate_f32(&fil ,fsegundo, clear, 512);


//while(1)
//{
//	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)((ffiltrada2[indice]/4096)+2046));

//}

	  /*
	  for(k=0;k<2048;k++)
	  {
		  HAL_I2S_Receive(&hi2s2,s,2,100);
		  segundo[k*2]=s[0];
		  segundo[k*2+1]=s[1];
		  HAL_Delay(1);
	  }

	  for(i=0;i<2048;i++)
	  {
		  k=0;
		  for(m=0;m<2;m++)
		  {
			  for(j=0;j<16;j++)
			  {
				  fsegundo[
				  k]=(((segundo[i*2+m]>>j)&0x0001)*512.0f)+1536+256;
				  k++;
			  }
		  }
		  fbuf=0;
		  for(j=0;j<32;j++)
		  {
			  fbuf+=fsegundo[j];
		  }
		  dato[i]=(uint16_t)(fbuf/32.0);
	  }
	  /*for(i=0;i<1024;i++)
	  {
		  fbuf=0;
		  for(j=0;j<32;j++)
		  {
			  fbuf+=fsegundo[i*32+j];
		  }
		  fsegundo[i]=fbuf/32;
	  }
*/
	  k=0;
	  for(j=0;j<20;j++)
	  {
		  for(i=0;i<14;i++)
		  {
			  sprintf(str,"%d",(uint16_t)fsegundo[k]);
			  ST7735_STR.pchString = str;
			  ST7735_STR.u32StrColour=0;
			  ST7735_STR.u8XCursor=j*10;
			  ST7735_STR.u8Ycursor=i*10;
			  ST7735_enPrintStr(&ST7735_STR);
			  k++;
		  }
	  }
	  while(1);
	  /*for(i=0;i<1023;i++)
	  {
		  dato[i] = (uint16_t)fsegundo[i+1];
	  }*/
/*
	  HAL_TIM_Base_Start_IT(&htim6);
	  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	  k=0;
	  while(1)
	  {

		  /*HAL_UART_Receive(&huart3,uartbuf,1,100);
		  if(uartbuf[0]==0x30)
		  {
			  uartbuf[0]=0;
			  HAL_UART_Transmit(&huart3,&dato[k],1,100);
			  k++;
			  if(k==16000)
				  k=0;
		  }*//*
		  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)dato[index]);

	  }*/
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AMP_EN_Pin|TFT_BKL_Pin|SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SALIDA_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TFT_RS_Pin|TFT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AMP_EN_Pin TFT_BKL_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = AMP_EN_Pin|TFT_BKL_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SALIDA_Pin */
  GPIO_InitStruct.Pin = SALIDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SALIDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RS_Pin TFT_RST_Pin */
  GPIO_InitStruct.Pin = TFT_RS_Pin|TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bintofloat(uint16_t* datain, float* dataout)
{
	int r,r2,r3;
	r3=0;
	for(r=0;r<64;r++)
	{
		for(r2=0;r2<16;r2++)
		{
		  *(dataout+r3)=((*(datain+r)>>r2)&0x0001)*1.0f;
		  r3++;
		}
	}
}
void decimador(uint16_t* datain, uint16_t* dataout)
{
	float fbuf;
	for(k=0;k<16;k++)
	{
		fbuf=0;
		for(i=0;i<4;i++)
		  {
			  for(j=0;j<16;j++)
			  {
				  fbuf+=(((*(datain+(k*4)+i)>>j)&0x0001)*1024.0f)+(2048.0f-(1024.0f/2.0f));

			  }
		  }
		*(dataout+k)=(uint16_t)(fbuf/64);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//int in;
	//float fbuf;
	//float fsegundo[16];
	if ( htim->Instance == TIM6){

		//{
			//for(j=0;j<16;j++)
					  //{
						// k=((segundo[i]>>indice)&0x0001);
						 HAL_GPIO_TogglePin(SALIDA_GPIO_Port,SALIDA_Pin);
						// indice++;
						// if(indice == 16)
						 //{
						//	 indice = 0;
						//	 i++;
						//	 if(i==64)
						//		 i=0;
						 //}
					  //}

			//HAL_GPIO_TogglePin(SALIDA_GPIO_Port,SALIDA_Pin);

		//	if(indice == 15)
			//{


				/*for(k=0;k<64;k++)
				  {
					  segundo8[k*2]=(uint8_t)segundo[k];
					  segundo8[k*2+1]=(uint8_t)segundo[k]>>8;
				  }*/
	/*			for(in = 0; in<BUFF_SIZE/2; in++)
				  {
				    segundo8[in] = HTONS(segundo[in]);
				  }
				PDM_Filter_64_LSB((uint8_t*)segundo8,filtrada,64,&Filter);*/
			//	indice = 0;
			//}
			//else
			//{
				//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)((ffiltrada2[indice])));
				//indice++;
			//}

	//	}
		//if(conta==16)
		//{
		//	HAL_I2S_Receive(&hi2s2,segundo,64,100);
		//	conta =0;
		//}
		//else
		//{
		//	conta++;
		//}
	}

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
  while(1) 
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
