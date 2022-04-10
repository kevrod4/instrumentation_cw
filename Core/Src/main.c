/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define MAX_SINE_VAL_ARR 300
#define CURRENT_SINE_ARR 			100
#define ADC_SAMPLE_SIZE 			4096
#define FFT_SAMPLE_SIZE 			2048
#define ADC_SCALING					0.0008056640508584678173065185546875
//#define PI 							3.141592

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint32_t adcVoltageData[ADC_SAMPLE_SIZE];
uint32_t adcCurrentData[ADC_SAMPLE_SIZE];
float32_t adcVoltageFloat[ADC_SAMPLE_SIZE];
float32_t adcCurrentFloat[ADC_SAMPLE_SIZE];
float32_t adcVoltageFFT[ADC_SAMPLE_SIZE];
float32_t adcCurrentFFT[ADC_SAMPLE_SIZE];
float32_t testVoltageOutput[FFT_SAMPLE_SIZE];
float32_t testCurrentOutput[FFT_SAMPLE_SIZE];
uint16_t UART_size = 0;
uint8_t adcTransfer = 0;
char UART_BUF[1024];
char str[150] = { ' ' }; //Temp Char Buffer

arm_rfft_fast_instance_f32 fSV;
arm_rfft_fast_instance_f32 fSC;
uint32_t ifftFlag = 0;

uint32_t maxcurrentval = 0;
//uint32_t maxcurrentindex = 0;
uint32_t mincurrentval = 0;
//uint32_t mincurrentindex = 0;
float32_t maxFFTVoltageValue = 0;
uint32_t maxFFTVoltageindex = 0;
float32_t maxFFTCurrentValue = 0;
uint32_t maxFFTCurrentindex = 0;

uint8_t range_resistor = 0;

uint32_t sine_values[CURRENT_SINE_ARR];

float32_t range_resistor_table[8] = { 100, 320, 660, 920, 5200, 10100, 47100, 350100};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void My_TIM2_Init(int period){


	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 0;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = period - 1;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


static void My_TIM3_Init(int period)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
//Initiate the ADC via DMA and wait for sample buffer to fill up before leaving control
//
//void sync_sample_blocking(uint32_t *buffer, int buffer_size){
//	//Set the sampling rate for DAC and ADC
//	My_TIM2_Init(SINE_1K);
//	//Start DAC via DMA
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sine_wave_array_1k ,
//			SINE_1K_LUT_SIZE, DAC_ALIGN_12B_R);
//
//	//Start ADC
//	//Disable DMA before Start
//	HAL_ADC_Stop_DMA(&hadc1);
//	buffer_is_full = 0;
//	HAL_ADC_Start_DMA(&hadc1, buffer, buffer_size);
//
//	//Setup common sync timer
//	HAL_TIM_Base_Stop(&htim2);
//	HAL_TIM_Base_Start(&htim2);
//	while (buffer_is_full == 0);
//	HAL_TIM_Base_Start(&htim2);
//	HAL_TIM_Base_Stop(&htim2);
//
//}

//Callback for DMA calls after ADC buffer is full

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	//Disable the sampling timer for both the ADC and DAC
//	HAL_TIM_Base_Stop(&htim2);
////Set the buffer full flag
//	buffer_is_full = 1;
//}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//uint16_t frequency_hold;
//uint16_t resistor_hold;
//uint16_t path_hold;

//void DMATransferComplete(DMA_HandleTypeDef *hdma)
//{
//	//Disable UART interrupt to use own interrupt
//	huart2.Instance->CR3 &= ~USART_CR3_DMAT;
//
//	//Perform the interrupt
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//
//
//}

void sine_generator(){
	for (int i=0; i<CURRENT_SINE_ARR; i++){
		sine_values[i] = ((sin(i*2*PI/CURRENT_SINE_ARR) + 1)*(4095/2));
	}
}

void path_select(uint16_t path_hold) //0 for Voltage; 1 for Current
{
	if(path_hold==0){
		//Enter PIN HERE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	else{
		//Enter PIN HERE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	}
}

void frequency_select(uint16_t frequency_hold)
{
	  switch(frequency_hold){
	  case 0: //f = 100Hz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(9000);
		  My_TIM3_Init(45000);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 10-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 900-1);
		  //HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 1: // f = 141.28Hz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(6370);
		  My_TIM3_Init(31850);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 10-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 637-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 2: // f = 500Hz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(1800);
		  My_TIM3_Init(9000);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 9-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 200-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 3: // f = 1kHz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(900);
		  //My_TIM3_Init(4500);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 9-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 100-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 4: // f = 10kHz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(90);
		  My_TIM3_Init(450);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 5: // f = 25kHz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(36);
		  My_TIM3_Init(180);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 6-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 6-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 6: // f = 75Hz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(12);
		  My_TIM3_Init(60);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 4-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 3-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  case 7: // f = 100kHz
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_TIM_Base_Stop(&htim2);
		  My_TIM2_Init(9);
		  My_TIM3_Init(45);
//		  __HAL_TIM_SET_PRESCALER(&htim2, 3-1);
//		  __HAL_TIM_SET_AUTORELOAD(&htim2, 3-1);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_values, CURRENT_SINE_ARR, DAC_ALIGN_12B_R);
		  break;
	  }
}

void resistor_select(uint16_t resistor_hold)
{
	switch (resistor_hold){
	case 0: //000
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		break;
	case 1: //001
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		break;
	case 2: //010
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		break;
	case 3: //011
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		break;
	case 4: //100
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//S2
		break;
	case 5:	//101
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//S2
		break;
	case 6:	//110
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//S2
		break;
	case 7: //111
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  //Enable - Low Enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //S0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// S1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//S2
		break;
	case 8: //No MUX, only f/b res 1Mohm
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  //Enable - Low Enable
	}
}


void sync_sample_blocking(uint32_t *buffer) {

//Set the correct samplying rate for DAC and ADC
	frequency_select(0);

//Setup ADC
//Must disable the DMA first otherwise, it will not let you change the destination pointer as you intend!
	HAL_ADC_Stop_DMA(&hadc1);
	adcTransfer = 0;
	HAL_ADC_Start_DMA(&hadc1, buffer, ADC_SAMPLE_SIZE);

//Setup common sync timer
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Start(&htim2);
	while (adcTransfer == 0);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Stop(&htim2);
}

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //INTERRUPTS -> CALLBACK ID FROM HAL_DMA.H FILE ||  Address of the callback function for pointer
//  HAL_DMA_RegisterCallback(&hdma_usart2_tx, HAL_DMA_XFER_CPLT_CB_ID , &DMATransferComplete);




  sine_generator(); //Generate the samples
  //Note: sine amplitude might vary with frequency
//
//  path_select(0);
//  resistor_select(0);
//
//  frequency_select(4);
//  HAL_Delay(500);
//  HAL_ADC_Start_DMA(&hadc1, adcVoltageData, ADC_SAMPLE_SIZE);
//  //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//
//  /* Transfer data to computer for validation */
////	  for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
////	  {
////		  adcVoltageFloat[i] = adcVoltageData[i] * ADC_SCALING;
////	  }
  //sync_sample_blocking(&adcVoltageData[0]);
  frequency_select(1); // Change the frequency here

  resistor_select(0);
  path_select(0);
  adcTransfer = 0;
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcVoltageData, ADC_SAMPLE_SIZE);
  HAL_Delay(1000);
  path_select(1);
  for (int i=0; i<8; i++){
	  adcTransfer = 0;
	  range_resistor = i;
	  resistor_select(range_resistor); // Change resistor RANGE HERE -----------^^^^^^^
	  HAL_Delay(1000);
	  HAL_TIM_Base_Start(&htim3);
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcCurrentData, ADC_SAMPLE_SIZE);

	  while(adcTransfer != 1);
	  maxcurrentval = adcCurrentData[0];
	  for (uint32_t i = 1; i < ADC_SAMPLE_SIZE; i++)
		  if (adcCurrentData[i] > maxcurrentval){
			  maxcurrentval = adcCurrentData[i];
		  }
	  mincurrentval = adcCurrentData[0];
	  for (uint32_t i = 1; i < ADC_SAMPLE_SIZE; i++)
		  if (adcCurrentData[i] < mincurrentval){
			  mincurrentval = adcCurrentData[i];
		  }

	  if (maxcurrentval >= 4095 || mincurrentval <= 0){
		  adcTransfer = 0;
		  range_resistor = i-1;
		  resistor_select(range_resistor);
		  HAL_Delay(500);
		  HAL_TIM_Base_Start(&htim2);
		  HAL_TIM_Base_Start(&htim3);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcCurrentData, ADC_SAMPLE_SIZE);
		  while(adcTransfer!=1);
		  break;
		  }


	  //uint8_t temphold = 0;

//	  if ((uint32_t)maxcurrentval >= 4095 || (uint32_t)mincurrentval <= 1){
//		  adcTransfer = 0;
//		  resistor_select(i-1);
//		  HAL_Delay(500);
//		  HAL_TIM_Base_Start(&htim3);
//		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcCurrentData, ADC_SAMPLE_SIZE);
//		  while (adcTransfer !=1);
//		  break;
//	  }


  }
  for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
    	  {
    	  	adcVoltageFloat[i] = adcVoltageData[i] * ADC_SCALING;
    	  	adcCurrentFloat[i] = adcCurrentData[i] * ADC_SCALING;
    	  }
//
  for (int i = 0; i < 4096; i++) {
        float32_t multiplier = 0.5 * (1 - cos(2*PI*i/4095));
        adcVoltageFloat[i] = multiplier * adcVoltageFloat[i];
    }
	for (int i = 0; i < 4096; i++) {
		  float32_t multiplier = 0.5 * (1 - cos(2*PI*i/4095));
		  adcCurrentFloat[i] = multiplier * adcCurrentFloat[i];
	  }

  arm_rfft_fast_init_f32(&fSV, ADC_SAMPLE_SIZE);
  arm_rfft_fast_f32(&fSV, adcVoltageFloat, adcVoltageFFT, ifftFlag);
  arm_cmplx_mag_f32(adcVoltageFFT, testVoltageOutput, FFT_SAMPLE_SIZE);
  testVoltageOutput[0] = 0;
  arm_max_f32(testVoltageOutput, FFT_SAMPLE_SIZE, &maxFFTVoltageValue, &maxFFTVoltageindex);

  arm_rfft_fast_init_f32(&fSC, ADC_SAMPLE_SIZE);
  arm_rfft_fast_f32(&fSC, adcCurrentFloat, adcCurrentFFT, ifftFlag);
  arm_cmplx_mag_f32(adcCurrentFFT, testCurrentOutput, FFT_SAMPLE_SIZE);
  testCurrentOutput[0] = 0;
  arm_max_f32(testCurrentOutput, FFT_SAMPLE_SIZE, &maxFFTCurrentValue, &maxFFTCurrentindex);



  //Magnitude Calculation

  float32_t RealVoltage = adcVoltageFFT[maxFFTVoltageindex*2];
  float32_t ImagVoltage = adcVoltageFFT[maxFFTVoltageindex*2+1];
  float32_t RealCurrent = adcCurrentFFT[maxFFTCurrentindex*2];
  float32_t ImagCurrent = adcCurrentFFT[maxFFTCurrentindex*2+1];

  float32_t RealImpedance = ((RealVoltage*RealCurrent)+(ImagVoltage*ImagCurrent))/(RealCurrent*RealCurrent + ImagCurrent*ImagCurrent);
  RealImpedance = range_resistor_table[range_resistor]*RealImpedance;
  float32_t ImagImpedance = ((RealCurrent*ImagVoltage)-(RealVoltage*ImagCurrent))/(RealCurrent*RealCurrent + ImagCurrent*ImagCurrent);
  ImagImpedance = range_resistor_table[range_resistor]*ImagImpedance;
  //float32_t RealImpedance = (range_resistor_table[range_resistor]*((RealVoltage*RealCurrent)+(ImagVoltage*ImagCurrent)))/(RealCurrent*RealCurrent + ImagCurrent*ImagCurrent);




















  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(adcTransfer == 1){
//		 for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
//		 {
//			 UART_size = sprintf(UART_BUF,"%ld,",adcVoltageData[i]);
//			 HAL_UART_Transmit(&huart2, (uint8_t *)UART_BUF, UART_size, HAL_MAX_DELAY);
//		 }
//		 UART_size = sprintf(UART_BUF,"\n,");
//		 HAL_UART_Transmit(&huart2, (uint8_t *)UART_BUF, UART_size, HAL_MAX_DELAY);
//
//		 adcTransfer = 0;
	  }






//	  huart2.Instance->CR3 |= USART_CR3_DMAT; //Disable IT so can use own interrupt
//	  //Starting the DMA in interrupt mode and saving the message
//	  HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg, (uint32_t)&huart2.Instance->DR, strlen(msg));

//	  HAL_Delay(1000);





//	  //TESTING ADC CODE
//	  //GET ADC VALUE
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
//
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	  raw = HAL_ADC_GetValue(&hadc1);
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//
//	  sprintf(msg, "%hu\r\n", raw);
//	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//	  HAL_Delay(1);
//







//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
//	  HAL_Delay(1000);
//	  frequency_select(3);
//	  HAL_Delay(1000);
//	  frequency_select(4);
//	  	  HAL_Delay(1000);
//	  	frequency_select(5);
//	  		  HAL_Delay(1000);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	sync_sample_blocking(&adc_buffer_A[0],ADC_BUFFER_SIZE);
//
//		//Print
//	//HAL_UART_Transmit(&huart2,"Start\n",6,1);
//	for(int i=0;i<ADC_BUFFER_SIZE;i++){
//		sprintf( str, "%d",(uint16_t)(adc_buffer_A[i]& 0x00000FFF));
//		HAL_UART_Transmit(&huart2,(uint8_t*)str,50,1);
//		HAL_UART_Transmit(&huart2,(uint8_t*)"\n",1,1);
//	}
//	HAL_UART_Transmit(&huart2,(uint8_t*)"End\n",4,1);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Initializes the CPU, AHB and APB buses clocks
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	// ADC Conversion Completed
	HAL_TIM_Base_Stop(&htim3);
	adcTransfer = 1;
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
