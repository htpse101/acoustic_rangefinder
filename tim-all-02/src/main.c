/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "diag/Trace.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//#include "arm_math.h"
//#include "arm_const_structs.h"

#define ex_cascade_tim 0
#define ex_dac_adc 1
#define BLOCK_LEN (uint16_t)1024  // this is the received block length. We anlyze this data.
#define MAX_ARRAY_SIZE 2000  // when pulling arrays from files, do not exceed this limit

#define SHOW_DAC_ERRORS
//#define SHOW_ADC_ERRORS

// Un-comment one desired sampling frequency
//#define FS_45kHz
#define FS_36kHz
//#define FS_20Hz

#ifdef FS_45kHz
	#define BASE_PERIOD 19
	#define BASE_Scaler 0
#endif
#ifdef FS_36kHz
	#define BASE_PERIOD 24
	#define BASE_Scaler 0
#endif
#ifdef FS_20Hz
	#define BASE_PERIOD 999
	#define BASE_Scaler 44
#endif

// mode selection
// DAC out is pin pa4
// ADC in is pin pa0
#define PI 3.14159
#define RecLen 1024
#define FILE_OUT "C:\\STM32ToolChain\\Downloads\\ctr_out.csv"
#define FILE_IN  "C:\\STM32ToolChain\\Downloads\\ctr_in.csv"


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

volatile uint32_t rec_len = RecLen;
volatile uint16_t rawValues[RecLen];


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct SIG {
	uint32_t len;
	uint16_t sig[MAX_ARRAY_SIZE];
};

volatile struct SIG matlab_signal;
//volatile struct SIG rx_signal.sig = *pt_TX_GOLD_CODE;
//volatile struct SIG tx_signal;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void init_hardware(void);
void print_csv_data(uint16_t* pt_sig, uint32_t sig_len);
void print_csv_data32(uint16_t* pt_sig1, uint16_t* pt_sig2, uint32_t sig_len);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
void place_signal(uint16_t* pt_block, uint16_t block_len, uint16_t* pt_sig, uint16_t sig_len, uint16_t idx_loc);
void error();
void fliplr(uint16_t* ref_sig, uint16_t* new_sig, uint16_t sig_len);
void copy_2butterfly(uint16_t* pt_real_array, uint16_t r_len, uint16_t* pt_cpx_array);
void fetch_matlab_input(struct SIG *sig);  // Uploads the recent signal structure from matlab
void run_DAC(uint16_t* pt_sig, uint32_t len_sig);
void run_ADC(uint16_t* pt_sig, uint32_t len_sig);


extern void initialise_monitor_handles();
extern arm_cfft_sR_q15_len1024;

// Interrupt Flags
int f_user_btn_pushed = 0;
int f_dac_done = 0;
int f_adc_half_done = 0;
int f_adc_full_done = 0;
// end Interrupt Flags

int done = 0;
int cur_buffer = 0;
int saving = 1;
uint16_t buffer_1[512];
uint16_t buffer_2[512];

int main(void)
{
  fetch_matlab_input(&matlab_signal);
  initialise_monitor_handles();
  init_hardware();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, rec_len);

//  struct SIG matlab_signal;

  // This is the main event loop.
  while (1)
  {
	  if(f_user_btn_pushed == 1){
		  // do stuff
		  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);  // LD1 is green, 2 is blue, 3 is red
		  HAL_Delay(1000); // delay one second
		  f_user_btn_pushed = 0;
		  run_DAC(matlab_signal.sig, matlab_signal.len);
	  }
	  if(f_dac_done == 1){
		  // do stuff
		  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);  // LD1 is green, 2 is blue, 3 is red
		  HAL_Delay(1000);
		  f_dac_done = 0;
		  // start timer to count upwards
		  // set t1 = 0 so you know where t2 is
	  }


	  // code to switch buffer back and forth
	  if(done){
		  if(done == 2) {
			  //HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
			  if(saving) {
				  memcpy(buffer_1, rawValues, 512*sizeof(uint16_t));
			  }
		  } else {
			  if(saving) {
				  memcpy(buffer_2, &rawValues[512], 512*sizeof(uint16_t));
				  print_csv_data32(buffer_1, buffer_2, 512);
			  }
			  saving = 0;
		  }
		  done = 0;
	  }
  }
}


/*************************************** Signal Support Functions ******************************************/
void fetch_matlab_input(struct SIG *sig){
	// Fetches the recent signal structure from matlab
  FILE *fd = fopen(FILE_IN, "r");
  fscanf(fd, "%u", &sig->len);
  for (uint16_t i = 0; i < sig->len; i++){
	  fscanf(fd, ",%hu", &sig->sig[i]);
  }
  fclose(fd);
}

void copy_2butterfly(uint16_t* pt_real_array, uint16_t r_len, uint16_t* pt_cpx_array){
	// converts a real array into a complex array. the cpx_array must be at least 2x the len of the real array
	for (uint16_t i = 0; i < r_len; i++){
		pt_cpx_array[2*i] = pt_real_array[i];
	}
}
void copy_frombutterfly(uint16_t* pt_real_array, uint16_t r_len, uint16_t* pt_cpx_array){
	// converts a complex array into a real array
	for (uint16_t i = 0; i < r_len; i++){
		pt_real_array[i] = pt_cpx_array[i*2];
	}
}

void fliplr(uint16_t* ref_sig, uint16_t* new_sig, uint16_t sig_len){
	// This function flips an array from left to right
	for (uint16_t i = 0; i < sig_len; i++){
		new_sig[sig_len - i - 1] = ref_sig[i]; // -1 for the fact that an array of len 5 has only idx 4.
	}
}

void place_signal(uint16_t* pt_block, uint16_t block_len, uint16_t* pt_sig, uint16_t sig_len, uint16_t idx_loc){
	// This function embeds an array inside another array starting at a given idx
	// check, ensure that there is enough space to embed the signal
	// Tested
	if (idx_loc + sig_len < block_len){
		for (uint16_t i = 0; i < sig_len; i++){
			pt_block[i+idx_loc] = pt_sig[i];
		}
	} else{
		error();
	}
}

void print_csv_data32(uint16_t* pt_sig1, uint16_t* pt_sig2, uint32_t sig_len){
#ifdef OS_USE_SEMIHOSTING
	FILE *fd = fopen(FILE_OUT, "w+");
	fprintf(fd, "%d", pt_sig1[0]);  // print first digit
	for (uint32_t i = 1; i < sig_len; i++){
		fprintf(fd, ",%d", pt_sig1[i]);  // print successive digits
	}
	for (uint32_t i = 0; i < sig_len; i++){
		fprintf(fd, ",%d", pt_sig2[i]);  // print successive digits
	}

	fclose(fd);
#endif
}

void print_csv_data(uint16_t* pt_sig, uint32_t sig_len){
	// tested
#ifdef OS_USE_SEMIHOSTING
	FILE *fd = fopen(FILE_OUT, "w+");
	fprintf(fd, "%d", pt_sig[0]);  // print first digit
	for (uint32_t i = 1; i < sig_len; i++){
		fprintf(fd, ",%d", pt_sig[i]);  // print successive digits
	}

	fclose(fd);
#endif
}

/*************************************** End Signal Support Functions ******************************************/


/*************************************** DAC ******************************************/
void run_DAC(uint16_t* pt_sig, uint32_t len_sig){
	//the output is PA4
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);  // you have to call STOP before running it again
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) pt_sig, len_sig, DAC_ALIGN_12B_R);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac){
	// This is called when the DAC has finished sending something. You might want to start counting when this is called.
	f_dac_done = 1;
}
/*************************************** End DAC ******************************************/

/*************************************** ADC ******************************************/
void run_ADC(uint16_t* pt_sig, uint32_t len_sig){
	// called by something else, may not need this
	if (hadc1.State != HAL_ADC_STATE_REG_BUSY){
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) pt_sig, len_sig);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	done = 2;
	f_adc_half_done = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	done = 1;
	f_adc_full_done = 0;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
#ifdef SHOW_ADC_ERRORS
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);  // debugging this will cause an overrun error.
#endif
}
/***************************************End ADC ******************************************/


/***************************************User Button******************************************/

void EXTI15_10_IRQHandler(void){
	// Whenever the user button is pushed, this function is called. It is the event handler for that interrupt
	f_user_btn_pushed = 1;
	__HAL_GPIO_EXTI_CLEAR_IT(USER_Btn_Pin);  // must clear an interrupt once it has entered the call back function (IRQHandler)
}

/***************************************End User Button******************************************/





/*************************************** Hardware setup ******************************************/

static void init_hardware(void){
	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM8_Init();
	MX_TIM5_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_DAC_Init();

	// confirm that each child timer works as expected
	HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);

	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  // user push button

	// start children timers first, then master
	HAL_TIM_Base_Start(&htim8);
//	HAL_TIM_Base_Start_IT(&htim5);
//	HAL_TIM_Base_Start_IT(&htim3);

	HAL_TIM_Base_Start_IT(&htim2);  // start master timer last
}

// Test setup for master timer
void TIM2_IRQHandler(void){ // Master Timer
	HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
}

int adc_started = 0;
void TIM8_UP_TIM13_IRQHandler(void){
	if (adc_started == 0){
		adc_started = 1;
	}
	HAL_TIM_IRQHandler(&htim8);
}

void TIM5_IRQHandler(void){
	HAL_TIM_IRQHandler(&htim5);
}

void TIM3_IRQHandler(void){
	HAL_TIM_IRQHandler(&htim3);
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6; //ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; //Samples at half the DAC rate
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO; //ADC_SOFTWARE_START; //ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.NbrOfDiscConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;  // ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV; //ADC_EOC_SINGLE_CONV;  //ADC_EOC_SEQ_CONV; //ADC_EOC_SINGLE_CONV; //ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_480CYCLES; //ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_480CYCLES; //ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_480CYCLES; //ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_3CYCLES; //ADC_SAMPLETIME_480CYCLES; //ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 14;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; //TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = BASE_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{
  // this one can count basically to 1B
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = BASE_PERIOD;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim5, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = BASE_Scaler;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = BASE_PERIOD;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC1   ------> ETH_MDC
     PA1   ------> ETH_REF_CLK
     PA2   ------> ETH_MDIO
     PA7   ------> ETH_CRS_DV
     PC4   ------> ETH_RXD0
     PC5   ------> ETH_RXD1
     PB13   ------> ETH_TXD1
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PG11   ------> ETH_TX_EN
     PG13   ------> ETH_TXD0
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/*************************************** End Hardware setup ******************************************/

/*************************************** Error Functions ******************************************/

void error(){
	while(1){
		//  The system crashed!
		//  Consider stepping back through the stack frame to find out why.
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/*************************************** End Error Functions ******************************************/



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
