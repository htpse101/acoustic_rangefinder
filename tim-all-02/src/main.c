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

/* USER CODE BEGIN Includes */
//#define Enable_Timers 0

#define ex_cascade_tim 0
#define ex_dac_adc 1
#define BLOCK_LEN (uint16_t)1024  // this is the received block length. We anlyze this data.
#define MAX_ARRAY_SIZE 2000  // when pulling arrays from files, do not exceed this limit

// Modes  Un-comment out the desired example
#define EXAMPLE_DAC_ADC
//#define EXAMPLE_TIMERS  // not used yet

//#define SHOW_DAC_ERRORS
#define SHOW_ADC_ERRORS

// Un-comment one desired sampling frequency
#define FS_90kHz
//#define FS_75kHz
//#define FS_60kHz
//#define FS_50kHz
//#define FS_45kHz
//#define FS_36kHz
//#define FS_30kHz
//#define FS_900Hz
//#define FS_20Hz

#ifdef FS_90kHz  // Cannot light LED this fast
	#define BASE_PERIOD 9
	#define BASE_Scaler 0
	#define FS 90000
#endif
#ifdef FS_75kHz
	#define BASE_PERIOD 11
	#define BASE_Scaler 0
	#define FS 75000
#endif
#ifdef FS_60kHz  // Cannot light LED this fast
	#define BASE_PERIOD 14
	#define BASE_Scaler 0
	#define FS 60000
#endif
#ifdef FS_50kHz
	#define BASE_PERIOD 17
	#define BASE_Scaler 0
	#define FS 50000
#endif
#ifdef FS_45kHz
	#define BASE_PERIOD 19
	#define BASE_Scaler 0
	#define SIG_LEN 279
	#define FS 45000
	uint16_t signal[SIG_LEN] = {3016,838,552,2424,3380,2303,1031,1532,3103,2785,725,947,3360,3289,1153,1314,2579,1778,1345,3089,3299,976,506,2674,3268,1604,1368,2881,2616,734,1074,3215,3254,1715,1472,1552,1276,2669,3916,1886,153,1940,3021,1711,2053,3151,1659,739,2407,2618,1488,2566,3220,1037,391,2610,3423,2346,1639,1176,1565,3048,2666,977,1995,3428,1434,274,2897,3903,1476,598,2018,2718,2747,2325,899,859,3083,3765,1773,575,1369,2492,3100,2732,1539,1059,1616,2208,2890,3148,1698,284,1501,3493,3138,1375,923,2194,3156,2046,645,1917,3789,2458,442,1561,2991,2001,1712,3032,2363,423,1312,3565,2988,1035,1461,2890,2101,798,2065,3698,2474,747,1333,2215,2244,3005,2965,746,367,2953,3403,1536,1840,2680,1253,1095,3034,2768,1415,2321,2510,696,1270,3643,3175,1244,1084,1689,2395,3132,2119,859,2243,3107,1088,975,3627,3244,456,850,3028,3008,2033,1519,1064,1967,3662,2852,564,705,2587,3277,2671,1667,941,1450,2565,2863,2615,2045,895,875,2841,3801,2096,579,1387,2915,2916,1453,993,2710,3499,1405,323,2409,3427,1836,1561,2651,1765,689,2419,3853,2070,479,1838,3076,1883,1159,2727,3342,1369,452,2140,3092,2455,2345,1859,507,1478,3889,3035,798,1584,2590,1408,1738,3386,2368,966,2108,2332,1065,2225,3833,2107,355,1474,2710,2812,2571,1444,974,2616,2921,1038,1625,3732,2240,0,1782,3814,2516,1114,1390,1875,2752,3275,1620,206,1813,3663,2979,1495,977,1374,2468,3152,2517,1663,1318,1205,2098,3568};
	uint16_t rx_sig[BLOCK_LEN];
	uint16_t tx_sig[BLOCK_LEN];

//	volatile uint16_t* blockA[BLOCK_LEN];

	volatile uint16_t blockA[1024];
	volatile uint16_t blockB[BLOCK_LEN];
#endif
#ifdef FS_36kHz
	#define BASE_PERIOD 24
	#define BASE_Scaler 0
	#define FS 36000
#endif
#ifdef FS_30kHz
	#define BASE_PERIOD 29
	#define BASE_Scaler 0
	#define FS 30000
#endif
#ifdef FS_900Hz  // confirmed 23 Jan 2018 - closer to 909 Hz
	#define BASE_PERIOD 999
	#define BASE_Scaler 0
	#define FS 900
#endif
#ifdef FS_20Hz  // confirmed 23 Jan 2018
	#define BASE_PERIOD 999
	#define BASE_Scaler 44
	#define FS 20
#endif

// mode selection
#ifdef EXAMPLE_DAC_ADC
	// DAC out is pin pa4
	// ADC in is pin pa0
	#define Samples 10  //200
	#define PI 3.14159
	#define RecLen 2500
	#define FILE_OUT "C:\\STM32ToolChain\\Downloads\\ctr_out.csv"
	#define FS_OUT "C:\\STM32ToolChain\\Downloads\\mc_fs.csv"
	#define FILE_IN  "C:\\STM32ToolChain\\Downloads\\ctr_in.csv"
	#define ENABLE_DAC_ADC  // call whenever you want the DAC and ADC to work
//	#define HalfADC
	int16_t IV[Samples], value;
#else
	#define FILE_OUT "C:\\STM32ToolChain\\Downloads\\test_ctr_out.csv"
#endif

/* USER CODE END Includes */

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

// This may not be the best way to switch signals, but it's an idea.

volatile uint16_t rawValues[RecLen];
volatile uint16_t array_oBuffA[MAX_ARRAY_SIZE];
volatile uint32_t len_oBuffA;  // for output signal A
volatile uint16_t array_oBuffB[MAX_ARRAY_SIZE];
volatile uint32_t len_oBuffB;  // for output signal B

volatile uint16_t array_iBuffA[MAX_ARRAY_SIZE];
volatile uint16_t array_iBuffB[MAX_ARRAY_SIZE];
volatile uint16_t* array_iActive;  // toggle between input buffers and this is the active one
volatile uint32_t len_iActive;	// length of active array (
volatile uint16_t active_iBuff = 0; // 0 = A, 1 = B; // toggle this to switch
// end section that may not be best idea

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct SIG {
//	uint16_t len;
	uint32_t len;
	uint16_t sig[MAX_ARRAY_SIZE];
};
volatile struct SIG matlab_signal;
/* USER CODE END PV */

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
//void print_csv_data(uint16_t* pt_sig, uint16_t sig_len);
void print_csv_data(uint16_t* pt_sig, uint32_t sig_len);
void print_csv_fs();  // prints out the sampling frequency for relating the data to the final results
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
void test_file_writing();
void place_signal(uint16_t* pt_block, uint16_t block_len, uint16_t* pt_sig, uint16_t sig_len, uint16_t idx_loc);
void error();
void fliplr(uint16_t* ref_sig, uint16_t* new_sig, uint16_t sig_len);
void copy_2butterfly(uint16_t* pt_real_array, uint16_t r_len, uint16_t* pt_cpx_array);
void example_dac_adc();
void fetch_matlab_input(struct SIG *sig);  // Uploads the recent signal structure from matlab
//void run_DAC(uint16_t* pt_sig, uint16_t len_sig);
void run_DAC(uint16_t* pt_sig, uint32_t len_sig);
void run_ADC(uint16_t* pt_sig, uint32_t len_sig);
void recover_ADC_Error();


extern void initialise_monitor_handles();
extern arm_cfft_sR_q15_len1024;

int main(void)
{
  fetch_matlab_input(&matlab_signal);
  init_hardware();
  initialise_monitor_handles();
//  struct SIG matlab_signal;



  // confirm using the DMA with ADC and DAC
#ifdef EXAMPLE_DAC_ADC
  example_dac_adc();
#endif

  // then get serial communication working with usart, I guess

#ifdef FS_45kHz

  //adc dac test
  /*
  arm_fill_q15(0, rx_sig, BLOCK_LEN);
  arm_fill_q15(0, tx_sig, BLOCK_LEN);
  uint16_t idx_loc = 250;
  place_signal(tx_sig, BLOCK_LEN, signal, SIG_LEN, idx_loc);
  */

/*
  // flip reference signal
  uint16_t new_signal[SIG_LEN];
  fliplr(signal, new_signal, SIG_LEN);

  // prepare ADC signal
  uint16_t ADC_LEN = BLOCK_LEN - SIG_LEN;
  uint16_t blockTX[ADC_LEN];
  arm_fill_q15(0, blockTX, ADC_LEN);
  uint16_t idx_loc = 250;
  place_signal(blockTX, ADC_LEN, signal, SIG_LEN, idx_loc);

/*
  // prepare arrays
  uint16_t ref_sig_len = SIG_LEN + ADC_LEN;
  uint16_t real_num_samples = BLOCK_LEN - 1 + 1;
  uint16_t* pt_final_res[real_num_samples];
  uint32_t cpx_num_samples = real_num_samples*2;
  uint16_t* pt_ref_sig_cpx[cpx_num_samples];
  uint16_t* pt_cap_sig_cpx[cpx_num_samples];
  uint16_t* pt_res_fft[cpx_num_samples];
  arm_fill_q15(0, pt_ref_sig_cpx, cpx_num_samples);
  arm_fill_q15(0, pt_cap_sig_cpx, cpx_num_samples);
  arm_fill_q15(0, pt_res_fft, cpx_num_samples);
  arm_fill_q15(0, pt_final_res, real_num_samples);
  copy_2butterfly(new_signal, SIG_LEN, pt_ref_sig_cpx);

//  print_csv_data(pt_ref_sig_cpx, cpx_num_samples);  // test code

  copy_2butterfly(blockTX, ADC_LEN, pt_cap_sig_cpx);
//  print_csv_data(pt_cap_sig_cpx, cpx_num_samples);  // test code



  // Perform FFT
  uint8_t ifftFlag = 1;
  uint8_t bitReverseFlag = 0;
  arm_cfft_q15(&arm_cfft_sR_q15_len1024, pt_ref_sig_cpx, !ifftFlag, bitReverseFlag);
//  print_csv_data(pt_ref_sig_cpx, cpx_num_samples);

  arm_cfft_q15(&arm_cfft_sR_q15_len1024, pt_cap_sig_cpx, !ifftFlag, bitReverseFlag);
//  print_csv_data(pt_cap_sig_cpx, cpx_num_samples);

  // multiply results
  arm_cmplx_mult_cmplx_q15(pt_ref_sig_cpx, pt_cap_sig_cpx, pt_res_fft, cpx_num_samples);

  // do inverse FFT
  arm_cfft_q15(&arm_cfft_sR_q15_len1024, pt_res_fft, ifftFlag, bitReverseFlag);
//  print_csv_data(pt_res_fft, cpx_num_samples);

  //convert to real
  copy_frombutterfly(pt_final_res, real_num_samples, pt_res_fft);
//  print_csv_data(pt_final_res, real_num_samples);

*/
  // view results
//  volatile uint32_t* max_val;
//  volatile uint32_t* max_idx;
//  arm_max_q15(pt_final_res, real_num_samples, max_val, max_idx);
//  printf("Max val: %d\n", *max_val);
//  printf("Max idx: %d\n", *max_idx);
//  print_csv_data(pt_final_res, real_num_samples);

//
//  uint16_t pt_sig[10] = {1,2,3,4,5,6,7,8,9,10};
//  uint16_t new_sig[10] = {0,0,0,0,0,0,0,0,0,0};
//  uint16_t blk[10] = {0,0,0,0,0,0,0,0,0,0};
//
//
//  fliplr(pt_sig, new_sig, 10);
//
//
//  fliplr(signal, new_signal, SIG_LEN);
//  arm_fill_q15(0, blockA, BLOCK_LEN);
//
////  uint16_t idx_loc = 250;
//  place_signal(blockTX, BLOCK_LEN, signal, SIG_LEN, idx_loc);
//  place_signal(blockTX, BLOCK_LEN, pt_sig, 10, idx_loc);
//
//  uint16_t pt_resolved[2*BLOCK_LEN - 1];
//  arm_fill_q15(0, pt_resolved, 2*BLOCK_LEN - 1);
//  uint32_t res_size = 2*BLOCK_LEN - 1;
//  volatile uint32_t* max_val;
//  volatile uint32_t* max_idx;
//
////  arm_correlate_q15(new_sig, 10, blockTX, BLOCK_LEN, pt_resolved);
//  arm_conv_q15(new_sig, 10, blockTX, BLOCK_LEN, pt_resolved);
//
//  arm_max_q15(pt_resolved, res_size, max_val, max_idx);
//  printf("Max val: %d\n", *max_val);
//  printf("Max idx: %d\n", *max_idx);
//
//
//  print_csv_data(pt_resolved, 2*BLOCK_LEN - 1);
  while(1){}
#endif

//  test_file_writing();

  while (1)
  {
  }
}

void fetch_matlab_input(struct SIG *sig){
	// Fetches the recent signal structure from matlab
	FILE *fd = fopen(FILE_IN, "r");

	//  int numarray[93];
//	  uint16_t i_array_len;
	  fscanf(fd, "%u", &sig->len);


	  for (uint16_t i = 0; i < sig->len; i++){
		  fscanf(fd, ",%hu", &sig->sig[i]);
	  }

//	  print_csv_data(sig->sig, sig->len);  // checks to make sure this works
	  // this works for dynamically allocating the right amount of memory
	  //	  uint16_t* numarray;
	  //	  numarray = (uint16_t*) malloc(i_array_len * sizeof(uint16_t));
	  //	  for (uint16_t i = 0; i < i_array_len; i++){
	  //		  fscanf(fd, ",%u", &numarray[i]);
	  //	  }
	  //	  print_csv_data(numarray, i_array_len);
	  //	  free(numarray);

}

void example_dac_adc(){
	print_csv_fs();


	for (uint32_t i = 0; i < rec_len; i++){
		rawValues[i] = 0;
	}
	// DAC
//	int16_t IV[Samples], value;
	for (uint16_t i = 0; i < Samples; i++){
		value = (uint16_t) rint(2048*(sinf(((2*PI)/Samples)*i)+1));
		IV[i] = value < 4096 ? value : 4095;
	}
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) &matlab_signal.sig, &matlab_signal.len, DAC_ALIGN_12B_R);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawValues, rec_len);



//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) IV, Samples, DAC_ALIGN_12B_R);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawValues, rec_len);
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

void print_csv_fs(){
	//
#ifdef OS_USE_SEMIHOSTING
	FILE *fd = fopen(FS_OUT, "w+");
	fprintf(fd, "%d", FS);  // print first digit
	fclose(fd);
#endif
}


void test_file_writing(){
#ifdef ex_log_msg
	  char camessage[50] = "This is a message\nLine two";
	#ifdef OS_USE_TRACE_SEMIHOSTING_DEBUG
	  trace_printf("using trace_printf\n");
	#endif
	#ifdef OS_USE_SEMIHOSTING
	  printf("Does this work? \n");
	  printf("or this? \r\n");
	  FILE *fd = fopen("C:\\STM32ToolChain\\Downloads\\junk.txt", "w+");
	//  FILE *fd = fopen("C:/STM32ToolChain/Downloads/junk.txt", "w+");
	  fwrite(camessage, sizeof(char), strlen(camessage), fd);
	  fclose(fd);
	#endif
#endif
}

void run_DAC(uint16_t* pt_sig, uint32_t len_sig){
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);  // you have to call STOP before running it again
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) pt_sig, len_sig, DAC_ALIGN_12B_R);
}

void run_ADC(uint16_t* pt_sig, uint32_t len_sig){
//	HAL_ADC_Stop_DMA(&hadc1);  // This is being done in the conversion complete call back
	if (hadc1.State != HAL_ADC_STATE_REG_BUSY){
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) pt_sig, len_sig);
	}
}

//void run_DAC(uint16_t* pt_sig, uint16_t len_sig){
//	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);  // you have to call STOP before running it again
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) pt_sig, len_sig, DAC_ALIGN_12B_R);
//}

void EXTI15_10_IRQHandler(void){
	// Whenever the user button is pushed, this function is called. It is the event handler for that interrupt

//	__HAL_GPIO_EXTI_CLEAR_IT(USER_Btn_Pin);  // must clear an interrupt once it has entered the call back function (IRQHandler)
	run_ADC(rawValues, rec_len);
	uint32_t sig_len = matlab_signal.len;
	run_DAC(&matlab_signal.sig, sig_len);
//	HAL_Delay(10);
	if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_READY){
//		fetch_matlab_input(&matlab_signal);  // fetch new signal
		#ifdef FS_45kHz
//			fetch_matlab_input(&matlab_signal);  // fetch new signal
			// start ADC and DAC for one cycle
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) IV, Samples, DAC_ALIGN_12B_R);
			run_DAC(&matlab_signal.sig, &matlab_signal.len);

		//	run_DAC(&matlab_signal.sig, 16);
		//	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
		//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) &matlab_signal.sig, &matlab_signal.len, DAC_ALIGN_12B_R);

		//	HAL_ADC_Stop_DMA(&hadc1);
		//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawValues, rec_len);

			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*) tx_sig, BLOCK_LEN, DAC_ALIGN_12B_R);
		//
		//	HAL_ADC_Stop_DMA(&hadc1);
		//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rx_sig, BLOCK_LEN);
		#endif
//			fetch_matlab_input(&matlab_signal);  // fetch new signal


			//			run_ADC(rawValues, rec_len);

			//			HAL_ADC_Stop_DMA(&hadc1);
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawValues, rec_len);
	}
	__HAL_GPIO_EXTI_CLEAR_IT(USER_Btn_Pin);  // must clear an interrupt once it has entered the call back function (IRQHandler)

//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) IV, Samples, DAC_ALIGN_12B_R);

//	HAL_ADC_Stop_DMA(&hadc1);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawValues, rec_len);
//		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
//		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*) dac_mod_sig, Len, DAC_ALIGN_12B_R);
//	zonk if cannot find their convenience function:
//	g_DMA_S2CR_copy = hdac->DMA_Handle2->Instance->CR;
//	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

#ifdef EXAMPLE_DAC_ADC
	if (hadc->Instance == ADC1){
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
//		print_csv_data(rawValues, RecLen);
		// write to the file output here.
	}
#endif
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
#ifdef SHOW_ADC_ERRORS
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);  // debugging this will cause an overrun error.
	print_csv_data(rawValues, RecLen);
	recover_ADC_Error();
#endif
}

void recover_ADC_Error(){

	hadc1.DMA_Handle->State= HAL_DMA_STATE_BUSY; //HAL_DMA_STATE_READY;
	HAL_ADC_Stop_DMA(&hadc1);

	//	hdma->State= HAL_DMA_STATE_READY

//	hdma->Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
//	    hdma->Instance->FCR &= ~(DMA_IT_FE);
////	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);
////	DMA_Cmd(DMA2_Stream0, DISABLE);while (DMA2_Stream0->CR & DMA_SxCR_EN);

	//	HAL_ADC_Stop_DMA(&hadc1);

	//	HAL_DAC_Stop_DMA()
//	HAL_DMA_DeInit()
//	uint32_t tmpreg = (uint32_t)&hdac->Instance->DHR12R2;
//
//	uint32_t sig_len;
//	arm_q15_to_q31(&matlab_signal.len, sig_len);
//
//	DMA_SetConfig(hdac->DMA_Handle1, tmpreg, (uint32_t*)&matlab_signal.sig, sig_len);
}


void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac){
#ifdef SHOW_DAC_ERRORS
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);  // This happens, but with how things are reset, the system is working.
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);  // this function should clear the error
#endif
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac){
	HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
//	btn_pushed = 0;
//	_error(); // The DAC or DMA is not properly being reset. Do this, then delete this line
//	HAL_DMA_Init(&hdac);
//	MX_DMA_Init();
//	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
}


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

	// Master timer
//	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// confirm that each child timer works as expected

	HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);

	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  // user push button

	// start children timers first, then master


//	HAL_TIM_Base_Start(&htim8);
//	HAL_TIM_Base_Start_IT(&htim5);
//	HAL_TIM_Base_Start_IT(&htim3);
//
//	HAL_TIM_Base_Start_IT(&htim2);  // start master timer last

	HAL_TIM_Base_Start(&htim8);
//	HAL_TIM_Base_Start_IT(&htim5);
//	HAL_TIM_Base_Start_IT(&htim3);

	HAL_TIM_Base_Start_IT(&htim2);  // start master timer last

//	// Sets up all timers in interrupt mode. Will not run ADC or DAC
//	HAL_TIM_Base_Start_IT(&htim8);
//	HAL_TIM_Base_Start_IT(&htim5);
//	HAL_TIM_Base_Start_IT(&htim3);
//
//	HAL_TIM_Base_Start_IT(&htim2);  // start master timer last
//#else


}

// Test setup for master timer
void TIM2_IRQHandler(void){ // Master Timer
	HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

#ifdef EXAMPLE_TIMERS
	if (htim->Instance == TIM2){  //		 Master Timer
		HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
	} else if (htim->Instance == TIM8){
		HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
	} else if (htim->Instance == TIM5){
		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
	} else if (htim->Instance == TIM3){
		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
	}
#else

#endif
}


// Test part 2. Confirm that all child timers work

void TIM8_UP_TIM13_IRQHandler(void){
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; //ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
#ifdef HalfADC
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; //Samples at half the DAC rate
#else
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING; //ADC_EXTERNALTRIGCONVEDGE_RISING;
#endif
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO; //ADC_SOFTWARE_START; //ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;  // ENABLE;
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

/* USER CODE BEGIN 4 */

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
