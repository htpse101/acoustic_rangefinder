/* This is the configuration file for the Project
 *
 * Un-comment blocks or comment them to control the flow of the program
 *
 */


#define ex_cascade_tim 0
#define ex_dac_adc 1
#define BLOCK_LEN (uint16_t)1024  // this is the received block length. We anlyze this data.
#define MAX_ARRAY_SIZE 2000  // when pulling arrays from files, do not exceed this limit

// Modes  Un-comment out the desired example
#define EXAMPLE_DAC_ADC
//#define EXAMPLE_TIMERS  // not used yet


// Un-comment one desired sampling frequency
//#define FS_90kHz
//#define FS_75kHz
//#define FS_60kHz
//#define FS_50kHz
#define FS_45kHz
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
//	uint16_t signal[SIG_LEN] = {3016,838,552,2424,3380,2303,1031,1532,3103,2785,725,947,3360,3289,1153,1314,2579,1778,1345,3089,3299,976,506,2674,3268,1604,1368,2881,2616,734,1074,3215,3254,1715,1472,1552,1276,2669,3916,1886,153,1940,3021,1711,2053,3151,1659,739,2407,2618,1488,2566,3220,1037,391,2610,3423,2346,1639,1176,1565,3048,2666,977,1995,3428,1434,274,2897,3903,1476,598,2018,2718,2747,2325,899,859,3083,3765,1773,575,1369,2492,3100,2732,1539,1059,1616,2208,2890,3148,1698,284,1501,3493,3138,1375,923,2194,3156,2046,645,1917,3789,2458,442,1561,2991,2001,1712,3032,2363,423,1312,3565,2988,1035,1461,2890,2101,798,2065,3698,2474,747,1333,2215,2244,3005,2965,746,367,2953,3403,1536,1840,2680,1253,1095,3034,2768,1415,2321,2510,696,1270,3643,3175,1244,1084,1689,2395,3132,2119,859,2243,3107,1088,975,3627,3244,456,850,3028,3008,2033,1519,1064,1967,3662,2852,564,705,2587,3277,2671,1667,941,1450,2565,2863,2615,2045,895,875,2841,3801,2096,579,1387,2915,2916,1453,993,2710,3499,1405,323,2409,3427,1836,1561,2651,1765,689,2419,3853,2070,479,1838,3076,1883,1159,2727,3342,1369,452,2140,3092,2455,2345,1859,507,1478,3889,3035,798,1584,2590,1408,1738,3386,2368,966,2108,2332,1065,2225,3833,2107,355,1474,2710,2812,2571,1444,974,2616,2921,1038,1625,3732,2240,0,1782,3814,2516,1114,1390,1875,2752,3275,1620,206,1813,3663,2979,1495,977,1374,2468,3152,2517,1663,1318,1205,2098,3568};
//	uint16_t rx_sig[BLOCK_LEN];
//	uint16_t tx_sig[BLOCK_LEN];

//	volatile uint16_t* blockA[BLOCK_LEN];

//	volatile uint16_t blockA[1024];
//	volatile uint16_t blockB[BLOCK_LEN];
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

