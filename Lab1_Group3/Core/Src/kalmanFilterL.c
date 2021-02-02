/*
 * kalmanFilterC.c
 *
 *  Created on: Jan 23, 2021
 *      Author: ah
 */
#include "main.h"
#include "lab1util.h"
#include "arm_math.h"
#include "cmsis_gcc.h"

int kalmanFilterL(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length){
	float stdDiff= 0.0;
	float avgDiff = 0.0;
	uint32_t u_len = (uint32_t) length;
	float diffArray[length];
	float corrArray[length*2-1];
	float convArray[length*2-1];

	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);

	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateA(kstate, InputArray[i]);
		if (isnan(updateResult) || isinf(updateResult)){
			//Return FPSCR as an error flag
			return __get_FPSCR() & 0x0000000F;
		}
		OutputArray[i] = updateResult;
	}

	// a. Subtraction of original and data obtained by Kalman filter tracking.
	arm_sub_f32(OutputArray, InputArray, diffArray, u_len);

	// b. Calculation of the standard deviation and the average of the difference obtained in a).
	arm_mean_f32(diffArray, u_len, &avgDiff);
	arm_std_f32(diffArray, u_len, &stdDiff);

	// c. Calculation of the correlation between the original and tracked vectors.
	arm_correlate_f32(InputArray, u_len, OutputArray, u_len, corrArray);

	// d. Calculation of the convolution between the two vectors.
	arm_conv_f32(InputArray, u_len, OutputArray, u_len, convArray);

	return 0;
}
