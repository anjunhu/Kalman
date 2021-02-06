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

int kalmanFilterL(float* InputArray, float* OutputArray, struct KalmanState* ks, int length, int analysis){
	float stdDiff= 0.0;
	float avgDiff = 0.0;
	uint32_t u_len = (uint32_t) length;
	float diffArray[length];
	float corrArray[length*2-1];
	float convArray[length*2-1];

	arm_biquad_casd_df1_inst_f32 filter_obj;
	uint16_t numStages = 1;
	uint32_t blockSize = 1;
	float32_t pCoeffs[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	float32_t pState[4];
	arm_biquad_cascade_df1_init_f32 (&filter_obj, numStages, pCoeffs, pState);
	pState[2] = ks->x;

	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);
	int status = 0;

	for(int i = 0; i < length; i++){
		ks->p += ks->q;
		ks->k = ks->p / (ks->p + ks->r);
		pCoeffs[0] = ks->k;
		pCoeffs[3] = 1-(ks->k);
		filter_obj.pCoeffs = pCoeffs;
		arm_biquad_cascade_df1_f32(&filter_obj, InputArray+i, OutputArray+i, blockSize);
		ks->x = *(OutputArray+i);
		ks->p -= ks->k * ks->p;

		status = __get_FPSCR() & 0x0000000F;
		if (status != 0)
			return status;
	}

	if (analysis != 0){
		// a. Subtraction of original and data obtained by Kalman filter tracking.
		arm_sub_f32(OutputArray, InputArray, diffArray, u_len);

		// b. Calculation of the standard deviation and the average of the difference obtained in a).
		arm_mean_f32(diffArray, u_len, &avgDiff);
		arm_std_f32(diffArray, u_len, &stdDiff);

		// c. Calculation of the correlation between the original and tracked vectors.
		arm_correlate_f32(InputArray, u_len, OutputArray, u_len, corrArray);

		// d. Calculation of the convolution between the two vectors.
		arm_conv_f32(InputArray, u_len, OutputArray, u_len, convArray);
	}


	return status;
}



int kalmanFilterAinL(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis){
	// a. Subtraction of original and data obtained by Kalman filter tracking.
	float diffArray[length];
	uint32_t u_len = (uint32_t) length;

	// b. Calculation of the standard deviation and the average of the difference obtained in a).
	float avgDiff = 0.0;
	float stdDiff = 0.0;

	// c. Correlation
	float avgIn = 0.0;
	float avgOut = 0.0;
	float corrCoef = 0.0;

	float convArray[length*2-1];
	float corrArray[length*2-1];

	int status = 0;
	status = kalmanFilterA_noStats(InputArray, OutputArray, kstate, length);
	if (status != 0)
		return status;

	if (analysis != 0){
		// a. Subtraction of original and data obtained by Kalman filter tracking.
		arm_sub_f32(OutputArray, InputArray, diffArray, u_len);

		// b. Calculation of the standard deviation and the average of the difference obtained in a).
		arm_mean_f32(diffArray, u_len, &avgDiff);
		arm_std_f32(diffArray, u_len, &stdDiff);

		// c. Calculation of the correlation between the original and tracked vectors.
		arm_correlate_f32(InputArray, u_len, OutputArray, u_len, corrArray);

		// d. Calculation of the convolution between the two vectors.
		arm_conv_f32(InputArray, u_len, OutputArray, u_len, convArray);
	}



	return status;
}


int kalmanFilterCinL(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis){
	// a. Subtraction of original and data obtained by Kalman filter tracking.
	float diffArray[length];
	uint32_t u_len = (uint32_t) length;

	// b. Calculation of the standard deviation and the average of the difference obtained in a).
	float avgDiff = 0.0;
	float stdDiff = 0.0;

	// c. Correlation
	float avgIn = 0.0;
	float avgOut = 0.0;
	float corrCoef = 0.0;

	float convArray[length*2-1];
	float corrArray[length*2-1];

	int status = 0;
	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateC(kstate, InputArray[i]);
		status = __get_FPSCR() & 0x0000000F;
		if (status != 0)
			return status;
		OutputArray[i] = updateResult;
	}

	if (analysis != 0){
		// a. Subtraction of original and data obtained by Kalman filter tracking.
		arm_sub_f32(OutputArray, InputArray, diffArray, u_len);

		// b. Calculation of the standard deviation and the average of the difference obtained in a).
		arm_mean_f32(diffArray, u_len, &avgDiff);
		arm_std_f32(diffArray, u_len, &stdDiff);

		// c. Calculation of the correlation between the original and tracked vectors.
		arm_correlate_f32(InputArray, u_len, OutputArray, u_len, corrArray);

		// d. Calculation of the convolution between the two vectors.
		arm_conv_f32(InputArray, u_len, OutputArray, u_len, convArray);
	}



	return status;
}
