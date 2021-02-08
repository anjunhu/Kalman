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

int kalmanFilterL(const float* InputArray, float* OutputArray, struct KalmanState* ksp, int length, int analysis){
	arm_biquad_casd_df1_inst_f32 filter_obj;
	uint16_t numStages = 1;
	uint32_t blockSize = 1;
	float32_t pCoeffs[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	float32_t pState[4];
	arm_biquad_cascade_df1_init_f32 (&filter_obj, numStages, pCoeffs, pState);
	pState[2] = ksp->x;
	float temp = 0.0;

	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);
	int status = 0;

	for(int i = 0; i < length; i++){
		ksp->p += ksp->q;
//		arm_add_f32(&ksp->p, &ksp->q, &ksp->p, 1);

		ksp->k = ksp->p / (ksp->p + ksp->r);
//		arm_add_f32(&ksp->p, &ksp->r, &temp, 1);
//		ksp->k = ksp->p / temp;

		pCoeffs[0] = ksp->k;
//		arm_copy_f32(&ksp->k, pCoeffs, 1);
		pCoeffs[3] = 1-(ksp->k);
//		arm_negate_f32(&ksp->k, pCoeffs+3, 1);
//		arm_offset_f32(pCoeffs+3, 1.0, pCoeffs+3, 1);

		filter_obj.pCoeffs = pCoeffs;
		arm_biquad_cascade_df1_f32(&filter_obj, InputArray+i, OutputArray+i, blockSize);

		ksp->x = *(OutputArray+i);
//		arm_copy_f32(OutputArray+i, &ksp->x, 1);

		ksp->p -= ksp->k * ksp->p;
//		arm_mult_f32(&ksp->p, &ksp->k, &temp, 1);
//		arm_sub_f32(&ksp->p, &temp, &ksp->p, 1);

		status = __get_FPSCR() & 0x0000000F;
		if (status != 0)
			return status;
	}

	if (analysis != 0){
		status = dsp_analysis(InputArray, OutputArray, (uint32_t)length);
	}

	return status;
}



int kalmanFilterAinL(float* InputArray, float* OutputArray, struct KalmanState* ksp, int length, int analysis){
	int status = 0;
	status = kalmanFilterA_noStats(InputArray, OutputArray, ksp, length);
	if (status != 0)
		return status;

	if (analysis != 0){
		status = dsp_analysis(InputArray, OutputArray, (uint32_t)length);
	}

	return status;
}



int kalmanFilterCinL(float* InputArray, float* OutputArray, struct KalmanState* ksp, int length, int analysis){
	int status = 0;
	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateC(ksp, InputArray[i]);
		status = __get_FPSCR() & 0x0000000F;
		if (status != 0)
			return status;
		OutputArray[i] = updateResult;
	}

	if (analysis != 0){
		status = dsp_analysis(InputArray, OutputArray, (uint32_t)length);
	}

	return status;
}






int dsp_analysis (float* InputArray, float* OutputArray, uint32_t u_len){
	// clear error flags
	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);
	float flength = (float) u_len;
	// a. Subtraction of original and data obtained by Kalman filter tracking.
	float diffArray[u_len];
	arm_sub_f32(OutputArray, InputArray, diffArray, u_len);

	// b. Calculation of the standard deviation and the average of the difference obtained in a).
	float avgDiff = 0.0;
	float stdDiff = 0.0;
	arm_mean_f32(diffArray, u_len, &avgDiff);
	arm_std_f32(diffArray, u_len, &stdDiff);

	// c. Calculation of the correlation between the original and tracked vectors.
	float corrArray[u_len*2-1];
	arm_correlate_f32(InputArray, u_len, OutputArray, u_len, corrArray);

	float avgIn = 0.0;
	float avgOut = 0.0;
	float corrCoef = 0.0;
	float corrNume = 0.0;
	float tempAryIn[u_len];
	float tempAryOut[u_len];
	float corDeno = 0.0;
	arm_mean_f32(InputArray, u_len, &avgIn);
	arm_mean_f32(OutputArray, u_len, &avgOut);
	arm_offset_f32(InputArray, -avgIn, tempAryIn, u_len);
	arm_offset_f32(OutputArray, -avgOut, tempAryOut, u_len);
	arm_dot_prod_f32(tempAryIn, tempAryOut, u_len, &corrNume);
	float rms_in;
	float rms_out;
	arm_rms_f32 (tempAryIn, u_len, &rms_in);
	arm_rms_f32 (tempAryOut, u_len, &rms_out);
	arm_mult_f32(&rms_in, &rms_out, &corDeno, 1);
	arm_mult_f32(&corDeno, &flength, &corDeno, 1);
	corrCoef = corrNume/corDeno;

	// d. Calculation of the convolution between the two vectors.
	float convArray[u_len*2-1];
	arm_conv_f32(InputArray, u_len, OutputArray, u_len, convArray);

	return  __get_FPSCR() & 0x0000000F;
}
