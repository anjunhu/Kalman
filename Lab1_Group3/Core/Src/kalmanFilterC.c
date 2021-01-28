/*
 * kalmanFilterC.c
 *
 *  Created on: Jan 23, 2021
 *      Author: ah
 */
#include "main.h"
#include "math.h"
#include "lab1util.h"
#include "cmsis_gcc.h"

int kalmanFilterC(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length){
//	float avgIn = 0.0;
//	float avgOut = 0.0;
//	float avgDiff = 0.0;
//
//
//	float diffArray[length];
//	float convArray[length*2-1];

	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);

	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateC(kstate, InputArray[i]);
		if (isnan(updateResult) || isinf(updateResult)){
			//Return FPSCR as an error flag
			return __get_FPSCR() & 0x0000000F;
		}
		OutputArray[i] = updateResult;
//		diffArray[i] = OutputArray[i] - InputArray[i];
//
//		avgIn += InputArray[i];
//		avgOut += OutputArray[i];
//		avgDiff += diffArray[i];
	}

//	avgIn = avgIn/(float)length;
//	avgOut = avgOut/(float)length;
//	avgDiff = avgDiff/(float)length;
//
//	float varDiff = sumSqDev(diffArray, avgDiff, length) / (float)length;
//	float stdDiff = powf(varDiff, 0.5);
//	float correlation = corrC(InputArray, OutputArray, avgIn, avgOut, length);
//
//	convC(InputArray, OutputArray, convArray, length, length);

	return 0;
}


int kalmanFilterAinC(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length){

	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateA(kstate, InputArray[i]);
		if (isnan(updateResult) || isinf(updateResult)){
			return __get_FPSCR() & 0x0000000F;
		}
		OutputArray[i] = updateResult;
	}

	return 0;
}



/*
 * PART II
 */

float sumSqDev (float* inputArray, float avg, int length){
	float sumSqDev = 0.0;
	for(int i = 0; i < length; i++){
		sumSqDev += powf(inputArray[i] - avg, 2);
	}
	return sumSqDev;
}

float corrC (float* inputArray1, float* inputArray2, int avg1, int avg2, int length){
	float corNume = 0.0;
	float corDeno = 0.0;

	for(int i = 0; i < length; i++){
		corNume += (inputArray1[i] - avg1) * (inputArray2[i] - avg2);
	}
	corDeno = powf( (sumSqDev(inputArray1, avg1, length)*sumSqDev(inputArray2, avg2, length)) , 0.5);

	return corNume/corDeno;
}

int convC (float* inputArrayL, float* inputArrayS, float* convolvedArray, int lengthL, int lengthS){
	for (int i = 0; i < lengthS + lengthL -1; i++){
		convolvedArray[i] = 0.0;
			for (int j = 0; j < lengthL && i>=j ; j++){
				if ((i-j) < lengthS){
					convolvedArray[i] += inputArrayL[j] * inputArrayS[i-j];
				}

			}
		}
	return 0;
}
