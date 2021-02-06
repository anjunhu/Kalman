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

int kalmanFilterC(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis){
	float avgIn = 0.0;
	float avgOut = 0.0;
	float avgDiff = 0.0;

	// a. Subtraction of original and data obtained by Kalman filter tracking.
	float diffArray[length];
	float corrArray[length*2-1];
	float convArray[length*2-1];

	__set_FPSCR(__get_FPSCR() & 0xFFFFFFF0);
	int status = 0;

	for(int i = 0; i < length; i++){
		float updateResult = kalmanUpdateC(kstate, InputArray[i]);
		status = __get_FPSCR() & 0x0000000F;
		if (status != 0)
			return status;
		OutputArray[i] = updateResult;

		if (analysis != 0){
			diffArray[i] = updateResult - InputArray[i];
			avgIn += InputArray[i];
			avgOut += updateResult;
			avgDiff += diffArray[i];
		}

	}

	if (analysis != 0){
		// b. Calculation of the standard deviation and the average of the difference obtained in a).
		avgIn = avgIn/(float)length;
		avgOut = avgOut/(float)length;
		avgDiff = avgDiff/(float)length;

		float varDiff = sumSqDev(diffArray, avgDiff, length) / (float)length;
		float stdDiff = powf(varDiff, 0.5);

		// c. Calculation of the correlation between the original and tracked vectors.
		float correlation = corrCoefC(InputArray, OutputArray, avgIn, avgOut, length);
		status = corrC(InputArray, OutputArray, corrArray, length);

		// d. Calculation of the convolution between the two vectors.
		convC(InputArray, OutputArray, convArray, length, length);
	}
	return 0;
}


int kalmanFilterAinC(float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis){
	// a. Subtraction of original and data obtained by Kalman filter tracking.
	float diffArray[length];

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

	if (analysis == 0){
		status = kalmanFilterA_noStats(InputArray, OutputArray, kstate, length);
		if (status != 0)
			return status;
	}
	else{
		status = kalmanFilterA(InputArray, OutputArray, kstate, length,
			diffArray, &avgIn, &avgOut, &avgDiff);
		if (status != 0)
			return status;

		status = kalmanStatsA(InputArray, OutputArray, diffArray, length,
					avgIn, avgOut, avgDiff,
					&stdDiff, &corrCoef);
		if (status != 0)
			return status;

		// c. Calculation of the correlation between the original and tracked vectors.
		status = corrC(InputArray, OutputArray, corrArray, length);

		// d. Calculation of the convolution between the two vectors.
		convC(InputArray, OutputArray, convArray, length, length);
	}

	return status;
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

float corrCoefC (float* inputArray1, float* inputArray2, int avg1, int avg2, int length){
	float corNume = 0.0;
	float corDeno = 0.0;

	for(int i = 0; i < length; i++){
		corNume += (inputArray1[i] - avg1) * (inputArray2[i] - avg2);
	}
	corDeno = powf( (sumSqDev(inputArray1, avg1, length)*sumSqDev(inputArray2, avg2, length)) , 0.5);

	return corNume/corDeno;
}

int corrC (float* inputArrayL, float* inputArrayS, float* corrArray, int length){
	for (int i = 0; i < length + length -1; i++){
		float temp = 0.0;
		if (i<length){
			int k = 0;
			for (int j = length-1-i; j <length; j++){
				temp += inputArrayL[k] * inputArrayS[j];
				k++;
			}
		}
		else{
			int j = 0 ;
			for (int k = i-length+1; k < length; k++){
				temp += inputArrayL[k] * inputArrayS[j];
				j++;
			}

		}
		corrArray[i] = temp;
	}
	return 0;
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
