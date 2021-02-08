/*
 * lab1util.h
 *
 *  Created on: Jan 20, 2021
 *      Author: ah
 */

#ifndef INC_LAB1UTIL_H_
#define INC_LAB1UTIL_H_

struct KalmanState{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //estimated value
	float p; //estimation error covariance
	float k; // adaptive Kalman filter gain.;
};

/*
 * Single measurement update
 */
extern float kalmanUpdateA (struct KalmanState* ksp, float measurement);
float kalmanUpdateC (struct KalmanState* ksp, float measurement);

/*
 * Combos
 * -InputArray is the array of measurements
 * -OutputArray is the array of values x obtained by Kalman fiter calculations over the input field
 * -Integer Length specifies the length of the data array to process
 * The output encodes whether the function executed properly (by returning 0) or the result is not a
 * correct arithmetic value (e.g., it has run into numerical conditions leading to NaN)
 */

int kalmanFilterL (const float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis);
int kalmanFilterC (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis);

extern int kalmanFilterA (const float* InputArray, float* OutputArray, struct KalmanState* kstate, int length,
		float* DiffArray, float* inAvg, float* outAvg, float* diffAvg);
extern int kalmanStatsA (const float* InputArray, const float* OutputArray, const float* DiffArray, int length,
		float inAvg, float outAvg, float diffAvg,
		float* diffStd, float* corrCoef);
extern int kalmanFilterA_noStats (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length);

int kalmanFilterAinC (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis);
int kalmanFilterAinL (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis);
int kalmanFilterCinL (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length, int analysis);
/*
 * Utilities written with C
 */
float sumSqDev (float* inputArray, float avg, int length);
float corrCoefC (float* inputArray1, float* inputArray2, float avg1, float avg2, int length);
int corrC (float* inputArray1, float* inputArray2, float* corrArray, int length);
int convC (float* inputArray1, float* inputArray2, float* convolvedArray, int length1, int length2);


#endif /* INC_LAB1UTIL_H_ */
