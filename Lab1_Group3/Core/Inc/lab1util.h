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
extern float kalmanUpdateA (struct KalmanState* ks, float measurement);
float kalmanUpdateC (struct KalmanState* ks, float measurement);
float kalmanUpdateL (struct KalmanState* ks, float measurement);

/*
 * Combos
 * -InputArray is the array of measurements
 * -OutputArray is the array of values x obtained by Kalman fiter calculations over the input field
 * -Integer Length specifies the length of the data array to process
 * The output encodes whether the function executed properly (by returning 0) or the result is not a
 * correct arithmetic value (e.g., it has run into numerical conditions leading to NaN)
 */

int kalmanFilterL (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length);
extern int kalmanFilterA (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length);
int kalmanFilterAinC (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length);
int kalmanFilterC (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length);

/*
 * Utilities written with C
 */
float sumSqDev (float* inputArray, float avg, int length);
float corrC (float* inputArray1, float* inputArray2, int avg1, int avg2, int length);
int convC (float* inputArray1, float* inputArray2, float* convolvedArray, int length1, int length2);


#endif /* INC_LAB1UTIL_H_ */
