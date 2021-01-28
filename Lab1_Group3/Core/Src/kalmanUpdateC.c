/*
 * kalmanFilterC.c
 *
 *  Created on: Jan 23, 2021
 *      Author: ah
 */
#include "math.h"
#include "string.h"
#include "lab1util.h"

float kalmanUpdateC(struct KalmanState* ksp, float measurement){
//	if (isnan(ksp->k) || isnan(ksp->p) || isnan(ksp->q) || isnan(ksp->x) || isnan(ksp->r)
//			|| isinf(ksp->k) || isinf(ksp->p) || isinf(ksp->q) || isinf(ksp->x) || isinf(ksp->r)){
//		return NAN;
//	}

	struct KalmanState ks;
	memcpy(&ks, ksp, sizeof(ks));

	ks.p += ks.q;
	ks.k = ks.p / (ks.p + ks.r);
	ks.x += ks.k * (measurement - ks.x);
	ks.p -= ks.k * ks.p;

	if (isnan(ks.k) || isinf(ks.p) || isinf(ks.x) || isnan(ks.x)
			|| isnan(ks.p) || isinf(ks.k))	return NAN;

	memcpy(ksp, &ks, sizeof(ks));
	return ks.x;
}
