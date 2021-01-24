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
	if (isnan(ksp->k) || isnan(ksp->p) || isnan(ksp->q) || isnan(ksp->x) || isnan(ksp->r)
			|| isinf(ksp->k) || isinf(ksp->p) || isinf(ksp->q) || isinf(ksp->x) || isinf(ksp->r)){
		return NAN;
	}

	struct KalmanState ks;
	ks = *ksp;

	ks.p += ks.q;
	ks.k = ks.p / (ks.p + ks.r);
	ks.x += ks.k * (measurement - ks.x);
	ks.p -= ks.k * ks.p;

	if (isnan(ks.k) || isnan(ks.p) || isnan(ks.q) || isnan(ks.x) || isnan(ks.r)
				|| isinf(ks.k) || isinf(ks.p) || isinf(ks.q) || isinf(ks.x) || isinf(ks.r)){
			return NAN;
		}
	memcpy(&ks, ksp, sizeof(ks));
	return ks.x;
}
