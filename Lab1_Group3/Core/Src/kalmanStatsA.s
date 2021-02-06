.syntax unified	// without this we get SUBS instruction not supported in Thumb16 mode
.section .text	//AREA PROGRAM, CODE

//.global (EXPORT) is a keyword indicating functions,
// variables, etc. visible to external segment
.global kalmanStatsA

/**
* extern int kalmanStatsA (
*		float* InputArray, float* OutputArray, float* DiffArray, int length,
*		float inAvg, float outAvg, float diffAvg,
*		float* diffStd, float* corrCoef);
*/

kalmanStatsA:
			PUSH {R4-R6, LR}		// 4 ints

			MOV R4, R0				// local pointer to current element in InputArray
			LDR R5, [sp, #16]		// pointer to diffStd
			LDR R6, [sp, #20]		// pointer to corrCoef

			VSTMDB.f32 SP!,{S4-S8}

			VMRS R0, FPSCR			// flushing out the error code
			BIC R0, R0, #15
			VMSR FPSCR, R0

			VSUB.f32 S7, S7, S7		// stdDiff = 0.0;
			VSUB.f32 S8, S8, S8		// corNume = 0.0;
			VSUB.f32 S9, S9, S9		// corDenoIn = 0.0;
			VSUB.f32 S10, S10, S10		// corDenoOut = 0.0;
			VMOV.f32 S11, R3
			VCVT.f32.S32 S11, S11		// (float) length

loop:		SUBS R3, R3, #1
			BLT return

			VLDR.f32 S4, [R4]		// S4 = current InputArray element
			VLDR.f32 S5, [R1]		// S5 = current OutArray element
			VLDR.f32 S6, [R2]		// S6 = current DiffArray element

			// sumSqDev += powf(array[i] - avg, 2);
			VSUB.f32 S4, S4, S0
			VSUB.f32 S5, S5, S1
			VSUB.f32 S6, S6, S2
			VMLA.f32 S7, S6, S6		// stdDiff
			VMLA.f32 S8, S4, S5		// corNume
			VMLA.f32 S9, S4, S4		// corDenoIn
			VMLA.f32 S10, S5, S5	// corDenoOut

			VMRS R0, FPSCR
			ANDS R0, R0, #15		// check for exceptions LSL R0, R0, #28
			BNE exception

			ADD R4, R4, #4			// Moving on for I, O, D arrays
			ADD R1, R1, #4
			ADD R2, R2, #4
			B loop

return:
			VDIV.f32 S7, S7, S11	// varDiff = sumSqDev(diffArray, avgDiff, length) / (float)length;
			VSQRT.f32 S7, S7		// stdDiff

			VMUL.f32 S9, S9, S10	// corDeno = sqrt((sumSqDev(In)*sumSqDev(Out)));
			VSQRT.f32 S9, S9
			VDIV.f32 S9, S8, S9		// corrCoef = num / denom

			VSTR.f32 S7, [R5]
			VSTR.f32 S9, [R6]

			VSTMDB.f32 R2!, {S4-S8} // update kstate, but only if everything went well...
			VLDMIA.f32 SP!,{S4-S8}
			POP {R4-R6, PC}

exception:
			VLDMIA.f32 SP!,{S4-S8}
			POP {R4-R6, PC}

.end

