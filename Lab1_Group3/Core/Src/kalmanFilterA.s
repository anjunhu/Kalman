.syntax unified	// without this we get SUBS instruction not supported in Thumb16 mode
.section .text	//AREA PROGRAM, CODE

//.global (EXPORT) is a keyword indicating functions,
// variables, etc. visible to external segment
.global kalmanFilterA

/**
* int kalmanFilterA (float* InputArray, float* OutputArray, struct KalmanState* kstate, int length  	// R0-R3
* 					 float* DiffArray, float* avgIn, float* avgOut, float* avgDiff);					// on the stack
*/

kalmanFilterA:
			PUSH {R4-R8, LR}		// 6 ints

			MOV R4, R0			// local pointer to current element in InputArray
			LDR R5, [sp, #24]	// pointer to DiffArray
			LDR R6, [sp, #28]	// pointer to avgIn
			LDR R7, [sp, #32]	// pointer to avgOut
			LDR R8, [sp, #36]	// pointer to avgDiff

			VPUSH.f32 {S4-S14}
			VLDMIA.f32 R2!, {S4-S8} // local copy of kstate

			VMRS R0, FPSCR			// flushing out previous error code
			BIC R0, R0, #15
			VMSR FPSCR, R0

			VSUB.f32 S11, S11, S11		// Avg In = 0.0
			VSUB.f32 S12, S12, S12		// Avg Out = 0.0
			VSUB.f32 S13, S13, S13		// Avg Diff = 0.0
			VMOV.f32 S14, R3
			VCVT.f32.S32 S14, S14		// (float) length

loop:		SUBS R3, R3, #1
			BLT return

			VLDR.f32 S10, [R4]		// S10 = current InputArray element

			VADD.f32 S7, S7, S4 	// p = p + q
			VADD.f32 S9, S7, S5 	// p + r
			VDIV.f32 S8, S7, S9 	// k = p / (p + r)
			VSUB.f32 S9, S10, S6 	// measurement - x
			VMLA.f32 S6, S8, S9 	// x = x + k*(measurement - x)
			VMLS.f32 S7, S8, S7 	// p = p - k*p

			VSUB.f32 S9, S6, S10
			VADD.f32 S12, S12, S6	// avgOut += OutputArray[i]; S6 = x
			VADD.f32 S11, S11, S10	// avgIn += InputArray[i]; S10 is the measurement
			VADD.f32 S13, S13, S9	// avgDiff += diffArray[i];

			VMRS R0, FPSCR
			ANDS R0, R0, #15		// check for exceptions LSL R0, R0, #28
			BNE exception

			VSTR.f32 S6, [R1]		// current OutputArray element = x
			VSTR.f32 S9, [R5] 		// diffArray[i] = OutputArray[i] - InputArray[i];

			ADD R4, R4, #4			// Moving on for I, O, D arrays
			ADD R1, R1, #4
			ADD R5, R5, #4
			B loop

return:
			VDIV.f32 S11, S11, S14	// avgIn = avgIn/(float)length;
			VDIV.f32 S12, S12, S14	// avgOut = avgOut/(float)length;
			VDIV.f32 S13, S13, S14	// avgDiff = avgDiff/(float)length;

			VSTR.f32 S11, [R6]		// pointer to avgIn
			VSTR.f32 S12, [R7]		// pointer to avgOut
			VSTR.f32 S13, [R8]		// pointer to avgDiff

			VSTMDB.f32 R2!, {S4-S8} // update kstate, but only if everything went well...
			VPOP.f32 {S4-S14}
			POP {R4-R8, PC}

exception:
			VPOP.f32 {S4-S14}
			POP {R4-R8, PC}

.end
