.section .text //AREA PROGRAM, CODE

//.global (EXPORT) is a keyword indicating functions,
// variables, etc. visible to external segment
.global kalmanUpdateA

/**
* extern float kalmanUpdateA ( KalmanState* ks, float measurement);
*
* R0 = pointer to struct kalmanState
* S0 = measurement, then the return value x
*/

kalmanUpdateA:
			PUSH {R4, LR}
			VSTMDB.f32 SP!,{S4-S9}

			VLDMIA.f32 R0!, {S4-S8}

			VADD.f32 S7, S7, S4
			VADD.f32 S9, S7, S5
			VDIV.f32 S8, S7, S9
			VSUB.f32 S9, S0, S6
			VMLA.f32 S6, S8, S9
			VMLS.f32 S7, S8, S7

			VMRS R4, FPSCR
			LSL R4, R4, #28
			BNE exception

			VMOV.f32 S0, S6					// return self.x
			VSTMDB.f32 R0!, {S4-S8} 		// !!We still need this right???
			VLDMIA.f32 SP!,{S4-S9}
			POP {R4, PC}

exception:	VMOV.f32 S0, S6					// force NaN or let it be x?
			VLDMIA.f32 SP!,{S4-S9}
			POP {R4, PC}

.end
