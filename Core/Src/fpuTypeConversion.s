  .syntax unified
	.cpu cortex-m4
	.fpu vfpv3
	.thumb

.global fixedToFloat
.global floatToFixed

fixedToFloat:
	vldmia.32 a1, {s0-s1}
    vcvt.f32.s16 s0, s0, #0xf
    vcvt.f32.s16 s1, s1, #0xf
    vstmia.32 a2, {s0-s1}
    mov pc,lr

floatToFixed:
	vcvt.s32.f32 s0, s0, #0xf
	mov pc,lr
