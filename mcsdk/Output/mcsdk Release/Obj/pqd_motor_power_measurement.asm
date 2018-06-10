	.cpu cortex-m4
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 1
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"pqd_motor_power_measurement.c"
	.section	.text.PQD_CalcElMotorPower,"ax",%progbits
	.align	1
	.global	PQD_CalcElMotorPower
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	PQD_CalcElMotorPower, %function
PQD_CalcElMotorPower:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	mov	r5, r0
	ldr	r3, [r0, #268]
	ldrsh	r6, [r3, #12]
	ldrsh	r2, [r3, #22]
	ldrsh	r1, [r3, #14]
	ldrsh	r3, [r3, #24]
	mul	r3, r3, r1
	mla	r6, r2, r6, r3
	ldr	r4, [r0, #264]
	ldr	r0, [r0, #272]
	bl	VBS_GetAvBusVoltage_V
	mul	r0, r4, r0
	mov	r3, r6
	cmp	r6, #0
	itt	lt
	addlt	r3, r6, #65280
	addlt	r3, r3, #255
	asrs	r1, r3, #16
	ldr	r2, .L4
	smull	r3, r2, r2, r0
	asrs	r3, r0, #31
	rsb	r3, r3, r2, asr #6
	mul	r3, r3, r1
	add	r3, r3, r3, lsl #1
	lsls	r3, r3, #1
	ldr	r1, .L4+4
	smull	r2, r1, r1, r3
	asrs	r3, r3, #31
	rsb	r1, r3, r1, asr #18
	mov	r0, r5
	bl	MPM_CalcElMotorPower
	pop	{r4, r5, r6, pc}
.L5:
	.align	2
.L4:
	.word	458129845
	.word	1717986919
	.size	PQD_CalcElMotorPower, .-PQD_CalcElMotorPower
	.ident	"GCC: (GNU) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]"
