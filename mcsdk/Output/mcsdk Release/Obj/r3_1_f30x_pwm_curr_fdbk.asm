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
	.file	"r3_1_f30x_pwm_curr_fdbk.c"
	.section	.text.R3_1_F30X_GetPhaseCurrents,"ax",%progbits
	.align	1
	.global	R3_1_F30X_GetPhaseCurrents
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_GetPhaseCurrents, %function
R3_1_F30X_GetPhaseCurrents:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4, r5, r6}
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	mvn	r2, #1
	str	r2, [r3, #16]
	ldr	r3, [r0, #184]
	ldr	r3, [r3]
	ldr	r4, [r3, #128]
	ldr	r2, [r3, #132]
	ldrb	r3, [r0, #78]	@ zero_extendqisi2
	cmp	r3, #5
	bhi	.L2
	tbb	[pc, r3]
.L4:
	.byte	(.L3-.L4)/2
	.byte	(.L5-.L4)/2
	.byte	(.L5-.L4)/2
	.byte	(.L6-.L4)/2
	.byte	(.L6-.L4)/2
	.byte	(.L3-.L4)/2
	.p2align 1
.L6:
	cmp	r3, #3
	ite	eq
	uxtheq	r5, r2
	uxthne	r5, r4
	ldr	r6, [r0, #116]
	subs	r5, r6, r5
	cmn	r5, #32768
	bgt	.L9
	movw	r5, #32769
	strh	r5, [r1]	@ movhi
.L10:
	cmp	r3, #3
	beq	.L35
	uxth	r2, r2
	ldr	r4, [r0, #120]
	subs	r2, r4, r2
.L13:
	cmn	r2, #32768
	ble	.L36
	cmp	r2, #32768
	itte	ge
	movwge	r3, #32767
	strhge	r3, [r1, #2]	@ movhi
	strhlt	r2, [r1, #2]	@ movhi
.L2:
	ldrsh	r3, [r1]
	strh	r3, [r0, #94]	@ movhi
	ldrsh	r3, [r1, #2]
	strh	r3, [r0, #96]	@ movhi
	ldrh	r3, [r1]
	ldrh	r2, [r1, #2]
	add	r3, r3, r2
	negs	r3, r3
	strh	r3, [r0, #98]	@ movhi
	pop	{r4, r5, r6}
	bx	lr
.L9:
	cmp	r5, #32768
	it	ge
	movwge	r5, #32767
	strh	r5, [r1]	@ movhi
	b	.L10
.L35:
	uxth	r4, r4
	ldr	r2, [r0, #120]
	subs	r2, r2, r4
	b	.L13
.L36:
	movw	r3, #32769
	strh	r3, [r1, #2]	@ movhi
	b	.L2
.L3:
	cmp	r3, #5
	ite	eq
	uxtheq	r5, r2
	uxthne	r5, r4
	ldr	r6, [r0, #120]
	subs	r5, r6, r5
	cmn	r5, #32768
	bgt	.L18
	movw	r5, #32769
	strh	r5, [r1, #2]	@ movhi
.L19:
	cmp	r3, #5
	beq	.L37
	uxth	r2, r2
	ldr	r3, [r0, #124]
	subs	r3, r3, r2
.L22:
	negs	r3, r3
	ldrsh	r2, [r1, #2]
	subs	r3, r3, r2
	cmp	r3, #32768
	bge	.L38
	cmn	r3, #32768
	it	le
	movwle	r3, #32769
	strh	r3, [r1]	@ movhi
	b	.L2
.L18:
	cmp	r5, #32768
	it	ge
	movwge	r5, #32767
	strh	r5, [r1, #2]	@ movhi
	b	.L19
.L37:
	uxth	r4, r4
	ldr	r3, [r0, #124]
	subs	r3, r3, r4
	b	.L22
.L38:
	movw	r3, #32767
	strh	r3, [r1]	@ movhi
	b	.L2
.L5:
	cmp	r3, #2
	ite	eq
	uxtheq	r5, r2
	uxthne	r5, r4
	ldr	r6, [r0, #116]
	subs	r5, r6, r5
	cmn	r5, #32768
	bgt	.L27
	movw	r5, #32769
	strh	r5, [r1]	@ movhi
.L28:
	cmp	r3, #2
	beq	.L39
	uxth	r2, r2
	ldr	r3, [r0, #124]
	subs	r3, r3, r2
.L31:
	negs	r3, r3
	ldrsh	r2, [r1]
	subs	r3, r3, r2
	cmp	r3, #32768
	bge	.L40
	cmn	r3, #32768
	it	le
	movwle	r3, #32769
	strh	r3, [r1, #2]	@ movhi
	b	.L2
.L27:
	cmp	r5, #32768
	it	ge
	movwge	r5, #32767
	strh	r5, [r1]	@ movhi
	b	.L28
.L39:
	uxth	r4, r4
	ldr	r3, [r0, #124]
	subs	r3, r3, r4
	b	.L31
.L40:
	movw	r3, #32767
	strh	r3, [r1, #2]	@ movhi
	b	.L2
	.size	R3_1_F30X_GetPhaseCurrents, .-R3_1_F30X_GetPhaseCurrents
	.section	.text.R3_1_F30X_HFCurrentsCalibrationAB,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_HFCurrentsCalibrationAB, %function
R3_1_F30X_HFCurrentsCalibrationAB:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	mvn	r2, #1
	str	r2, [r3, #16]
	ldrb	r3, [r0, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L41
	ldr	r2, [r0, #184]
	ldr	r3, [r2]
	ldr	r1, [r3, #128]
	ldr	r3, [r0, #116]
	add	r3, r3, r1
	str	r3, [r0, #116]
	ldr	r3, [r2]
	ldr	r2, [r3, #132]
	ldr	r3, [r0, #120]
	add	r3, r3, r2
	str	r3, [r0, #120]
	ldrb	r3, [r0, #176]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r0, #176]
.L41:
	bx	lr
	.size	R3_1_F30X_HFCurrentsCalibrationAB, .-R3_1_F30X_HFCurrentsCalibrationAB
	.section	.text.R3_1_F30X_HFCurrentsCalibrationC,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_HFCurrentsCalibrationC, %function
R3_1_F30X_HFCurrentsCalibrationC:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	mvn	r2, #1
	str	r2, [r3, #16]
	ldrb	r3, [r0, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L43
	ldr	r3, [r0, #184]
	ldr	r3, [r3]
	ldr	r2, [r3, #128]
	ldr	r3, [r0, #124]
	add	r3, r3, r2
	str	r3, [r0, #124]
	ldrb	r3, [r0, #176]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r0, #176]
.L43:
	bx	lr
	.size	R3_1_F30X_HFCurrentsCalibrationC, .-R3_1_F30X_HFCurrentsCalibrationC
	.section	.text.R3_1_F30X_TurnOnLowSides,"ax",%progbits
	.align	1
	.global	R3_1_F30X_TurnOnLowSides
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_TurnOnLowSides, %function
R3_1_F30X_TurnOnLowSides:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	movs	r2, #1
	strb	r2, [r0, #88]
	mvn	r2, #1
	str	r2, [r3, #16]
	movs	r2, #0
	str	r2, [r3, #52]
	str	r2, [r3, #56]
	str	r2, [r3, #60]
.L46:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L46
	ldr	r2, [r3, #68]
	orr	r2, r2, #32768
	str	r2, [r3, #68]
	ldr	r3, [r0, #184]
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L49
.L45:
	bx	lr
.L49:
	ldr	r2, [r3, #20]
	ldrh	r3, [r3, #24]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #24]
	b	.L45
	.size	R3_1_F30X_TurnOnLowSides, .-R3_1_F30X_TurnOnLowSides
	.section	.text.R3_1_F30X_SwitchOnPWM,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SwitchOnPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SwitchOnPWM, %function
R3_1_F30X_SwitchOnPWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	movs	r2, #0
	strb	r2, [r0, #88]
	mvn	r2, #1
	str	r2, [r3, #16]
.L51:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L51
	mvn	r2, #1
	str	r2, [r3, #16]
	ldrb	r2, [r0, #93]	@ zero_extendqisi2
	cbz	r2, .L52
	movs	r2, #1
	str	r2, [r3, #52]
	ldr	r2, [r0, #184]
	ldr	r2, [r2]
	ldr	r1, [r0, #132]
	str	r1, [r2, #76]
.L53:
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #56]
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #60]
	ldrh	r2, [r0, #128]
	subs	r2, r2, #5
	str	r2, [r3, #64]
.L54:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L54
	ldr	r2, [r3, #68]
	orr	r2, r2, #1024
	str	r2, [r3, #68]
	ldr	r2, [r3, #68]
	orr	r2, r2, #32768
	str	r2, [r3, #68]
	ldr	r2, [r0, #184]
	ldrb	r1, [r2, #16]	@ zero_extendqisi2
	cmp	r1, #2
	beq	.L59
.L55:
	ldr	r3, [r0, #184]
	ldr	r3, [r3]
	ldr	r2, [r0, #132]
	str	r2, [r3, #76]
	bx	lr
.L52:
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #52]
	b	.L53
.L59:
	ldr	r1, [r3, #32]
	movw	r3, #1365
	tst	r1, r3
	bne	.L60
	ldr	r3, [r2, #20]
	ldrh	r2, [r2, #24]
	str	r2, [r3, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #40]
	b	.L55
.L60:
	ldr	r3, [r2, #20]
	ldrh	r2, [r2, #24]
	str	r2, [r3, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #24]
	b	.L55
	.size	R3_1_F30X_SwitchOnPWM, .-R3_1_F30X_SwitchOnPWM
	.section	.text.R3_1_F30X_SwitchOffPWM,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SwitchOffPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SwitchOffPWM, %function
R3_1_F30X_SwitchOffPWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	ldr	r3, [r0, #184]
	ldr	r1, [r3, #12]
	ldr	r3, [r3]
	movs	r2, #0
	strb	r2, [r0, #88]
	ldrb	r2, [r0, #182]	@ zero_extendqisi2
	cbnz	r2, .L62
	ldr	r2, [r1, #68]
	bic	r2, r2, #1024
	str	r2, [r1, #68]
	ldr	r2, [r0, #184]
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	cmp	r4, #2
	beq	.L70
.L62:
	ldr	r2, [r1, #68]
	bic	r2, r2, #32768
	str	r2, [r1, #68]
	ldr	r2, [r3, #8]
	bic	r2, r2, #-2147483648
	bic	r2, r2, #63
	orr	r2, r2, #32
	str	r2, [r3, #8]
.L63:
	ldr	r2, [r3, #8]
	tst	r2, #8
	bne	.L63
	ldr	r2, [r3, #4]
	bic	r2, r2, #64
	str	r2, [r3, #4]
	ldr	r2, [r0, #184]
	ldrb	r0, [r2, #4]	@ zero_extendqisi2
	cmp	r0, #9
	bhi	.L64
	add	r4, r0, r0, lsl #1
	lsls	r2, r0, #26
	orr	r2, r2, r4, lsl #20
	movs	r4, #1
	lsl	r0, r4, r0
	orrs	r2, r2, r0
.L65:
	ldr	r0, [r3, #76]
	bic	r0, r0, #2105540608
	bic	r0, r0, #7831552
	bic	r0, r0, #24448
	bic	r0, r0, #127
	orr	r0, r0, #65
	and	r4, r2, #2080374784
	orrs	r0, r0, r4
	lsrs	r4, r2, #6
	and	r4, r4, #32505856
	orrs	r0, r0, r4
	lsrs	r4, r2, #12
	and	r4, r4, #507904
	orrs	r0, r0, r4
	lsrs	r2, r2, #18
	and	r2, r2, #7936
	orrs	r2, r2, r0
	str	r2, [r3, #76]
	ldr	r2, [r3, #8]
	bic	r2, r2, #-2147483648
	bic	r2, r2, #63
	orr	r2, r2, #8
	str	r2, [r3, #8]
	ldr	r2, [r1, #28]
	bic	r2, r2, #2048
	str	r2, [r1, #28]
	ldr	r2, [r1, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r1, #28]
	ldr	r2, [r1, #28]
	orr	r2, r2, #28672
	str	r2, [r1, #28]
	movw	r2, #65535
	str	r2, [r1, #64]
	movs	r2, #0
	str	r2, [r1, #64]
	ldr	r2, [r1, #28]
	orr	r2, r2, #2048
	str	r2, [r1, #28]
.L66:
	ldr	r2, [r3]
	tst	r2, #64
	beq	.L66
	movs	r2, #64
	str	r2, [r3]
	ldr	r2, [r3, #4]
	orr	r2, r2, #64
	str	r2, [r3, #4]
	ldr	r4, [sp], #4
	bx	lr
.L70:
	ldr	r4, [r2, #20]
	ldrh	r2, [r2, #24]
	str	r2, [r4, #40]
	ldr	r2, [r0, #184]
	ldr	r4, [r2, #28]
	ldrh	r2, [r2, #32]
	str	r2, [r4, #40]
	ldr	r2, [r0, #184]
	ldr	r4, [r2, #36]
	ldrh	r2, [r2, #40]
	str	r2, [r4, #40]
	b	.L62
.L64:
	movs	r2, #1
	lsls	r2, r2, r0
	orr	r2, r2, #33554432
	orr	r2, r2, r0, lsl #26
	add	r0, r0, r0, lsl #1
	subs	r0, r0, #30
	orr	r2, r2, r0, lsl #20
	b	.L65
	.size	R3_1_F30X_SwitchOffPWM, .-R3_1_F30X_SwitchOffPWM
	.section	.text.R3_1_F30X_WriteTIMRegisters,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_WriteTIMRegisters, %function
R3_1_F30X_WriteTIMRegisters:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	ldrh	r2, [r0, #80]
	str	r2, [r3, #52]
	ldrh	r2, [r0, #82]
	str	r2, [r3, #56]
	ldrh	r2, [r0, #84]
	str	r2, [r3, #60]
	ldr	r2, [r3, #64]
	uxth	r2, r2
	ldr	r1, [r3, #28]
	movw	r4, #65535
	str	r4, [r3, #64]
	orr	r1, r1, #2048
	str	r1, [r3, #28]
	str	r2, [r3, #64]
	ldr	r2, [r0, #184]
	ldr	r2, [r2]
	ldr	r1, [r0, #132]
	str	r1, [r2, #76]
	ldr	r3, [r3, #16]
	ldrh	r2, [r0, #86]
	cmp	r2, #1
	iteee	ne
	andne	r0, r3, #1
	moveq	r3, #0
	strheq	r3, [r0, #86]	@ movhi
	moveq	r0, #1
	ldr	r4, [sp], #4
	bx	lr
	.size	R3_1_F30X_WriteTIMRegisters, .-R3_1_F30X_WriteTIMRegisters
	.section	.text.R3_1_F30X_SetADCSampPointCalibration,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointCalibration
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointCalibration, %function
R3_1_F30X_SetADCSampPointCalibration:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r2, [r0, #184]
	ldr	r2, [r2, #12]
	ldrh	r3, [r0, #128]
	subs	r3, r3, #1
	str	r3, [r2, #64]
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r3, pc}
	.size	R3_1_F30X_SetADCSampPointCalibration, .-R3_1_F30X_SetADCSampPointCalibration
	.section	.text.R3_1_F30X_SetADCSampPointSect1,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect1, %function
R3_1_F30X_SetADCSampPointSect1:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	ldrh	r1, [r0, #128]
	ldrh	r3, [r0, #80]
	subs	r2, r1, r3
	uxth	r2, r2
	ldr	r4, [r0, #184]
	ldrh	r5, [r4, #8]
	cmp	r2, r5
	bls	.L78
	ldrh	r2, [r0, #82]
	subs	r2, r1, r2
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L79
	ldrh	r3, [r0, #84]
	subs	r3, r1, r3
	uxth	r3, r3
	cmp	r5, r3
	bcc	.L84
.L79:
	subs	r3, r1, #1
	uxth	r3, r3
.L81:
	ldr	r2, [r0, #168]
	str	r2, [r0, #132]
	ldr	r2, [r4, #12]
	str	r3, [r2, #64]
.L80:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r4, r5, r6, pc}
.L84:
	ldr	r2, [r4, #12]
	subs	r1, r1, #1
	str	r1, [r2, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L80
.L78:
	ldrh	r6, [r0, #82]
	subs	r6, r3, r6
	uxth	r6, r6
	cmp	r6, r2, lsl #1
	bls	.L82
	ldrh	r2, [r4, #10]
	subs	r3, r3, r2
	uxth	r3, r3
	b	.L81
.L82:
	add	r3, r3, r5
	uxth	r3, r3
	cmp	r1, r3
	bhi	.L81
	ldr	r2, [r0, #168]
	bic	r2, r2, #192
	orr	r2, r2, #128
	str	r2, [r0, #168]
	mvns	r3, r3
	add	r3, r3, r1, lsl #1
	uxth	r3, r3
	b	.L81
	.size	R3_1_F30X_SetADCSampPointSect1, .-R3_1_F30X_SetADCSampPointSect1
	.section	.text.R3_1_F30X_SetADCSampPointSect2,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect2
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect2, %function
R3_1_F30X_SetADCSampPointSect2:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	ldrh	r1, [r0, #128]
	ldrh	r4, [r0, #80]
	ldr	r5, [r0, #184]
	ldrh	r3, [r5, #8]
	subs	r2, r1, r4
	uxth	r2, r2
	cmp	r2, r3
	bls	.L86
	ldrh	r6, [r0, #82]
	subs	r2, r1, r6
	uxth	r2, r2
	cmp	r3, r2
	bcs	.L87
	ldrh	r2, [r0, #84]
	subs	r2, r1, r2
	uxth	r2, r2
	cmp	r3, r2
	bcs	.L88
	ldr	r2, [r5, #12]
	subs	r1, r1, #1
	str	r1, [r2, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L89
.L86:
	ldrh	r6, [r0, #82]
	subs	r2, r1, r6
	uxth	r2, r2
	cmp	r3, r2
	bcs	.L87
.L88:
	subs	r3, r1, #1
	uxth	r3, r3
.L90:
	ldr	r2, [r0, #160]
	str	r2, [r0, #132]
	ldr	r2, [r5, #12]
	str	r3, [r2, #64]
.L89:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r4, r5, r6, pc}
.L87:
	subs	r4, r6, r4
	uxth	r4, r4
	cmp	r4, r2, lsl #1
	bls	.L91
	ldrh	r3, [r5, #10]
	subs	r3, r6, r3
	uxth	r3, r3
	b	.L90
.L91:
	add	r3, r3, r6
	uxth	r3, r3
	cmp	r1, r3
	bhi	.L90
	ldr	r2, [r0, #160]
	bic	r2, r2, #192
	orr	r2, r2, #128
	str	r2, [r0, #160]
	mvns	r3, r3
	add	r3, r3, r1, lsl #1
	uxth	r3, r3
	b	.L90
	.size	R3_1_F30X_SetADCSampPointSect2, .-R3_1_F30X_SetADCSampPointSect2
	.section	.text.R3_1_F30X_SetADCSampPointSect3,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect3
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect3, %function
R3_1_F30X_SetADCSampPointSect3:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldrh	r4, [r0, #128]
	ldr	r1, [r0, #184]
	ldrh	r5, [r1, #8]
	ldrh	r2, [r0, #80]
	subs	r2, r4, r2
	uxth	r2, r2
	cmp	r2, r5
	bls	.L94
	ldrh	r3, [r0, #82]
	subs	r2, r4, r3
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L95
	ldrh	r3, [r0, #84]
	subs	r3, r4, r3
	uxth	r3, r3
	cmp	r5, r3
	bcc	.L104
	ldr	r6, [r0, #164]
	b	.L101
.L104:
	ldr	r2, [r1, #12]
	subs	r4, r4, #1
	str	r4, [r2, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L97
.L94:
	ldr	r6, [r0, #164]
	ldrh	r3, [r0, #82]
	subs	r2, r4, r3
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L98
.L101:
	subs	r3, r4, #1
	uxth	r3, r3
.L99:
	str	r6, [r0, #132]
	ldr	r2, [r1, #12]
	str	r3, [r2, #64]
.L97:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L100:
	add	r3, r3, r5
	uxth	r3, r3
	cmp	r4, r3
	bhi	.L99
	bic	r6, r6, #192
	orr	r6, r6, #128
	mvns	r3, r3
	add	r3, r3, r4, lsl #1
	uxth	r3, r3
	b	.L99
.L95:
	ldr	r6, [r0, #164]
.L98:
	ldrh	r7, [r0, #84]
	subs	r7, r3, r7
	uxth	r7, r7
	cmp	r7, r2, lsl #1
	bls	.L100
	ldrh	r2, [r1, #10]
	subs	r3, r3, r2
	uxth	r3, r3
	b	.L99
	.size	R3_1_F30X_SetADCSampPointSect3, .-R3_1_F30X_SetADCSampPointSect3
	.section	.text.R3_1_F30X_SetADCSampPointSect4,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect4
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect4, %function
R3_1_F30X_SetADCSampPointSect4:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldrh	r1, [r0, #128]
	ldr	r4, [r0, #184]
	ldrh	r5, [r4, #8]
	ldrh	r2, [r0, #80]
	subs	r2, r1, r2
	uxth	r2, r2
	cmp	r2, r5
	bls	.L106
	ldrh	r3, [r0, #82]
	subs	r3, r1, r3
	uxth	r3, r3
	cmp	r5, r3
	bcs	.L106
	ldrh	r3, [r0, #84]
	subs	r2, r1, r3
	uxth	r2, r2
	cmp	r5, r2
	bcc	.L114
	ldr	r6, [r0, #156]
.L109:
	ldrh	r7, [r0, #82]
	subs	r7, r3, r7
	uxth	r7, r7
	cmp	r7, r2, lsl #1
	bls	.L111
	ldrh	r2, [r4, #10]
	subs	r3, r3, r2
	uxth	r3, r3
	b	.L110
.L114:
	ldr	r2, [r4, #12]
	subs	r1, r1, #1
	str	r1, [r2, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L108
.L106:
	ldr	r6, [r0, #156]
	ldrh	r3, [r0, #84]
	subs	r2, r1, r3
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L109
	subs	r3, r1, #1
	uxth	r3, r3
.L110:
	str	r6, [r0, #132]
	ldr	r2, [r4, #12]
	str	r3, [r2, #64]
.L108:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L111:
	add	r3, r3, r5
	uxth	r3, r3
	cmp	r1, r3
	bhi	.L110
	bic	r6, r6, #192
	orr	r6, r6, #128
	mvns	r3, r3
	add	r3, r3, r1, lsl #1
	uxth	r3, r3
	b	.L110
	.size	R3_1_F30X_SetADCSampPointSect4, .-R3_1_F30X_SetADCSampPointSect4
	.section	.text.R3_1_F30X_SetADCSampPointSect5,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect5
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect5, %function
R3_1_F30X_SetADCSampPointSect5:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldrh	r2, [r0, #128]
	ldrh	r4, [r0, #80]
	ldr	r5, [r0, #184]
	ldrh	r7, [r5, #8]
	subs	r1, r2, r4
	uxth	r1, r1
	cmp	r1, r7
	bls	.L116
	ldrh	r3, [r0, #82]
	subs	r3, r2, r3
	uxth	r3, r3
	cmp	r7, r3
	bcs	.L116
	ldrh	r3, [r0, #84]
	subs	r1, r2, r3
	uxth	r1, r1
	cmp	r7, r1
	bcc	.L124
	ldr	r6, [r0, #152]
.L119:
	subs	r4, r3, r4
	uxth	r4, r4
	cmp	r4, r1, lsl #1
	bls	.L121
	ldrh	r2, [r5, #10]
	subs	r3, r3, r2
	uxth	r3, r3
	b	.L120
.L124:
	ldr	r1, [r5, #12]
	subs	r2, r2, #1
	str	r2, [r1, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L118
.L116:
	ldr	r6, [r0, #152]
	ldrh	r3, [r0, #84]
	subs	r1, r2, r3
	uxth	r1, r1
	cmp	r7, r1
	bcs	.L119
	subs	r3, r2, #1
	uxth	r3, r3
.L120:
	str	r6, [r0, #132]
	ldr	r2, [r5, #12]
	str	r3, [r2, #64]
.L118:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L121:
	add	r3, r3, r7
	uxth	r3, r3
	cmp	r2, r3
	bhi	.L120
	bic	r6, r6, #192
	orr	r6, r6, #128
	mvns	r3, r3
	add	r3, r3, r2, lsl #1
	uxth	r3, r3
	b	.L120
	.size	R3_1_F30X_SetADCSampPointSect5, .-R3_1_F30X_SetADCSampPointSect5
	.section	.text.R3_1_F30X_SetADCSampPointSect6,"ax",%progbits
	.align	1
	.global	R3_1_F30X_SetADCSampPointSect6
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_SetADCSampPointSect6, %function
R3_1_F30X_SetADCSampPointSect6:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldrh	r1, [r0, #128]
	ldrh	r3, [r0, #80]
	subs	r2, r1, r3
	uxth	r2, r2
	ldr	r4, [r0, #184]
	ldrh	r7, [r4, #8]
	cmp	r2, r7
	bls	.L126
	ldrh	r2, [r0, #82]
	subs	r2, r1, r2
	uxth	r2, r2
	cmp	r7, r2
	bcs	.L127
	ldrh	r3, [r0, #84]
	subs	r3, r1, r3
	uxth	r3, r3
	cmp	r7, r3
	bcc	.L135
.L127:
	ldr	r6, [r0, #172]
	subs	r3, r1, #1
	uxth	r3, r3
.L129:
	str	r6, [r0, #132]
	ldr	r2, [r4, #12]
	str	r3, [r2, #64]
.L128:
	bl	R3_1_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L135:
	ldr	r2, [r4, #12]
	subs	r1, r1, #1
	str	r1, [r2, #64]
	movs	r3, #4
	strh	r3, [r0, #78]	@ movhi
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	b	.L128
.L130:
	ldrh	r2, [r4, #10]
	subs	r3, r3, r2
	uxth	r3, r3
	b	.L129
.L126:
	ldr	r6, [r0, #172]
	ldrh	r5, [r0, #84]
	subs	r5, r3, r5
	uxth	r5, r5
	cmp	r5, r2, lsl #1
	bhi	.L130
	add	r3, r3, r7
	uxth	r3, r3
	cmp	r1, r3
	bhi	.L129
	bic	r6, r6, #192
	orr	r6, r6, #128
	mvns	r3, r3
	add	r3, r3, r1, lsl #1
	uxth	r3, r3
	b	.L129
	.size	R3_1_F30X_SetADCSampPointSect6, .-R3_1_F30X_SetADCSampPointSect6
	.section	.text.R3_1_F30X_RLGetPhaseCurrents,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLGetPhaseCurrents
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLGetPhaseCurrents, %function
R3_1_F30X_RLGetPhaseCurrents:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	mvn	r2, #1
	str	r2, [r3, #16]
	ldr	r2, [r0, #120]
	ldr	r3, [r0, #184]
	ldr	r0, [r3]
	ldr	r3, [r0, #128]
	subs	r3, r2, r3
	ldr	r4, .L142
	cmp	r3, r4
	blt	.L139
	movw	r4, #32767
	cmp	r3, r4
	it	ge
	movge	r3, r4
.L137:
	sxth	r3, r3
	ldr	r0, [r0, #132]
	subs	r2, r2, r0
	ldr	r0, .L142
	cmp	r2, r0
	blt	.L140
	movw	r0, #32767
	cmp	r2, r0
	it	ge
	movge	r2, r0
.L138:
	strh	r3, [r1]	@ movhi
	strh	r2, [r1, #2]	@ movhi
	ldr	r4, [sp], #4
	bx	lr
.L139:
	ldr	r3, .L142+4
	b	.L137
.L140:
	ldr	r2, .L142+4
	b	.L138
.L143:
	.align	2
.L142:
	.word	-32766
	.word	-32767
	.size	R3_1_F30X_RLGetPhaseCurrents, .-R3_1_F30X_RLGetPhaseCurrents
	.section	.text.R3_1_F30X_RLTurnOnLowSides,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLTurnOnLowSides
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLTurnOnLowSides, %function
R3_1_F30X_RLTurnOnLowSides:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #12]
	movs	r3, #0
	str	r3, [r2, #52]
	mvn	r3, #1
	str	r3, [r2, #16]
.L145:
	ldr	r3, [r2, #16]
	tst	r3, #1
	beq	.L145
	ldr	r3, [r2, #68]
	orr	r3, r3, #32768
	str	r3, [r2, #68]
	ldr	r3, [r0, #184]
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L148
.L144:
	bx	lr
.L148:
	ldr	r2, [r3, #20]
	ldrh	r3, [r3, #24]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #24]
	b	.L144
	.size	R3_1_F30X_RLTurnOnLowSides, .-R3_1_F30X_RLTurnOnLowSides
	.section	.text.R3_1_F30X_RLSwitchOnPWM,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLSwitchOnPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLSwitchOnPWM, %function
R3_1_F30X_RLSwitchOnPWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	movs	r2, #0
	strb	r2, [r0, #88]
.L150:
	ldr	r2, [r3]
	tst	r2, #16
	bne	.L150
.L151:
	ldr	r2, [r3]
	tst	r2, #16
	beq	.L151
	movs	r2, #0
	str	r2, [r3, #48]
	movs	r2, #1
	str	r2, [r3, #52]
	ldr	r2, [r0, #184]
	ldr	r2, [r2]
	ldr	r1, [r0, #144]
	str	r1, [r2, #76]
	mvn	r2, #1
	str	r2, [r3, #16]
.L152:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L152
	ldr	r2, [r3, #68]
	orr	r2, r2, #1024
	str	r2, [r3, #68]
	ldr	r2, [r3, #68]
	orr	r2, r2, #32768
	str	r2, [r3, #68]
	ldr	r2, [r0, #184]
	ldrb	r1, [r2, #16]	@ zero_extendqisi2
	cmp	r1, #2
	beq	.L158
.L149:
	bx	lr
.L158:
	ldr	r1, [r3, #32]
	movw	r3, #1365
	tst	r1, r3
	bne	.L159
	ldr	r3, [r2, #20]
	ldrh	r2, [r2, #24]
	str	r2, [r3, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #40]
	b	.L149
.L159:
	ldr	r3, [r2, #20]
	ldrh	r2, [r2, #24]
	str	r2, [r3, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #24]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #24]
	bx	lr
	.size	R3_1_F30X_RLSwitchOnPWM, .-R3_1_F30X_RLSwitchOnPWM
	.section	.text.SingleADC_InjectedConfig,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	SingleADC_InjectedConfig, %function
SingleADC_InjectedConfig:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #76]
	ldr	r3, [r1]
	ldr	r2, [r1, #4]
	orrs	r3, r3, r2
	ldr	r0, [r1, #12]
	orr	r3, r3, r0, lsl #8
	ldr	r0, [r1, #16]
	orr	r3, r3, r0, lsl #14
	ldr	r0, [r1, #20]
	orr	r3, r3, r0, lsl #20
	ldr	r0, [r1, #24]
	orr	r3, r3, r0, lsl #26
	ldrb	r0, [r1, #8]	@ zero_extendqisi2
	subs	r0, r0, #1
	orrs	r0, r0, r3
	bx	lr
	.size	SingleADC_InjectedConfig, .-SingleADC_InjectedConfig
	.section	.text.R3_1_F30X_RLSwitchOffPWM,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLSwitchOffPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLSwitchOffPWM, %function
R3_1_F30X_RLSwitchOffPWM:
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	sub	sp, sp, #36
	mov	r6, r0
	ldr	r3, [r0, #184]
	ldr	r4, [r3, #12]
	ldr	r5, [r3]
	ldrb	r3, [r0, #182]	@ zero_extendqisi2
	cbnz	r3, .L162
	ldr	r3, [r4, #68]
	bic	r3, r3, #1024
	str	r3, [r4, #68]
	ldr	r3, [r0, #184]
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L166
.L162:
	ldr	r3, [r4, #68]
	bic	r3, r3, #32768
	str	r3, [r4, #68]
	ldr	r3, [r6, #184]
	ldr	r2, [r3]
	ldr	r3, [r2, #4]
	bic	r3, r3, #64
	str	r3, [r2, #4]
	ldr	r3, [r5, #8]
	orr	r3, r3, #32
	str	r3, [r5, #8]
	movs	r3, #2
	strb	r3, [sp, #12]
	movs	r7, #0
	str	r7, [sp, #16]
	movs	r3, #64
	str	r3, [sp, #8]
	add	r1, sp, #32
	str	r3, [r1, #-28]!
	mov	r0, r5
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #76]
	ldr	r3, [r5, #8]
	orr	r3, r3, #8
	str	r3, [r5, #8]
	ldr	r3, [r4, #28]
	bic	r3, r3, #2048
	str	r3, [r4, #28]
	ldr	r3, [r4, #28]
	bic	r3, r3, #28672
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r4, #28]
	ldr	r3, [r4, #28]
	orr	r3, r3, #28672
	str	r3, [r4, #28]
	movw	r3, #65535
	str	r3, [r4, #64]
	str	r7, [r4, #64]
	ldr	r3, [r4, #28]
	orr	r3, r3, #2048
	str	r3, [r4, #28]
.L163:
	ldr	r3, [r5]
	tst	r3, #64
	beq	.L163
	ldr	r3, [r6, #184]
	ldr	r3, [r3]
	movs	r2, #64
	str	r2, [r3]
	ldr	r3, [r6, #184]
	ldr	r2, [r3]
	ldr	r3, [r2, #4]
	orr	r3, r3, #64
	str	r3, [r2, #4]
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L166:
	ldr	r2, [r3, #20]
	ldrh	r3, [r3, #24]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #40]
	b	.L162
	.size	R3_1_F30X_RLSwitchOffPWM, .-R3_1_F30X_RLSwitchOffPWM
	.section	.text.R3_1_F30X_Init,"ax",%progbits
	.align	1
	.global	R3_1_F30X_Init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_Init, %function
R3_1_F30X_Init:
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	sub	sp, sp, #40
	mov	r5, r0
	ldr	r3, [r0, #184]
	ldr	r1, [r3, #44]
	ldr	r7, [r3, #52]
	ldr	r0, [r3, #60]
	ldr	r2, [r3, #68]
	ldr	r6, [r3, #12]
	ldr	r4, [r3]
	ldr	r3, [r4, #4]
	bic	r3, r3, #4
	str	r3, [r4, #4]
	mov	lr, #4
	str	lr, [r4]
	ldr	r3, [r4, #4]
	bic	r3, r3, #32
	str	r3, [r4, #4]
	mov	ip, #32
	str	ip, [r4]
	ldr	r3, [r5, #184]
	ldr	r8, [r3, #80]
	ldr	r3, [r8, #4]
	bic	r3, r3, #4
	str	r3, [r8, #4]
	ldr	r3, [r5, #184]
	ldr	r3, [r3, #80]
	str	lr, [r3]
	ldr	r3, [r5, #184]
	ldr	lr, [r3, #80]
	ldr	r3, [lr, #4]
	bic	r3, r3, #32
	str	r3, [lr, #4]
	ldr	r3, [r5, #184]
	ldr	r3, [r3, #80]
	str	ip, [r3]
	ldr	r3, [r6]
	bic	r3, r3, #1
	str	r3, [r6]
	cbz	r1, .L169
	ldr	ip, [r5, #184]
	ldrb	r3, [ip, #48]	@ zero_extendqisi2
	cmp	r3, #1
	beq	.L170
	ldr	r3, [r1]
	and	r3, r3, #112
	cmp	r3, #64
	beq	.L192
	ldr	r3, [r1]
	and	r3, r3, #112
	cmp	r3, #80
	beq	.L193
.L170:
	movs	r3, #0
	strh	r3, [sp, #6]	@ movhi
	strh	r3, [sp, #6]	@ movhi
	ldrh	r3, [sp, #6]
	uxth	r3, r3
	cmp	r3, #1000
	bcs	.L172
.L173:
	ldrh	r3, [sp, #6]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [sp, #6]	@ movhi
	ldrh	r3, [sp, #6]
	uxth	r3, r3
	cmp	r3, #1000
	bcc	.L173
.L172:
	ldr	r3, [r1]
	orr	r3, r3, #1
	str	r3, [r1]
	ldr	r3, [r1]
	orr	r3, r3, #-2147483648
	str	r3, [r1]
.L169:
	cbz	r7, .L174
	ldr	r3, [r7]
	orr	r3, r3, #1
	str	r3, [r7]
	ldr	r3, [r7]
	orr	r3, r3, #-2147483648
	str	r3, [r7]
.L174:
	cbz	r0, .L175
	ldr	r3, [r0]
	orr	r3, r3, #1
	str	r3, [r0]
	ldr	r3, [r0]
	orr	r3, r3, #-2147483648
	str	r3, [r0]
.L175:
	cbz	r2, .L176
	ldr	r1, [r5, #184]
	ldrb	r3, [r1, #72]	@ zero_extendqisi2
	cmp	r3, #1
	beq	.L177
	ldr	r3, [r2]
	and	r3, r3, #112
	cmp	r3, #64
	beq	.L194
	ldr	r3, [r2]
	and	r3, r3, #112
	cmp	r3, #80
	beq	.L195
.L177:
	movs	r3, #0
	strh	r3, [sp, #8]	@ movhi
	strh	r3, [sp, #8]	@ movhi
	ldrh	r3, [sp, #8]
	uxth	r3, r3
	cmp	r3, #1000
	bcs	.L179
.L180:
	ldrh	r3, [sp, #8]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [sp, #8]	@ movhi
	ldrh	r3, [sp, #8]
	uxth	r3, r3
	cmp	r3, #1000
	bcc	.L180
.L179:
	ldr	r3, [r2]
	orr	r3, r3, #1
	str	r3, [r2]
	ldr	r3, [r2]
	orr	r3, r3, #-2147483648
	str	r3, [r2]
.L176:
	mvn	r3, #128
	str	r3, [r6, #16]
	ldr	r3, [r5, #184]
	ldrb	r3, [r3, #42]	@ zero_extendqisi2
	cbz	r3, .L181
	mvn	r3, #256
	str	r3, [r6, #16]
.L181:
	ldr	r3, [r6, #12]
	orr	r3, r3, #128
	str	r3, [r6, #12]
	ldr	r3, [r4, #8]
	bic	r3, r3, #805306368
	str	r3, [r4, #8]
	ldr	r3, [r4, #8]
	bic	r3, r3, #-1879048192
	bic	r3, r3, #63
	orr	r3, r3, #268435456
	str	r3, [r4, #8]
	movs	r3, #0
	strh	r3, [sp, #10]	@ movhi
	strh	r3, [sp, #10]	@ movhi
	ldrh	r3, [sp, #10]
	uxth	r3, r3
	movw	r2, #64999
	cmp	r3, r2
	bhi	.L182
.L183:
	ldrh	r3, [sp, #10]
	uxth	r3, r3
	strh	r3, [sp, #10]	@ movhi
	ldrh	r3, [sp, #10]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [sp, #10]	@ movhi
	ldrh	r3, [sp, #10]
	uxth	r3, r3
	cmp	r3, r2
	bls	.L183
.L182:
	ldr	r3, [r4, #8]
	bic	r3, r3, #-1073741824
	bic	r3, r3, #63
	orr	r3, r3, #-2147483648
	str	r3, [r4, #8]
.L184:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	blt	.L184
	ldr	r3, [r4, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #1
	str	r3, [r4, #8]
	ldr	r3, [r4, #48]
	bic	r3, r3, #15
	str	r3, [r4, #48]
	movs	r3, #64
	str	r3, [sp, #12]
	str	r3, [sp, #16]
	mov	r8, #2
	strb	r8, [sp, #20]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #4]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #5]	@ zero_extendqisi2
	str	r3, [sp, #28]
	movs	r7, #0
	str	r7, [sp, #32]
	str	r7, [sp, #36]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #152]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #5]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	str	r3, [sp, #28]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #156]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #4]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #6]	@ zero_extendqisi2
	str	r3, [sp, #28]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #160]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #6]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	str	r3, [sp, #28]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #164]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #5]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #6]	@ zero_extendqisi2
	str	r3, [sp, #28]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #168]
	ldr	r3, [r5, #184]
	ldrb	r2, [r3, #6]	@ zero_extendqisi2
	str	r2, [sp, #24]
	ldrb	r3, [r3, #5]	@ zero_extendqisi2
	str	r3, [sp, #28]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #172]
	movs	r3, #1
	strb	r3, [sp, #20]
	ldr	r3, [r5, #184]
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	str	r3, [sp, #24]
	str	r7, [sp, #28]
	str	r7, [sp, #32]
	str	r7, [sp, #36]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #136]
	ldr	r3, [r5, #184]
	ldrb	r3, [r3, #5]	@ zero_extendqisi2
	str	r3, [sp, #24]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #140]
	ldr	r3, [r5, #184]
	ldrb	r3, [r3, #6]	@ zero_extendqisi2
	str	r3, [sp, #24]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r5, #148]
	strb	r8, [sp, #20]
	str	r7, [sp, #24]
	add	r1, sp, #12
	mov	r0, r4
	bl	SingleADC_InjectedConfig
	str	r0, [r4, #76]
	ldr	r3, [r4, #8]
	orr	r3, r3, #8
	str	r3, [r4, #8]
	ldr	r3, [r6, #28]
	movw	r2, #65535
	str	r2, [r6, #64]
	str	r7, [r6, #64]
	orr	r3, r3, #2048
	str	r3, [r6, #28]
.L185:
	ldr	r3, [r4]
	tst	r3, #64
	beq	.L185
	movs	r3, #64
	str	r3, [r4]
	ldr	r3, [r4, #4]
	orr	r3, r3, #64
	str	r3, [r4, #4]
	movs	r3, #0
	strb	r3, [r5, #181]
	strb	r3, [r5, #180]
	strh	r3, [r5, #100]	@ movhi
	ldrh	r3, [r5, #108]
	strh	r3, [r5, #102]	@ movhi
	add	sp, sp, #40
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L192:
	mov	r3, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r3, r3
	mov	lr, #1048576
	lsr	lr, lr, r3
	ldr	r8, .L196+4
	ldr	r3, [r8, lr, lsl #2]
	bic	r3, r3, #65280
	bic	r3, r3, #240
	ldrh	ip, [ip, #74]
	orr	r3, r3, ip
	str	r3, [r8, lr, lsl #2]
	ldr	r3, .L196
	ldr	ip, [r3, #4]
	orr	ip, ip, #1
	str	ip, [r3, #4]
	ldr	ip, [r3]
	orr	ip, ip, #1
	str	ip, [r3]
	b	.L170
.L193:
	mov	r3, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r3, r3
	mov	lr, #4194304
	lsr	lr, lr, r3
	ldr	r8, .L196+4
	ldr	r3, [r8, lr, lsl #2]
	bic	r3, r3, #65280
	bic	r3, r3, #240
	ldrh	ip, [ip, #74]
	orr	r3, r3, ip
	str	r3, [r8, lr, lsl #2]
	ldr	r3, .L196
	ldr	ip, [r3, #4]
	orr	ip, ip, #2
	str	ip, [r3, #4]
	ldr	ip, [r3]
	orr	ip, ip, #65536
	str	ip, [r3]
	b	.L170
.L194:
	mov	r3, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r3, r3
	mov	r0, #1048576
	lsrs	r0, r0, r3
	ldr	r7, .L196+4
	ldr	r3, [r7, r0, lsl #2]
	bic	r3, r3, #65280
	bic	r3, r3, #240
	ldrh	r1, [r1, #76]
	orrs	r3, r3, r1
	str	r3, [r7, r0, lsl #2]
	ldr	r3, .L196
	ldr	r1, [r3, #4]
	orr	r1, r1, #1
	str	r1, [r3, #4]
	ldr	r1, [r3]
	orr	r1, r1, #1
	str	r1, [r3]
	b	.L177
.L195:
	mov	r3, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r3, r3
	mov	r0, #4194304
	lsrs	r0, r0, r3
	ldr	r7, .L196+4
	ldr	r3, [r7, r0, lsl #2]
	bic	r3, r3, #65280
	bic	r3, r3, #240
	ldrh	r1, [r1, #76]
	orrs	r3, r3, r1
	str	r3, [r7, r0, lsl #2]
	ldr	r3, .L196
	ldr	r1, [r3, #4]
	orr	r1, r1, #2
	str	r1, [r3, #4]
	ldr	r1, [r3]
	orr	r1, r1, #65536
	str	r1, [r3]
	b	.L177
.L197:
	.align	2
.L196:
	.word	1073771520
	.word	1073771528
	.size	R3_1_F30X_Init, .-R3_1_F30X_Init
	.section	.text.R3_1_F30X_CurrentReadingCalibration,"ax",%progbits
	.align	1
	.global	R3_1_F30X_CurrentReadingCalibration
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_CurrentReadingCalibration, %function
R3_1_F30X_CurrentReadingCalibration:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldr	r3, [r0, #184]
	ldr	r5, [r3, #12]
	movs	r7, #0
	str	r7, [r0, #116]
	str	r7, [r0, #120]
	str	r7, [r0, #124]
	strb	r7, [r0, #176]
	ldr	r3, [r5, #32]
	bic	r3, r3, #1360
	bic	r3, r3, #5
	str	r3, [r5, #32]
	ldr	r3, .L210
	str	r3, [r0, #4]
	ldr	r3, .L210+4
	str	r3, [r0, #28]
	str	r3, [r0, #32]
	str	r3, [r0, #36]
	str	r3, [r0, #40]
	str	r3, [r0, #44]
	str	r3, [r0, #48]
	ldr	r3, [r0, #152]
	str	r3, [r0, #132]
	bl	R3_1_F30X_SwitchOnPWM
	ldr	r3, [r4, #184]
	ldrb	r6, [r3, #7]	@ zero_extendqisi2
	adds	r6, r6, #1
	lsrs	r6, r6, #1
	add	r6, r6, r6, lsl #4
	movw	r3, #65533
	str	r3, [r5, #16]
	mov	r2, r3
.L200:
	ldrb	r3, [r4, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L201
	ldr	r3, [r5, #16]
	tst	r3, #2
	beq	.L200
	str	r2, [r5, #16]
	adds	r7, r7, #1
	uxth	r7, r7
	cmp	r6, r7
	bhi	.L200
	ldrb	r3, [r4, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L200
	movs	r3, #1
	strh	r3, [r4, #86]	@ movhi
.L201:
	mov	r0, r4
	bl	R3_1_F30X_SwitchOffPWM
	movs	r7, #0
	strb	r7, [r4, #176]
	ldr	r3, .L210+8
	str	r3, [r4, #4]
	ldr	r3, [r4, #148]
	str	r3, [r4, #132]
	mov	r0, r4
	bl	R3_1_F30X_SwitchOnPWM
	movw	r3, #65533
	str	r3, [r5, #16]
	mov	r2, r7
	mov	r1, r3
.L204:
	ldrb	r3, [r4, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L205
	ldr	r3, [r5, #16]
	tst	r3, #2
	beq	.L204
	str	r1, [r5, #16]
	adds	r2, r2, #1
	uxth	r2, r2
	cmp	r6, r2
	bhi	.L204
	ldrb	r3, [r4, #176]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L204
	movs	r3, #1
	strh	r3, [r4, #86]	@ movhi
.L205:
	mov	r0, r4
	bl	R3_1_F30X_SwitchOffPWM
	ldr	r3, [r4, #116]
	lsrs	r3, r3, #4
	str	r3, [r4, #116]
	ldr	r3, [r4, #120]
	lsrs	r3, r3, #4
	str	r3, [r4, #120]
	ldr	r3, [r4, #124]
	lsrs	r3, r3, #4
	str	r3, [r4, #124]
	ldr	r3, .L210+12
	str	r3, [r4, #4]
	ldr	r3, .L210+16
	str	r3, [r4, #28]
	ldr	r3, .L210+20
	str	r3, [r4, #32]
	ldr	r3, .L210+24
	str	r3, [r4, #36]
	ldr	r3, .L210+28
	str	r3, [r4, #40]
	ldr	r3, .L210+32
	str	r3, [r4, #44]
	ldr	r3, .L210+36
	str	r3, [r4, #48]
	ldr	r3, [r4, #152]
	str	r3, [r4, #132]
	ldr	r2, [r5, #24]
	movw	r1, #63479
	ands	r2, r2, r1
	str	r2, [r5, #24]
	ldr	r3, [r5, #28]
	ands	r3, r3, r1
	str	r3, [r5, #28]
	ldrh	r3, [r4, #128]
	str	r3, [r5, #52]
	ldrh	r3, [r4, #128]
	str	r3, [r5, #56]
	ldrh	r3, [r4, #128]
	str	r3, [r5, #60]
	ldr	r2, [r5, #24]
	movw	r1, #2056
	orrs	r2, r2, r1
	str	r2, [r5, #24]
	ldr	r3, [r5, #28]
	orrs	r3, r3, r1
	str	r3, [r5, #28]
	ldr	r3, [r5, #32]
	orr	r3, r3, #1360
	orr	r3, r3, #5
	str	r3, [r5, #32]
	movs	r3, #0
	strb	r3, [r4, #182]
	pop	{r3, r4, r5, r6, r7, pc}
.L211:
	.align	2
.L210:
	.word	R3_1_F30X_HFCurrentsCalibrationAB
	.word	R3_1_F30X_SetADCSampPointCalibration
	.word	R3_1_F30X_HFCurrentsCalibrationC
	.word	R3_1_F30X_GetPhaseCurrents
	.word	R3_1_F30X_SetADCSampPointSect1
	.word	R3_1_F30X_SetADCSampPointSect2
	.word	R3_1_F30X_SetADCSampPointSect3
	.word	R3_1_F30X_SetADCSampPointSect4
	.word	R3_1_F30X_SetADCSampPointSect5
	.word	R3_1_F30X_SetADCSampPointSect6
	.size	R3_1_F30X_CurrentReadingCalibration, .-R3_1_F30X_CurrentReadingCalibration
	.section	.text.R3_1_F30X_TIMx_UP_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_1_F30X_TIMx_UP_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_TIMx_UP_IRQHandler, %function
R3_1_F30X_TIMx_UP_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	adds	r0, r0, #92
	bx	lr
	.size	R3_1_F30X_TIMx_UP_IRQHandler, .-R3_1_F30X_TIMx_UP_IRQHandler
	.section	.text.R3_1_F30X_BRK2_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_1_F30X_BRK2_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_BRK2_IRQHandler, %function
R3_1_F30X_BRK2_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrb	r3, [r0, #182]	@ zero_extendqisi2
	cbnz	r3, .L214
	ldr	r3, [r0, #184]
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L215
.L214:
	movs	r3, #1
	strb	r3, [r0, #180]
	adds	r0, r0, #92
	bx	lr
.L215:
	ldr	r2, [r3, #20]
	ldrh	r3, [r3, #24]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #28]
	ldrh	r3, [r3, #32]
	str	r3, [r2, #40]
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #36]
	ldrh	r3, [r3, #40]
	str	r3, [r2, #40]
	b	.L214
	.size	R3_1_F30X_BRK2_IRQHandler, .-R3_1_F30X_BRK2_IRQHandler
	.section	.text.R3_1_F30X_BRK_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_1_F30X_BRK_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_BRK_IRQHandler, %function
R3_1_F30X_BRK_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #184]
	ldr	r2, [r3, #12]
	ldr	r3, [r2, #68]
	orr	r3, r3, #1024
	str	r3, [r2, #68]
	movs	r3, #1
	strb	r3, [r0, #181]
	strb	r3, [r0, #182]
	adds	r0, r0, #92
	bx	lr
	.size	R3_1_F30X_BRK_IRQHandler, .-R3_1_F30X_BRK_IRQHandler
	.section	.text.R3_1_F30X_ExecRegularConv,"ax",%progbits
	.align	1
	.global	R3_1_F30X_ExecRegularConv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_ExecRegularConv, %function
R3_1_F30X_ExecRegularConv:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	mov	r4, r0
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #80]
	cmp	r1, #9
	bhi	.L218
	add	r0, r1, r1, lsl #1
	lsls	r2, r1, #26
	orr	r2, r2, r0, lsl #20
	movs	r0, #1
	lsl	r1, r0, r1
	orrs	r2, r2, r1
.L219:
	mov	r1, #768
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r1, r1
@ 0 "" 2
	.thumb
	.syntax unified
	ldr	r1, [r3, #48]
	bic	r1, r1, #1984
	lsrs	r2, r2, #20
	and	r2, r2, #1984
	orrs	r2, r2, r1
	str	r2, [r3, #48]
	ldr	r2, [r3, #64]
	ldr	r2, [r3, #8]
	bic	r2, r2, #-2147483648
	bic	r2, r2, #63
	orr	r2, r2, #4
	str	r2, [r3, #8]
.L220:
	ldr	r2, [r3]
	tst	r2, #4
	beq	.L220
	ldr	r0, [r3, #64]
	uxth	r0, r0
	strh	r0, [r4, #130]	@ movhi
	ldr	r4, [sp], #4
	bx	lr
.L218:
	movs	r2, #1
	lsls	r2, r2, r1
	orr	r2, r2, #33554432
	orr	r2, r2, r1, lsl #26
	add	r1, r1, r1, lsl #1
	subs	r1, r1, #30
	orr	r2, r2, r1, lsl #20
	b	.L219
	.size	R3_1_F30X_ExecRegularConv, .-R3_1_F30X_ExecRegularConv
	.section	.text.R3_1_F30X_ADC_SetSamplingTime,"ax",%progbits
	.align	1
	.global	R3_1_F30X_ADC_SetSamplingTime
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_ADC_SetSamplingTime, %function
R3_1_F30X_ADC_SetSamplingTime:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4, r5, r6}
	sub	sp, sp, #12
	strh	r1, [sp, #4]	@ movhi
	uxtb	r3, r1
	ubfx	r1, r1, #8, #8
	cmp	r3, #9
	bhi	.L227
	add	r3, r3, r3, lsl #1
	ldr	r2, [r0, #184]
	ldr	r5, [r2, #80]
	ldr	r2, [r5, #20]
	subs	r6, r3, #3
	movs	r4, #56
	lsls	r4, r4, r6
	bic	r2, r2, r4
	str	r2, [r5, #20]
	ldr	r2, [r0, #184]
	ldr	r0, [r2, #80]
	ldr	r2, [r0, #20]
	lsl	r3, r1, r3
	orrs	r3, r3, r2
	str	r3, [r0, #20]
.L223:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6}
	bx	lr
.L227:
	add	r3, r3, r3, lsl #1
	sub	r2, r3, #30
	ldr	r3, [r0, #184]
	ldr	r5, [r3, #80]
	ldr	r3, [r5, #24]
	movs	r4, #7
	lsls	r4, r4, r2
	bic	r3, r3, r4
	str	r3, [r5, #24]
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #80]
	ldr	r0, [r3, #24]
	lsl	r2, r1, r2
	orrs	r2, r2, r0
	str	r2, [r3, #24]
	b	.L223
	.size	R3_1_F30X_ADC_SetSamplingTime, .-R3_1_F30X_ADC_SetSamplingTime
	.section	.text.R3_1_F30X_IsOverCurrentOccurred,"ax",%progbits
	.align	1
	.global	R3_1_F30X_IsOverCurrentOccurred
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_IsOverCurrentOccurred, %function
R3_1_F30X_IsOverCurrentOccurred:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, r0
	ldrb	r2, [r0, #181]	@ zero_extendqisi2
	cbz	r2, .L231
	movs	r2, #0
	strb	r2, [r0, #181]
	movs	r0, #2
.L229:
	ldrb	r2, [r3, #180]	@ zero_extendqisi2
	cbz	r2, .L230
	orr	r0, r0, #64
	movs	r2, #0
	strb	r2, [r3, #180]
.L230:
	bx	lr
.L231:
	movs	r0, #0
	b	.L229
	.size	R3_1_F30X_IsOverCurrentOccurred, .-R3_1_F30X_IsOverCurrentOccurred
	.section	.text.R3_1_F30X_RLDetectionModeEnable,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLDetectionModeEnable
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLDetectionModeEnable, %function
R3_1_F30X_RLDetectionModeEnable:
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	sub	sp, sp, #36
	mov	r4, r0
	ldrb	r3, [r0, #93]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L233
	ldr	r2, [r0, #184]
	ldr	r3, [r2, #12]
	ldr	r0, [r2]
	add	r1, r3, #24
	ldr	r5, [r3, #24]
	bic	r2, r5, #65536
	bic	r2, r2, #115
	orr	r2, r2, #96
	str	r2, [r3, #24]
	ldr	r2, [r3, #32]
	orr	r2, r2, #1
	str	r2, [r3, #32]
	ldr	r2, [r3, #32]
	bic	r2, r2, #4
	str	r2, [r3, #32]
	movs	r2, #0
	str	r2, [r3, #52]
	ldr	r2, [r4, #184]
	ldrb	r2, [r2, #16]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L237
	cmp	r2, #2
	beq	.L238
.L235:
	ldr	r2, [r1, #4]
	bic	r2, r2, #65536
	bic	r2, r2, #115
	orr	r2, r2, #112
	str	r2, [r1, #4]
	ldr	r2, [r3, #32]
	bic	r2, r2, #256
	str	r2, [r3, #32]
	ldr	r2, [r3, #32]
	bic	r2, r2, #1024
	str	r2, [r3, #32]
	ldr	r2, [r3, #4]
	bic	r2, r2, #112
	orr	r2, r2, #32
	str	r2, [r3, #4]
	movs	r3, #64
	str	r3, [sp, #4]
	str	r3, [sp, #8]
	movs	r3, #2
	strb	r3, [sp, #12]
	ldr	r3, [r4, #184]
	ldrb	r2, [r3, #5]	@ zero_extendqisi2
	str	r2, [sp, #16]
	ldrb	r3, [r3, #5]	@ zero_extendqisi2
	str	r3, [sp, #20]
	movs	r3, #0
	str	r3, [sp, #24]
	str	r3, [sp, #28]
	ldr	r3, [r0, #12]
	orr	r3, r3, #1048576
	str	r3, [r0, #12]
	add	r1, sp, #4
	bl	SingleADC_InjectedConfig
	str	r0, [r4, #144]
.L233:
	ldr	r3, .L239
	str	r3, [r4, #4]
	ldr	r3, .L239+4
	str	r3, [r4, #20]
	ldr	r3, .L239+8
	str	r3, [r4, #12]
	ldr	r3, .L239+12
	str	r3, [r4, #8]
	movs	r3, #1
	strb	r3, [r4, #93]
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, pc}
.L237:
	ldr	r2, .L239+16
	ands	r2, r2, r5
	orr	r2, r2, #4192
	str	r2, [r3, #24]
	ldr	r2, [r3, #32]
	bic	r2, r2, #16
	str	r2, [r3, #32]
	ldr	r2, [r3, #32]
	orr	r2, r2, #64
	str	r2, [r3, #32]
	b	.L235
.L238:
	ldr	r2, .L239+16
	ands	r2, r2, r5
	orr	r2, r2, #8256
	orr	r2, r2, #32
	str	r2, [r3, #24]
	ldr	r2, [r3, #32]
	orr	r2, r2, #16
	str	r2, [r3, #32]
	ldr	r2, [r3, #32]
	bic	r2, r2, #64
	str	r2, [r3, #32]
	b	.L235
.L240:
	.align	2
.L239:
	.word	R3_1_F30X_RLGetPhaseCurrents
	.word	R3_1_F30X_RLTurnOnLowSides
	.word	R3_1_F30X_RLSwitchOnPWM
	.word	R3_1_F30X_RLSwitchOffPWM
	.word	-16872308
	.size	R3_1_F30X_RLDetectionModeEnable, .-R3_1_F30X_RLDetectionModeEnable
	.section	.text.R3_1_F30X_RLDetectionModeDisable,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLDetectionModeDisable
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLDetectionModeDisable, %function
R3_1_F30X_RLDetectionModeDisable:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrb	r3, [r0, #93]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L254
	push	{r4}
	ldr	r1, [r0, #184]
	ldr	r3, [r1, #12]
.L243:
	ldr	r2, [r3]
	tst	r2, #16
	beq	.L243
.L244:
	ldr	r2, [r3]
	tst	r2, #16
	bne	.L244
	ldrb	r2, [r1, #7]	@ zero_extendqisi2
	str	r2, [r3, #48]
	add	r1, r3, #24
	ldr	r4, [r3, #24]
	bic	r2, r4, #65536
	bic	r2, r2, #115
	orr	r2, r2, #96
	str	r2, [r3, #24]
	ldr	r2, [r3, #32]
	orr	r2, r2, #1
	str	r2, [r3, #32]
	ldr	r2, [r0, #184]
	ldrb	r2, [r2, #16]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L257
	cmp	r2, #2
	beq	.L258
.L246:
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #52]
	ldr	r2, .L263
	ands	r2, r2, r4
	orr	r2, r2, #24576
	orr	r2, r2, #96
	str	r2, [r3, #24]
	ldr	r2, [r3, #32]
	orr	r2, r2, #16
	str	r2, [r3, #32]
	ldr	r2, [r0, #184]
	ldrb	r2, [r2, #16]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L259
	cmp	r2, #2
	beq	.L260
.L248:
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #56]
	ldr	r2, [r1, #4]
	bic	r2, r2, #65536
	bic	r2, r2, #115
	orr	r2, r2, #96
	str	r2, [r1, #4]
	ldr	r2, [r3, #32]
	orr	r2, r2, #256
	str	r2, [r3, #32]
	ldr	r2, [r0, #184]
	ldrb	r2, [r2, #16]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L261
	cmp	r2, #2
	beq	.L262
.L250:
	ldrh	r2, [r0, #128]
	lsrs	r2, r2, #1
	str	r2, [r3, #60]
	ldr	r2, [r3, #4]
	orr	r2, r2, #112
	str	r2, [r3, #4]
	ldr	r3, [r0, #184]
	ldr	r2, [r3]
	ldr	r3, [r2, #12]
	bic	r3, r3, #1048576
	str	r3, [r2, #12]
	ldr	r3, .L263+4
	str	r3, [r0, #4]
	ldr	r3, .L263+8
	str	r3, [r0, #20]
	ldr	r3, .L263+12
	str	r3, [r0, #12]
	ldr	r3, .L263+16
	str	r3, [r0, #8]
	movs	r3, #0
	strb	r3, [r0, #93]
	ldr	r4, [sp], #4
	bx	lr
.L257:
	ldr	r2, [r3, #32]
	orr	r2, r2, #4
	str	r2, [r3, #32]
	b	.L246
.L258:
	ldr	r2, [r3, #32]
	bic	r2, r2, #4
	str	r2, [r3, #32]
	b	.L246
.L259:
	ldr	r2, [r3, #32]
	orr	r2, r2, #64
	str	r2, [r3, #32]
	b	.L248
.L260:
	ldr	r2, [r3, #32]
	bic	r2, r2, #64
	str	r2, [r3, #32]
	b	.L248
.L261:
	ldr	r2, [r3, #32]
	orr	r2, r2, #1024
	str	r2, [r3, #32]
	b	.L250
.L262:
	ldr	r2, [r3, #32]
	bic	r2, r2, #1024
	str	r2, [r3, #32]
	b	.L250
.L254:
	bx	lr
.L264:
	.align	2
.L263:
	.word	-16872308
	.word	R3_1_F30X_GetPhaseCurrents
	.word	R3_1_F30X_TurnOnLowSides
	.word	R3_1_F30X_SwitchOnPWM
	.word	R3_1_F30X_SwitchOffPWM
	.size	R3_1_F30X_RLDetectionModeDisable, .-R3_1_F30X_RLDetectionModeDisable
	.section	.text.R3_1_F30X_RLDetectionModeSetDuty,"ax",%progbits
	.align	1
	.global	R3_1_F30X_RLDetectionModeSetDuty
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_1_F30X_RLDetectionModeSetDuty, %function
R3_1_F30X_RLDetectionModeSetDuty:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrh	r3, [r0, #128]
	mul	r1, r1, r3
	lsrs	r1, r1, #16
	strh	r1, [r0, #80]	@ movhi
	ldr	r3, [r0, #184]
	ldr	r3, [r3]
	ldr	r2, [r0, #144]
	str	r2, [r3, #76]
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	ldrh	r2, [r0, #80]
	str	r2, [r3, #52]
	ldr	r3, [r0, #184]
	ldr	r3, [r3, #12]
	ldr	r3, [r3, #16]
	ldrh	r2, [r0, #86]
	cmp	r2, #1
	iteee	ne
	andne	r0, r3, #1
	moveq	r3, #0
	strheq	r3, [r0, #86]	@ movhi
	moveq	r0, #1
	bx	lr
	.size	R3_1_F30X_RLDetectionModeSetDuty, .-R3_1_F30X_RLDetectionModeSetDuty
	.ident	"GCC: (GNU) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]"
