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
	.file	"r3_2_f30x_pwm_curr_fdbk.c"
	.section	.text.R3_2_F30X_GetPhaseCurrents,"ax",%progbits
	.align	1
	.global	R3_2_F30X_GetPhaseCurrents
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_GetPhaseCurrents, %function
R3_2_F30X_GetPhaseCurrents:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	movs	r3, #0
	strb	r3, [r0, #140]
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #4]
	ldr	r2, [r2, #128]
	ldr	r3, [r3, #8]
	ldr	r3, [r3, #128]
	ldrb	r4, [r0, #78]	@ zero_extendqisi2
	cmp	r4, #5
	bhi	.L2
	tbb	[pc, r4]
.L4:
	.byte	(.L3-.L4)/2
	.byte	(.L5-.L4)/2
	.byte	(.L5-.L4)/2
	.byte	(.L6-.L4)/2
	.byte	(.L6-.L4)/2
	.byte	(.L3-.L4)/2
	.p2align 1
.L6:
	uxth	r4, r3
	ldr	r3, [r0, #116]
	subs	r3, r3, r4
	cmn	r3, #32768
	bgt	.L7
	movw	r3, #32769
	strh	r3, [r1]	@ movhi
.L8:
	uxth	r2, r2
	ldr	r3, [r0, #120]
	subs	r2, r3, r2
	cmn	r2, #32768
	ble	.L23
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
	ldr	r4, [sp], #4
	bx	lr
.L7:
	cmp	r3, #32768
	it	ge
	movwge	r3, #32767
	strh	r3, [r1]	@ movhi
	b	.L8
.L23:
	movw	r3, #32769
	strh	r3, [r1, #2]	@ movhi
	b	.L2
.L3:
	uxth	r2, r2
	ldr	r4, [r0, #120]
	subs	r2, r4, r2
	cmn	r2, #32768
	bgt	.L12
	movw	r2, #32769
	strh	r2, [r1, #2]	@ movhi
.L13:
	uxth	r3, r3
	ldr	r2, [r0, #124]
	subs	r3, r3, r2
	ldrsh	r2, [r1, #2]
	subs	r3, r3, r2
	cmp	r3, #32768
	bge	.L24
	cmn	r3, #32768
	it	le
	movwle	r3, #32769
	strh	r3, [r1]	@ movhi
	b	.L2
.L12:
	cmp	r2, #32768
	it	ge
	movwge	r2, #32767
	strh	r2, [r1, #2]	@ movhi
	b	.L13
.L24:
	movw	r3, #32767
	strh	r3, [r1]	@ movhi
	b	.L2
.L5:
	uxth	r2, r2
	ldr	r4, [r0, #116]
	subs	r2, r4, r2
	cmn	r2, #32768
	bgt	.L17
	movw	r2, #32769
	strh	r2, [r1]	@ movhi
.L18:
	uxth	r3, r3
	ldr	r2, [r0, #124]
	subs	r3, r3, r2
	ldrsh	r2, [r1]
	subs	r3, r3, r2
	cmp	r3, #32768
	bge	.L25
	cmn	r3, #32768
	it	le
	movwle	r3, #32769
	strh	r3, [r1, #2]	@ movhi
	b	.L2
.L17:
	cmp	r2, #32768
	it	ge
	movwge	r2, #32767
	strh	r2, [r1]	@ movhi
	b	.L18
.L25:
	movw	r3, #32767
	strh	r3, [r1, #2]	@ movhi
	b	.L2
	.size	R3_2_F30X_GetPhaseCurrents, .-R3_2_F30X_GetPhaseCurrents
	.section	.text.R3_2_F30X_HFCurrentsCalibrationAB,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_HFCurrentsCalibrationAB, %function
R3_2_F30X_HFCurrentsCalibrationAB:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r3, #0
	strb	r3, [r0, #140]
	ldrb	r3, [r0, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L26
	ldr	r2, [r0, #172]
	ldr	r3, [r2, #8]
	ldr	r1, [r3, #128]
	ldr	r3, [r0, #116]
	add	r3, r3, r1
	str	r3, [r0, #116]
	ldr	r3, [r2, #4]
	ldr	r2, [r3, #128]
	ldr	r3, [r0, #120]
	add	r3, r3, r2
	str	r3, [r0, #120]
	ldrb	r3, [r0, #164]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r0, #164]
.L26:
	bx	lr
	.size	R3_2_F30X_HFCurrentsCalibrationAB, .-R3_2_F30X_HFCurrentsCalibrationAB
	.section	.text.R3_2_F30X_HFCurrentsCalibrationC,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_HFCurrentsCalibrationC, %function
R3_2_F30X_HFCurrentsCalibrationC:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r3, #0
	strb	r3, [r0, #140]
	ldrb	r3, [r0, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L28
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #8]
	ldr	r2, [r3, #128]
	ldr	r3, [r0, #124]
	add	r3, r3, r2
	str	r3, [r0, #124]
	ldrb	r3, [r0, #164]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r0, #164]
.L28:
	bx	lr
	.size	R3_2_F30X_HFCurrentsCalibrationC, .-R3_2_F30X_HFCurrentsCalibrationC
	.section	.text.R3_2_F30X_WriteTIMRegisters,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_WriteTIMRegisters, %function
R3_2_F30X_WriteTIMRegisters:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #24]
	ldrh	r2, [r0, #80]
	str	r2, [r3, #52]
	ldrh	r2, [r0, #82]
	str	r2, [r3, #56]
	ldrh	r2, [r0, #84]
	str	r2, [r3, #60]
	ldrb	r3, [r0, #140]	@ zero_extendqisi2
	uxtb	r3, r3
	ldrh	r2, [r0, #86]
	cmp	r2, #1
	beq	.L31
	adds	r0, r3, #0
	it	ne
	movne	r0, #1
	bx	lr
.L31:
	movs	r3, #0
	strh	r3, [r0, #86]	@ movhi
	movs	r0, #1
	bx	lr
	.size	R3_2_F30X_WriteTIMRegisters, .-R3_2_F30X_WriteTIMRegisters
	.section	.text.R3_2_F30X_ADC_InjectedChannelConfig,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_ADC_InjectedChannelConfig, %function
R3_2_F30X_ADC_InjectedChannelConfig:
	@ args = 12, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4, r5, r6, r7}
	ldr	r5, [r0, #76]
	bic	r5, r5, #3
	ldrb	r4, [sp, #16]	@ zero_extendqisi2
	subs	r4, r4, #1
	orrs	r4, r4, r5
	bic	r4, r4, #192
	ldrh	r5, [sp, #20]
	orrs	r4, r4, r5
	bic	r4, r4, #60
	ldrh	r5, [sp, #24]
	orrs	r4, r4, r5
	cmp	r1, #9
	bhi	.L37
	cbz	r1, .L35
	ldr	r6, [r0, #20]
	add	r7, r1, r1, lsl #1
	movs	r5, #7
	lsls	r5, r5, r7
	bic	r6, r6, r5
	lsl	r5, r3, r7
	orrs	r5, r5, r6
	str	r5, [r0, #20]
	b	.L35
.L37:
	ldr	r7, [r0, #24]
	add	r6, r1, r1, lsl #1
	sub	r5, r6, #30
	movs	r6, #7
	lsls	r6, r6, r5
	bic	r6, r7, r6
	lsl	r5, r3, r5
	orrs	r5, r5, r6
	str	r5, [r0, #24]
.L35:
	add	r2, r2, r2, lsl #1
	lsls	r2, r2, #1
	subs	r3, r2, #6
	mov	r0, #7936
	lsls	r0, r0, r3
	bic	r0, r4, r0
	adds	r2, r2, #2
	lsl	r2, r1, r2
	orrs	r0, r0, r2
	pop	{r4, r5, r6, r7}
	bx	lr
	.size	R3_2_F30X_ADC_InjectedChannelConfig, .-R3_2_F30X_ADC_InjectedChannelConfig
	.section	.text.R3_2_F30X_SetAOReferenceVoltage,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetAOReferenceVoltage, %function
R3_2_F30X_SetAOReferenceVoltage:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4}
	ldr	r3, .L43
	cmp	r0, r3
	beq	.L42
	mov	r2, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r2, r2
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r2, r2
	mov	r4, #1048576
	lsr	r2, r4, r2
	ldr	r4, .L43+4
	ldr	r3, [r4, r2, lsl #2]
	bic	r3, r3, #65280
	bic	r3, r3, #240
	orrs	r3, r3, r1
	str	r3, [r4, r2, lsl #2]
.L40:
	ldr	r2, .L43+8
	ldr	r1, [r2, #4]
	and	r3, r0, #3
	orrs	r3, r3, r1
	str	r3, [r2, #4]
	ldr	r1, [r2]
	and	r3, r0, #16
	movs	r0, #1
	lsls	r0, r0, r3
	orrs	r0, r0, r1
	str	r0, [r2]
	ldr	r4, [sp], #4
	bx	lr
.L42:
	mov	r3, #15728640
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	clz	r3, r3
	mov	r2, #4194304
	lsr	r3, r2, r3
	ldr	r4, .L43+4
	ldr	r2, [r4, r3, lsl #2]
	bic	r2, r2, #65280
	bic	r2, r2, #240
	orrs	r2, r2, r1
	str	r2, [r4, r3, lsl #2]
	b	.L40
.L44:
	.align	2
.L43:
	.word	356712466
	.word	1073771528
	.word	1073771520
	.size	R3_2_F30X_SetAOReferenceVoltage, .-R3_2_F30X_SetAOReferenceVoltage
	.section	.text.R3_2_F30X_Init,"ax",%progbits
	.align	1
	.global	R3_2_F30X_Init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_Init, %function
R3_2_F30X_Init:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	sub	sp, sp, #28
	mov	r4, r0
	ldr	r3, [r0, #172]
	ldr	r1, [r3, #56]
	ldr	r7, [r3, #60]
	ldr	r9, [r3, #68]
	ldr	r8, [r3, #76]
	ldr	r6, [r3, #84]
	ldr	r5, [r3, #24]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #4]
	bic	r3, r3, #4
	str	r3, [r2, #4]
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #4]
	movs	r2, #4
	str	r2, [r3]
	ldr	r3, [r0, #172]
	ldr	r0, [r3, #4]
	ldr	r3, [r0, #4]
	bic	r3, r3, #32
	str	r3, [r0, #4]
	ldr	r3, [r4, #172]
	ldr	r0, [r3, #4]
	movs	r3, #32
	str	r3, [r0]
	ldr	r0, [r4, #172]
	ldr	ip, [r0, #8]
	ldr	r0, [ip, #4]
	bic	r0, r0, #4
	str	r0, [ip, #4]
	ldr	r0, [r4, #172]
	ldr	r0, [r0, #8]
	str	r2, [r0]
	ldr	r0, [r4, #172]
	ldr	ip, [r0, #8]
	ldr	r0, [ip, #4]
	bic	r0, r0, #32
	str	r0, [ip, #4]
	ldr	r0, [r4, #172]
	ldr	r0, [r0, #8]
	str	r3, [r0]
	ldr	r0, [r4, #172]
	ldr	ip, [r0, #96]
	ldr	r0, [ip, #4]
	bic	r0, r0, #4
	str	r0, [ip, #4]
	ldr	r0, [r4, #172]
	ldr	r0, [r0, #96]
	str	r2, [r0]
	ldr	r2, [r4, #172]
	ldr	r0, [r2, #96]
	ldr	r2, [r0, #4]
	bic	r2, r2, #32
	str	r2, [r0, #4]
	ldr	r2, [r4, #172]
	ldr	r2, [r2, #96]
	str	r3, [r2]
	ldr	r3, [r5]
	bic	r3, r3, #1
	str	r3, [r5]
	movw	r0, #2056
	ldr	r2, [r5, #24]
	orrs	r2, r2, r0
	str	r2, [r5, #24]
	ldr	r3, [r5, #28]
	orrs	r3, r3, r0
	str	r3, [r5, #28]
	mvn	r3, #128
	str	r3, [r5, #16]
	ldr	r3, [r4, #172]
	ldrb	r3, [r3, #54]	@ zero_extendqisi2
	cbz	r3, .L47
	mvn	r3, #256
	str	r3, [r5, #16]
.L47:
	ldr	r3, [r5, #12]
	orr	r3, r3, #128
	str	r3, [r5, #12]
	ldr	r3, [r5, #20]
	orr	r3, r3, #1
	str	r3, [r5, #20]
	ldr	r3, [r4, #172]
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L91
	ldrb	r2, [r4, #92]	@ zero_extendqisi2
	cbnz	r2, .L50
	ldrb	r3, [r3, #18]	@ zero_extendqisi2
	cmp	r3, #1
	beq	.L92
	cmp	r3, #3
	bne	.L50
	movs	r3, #1
	str	r3, [r5, #48]
	ldr	r3, [r5, #20]
	orr	r3, r3, #1
	str	r3, [r5, #20]
	movs	r3, #3
	str	r3, [r5, #48]
	b	.L50
.L91:
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L93
.L49:
	ldrh	r3, [r4, #136]
	subs	r3, r3, #1
	str	r3, [r5, #36]
.L50:
	ldr	r3, .L98
	cmp	r5, r3
	ldr	r2, .L98+4
	ldr	r3, [r2, #12]
	ite	eq
	orreq	r3, r3, #1
	orrne	r3, r3, #2
	str	r3, [r2, #12]
	cbz	r1, .L54
	ldr	r2, .L98+8
	ldr	r3, [r2]
	orr	r3, r3, #1
	str	r3, [r2]
	adds	r2, r2, #8
	ldr	r3, [r2]
	orr	r3, r3, #1
	str	r3, [r2]
.L54:
	cbz	r7, .L55
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #64]	@ zero_extendqisi2
	cmp	r3, #1
	beq	.L56
	ldr	r3, [r7]
	and	r3, r3, #112
	cmp	r3, #64
	beq	.L94
	ldr	r3, [r7]
	and	r3, r3, #112
	cmp	r3, #80
	beq	.L95
.L56:
	movs	r3, #0
	strh	r3, [sp, #20]	@ movhi
	strh	r3, [sp, #20]	@ movhi
	ldrh	r3, [sp, #20]
	uxth	r3, r3
	cmp	r3, #1000
	bcs	.L58
.L59:
	ldrh	r3, [sp, #20]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [sp, #20]	@ movhi
	ldrh	r3, [sp, #20]
	uxth	r3, r3
	cmp	r3, #1000
	bcc	.L59
.L58:
	ldr	r3, [r7]
	orr	r3, r3, #1
	str	r3, [r7]
	ldr	r3, [r7]
	orr	r3, r3, #-2147483648
	str	r3, [r7]
.L55:
	cmp	r9, #0
	beq	.L60
	ldr	r3, [r9]
	orr	r3, r3, #1
	str	r3, [r9]
	ldr	r3, [r9]
	orr	r3, r3, #-2147483648
	str	r3, [r9]
.L60:
	cmp	r8, #0
	beq	.L61
	ldr	r3, [r8]
	orr	r3, r3, #1
	str	r3, [r8]
	ldr	r3, [r8]
	orr	r3, r3, #-2147483648
	str	r3, [r8]
.L61:
	cbz	r6, .L62
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #88]	@ zero_extendqisi2
	cmp	r3, #1
	beq	.L63
	ldr	r3, [r6]
	and	r3, r3, #112
	cmp	r3, #64
	beq	.L96
	ldr	r3, [r6]
	and	r3, r3, #112
	cmp	r3, #80
	beq	.L97
.L63:
	movs	r3, #0
	strh	r3, [sp, #22]	@ movhi
	strh	r3, [sp, #22]	@ movhi
	ldrh	r3, [sp, #22]
	uxth	r3, r3
	cmp	r3, #1000
	bcs	.L65
.L66:
	ldrh	r3, [sp, #22]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [sp, #22]	@ movhi
	ldrh	r3, [sp, #22]
	uxth	r3, r3
	cmp	r3, #1000
	bcc	.L66
.L65:
	ldr	r3, [r6]
	orr	r3, r3, #1
	str	r3, [r6]
	ldr	r3, [r6]
	orr	r3, r3, #-2147483648
	str	r3, [r6]
.L62:
	ldrb	r3, [r4, #92]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L67
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #8]
	bic	r3, r3, #805306368
	str	r3, [r2, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-1879048192
	bic	r3, r3, #63
	orr	r3, r3, #268435456
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #805306368
	str	r3, [r2, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-1879048192
	bic	r3, r3, #63
	orr	r3, r3, #268435456
	str	r3, [r2, #8]
	movw	r3, #65000
.L68:
	subs	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #0
	bne	.L68
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-1073741824
	bic	r3, r3, #63
	orr	r3, r3, #-2147483648
	str	r3, [r2, #8]
	ldr	r1, [r4, #172]
	ldr	r2, [r1, #4]
.L69:
	ldr	r3, [r2, #8]
	cmp	r3, #0
	blt	.L69
	ldr	r2, [r1, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-1073741824
	bic	r3, r3, #63
	orr	r3, r3, #-2147483648
	str	r3, [r2, #8]
	ldr	r1, [r4, #172]
	ldr	r2, [r1, #8]
.L70:
	ldr	r3, [r2, #8]
	cmp	r3, #0
	blt	.L70
	ldr	r3, [r1, #96]
	ldr	r1, [r1, #4]
	cmp	r3, r1
	beq	.L71
	cmp	r2, r3
	beq	.L71
	ldr	r2, [r3, #8]
	bic	r2, r2, #805306368
	str	r2, [r3, #8]
	ldr	r2, [r3, #8]
	bic	r2, r2, #-1879048192
	bic	r2, r2, #63
	orr	r2, r2, #268435456
	str	r2, [r3, #8]
	movw	r3, #65000
.L72:
	subs	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #0
	bne	.L72
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #96]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-1073741824
	bic	r3, r3, #63
	orr	r3, r3, #-2147483648
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #96]
.L73:
	ldr	r3, [r2, #8]
	cmp	r3, #0
	blt	.L73
.L71:
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #1
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #1
	str	r3, [r2, #8]
.L67:
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #96]
	ldr	r1, [r3, #4]
	cmp	r2, r1
	beq	.L74
	ldr	r3, [r3, #8]
	cmp	r2, r3
	beq	.L74
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #1
	str	r3, [r2, #8]
.L74:
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #96]
	ldr	r3, [r2, #48]
	bic	r3, r3, #15
	str	r3, [r2, #48]
	ldr	r6, .L98
	cmp	r5, r6
	ite	eq
	moveq	r6, #64
	movne	r6, #100
	ldr	r2, [r4, #172]
	ldr	r3, [r2, #56]
	cmp	r3, #0
	beq	.L76
	ldrb	r3, [r2, #13]	@ zero_extendqisi2
	ldr	r0, [r2, #4]
	str	r6, [sp, #8]
	mov	r8, #64
	str	r8, [sp, #4]
	movs	r7, #1
	str	r7, [sp]
	mov	r2, r7
	movs	r1, #3
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r4, #152]
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #15]	@ zero_extendqisi2
	ldr	r0, [r2, #8]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	mov	r1, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r4, #156]
	ldr	r3, [r4, #152]
	str	r3, [r4, #144]
	str	r0, [r4, #148]
	b	.L99
.L100:
	.align	2
.L98:
	.word	1073818624
	.word	-536600576
	.word	1073807416
.L99:
.L77:
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #32
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #32
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r8, [r3, #4]
	str	r6, [sp, #8]
	mov	r9, #64
	str	r9, [sp, #4]
	movs	r7, #1
	str	r7, [sp]
	movs	r3, #0
	mov	r2, r7
	mov	r1, r3
	mov	r0, r8
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r8, #76]
	ldr	r3, [r4, #172]
	ldr	r8, [r3, #8]
	str	r6, [sp, #8]
	str	r9, [sp, #4]
	str	r7, [sp]
	movs	r3, #0
	mov	r2, r7
	mov	r1, r3
	mov	r0, r8
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r8, #76]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #8
	str	r3, [r2, #8]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #8]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #8
	str	r3, [r2, #8]
	ldr	r3, [r5, #28]
	movw	r2, #65535
	str	r2, [r5, #64]
	movs	r2, #0
	str	r2, [r5, #64]
	orr	r3, r3, #2048
	str	r3, [r5, #28]
	ldr	r1, [r4, #172]
	ldr	r2, [r1, #4]
.L78:
	ldr	r3, [r2]
	tst	r3, #64
	beq	.L78
	ldr	r1, [r1, #8]
.L79:
	ldr	r3, [r1]
	tst	r3, #64
	beq	.L79
	movs	r3, #64
	str	r3, [r2]
	ldr	r2, [r4, #172]
	ldr	r2, [r2, #8]
	str	r3, [r2]
	ldr	r3, .L101
	mov	r2, #262144
	str	r2, [r3, #384]
	ldr	r3, [r4, #172]
	ldr	r2, [r3, #4]
	ldr	r3, [r2, #4]
	orr	r3, r3, #64
	str	r3, [r2, #4]
	movs	r3, #0
	strb	r3, [r4, #169]
	strb	r3, [r4, #168]
	strh	r3, [r4, #100]	@ movhi
	ldrh	r3, [r4, #108]
	strh	r3, [r4, #102]	@ movhi
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L93:
	ldrb	r3, [r3, #18]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L49
	movs	r3, #1
	str	r3, [r5, #48]
	ldr	r3, [r5, #20]
	orr	r3, r3, #1
	str	r3, [r5, #20]
	movs	r3, #3
	str	r3, [r5, #48]
	b	.L49
.L92:
	ldrh	r3, [r4, #136]
	subs	r3, r3, #1
	str	r3, [r5, #36]
	b	.L50
.L94:
	ldrh	r1, [r2, #90]
	ldr	r0, .L101+4
	bl	R3_2_F30X_SetAOReferenceVoltage
	b	.L56
.L95:
	ldrh	r1, [r2, #90]
	ldr	r0, .L101+8
	bl	R3_2_F30X_SetAOReferenceVoltage
	b	.L56
.L96:
	ldrh	r1, [r2, #92]
	ldr	r0, .L101+4
	bl	R3_2_F30X_SetAOReferenceVoltage
	b	.L63
.L97:
	ldrh	r1, [r2, #92]
	ldr	r0, .L101+8
	bl	R3_2_F30X_SetAOReferenceVoltage
	b	.L63
.L76:
	ldrb	r3, [r2, #13]	@ zero_extendqisi2
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	ldr	r0, [r2, #4]
	str	r6, [sp, #8]
	mov	r8, #64
	str	r8, [sp, #4]
	movs	r7, #1
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r4, #152]
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #15]	@ zero_extendqisi2
	ldrb	r1, [r2, #14]	@ zero_extendqisi2
	ldr	r0, [r2, #8]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r4, #156]
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #17]	@ zero_extendqisi2
	ldrb	r1, [r2, #16]	@ zero_extendqisi2
	ldr	r0, [r2, #8]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	str	r0, [r4, #160]
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #13]	@ zero_extendqisi2
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	ldr	r0, [r2, #8]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #15]	@ zero_extendqisi2
	ldrb	r1, [r2, #14]	@ zero_extendqisi2
	ldr	r0, [r2, #4]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	ldr	r2, [r4, #172]
	ldrb	r3, [r2, #17]	@ zero_extendqisi2
	ldrb	r1, [r2, #16]	@ zero_extendqisi2
	ldr	r0, [r2, #4]
	str	r6, [sp, #8]
	str	r8, [sp, #4]
	str	r7, [sp]
	mov	r2, r7
	bl	R3_2_F30X_ADC_InjectedChannelConfig
	b	.L77
.L102:
	.align	2
.L101:
	.word	-536813312
	.word	34603009
	.word	356712466
	.size	R3_2_F30X_Init, .-R3_2_F30X_Init
	.section	.text.R3_2_F30X_TurnOnLowSides,"ax",%progbits
	.align	1
	.global	R3_2_F30X_TurnOnLowSides
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_TurnOnLowSides, %function
R3_2_F30X_TurnOnLowSides:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #24]
	movs	r1, #1
	strb	r1, [r0, #88]
	ldr	r3, [r3, #24]
	mvn	r1, #1
	str	r1, [r3, #16]
	movs	r3, #0
	str	r3, [r2, #52]
	str	r3, [r2, #56]
	str	r3, [r2, #60]
.L104:
	ldr	r3, [r2, #16]
	tst	r3, #1
	beq	.L104
	ldr	r3, [r2, #68]
	orr	r3, r3, #32768
	str	r3, [r2, #68]
	ldr	r3, [r0, #172]
	ldrb	r2, [r3, #28]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L107
.L103:
	bx	lr
.L107:
	ldr	r2, [r3, #32]
	ldrh	r3, [r3, #36]
	str	r3, [r2, #24]
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #40]
	ldrh	r3, [r3, #44]
	str	r3, [r2, #24]
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #48]
	ldrh	r3, [r3, #52]
	str	r3, [r2, #24]
	b	.L103
	.size	R3_2_F30X_TurnOnLowSides, .-R3_2_F30X_TurnOnLowSides
	.section	.text.R3_2_F30X_SwitchOnPWM,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SwitchOnPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SwitchOnPWM, %function
R3_2_F30X_SwitchOnPWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #24]
	movs	r2, #0
	strb	r2, [r0, #88]
	mvn	r2, #1
	str	r2, [r3, #16]
.L109:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L109
	mvn	r2, #1
	str	r2, [r3, #16]
	ldrh	r2, [r0, #136]
	lsrs	r2, r2, #1
	str	r2, [r3, #52]
	ldrh	r2, [r0, #136]
	lsrs	r2, r2, #1
	str	r2, [r3, #56]
	ldrh	r2, [r0, #136]
	lsrs	r2, r2, #1
	str	r2, [r3, #60]
	ldrh	r2, [r0, #136]
	subs	r2, r2, #5
	str	r2, [r3, #64]
.L110:
	ldr	r2, [r3, #16]
	tst	r2, #1
	beq	.L110
	ldr	r2, [r3, #68]
	orr	r2, r2, #1024
	str	r2, [r3, #68]
	ldr	r2, [r3, #68]
	orr	r2, r2, #32768
	str	r2, [r3, #68]
	ldr	r2, [r0, #172]
	ldrb	r1, [r2, #28]	@ zero_extendqisi2
	cmp	r1, #2
	beq	.L114
.L111:
	mvn	r2, #1
	str	r2, [r3, #16]
	ldr	r2, [r3, #12]
	orr	r2, r2, #1
	str	r2, [r3, #12]
	bx	lr
.L114:
	ldr	r1, [r2, #32]
	ldrh	r2, [r2, #36]
	str	r2, [r1, #24]
	ldr	r2, [r0, #172]
	ldr	r1, [r2, #40]
	ldrh	r2, [r2, #44]
	str	r2, [r1, #24]
	ldr	r2, [r0, #172]
	ldr	r1, [r2, #48]
	ldrh	r2, [r2, #52]
	str	r2, [r1, #24]
	b	.L111
	.size	R3_2_F30X_SwitchOnPWM, .-R3_2_F30X_SwitchOnPWM
	.section	.text.R3_2_F30X_SwitchOffPWM,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SwitchOffPWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SwitchOffPWM, %function
R3_2_F30X_SwitchOffPWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #24]
	movs	r2, #0
	strb	r2, [r0, #88]
	ldrb	r2, [r0, #170]	@ zero_extendqisi2
	cbnz	r2, .L116
	ldr	r2, [r3, #68]
	bic	r2, r2, #1024
	str	r2, [r3, #68]
	ldr	r2, [r0, #172]
	ldrb	r1, [r2, #28]	@ zero_extendqisi2
	cmp	r1, #2
	beq	.L117
.L116:
	ldr	r2, [r3, #68]
	bic	r2, r2, #32768
	str	r2, [r3, #68]
	ldr	r2, [r3, #12]
	bic	r2, r2, #1
	str	r2, [r3, #12]
	bx	lr
.L117:
	ldr	r1, [r2, #32]
	ldrh	r2, [r2, #36]
	str	r2, [r1, #40]
	ldr	r2, [r0, #172]
	ldr	r1, [r2, #40]
	ldrh	r2, [r2, #44]
	str	r2, [r1, #40]
	ldr	r2, [r0, #172]
	ldr	r1, [r2, #48]
	ldrh	r2, [r2, #52]
	str	r2, [r1, #40]
	b	.L116
	.size	R3_2_F30X_SwitchOffPWM, .-R3_2_F30X_SwitchOffPWM
	.section	.text.R3_2_F30X_CurrentReadingCalibration,"ax",%progbits
	.align	1
	.global	R3_2_F30X_CurrentReadingCalibration
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_CurrentReadingCalibration, %function
R3_2_F30X_CurrentReadingCalibration:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldr	r3, [r0, #172]
	ldr	r7, [r3, #56]
	ldr	r5, [r3, #24]
	movs	r3, #0
	str	r3, [r0, #116]
	str	r3, [r0, #120]
	str	r3, [r0, #124]
	strb	r3, [r0, #164]
	ldr	r3, [r5, #32]
	bic	r3, r3, #1360
	bic	r3, r3, #5
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r5, #32]
	ldr	r3, .L134
	str	r3, [r0, #4]
	cmp	r7, #0
	beq	.L119
	ldr	r3, .L134+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r7, #4]
	ldr	r1, [r7, #12]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L134+8
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r7, #20]
	ldr	r1, [r7, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
.L120:
	mov	r0, r4
	bl	R3_2_F30X_SwitchOnPWM
	ldr	r3, [r4, #172]
	ldrb	r6, [r3, #18]	@ zero_extendqisi2
	adds	r6, r6, #1
	lsrs	r6, r6, #1
	add	r6, r6, r6, lsl #4
	mvn	r3, #2
	str	r3, [r5, #16]
	movs	r2, #0
	mov	r1, r3
.L122:
	ldrb	r3, [r4, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L123
	ldr	r3, [r5, #16]
	tst	r3, #2
	beq	.L122
	str	r1, [r5, #16]
	adds	r2, r2, #1
	uxth	r2, r2
	cmp	r6, r2
	bhi	.L122
	ldrb	r3, [r4, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L122
	movs	r3, #1
	strh	r3, [r4, #86]	@ movhi
.L123:
	mov	r0, r4
	bl	R3_2_F30X_SwitchOffPWM
	movs	r3, #0
	strb	r3, [r4, #164]
	ldr	r3, .L134+12
	str	r3, [r4, #4]
	cmp	r7, #0
	beq	.L125
	ldr	r3, .L134+8
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r4, #132]
	ldr	r3, [r7, #20]
	ldr	r1, [r7, #28]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r4, #132]
.L126:
	mov	r0, r4
	bl	R3_2_F30X_SwitchOnPWM
	mvn	r3, #2
	str	r3, [r5, #16]
	movs	r2, #0
	mov	r1, r3
.L128:
	ldrb	r3, [r4, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L129
	ldr	r3, [r5, #16]
	tst	r3, #2
	beq	.L128
	str	r1, [r5, #16]
	adds	r2, r2, #1
	uxth	r2, r2
	cmp	r6, r2
	bhi	.L128
	ldrb	r3, [r4, #164]	@ zero_extendqisi2
	uxtb	r3, r3
	cmp	r3, #15
	bhi	.L128
	movs	r3, #1
	strh	r3, [r4, #86]	@ movhi
.L129:
	mov	r0, r4
	bl	R3_2_F30X_SwitchOffPWM
	ldr	r3, [r4, #116]
	lsrs	r3, r3, #4
	str	r3, [r4, #116]
	ldr	r3, [r4, #120]
	lsrs	r3, r3, #4
	str	r3, [r4, #120]
	ldr	r3, [r4, #124]
	lsrs	r3, r3, #4
	str	r3, [r4, #124]
	ldr	r3, .L134+16
	str	r3, [r4, #4]
	ldr	r2, [r5, #24]
	movw	r1, #63479
	ands	r2, r2, r1
	str	r2, [r5, #24]
	ldr	r3, [r5, #28]
	ands	r3, r3, r1
	str	r3, [r5, #28]
	ldrh	r3, [r4, #136]
	str	r3, [r5, #52]
	ldrh	r3, [r4, #136]
	str	r3, [r5, #56]
	ldrh	r3, [r4, #136]
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
	strb	r3, [r4, #170]
	pop	{r3, r4, r5, r6, r7, pc}
.L119:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L120
.L125:
	ldr	r3, [r4, #160]
	str	r3, [r4, #144]
	b	.L126
.L135:
	.align	2
.L134:
	.word	R3_2_F30X_HFCurrentsCalibrationAB
	.word	1073807416
	.word	1073807424
	.word	R3_2_F30X_HFCurrentsCalibrationC
	.word	R3_2_F30X_GetPhaseCurrents
	.size	R3_2_F30X_CurrentReadingCalibration, .-R3_2_F30X_CurrentReadingCalibration
	.section	.text.R3_2_F30X_SetADCSampPointSect1,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect1, %function
R3_2_F30X_SetADCSampPointSect1:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r2, [r0, #172]
	ldr	r3, [r2, #24]
	ldr	r5, [r2, #56]
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #28672
	str	r2, [r3, #28]
	ldrh	r2, [r0, #136]
	ldrh	r4, [r0, #80]
	subs	r1, r2, r4
	uxth	r1, r1
	ldr	ip, [r0, #172]
	ldrh	r6, [ip, #20]
	cmp	r1, r6
	bls	.L137
	ldrh	r1, [r0, #82]
	subs	r1, r2, r1
	uxth	r1, r1
	cmp	r6, r1
	bcs	.L138
	subs	r2, r2, #1
	str	r2, [r3, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r5, .L139
	ldr	r3, .L146
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r5, #4]
	ldr	r1, [r5, #12]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L146+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r5, #20]
	ldr	r1, [r5, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L140
.L139:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L140
.L138:
	subs	r4, r2, #1
	uxth	r4, r4
.L141:
	cmp	r5, #0
	beq	.L143
	ldr	r2, .L146
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #128]
	ldr	r2, [r5, #4]
	ldr	r6, [r5, #12]
	orrs	r2, r2, r6
	orrs	r2, r2, r1
	str	r2, [r0, #128]
	ldr	r2, .L146+4
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #132]
	ldr	r2, [r5, #20]
	ldr	r5, [r5, #28]
	orrs	r2, r2, r5
	orrs	r2, r2, r1
	str	r2, [r0, #132]
.L144:
	str	r4, [r3, #64]
.L140:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L137:
	ldrh	r7, [r0, #82]
	subs	r7, r4, r7
	uxth	r7, r7
	cmp	r7, r1, lsl #1
	bls	.L142
	ldrh	r2, [ip, #22]
	subs	r4, r4, r2
	uxth	r4, r4
	b	.L141
.L142:
	add	r4, r4, r6
	uxth	r4, r4
	cmp	r2, r4
	bhi	.L141
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #24576
	str	r2, [r3, #28]
	ldrh	r2, [r0, #136]
	mvns	r4, r4
	add	r4, r4, r2, lsl #1
	uxth	r4, r4
	b	.L141
.L143:
	ldr	r2, [r0, #156]
	str	r2, [r0, #144]
	ldr	r2, [r0, #160]
	str	r2, [r0, #148]
	b	.L144
.L147:
	.align	2
.L146:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect1, .-R3_2_F30X_SetADCSampPointSect1
	.section	.text.R3_2_F30X_SetADCSampPointSect2,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect2
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect2, %function
R3_2_F30X_SetADCSampPointSect2:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #24]
	ldr	r1, [r3, #56]
	ldr	r3, [r2, #28]
	bic	r3, r3, #28672
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r2, #28]
	ldr	r3, [r2, #28]
	orr	r3, r3, #28672
	str	r3, [r2, #28]
	ldrh	r5, [r0, #136]
	ldrh	r6, [r0, #80]
	ldr	r7, [r0, #172]
	ldrh	r3, [r7, #20]
	subs	r4, r5, r6
	uxth	r4, r4
	cmp	r4, r3
	bls	.L149
	ldrh	ip, [r0, #82]
	sub	r4, r5, ip
	uxth	r4, r4
	cmp	r3, r4
	bcs	.L150
	subs	r5, r5, #1
	str	r5, [r2, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r1, .L151
	ldr	r3, .L158
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r1, #4]
	ldr	r4, [r1, #12]
	orrs	r3, r3, r4
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L158+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r1, #20]
	ldr	r1, [r1, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L152
.L151:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L152
.L149:
	ldrh	ip, [r0, #82]
	sub	r4, r5, ip
	uxth	r4, r4
	cmp	r3, r4
	bcs	.L150
	subs	r3, r5, #1
	uxth	r3, r3
.L153:
	cmp	r1, #0
	beq	.L155
	ldr	r4, .L158
	ldr	r5, [r4]
	bic	r5, r5, #108
	str	r5, [r0, #128]
	ldr	r4, [r1, #4]
	ldr	r6, [r1, #8]
	orrs	r4, r4, r6
	orrs	r4, r4, r5
	str	r4, [r0, #128]
	ldr	r4, .L158+4
	ldr	r5, [r4]
	bic	r5, r5, #108
	str	r5, [r0, #132]
	ldr	r4, [r1, #20]
	ldr	r1, [r1, #28]
	orrs	r1, r1, r4
	orrs	r1, r1, r5
	str	r1, [r0, #132]
.L156:
	str	r3, [r2, #64]
.L152:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L150:
	sub	r6, ip, r6
	uxth	r6, r6
	cmp	r6, r4, lsl #1
	bls	.L154
	ldrh	r3, [r7, #22]
	sub	r3, ip, r3
	uxth	r3, r3
	b	.L153
.L154:
	add	r3, r3, ip
	uxth	r3, r3
	cmp	r5, r3
	bhi	.L153
	ldr	r4, [r2, #28]
	bic	r4, r4, #28672
	lsls	r4, r4, #16
	lsrs	r4, r4, #16
	str	r4, [r2, #28]
	ldr	r4, [r2, #28]
	orr	r4, r4, #24576
	str	r4, [r2, #28]
	ldrh	r4, [r0, #136]
	mvns	r3, r3
	add	r3, r3, r4, lsl #1
	uxth	r3, r3
	b	.L153
.L155:
	ldr	r1, [r0, #152]
	str	r1, [r0, #144]
	ldr	r1, [r0, #160]
	str	r1, [r0, #148]
	b	.L156
.L159:
	.align	2
.L158:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect2, .-R3_2_F30X_SetADCSampPointSect2
	.section	.text.R3_2_F30X_SetADCSampPointSect3,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect3
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect3, %function
R3_2_F30X_SetADCSampPointSect3:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r2, [r0, #172]
	ldr	r3, [r2, #24]
	ldr	r4, [r2, #56]
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #28672
	str	r2, [r3, #28]
	ldrh	r1, [r0, #136]
	ldr	ip, [r0, #172]
	ldrh	r5, [ip, #20]
	ldrh	r2, [r0, #80]
	subs	r2, r1, r2
	uxth	r2, r2
	cmp	r2, r5
	bls	.L161
	ldrh	r6, [r0, #82]
	subs	r2, r1, r6
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L162
	subs	r1, r1, #1
	str	r1, [r3, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r4, .L163
	ldr	r3, .L170
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r4, #4]
	ldr	r1, [r4, #12]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L170+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r4, #20]
	ldr	r1, [r4, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L164
.L163:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L164
.L161:
	ldrh	r6, [r0, #82]
	subs	r2, r1, r6
	uxth	r2, r2
	cmp	r5, r2
	bcs	.L162
	subs	r5, r1, #1
	uxth	r5, r5
.L165:
	cmp	r4, #0
	beq	.L167
	ldr	r2, .L170
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #128]
	ldr	r2, [r4, #4]
	ldr	r6, [r4, #8]
	orrs	r2, r2, r6
	orrs	r2, r2, r1
	str	r2, [r0, #128]
	ldr	r2, .L170+4
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #132]
	ldr	r2, [r4, #20]
	ldr	r4, [r4, #28]
	orrs	r2, r2, r4
	orrs	r2, r2, r1
	str	r2, [r0, #132]
.L168:
	str	r5, [r3, #64]
.L164:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L162:
	ldrh	r7, [r0, #84]
	subs	r7, r6, r7
	uxth	r7, r7
	cmp	r7, r2, lsl #1
	bls	.L166
	ldrh	r5, [ip, #22]
	subs	r5, r6, r5
	uxth	r5, r5
	b	.L165
.L166:
	add	r5, r5, r6
	uxth	r5, r5
	cmp	r1, r5
	bhi	.L165
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #24576
	str	r2, [r3, #28]
	ldrh	r1, [r0, #136]
	mvns	r5, r5
	add	r5, r5, r1, lsl #1
	uxth	r5, r5
	b	.L165
.L167:
	ldr	r2, [r0, #152]
	str	r2, [r0, #144]
	ldr	r2, [r0, #160]
	str	r2, [r0, #148]
	b	.L168
.L171:
	.align	2
.L170:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect3, .-R3_2_F30X_SetADCSampPointSect3
	.section	.text.R3_2_F30X_SetADCSampPointSect4,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect4
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect4, %function
R3_2_F30X_SetADCSampPointSect4:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r3, [r0, #172]
	ldr	r1, [r3, #24]
	ldr	r4, [r3, #56]
	ldr	r3, [r1, #28]
	bic	r3, r3, #28672
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r1, #28]
	ldr	r3, [r1, #28]
	orr	r3, r3, #28672
	str	r3, [r1, #28]
	ldrh	r5, [r0, #136]
	ldr	r6, [r0, #172]
	ldrh	r2, [r6, #20]
	ldrh	r3, [r0, #80]
	subs	r3, r5, r3
	uxth	r3, r3
	cmp	r3, r2
	bls	.L173
	ldrh	r3, [r0, #82]
	subs	r3, r5, r3
	uxth	r3, r3
	cmp	r2, r3
	bcs	.L173
	subs	r5, r5, #1
	str	r5, [r1, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r4, .L174
	ldr	r3, .L182
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r4, #4]
	ldr	r1, [r4, #12]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L182+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r4, #20]
	ldr	r1, [r4, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L175
.L174:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L175
.L173:
	ldrh	r7, [r0, #84]
	subs	r3, r5, r7
	uxth	r3, r3
	cmp	r2, r3
	bcs	.L176
	subs	r2, r5, #1
	uxth	r2, r2
.L177:
	cmp	r4, #0
	beq	.L179
	ldr	r3, .L182
	ldr	r5, [r3]
	bic	r5, r5, #108
	str	r5, [r0, #128]
	ldr	r3, [r4, #4]
	ldr	r6, [r4, #12]
	orrs	r3, r3, r6
	orrs	r3, r3, r5
	str	r3, [r0, #128]
	ldr	r3, .L182+4
	ldr	r5, [r3]
	bic	r5, r5, #108
	str	r5, [r0, #132]
	ldr	r3, [r4, #20]
	ldr	r4, [r4, #24]
	orrs	r3, r3, r4
	orrs	r3, r3, r5
	str	r3, [r0, #132]
.L180:
	str	r2, [r1, #64]
.L175:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L176:
	ldrh	ip, [r0, #82]
	sub	ip, r7, ip
	uxth	ip, ip
	cmp	ip, r3, lsl #1
	bls	.L178
	ldrh	r2, [r6, #22]
	subs	r2, r7, r2
	uxth	r2, r2
	b	.L177
.L178:
	add	r2, r2, r7
	uxth	r2, r2
	cmp	r5, r2
	bhi	.L177
	ldr	r3, [r1, #28]
	bic	r3, r3, #28672
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r1, #28]
	ldr	r3, [r1, #28]
	orr	r3, r3, #24576
	str	r3, [r1, #28]
	ldrh	r3, [r0, #136]
	mvns	r2, r2
	add	r2, r2, r3, lsl #1
	uxth	r2, r2
	b	.L177
.L179:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L180
.L183:
	.align	2
.L182:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect4, .-R3_2_F30X_SetADCSampPointSect4
	.section	.text.R3_2_F30X_SetADCSampPointSect5,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect5
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect5, %function
R3_2_F30X_SetADCSampPointSect5:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #24]
	ldr	r1, [r3, #56]
	ldr	r3, [r2, #28]
	bic	r3, r3, #28672
	lsls	r3, r3, #16
	lsrs	r3, r3, #16
	str	r3, [r2, #28]
	ldr	r3, [r2, #28]
	orr	r3, r3, #28672
	str	r3, [r2, #28]
	ldrh	r6, [r0, #136]
	ldrh	r5, [r0, #80]
	ldr	r7, [r0, #172]
	ldrh	r3, [r7, #20]
	subs	r4, r6, r5
	uxth	r4, r4
	cmp	r4, r3
	bls	.L185
	ldrh	r4, [r0, #82]
	subs	r4, r6, r4
	uxth	r4, r4
	cmp	r3, r4
	bcs	.L185
	subs	r6, r6, #1
	str	r6, [r2, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r1, .L186
	ldr	r3, .L194
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r1, #4]
	ldr	r4, [r1, #12]
	orrs	r3, r3, r4
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L194+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r1, #20]
	ldr	r1, [r1, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L187
.L186:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L187
.L185:
	ldrh	ip, [r0, #84]
	sub	r4, r6, ip
	uxth	r4, r4
	cmp	r3, r4
	bcs	.L188
	subs	r3, r6, #1
	uxth	r3, r3
.L189:
	cmp	r1, #0
	beq	.L191
	ldr	r4, .L194
	ldr	r5, [r4]
	bic	r5, r5, #108
	str	r5, [r0, #128]
	ldr	r4, [r1, #4]
	ldr	r6, [r1, #12]
	orrs	r4, r4, r6
	orrs	r4, r4, r5
	str	r4, [r0, #128]
	ldr	r4, .L194+4
	ldr	r5, [r4]
	bic	r5, r5, #108
	str	r5, [r0, #132]
	ldr	r4, [r1, #20]
	ldr	r1, [r1, #24]
	orrs	r1, r1, r4
	orrs	r1, r1, r5
	str	r1, [r0, #132]
.L192:
	str	r3, [r2, #64]
.L187:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L188:
	sub	r5, ip, r5
	uxth	r5, r5
	cmp	r5, r4, lsl #1
	bls	.L190
	ldrh	r3, [r7, #22]
	sub	r3, ip, r3
	uxth	r3, r3
	b	.L189
.L190:
	add	r3, r3, ip
	uxth	r3, r3
	cmp	r6, r3
	bhi	.L189
	ldr	r4, [r2, #28]
	bic	r4, r4, #28672
	lsls	r4, r4, #16
	lsrs	r4, r4, #16
	str	r4, [r2, #28]
	ldr	r4, [r2, #28]
	orr	r4, r4, #24576
	str	r4, [r2, #28]
	ldrh	r4, [r0, #136]
	mvns	r3, r3
	add	r3, r3, r4, lsl #1
	uxth	r3, r3
	b	.L189
.L191:
	ldr	r1, [r0, #156]
	str	r1, [r0, #144]
	ldr	r1, [r0, #152]
	str	r1, [r0, #148]
	b	.L192
.L195:
	.align	2
.L194:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect5, .-R3_2_F30X_SetADCSampPointSect5
	.section	.text.R3_2_F30X_SetADCSampPointSect6,"ax",%progbits
	.align	1
	.global	R3_2_F30X_SetADCSampPointSect6
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_SetADCSampPointSect6, %function
R3_2_F30X_SetADCSampPointSect6:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	ldr	r2, [r0, #172]
	ldr	r3, [r2, #24]
	ldr	r5, [r2, #56]
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #28672
	str	r2, [r3, #28]
	ldrh	r2, [r0, #136]
	ldrh	r4, [r0, #80]
	subs	r1, r2, r4
	uxth	r1, r1
	ldr	ip, [r0, #172]
	ldrh	r6, [ip, #20]
	cmp	r1, r6
	bls	.L197
	ldrh	r1, [r0, #82]
	subs	r1, r2, r1
	uxth	r1, r1
	cmp	r6, r1
	bcs	.L198
	subs	r2, r2, #1
	str	r2, [r3, #64]
	movs	r3, #3
	strh	r3, [r0, #78]	@ movhi
	cbz	r5, .L199
	ldr	r3, .L206
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #128]
	ldr	r3, [r5, #4]
	ldr	r1, [r5, #12]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #128]
	ldr	r3, .L206+4
	ldr	r2, [r3]
	bic	r2, r2, #108
	str	r2, [r0, #132]
	ldr	r3, [r5, #20]
	ldr	r1, [r5, #24]
	orrs	r3, r3, r1
	orrs	r3, r3, r2
	str	r3, [r0, #132]
	b	.L200
.L199:
	ldr	r3, [r0, #156]
	str	r3, [r0, #144]
	ldr	r3, [r0, #152]
	str	r3, [r0, #148]
	b	.L200
.L198:
	subs	r4, r2, #1
	uxth	r4, r4
.L201:
	cmp	r5, #0
	beq	.L203
	ldr	r2, .L206
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #128]
	ldr	r2, [r5, #4]
	ldr	r6, [r5, #12]
	orrs	r2, r2, r6
	orrs	r2, r2, r1
	str	r2, [r0, #128]
	ldr	r2, .L206+4
	ldr	r1, [r2]
	bic	r1, r1, #108
	str	r1, [r0, #132]
	ldr	r2, [r5, #20]
	ldr	r5, [r5, #28]
	orrs	r2, r2, r5
	orrs	r2, r2, r1
	str	r2, [r0, #132]
.L204:
	str	r4, [r3, #64]
.L200:
	bl	R3_2_F30X_WriteTIMRegisters
	pop	{r3, r4, r5, r6, r7, pc}
.L197:
	ldrh	r7, [r0, #84]
	subs	r7, r4, r7
	uxth	r7, r7
	cmp	r7, r1, lsl #1
	bls	.L202
	ldrh	r2, [ip, #22]
	subs	r4, r4, r2
	uxth	r4, r4
	b	.L201
.L202:
	add	r4, r4, r6
	uxth	r4, r4
	cmp	r2, r4
	bhi	.L201
	ldr	r2, [r3, #28]
	bic	r2, r2, #28672
	lsls	r2, r2, #16
	lsrs	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r2, [r3, #28]
	orr	r2, r2, #24576
	str	r2, [r3, #28]
	ldrh	r2, [r0, #136]
	mvns	r4, r4
	add	r4, r4, r2, lsl #1
	uxth	r4, r4
	b	.L201
.L203:
	ldr	r2, [r0, #156]
	str	r2, [r0, #144]
	ldr	r2, [r0, #160]
	str	r2, [r0, #148]
	b	.L204
.L207:
	.align	2
.L206:
	.word	1073807416
	.word	1073807424
	.size	R3_2_F30X_SetADCSampPointSect6, .-R3_2_F30X_SetADCSampPointSect6
	.section	.text.R3_2_F30X_TIMx_UP_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_2_F30X_TIMx_UP_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_TIMx_UP_IRQHandler, %function
R3_2_F30X_TIMx_UP_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #56]
	cbz	r2, .L209
	ldr	r2, [r3, #4]
.L210:
	ldr	r3, [r2, #76]
	cmp	r3, #0
	bne	.L210
	ldr	r2, [r0, #128]
	ldr	r3, .L212
	str	r2, [r3]
	ldr	r2, [r0, #132]
	adds	r3, r3, #8
	str	r2, [r3]
.L209:
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #4]
	ldr	r2, [r0, #144]
	str	r2, [r3, #76]
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #8]
	ldr	r2, [r0, #148]
	str	r2, [r3, #76]
	movs	r3, #1
	strb	r3, [r0, #140]
	adds	r0, r0, #92
	bx	lr
.L213:
	.align	2
.L212:
	.word	1073807416
	.size	R3_2_F30X_TIMx_UP_IRQHandler, .-R3_2_F30X_TIMx_UP_IRQHandler
	.section	.text.R3_2_F30X_BRK2_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_2_F30X_BRK2_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_BRK2_IRQHandler, %function
R3_2_F30X_BRK2_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrb	r3, [r0, #170]	@ zero_extendqisi2
	cbnz	r3, .L215
	ldr	r3, [r0, #172]
	ldrb	r2, [r3, #28]	@ zero_extendqisi2
	cmp	r2, #2
	beq	.L216
.L215:
	movs	r3, #1
	strb	r3, [r0, #168]
	adds	r0, r0, #92
	bx	lr
.L216:
	ldr	r2, [r3, #32]
	ldrh	r3, [r3, #36]
	str	r3, [r2, #40]
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #40]
	ldrh	r3, [r3, #44]
	str	r3, [r2, #40]
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #48]
	ldrh	r3, [r3, #52]
	str	r3, [r2, #40]
	b	.L215
	.size	R3_2_F30X_BRK2_IRQHandler, .-R3_2_F30X_BRK2_IRQHandler
	.section	.text.R3_2_F30X_BRK_IRQHandler,"ax",%progbits
	.align	1
	.global	R3_2_F30X_BRK_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_BRK_IRQHandler, %function
R3_2_F30X_BRK_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #24]
	ldr	r3, [r2, #68]
	orr	r3, r3, #1024
	str	r3, [r2, #68]
	movs	r3, #1
	strb	r3, [r0, #169]
	strb	r3, [r0, #170]
	adds	r0, r0, #92
	bx	lr
	.size	R3_2_F30X_BRK_IRQHandler, .-R3_2_F30X_BRK_IRQHandler
	.section	.text.R3_2_F30X_ExecRegularConv,"ax",%progbits
	.align	1
	.global	R3_2_F30X_ExecRegularConv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_ExecRegularConv, %function
R3_2_F30X_ExecRegularConv:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #172]
	ldr	r2, [r3, #96]
	mov	r3, #768
	.syntax unified
@ 531 "../CMSIS/cmsis_gcc.h" 1
	rbit r3, r3
@ 0 "" 2
	.thumb
	.syntax unified
	lsls	r3, r1, #6
	and	r3, r3, #1984
	ldr	r1, [r2, #48]
	bic	r1, r1, #1984
	orrs	r3, r3, r1
	str	r3, [r2, #48]
	ldr	r3, [r2, #8]
	bic	r3, r3, #-2147483648
	bic	r3, r3, #63
	orr	r3, r3, #4
	str	r3, [r2, #8]
.L219:
	ldr	r3, [r2]
	tst	r3, #4
	beq	.L219
	ldr	r3, [r2, #64]
	uxth	r3, r3
	strh	r3, [r0, #138]	@ movhi
	mov	r0, r3
	bx	lr
	.size	R3_2_F30X_ExecRegularConv, .-R3_2_F30X_ExecRegularConv
	.section	.text.R3_2_F30X_ADC_SetSamplingTime,"ax",%progbits
	.align	1
	.global	R3_2_F30X_ADC_SetSamplingTime
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_ADC_SetSamplingTime, %function
R3_2_F30X_ADC_SetSamplingTime:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r4, r5, r6}
	sub	sp, sp, #12
	strh	r1, [sp, #4]	@ movhi
	uxtb	r3, r1
	ubfx	r1, r1, #8, #8
	cmp	r3, #9
	bhi	.L225
	add	r3, r3, r3, lsl #1
	ldr	r2, [r0, #172]
	ldr	r5, [r2, #96]
	ldr	r2, [r5, #20]
	subs	r6, r3, #3
	movs	r4, #56
	lsls	r4, r4, r6
	bic	r2, r2, r4
	str	r2, [r5, #20]
	ldr	r2, [r0, #172]
	ldr	r0, [r2, #96]
	ldr	r2, [r0, #20]
	lsl	r3, r1, r3
	orrs	r3, r3, r2
	str	r3, [r0, #20]
.L221:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6}
	bx	lr
.L225:
	add	r3, r3, r3, lsl #1
	sub	r2, r3, #30
	ldr	r3, [r0, #172]
	ldr	r5, [r3, #96]
	ldr	r3, [r5, #24]
	movs	r4, #7
	lsls	r4, r4, r2
	bic	r3, r3, r4
	str	r3, [r5, #24]
	ldr	r3, [r0, #172]
	ldr	r3, [r3, #96]
	ldr	r0, [r3, #24]
	lsl	r2, r1, r2
	orrs	r2, r2, r0
	str	r2, [r3, #24]
	b	.L221
	.size	R3_2_F30X_ADC_SetSamplingTime, .-R3_2_F30X_ADC_SetSamplingTime
	.section	.text.R3_2_F30X_IsOverCurrentOccurred,"ax",%progbits
	.align	1
	.global	R3_2_F30X_IsOverCurrentOccurred
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	R3_2_F30X_IsOverCurrentOccurred, %function
R3_2_F30X_IsOverCurrentOccurred:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, r0
	ldrb	r2, [r0, #169]	@ zero_extendqisi2
	cbz	r2, .L229
	movs	r2, #0
	strb	r2, [r0, #169]
	movs	r0, #2
.L227:
	ldrb	r2, [r3, #168]	@ zero_extendqisi2
	cbz	r2, .L228
	orr	r0, r0, #64
	movs	r2, #0
	strb	r2, [r3, #168]
.L228:
	bx	lr
.L229:
	movs	r0, #0
	b	.L227
	.size	R3_2_F30X_IsOverCurrentOccurred, .-R3_2_F30X_IsOverCurrentOccurred
	.ident	"GCC: (GNU) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]"
