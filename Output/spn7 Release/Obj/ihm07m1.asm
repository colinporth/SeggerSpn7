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
	.file	"ihm07m1.cpp"
	.section	.text.ADC1_2_IRQHandler,"ax",%progbits
	.align	1
	.global	ADC1_2_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	ADC1_2_IRQHandler, %function
ADC1_2_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r0, .L3
	bl	HAL_ADC_IRQHandler
	pop	{r3, pc}
.L4:
	.align	2
.L3:
	.word	.LANCHOR0
	.size	ADC1_2_IRQHandler, .-ADC1_2_IRQHandler
	.section	.text.TIM1_BRK_TIM15_IRQHandler,"ax",%progbits
	.align	1
	.global	TIM1_BRK_TIM15_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	TIM1_BRK_TIM15_IRQHandler, %function
TIM1_BRK_TIM15_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r3, .L9
	ldr	r3, [r3]
	ldr	r3, [r3, #16]
	tst	r3, #128
	bne	.L8
.L6:
	ldr	r0, .L9
	bl	HAL_TIM_IRQHandler
	pop	{r3, pc}
.L8:
	bl	MC_Panic
	b	.L6
.L10:
	.align	2
.L9:
	.word	.LANCHOR1
	.size	TIM1_BRK_TIM15_IRQHandler, .-TIM1_BRK_TIM15_IRQHandler
	.section	.text.TIM6_DAC_IRQHandler,"ax",%progbits
	.align	1
	.global	TIM6_DAC_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	TIM6_DAC_IRQHandler, %function
TIM6_DAC_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r0, .L13
	bl	HAL_TIM_IRQHandler
	ldr	r0, .L13+4
	bl	HAL_DAC_IRQHandler
	pop	{r3, pc}
.L14:
	.align	2
.L13:
	.word	.LANCHOR2
	.word	.LANCHOR3
	.size	TIM6_DAC_IRQHandler, .-TIM6_DAC_IRQHandler
	.section	.text._Z9GPIO_Initv,"ax",%progbits
	.align	1
	.global	_Z9GPIO_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z9GPIO_Initv, %function
_Z9GPIO_Initv:
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	sub	sp, sp, #40
	ldr	r3, .L17
	ldr	r2, [r3, #20]
	orr	r2, r2, #524288
	str	r2, [r3, #20]
	ldr	r2, [r3, #20]
	and	r2, r2, #524288
	str	r2, [sp, #4]
	ldr	r2, [r3, #20]
	orr	r2, r2, #4194304
	str	r2, [r3, #20]
	ldr	r2, [r3, #20]
	and	r2, r2, #4194304
	str	r2, [sp, #8]
	ldr	r2, [r3, #20]
	orr	r2, r2, #131072
	str	r2, [r3, #20]
	ldr	r2, [r3, #20]
	and	r2, r2, #131072
	str	r2, [sp, #12]
	ldr	r2, [r3, #20]
	orr	r2, r2, #262144
	str	r2, [r3, #20]
	ldr	r3, [r3, #20]
	and	r3, r3, #262144
	str	r3, [sp, #16]
	mov	r3, #8192
	str	r3, [sp, #20]
	ldr	r3, .L17+4
	str	r3, [sp, #24]
	movs	r4, #0
	str	r4, [sp, #28]
	ldr	r5, .L17+8
	add	r1, sp, #20
	mov	r0, r5
	bl	HAL_GPIO_Init
	ldr	r6, .L17+12
	mov	r2, r4
	movs	r1, #4
	mov	r0, r6
	bl	HAL_GPIO_WritePin
	mov	r8, #7168
	str	r8, [sp, #20]
	movs	r7, #1
	str	r7, [sp, #24]
	str	r4, [sp, #28]
	str	r4, [sp, #32]
	add	r1, sp, #20
	mov	r0, r5
	bl	HAL_GPIO_Init
	mov	r2, r4
	mov	r1, r8
	mov	r0, r5
	bl	HAL_GPIO_WritePin
	movs	r3, #4
	str	r3, [sp, #20]
	str	r7, [sp, #24]
	str	r4, [sp, #28]
	str	r4, [sp, #32]
	add	r1, sp, #20
	mov	r0, r6
	bl	HAL_GPIO_Init
	mov	r2, r4
	mov	r1, r4
	movs	r0, #40
	bl	HAL_NVIC_SetPriority
	movs	r0, #40
	bl	HAL_NVIC_EnableIRQ
	add	sp, sp, #40
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L18:
	.align	2
.L17:
	.word	1073876992
	.word	269549568
	.word	1207961600
	.word	1207960576
	.size	_Z9GPIO_Initv, .-_Z9GPIO_Initv
	.section	.text._Z9ADC1_Initv,"ax",%progbits
	.align	1
	.global	_Z9ADC1_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z9ADC1_Initv, %function
_Z9ADC1_Initv:
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	sub	sp, sp, #52
	ldr	r3, .L25
	ldr	r2, [r3, #20]
	orr	r2, r2, #268435456
	str	r2, [r3, #20]
	ldr	r3, [r3, #20]
	and	r3, r3, #268435456
	str	r3, [sp]
	movs	r3, #14
	str	r3, [sp, #28]
	movs	r5, #3
	str	r5, [sp, #32]
	movs	r4, #0
	str	r4, [sp, #36]
	add	r1, sp, #28
	ldr	r0, .L25+4
	bl	HAL_GPIO_Init
	movs	r3, #130
	str	r3, [sp, #28]
	str	r5, [sp, #32]
	str	r4, [sp, #36]
	add	r1, sp, #28
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	str	r5, [sp, #28]
	str	r5, [sp, #32]
	str	r4, [sp, #36]
	add	r1, sp, #28
	ldr	r0, .L25+8
	bl	HAL_GPIO_Init
	mov	r2, r4
	mov	r1, r4
	movs	r0, #18
	bl	HAL_NVIC_SetPriority
	movs	r0, #18
	bl	HAL_NVIC_EnableIRQ
	ldr	r0, .L25+12
	mov	r3, #1342177280
	str	r3, [r0]
	str	r4, [r0, #4]
	str	r4, [r0, #8]
	str	r4, [r0, #16]
	str	r4, [r0, #28]
	str	r4, [r0, #36]
	mov	r3, #1024
	str	r3, [r0, #48]
	mov	r3, #576
	str	r3, [r0, #44]
	str	r4, [r0, #12]
	movs	r3, #1
	str	r3, [r0, #32]
	str	r4, [r0, #52]
	movs	r3, #4
	str	r3, [r0, #20]
	str	r4, [r0, #24]
	str	r4, [r0, #56]
	bl	HAL_ADC_Init
	cbnz	r0, .L23
.L20:
	movs	r3, #12
	str	r3, [sp, #4]
	movs	r3, #1
	str	r3, [sp, #8]
	movs	r3, #0
	str	r3, [sp, #16]
	movs	r2, #3
	str	r2, [sp, #12]
	str	r3, [sp, #20]
	str	r3, [sp, #24]
	add	r1, sp, #4
	ldr	r0, .L25+12
	bl	HAL_ADC_ConfigChannel
	cbnz	r0, .L24
.L19:
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, pc}
.L23:
	ldr	r0, .L25+16
	bl	printf
	b	.L20
.L24:
	ldr	r0, .L25+20
	bl	printf
	b	.L19
.L26:
	.align	2
.L25:
	.word	1073876992
	.word	1207961600
	.word	1207960576
	.word	.LANCHOR0
	.word	.LC0
	.word	.LC1
	.size	_Z9ADC1_Initv, .-_Z9ADC1_Initv
	.section	.text._Z9TIM1_Initv,"ax",%progbits
	.align	1
	.global	_Z9TIM1_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z9TIM1_Initv, %function
_Z9TIM1_Initv:
	@ args = 0, pretend = 0, frame = 144
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	sub	sp, sp, #148
	ldr	r3, .L51
	ldr	r2, [r3, #24]
	orr	r2, r2, #2048
	str	r2, [r3, #24]
	ldr	r3, [r3, #24]
	and	r3, r3, #2048
	str	r3, [sp]
	movs	r3, #64
	str	r3, [sp, #124]
	movs	r5, #2
	str	r5, [sp, #128]
	movs	r4, #0
	str	r4, [sp, #132]
	str	r4, [sp, #136]
	movs	r3, #6
	str	r3, [sp, #140]
	add	r1, sp, #124
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	mov	r3, #4096
	str	r3, [sp, #124]
	str	r5, [sp, #128]
	str	r4, [sp, #132]
	str	r4, [sp, #136]
	movs	r3, #11
	str	r3, [sp, #140]
	add	r1, sp, #124
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	ldr	r0, .L51+4
	ldr	r3, .L51+8
	str	r3, [r0]
	str	r4, [r0, #4]
	movs	r3, #32
	str	r3, [r0, #8]
	movw	r3, #719
	str	r3, [r0, #12]
	str	r4, [r0, #16]
	str	r4, [r0, #20]
	str	r4, [r0, #24]
	bl	HAL_TIM_Base_Init
	cmp	r0, #0
	bne	.L40
.L28:
	add	r1, sp, #144
	mov	r3, #4096
	str	r3, [r1, #-36]!
	ldr	r0, .L51+4
	bl	HAL_TIM_ConfigClockSource
	cmp	r0, #0
	bne	.L41
.L29:
	ldr	r0, .L51+4
	bl	HAL_TIM_PWM_Init
	cmp	r0, #0
	bne	.L42
.L30:
	movs	r3, #1
	str	r3, [sp, #88]
	str	r3, [sp, #92]
	movs	r2, #0
	str	r2, [sp, #96]
	str	r2, [sp, #100]
	str	r2, [sp, #104]
	add	r1, sp, #88
	ldr	r0, .L51+4
	bl	HAL_TIM_ConfigOCrefClear
	cmp	r0, #0
	bne	.L43
.L31:
	movs	r2, #4
	add	r1, sp, #88
	ldr	r0, .L51+4
	bl	HAL_TIM_ConfigOCrefClear
	cmp	r0, #0
	bne	.L44
.L32:
	movs	r2, #8
	add	r1, sp, #88
	ldr	r0, .L51+4
	bl	HAL_TIM_ConfigOCrefClear
	cmp	r0, #0
	bne	.L45
.L33:
	movs	r3, #32
	str	r3, [sp, #76]
	movs	r3, #0
	str	r3, [sp, #80]
	movs	r3, #128
	str	r3, [sp, #84]
	add	r1, sp, #76
	ldr	r0, .L51+4
	bl	HAL_TIMEx_MasterConfigSynchronization
	cmp	r0, #0
	bne	.L46
.L34:
	movs	r3, #96
	str	r3, [sp, #48]
	movw	r3, #575
	str	r3, [sp, #52]
	movs	r2, #0
	str	r2, [sp, #56]
	str	r2, [sp, #60]
	str	r2, [sp, #64]
	str	r2, [sp, #68]
	str	r2, [sp, #72]
	add	r1, sp, #48
	ldr	r0, .L51+4
	bl	HAL_TIM_PWM_ConfigChannel
	cmp	r0, #0
	bne	.L47
.L35:
	movs	r2, #4
	add	r1, sp, #48
	ldr	r0, .L51+4
	bl	HAL_TIM_PWM_ConfigChannel
	cmp	r0, #0
	bne	.L48
.L36:
	movs	r2, #8
	add	r1, sp, #48
	ldr	r0, .L51+4
	bl	HAL_TIM_PWM_ConfigChannel
	cmp	r0, #0
	bne	.L49
.L37:
	movs	r3, #0
	str	r3, [sp, #4]
	str	r3, [sp, #8]
	str	r3, [sp, #12]
	str	r3, [sp, #16]
	mov	r2, #4096
	str	r2, [sp, #20]
	str	r3, [sp, #24]
	str	r3, [sp, #28]
	str	r3, [sp, #32]
	str	r3, [sp, #36]
	str	r3, [sp, #40]
	str	r3, [sp, #44]
	add	r1, sp, #4
	ldr	r0, .L51+4
	bl	HAL_TIMEx_ConfigBreakDeadTime
	cmp	r0, #0
	bne	.L50
.L38:
	mov	r3, #1792
	str	r3, [sp, #124]
	movs	r3, #2
	str	r3, [sp, #128]
	movs	r3, #0
	str	r3, [sp, #132]
	str	r3, [sp, #136]
	movs	r3, #6
	str	r3, [sp, #140]
	add	r1, sp, #124
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	add	sp, sp, #148
	@ sp needed
	pop	{r4, r5, pc}
.L40:
	ldr	r0, .L51+12
	bl	printf
	b	.L28
.L41:
	ldr	r0, .L51+16
	bl	printf
	b	.L29
.L42:
	ldr	r0, .L51+20
	bl	printf
	b	.L30
.L43:
	ldr	r0, .L51+24
	bl	printf
	b	.L31
.L44:
	ldr	r0, .L51+28
	bl	printf
	b	.L32
.L45:
	ldr	r0, .L51+32
	bl	printf
	b	.L33
.L46:
	ldr	r0, .L51+36
	bl	printf
	b	.L34
.L47:
	ldr	r0, .L51+40
	bl	printf
	b	.L35
.L48:
	ldr	r0, .L51+44
	bl	printf
	b	.L36
.L49:
	ldr	r0, .L51+48
	bl	printf
	b	.L37
.L50:
	ldr	r0, .L51+52
	bl	printf
	b	.L38
.L52:
	.align	2
.L51:
	.word	1073876992
	.word	.LANCHOR1
	.word	1073818624
	.word	.LC2
	.word	.LC3
	.word	.LC4
	.word	.LC5
	.word	.LC6
	.word	.LC7
	.word	.LC8
	.word	.LC9
	.word	.LC10
	.word	.LC11
	.word	.LC12
	.size	_Z9TIM1_Initv, .-_Z9TIM1_Initv
	.section	.text._Z9TIM2_Initv,"ax",%progbits
	.align	1
	.global	_Z9TIM2_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z9TIM2_Initv, %function
_Z9TIM2_Initv:
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	sub	sp, sp, #80
	ldr	r3, .L65
	ldr	r2, [r3, #28]
	orr	r2, r2, #1
	str	r2, [r3, #28]
	ldr	r3, [r3, #28]
	and	r3, r3, #1
	str	r3, [sp]
	mov	r3, #1032
	str	r3, [sp, #60]
	movs	r3, #2
	str	r3, [sp, #64]
	movs	r4, #0
	str	r4, [sp, #68]
	str	r4, [sp, #72]
	movs	r3, #1
	str	r3, [sp, #76]
	add	r1, sp, #60
	ldr	r0, .L65+4
	bl	HAL_GPIO_Init
	ldr	r0, .L65+8
	mov	r3, #1073741824
	str	r3, [r0]
	movw	r3, #719
	str	r3, [r0, #4]
	str	r4, [r0, #8]
	movw	r3, #50000
	str	r3, [r0, #12]
	str	r4, [r0, #16]
	str	r4, [r0, #24]
	bl	HAL_TIM_Base_Init
	cmp	r0, #0
	bne	.L60
.L54:
	add	r1, sp, #80
	mov	r3, #4096
	str	r3, [r1, #-36]!
	ldr	r0, .L65+8
	bl	HAL_TIM_ConfigClockSource
	cbnz	r0, .L61
.L55:
	ldr	r0, .L65+8
	bl	HAL_TIM_PWM_Init
	cbnz	r0, .L62
.L56:
	movs	r3, #0
	str	r3, [sp, #32]
	str	r3, [sp, #40]
	add	r1, sp, #32
	ldr	r0, .L65+8
	bl	HAL_TIMEx_MasterConfigSynchronization
	cbnz	r0, .L63
.L57:
	movs	r3, #96
	str	r3, [sp, #4]
	movs	r2, #0
	str	r2, [sp, #8]
	str	r2, [sp, #12]
	str	r2, [sp, #20]
	add	r1, sp, #4
	ldr	r0, .L65+8
	bl	HAL_TIM_PWM_ConfigChannel
	cbnz	r0, .L64
.L58:
	movs	r3, #32
	str	r3, [sp, #60]
	movs	r3, #2
	str	r3, [sp, #64]
	movs	r3, #0
	str	r3, [sp, #68]
	str	r3, [sp, #72]
	movs	r3, #1
	str	r3, [sp, #76]
	add	r1, sp, #60
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	add	sp, sp, #80
	@ sp needed
	pop	{r4, pc}
.L60:
	ldr	r0, .L65+12
	bl	printf
	b	.L54
.L61:
	ldr	r0, .L65+16
	bl	printf
	b	.L55
.L62:
	ldr	r0, .L65+20
	bl	printf
	b	.L56
.L63:
	ldr	r0, .L65+24
	bl	printf
	b	.L57
.L64:
	ldr	r0, .L65+28
	bl	printf
	b	.L58
.L66:
	.align	2
.L65:
	.word	1073876992
	.word	1207960576
	.word	.LANCHOR4
	.word	.LC2
	.word	.LC3
	.word	.LC4
	.word	.LC8
	.word	.LC13
	.size	_Z9TIM2_Initv, .-_Z9TIM2_Initv
	.section	.text._Z9TIM6_Initv,"ax",%progbits
	.align	1
	.global	_Z9TIM6_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z9TIM6_Initv, %function
_Z9TIM6_Initv:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{lr}
	sub	sp, sp, #20
	ldr	r3, .L73
	ldr	r2, [r3, #28]
	orr	r2, r2, #16
	str	r2, [r3, #28]
	ldr	r3, [r3, #28]
	and	r3, r3, #16
	str	r3, [sp]
	movs	r2, #0
	movs	r1, #1
	movs	r0, #54
	bl	HAL_NVIC_SetPriority
	movs	r0, #54
	bl	HAL_NVIC_EnableIRQ
	ldr	r0, .L73+4
	ldr	r3, .L73+8
	str	r3, [r0]
	movs	r3, #11
	str	r3, [r0, #4]
	movs	r3, #0
	str	r3, [r0, #8]
	movw	r2, #24000
	str	r2, [r0, #12]
	str	r3, [r0, #24]
	bl	HAL_TIM_Base_Init
	cbnz	r0, .L71
.L68:
	movs	r3, #0
	str	r3, [sp, #4]
	str	r3, [sp, #12]
	add	r1, sp, #4
	ldr	r0, .L73+4
	bl	HAL_TIMEx_MasterConfigSynchronization
	cbnz	r0, .L72
.L67:
	add	sp, sp, #20
	@ sp needed
	ldr	pc, [sp], #4
.L71:
	ldr	r0, .L73+12
	bl	printf
	b	.L68
.L72:
	ldr	r0, .L73+16
	bl	printf
	b	.L67
.L74:
	.align	2
.L73:
	.word	1073876992
	.word	.LANCHOR2
	.word	1073745920
	.word	.LC2
	.word	.LC8
	.size	_Z9TIM6_Initv, .-_Z9TIM6_Initv
	.section	.text._Z10TIM16_Initv,"ax",%progbits
	.align	1
	.global	_Z10TIM16_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z10TIM16_Initv, %function
_Z10TIM16_Initv:
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{lr}
	sub	sp, sp, #100
	ldr	r3, .L85
	ldr	r2, [r3, #24]
	orr	r2, r2, #131072
	str	r2, [r3, #24]
	ldr	r3, [r3, #24]
	and	r3, r3, #131072
	str	r3, [sp]
	ldr	r0, .L85+4
	ldr	r3, .L85+8
	str	r3, [r0]
	movs	r3, #0
	str	r3, [r0, #4]
	str	r3, [r0, #8]
	movw	r2, #1439
	str	r2, [r0, #12]
	str	r3, [r0, #16]
	str	r3, [r0, #20]
	str	r3, [r0, #24]
	bl	HAL_TIM_Base_Init
	cmp	r0, #0
	bne	.L81
.L76:
	ldr	r0, .L85+4
	bl	HAL_TIM_PWM_Init
	cmp	r0, #0
	bne	.L82
.L77:
	movs	r3, #96
	str	r3, [sp, #68]
	mov	r3, #720
	str	r3, [sp, #72]
	movs	r2, #0
	str	r2, [sp, #76]
	str	r2, [sp, #80]
	movs	r3, #4
	str	r3, [sp, #84]
	str	r2, [sp, #88]
	str	r2, [sp, #92]
	add	r1, sp, #68
	ldr	r0, .L85+4
	bl	HAL_TIM_PWM_ConfigChannel
	cbnz	r0, .L83
.L78:
	movs	r3, #0
	str	r3, [sp, #24]
	str	r3, [sp, #28]
	str	r3, [sp, #32]
	str	r3, [sp, #36]
	str	r3, [sp, #40]
	mov	r2, #8192
	str	r2, [sp, #44]
	str	r3, [sp, #48]
	str	r3, [sp, #64]
	add	r1, sp, #24
	ldr	r0, .L85+4
	bl	HAL_TIMEx_ConfigBreakDeadTime
	cbnz	r0, .L84
.L79:
	movs	r3, #16
	str	r3, [sp, #4]
	movs	r3, #2
	str	r3, [sp, #8]
	movs	r3, #0
	str	r3, [sp, #12]
	str	r3, [sp, #16]
	movs	r3, #1
	str	r3, [sp, #20]
	add	r1, sp, #4
	ldr	r0, .L85+12
	bl	HAL_GPIO_Init
	add	sp, sp, #100
	@ sp needed
	ldr	pc, [sp], #4
.L81:
	ldr	r0, .L85+16
	bl	printf
	b	.L76
.L82:
	ldr	r0, .L85+20
	bl	printf
	b	.L77
.L83:
	ldr	r0, .L85+24
	bl	printf
	b	.L78
.L84:
	ldr	r0, .L85+28
	bl	printf
	b	.L79
.L86:
	.align	2
.L85:
	.word	1073876992
	.word	.LANCHOR5
	.word	1073824768
	.word	1207960576
	.word	.LC2
	.word	.LC4
	.word	.LC13
	.word	.LC12
	.size	_Z10TIM16_Initv, .-_Z10TIM16_Initv
	.section	.text._Z8DAC_Initv,"ax",%progbits
	.align	1
	.global	_Z8DAC_Initv
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	_Z8DAC_Initv, %function
_Z8DAC_Initv:
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	sub	sp, sp, #40
	ldr	r3, .L93
	ldr	r2, [r3, #28]
	orr	r2, r2, #536870912
	str	r2, [r3, #28]
	ldr	r3, [r3, #28]
	and	r3, r3, #536870912
	str	r3, [sp, #4]
	movs	r3, #16
	str	r3, [sp, #20]
	movs	r3, #3
	str	r3, [sp, #24]
	movs	r4, #0
	str	r4, [sp, #28]
	add	r1, sp, #20
	mov	r0, #1207959552
	bl	HAL_GPIO_Init
	mov	r2, r4
	movs	r1, #1
	movs	r0, #54
	bl	HAL_NVIC_SetPriority
	movs	r0, #54
	bl	HAL_NVIC_EnableIRQ
	ldr	r0, .L93+4
	ldr	r3, .L93+8
	str	r3, [r0]
	bl	HAL_DAC_Init
	cbnz	r0, .L91
.L88:
	movs	r2, #0
	str	r2, [sp, #8]
	str	r2, [sp, #12]
	add	r1, sp, #8
	ldr	r0, .L93+4
	bl	HAL_DAC_ConfigChannel
	cbnz	r0, .L92
.L87:
	add	sp, sp, #40
	@ sp needed
	pop	{r4, pc}
.L91:
	ldr	r0, .L93+12
	bl	printf
	b	.L88
.L92:
	ldr	r0, .L93+16
	bl	printf
	b	.L87
.L94:
	.align	2
.L93:
	.word	1073876992
	.word	.LANCHOR3
	.word	1073771520
	.word	.LC14
	.word	.LC15
	.size	_Z8DAC_Initv, .-_Z8DAC_Initv
	.section	.text.MC_ADC_Channel,"ax",%progbits
	.align	1
	.global	MC_ADC_Channel
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_ADC_Channel, %function
MC_ADC_Channel:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L98
	ldr	r3, [r3]
	ldr	r2, [r3, #8]
	orr	r2, r2, #16
	str	r2, [r3, #8]
.L96:
	ldr	r2, [r3, #8]
	tst	r2, #16
	bne	.L96
	ldr	r2, [r3, #48]
	bic	r2, r2, #1984
	str	r2, [r3, #48]
	ldr	r2, [r3, #48]
	orr	r0, r2, r0, lsl #6
	str	r0, [r3, #48]
	ldr	r2, [r3, #8]
	orr	r2, r2, #4
	str	r2, [r3, #8]
	bx	lr
.L99:
	.align	2
.L98:
	.word	.LANCHOR0
	.size	MC_ADC_Channel, .-MC_ADC_Channel
	.section	.text.MC_EnableInput_CH1_E_CH2_E_CH3_D,"ax",%progbits
	.align	1
	.global	MC_EnableInput_CH1_E_CH2_E_CH3_D
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_EnableInput_CH1_E_CH2_E_CH3_D, %function
MC_EnableInput_CH1_E_CH2_E_CH3_D:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L102
	movs	r2, #1
	mov	r1, #1024
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #1
	mov	r1, #2048
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #0
	mov	r1, #4096
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	pop	{r4, pc}
.L103:
	.align	2
.L102:
	.word	1207961600
	.size	MC_EnableInput_CH1_E_CH2_E_CH3_D, .-MC_EnableInput_CH1_E_CH2_E_CH3_D
	.section	.text.MC_EnableInput_CH1_E_CH2_D_CH3_E,"ax",%progbits
	.align	1
	.global	MC_EnableInput_CH1_E_CH2_D_CH3_E
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_EnableInput_CH1_E_CH2_D_CH3_E, %function
MC_EnableInput_CH1_E_CH2_D_CH3_E:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L106
	movs	r2, #1
	mov	r1, #1024
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #0
	mov	r1, #2048
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #1
	mov	r1, #4096
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	pop	{r4, pc}
.L107:
	.align	2
.L106:
	.word	1207961600
	.size	MC_EnableInput_CH1_E_CH2_D_CH3_E, .-MC_EnableInput_CH1_E_CH2_D_CH3_E
	.section	.text.MC_EnableInput_CH1_D_CH2_E_CH3_E,"ax",%progbits
	.align	1
	.global	MC_EnableInput_CH1_D_CH2_E_CH3_E
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_EnableInput_CH1_D_CH2_E_CH3_E, %function
MC_EnableInput_CH1_D_CH2_E_CH3_E:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L110
	movs	r2, #0
	mov	r1, #1024
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #1
	mov	r1, #2048
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #1
	mov	r1, #4096
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	pop	{r4, pc}
.L111:
	.align	2
.L110:
	.word	1207961600
	.size	MC_EnableInput_CH1_D_CH2_E_CH3_E, .-MC_EnableInput_CH1_D_CH2_E_CH3_E
	.section	.text.MC_DisableInput_CH1_D_CH2_D_CH3_D,"ax",%progbits
	.align	1
	.global	MC_DisableInput_CH1_D_CH2_D_CH3_D
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_DisableInput_CH1_D_CH2_D_CH3_D, %function
MC_DisableInput_CH1_D_CH2_D_CH3_D:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L114
	movs	r2, #0
	mov	r1, #1024
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #0
	mov	r1, #2048
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	movs	r2, #0
	mov	r1, #4096
	mov	r0, r4
	bl	HAL_GPIO_WritePin
	pop	{r4, pc}
.L115:
	.align	2
.L114:
	.word	1207961600
	.size	MC_DisableInput_CH1_D_CH2_D_CH3_D, .-MC_DisableInput_CH1_D_CH2_D_CH3_D
	.section	.text.MC_Start_PWM,"ax",%progbits
	.align	1
	.global	MC_Start_PWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Start_PWM, %function
MC_Start_PWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L118
	movs	r1, #0
	mov	r0, r4
	bl	HAL_TIM_PWM_Start
	movs	r1, #4
	mov	r0, r4
	bl	HAL_TIM_PWM_Start
	movs	r1, #8
	mov	r0, r4
	bl	HAL_TIM_PWM_Start
	pop	{r4, pc}
.L119:
	.align	2
.L118:
	.word	.LANCHOR1
	.size	MC_Start_PWM, .-MC_Start_PWM
	.section	.text.MC_Stop_PWM,"ax",%progbits
	.align	1
	.global	MC_Stop_PWM
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Stop_PWM, %function
MC_Stop_PWM:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L122
	movs	r1, #0
	mov	r0, r4
	bl	HAL_TIM_PWM_Stop
	movs	r1, #4
	mov	r0, r4
	bl	HAL_TIM_PWM_Stop
	movs	r1, #8
	mov	r0, r4
	bl	HAL_TIM_PWM_Stop
	pop	{r4, pc}
.L123:
	.align	2
.L122:
	.word	.LANCHOR1
	.size	MC_Stop_PWM, .-MC_Stop_PWM
	.section	.text.MC_HF_TIMx_SetDutyCycle_CH1,"ax",%progbits
	.align	1
	.global	MC_HF_TIMx_SetDutyCycle_CH1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_HF_TIMx_SetDutyCycle_CH1, %function
MC_HF_TIMx_SetDutyCycle_CH1:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L125
	ldr	r3, [r3]
	str	r0, [r3, #52]
	bx	lr
.L126:
	.align	2
.L125:
	.word	.LANCHOR1
	.size	MC_HF_TIMx_SetDutyCycle_CH1, .-MC_HF_TIMx_SetDutyCycle_CH1
	.section	.text.MC_HF_TIMx_SetDutyCycle_CH2,"ax",%progbits
	.align	1
	.global	MC_HF_TIMx_SetDutyCycle_CH2
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_HF_TIMx_SetDutyCycle_CH2, %function
MC_HF_TIMx_SetDutyCycle_CH2:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L128
	ldr	r3, [r3]
	str	r0, [r3, #56]
	bx	lr
.L129:
	.align	2
.L128:
	.word	.LANCHOR1
	.size	MC_HF_TIMx_SetDutyCycle_CH2, .-MC_HF_TIMx_SetDutyCycle_CH2
	.section	.text.MC_HF_TIMx_SetDutyCycle_CH3,"ax",%progbits
	.align	1
	.global	MC_HF_TIMx_SetDutyCycle_CH3
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_HF_TIMx_SetDutyCycle_CH3, %function
MC_HF_TIMx_SetDutyCycle_CH3:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L131
	ldr	r3, [r3]
	str	r0, [r3, #60]
	bx	lr
.L132:
	.align	2
.L131:
	.word	.LANCHOR1
	.size	MC_HF_TIMx_SetDutyCycle_CH3, .-MC_HF_TIMx_SetDutyCycle_CH3
	.section	.text.MC_Current_Reference_Start,"ax",%progbits
	.align	1
	.global	MC_Current_Reference_Start
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Current_Reference_Start, %function
MC_Current_Reference_Start:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r0, .L135
	ldr	r3, [r0]
	movs	r1, #0
	str	r1, [r3, #52]
	bl	HAL_TIM_PWM_Start
	pop	{r3, pc}
.L136:
	.align	2
.L135:
	.word	.LANCHOR5
	.size	MC_Current_Reference_Start, .-MC_Current_Reference_Start
	.section	.text.MC_Current_Reference_Stop,"ax",%progbits
	.align	1
	.global	MC_Current_Reference_Stop
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Current_Reference_Stop, %function
MC_Current_Reference_Stop:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldr	r0, .L139
	ldr	r3, [r0]
	movs	r1, #0
	str	r1, [r3, #52]
	bl	HAL_TIM_PWM_Stop
	pop	{r3, pc}
.L140:
	.align	2
.L139:
	.word	.LANCHOR5
	.size	MC_Current_Reference_Stop, .-MC_Current_Reference_Stop
	.section	.text.MC_Current_Reference_Setvalue,"ax",%progbits
	.align	1
	.global	MC_Current_Reference_Setvalue
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Current_Reference_Setvalue, %function
MC_Current_Reference_Setvalue:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	mov	r4, r0
	mov	r1, r0
	ldr	r0, .L143
	bl	printf
	ldr	r3, .L143+4
	ldr	r3, [r3]
	ldr	r0, [r3, #44]
	mul	r0, r0, r4
	lsrs	r0, r0, #12
	str	r0, [r3, #52]
	pop	{r4, pc}
.L144:
	.align	2
.L143:
	.word	.LC16
	.word	.LANCHOR5
	.size	MC_Current_Reference_Setvalue, .-MC_Current_Reference_Setvalue
	.section	.text.NUCLEO_LED_ON,"ax",%progbits
	.align	1
	.global	NUCLEO_LED_ON
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	NUCLEO_LED_ON, %function
NUCLEO_LED_ON:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	movs	r2, #1
	movs	r1, #4
	ldr	r0, .L147
	bl	HAL_GPIO_WritePin
	pop	{r3, pc}
.L148:
	.align	2
.L147:
	.word	1207960576
	.size	NUCLEO_LED_ON, .-NUCLEO_LED_ON
	.section	.text.NUCLEO_LED_OFF,"ax",%progbits
	.align	1
	.global	NUCLEO_LED_OFF
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	NUCLEO_LED_OFF, %function
NUCLEO_LED_OFF:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	movs	r2, #0
	movs	r1, #4
	ldr	r0, .L151
	bl	HAL_GPIO_WritePin
	pop	{r3, pc}
.L152:
	.align	2
.L151:
	.word	1207960576
	.size	NUCLEO_LED_OFF, .-NUCLEO_LED_OFF
	.section	.text.MC_Nucleo_Init,"ax",%progbits
	.align	1
	.global	MC_Nucleo_Init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	MC_Nucleo_Init, %function
MC_Nucleo_Init:
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	sub	sp, sp, #48
	bl	_Z9GPIO_Initv
	bl	_Z9ADC1_Initv
	bl	_Z9TIM1_Initv
	bl	_Z9TIM2_Initv
	bl	_Z9TIM6_Initv
	bl	_Z10TIM16_Initv
	bl	_Z8DAC_Initv
	movs	r6, #1
	str	r6, [sp, #28]
	str	r6, [sp, #32]
	movs	r4, #0
	str	r4, [sp, #36]
	str	r4, [sp, #40]
	str	r4, [sp, #44]
	ldr	r5, .L155
	mov	r2, r4
	add	r1, sp, #28
	mov	r0, r5
	bl	HAL_TIM_ConfigOCrefClear
	movs	r2, #4
	add	r1, sp, #28
	mov	r0, r5
	bl	HAL_TIM_ConfigOCrefClear
	movs	r2, #8
	add	r1, sp, #28
	mov	r0, r5
	bl	HAL_TIM_ConfigOCrefClear
	ldr	r2, .L155+4
	ldr	r3, [r2, #12]
	orrs	r3, r3, r6
	str	r3, [r2, #12]
	ldr	r2, [r5]
	ldr	r3, [r2, #12]
	orr	r3, r3, #128
	str	r3, [r2, #12]
	movs	r3, #7
	str	r3, [sp, #4]
	str	r6, [sp, #8]
	str	r4, [sp, #16]
	str	r4, [sp, #12]
	str	r4, [sp, #20]
	str	r4, [sp, #24]
	ldr	r4, .L155+8
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #2
	str	r3, [sp, #4]
	movs	r5, #6
	str	r5, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #8
	str	r3, [sp, #4]
	str	r5, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #9
	str	r3, [sp, #4]
	movs	r6, #5
	str	r6, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #11
	str	r3, [sp, #4]
	str	r6, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #15
	str	r3, [sp, #4]
	str	r6, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	movs	r3, #12
	str	r3, [sp, #4]
	str	r5, [sp, #12]
	add	r1, sp, #4
	mov	r0, r4
	bl	HAL_ADC_ConfigChannel
	add	sp, sp, #48
	@ sp needed
	pop	{r4, r5, r6, pc}
.L156:
	.align	2
.L155:
	.word	.LANCHOR1
	.word	-536600576
	.word	.LANCHOR0
	.size	MC_Nucleo_Init, .-MC_Nucleo_Init
	.section	.bss._ZL4hdac,"aw",%nobits
	.align	2
	.set	.LANCHOR3,. + 0
	.type	_ZL4hdac, %object
	.size	_ZL4hdac, 20
_ZL4hdac:
	.space	20
	.section	.bss._ZL5hadc1,"aw",%nobits
	.align	2
	.set	.LANCHOR0,. + 0
	.type	_ZL5hadc1, %object
	.size	_ZL5hadc1, 84
_ZL5hadc1:
	.space	84
	.section	.bss._ZL5htim1,"aw",%nobits
	.align	2
	.set	.LANCHOR1,. + 0
	.type	_ZL5htim1, %object
	.size	_ZL5htim1, 64
_ZL5htim1:
	.space	64
	.section	.bss._ZL5htim2,"aw",%nobits
	.align	2
	.set	.LANCHOR4,. + 0
	.type	_ZL5htim2, %object
	.size	_ZL5htim2, 64
_ZL5htim2:
	.space	64
	.section	.bss._ZL5htim6,"aw",%nobits
	.align	2
	.set	.LANCHOR2,. + 0
	.type	_ZL5htim6, %object
	.size	_ZL5htim6, 64
_ZL5htim6:
	.space	64
	.section	.bss._ZL6htim16,"aw",%nobits
	.align	2
	.set	.LANCHOR5,. + 0
	.type	_ZL6htim16, %object
	.size	_ZL6htim16, 64
_ZL6htim16:
	.space	64
	.section	.rodata.MC_Current_Reference_Setvalue.str1.4,"aMS",%progbits,1
	.align	2
.LC16:
	.ascii	"MC_Current_Reference_Setvalue %d\012\000"
	.section	.rodata._Z8DAC_Initv.str1.4,"aMS",%progbits,1
	.align	2
.LC14:
	.ascii	"HAL_DAC_Init failed\012\000"
	.space	3
.LC15:
	.ascii	"HAL_DAC_ConfigChannel failed\012\000"
	.section	.rodata._Z9ADC1_Initv.str1.4,"aMS",%progbits,1
	.align	2
.LC0:
	.ascii	"HAL_ADC_Init failed\012\000"
	.space	3
.LC1:
	.ascii	"HAL_ADC_ConfigChannel failed\012\000"
	.section	.rodata._Z9TIM1_Initv.str1.4,"aMS",%progbits,1
	.align	2
.LC2:
	.ascii	"HAL_TIM_Base_Init failed\012\000"
	.space	2
.LC3:
	.ascii	"HAL_TIM_ConfigClockSource failed\012\000"
	.space	2
.LC4:
	.ascii	"HAL_TIM_PWM_Init failed\012\000"
	.space	3
.LC5:
	.ascii	"HAL_TIM_ConfigOCrefClear1 failed\012\000"
	.space	2
.LC6:
	.ascii	"HAL_TIM_ConfigOCrefClear2 failed\012\000"
	.space	2
.LC7:
	.ascii	"HAL_TIM_ConfigOCrefClear3 failed\012\000"
	.space	2
.LC8:
	.ascii	"HAL_TIMEx_MasterConfigSynchronization failed\012\000"
	.space	2
.LC9:
	.ascii	"HAL_TIM_PWM_ConfigChannel1 failed\012\000"
	.space	1
.LC10:
	.ascii	"HAL_TIM_PWM_ConfigChannel2 failed\012\000"
	.space	1
.LC11:
	.ascii	"HAL_TIM_PWM_ConfigChannel3 failed\012\000"
	.space	1
.LC12:
	.ascii	"HAL_TIMEx_ConfigBreakDeadTime failed\012\000"
	.section	.rodata._Z9TIM2_Initv.str1.4,"aMS",%progbits,1
	.align	2
.LC13:
	.ascii	"HAL_TIM_PWM_ConfigChannel failed\012\000"
	.ident	"GCC: (GNU) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]"
