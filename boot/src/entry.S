	.arm
	.section	".reset", "ax"
	.global		_start
	.global		enable_irq
	.global		disable_irq
_start:
	LDR		pc, =reset_handler
	b		.
	b		.
	b		.
	b		.
	b		.
	LDR		pc, =irq_handler
	b		.

reset_handler:
	mov		sp, #0x8000 @set svc sp

	mrs		r4, cpsr
	bic		r4, r4, #0xf
	orr		r4, r4, #0x2
	msr		cpsr_c, r4
	mov		sp, #0x7000 @set irq sp
	mrs		r4, cpsr
	bic		r4, r4, #0xf
	orr		r4, r4, #0x3
	msr		cpsr_c, r4

	mrc		p15, 0, r4, c1, c0, 0
	mov		r5, #1
	bic		r4, r4, r5, lsl #13
	mcr		p15, 0, r4, c1, c0, 0

	b		Main

enable_irq:
	stmfd		sp!, {r4, lr}
	mrs		r4, cpsr
	bic		r4, r4, #0x80
	msr		cpsr_c, r4
	ldmfd		sp!, {r4, pc}

disable_irq:
	stmfd		sp!, {r4, lr}
	mrs		r4, cpsr
	orr		r4, r4, #0x80
	msr		cpsr_c, r4
	ldmfd		sp!, {r4, pc}

irq_handler:
	subs		lr, lr, #4
	stmfd		sp!, {r0-r12, lr}

	ldmfd		sp!, {r0-r12, pc}^

	.end

