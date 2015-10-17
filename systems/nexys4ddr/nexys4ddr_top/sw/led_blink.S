#define GPIO_LED_BASE 0x91002000
#define GPIO_LED_DIR  4
#define GPIO_LED_DATA 0

.globl main
        .org 0x0100
main:
	l.movhi r0, 0

	l.movhi r5,     0xffff
	l.ori   r5, r5, 0xffff
        l.or    r6, r5, r0

	l.movhi r4,                  hi(GPIO_LED_BASE)
	l.ori   r4,              r4, lo(GPIO_LED_BASE)
        l.sw    GPIO_LED_DIR (r4), r5
loop:   l.sw    GPIO_LED_DATA(r4), r6
        l.xor   r6, r6, r5
	l.movhi r7, 0x10 //loop count
t1:
	l.addi 	r7, r7, -1
	l.sfeq 	r7, r0
	l.bnf 	t1
	l.nop

        l.j     loop
        l.nop
