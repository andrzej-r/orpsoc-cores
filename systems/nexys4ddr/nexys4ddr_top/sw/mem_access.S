#define OFFS 0x1000
#ifndef BOOTROM_ADDR
#define BOOTROM_ADDR 0x00000100
#endif

#define GPIO_LED_BASE 0x91002000
#define GPIO_LED_DIR  4
#define GPIO_LED_DATA 0

        .globl boot_init
        .org BOOTROM_ADDR
boot_init:
	l.movhi r0, 0
	l.movhi r2,   hi(OFFS)
        l.ori   r2,r2,lo(OFFS)

	l.movhi r5,     0xffff
	l.ori   r5, r5, 0xffff

	l.movhi r6,  hi(GPIO_LED_BASE)
	l.ori   r6,  r6, lo(GPIO_LED_BASE)
        l.sh    GPIO_LED_DIR (r6), r5

	l.movhi r3,   0x0001
        l.ori   r3,r3,0x0203
	l.sw    0x00(r2),r3

	l.movhi r3,   0x0405
        l.ori   r3,r3,0x0607
	l.sw    0x04(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0008
	l.sb    0x08(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0009
	l.sb    0x09(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0a0b
	l.sh    0x0a(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0c0d
	l.sh    0x0c(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x000e
	l.sb    0x0e(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x000f
	l.sb    0x0f(r2),r3

	l.movhi r3,   0x1011
        l.ori   r3,r3,0x1213
	l.sw    0x10(r2),r3

	l.movhi r3,   0x1415
        l.ori   r3,r3,0x1617
	l.sw    0x14(r2),r3

	l.movhi r3,   0x1819
        l.ori   r3,r3,0x1a1b
	l.sw    0x18(r2),r3

	l.movhi r3,   0x1c1d
        l.ori   r3,r3,0x1e1f
	l.sw    0x1c(r2),r3

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0000
        l.lbz   r4,0x00(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0001
        l.lbz   r4,0x01(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0203
        l.lhz   r4,0x02(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0405
        l.lhz   r4,0x04(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0006
        l.lbz   r4,0x06(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0000
        l.ori   r3,r3,0x0007
        l.lbz   r4,0x07(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0809
        l.ori   r3,r3,0x0a0b
        l.lwz   r4,0x08(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x0c0d
        l.ori   r3,r3,0x0e0f
        l.lwz   r4,0x0c(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x1011
        l.ori   r3,r3,0x1213
        l.lwz   r4,0x10(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x1415
        l.ori   r3,r3,0x1617
        l.lwz   r4,0x14(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x1819
        l.ori   r3,r3,0x1a1b
        l.lwz   r4,0x18(r2)
        l.sfeq  r3,r4
        l.bnf   error

	l.movhi r3,   0x1c1d
        l.ori   r3,r3,0x1e1f
        l.lwz   r4,0x1c(r2)
        l.sfeq  r3,r4
        l.bnf   error

finish_ok:
	l.movhi r5,     0xffff
	l.ori   r5, r5, 0xffff
        l.sh    GPIO_LED_DATA (r6), r5
        l.j     finish_ok
        l.nop

error:
	l.movhi r5,     0xdead
	l.ori   r5, r5, 0xdead
        l.sh    GPIO_LED_DATA (r6), r5
        l.j     error
        l.nop
