#define MEM_START 0x00000400
#define MEM_END   0x02000000
#define MEM_DST_START 0x02000400
#ifndef BOOTROM_ADDR
#define BOOTROM_ADDR 0x00000100
#endif

#define GPIO_LED_BASE 0x91002000
#define GPIO_LED_DIR  4
#define GPIO_LED_DATA 0

#define SSEG_CTRL_BASE 0x92000000
#define SSEG_CTRL_EN   0
#define SSEG_CTRL_DIG0 0x20
#define SSEG_CTRL_DIG1 0x24
#define SSEG_CTRL_DIG2 0x28
#define SSEG_CTRL_DIG3 0x2c
#define SSEG_CTRL_DIG4 0x30
#define SSEG_CTRL_DIG5 0x34
#define SSEG_CTRL_DIG6 0x38
#define SSEG_CTRL_DIG7 0x3c

#define ENABLE_ICACHE
#define ENABLE_DCACHE

        .global boot_init
        .org BOOTROM_ADDR
boot_init:
        l.movhi r0, 0

#ifdef ENABLE_ICACHE
        # invalidate icache, addresses 00000100-00000300
        l.movhi r5,   0x0000
        l.ori   r5,r5,0x0300
        l.movhi r4,   0x0000
        l.ori   r4,r4,0x0100
invalidate_i:
        l.mtspr r0,r4,0x2002
        l.addi  r4,r4,32
        l.sfltu r4,r5        # write address to ICBIR
        l.bf invalidate_i
        l.nop

        # enable the instruction cache
        l.mfspr r4,r0,17 # SR register, group 0, reg 17
        l.ori r4,r4,0x10 # set bit 4 (ICE - instruction set enable)
        l.mtspr r0,r4,17

        # enable all (2) instruction cache ways
        #l.movhi r4, 0
        #l.ori r4,r4,0x3
        #l.mtspr r0,r4,0x2000  # ICCR SPR, group 4, reg 0
#endif
#ifdef ENABLE_DCACHE
        # invalidate dcache, addresses 00000400-00000600
        l.movhi r5,0x0000
        l.ori   r5,r5,0x0600
        l.movhi r4,0x0000
        l.ori   r4,r4,0x0400
invalidate_d:
        l.mtspr r0,r4,0x1803
        l.addi r4,r4,32
        l.sfltu r4,r5 # write address to invalidate to DCBIR
        l.bf invalidate_d
        l.nop

        # enable the data cache
        l.mfspr r4,r0,17 # SR register, group 0, reg 17
        l.ori r4,r4,0x08 # set bit 3 (DCE - instruction set enable)
        l.mtspr r0,r4,17
#endif

        l.movhi r1, hi(hexDigits)
	l.ori   r1, r1, lo(hexDigits)
	l.movhi r2, hi(MEM_START)
	l.ori   r2, r2, lo(MEM_START)
	l.movhi r3, hi(MEM_END)
	l.ori   r3, r3, lo(MEM_END)
	l.movhi r31, hi(MEM_DST_START)
	l.ori   r31, r31, lo(MEM_DST_START)

	l.movhi r5,     0xffff
	l.ori   r5, r5, 0xffff
	l.movhi r6,  hi(GPIO_LED_BASE)
	l.ori   r6,  r6, lo(GPIO_LED_BASE)
        l.sw    GPIO_LED_DIR (r6), r5

	l.movhi r7,  hi(SSEG_CTRL_BASE)
	l.ori   r7,  r7, lo(SSEG_CTRL_BASE)
	l.movhi r5,     0x0
	l.ori   r5, r5, 0x1
        l.sw    SSEG_CTRL_EN (r7), r5

1:
        l.lwz   r16,0x00(r2)
        l.lwz   r17,0x04(r2)
        l.lwz   r18,0x08(r2)
        l.lwz   r19,0x0c(r2)
        l.lwz   r20,0x10(r2)
        l.lwz   r21,0x14(r2)
        l.lwz   r22,0x18(r2)
        l.lwz   r23,0x1c(r2)
        l.sw    0x00(r31),r16
        l.sw    0x04(r31),r17
        l.sw    0x08(r31),r18
        l.sw    0x0c(r31),r19
        l.sw    0x10(r31),r20
        l.sw    0x14(r31),r21
        l.sw    0x18(r31),r22
        l.sw    0x1c(r31),r23
        l.andi  r5,r2,0xffff
        l.sfeq  r5,r0
        l.bnf   2f
        l.nop
        l.jal   displayReg
        l.nop
2:      l.addi  r2,r2,0x20
        l.addi  r31,r31,0x20
        l.sfeq  r2,r3
        l.bnf   1b
        l.nop
        l.jal   displayReg
        l.nop

1:      l.jal   1b
        l.nop

#r1 hexDigits
#r2 reg to display
#r4 tmp
#r5 tmp
#r7 SSEG_CTRL_BASE
displayReg:
        l.or r12,r9,r0
        l.srli r4,r2,0
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG7
        l.srli r4,r2,4
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG6
        l.srli r4,r2,8
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG5
        l.srli r4,r2,12
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG4
        l.srli r4,r2,16
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG3
        l.srli r4,r2,20
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG2
        l.srli r4,r2,24
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG1
        l.srli r4,r2,28
        l.jal displayDigit
        l.addi r5,r7,SSEG_CTRL_DIG0
        l.jr r12
        l.nop

#r4 hexDigit
#r5 digit addr
displayDigit:
        l.andi r4,r4,0x0f
        l.add  r4,r4,r1
        l.lbz  r4,0(r4)
        l.sw   0(r5),r4
        l.jr   r9
        l.nop

hexDigits:
        .byte 0x3f
        .byte 0x06
        .byte 0x5b
        .byte 0x4f
        .byte 0x66
        .byte 0x6d
        .byte 0x7d
        .byte 0x07
        .byte 0x7f
        .byte 0x6f
        .byte 0x77
        .byte 0x7c
        .byte 0x39
        .byte 0x5e
        .byte 0x79
        .byte 0x71

