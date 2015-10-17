#define OFFS1 0x00001000
#define OFFS2 0x04001000
#ifndef BOOTROM_ADDR
#define BOOTROM_ADDR 0x00000100
#endif
#define XADC   0x95000000
#define SDRAMC 0x96000000

        .globl boot_init
        .org BOOTROM_ADDR
boot_init:
	l.movhi r0, 0
	l.movhi r2,   hi(OFFS1)
        l.ori   r2,r2,lo(OFFS1)
	l.movhi r4,   hi(OFFS2)
        l.ori   r4,r4,lo(OFFS2)

        l.movhi r3,   hi(XADC)
        l.ori   r3,r3,lo(XADC)
        l.lwz   r5,0x200(r3) // read temperature
        l.srli  r5,r5,4 // shift 4b right
        l.movhi r3,   hi(SDRAMC)
        l.ori   r3,r3,lo(SDRAMC)
        l.sw    0x4(r3),r5 // write temperature value to SDRAM ctrl

	l.movhi r3,   0x0001
        l.ori   r3,r3,0x0203
	l.sw    0x00(r2),r3

	l.movhi r3,   0x0405
        l.ori   r3,r3,0x0607
	l.sw    0x04(r2),r3

	l.movhi r3,   0x0809
        l.ori   r3,r3,0x0a0b
	l.sw    0x08(r2),r3

	l.movhi r3,   0x0c0d
        l.ori   r3,r3,0x0e0f
	l.sw    0x0c(r2),r3

	l.movhi r3,   0x1011
        l.ori   r3,r3,0x1213
	l.sw    0x00(r4),r3

	l.movhi r3,   0x1415
        l.ori   r3,r3,0x1617
	l.sw    0x04(r4),r3

	l.movhi r3,   0x1819
        l.ori   r3,r3,0x1a1b
	l.sw    0x08(r4),r3

	l.movhi r3,   0x1c1d
        l.ori   r3,r3,0x1e1f
	l.sw    0x0c(r4),r3

        l.lwz   r7,0x0(r2)
        l.lwz   r7,0x4(r2)
        l.lwz   r7,0x8(r2)
        l.lwz   r7,0xc(r2)
        l.lwz   r7,0x0(r4)
        l.lwz   r7,0x4(r4)
        l.lwz   r7,0x8(r4)
        l.lwz   r7,0xc(r4)
        l.nop   1
