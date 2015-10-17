#define GPIO_SW_BASE 0x91000000
#define GPIO_SW_DIR  4
#define GPIO_SW_DATA 0
#define DDR2_CFG_BASE 0x96000000
#ifndef BOOTROM_ADDR
#define BOOTROM_ADDR 0x00000100
#endif

boot_init:
	l.movhi r0, 0
	l.movhi r2, 0
	l.movhi r5, 0

	l.movhi r6,                  hi(BOOTROM_ADDR)
	l.ori   r6,              r6, lo(BOOTROM_ADDR)

	l.movhi r7,                  0x0000
	l.movhi r2,                  0x0102
	l.ori   r2,              r2, 0x0304
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r2,                  0x1819
	l.ori   r2,              r2, 0x1a1b
        l.sw    0x0004(r7),      r2
        l.lwz   r1,                  0x0004(r7)

	l.movhi r2,                  0x2829
	l.ori   r2,              r2, 0x2a2b
        l.sw    0x0008(r7),      r2
        l.lwz   r1,                  0x0008(r7)

	l.movhi r2,                  0x3839
	l.ori   r2,              r2, 0x3a3b
        l.sw    0x000c(r7),      r2
        l.lwz   r1,                  0x000c(r7)

	l.movhi r2,                  0x4849
	l.ori   r2,              r2, 0x4a4b
        l.sw    0x0010(r7),      r2
        l.lwz   r1,                  0x0010(r7)

	l.movhi r2,                  0x5859
	l.ori   r2,              r2, 0x5a5b
        l.sw    0x0014(r7),      r2
        l.lwz   r1,                  0x0014(r7)

	l.movhi r2,                  0x6869
	l.ori   r2,              r2, 0x6a6b
        l.sw    0x0018(r7),      r2
        l.lwz   r1,                  0x0018(r7)

	l.movhi r2,                  0x7879
	l.ori   r2,              r2, 0x7a7b
        l.sw    0x001c(r7),      r2
        l.lwz   r1,                  0x001c(r7)

	l.movhi r2,                  0x8889
	l.ori   r2,              r2, 0x8a8b
        l.sw    0x0020(r7),      r2
        l.lwz   r1,                  0x0020(r7)

	l.movhi r2,                  0x9899
	l.ori   r2,              r2, 0x9a9b
        l.sw    0x0024(r7),      r2
        l.lwz   r1,                  0x0024(r7)

	l.movhi r2,                  0xa8a9
	l.ori   r2,              r2, 0xaaab
        l.sw    0x0028(r7),      r2
        l.lwz   r1,                  0x0028(r7)

	l.movhi r2,                  0xb8b9
	l.ori   r2,              r2, 0xbabb
        l.sw    0x002c(r7),      r2
        l.lwz   r1,                  0x002c(r7)

	l.movhi r7,                  0x0100
	l.movhi r2,                  0x1112
	l.ori   r2,              r2, 0x1314
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0200
	l.movhi r2,                  0x2122
	l.ori   r2,              r2, 0x2324
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0300
	l.movhi r2,                  0x3132
	l.ori   r2,              r2, 0x3334
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0400
	l.movhi r2,                  0x4142
	l.ori   r2,              r2, 0x4344
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0500
	l.movhi r2,                  0x5152
	l.ori   r2,              r2, 0x5354
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0600
	l.movhi r2,                  0x6162
	l.ori   r2,              r2, 0x6364
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

	l.movhi r7,                  0x0700
	l.movhi r2,                  0x7172
	l.ori   r2,              r2, 0x7374
        l.sw    0x0000(r7),      r2
        l.lwz   r1,                  0x0000(r7)

        
	l.movhi r7,                  hi(DDR2_CFG_BASE)
	l.ori   r7,              r7, lo(DDR2_CFG_BASE)
1:      //l.lwz   r2,                  0x0000(r7)
        //l.andi  r2,r2,0x0001
        //l.sfnei r2,0x0001
        //l.bf    1b
        //l.nop

	l.movhi r2, 0
	l.ori   r2,              r2, 0xffff
        l.sw    0x0008(r7),      r2
	l.movhi r2, 0

        //loop until SW13 is switched on, then jumpt to BOOT_ADDR
	l.movhi r4,                  hi(GPIO_SW_BASE)
	l.ori   r4,              r4, lo(GPIO_SW_BASE)
	l.sw    GPIO_SW_DIR(r4), r0
1:      l.lwz   r5,                  GPIO_SW_DATA(r4)
        l.andi  r5,              r5, 0x2000
        l.sfeqi r5,              0
        l.bf    1b
        l.nop
        l.jr    r6
        l.nop
