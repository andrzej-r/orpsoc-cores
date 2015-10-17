.globl main

main:
	#Clear r0
	l.movhi r0, 0

        #set r4 to 0xdeadbeef
	l.movhi r4,   0xdead
	l.ori   r4,r4,0xbeef

lp0:    # memory access (write) test
        l.j lp0
        l.nop
        l.sw 0x0000(r0),r4
        l.sw 0x0004(r0),r4
        l.sw 0x0008(r0),r4
        l.sw 0x000c(r0),r4
        l.sw 0x0010(r0),r4
        l.sw 0x0014(r0),r4
        l.sw 0x0018(r0),r4
        l.sw 0x001c(r0),r4
        l.sw 0x0020(r0),r4
        l.sw 0x0024(r0),r4
        l.sw 0x0028(r0),r4
        l.sw 0x002c(r0),r4
        l.lwz r3,0x0000(r0)
        #l.lwz r3,0x0001(r0) # unaligned
        l.lwz r3,0x0004(r0)
        l.lwz r3,0x0008(r0)
        l.lwz r3,0x000c(r0)
        l.lwz r3,0x0040(r0)
        l.lwz r3,0x0044(r0)
        l.lwz r3,0x0048(r0)
        l.lwz r3,0x004c(r0)
        l.lwz r3,0x0040(r0)
        l.lwz r3,0x0044(r0)
        l.lwz r3,0x0048(r0)
        l.lwz r3,0x004c(r0)
        l.lwz r3,0x0000(r0)
        l.lwz r3,0x0004(r0)
        l.lwz r3,0x0008(r0)
        l.lwz r3,0x000c(r0)
        #l.lhz r3,0x0051(r0)
        #l.lhz r3,0x0055(r0)
        #l.lhz r3,0x0059(r0)
        #l.lhz r3,0x005d(r0)
        #l.lhz r3,0x0053(r0)
        #l.lhz r3,0x0057(r0)
        #l.lhz r3,0x005b(r0)
        #l.lhz r3,0x005f(r0)
        #l.j lp0
        #l.nop

        # invalidate icache, addresses f0000000-f0000100
        l.movhi r5,   0xf000
        l.ori   r5,r5,0x0400
        l.movhi r4,   0xf000
invalidate_i:
        l.mtspr r0,r4,0x2002
        l.addi  r4,r4,16
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

        # enable the data cache
        l.mfspr r4,r0,17 # SR register, group 0, reg 17
        l.ori r4,r4,0x08 # set bit 3 (DCE - instruction set enable)
        l.mtspr r0,r4,17

        # invalidate dcache, addresses 00000000-00000200
        l.movhi r5,0x0000
        l.ori   r5,r5,0x200
        l.movhi r4,0x0000
invalidate_d:
        l.mtspr r0,r4,0x1803
        l.addi r4,r4,16
        l.sfltu r4,r5 # write address to invalidate to DCBIR
        l.bf invalidate_d
        l.nop

        #set r4 to 0xdeadbeef
	l.movhi r4,   0xdead
	l.ori   r4,r4,0xbeef

lp1:    # memory access (write) test
	l.movhi r4,   0xdead
	l.ori   r4,r4,0xbeef
        l.sw 0x0000(r0),r4
        l.sw 0x0004(r0),r4
        l.sw 0x0008(r0),r4
        l.sw 0x000c(r0),r4
        l.sw 0x0010(r0),r4
        l.sw 0x0014(r0),r4
        l.sw 0x0018(r0),r4
        l.sw 0x001c(r0),r4
        l.sw 0x0020(r0),r4
        l.sw 0x0024(r0),r4
        l.sw 0x0028(r0),r4
        l.sw 0x002c(r0),r4
        l.lwz r3,0x0000(r0)
        l.lwz r3,0x0004(r0)
        l.lwz r3,0x0008(r0)
        l.lwz r3,0x000c(r0)
        l.lwz r3,0x0040(r0)
        l.lwz r3,0x0044(r0)
        l.lwz r3,0x0048(r0)
        l.lwz r3,0x004c(r0)
        l.lwz r3,0x0040(r0)
        l.lwz r3,0x0044(r0)
        l.lwz r3,0x0048(r0)
        l.lwz r3,0x004c(r0)
        l.lwz r3,0x0000(r0)
        l.lwz r3,0x0004(r0)
        l.lwz r3,0x0008(r0)
        l.lwz r3,0x000c(r0)
        #l.lhz r3,0x0051(r0)
        #l.lhz r3,0x0055(r0)
        #l.lhz r3,0x0059(r0)
        #l.lhz r3,0x005d(r0)
        #l.lhz r3,0x0053(r0)
        #l.lhz r3,0x0057(r0)
        #l.lhz r3,0x005b(r0)
        #l.lhz r3,0x005f(r0)
	l.movhi r4,   0xbeef
	l.ori   r4,r4,0xdead
        l.j lp1
        l.nop

