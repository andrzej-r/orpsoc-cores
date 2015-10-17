boot_init:
	l.movhi r0, 0
	l.movhi r1, 0
	l.movhi r2, 0
	l.movhi r3, 0
	l.movhi r4, 0
	l.movhi r5, 0

	l.movhi r6,                  hi(BOOTROM_ADDR)
	l.ori   r6,              r6, lo(BOOTROM_ADDR)

        l.jr    r6
        l.nop
