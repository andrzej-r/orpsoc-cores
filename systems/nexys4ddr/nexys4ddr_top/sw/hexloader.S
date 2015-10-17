#define UART_BASE 0x90000000

#r3 = byte
#r4 = tmp
#r5 = tmp
#r6 = BASEADDR
#r7 = ADDRESS
#r8 = tmp (in parseByte)
#r9 = Return address	
#r10 = tmp
#r11 = CNT
#r12 r9 shadowed	
#r13 = r12 shadowed
#r14 tmp (in parseHexChar)
.globl main
#	.org 0x100
main:	
	l.movhi r0, 0
	l.jal uartInit
	l.nop
1:	l.jal findRecord #Find next record delimiter
	l.nop
	#clearCRC

	#Get Byte count field and store to r11
	l.jal parseHexChar
	l.nop
	l.nop 0x2
	l.nop 0x1
	l.add r11, r3, r0
	l.nop

	#Get Address and store to r7
	l.jal parseByte
	l.nop

	l.slli r7, r3, 8
	l.jal parseHexChar
	l.nop

	l.add r7, r7, r3

	l.or r3, r7, r0

	#l.j 1b
	l.nop
	l.nop 0x1
	l.nop 0x0
	
err:
	l.ori r3, r0, 0xdead
	l.nop 0x2
	l.nop 0x1
	l.nop 0x0
#r4 = UART Base address
#r5 = Register value
uartInit:
	l.movhi r4, hi(UART_BASE)
	l.ori r4, r4, lo(UART_BASE)

	# Set LCR = LCR_DLA
	l.addi r5, r0, 0x80
	l.sb 3(r4), r5

	# Set divisor 0x1B = 50MHz/(115200*16)
	# Set divisor 0x24 = 66.6MHz/(115200*16)
	l.addi r5, r0, 0x24
	l.sb 0(r4), r5
	l.addi r5, r0, 0x0
	l.sb 1(r4), r5

	# Set line control register:
	# - 8 bits per character
	# - 1 stop bit
	# - No parity
	# - Break disabled
	# - Disallow access to divisor latch
	# LCR = LCR_BPS_8
	l.addi r5, r0, 0x3
	l.sb 3(r4), r5

	# Reset FIFOs and set trigger level to 14 bytes
	# REG8(FCR) = FCR_CLRRECV | FCR_CLRTMIT | FCR_TRIG_14;
	l.addi r5, r0, 0xc3
	l.sb 2(r4), r5

	# Disable all interrupts
	# REG8(IER) = 0;
	l.addi r5, r0, 0x0
	l.sb 1(r4), r5
	l.jr r9
	l.nop
	
#r8 = baseaddr
uartRead:
	l.movhi r8, hi(UART_BASE)
	l.ori r8, r8, lo(UART_BASE)

1:	l.lbz r3, 5(r8) #Read LSR
	l.andi r3, r3, 0x1 #Mask out Receiver ready
	l.sfeqi r3, 0
	l.bf 1b
	l.nop
	l.lbz r3, 0(r8) #Read word from UART to r3
	l.jr r9
	l.nop
	
#r4 = tmp
#r5 = tmp
setBaseAddress:
	l.jal parseByte
	l.slli r4, r3, 16
	l.jal parseByte
	l.slli r5, r3, 24
	l.or r6, r4, r5
	l.jr r9
	l.nop

#r4 = offset
#r5 = counter
#r10 = address
writeData:
	l.add r4, r6, r7 #Set start address
	l.movhi r5, 0    #Clear counter
1:	l.jal parseByte
	l.add r10, r4, r5 #address = start address + counter
	l.addi r5, r5, 1 #Increase counter
	l.sb 0(r10), r3
	l.sfeq r5, r10
	l.bnf 1b
	l.nop
	

findRecord:
	l.or r12, r9, r0 #Save return address to r12
1:	l.jal uartRead
	l.nop
	l.sfeqi r3, ':'
	l.bnf 1b
	l.nop
	l.jr r12
	l.nop

#Valid chars are 0-9 (0x30-0x39) and A-F (0x41-0x46)
parseByte:
	l.or r12, r9, r0 #Save return address to r12
	
	l.jal uartRead
	l.nop
	#l.nop 0x2

	#Error if r3 < 0
	l.sfleui r3, '0'
	l.bf err
	l.nop

	#OK

	#Next check if r3 > 9
	l.sfltui r3, '9'
	l.bnf 1f
	l.nop

	#If 0-9, subtract '0' and return
	l.addi r3, r3, -48
	l.jr r12
	l.nop

	#r3 is > 9 at this point
	#Error if r3 < A
1:	l.sfleui r3, 'A'
#	l.nop 0x1
	l.bf err
	l.nop


	#Error if r3 > F
	l.sfleui r3, 'F'
	l.bnf err
	l.nop


	#If A-F subtract ('A'-10) and return
	l.addi r3, r3, -55
	l.jr r12
	l.nop

parseHexChar:
	l.or r13, r9, r0 #Save return address to r13
	l.jal parseByte
	l.nop

	l.slli r14, r3, 4

	l.jal parseByte
	l.nop
	l.or r3, r14, r3
	l.jr r13
	l.nop
