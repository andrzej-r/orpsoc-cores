
hello.elf:     file format elf32-or1k


Disassembly of section .vectors:

00000000 <_or1k_reset-0x100>:
	...

00000100 <_or1k_reset>:
     100:	18 00 00 00 	l.movhi r0,0x0
     104:	18 20 00 00 	l.movhi r1,0x0
     108:	18 40 00 00 	l.movhi r2,0x0
     10c:	18 60 00 00 	l.movhi r3,0x0
     110:	18 80 00 00 	l.movhi r4,0x0
     114:	18 a0 00 00 	l.movhi r5,0x0
     118:	18 c0 00 00 	l.movhi r6,0x0
     11c:	18 e0 00 00 	l.movhi r7,0x0
     120:	19 00 00 00 	l.movhi r8,0x0
     124:	19 20 00 00 	l.movhi r9,0x0
     128:	19 40 00 00 	l.movhi r10,0x0
     12c:	19 60 00 00 	l.movhi r11,0x0
     130:	19 80 00 00 	l.movhi r12,0x0
     134:	19 a0 00 00 	l.movhi r13,0x0
     138:	19 c0 00 00 	l.movhi r14,0x0
     13c:	19 e0 00 00 	l.movhi r15,0x0
     140:	1a 00 00 00 	l.movhi r16,0x0
     144:	1a 20 00 00 	l.movhi r17,0x0
     148:	1a 40 00 00 	l.movhi r18,0x0
     14c:	1a 60 00 00 	l.movhi r19,0x0
     150:	1a 80 00 00 	l.movhi r20,0x0
     154:	1a a0 00 00 	l.movhi r21,0x0
     158:	1a c0 00 00 	l.movhi r22,0x0
     15c:	1a e0 00 00 	l.movhi r23,0x0
     160:	1b 00 00 00 	l.movhi r24,0x0
     164:	1b 20 00 00 	l.movhi r25,0x0
     168:	1b 40 00 00 	l.movhi r26,0x0
     16c:	1b 60 00 00 	l.movhi r27,0x0
     170:	1b 80 00 00 	l.movhi r28,0x0
     174:	1b a0 00 00 	l.movhi r29,0x0
     178:	1b c0 00 00 	l.movhi r30,0x0
     17c:	1b e0 00 00 	l.movhi r31,0x0
     180:	a8 20 00 01 	l.ori r1,r0,0x1
     184:	c0 00 08 11 	l.mtspr r0,r1,0x11
     188:	c1 40 00 00 	l.mtspr r0,r0,0x5000
     18c:	18 80 00 00 	l.movhi r4,0x0
     190:	a8 84 20 30 	l.ori r4,r4,0x2030
     194:	44 00 20 00 	l.jr r4
     198:	15 00 00 00 	l.nop 0x0
	...
     200:	d4 00 08 04 	l.sw 4(r0),r1
     204:	18 20 00 00 	l.movhi r1,0x0
     208:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     20c:	84 21 00 00 	l.lwz r1,0(r1)
     210:	e4 01 00 00 	l.sfeq r1,r0
     214:	0c 00 00 06 	l.bnf 22c <_or1k_reset+0x12c>
     218:	15 00 00 00 	l.nop 0x0
     21c:	18 20 00 00 	l.movhi r1,0x0
     220:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     224:	00 00 00 04 	l.j 234 <_or1k_reset+0x134>
     228:	84 21 00 00 	l.lwz r1,0(r1)
     22c:	84 20 00 04 	l.lwz r1,4(r0)
     230:	9c 21 ff 80 	l.addi r1,r1,-128
     234:	9c 21 ff 78 	l.addi r1,r1,-136
     238:	d4 01 18 0c 	l.sw 12(r1),r3
     23c:	84 60 00 04 	l.lwz r3,4(r0)
     240:	d4 01 18 04 	l.sw 4(r1),r3
     244:	d4 01 20 10 	l.sw 16(r1),r4
     248:	18 60 00 00 	l.movhi r3,0x0
     24c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     250:	84 83 00 00 	l.lwz r4,0(r3)
     254:	9c 84 00 01 	l.addi r4,r4,1
     258:	d4 03 20 00 	l.sw 0(r3),r4
     25c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     260:	00 00 0a d2 	l.j 2da8 <_or1k_exception_handler>
     264:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     300:	d4 00 08 04 	l.sw 4(r0),r1
     304:	18 20 00 00 	l.movhi r1,0x0
     308:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     30c:	84 21 00 00 	l.lwz r1,0(r1)
     310:	e4 01 00 00 	l.sfeq r1,r0
     314:	0c 00 00 06 	l.bnf 32c <_or1k_reset+0x22c>
     318:	15 00 00 00 	l.nop 0x0
     31c:	18 20 00 00 	l.movhi r1,0x0
     320:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     324:	00 00 00 04 	l.j 334 <_or1k_reset+0x234>
     328:	84 21 00 00 	l.lwz r1,0(r1)
     32c:	84 20 00 04 	l.lwz r1,4(r0)
     330:	9c 21 ff 80 	l.addi r1,r1,-128
     334:	9c 21 ff 78 	l.addi r1,r1,-136
     338:	d4 01 18 0c 	l.sw 12(r1),r3
     33c:	84 60 00 04 	l.lwz r3,4(r0)
     340:	d4 01 18 04 	l.sw 4(r1),r3
     344:	d4 01 20 10 	l.sw 16(r1),r4
     348:	18 60 00 00 	l.movhi r3,0x0
     34c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     350:	84 83 00 00 	l.lwz r4,0(r3)
     354:	9c 84 00 01 	l.addi r4,r4,1
     358:	d4 03 20 00 	l.sw 0(r3),r4
     35c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     360:	00 00 0a 92 	l.j 2da8 <_or1k_exception_handler>
     364:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     400:	d4 00 08 04 	l.sw 4(r0),r1
     404:	18 20 00 00 	l.movhi r1,0x0
     408:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     40c:	84 21 00 00 	l.lwz r1,0(r1)
     410:	e4 01 00 00 	l.sfeq r1,r0
     414:	0c 00 00 06 	l.bnf 42c <_or1k_reset+0x32c>
     418:	15 00 00 00 	l.nop 0x0
     41c:	18 20 00 00 	l.movhi r1,0x0
     420:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     424:	00 00 00 04 	l.j 434 <_or1k_reset+0x334>
     428:	84 21 00 00 	l.lwz r1,0(r1)
     42c:	84 20 00 04 	l.lwz r1,4(r0)
     430:	9c 21 ff 80 	l.addi r1,r1,-128
     434:	9c 21 ff 78 	l.addi r1,r1,-136
     438:	d4 01 18 0c 	l.sw 12(r1),r3
     43c:	84 60 00 04 	l.lwz r3,4(r0)
     440:	d4 01 18 04 	l.sw 4(r1),r3
     444:	d4 01 20 10 	l.sw 16(r1),r4
     448:	18 60 00 00 	l.movhi r3,0x0
     44c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     450:	84 83 00 00 	l.lwz r4,0(r3)
     454:	9c 84 00 01 	l.addi r4,r4,1
     458:	d4 03 20 00 	l.sw 0(r3),r4
     45c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     460:	00 00 0a 52 	l.j 2da8 <_or1k_exception_handler>
     464:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     500:	d4 00 08 04 	l.sw 4(r0),r1
     504:	18 20 00 00 	l.movhi r1,0x0
     508:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     50c:	84 21 00 00 	l.lwz r1,0(r1)
     510:	e4 01 00 00 	l.sfeq r1,r0
     514:	0c 00 00 06 	l.bnf 52c <_or1k_reset+0x42c>
     518:	15 00 00 00 	l.nop 0x0
     51c:	18 20 00 00 	l.movhi r1,0x0
     520:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     524:	00 00 00 04 	l.j 534 <_or1k_reset+0x434>
     528:	84 21 00 00 	l.lwz r1,0(r1)
     52c:	84 20 00 04 	l.lwz r1,4(r0)
     530:	9c 21 ff 80 	l.addi r1,r1,-128
     534:	9c 21 ff 78 	l.addi r1,r1,-136
     538:	d4 01 18 0c 	l.sw 12(r1),r3
     53c:	84 60 00 04 	l.lwz r3,4(r0)
     540:	d4 01 18 04 	l.sw 4(r1),r3
     544:	d4 01 20 10 	l.sw 16(r1),r4
     548:	18 60 00 00 	l.movhi r3,0x0
     54c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     550:	84 83 00 00 	l.lwz r4,0(r3)
     554:	9c 84 00 01 	l.addi r4,r4,1
     558:	d4 03 20 00 	l.sw 0(r3),r4
     55c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     560:	00 00 0a 12 	l.j 2da8 <_or1k_exception_handler>
     564:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     600:	d4 00 08 04 	l.sw 4(r0),r1
     604:	18 20 00 00 	l.movhi r1,0x0
     608:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     60c:	84 21 00 00 	l.lwz r1,0(r1)
     610:	e4 01 00 00 	l.sfeq r1,r0
     614:	0c 00 00 06 	l.bnf 62c <_or1k_reset+0x52c>
     618:	15 00 00 00 	l.nop 0x0
     61c:	18 20 00 00 	l.movhi r1,0x0
     620:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     624:	00 00 00 04 	l.j 634 <_or1k_reset+0x534>
     628:	84 21 00 00 	l.lwz r1,0(r1)
     62c:	84 20 00 04 	l.lwz r1,4(r0)
     630:	9c 21 ff 80 	l.addi r1,r1,-128
     634:	9c 21 ff 78 	l.addi r1,r1,-136
     638:	d4 01 18 0c 	l.sw 12(r1),r3
     63c:	84 60 00 04 	l.lwz r3,4(r0)
     640:	d4 01 18 04 	l.sw 4(r1),r3
     644:	d4 01 20 10 	l.sw 16(r1),r4
     648:	18 60 00 00 	l.movhi r3,0x0
     64c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     650:	84 83 00 00 	l.lwz r4,0(r3)
     654:	9c 84 00 01 	l.addi r4,r4,1
     658:	d4 03 20 00 	l.sw 0(r3),r4
     65c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     660:	00 00 09 d2 	l.j 2da8 <_or1k_exception_handler>
     664:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     700:	d4 00 08 04 	l.sw 4(r0),r1
     704:	18 20 00 00 	l.movhi r1,0x0
     708:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     70c:	84 21 00 00 	l.lwz r1,0(r1)
     710:	e4 01 00 00 	l.sfeq r1,r0
     714:	0c 00 00 06 	l.bnf 72c <_or1k_reset+0x62c>
     718:	15 00 00 00 	l.nop 0x0
     71c:	18 20 00 00 	l.movhi r1,0x0
     720:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     724:	00 00 00 04 	l.j 734 <_or1k_reset+0x634>
     728:	84 21 00 00 	l.lwz r1,0(r1)
     72c:	84 20 00 04 	l.lwz r1,4(r0)
     730:	9c 21 ff 80 	l.addi r1,r1,-128
     734:	9c 21 ff 78 	l.addi r1,r1,-136
     738:	d4 01 18 0c 	l.sw 12(r1),r3
     73c:	84 60 00 04 	l.lwz r3,4(r0)
     740:	d4 01 18 04 	l.sw 4(r1),r3
     744:	d4 01 20 10 	l.sw 16(r1),r4
     748:	18 60 00 00 	l.movhi r3,0x0
     74c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     750:	84 83 00 00 	l.lwz r4,0(r3)
     754:	9c 84 00 01 	l.addi r4,r4,1
     758:	d4 03 20 00 	l.sw 0(r3),r4
     75c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     760:	00 00 09 92 	l.j 2da8 <_or1k_exception_handler>
     764:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     800:	d4 00 08 04 	l.sw 4(r0),r1
     804:	18 20 00 00 	l.movhi r1,0x0
     808:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     80c:	84 21 00 00 	l.lwz r1,0(r1)
     810:	e4 01 00 00 	l.sfeq r1,r0
     814:	0c 00 00 06 	l.bnf 82c <_or1k_reset+0x72c>
     818:	15 00 00 00 	l.nop 0x0
     81c:	18 20 00 00 	l.movhi r1,0x0
     820:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     824:	00 00 00 04 	l.j 834 <_or1k_reset+0x734>
     828:	84 21 00 00 	l.lwz r1,0(r1)
     82c:	84 20 00 04 	l.lwz r1,4(r0)
     830:	9c 21 ff 80 	l.addi r1,r1,-128
     834:	9c 21 ff 78 	l.addi r1,r1,-136
     838:	d4 01 18 0c 	l.sw 12(r1),r3
     83c:	84 60 00 04 	l.lwz r3,4(r0)
     840:	d4 01 18 04 	l.sw 4(r1),r3
     844:	d4 01 20 10 	l.sw 16(r1),r4
     848:	18 60 00 00 	l.movhi r3,0x0
     84c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     850:	84 83 00 00 	l.lwz r4,0(r3)
     854:	9c 84 00 01 	l.addi r4,r4,1
     858:	d4 03 20 00 	l.sw 0(r3),r4
     85c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     860:	00 00 09 52 	l.j 2da8 <_or1k_exception_handler>
     864:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     900:	d4 00 08 04 	l.sw 4(r0),r1
     904:	18 20 00 00 	l.movhi r1,0x0
     908:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     90c:	84 21 00 00 	l.lwz r1,0(r1)
     910:	e4 01 00 00 	l.sfeq r1,r0
     914:	0c 00 00 06 	l.bnf 92c <_or1k_reset+0x82c>
     918:	15 00 00 00 	l.nop 0x0
     91c:	18 20 00 00 	l.movhi r1,0x0
     920:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     924:	00 00 00 04 	l.j 934 <_or1k_reset+0x834>
     928:	84 21 00 00 	l.lwz r1,0(r1)
     92c:	84 20 00 04 	l.lwz r1,4(r0)
     930:	9c 21 ff 80 	l.addi r1,r1,-128
     934:	9c 21 ff 78 	l.addi r1,r1,-136
     938:	d4 01 18 0c 	l.sw 12(r1),r3
     93c:	84 60 00 04 	l.lwz r3,4(r0)
     940:	d4 01 18 04 	l.sw 4(r1),r3
     944:	d4 01 20 10 	l.sw 16(r1),r4
     948:	18 60 00 00 	l.movhi r3,0x0
     94c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     950:	84 83 00 00 	l.lwz r4,0(r3)
     954:	9c 84 00 01 	l.addi r4,r4,1
     958:	d4 03 20 00 	l.sw 0(r3),r4
     95c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     960:	00 00 09 12 	l.j 2da8 <_or1k_exception_handler>
     964:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     a00:	d4 00 08 04 	l.sw 4(r0),r1
     a04:	18 20 00 00 	l.movhi r1,0x0
     a08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     a0c:	84 21 00 00 	l.lwz r1,0(r1)
     a10:	e4 01 00 00 	l.sfeq r1,r0
     a14:	0c 00 00 06 	l.bnf a2c <_or1k_reset+0x92c>
     a18:	15 00 00 00 	l.nop 0x0
     a1c:	18 20 00 00 	l.movhi r1,0x0
     a20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     a24:	00 00 00 04 	l.j a34 <_or1k_reset+0x934>
     a28:	84 21 00 00 	l.lwz r1,0(r1)
     a2c:	84 20 00 04 	l.lwz r1,4(r0)
     a30:	9c 21 ff 80 	l.addi r1,r1,-128
     a34:	9c 21 ff 78 	l.addi r1,r1,-136
     a38:	d4 01 18 0c 	l.sw 12(r1),r3
     a3c:	84 60 00 04 	l.lwz r3,4(r0)
     a40:	d4 01 18 04 	l.sw 4(r1),r3
     a44:	d4 01 20 10 	l.sw 16(r1),r4
     a48:	18 60 00 00 	l.movhi r3,0x0
     a4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     a50:	84 83 00 00 	l.lwz r4,0(r3)
     a54:	9c 84 00 01 	l.addi r4,r4,1
     a58:	d4 03 20 00 	l.sw 0(r3),r4
     a5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     a60:	00 00 08 d2 	l.j 2da8 <_or1k_exception_handler>
     a64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     b00:	d4 00 08 04 	l.sw 4(r0),r1
     b04:	18 20 00 00 	l.movhi r1,0x0
     b08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     b0c:	84 21 00 00 	l.lwz r1,0(r1)
     b10:	e4 01 00 00 	l.sfeq r1,r0
     b14:	0c 00 00 06 	l.bnf b2c <_or1k_reset+0xa2c>
     b18:	15 00 00 00 	l.nop 0x0
     b1c:	18 20 00 00 	l.movhi r1,0x0
     b20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     b24:	00 00 00 04 	l.j b34 <_or1k_reset+0xa34>
     b28:	84 21 00 00 	l.lwz r1,0(r1)
     b2c:	84 20 00 04 	l.lwz r1,4(r0)
     b30:	9c 21 ff 80 	l.addi r1,r1,-128
     b34:	9c 21 ff 78 	l.addi r1,r1,-136
     b38:	d4 01 18 0c 	l.sw 12(r1),r3
     b3c:	84 60 00 04 	l.lwz r3,4(r0)
     b40:	d4 01 18 04 	l.sw 4(r1),r3
     b44:	d4 01 20 10 	l.sw 16(r1),r4
     b48:	18 60 00 00 	l.movhi r3,0x0
     b4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     b50:	84 83 00 00 	l.lwz r4,0(r3)
     b54:	9c 84 00 01 	l.addi r4,r4,1
     b58:	d4 03 20 00 	l.sw 0(r3),r4
     b5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     b60:	00 00 08 92 	l.j 2da8 <_or1k_exception_handler>
     b64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     c00:	d4 00 08 04 	l.sw 4(r0),r1
     c04:	18 20 00 00 	l.movhi r1,0x0
     c08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     c0c:	84 21 00 00 	l.lwz r1,0(r1)
     c10:	e4 01 00 00 	l.sfeq r1,r0
     c14:	0c 00 00 06 	l.bnf c2c <_or1k_reset+0xb2c>
     c18:	15 00 00 00 	l.nop 0x0
     c1c:	18 20 00 00 	l.movhi r1,0x0
     c20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     c24:	00 00 00 04 	l.j c34 <_or1k_reset+0xb34>
     c28:	84 21 00 00 	l.lwz r1,0(r1)
     c2c:	84 20 00 04 	l.lwz r1,4(r0)
     c30:	9c 21 ff 80 	l.addi r1,r1,-128
     c34:	9c 21 ff 78 	l.addi r1,r1,-136
     c38:	d4 01 18 0c 	l.sw 12(r1),r3
     c3c:	84 60 00 04 	l.lwz r3,4(r0)
     c40:	d4 01 18 04 	l.sw 4(r1),r3
     c44:	d4 01 20 10 	l.sw 16(r1),r4
     c48:	18 60 00 00 	l.movhi r3,0x0
     c4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     c50:	84 83 00 00 	l.lwz r4,0(r3)
     c54:	9c 84 00 01 	l.addi r4,r4,1
     c58:	d4 03 20 00 	l.sw 0(r3),r4
     c5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     c60:	00 00 08 52 	l.j 2da8 <_or1k_exception_handler>
     c64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     d00:	d4 00 08 04 	l.sw 4(r0),r1
     d04:	18 20 00 00 	l.movhi r1,0x0
     d08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     d0c:	84 21 00 00 	l.lwz r1,0(r1)
     d10:	e4 01 00 00 	l.sfeq r1,r0
     d14:	0c 00 00 06 	l.bnf d2c <_or1k_reset+0xc2c>
     d18:	15 00 00 00 	l.nop 0x0
     d1c:	18 20 00 00 	l.movhi r1,0x0
     d20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     d24:	00 00 00 04 	l.j d34 <_or1k_reset+0xc34>
     d28:	84 21 00 00 	l.lwz r1,0(r1)
     d2c:	84 20 00 04 	l.lwz r1,4(r0)
     d30:	9c 21 ff 80 	l.addi r1,r1,-128
     d34:	9c 21 ff 78 	l.addi r1,r1,-136
     d38:	d4 01 18 0c 	l.sw 12(r1),r3
     d3c:	84 60 00 04 	l.lwz r3,4(r0)
     d40:	d4 01 18 04 	l.sw 4(r1),r3
     d44:	d4 01 20 10 	l.sw 16(r1),r4
     d48:	18 60 00 00 	l.movhi r3,0x0
     d4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     d50:	84 83 00 00 	l.lwz r4,0(r3)
     d54:	9c 84 00 01 	l.addi r4,r4,1
     d58:	d4 03 20 00 	l.sw 0(r3),r4
     d5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     d60:	00 00 08 12 	l.j 2da8 <_or1k_exception_handler>
     d64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     e00:	d4 00 08 04 	l.sw 4(r0),r1
     e04:	18 20 00 00 	l.movhi r1,0x0
     e08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     e0c:	84 21 00 00 	l.lwz r1,0(r1)
     e10:	e4 01 00 00 	l.sfeq r1,r0
     e14:	0c 00 00 06 	l.bnf e2c <_or1k_reset+0xd2c>
     e18:	15 00 00 00 	l.nop 0x0
     e1c:	18 20 00 00 	l.movhi r1,0x0
     e20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     e24:	00 00 00 04 	l.j e34 <_or1k_reset+0xd34>
     e28:	84 21 00 00 	l.lwz r1,0(r1)
     e2c:	84 20 00 04 	l.lwz r1,4(r0)
     e30:	9c 21 ff 80 	l.addi r1,r1,-128
     e34:	9c 21 ff 78 	l.addi r1,r1,-136
     e38:	d4 01 18 0c 	l.sw 12(r1),r3
     e3c:	84 60 00 04 	l.lwz r3,4(r0)
     e40:	d4 01 18 04 	l.sw 4(r1),r3
     e44:	d4 01 20 10 	l.sw 16(r1),r4
     e48:	18 60 00 00 	l.movhi r3,0x0
     e4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     e50:	84 83 00 00 	l.lwz r4,0(r3)
     e54:	9c 84 00 01 	l.addi r4,r4,1
     e58:	d4 03 20 00 	l.sw 0(r3),r4
     e5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     e60:	00 00 07 d2 	l.j 2da8 <_or1k_exception_handler>
     e64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
     f00:	d4 00 08 04 	l.sw 4(r0),r1
     f04:	18 20 00 00 	l.movhi r1,0x0
     f08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
     f0c:	84 21 00 00 	l.lwz r1,0(r1)
     f10:	e4 01 00 00 	l.sfeq r1,r0
     f14:	0c 00 00 06 	l.bnf f2c <_or1k_reset+0xe2c>
     f18:	15 00 00 00 	l.nop 0x0
     f1c:	18 20 00 00 	l.movhi r1,0x0
     f20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
     f24:	00 00 00 04 	l.j f34 <_or1k_reset+0xe34>
     f28:	84 21 00 00 	l.lwz r1,0(r1)
     f2c:	84 20 00 04 	l.lwz r1,4(r0)
     f30:	9c 21 ff 80 	l.addi r1,r1,-128
     f34:	9c 21 ff 78 	l.addi r1,r1,-136
     f38:	d4 01 18 0c 	l.sw 12(r1),r3
     f3c:	84 60 00 04 	l.lwz r3,4(r0)
     f40:	d4 01 18 04 	l.sw 4(r1),r3
     f44:	d4 01 20 10 	l.sw 16(r1),r4
     f48:	18 60 00 00 	l.movhi r3,0x0
     f4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
     f50:	84 83 00 00 	l.lwz r4,0(r3)
     f54:	9c 84 00 01 	l.addi r4,r4,1
     f58:	d4 03 20 00 	l.sw 0(r3),r4
     f5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
     f60:	00 00 07 92 	l.j 2da8 <_or1k_exception_handler>
     f64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1000:	d4 00 08 04 	l.sw 4(r0),r1
    1004:	18 20 00 00 	l.movhi r1,0x0
    1008:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    100c:	84 21 00 00 	l.lwz r1,0(r1)
    1010:	e4 01 00 00 	l.sfeq r1,r0
    1014:	0c 00 00 06 	l.bnf 102c <_or1k_reset+0xf2c>
    1018:	15 00 00 00 	l.nop 0x0
    101c:	18 20 00 00 	l.movhi r1,0x0
    1020:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1024:	00 00 00 04 	l.j 1034 <_or1k_reset+0xf34>
    1028:	84 21 00 00 	l.lwz r1,0(r1)
    102c:	84 20 00 04 	l.lwz r1,4(r0)
    1030:	9c 21 ff 80 	l.addi r1,r1,-128
    1034:	9c 21 ff 78 	l.addi r1,r1,-136
    1038:	d4 01 18 0c 	l.sw 12(r1),r3
    103c:	84 60 00 04 	l.lwz r3,4(r0)
    1040:	d4 01 18 04 	l.sw 4(r1),r3
    1044:	d4 01 20 10 	l.sw 16(r1),r4
    1048:	18 60 00 00 	l.movhi r3,0x0
    104c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1050:	84 83 00 00 	l.lwz r4,0(r3)
    1054:	9c 84 00 01 	l.addi r4,r4,1
    1058:	d4 03 20 00 	l.sw 0(r3),r4
    105c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1060:	00 00 07 52 	l.j 2da8 <_or1k_exception_handler>
    1064:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1100:	d4 00 08 04 	l.sw 4(r0),r1
    1104:	18 20 00 00 	l.movhi r1,0x0
    1108:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    110c:	84 21 00 00 	l.lwz r1,0(r1)
    1110:	e4 01 00 00 	l.sfeq r1,r0
    1114:	0c 00 00 06 	l.bnf 112c <_or1k_reset+0x102c>
    1118:	15 00 00 00 	l.nop 0x0
    111c:	18 20 00 00 	l.movhi r1,0x0
    1120:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1124:	00 00 00 04 	l.j 1134 <_or1k_reset+0x1034>
    1128:	84 21 00 00 	l.lwz r1,0(r1)
    112c:	84 20 00 04 	l.lwz r1,4(r0)
    1130:	9c 21 ff 80 	l.addi r1,r1,-128
    1134:	9c 21 ff 78 	l.addi r1,r1,-136
    1138:	d4 01 18 0c 	l.sw 12(r1),r3
    113c:	84 60 00 04 	l.lwz r3,4(r0)
    1140:	d4 01 18 04 	l.sw 4(r1),r3
    1144:	d4 01 20 10 	l.sw 16(r1),r4
    1148:	18 60 00 00 	l.movhi r3,0x0
    114c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1150:	84 83 00 00 	l.lwz r4,0(r3)
    1154:	9c 84 00 01 	l.addi r4,r4,1
    1158:	d4 03 20 00 	l.sw 0(r3),r4
    115c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1160:	00 00 07 12 	l.j 2da8 <_or1k_exception_handler>
    1164:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1200:	d4 00 08 04 	l.sw 4(r0),r1
    1204:	18 20 00 00 	l.movhi r1,0x0
    1208:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    120c:	84 21 00 00 	l.lwz r1,0(r1)
    1210:	e4 01 00 00 	l.sfeq r1,r0
    1214:	0c 00 00 06 	l.bnf 122c <_or1k_reset+0x112c>
    1218:	15 00 00 00 	l.nop 0x0
    121c:	18 20 00 00 	l.movhi r1,0x0
    1220:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1224:	00 00 00 04 	l.j 1234 <_or1k_reset+0x1134>
    1228:	84 21 00 00 	l.lwz r1,0(r1)
    122c:	84 20 00 04 	l.lwz r1,4(r0)
    1230:	9c 21 ff 80 	l.addi r1,r1,-128
    1234:	9c 21 ff 78 	l.addi r1,r1,-136
    1238:	d4 01 18 0c 	l.sw 12(r1),r3
    123c:	84 60 00 04 	l.lwz r3,4(r0)
    1240:	d4 01 18 04 	l.sw 4(r1),r3
    1244:	d4 01 20 10 	l.sw 16(r1),r4
    1248:	18 60 00 00 	l.movhi r3,0x0
    124c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1250:	84 83 00 00 	l.lwz r4,0(r3)
    1254:	9c 84 00 01 	l.addi r4,r4,1
    1258:	d4 03 20 00 	l.sw 0(r3),r4
    125c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1260:	00 00 06 d2 	l.j 2da8 <_or1k_exception_handler>
    1264:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1300:	d4 00 08 04 	l.sw 4(r0),r1
    1304:	18 20 00 00 	l.movhi r1,0x0
    1308:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    130c:	84 21 00 00 	l.lwz r1,0(r1)
    1310:	e4 01 00 00 	l.sfeq r1,r0
    1314:	0c 00 00 06 	l.bnf 132c <_or1k_reset+0x122c>
    1318:	15 00 00 00 	l.nop 0x0
    131c:	18 20 00 00 	l.movhi r1,0x0
    1320:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1324:	00 00 00 04 	l.j 1334 <_or1k_reset+0x1234>
    1328:	84 21 00 00 	l.lwz r1,0(r1)
    132c:	84 20 00 04 	l.lwz r1,4(r0)
    1330:	9c 21 ff 80 	l.addi r1,r1,-128
    1334:	9c 21 ff 78 	l.addi r1,r1,-136
    1338:	d4 01 18 0c 	l.sw 12(r1),r3
    133c:	84 60 00 04 	l.lwz r3,4(r0)
    1340:	d4 01 18 04 	l.sw 4(r1),r3
    1344:	d4 01 20 10 	l.sw 16(r1),r4
    1348:	18 60 00 00 	l.movhi r3,0x0
    134c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1350:	84 83 00 00 	l.lwz r4,0(r3)
    1354:	9c 84 00 01 	l.addi r4,r4,1
    1358:	d4 03 20 00 	l.sw 0(r3),r4
    135c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1360:	00 00 06 92 	l.j 2da8 <_or1k_exception_handler>
    1364:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1400:	d4 00 08 04 	l.sw 4(r0),r1
    1404:	18 20 00 00 	l.movhi r1,0x0
    1408:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    140c:	84 21 00 00 	l.lwz r1,0(r1)
    1410:	e4 01 00 00 	l.sfeq r1,r0
    1414:	0c 00 00 06 	l.bnf 142c <_or1k_reset+0x132c>
    1418:	15 00 00 00 	l.nop 0x0
    141c:	18 20 00 00 	l.movhi r1,0x0
    1420:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1424:	00 00 00 04 	l.j 1434 <_or1k_reset+0x1334>
    1428:	84 21 00 00 	l.lwz r1,0(r1)
    142c:	84 20 00 04 	l.lwz r1,4(r0)
    1430:	9c 21 ff 80 	l.addi r1,r1,-128
    1434:	9c 21 ff 78 	l.addi r1,r1,-136
    1438:	d4 01 18 0c 	l.sw 12(r1),r3
    143c:	84 60 00 04 	l.lwz r3,4(r0)
    1440:	d4 01 18 04 	l.sw 4(r1),r3
    1444:	d4 01 20 10 	l.sw 16(r1),r4
    1448:	18 60 00 00 	l.movhi r3,0x0
    144c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1450:	84 83 00 00 	l.lwz r4,0(r3)
    1454:	9c 84 00 01 	l.addi r4,r4,1
    1458:	d4 03 20 00 	l.sw 0(r3),r4
    145c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1460:	00 00 06 52 	l.j 2da8 <_or1k_exception_handler>
    1464:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1500:	d4 00 08 04 	l.sw 4(r0),r1
    1504:	18 20 00 00 	l.movhi r1,0x0
    1508:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    150c:	84 21 00 00 	l.lwz r1,0(r1)
    1510:	e4 01 00 00 	l.sfeq r1,r0
    1514:	0c 00 00 06 	l.bnf 152c <_or1k_reset+0x142c>
    1518:	15 00 00 00 	l.nop 0x0
    151c:	18 20 00 00 	l.movhi r1,0x0
    1520:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1524:	00 00 00 04 	l.j 1534 <_or1k_reset+0x1434>
    1528:	84 21 00 00 	l.lwz r1,0(r1)
    152c:	84 20 00 04 	l.lwz r1,4(r0)
    1530:	9c 21 ff 80 	l.addi r1,r1,-128
    1534:	9c 21 ff 78 	l.addi r1,r1,-136
    1538:	d4 01 18 0c 	l.sw 12(r1),r3
    153c:	84 60 00 04 	l.lwz r3,4(r0)
    1540:	d4 01 18 04 	l.sw 4(r1),r3
    1544:	d4 01 20 10 	l.sw 16(r1),r4
    1548:	18 60 00 00 	l.movhi r3,0x0
    154c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1550:	84 83 00 00 	l.lwz r4,0(r3)
    1554:	9c 84 00 01 	l.addi r4,r4,1
    1558:	d4 03 20 00 	l.sw 0(r3),r4
    155c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1560:	00 00 06 12 	l.j 2da8 <_or1k_exception_handler>
    1564:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1600:	d4 00 08 04 	l.sw 4(r0),r1
    1604:	18 20 00 00 	l.movhi r1,0x0
    1608:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    160c:	84 21 00 00 	l.lwz r1,0(r1)
    1610:	e4 01 00 00 	l.sfeq r1,r0
    1614:	0c 00 00 06 	l.bnf 162c <_or1k_reset+0x152c>
    1618:	15 00 00 00 	l.nop 0x0
    161c:	18 20 00 00 	l.movhi r1,0x0
    1620:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1624:	00 00 00 04 	l.j 1634 <_or1k_reset+0x1534>
    1628:	84 21 00 00 	l.lwz r1,0(r1)
    162c:	84 20 00 04 	l.lwz r1,4(r0)
    1630:	9c 21 ff 80 	l.addi r1,r1,-128
    1634:	9c 21 ff 78 	l.addi r1,r1,-136
    1638:	d4 01 18 0c 	l.sw 12(r1),r3
    163c:	84 60 00 04 	l.lwz r3,4(r0)
    1640:	d4 01 18 04 	l.sw 4(r1),r3
    1644:	d4 01 20 10 	l.sw 16(r1),r4
    1648:	18 60 00 00 	l.movhi r3,0x0
    164c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1650:	84 83 00 00 	l.lwz r4,0(r3)
    1654:	9c 84 00 01 	l.addi r4,r4,1
    1658:	d4 03 20 00 	l.sw 0(r3),r4
    165c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1660:	00 00 05 d2 	l.j 2da8 <_or1k_exception_handler>
    1664:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1700:	d4 00 08 04 	l.sw 4(r0),r1
    1704:	18 20 00 00 	l.movhi r1,0x0
    1708:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    170c:	84 21 00 00 	l.lwz r1,0(r1)
    1710:	e4 01 00 00 	l.sfeq r1,r0
    1714:	0c 00 00 06 	l.bnf 172c <_or1k_reset+0x162c>
    1718:	15 00 00 00 	l.nop 0x0
    171c:	18 20 00 00 	l.movhi r1,0x0
    1720:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1724:	00 00 00 04 	l.j 1734 <_or1k_reset+0x1634>
    1728:	84 21 00 00 	l.lwz r1,0(r1)
    172c:	84 20 00 04 	l.lwz r1,4(r0)
    1730:	9c 21 ff 80 	l.addi r1,r1,-128
    1734:	9c 21 ff 78 	l.addi r1,r1,-136
    1738:	d4 01 18 0c 	l.sw 12(r1),r3
    173c:	84 60 00 04 	l.lwz r3,4(r0)
    1740:	d4 01 18 04 	l.sw 4(r1),r3
    1744:	d4 01 20 10 	l.sw 16(r1),r4
    1748:	18 60 00 00 	l.movhi r3,0x0
    174c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1750:	84 83 00 00 	l.lwz r4,0(r3)
    1754:	9c 84 00 01 	l.addi r4,r4,1
    1758:	d4 03 20 00 	l.sw 0(r3),r4
    175c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1760:	00 00 05 92 	l.j 2da8 <_or1k_exception_handler>
    1764:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1800:	d4 00 08 04 	l.sw 4(r0),r1
    1804:	18 20 00 00 	l.movhi r1,0x0
    1808:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    180c:	84 21 00 00 	l.lwz r1,0(r1)
    1810:	e4 01 00 00 	l.sfeq r1,r0
    1814:	0c 00 00 06 	l.bnf 182c <_or1k_reset+0x172c>
    1818:	15 00 00 00 	l.nop 0x0
    181c:	18 20 00 00 	l.movhi r1,0x0
    1820:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1824:	00 00 00 04 	l.j 1834 <_or1k_reset+0x1734>
    1828:	84 21 00 00 	l.lwz r1,0(r1)
    182c:	84 20 00 04 	l.lwz r1,4(r0)
    1830:	9c 21 ff 80 	l.addi r1,r1,-128
    1834:	9c 21 ff 78 	l.addi r1,r1,-136
    1838:	d4 01 18 0c 	l.sw 12(r1),r3
    183c:	84 60 00 04 	l.lwz r3,4(r0)
    1840:	d4 01 18 04 	l.sw 4(r1),r3
    1844:	d4 01 20 10 	l.sw 16(r1),r4
    1848:	18 60 00 00 	l.movhi r3,0x0
    184c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1850:	84 83 00 00 	l.lwz r4,0(r3)
    1854:	9c 84 00 01 	l.addi r4,r4,1
    1858:	d4 03 20 00 	l.sw 0(r3),r4
    185c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1860:	00 00 05 52 	l.j 2da8 <_or1k_exception_handler>
    1864:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1900:	d4 00 08 04 	l.sw 4(r0),r1
    1904:	18 20 00 00 	l.movhi r1,0x0
    1908:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    190c:	84 21 00 00 	l.lwz r1,0(r1)
    1910:	e4 01 00 00 	l.sfeq r1,r0
    1914:	0c 00 00 06 	l.bnf 192c <_or1k_reset+0x182c>
    1918:	15 00 00 00 	l.nop 0x0
    191c:	18 20 00 00 	l.movhi r1,0x0
    1920:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1924:	00 00 00 04 	l.j 1934 <_or1k_reset+0x1834>
    1928:	84 21 00 00 	l.lwz r1,0(r1)
    192c:	84 20 00 04 	l.lwz r1,4(r0)
    1930:	9c 21 ff 80 	l.addi r1,r1,-128
    1934:	9c 21 ff 78 	l.addi r1,r1,-136
    1938:	d4 01 18 0c 	l.sw 12(r1),r3
    193c:	84 60 00 04 	l.lwz r3,4(r0)
    1940:	d4 01 18 04 	l.sw 4(r1),r3
    1944:	d4 01 20 10 	l.sw 16(r1),r4
    1948:	18 60 00 00 	l.movhi r3,0x0
    194c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1950:	84 83 00 00 	l.lwz r4,0(r3)
    1954:	9c 84 00 01 	l.addi r4,r4,1
    1958:	d4 03 20 00 	l.sw 0(r3),r4
    195c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1960:	00 00 05 12 	l.j 2da8 <_or1k_exception_handler>
    1964:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1a00:	d4 00 08 04 	l.sw 4(r0),r1
    1a04:	18 20 00 00 	l.movhi r1,0x0
    1a08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1a0c:	84 21 00 00 	l.lwz r1,0(r1)
    1a10:	e4 01 00 00 	l.sfeq r1,r0
    1a14:	0c 00 00 06 	l.bnf 1a2c <_or1k_reset+0x192c>
    1a18:	15 00 00 00 	l.nop 0x0
    1a1c:	18 20 00 00 	l.movhi r1,0x0
    1a20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1a24:	00 00 00 04 	l.j 1a34 <_or1k_reset+0x1934>
    1a28:	84 21 00 00 	l.lwz r1,0(r1)
    1a2c:	84 20 00 04 	l.lwz r1,4(r0)
    1a30:	9c 21 ff 80 	l.addi r1,r1,-128
    1a34:	9c 21 ff 78 	l.addi r1,r1,-136
    1a38:	d4 01 18 0c 	l.sw 12(r1),r3
    1a3c:	84 60 00 04 	l.lwz r3,4(r0)
    1a40:	d4 01 18 04 	l.sw 4(r1),r3
    1a44:	d4 01 20 10 	l.sw 16(r1),r4
    1a48:	18 60 00 00 	l.movhi r3,0x0
    1a4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1a50:	84 83 00 00 	l.lwz r4,0(r3)
    1a54:	9c 84 00 01 	l.addi r4,r4,1
    1a58:	d4 03 20 00 	l.sw 0(r3),r4
    1a5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1a60:	00 00 04 d2 	l.j 2da8 <_or1k_exception_handler>
    1a64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1b00:	d4 00 08 04 	l.sw 4(r0),r1
    1b04:	18 20 00 00 	l.movhi r1,0x0
    1b08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1b0c:	84 21 00 00 	l.lwz r1,0(r1)
    1b10:	e4 01 00 00 	l.sfeq r1,r0
    1b14:	0c 00 00 06 	l.bnf 1b2c <_or1k_reset+0x1a2c>
    1b18:	15 00 00 00 	l.nop 0x0
    1b1c:	18 20 00 00 	l.movhi r1,0x0
    1b20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1b24:	00 00 00 04 	l.j 1b34 <_or1k_reset+0x1a34>
    1b28:	84 21 00 00 	l.lwz r1,0(r1)
    1b2c:	84 20 00 04 	l.lwz r1,4(r0)
    1b30:	9c 21 ff 80 	l.addi r1,r1,-128
    1b34:	9c 21 ff 78 	l.addi r1,r1,-136
    1b38:	d4 01 18 0c 	l.sw 12(r1),r3
    1b3c:	84 60 00 04 	l.lwz r3,4(r0)
    1b40:	d4 01 18 04 	l.sw 4(r1),r3
    1b44:	d4 01 20 10 	l.sw 16(r1),r4
    1b48:	18 60 00 00 	l.movhi r3,0x0
    1b4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1b50:	84 83 00 00 	l.lwz r4,0(r3)
    1b54:	9c 84 00 01 	l.addi r4,r4,1
    1b58:	d4 03 20 00 	l.sw 0(r3),r4
    1b5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1b60:	00 00 04 92 	l.j 2da8 <_or1k_exception_handler>
    1b64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1c00:	d4 00 08 04 	l.sw 4(r0),r1
    1c04:	18 20 00 00 	l.movhi r1,0x0
    1c08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1c0c:	84 21 00 00 	l.lwz r1,0(r1)
    1c10:	e4 01 00 00 	l.sfeq r1,r0
    1c14:	0c 00 00 06 	l.bnf 1c2c <_or1k_reset+0x1b2c>
    1c18:	15 00 00 00 	l.nop 0x0
    1c1c:	18 20 00 00 	l.movhi r1,0x0
    1c20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1c24:	00 00 00 04 	l.j 1c34 <_or1k_reset+0x1b34>
    1c28:	84 21 00 00 	l.lwz r1,0(r1)
    1c2c:	84 20 00 04 	l.lwz r1,4(r0)
    1c30:	9c 21 ff 80 	l.addi r1,r1,-128
    1c34:	9c 21 ff 78 	l.addi r1,r1,-136
    1c38:	d4 01 18 0c 	l.sw 12(r1),r3
    1c3c:	84 60 00 04 	l.lwz r3,4(r0)
    1c40:	d4 01 18 04 	l.sw 4(r1),r3
    1c44:	d4 01 20 10 	l.sw 16(r1),r4
    1c48:	18 60 00 00 	l.movhi r3,0x0
    1c4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1c50:	84 83 00 00 	l.lwz r4,0(r3)
    1c54:	9c 84 00 01 	l.addi r4,r4,1
    1c58:	d4 03 20 00 	l.sw 0(r3),r4
    1c5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1c60:	00 00 04 52 	l.j 2da8 <_or1k_exception_handler>
    1c64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1d00:	d4 00 08 04 	l.sw 4(r0),r1
    1d04:	18 20 00 00 	l.movhi r1,0x0
    1d08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1d0c:	84 21 00 00 	l.lwz r1,0(r1)
    1d10:	e4 01 00 00 	l.sfeq r1,r0
    1d14:	0c 00 00 06 	l.bnf 1d2c <_or1k_reset+0x1c2c>
    1d18:	15 00 00 00 	l.nop 0x0
    1d1c:	18 20 00 00 	l.movhi r1,0x0
    1d20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1d24:	00 00 00 04 	l.j 1d34 <_or1k_reset+0x1c34>
    1d28:	84 21 00 00 	l.lwz r1,0(r1)
    1d2c:	84 20 00 04 	l.lwz r1,4(r0)
    1d30:	9c 21 ff 80 	l.addi r1,r1,-128
    1d34:	9c 21 ff 78 	l.addi r1,r1,-136
    1d38:	d4 01 18 0c 	l.sw 12(r1),r3
    1d3c:	84 60 00 04 	l.lwz r3,4(r0)
    1d40:	d4 01 18 04 	l.sw 4(r1),r3
    1d44:	d4 01 20 10 	l.sw 16(r1),r4
    1d48:	18 60 00 00 	l.movhi r3,0x0
    1d4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1d50:	84 83 00 00 	l.lwz r4,0(r3)
    1d54:	9c 84 00 01 	l.addi r4,r4,1
    1d58:	d4 03 20 00 	l.sw 0(r3),r4
    1d5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1d60:	00 00 04 12 	l.j 2da8 <_or1k_exception_handler>
    1d64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1e00:	d4 00 08 04 	l.sw 4(r0),r1
    1e04:	18 20 00 00 	l.movhi r1,0x0
    1e08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1e0c:	84 21 00 00 	l.lwz r1,0(r1)
    1e10:	e4 01 00 00 	l.sfeq r1,r0
    1e14:	0c 00 00 06 	l.bnf 1e2c <_or1k_reset+0x1d2c>
    1e18:	15 00 00 00 	l.nop 0x0
    1e1c:	18 20 00 00 	l.movhi r1,0x0
    1e20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1e24:	00 00 00 04 	l.j 1e34 <_or1k_reset+0x1d34>
    1e28:	84 21 00 00 	l.lwz r1,0(r1)
    1e2c:	84 20 00 04 	l.lwz r1,4(r0)
    1e30:	9c 21 ff 80 	l.addi r1,r1,-128
    1e34:	9c 21 ff 78 	l.addi r1,r1,-136
    1e38:	d4 01 18 0c 	l.sw 12(r1),r3
    1e3c:	84 60 00 04 	l.lwz r3,4(r0)
    1e40:	d4 01 18 04 	l.sw 4(r1),r3
    1e44:	d4 01 20 10 	l.sw 16(r1),r4
    1e48:	18 60 00 00 	l.movhi r3,0x0
    1e4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1e50:	84 83 00 00 	l.lwz r4,0(r3)
    1e54:	9c 84 00 01 	l.addi r4,r4,1
    1e58:	d4 03 20 00 	l.sw 0(r3),r4
    1e5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1e60:	00 00 03 d2 	l.j 2da8 <_or1k_exception_handler>
    1e64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1f00:	d4 00 08 04 	l.sw 4(r0),r1
    1f04:	18 20 00 00 	l.movhi r1,0x0
    1f08:	a8 21 49 e0 	l.ori r1,r1,0x49e0
    1f0c:	84 21 00 00 	l.lwz r1,0(r1)
    1f10:	e4 01 00 00 	l.sfeq r1,r0
    1f14:	0c 00 00 06 	l.bnf 1f2c <_or1k_reset+0x1e2c>
    1f18:	15 00 00 00 	l.nop 0x0
    1f1c:	18 20 00 00 	l.movhi r1,0x0
    1f20:	a8 21 49 e4 	l.ori r1,r1,0x49e4
    1f24:	00 00 00 04 	l.j 1f34 <_or1k_reset+0x1e34>
    1f28:	84 21 00 00 	l.lwz r1,0(r1)
    1f2c:	84 20 00 04 	l.lwz r1,4(r0)
    1f30:	9c 21 ff 80 	l.addi r1,r1,-128
    1f34:	9c 21 ff 78 	l.addi r1,r1,-136
    1f38:	d4 01 18 0c 	l.sw 12(r1),r3
    1f3c:	84 60 00 04 	l.lwz r3,4(r0)
    1f40:	d4 01 18 04 	l.sw 4(r1),r3
    1f44:	d4 01 20 10 	l.sw 16(r1),r4
    1f48:	18 60 00 00 	l.movhi r3,0x0
    1f4c:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    1f50:	84 83 00 00 	l.lwz r4,0(r3)
    1f54:	9c 84 00 01 	l.addi r4,r4,1
    1f58:	d4 03 20 00 	l.sw 0(r3),r4
    1f5c:	b4 60 00 10 	l.mfspr r3,r0,0x10
    1f60:	00 00 03 92 	l.j 2da8 <_or1k_exception_handler>
    1f64:	b4 80 00 20 	l.mfspr r4,r0,0x20
	...
    1ffc:	15 00 00 00 	l.nop 0x0

Disassembly of section .init:

00002000 <_init-0x4>:
    2000:	15 00 00 00 	l.nop 0x0

00002004 <_init>:
    2004:	9c 21 ff fc 	l.addi r1,r1,-4
    2008:	d4 01 48 00 	l.sw 0(r1),r9
    200c:	04 00 00 bc 	l.jal 22fc <frame_dummy>
    2010:	15 00 00 00 	l.nop 0x0
    2014:	04 00 06 65 	l.jal 39a8 <__do_global_ctors_aux>
    2018:	15 00 00 00 	l.nop 0x0
    201c:	85 21 00 00 	l.lwz r9,0(r1)
    2020:	44 00 48 00 	l.jr r9
    2024:	9c 21 00 04 	l.addi r1,r1,4

Disassembly of section .text:

00002028 <_or1k_start-0x8>:
    2028:	44 00 48 00 	l.jr r9
    202c:	15 00 00 00 	l.nop 0x0

00002030 <_or1k_start>:
    2030:	04 00 02 f6 	l.jal 2c08 <_or1k_cache_init>
    2034:	15 00 00 00 	l.nop 0x0
    2038:	04 00 05 b5 	l.jal 370c <_or1k_board_init_early>
    203c:	15 00 00 00 	l.nop 0x0
    2040:	18 60 00 00 	l.movhi r3,0x0
    2044:	a8 63 48 a4 	l.ori r3,r3,0x48a4
    2048:	18 80 00 00 	l.movhi r4,0x0
    204c:	a8 84 4a 6c 	l.ori r4,r4,0x4a6c
    2050:	d4 03 00 00 	l.sw 0(r3),r0
    2054:	e4 83 20 00 	l.sfltu r3,r4
    2058:	13 ff ff fe 	l.bf 2050 <_or1k_start+0x20>
    205c:	9c 63 00 04 	l.addi r3,r3,4
    2060:	18 20 00 00 	l.movhi r1,0x0
    2064:	a8 21 36 e8 	l.ori r1,r1,0x36e8
    2068:	84 21 00 00 	l.lwz r1,0(r1)
    206c:	18 40 00 00 	l.movhi r2,0x0
    2070:	a8 42 36 ec 	l.ori r2,r2,0x36ec
    2074:	84 42 00 00 	l.lwz r2,0(r2)
    2078:	e0 21 10 00 	l.add r1,r1,r2
    207c:	18 60 00 00 	l.movhi r3,0x0
    2080:	a8 63 49 e4 	l.ori r3,r3,0x49e4
    2084:	d4 03 08 00 	l.sw 0(r3),r1
    2088:	18 60 00 00 	l.movhi r3,0x0
    208c:	a8 63 40 48 	l.ori r3,r3,0x4048
    2090:	84 63 00 00 	l.lwz r3,0(r3)
    2094:	e0 81 18 02 	l.sub r4,r1,r3
    2098:	18 a0 00 00 	l.movhi r5,0x0
    209c:	a8 a5 49 ec 	l.ori r5,r5,0x49ec
    20a0:	d4 05 20 00 	l.sw 0(r5),r4
    20a4:	e0 20 20 04 	l.or r1,r0,r4
    20a8:	e0 41 08 04 	l.or r2,r1,r1
    20ac:	18 60 00 00 	l.movhi r3,0x0
    20b0:	a8 63 49 e8 	l.ori r3,r3,0x49e8
    20b4:	d4 03 08 00 	l.sw 0(r3),r1
    20b8:	18 60 00 00 	l.movhi r3,0x0
    20bc:	a8 63 40 44 	l.ori r3,r3,0x4044
    20c0:	84 63 00 00 	l.lwz r3,0(r3)
    20c4:	e0 81 18 02 	l.sub r4,r1,r3
    20c8:	18 a0 00 00 	l.movhi r5,0x0
    20cc:	a8 a5 49 f0 	l.ori r5,r5,0x49f0
    20d0:	d4 05 20 00 	l.sw 0(r5),r4
    20d4:	04 00 04 6f 	l.jal 3290 <_or1k_init>
    20d8:	15 00 00 00 	l.nop 0x0
    20dc:	04 00 04 02 	l.jal 30e4 <_or1k_libc_impure_init>
    20e0:	15 00 00 00 	l.nop 0x0
    20e4:	07 ff ff c8 	l.jal 2004 <_init>
    20e8:	15 00 00 00 	l.nop 0x0
    20ec:	18 60 00 00 	l.movhi r3,0x0
    20f0:	04 00 00 bd 	l.jal 23e4 <atexit>
    20f4:	a8 63 3a 18 	l.ori r3,r3,0x3a18
    20f8:	18 80 00 00 	l.movhi r4,0x0
    20fc:	a8 84 36 f4 	l.ori r4,r4,0x36f4
    2100:	84 84 00 00 	l.lwz r4,0(r4)
    2104:	e4 24 00 00 	l.sfne r4,r0
    2108:	0c 00 00 04 	l.bnf 2118 <_or1k_start+0xe8>
    210c:	e0 60 00 04 	l.or r3,r0,r0
    2110:	04 00 02 51 	l.jal 2a54 <_or1k_uart_init>
    2114:	15 00 00 00 	l.nop 0x0
    2118:	04 00 05 7f 	l.jal 3714 <_or1k_board_init>
    211c:	15 00 00 00 	l.nop 0x0
    2120:	e0 60 00 04 	l.or r3,r0,r0
    2124:	e0 80 00 04 	l.or r4,r0,r0
    2128:	04 00 00 9c 	l.jal 2398 <main>
    212c:	e0 a0 00 04 	l.or r5,r0,r0
    2130:	04 00 00 b9 	l.jal 2414 <exit>
    2134:	9c 6b 00 00 	l.addi r3,r11,0
    2138:	00 00 00 00 	l.j 2138 <_or1k_start+0x108>
    213c:	15 00 00 00 	l.nop 0x0

00002140 <deregister_tm_clones>:
    2140:	d7 e1 17 f8 	l.sw -8(r1),r2
    2144:	18 60 00 00 	l.movhi r3,0x0
    2148:	18 40 00 00 	l.movhi r2,0x0
    214c:	a8 63 48 a7 	l.ori r3,r3,0x48a7
    2150:	a8 42 48 a4 	l.ori r2,r2,0x48a4
    2154:	d7 e1 4f fc 	l.sw -4(r1),r9
    2158:	e0 63 10 02 	l.sub r3,r3,r2
    215c:	d7 e1 0f f4 	l.sw -12(r1),r1
    2160:	bc a3 00 06 	l.sfleui r3,6
    2164:	10 00 00 09 	l.bf 2188 <deregister_tm_clones+0x48>
    2168:	9c 21 ff f4 	l.addi r1,r1,-12
    216c:	18 80 00 00 	l.movhi r4,0x0
    2170:	a8 84 00 00 	l.ori r4,r4,0x0
    2174:	bc 04 00 00 	l.sfeqi r4,0
    2178:	10 00 00 04 	l.bf 2188 <deregister_tm_clones+0x48>
    217c:	15 00 00 00 	l.nop 0x0
    2180:	48 00 20 00 	l.jalr r4
    2184:	a8 62 00 00 	l.ori r3,r2,0x0
    2188:	9c 21 00 0c 	l.addi r1,r1,12
    218c:	85 21 ff fc 	l.lwz r9,-4(r1)
    2190:	84 21 ff f4 	l.lwz r1,-12(r1)
    2194:	44 00 48 00 	l.jr r9
    2198:	84 41 ff f8 	l.lwz r2,-8(r1)

0000219c <register_tm_clones>:
    219c:	d7 e1 17 f8 	l.sw -8(r1),r2
    21a0:	18 60 00 00 	l.movhi r3,0x0
    21a4:	18 40 00 00 	l.movhi r2,0x0
    21a8:	a8 63 48 a4 	l.ori r3,r3,0x48a4
    21ac:	a8 42 48 a4 	l.ori r2,r2,0x48a4
    21b0:	d7 e1 4f fc 	l.sw -4(r1),r9
    21b4:	e0 63 10 02 	l.sub r3,r3,r2
    21b8:	d7 e1 0f f4 	l.sw -12(r1),r1
    21bc:	b8 63 00 82 	l.srai r3,r3,0x2
    21c0:	b8 83 00 5f 	l.srli r4,r3,0x1f
    21c4:	e0 84 18 00 	l.add r4,r4,r3
    21c8:	b8 84 00 81 	l.srai r4,r4,0x1
    21cc:	bc 04 00 00 	l.sfeqi r4,0
    21d0:	10 00 00 09 	l.bf 21f4 <register_tm_clones+0x58>
    21d4:	9c 21 ff f4 	l.addi r1,r1,-12
    21d8:	18 a0 00 00 	l.movhi r5,0x0
    21dc:	a8 a5 00 00 	l.ori r5,r5,0x0
    21e0:	bc 05 00 00 	l.sfeqi r5,0
    21e4:	10 00 00 04 	l.bf 21f4 <register_tm_clones+0x58>
    21e8:	15 00 00 00 	l.nop 0x0
    21ec:	48 00 28 00 	l.jalr r5
    21f0:	a8 62 00 00 	l.ori r3,r2,0x0
    21f4:	9c 21 00 0c 	l.addi r1,r1,12
    21f8:	85 21 ff fc 	l.lwz r9,-4(r1)
    21fc:	84 21 ff f4 	l.lwz r1,-12(r1)
    2200:	44 00 48 00 	l.jr r9
    2204:	84 41 ff f8 	l.lwz r2,-8(r1)

00002208 <__do_global_dtors_aux>:
    2208:	d7 e1 97 f8 	l.sw -8(r1),r18
    220c:	1a 40 00 00 	l.movhi r18,0x0
    2210:	d7 e1 17 f0 	l.sw -16(r1),r2
    2214:	aa 52 48 a4 	l.ori r18,r18,0x48a4
    2218:	d7 e1 4f fc 	l.sw -4(r1),r9
    221c:	8c 52 00 00 	l.lbz r2,0(r18)
    2220:	d7 e1 0f ec 	l.sw -20(r1),r1
    2224:	d7 e1 77 f4 	l.sw -12(r1),r14
    2228:	bc 22 00 00 	l.sfnei r2,0
    222c:	10 00 00 26 	l.bf 22c4 <__do_global_dtors_aux+0xbc>
    2230:	9c 21 ff ec 	l.addi r1,r1,-20
    2234:	19 c0 00 00 	l.movhi r14,0x0
    2238:	18 80 00 00 	l.movhi r4,0x0
    223c:	a9 ce 40 3c 	l.ori r14,r14,0x403c
    2240:	a8 84 40 38 	l.ori r4,r4,0x4038
    2244:	18 40 00 00 	l.movhi r2,0x0
    2248:	e1 ce 20 02 	l.sub r14,r14,r4
    224c:	a8 42 48 a8 	l.ori r2,r2,0x48a8
    2250:	b9 ce 00 82 	l.srai r14,r14,0x2
    2254:	84 62 00 00 	l.lwz r3,0(r2)
    2258:	9d ce ff ff 	l.addi r14,r14,-1
    225c:	e4 83 70 00 	l.sfltu r3,r14
    2260:	0c 00 00 0e 	l.bnf 2298 <__do_global_dtors_aux+0x90>
    2264:	9c 63 00 01 	l.addi r3,r3,1
    2268:	18 a0 00 00 	l.movhi r5,0x0
    226c:	b8 83 00 02 	l.slli r4,r3,0x2
    2270:	a8 a5 40 38 	l.ori r5,r5,0x4038
    2274:	d4 02 18 00 	l.sw 0(r2),r3
    2278:	e0 64 28 00 	l.add r3,r4,r5
    227c:	84 63 00 00 	l.lwz r3,0(r3)
    2280:	48 00 18 00 	l.jalr r3
    2284:	15 00 00 00 	l.nop 0x0
    2288:	84 62 00 00 	l.lwz r3,0(r2)
    228c:	e4 83 70 00 	l.sfltu r3,r14
    2290:	13 ff ff f6 	l.bf 2268 <__do_global_dtors_aux+0x60>
    2294:	9c 63 00 01 	l.addi r3,r3,1
    2298:	07 ff ff aa 	l.jal 2140 <deregister_tm_clones>
    229c:	18 40 00 00 	l.movhi r2,0x0
    22a0:	a8 42 00 00 	l.ori r2,r2,0x0
    22a4:	bc 22 00 00 	l.sfnei r2,0
    22a8:	0c 00 00 06 	l.bnf 22c0 <__do_global_dtors_aux+0xb8>
    22ac:	9c 40 00 01 	l.addi r2,r0,1
    22b0:	18 60 00 00 	l.movhi r3,0x0
    22b4:	07 ff f7 53 	l.jal 0 <_or1k_reset-0x100>
    22b8:	a8 63 40 00 	l.ori r3,r3,0x4000
    22bc:	9c 40 00 01 	l.addi r2,r0,1
    22c0:	d8 12 10 00 	l.sb 0(r18),r2
    22c4:	9c 21 00 14 	l.addi r1,r1,20
    22c8:	85 21 ff fc 	l.lwz r9,-4(r1)
    22cc:	84 21 ff ec 	l.lwz r1,-20(r1)
    22d0:	84 41 ff f0 	l.lwz r2,-16(r1)
    22d4:	85 c1 ff f4 	l.lwz r14,-12(r1)
    22d8:	44 00 48 00 	l.jr r9
    22dc:	86 41 ff f8 	l.lwz r18,-8(r1)

000022e0 <call___do_global_dtors_aux>:
    22e0:	d7 e1 4f fc 	l.sw -4(r1),r9
    22e4:	d7 e1 0f f8 	l.sw -8(r1),r1
    22e8:	9c 21 ff f8 	l.addi r1,r1,-8
    22ec:	9c 21 00 08 	l.addi r1,r1,8
    22f0:	85 21 ff fc 	l.lwz r9,-4(r1)
    22f4:	44 00 48 00 	l.jr r9
    22f8:	84 21 ff f8 	l.lwz r1,-8(r1)

000022fc <frame_dummy>:
    22fc:	18 60 00 00 	l.movhi r3,0x0
    2300:	d7 e1 4f fc 	l.sw -4(r1),r9
    2304:	a8 63 00 00 	l.ori r3,r3,0x0
    2308:	d7 e1 0f f8 	l.sw -8(r1),r1
    230c:	bc 03 00 00 	l.sfeqi r3,0
    2310:	10 00 00 07 	l.bf 232c <frame_dummy+0x30>
    2314:	9c 21 ff f8 	l.addi r1,r1,-8
    2318:	18 60 00 00 	l.movhi r3,0x0
    231c:	18 80 00 00 	l.movhi r4,0x0
    2320:	a8 63 40 00 	l.ori r3,r3,0x4000
    2324:	07 ff f7 37 	l.jal 0 <_or1k_reset-0x100>
    2328:	a8 84 48 ac 	l.ori r4,r4,0x48ac
    232c:	18 60 00 00 	l.movhi r3,0x0
    2330:	a8 63 40 40 	l.ori r3,r3,0x4040
    2334:	84 83 00 00 	l.lwz r4,0(r3)
    2338:	bc 04 00 00 	l.sfeqi r4,0
    233c:	0c 00 00 08 	l.bnf 235c <frame_dummy+0x60>
    2340:	18 80 00 00 	l.movhi r4,0x0
    2344:	07 ff ff 96 	l.jal 219c <register_tm_clones>
    2348:	15 00 00 00 	l.nop 0x0
    234c:	9c 21 00 08 	l.addi r1,r1,8
    2350:	85 21 ff fc 	l.lwz r9,-4(r1)
    2354:	44 00 48 00 	l.jr r9
    2358:	84 21 ff f8 	l.lwz r1,-8(r1)
    235c:	a8 84 00 00 	l.ori r4,r4,0x0
    2360:	bc 04 00 00 	l.sfeqi r4,0
    2364:	13 ff ff f8 	l.bf 2344 <frame_dummy+0x48>
    2368:	15 00 00 00 	l.nop 0x0
    236c:	48 00 20 00 	l.jalr r4
    2370:	15 00 00 00 	l.nop 0x0
    2374:	03 ff ff f4 	l.j 2344 <frame_dummy+0x48>
    2378:	15 00 00 00 	l.nop 0x0

0000237c <call_frame_dummy>:
    237c:	d7 e1 4f fc 	l.sw -4(r1),r9
    2380:	d7 e1 0f f8 	l.sw -8(r1),r1
    2384:	9c 21 ff f8 	l.addi r1,r1,-8
    2388:	9c 21 00 08 	l.addi r1,r1,8
    238c:	85 21 ff fc 	l.lwz r9,-4(r1)
    2390:	44 00 48 00 	l.jr r9
    2394:	84 21 ff f8 	l.lwz r1,-8(r1)

00002398 <main>:
    2398:	d7 e1 17 fc 	l.sw -4(r1),r2
    239c:	9c 41 00 00 	l.addi r2,r1,0
    23a0:	9c 21 ff f4 	l.addi r1,r1,-12
    23a4:	d7 e2 1f f8 	l.sw -8(r2),r3
    23a8:	d7 e2 27 f4 	l.sw -12(r2),r4
    23ac:	18 60 91 00 	l.movhi r3,0x9100
    23b0:	a8 63 20 04 	l.ori r3,r3,0x2004
    23b4:	9c 80 ff ff 	l.addi r4,r0,-1
    23b8:	d4 03 20 00 	l.sw 0(r3),r4
    23bc:	18 60 91 00 	l.movhi r3,0x9100
    23c0:	a8 63 20 00 	l.ori r3,r3,0x2000
    23c4:	9c 80 ff ff 	l.addi r4,r0,-1
    23c8:	d4 03 20 00 	l.sw 0(r3),r4
    23cc:	9c 60 00 00 	l.addi r3,r0,0
    23d0:	a9 63 00 00 	l.ori r11,r3,0x0
    23d4:	a8 22 00 00 	l.ori r1,r2,0x0
    23d8:	84 41 ff fc 	l.lwz r2,-4(r1)
    23dc:	44 00 48 00 	l.jr r9
    23e0:	15 00 00 00 	l.nop 0x0

000023e4 <atexit>:
    23e4:	a8 83 00 00 	l.ori r4,r3,0x0
    23e8:	9c 60 00 00 	l.addi r3,r0,0
    23ec:	d7 e1 4f fc 	l.sw -4(r1),r9
    23f0:	d7 e1 0f f8 	l.sw -8(r1),r1
    23f4:	a8 a3 00 00 	l.ori r5,r3,0x0
    23f8:	9c 21 ff f8 	l.addi r1,r1,-8
    23fc:	04 00 00 18 	l.jal 245c <__register_exitproc>
    2400:	a8 c3 00 00 	l.ori r6,r3,0x0
    2404:	9c 21 00 08 	l.addi r1,r1,8
    2408:	85 21 ff fc 	l.lwz r9,-4(r1)
    240c:	44 00 48 00 	l.jr r9
    2410:	84 21 ff f8 	l.lwz r1,-8(r1)

00002414 <exit>:
    2414:	d7 e1 17 f8 	l.sw -8(r1),r2
    2418:	d7 e1 4f fc 	l.sw -4(r1),r9
    241c:	d7 e1 0f f4 	l.sw -12(r1),r1
    2420:	9c 80 00 00 	l.addi r4,r0,0
    2424:	9c 21 ff f4 	l.addi r1,r1,-12
    2428:	04 00 00 5d 	l.jal 259c <__call_exitprocs>
    242c:	a8 43 00 00 	l.ori r2,r3,0x0
    2430:	18 60 00 00 	l.movhi r3,0x0
    2434:	a8 63 3a 34 	l.ori r3,r3,0x3a34
    2438:	84 63 00 00 	l.lwz r3,0(r3)
    243c:	84 83 00 3c 	l.lwz r4,60(r3)
    2440:	bc 04 00 00 	l.sfeqi r4,0
    2444:	10 00 00 04 	l.bf 2454 <exit+0x40>
    2448:	15 00 00 00 	l.nop 0x0
    244c:	48 00 20 00 	l.jalr r4
    2450:	15 00 00 00 	l.nop 0x0
    2454:	04 00 00 f2 	l.jal 281c <_exit>
    2458:	a8 62 00 00 	l.ori r3,r2,0x0

0000245c <__register_exitproc>:
    245c:	d7 e1 17 e8 	l.sw -24(r1),r2
    2460:	18 40 00 00 	l.movhi r2,0x0
    2464:	d7 e1 77 ec 	l.sw -20(r1),r14
    2468:	a8 42 3a 34 	l.ori r2,r2,0x3a34
    246c:	d7 e1 97 f0 	l.sw -16(r1),r18
    2470:	85 c2 00 00 	l.lwz r14,0(r2)
    2474:	d7 e1 a7 f4 	l.sw -12(r1),r20
    2478:	85 0e 01 48 	l.lwz r8,328(r14)
    247c:	d7 e1 b7 f8 	l.sw -8(r1),r22
    2480:	d7 e1 4f fc 	l.sw -4(r1),r9
    2484:	d7 e1 0f e4 	l.sw -28(r1),r1
    2488:	bc 28 00 00 	l.sfnei r8,0
    248c:	9c 21 ff e4 	l.addi r1,r1,-28
    2490:	aa c3 00 00 	l.ori r22,r3,0x0
    2494:	a8 44 00 00 	l.ori r2,r4,0x0
    2498:	aa 85 00 00 	l.ori r20,r5,0x0
    249c:	0c 00 00 3b 	l.bnf 2588 <__register_exitproc+0x12c>
    24a0:	aa 46 00 00 	l.ori r18,r6,0x0
    24a4:	84 e8 00 04 	l.lwz r7,4(r8)
    24a8:	bd 47 00 1f 	l.sfgtsi r7,31
    24ac:	10 00 00 14 	l.bf 24fc <__register_exitproc+0xa0>
    24b0:	18 60 00 00 	l.movhi r3,0x0
    24b4:	bc 36 00 00 	l.sfnei r22,0
    24b8:	10 00 00 25 	l.bf 254c <__register_exitproc+0xf0>
    24bc:	9c a7 00 01 	l.addi r5,r7,1
    24c0:	9c e7 00 02 	l.addi r7,r7,2
    24c4:	d4 08 28 04 	l.sw 4(r8),r5
    24c8:	b8 e7 00 02 	l.slli r7,r7,0x2
    24cc:	9d 60 00 00 	l.addi r11,r0,0
    24d0:	e0 e8 38 00 	l.add r7,r8,r7
    24d4:	d4 07 10 00 	l.sw 0(r7),r2
    24d8:	9c 21 00 1c 	l.addi r1,r1,28
    24dc:	85 21 ff fc 	l.lwz r9,-4(r1)
    24e0:	84 21 ff e4 	l.lwz r1,-28(r1)
    24e4:	84 41 ff e8 	l.lwz r2,-24(r1)
    24e8:	85 c1 ff ec 	l.lwz r14,-20(r1)
    24ec:	86 41 ff f0 	l.lwz r18,-16(r1)
    24f0:	86 81 ff f4 	l.lwz r20,-12(r1)
    24f4:	44 00 48 00 	l.jr r9
    24f8:	86 c1 ff f8 	l.lwz r22,-8(r1)
    24fc:	a8 63 00 00 	l.ori r3,r3,0x0
    2500:	bc 23 00 00 	l.sfnei r3,0
    2504:	0f ff ff f5 	l.bnf 24d8 <__register_exitproc+0x7c>
    2508:	9d 60 ff ff 	l.addi r11,r0,-1
    250c:	07 ff f6 bd 	l.jal 0 <_or1k_reset-0x100>
    2510:	9c 60 01 90 	l.addi r3,r0,400
    2514:	bc 0b 00 00 	l.sfeqi r11,0
    2518:	10 00 00 1f 	l.bf 2594 <__register_exitproc+0x138>
    251c:	a9 0b 00 00 	l.ori r8,r11,0x0
    2520:	84 6e 01 48 	l.lwz r3,328(r14)
    2524:	9c 80 00 00 	l.addi r4,r0,0
    2528:	d4 0b 18 00 	l.sw 0(r11),r3
    252c:	d4 0b 20 04 	l.sw 4(r11),r4
    2530:	d4 0e 59 48 	l.sw 328(r14),r11
    2534:	d4 0b 21 88 	l.sw 392(r11),r4
    2538:	d4 0b 21 8c 	l.sw 396(r11),r4
    253c:	bc 36 00 00 	l.sfnei r22,0
    2540:	9c a0 00 01 	l.addi r5,r0,1
    2544:	0f ff ff df 	l.bnf 24c0 <__register_exitproc+0x64>
    2548:	a8 e4 00 00 	l.ori r7,r4,0x0
    254c:	b9 67 00 02 	l.slli r11,r7,0x2
    2550:	9c 80 00 01 	l.addi r4,r0,1
    2554:	bc 36 00 02 	l.sfnei r22,2
    2558:	e0 68 58 00 	l.add r3,r8,r11
    255c:	e0 84 38 08 	l.sll r4,r4,r7
    2560:	d4 03 a0 88 	l.sw 136(r3),r20
    2564:	84 c8 01 88 	l.lwz r6,392(r8)
    2568:	e0 c6 20 04 	l.or r6,r6,r4
    256c:	d4 08 31 88 	l.sw 392(r8),r6
    2570:	13 ff ff d4 	l.bf 24c0 <__register_exitproc+0x64>
    2574:	d4 03 91 08 	l.sw 264(r3),r18
    2578:	84 68 01 8c 	l.lwz r3,396(r8)
    257c:	e0 83 20 04 	l.or r4,r3,r4
    2580:	03 ff ff d0 	l.j 24c0 <__register_exitproc+0x64>
    2584:	d4 08 21 8c 	l.sw 396(r8),r4
    2588:	9d 0e 01 4c 	l.addi r8,r14,332
    258c:	03 ff ff c6 	l.j 24a4 <__register_exitproc+0x48>
    2590:	d4 0e 41 48 	l.sw 328(r14),r8
    2594:	03 ff ff d1 	l.j 24d8 <__register_exitproc+0x7c>
    2598:	9d 60 ff ff 	l.addi r11,r0,-1

0000259c <__call_exitprocs>:
    259c:	d7 e1 17 d8 	l.sw -40(r1),r2
    25a0:	18 40 00 00 	l.movhi r2,0x0
    25a4:	d7 e1 f7 f8 	l.sw -8(r1),r30
    25a8:	a8 42 3a 34 	l.ori r2,r2,0x3a34
    25ac:	d7 e1 97 e0 	l.sw -32(r1),r18
    25b0:	87 c2 00 00 	l.lwz r30,0(r2)
    25b4:	d7 e1 d7 f0 	l.sw -16(r1),r26
    25b8:	d7 e1 e7 f4 	l.sw -12(r1),r28
    25bc:	d7 e1 4f fc 	l.sw -4(r1),r9
    25c0:	d7 e1 0f d4 	l.sw -44(r1),r1
    25c4:	d7 e1 77 dc 	l.sw -36(r1),r14
    25c8:	d7 e1 a7 e4 	l.sw -28(r1),r20
    25cc:	d7 e1 b7 e8 	l.sw -24(r1),r22
    25d0:	d7 e1 c7 ec 	l.sw -20(r1),r24
    25d4:	9c 5e 01 48 	l.addi r2,r30,328
    25d8:	9c 21 ff d0 	l.addi r1,r1,-48
    25dc:	1b 80 00 00 	l.movhi r28,0x0
    25e0:	ab 43 00 00 	l.ori r26,r3,0x0
    25e4:	aa 44 00 00 	l.ori r18,r4,0x0
    25e8:	d4 01 10 00 	l.sw 0(r1),r2
    25ec:	ab 9c 00 00 	l.ori r28,r28,0x0
    25f0:	86 9e 01 48 	l.lwz r20,328(r30)
    25f4:	bc 34 00 00 	l.sfnei r20,0
    25f8:	0c 00 00 43 	l.bnf 2704 <__call_exitprocs+0x168>
    25fc:	86 c1 00 00 	l.lwz r22,0(r1)
    2600:	84 54 00 04 	l.lwz r2,4(r20)
    2604:	9d c2 ff ff 	l.addi r14,r2,-1
    2608:	bd 6e 00 00 	l.sfgesi r14,0
    260c:	0c 00 00 2f 	l.bnf 26c8 <__call_exitprocs+0x12c>
    2610:	bc 1c 00 00 	l.sfeqi r28,0
    2614:	9c 42 00 01 	l.addi r2,r2,1
    2618:	b8 42 00 02 	l.slli r2,r2,0x2
    261c:	00 00 00 0a 	l.j 2644 <__call_exitprocs+0xa8>
    2620:	e0 54 10 00 	l.add r2,r20,r2
    2624:	84 62 01 00 	l.lwz r3,256(r2)
    2628:	e4 03 90 00 	l.sfeq r3,r18
    262c:	10 00 00 09 	l.bf 2650 <__call_exitprocs+0xb4>
    2630:	15 00 00 00 	l.nop 0x0
    2634:	9d ce ff ff 	l.addi r14,r14,-1
    2638:	bc 2e ff ff 	l.sfnei r14,-1
    263c:	0c 00 00 22 	l.bnf 26c4 <__call_exitprocs+0x128>
    2640:	9c 42 ff fc 	l.addi r2,r2,-4
    2644:	bc 12 00 00 	l.sfeqi r18,0
    2648:	0f ff ff f7 	l.bnf 2624 <__call_exitprocs+0x88>
    264c:	15 00 00 00 	l.nop 0x0
    2650:	84 74 00 04 	l.lwz r3,4(r20)
    2654:	9c 63 ff ff 	l.addi r3,r3,-1
    2658:	e4 23 70 00 	l.sfne r3,r14
    265c:	0c 00 00 41 	l.bnf 2760 <__call_exitprocs+0x1c4>
    2660:	84 a2 00 00 	l.lwz r5,0(r2)
    2664:	9c 60 00 00 	l.addi r3,r0,0
    2668:	d4 02 18 00 	l.sw 0(r2),r3
    266c:	bc 05 00 00 	l.sfeqi r5,0
    2670:	13 ff ff f1 	l.bf 2634 <__call_exitprocs+0x98>
    2674:	9c 60 00 01 	l.addi r3,r0,1
    2678:	e0 83 70 08 	l.sll r4,r3,r14
    267c:	84 74 01 88 	l.lwz r3,392(r20)
    2680:	e0 64 18 03 	l.and r3,r4,r3
    2684:	bc 03 00 00 	l.sfeqi r3,0
    2688:	0c 00 00 2c 	l.bnf 2738 <__call_exitprocs+0x19c>
    268c:	87 14 00 04 	l.lwz r24,4(r20)
    2690:	48 00 28 00 	l.jalr r5
    2694:	15 00 00 00 	l.nop 0x0
    2698:	84 74 00 04 	l.lwz r3,4(r20)
    269c:	e4 23 c0 00 	l.sfne r3,r24
    26a0:	13 ff ff d4 	l.bf 25f0 <__call_exitprocs+0x54>
    26a4:	15 00 00 00 	l.nop 0x0
    26a8:	84 76 00 00 	l.lwz r3,0(r22)
    26ac:	e4 23 a0 00 	l.sfne r3,r20
    26b0:	13 ff ff d0 	l.bf 25f0 <__call_exitprocs+0x54>
    26b4:	9d ce ff ff 	l.addi r14,r14,-1
    26b8:	bc 2e ff ff 	l.sfnei r14,-1
    26bc:	13 ff ff e2 	l.bf 2644 <__call_exitprocs+0xa8>
    26c0:	9c 42 ff fc 	l.addi r2,r2,-4
    26c4:	bc 1c 00 00 	l.sfeqi r28,0
    26c8:	10 00 00 0f 	l.bf 2704 <__call_exitprocs+0x168>
    26cc:	15 00 00 00 	l.nop 0x0
    26d0:	84 54 00 04 	l.lwz r2,4(r20)
    26d4:	bc 22 00 00 	l.sfnei r2,0
    26d8:	10 00 00 28 	l.bf 2778 <__call_exitprocs+0x1dc>
    26dc:	84 54 00 00 	l.lwz r2,0(r20)
    26e0:	bc 02 00 00 	l.sfeqi r2,0
    26e4:	10 00 00 25 	l.bf 2778 <__call_exitprocs+0x1dc>
    26e8:	a8 74 00 00 	l.ori r3,r20,0x0
    26ec:	07 ff f6 45 	l.jal 0 <_or1k_reset-0x100>
    26f0:	d4 16 10 00 	l.sw 0(r22),r2
    26f4:	86 96 00 00 	l.lwz r20,0(r22)
    26f8:	bc 34 00 00 	l.sfnei r20,0
    26fc:	13 ff ff c1 	l.bf 2600 <__call_exitprocs+0x64>
    2700:	15 00 00 00 	l.nop 0x0
    2704:	9c 21 00 30 	l.addi r1,r1,48
    2708:	85 21 ff fc 	l.lwz r9,-4(r1)
    270c:	84 21 ff d4 	l.lwz r1,-44(r1)
    2710:	84 41 ff d8 	l.lwz r2,-40(r1)
    2714:	85 c1 ff dc 	l.lwz r14,-36(r1)
    2718:	86 41 ff e0 	l.lwz r18,-32(r1)
    271c:	86 81 ff e4 	l.lwz r20,-28(r1)
    2720:	86 c1 ff e8 	l.lwz r22,-24(r1)
    2724:	87 01 ff ec 	l.lwz r24,-20(r1)
    2728:	87 41 ff f0 	l.lwz r26,-16(r1)
    272c:	87 81 ff f4 	l.lwz r28,-12(r1)
    2730:	44 00 48 00 	l.jr r9
    2734:	87 c1 ff f8 	l.lwz r30,-8(r1)
    2738:	84 74 01 8c 	l.lwz r3,396(r20)
    273c:	e0 64 18 03 	l.and r3,r4,r3
    2740:	bc 23 00 00 	l.sfnei r3,0
    2744:	10 00 00 09 	l.bf 2768 <__call_exitprocs+0x1cc>
    2748:	15 00 00 00 	l.nop 0x0
    274c:	a8 7a 00 00 	l.ori r3,r26,0x0
    2750:	48 00 28 00 	l.jalr r5
    2754:	84 82 00 80 	l.lwz r4,128(r2)
    2758:	03 ff ff d1 	l.j 269c <__call_exitprocs+0x100>
    275c:	84 74 00 04 	l.lwz r3,4(r20)
    2760:	03 ff ff c3 	l.j 266c <__call_exitprocs+0xd0>
    2764:	d4 14 70 04 	l.sw 4(r20),r14
    2768:	48 00 28 00 	l.jalr r5
    276c:	84 62 00 80 	l.lwz r3,128(r2)
    2770:	03 ff ff cb 	l.j 269c <__call_exitprocs+0x100>
    2774:	84 74 00 04 	l.lwz r3,4(r20)
    2778:	aa d4 00 00 	l.ori r22,r20,0x0
    277c:	03 ff ff df 	l.j 26f8 <__call_exitprocs+0x15c>
    2780:	aa 82 00 00 	l.ori r20,r2,0x0

00002784 <_write_r>:
    2784:	d7 e1 17 f0 	l.sw -16(r1),r2
    2788:	d7 e1 77 f4 	l.sw -12(r1),r14
    278c:	d7 e1 97 f8 	l.sw -8(r1),r18
    2790:	d7 e1 4f fc 	l.sw -4(r1),r9
    2794:	d7 e1 0f ec 	l.sw -20(r1),r1
    2798:	bc 26 00 00 	l.sfnei r6,0
    279c:	9c 21 ff ec 	l.addi r1,r1,-20
    27a0:	aa 46 00 00 	l.ori r18,r6,0x0
    27a4:	a8 45 00 00 	l.ori r2,r5,0x0
    27a8:	10 00 00 09 	l.bf 27cc <_write_r+0x48>
    27ac:	e1 c5 30 00 	l.add r14,r5,r6
    27b0:	00 00 00 14 	l.j 2800 <_write_r+0x7c>
    27b4:	9c 21 00 14 	l.addi r1,r1,20
    27b8:	04 00 01 0a 	l.jal 2be0 <_or1k_outbyte>
    27bc:	9c 42 00 01 	l.addi r2,r2,1
    27c0:	e4 22 70 00 	l.sfne r2,r14
    27c4:	0c 00 00 0e 	l.bnf 27fc <_write_r+0x78>
    27c8:	15 00 00 00 	l.nop 0x0
    27cc:	90 62 00 00 	l.lbs r3,0(r2)
    27d0:	bc 23 00 0a 	l.sfnei r3,10
    27d4:	13 ff ff f9 	l.bf 27b8 <_write_r+0x34>
    27d8:	15 00 00 00 	l.nop 0x0
    27dc:	9c 60 00 0d 	l.addi r3,r0,13
    27e0:	04 00 01 00 	l.jal 2be0 <_or1k_outbyte>
    27e4:	9c 42 00 01 	l.addi r2,r2,1
    27e8:	04 00 00 fe 	l.jal 2be0 <_or1k_outbyte>
    27ec:	90 62 ff ff 	l.lbs r3,-1(r2)
    27f0:	e4 22 70 00 	l.sfne r2,r14
    27f4:	13 ff ff f6 	l.bf 27cc <_write_r+0x48>
    27f8:	15 00 00 00 	l.nop 0x0
    27fc:	9c 21 00 14 	l.addi r1,r1,20
    2800:	a9 72 00 00 	l.ori r11,r18,0x0
    2804:	85 21 ff fc 	l.lwz r9,-4(r1)
    2808:	84 21 ff ec 	l.lwz r1,-20(r1)
    280c:	84 41 ff f0 	l.lwz r2,-16(r1)
    2810:	85 c1 ff f4 	l.lwz r14,-12(r1)
    2814:	44 00 48 00 	l.jr r9
    2818:	86 41 ff f8 	l.lwz r18,-8(r1)

0000281c <_exit>:
    281c:	d7 e1 4f fc 	l.sw -4(r1),r9
    2820:	d7 e1 0f f8 	l.sw -8(r1),r1
    2824:	04 00 03 b7 	l.jal 3700 <_or1k_board_exit>
    2828:	9c 21 ff f8 	l.addi r1,r1,-8
    282c:	00 00 00 00 	l.j 282c <_exit+0x10>
    2830:	15 00 00 00 	l.nop 0x0

00002834 <_close_r>:
    2834:	d7 e1 0f fc 	l.sw -4(r1),r1
    2838:	9c 21 ff fc 	l.addi r1,r1,-4
    283c:	9c 80 00 58 	l.addi r4,r0,88
    2840:	9c 21 00 04 	l.addi r1,r1,4
    2844:	9d 60 ff ff 	l.addi r11,r0,-1
    2848:	d4 03 20 00 	l.sw 0(r3),r4
    284c:	44 00 48 00 	l.jr r9
    2850:	84 21 ff fc 	l.lwz r1,-4(r1)

00002854 <_execve_r>:
    2854:	d7 e1 0f fc 	l.sw -4(r1),r1
    2858:	9c 21 ff fc 	l.addi r1,r1,-4
    285c:	9c 80 00 58 	l.addi r4,r0,88
    2860:	9c 21 00 04 	l.addi r1,r1,4
    2864:	9d 60 ff ff 	l.addi r11,r0,-1
    2868:	d4 03 20 00 	l.sw 0(r3),r4
    286c:	44 00 48 00 	l.jr r9
    2870:	84 21 ff fc 	l.lwz r1,-4(r1)

00002874 <_fork_r>:
    2874:	d7 e1 4f fc 	l.sw -4(r1),r9
    2878:	d7 e1 0f f8 	l.sw -8(r1),r1
    287c:	04 00 03 a8 	l.jal 371c <__errno>
    2880:	9c 21 ff f8 	l.addi r1,r1,-8
    2884:	9c 60 00 58 	l.addi r3,r0,88
    2888:	d4 0b 18 00 	l.sw 0(r11),r3
    288c:	9c 21 00 08 	l.addi r1,r1,8
    2890:	9d 60 ff ff 	l.addi r11,r0,-1
    2894:	85 21 ff fc 	l.lwz r9,-4(r1)
    2898:	44 00 48 00 	l.jr r9
    289c:	84 21 ff f8 	l.lwz r1,-8(r1)

000028a0 <_fstat_r>:
    28a0:	d7 e1 0f fc 	l.sw -4(r1),r1
    28a4:	9c 21 ff fc 	l.addi r1,r1,-4
    28a8:	9c 80 00 58 	l.addi r4,r0,88
    28ac:	9c 21 00 04 	l.addi r1,r1,4
    28b0:	9d 60 ff ff 	l.addi r11,r0,-1
    28b4:	d4 03 20 00 	l.sw 0(r3),r4
    28b8:	44 00 48 00 	l.jr r9
    28bc:	84 21 ff fc 	l.lwz r1,-4(r1)

000028c0 <_getpid_r>:
    28c0:	d7 e1 0f fc 	l.sw -4(r1),r1
    28c4:	9c 21 ff fc 	l.addi r1,r1,-4
    28c8:	9c 80 00 58 	l.addi r4,r0,88
    28cc:	9c 21 00 04 	l.addi r1,r1,4
    28d0:	9d 60 ff ff 	l.addi r11,r0,-1
    28d4:	d4 03 20 00 	l.sw 0(r3),r4
    28d8:	44 00 48 00 	l.jr r9
    28dc:	84 21 ff fc 	l.lwz r1,-4(r1)

000028e0 <_gettimeofday>:
    28e0:	d7 e1 0f fc 	l.sw -4(r1),r1
    28e4:	9c 21 ff fc 	l.addi r1,r1,-4
    28e8:	9c 80 00 58 	l.addi r4,r0,88
    28ec:	9c 21 00 04 	l.addi r1,r1,4
    28f0:	9d 60 ff ff 	l.addi r11,r0,-1
    28f4:	d4 03 20 00 	l.sw 0(r3),r4
    28f8:	44 00 48 00 	l.jr r9
    28fc:	84 21 ff fc 	l.lwz r1,-4(r1)

00002900 <_isatty_r>:
    2900:	d7 e1 0f fc 	l.sw -4(r1),r1
    2904:	9c 21 ff fc 	l.addi r1,r1,-4
    2908:	9c 80 00 58 	l.addi r4,r0,88
    290c:	9c 21 00 04 	l.addi r1,r1,4
    2910:	9d 60 00 00 	l.addi r11,r0,0
    2914:	d4 03 20 00 	l.sw 0(r3),r4
    2918:	44 00 48 00 	l.jr r9
    291c:	84 21 ff fc 	l.lwz r1,-4(r1)

00002920 <_kill_r>:
    2920:	d7 e1 0f fc 	l.sw -4(r1),r1
    2924:	9c 21 ff fc 	l.addi r1,r1,-4
    2928:	9c 80 00 58 	l.addi r4,r0,88
    292c:	9c 21 00 04 	l.addi r1,r1,4
    2930:	9d 60 ff ff 	l.addi r11,r0,-1
    2934:	d4 03 20 00 	l.sw 0(r3),r4
    2938:	44 00 48 00 	l.jr r9
    293c:	84 21 ff fc 	l.lwz r1,-4(r1)

00002940 <_link_r>:
    2940:	d7 e1 0f fc 	l.sw -4(r1),r1
    2944:	9c 21 ff fc 	l.addi r1,r1,-4
    2948:	9c 80 00 58 	l.addi r4,r0,88
    294c:	9c 21 00 04 	l.addi r1,r1,4
    2950:	9d 60 ff ff 	l.addi r11,r0,-1
    2954:	d4 03 20 00 	l.sw 0(r3),r4
    2958:	44 00 48 00 	l.jr r9
    295c:	84 21 ff fc 	l.lwz r1,-4(r1)

00002960 <_lseek_r>:
    2960:	d7 e1 4f fc 	l.sw -4(r1),r9
    2964:	d7 e1 0f f8 	l.sw -8(r1),r1
    2968:	04 00 03 6d 	l.jal 371c <__errno>
    296c:	9c 21 ff f8 	l.addi r1,r1,-8
    2970:	9c 60 00 58 	l.addi r3,r0,88
    2974:	d4 0b 18 00 	l.sw 0(r11),r3
    2978:	9c 21 00 08 	l.addi r1,r1,8
    297c:	9d 60 ff ff 	l.addi r11,r0,-1
    2980:	85 21 ff fc 	l.lwz r9,-4(r1)
    2984:	44 00 48 00 	l.jr r9
    2988:	84 21 ff f8 	l.lwz r1,-8(r1)

0000298c <_open>:
    298c:	d7 e1 0f fc 	l.sw -4(r1),r1
    2990:	9c 21 ff fc 	l.addi r1,r1,-4
    2994:	9c 80 00 58 	l.addi r4,r0,88
    2998:	9c 21 00 04 	l.addi r1,r1,4
    299c:	9d 60 ff ff 	l.addi r11,r0,-1
    29a0:	d4 03 20 00 	l.sw 0(r3),r4
    29a4:	44 00 48 00 	l.jr r9
    29a8:	84 21 ff fc 	l.lwz r1,-4(r1)

000029ac <_read_r>:
    29ac:	d7 e1 0f fc 	l.sw -4(r1),r1
    29b0:	9c 21 ff fc 	l.addi r1,r1,-4
    29b4:	9c 80 00 58 	l.addi r4,r0,88
    29b8:	9c 21 00 04 	l.addi r1,r1,4
    29bc:	9d 60 ff ff 	l.addi r11,r0,-1
    29c0:	d4 03 20 00 	l.sw 0(r3),r4
    29c4:	44 00 48 00 	l.jr r9
    29c8:	84 21 ff fc 	l.lwz r1,-4(r1)

000029cc <_readlink_r>:
    29cc:	d7 e1 0f fc 	l.sw -4(r1),r1
    29d0:	9c 21 ff fc 	l.addi r1,r1,-4
    29d4:	9c 80 00 58 	l.addi r4,r0,88
    29d8:	9c 21 00 04 	l.addi r1,r1,4
    29dc:	9d 60 ff ff 	l.addi r11,r0,-1
    29e0:	d4 03 20 00 	l.sw 0(r3),r4
    29e4:	44 00 48 00 	l.jr r9
    29e8:	84 21 ff fc 	l.lwz r1,-4(r1)

000029ec <_stat_r>:
    29ec:	d7 e1 0f fc 	l.sw -4(r1),r1
    29f0:	9c 21 ff fc 	l.addi r1,r1,-4
    29f4:	9c 80 00 05 	l.addi r4,r0,5
    29f8:	9c 21 00 04 	l.addi r1,r1,4
    29fc:	9d 60 ff ff 	l.addi r11,r0,-1
    2a00:	d4 03 20 00 	l.sw 0(r3),r4
    2a04:	44 00 48 00 	l.jr r9
    2a08:	84 21 ff fc 	l.lwz r1,-4(r1)

00002a0c <_unlink_r>:
    2a0c:	d7 e1 0f fc 	l.sw -4(r1),r1
    2a10:	9c 21 ff fc 	l.addi r1,r1,-4
    2a14:	9c 80 00 05 	l.addi r4,r0,5
    2a18:	9c 21 00 04 	l.addi r1,r1,4
    2a1c:	9d 60 ff ff 	l.addi r11,r0,-1
    2a20:	d4 03 20 00 	l.sw 0(r3),r4
    2a24:	44 00 48 00 	l.jr r9
    2a28:	84 21 ff fc 	l.lwz r1,-4(r1)

00002a2c <_or1k_uart_interrupt_handler>:
    2a2c:	18 60 00 00 	l.movhi r3,0x0
    2a30:	d7 e1 0f fc 	l.sw -4(r1),r1
    2a34:	a8 63 36 f4 	l.ori r3,r3,0x36f4
    2a38:	9c 21 ff fc 	l.addi r1,r1,-4
    2a3c:	84 63 00 00 	l.lwz r3,0(r3)
    2a40:	9c 63 00 02 	l.addi r3,r3,2
    2a44:	8c 63 00 00 	l.lbz r3,0(r3)
    2a48:	9c 21 00 04 	l.addi r1,r1,4
    2a4c:	44 00 48 00 	l.jr r9
    2a50:	84 21 ff fc 	l.lwz r1,-4(r1)

00002a54 <_or1k_uart_init>:
    2a54:	d7 e1 17 f4 	l.sw -12(r1),r2
    2a58:	18 40 00 00 	l.movhi r2,0x0
    2a5c:	d7 e1 77 f8 	l.sw -8(r1),r14
    2a60:	a8 42 36 f4 	l.ori r2,r2,0x36f4
    2a64:	d7 e1 4f fc 	l.sw -4(r1),r9
    2a68:	85 c2 00 00 	l.lwz r14,0(r2)
    2a6c:	d7 e1 0f f0 	l.sw -16(r1),r1
    2a70:	bc 0e 00 00 	l.sfeqi r14,0
    2a74:	10 00 00 2c 	l.bf 2b24 <_or1k_uart_init+0xd0>
    2a78:	9c 21 ff f0 	l.addi r1,r1,-16
    2a7c:	18 60 00 00 	l.movhi r3,0x0
    2a80:	18 a0 00 00 	l.movhi r5,0x0
    2a84:	a8 63 36 f8 	l.ori r3,r3,0x36f8
    2a88:	9c c0 00 00 	l.addi r6,r0,0
    2a8c:	84 83 00 00 	l.lwz r4,0(r3)
    2a90:	18 60 00 00 	l.movhi r3,0x0
    2a94:	a8 a5 48 c8 	l.ori r5,r5,0x48c8
    2a98:	a8 63 36 f0 	l.ori r3,r3,0x36f0
    2a9c:	b8 84 00 04 	l.slli r4,r4,0x4
    2aa0:	84 63 00 00 	l.lwz r3,0(r3)
    2aa4:	d4 05 30 00 	l.sw 0(r5),r6
    2aa8:	04 00 03 81 	l.jal 38ac <__udivsi3>
    2aac:	9d ce 00 03 	l.addi r14,r14,3
    2ab0:	9c 60 ff 80 	l.addi r3,r0,-128
    2ab4:	a4 8b 00 ff 	l.andi r4,r11,0xff
    2ab8:	d8 0e 18 00 	l.sb 0(r14),r3
    2abc:	a5 6b ff ff 	l.andi r11,r11,0xffff
    2ac0:	84 62 00 00 	l.lwz r3,0(r2)
    2ac4:	9c c0 ff c3 	l.addi r6,r0,-61
    2ac8:	d8 03 20 00 	l.sb 0(r3),r4
    2acc:	b8 6b 00 48 	l.srli r3,r11,0x8
    2ad0:	84 82 00 00 	l.lwz r4,0(r2)
    2ad4:	9d 60 00 00 	l.addi r11,r0,0
    2ad8:	9c 84 00 01 	l.addi r4,r4,1
    2adc:	d8 04 18 00 	l.sb 0(r4),r3
    2ae0:	9c 80 00 03 	l.addi r4,r0,3
    2ae4:	84 62 00 00 	l.lwz r3,0(r2)
    2ae8:	9c 63 00 03 	l.addi r3,r3,3
    2aec:	d8 03 20 00 	l.sb 0(r3),r4
    2af0:	84 62 00 00 	l.lwz r3,0(r2)
    2af4:	9c 63 00 02 	l.addi r3,r3,2
    2af8:	d8 03 30 00 	l.sb 0(r3),r6
    2afc:	9c 60 00 00 	l.addi r3,r0,0
    2b00:	84 42 00 00 	l.lwz r2,0(r2)
    2b04:	9c 42 00 01 	l.addi r2,r2,1
    2b08:	d8 02 18 00 	l.sb 0(r2),r3
    2b0c:	9c 21 00 10 	l.addi r1,r1,16
    2b10:	85 21 ff fc 	l.lwz r9,-4(r1)
    2b14:	84 21 ff f0 	l.lwz r1,-16(r1)
    2b18:	84 41 ff f4 	l.lwz r2,-12(r1)
    2b1c:	44 00 48 00 	l.jr r9
    2b20:	85 c1 ff f8 	l.lwz r14,-8(r1)
    2b24:	03 ff ff fa 	l.j 2b0c <_or1k_uart_init+0xb8>
    2b28:	9d 60 ff ff 	l.addi r11,r0,-1

00002b2c <_or1k_uart_write>:
    2b2c:	18 80 00 00 	l.movhi r4,0x0
    2b30:	b8 63 00 18 	l.slli r3,r3,0x18
    2b34:	a8 84 36 f4 	l.ori r4,r4,0x36f4
    2b38:	d7 e1 0f fc 	l.sw -4(r1),r1
    2b3c:	84 c4 00 00 	l.lwz r6,0(r4)
    2b40:	b8 63 00 98 	l.srai r3,r3,0x18
    2b44:	9c 21 ff fc 	l.addi r1,r1,-4
    2b48:	9c a6 00 05 	l.addi r5,r6,5
    2b4c:	8c 85 00 00 	l.lbz r4,0(r5)
    2b50:	a4 84 00 20 	l.andi r4,r4,0x20
    2b54:	bc 04 00 00 	l.sfeqi r4,0
    2b58:	13 ff ff fd 	l.bf 2b4c <_or1k_uart_write+0x20>
    2b5c:	15 00 00 00 	l.nop 0x0
    2b60:	a4 63 00 ff 	l.andi r3,r3,0xff
    2b64:	d8 06 18 00 	l.sb 0(r6),r3
    2b68:	9c 21 00 04 	l.addi r1,r1,4
    2b6c:	44 00 48 00 	l.jr r9
    2b70:	84 21 ff fc 	l.lwz r1,-4(r1)

00002b74 <or1k_uart_set_read_cb>:
    2b74:	d7 e1 17 f8 	l.sw -8(r1),r2
    2b78:	18 40 00 00 	l.movhi r2,0x0
    2b7c:	d7 e1 4f fc 	l.sw -4(r1),r9
    2b80:	a8 42 36 f4 	l.ori r2,r2,0x36f4
    2b84:	d7 e1 0f f4 	l.sw -12(r1),r1
    2b88:	84 82 00 00 	l.lwz r4,0(r2)
    2b8c:	18 40 00 00 	l.movhi r2,0x0
    2b90:	9c 84 00 01 	l.addi r4,r4,1
    2b94:	a8 42 48 c8 	l.ori r2,r2,0x48c8
    2b98:	9c 21 ff f4 	l.addi r1,r1,-12
    2b9c:	d4 02 18 00 	l.sw 0(r2),r3
    2ba0:	9c 40 00 01 	l.addi r2,r0,1
    2ba4:	9c a0 00 00 	l.addi r5,r0,0
    2ba8:	d8 04 10 00 	l.sb 0(r4),r2
    2bac:	18 40 00 00 	l.movhi r2,0x0
    2bb0:	18 80 00 00 	l.movhi r4,0x0
    2bb4:	a8 42 36 fc 	l.ori r2,r2,0x36fc
    2bb8:	a8 84 2a 2c 	l.ori r4,r4,0x2a2c
    2bbc:	04 00 00 df 	l.jal 2f38 <or1k_interrupt_handler_add>
    2bc0:	84 62 00 00 	l.lwz r3,0(r2)
    2bc4:	04 00 01 33 	l.jal 3090 <or1k_interrupt_enable>
    2bc8:	84 62 00 00 	l.lwz r3,0(r2)
    2bcc:	9c 21 00 0c 	l.addi r1,r1,12
    2bd0:	85 21 ff fc 	l.lwz r9,-4(r1)
    2bd4:	84 21 ff f4 	l.lwz r1,-12(r1)
    2bd8:	44 00 48 00 	l.jr r9
    2bdc:	84 41 ff f8 	l.lwz r2,-8(r1)

00002be0 <_or1k_outbyte>:
    2be0:	18 80 00 00 	l.movhi r4,0x0
    2be4:	a8 84 36 f4 	l.ori r4,r4,0x36f4
    2be8:	84 84 00 00 	l.lwz r4,0(r4)
    2bec:	e4 04 00 00 	l.sfeq r4,r0
    2bf0:	10 00 00 04 	l.bf 2c00 <_or1k_outbyte+0x20>
    2bf4:	15 00 00 00 	l.nop 0x0
    2bf8:	03 ff ff cd 	l.j 2b2c <_or1k_uart_write>
    2bfc:	15 00 00 00 	l.nop 0x0
    2c00:	44 00 48 00 	l.jr r9
    2c04:	15 00 00 04 	l.nop 0x4

00002c08 <_or1k_cache_init>:
    2c08:	b4 60 00 01 	l.mfspr r3,r0,0x1
    2c0c:	a4 83 00 04 	l.andi r4,r3,0x4
    2c10:	e4 04 00 00 	l.sfeq r4,r0
    2c14:	10 00 00 21 	l.bf 2c98 <_or1k_cache_init+0x90>
    2c18:	15 00 00 00 	l.nop 0x0
    2c1c:	b4 c0 00 11 	l.mfspr r6,r0,0x11
    2c20:	9c a0 ff ff 	l.addi r5,r0,-1
    2c24:	ac a5 00 10 	l.xori r5,r5,16
    2c28:	e0 a6 28 03 	l.and r5,r6,r5
    2c2c:	c0 00 28 11 	l.mtspr r0,r5,0x11
    2c30:	b4 60 00 06 	l.mfspr r3,r0,0x6
    2c34:	a4 83 00 80 	l.andi r4,r3,0x80
    2c38:	b8 e4 00 47 	l.srli r7,r4,0x7
    2c3c:	a9 00 00 10 	l.ori r8,r0,0x10
    2c40:	e1 c8 38 08 	l.sll r14,r8,r7
    2c44:	a4 83 00 78 	l.andi r4,r3,0x78
    2c48:	b8 e4 00 43 	l.srli r7,r4,0x3
    2c4c:	a9 00 00 01 	l.ori r8,r0,0x1
    2c50:	e1 a8 38 08 	l.sll r13,r8,r7
    2c54:	9c c0 00 00 	l.addi r6,r0,0
    2c58:	e0 ae 38 08 	l.sll r5,r14,r7
    2c5c:	c0 80 30 02 	l.mtspr r0,r6,0x2002
    2c60:	e4 26 28 00 	l.sfne r6,r5
    2c64:	13 ff ff fe 	l.bf 2c5c <_or1k_cache_init+0x54>
    2c68:	e0 c6 70 00 	l.add r6,r6,r14
    2c6c:	b4 c0 00 11 	l.mfspr r6,r0,0x11
    2c70:	a8 c6 00 10 	l.ori r6,r6,0x10
    2c74:	c0 00 30 11 	l.mtspr r0,r6,0x11
    2c78:	15 00 00 00 	l.nop 0x0
    2c7c:	15 00 00 00 	l.nop 0x0
    2c80:	15 00 00 00 	l.nop 0x0
    2c84:	15 00 00 00 	l.nop 0x0
    2c88:	15 00 00 00 	l.nop 0x0
    2c8c:	15 00 00 00 	l.nop 0x0
    2c90:	15 00 00 00 	l.nop 0x0
    2c94:	15 00 00 00 	l.nop 0x0
    2c98:	b4 60 00 01 	l.mfspr r3,r0,0x1
    2c9c:	a4 83 00 02 	l.andi r4,r3,0x2
    2ca0:	e4 04 00 00 	l.sfeq r4,r0
    2ca4:	10 00 00 19 	l.bf 2d08 <_or1k_cache_init+0x100>
    2ca8:	15 00 00 00 	l.nop 0x0
    2cac:	b4 c0 00 11 	l.mfspr r6,r0,0x11
    2cb0:	9c a0 ff ff 	l.addi r5,r0,-1
    2cb4:	ac a5 00 08 	l.xori r5,r5,8
    2cb8:	e0 a6 28 03 	l.and r5,r6,r5
    2cbc:	c0 00 28 11 	l.mtspr r0,r5,0x11
    2cc0:	b4 60 00 05 	l.mfspr r3,r0,0x5
    2cc4:	a4 83 00 80 	l.andi r4,r3,0x80
    2cc8:	b8 e4 00 47 	l.srli r7,r4,0x7
    2ccc:	a9 00 00 10 	l.ori r8,r0,0x10
    2cd0:	e1 c8 38 08 	l.sll r14,r8,r7
    2cd4:	a4 83 00 78 	l.andi r4,r3,0x78
    2cd8:	b8 e4 00 43 	l.srli r7,r4,0x3
    2cdc:	a9 00 00 01 	l.ori r8,r0,0x1
    2ce0:	e1 a8 38 08 	l.sll r13,r8,r7
    2ce4:	9c c0 00 00 	l.addi r6,r0,0
    2ce8:	e0 ae 38 08 	l.sll r5,r14,r7
    2cec:	c0 60 30 03 	l.mtspr r0,r6,0x1803
    2cf0:	e4 26 28 00 	l.sfne r6,r5
    2cf4:	13 ff ff fe 	l.bf 2cec <_or1k_cache_init+0xe4>
    2cf8:	e0 c6 70 00 	l.add r6,r6,r14
    2cfc:	b4 c0 00 11 	l.mfspr r6,r0,0x11
    2d00:	a8 c6 00 08 	l.ori r6,r6,0x8
    2d04:	c0 00 30 11 	l.mtspr r0,r6,0x11
    2d08:	44 00 48 00 	l.jr r9
    2d0c:	15 00 00 00 	l.nop 0x0

00002d10 <or1k_icache_enable>:
    2d10:	b5 a0 00 11 	l.mfspr r13,r0,0x11
    2d14:	a9 ad 00 10 	l.ori r13,r13,0x10
    2d18:	c0 00 68 11 	l.mtspr r0,r13,0x11
    2d1c:	15 00 00 00 	l.nop 0x0
    2d20:	15 00 00 00 	l.nop 0x0
    2d24:	15 00 00 00 	l.nop 0x0
    2d28:	15 00 00 00 	l.nop 0x0
    2d2c:	15 00 00 00 	l.nop 0x0
    2d30:	44 00 48 00 	l.jr r9
    2d34:	15 00 00 00 	l.nop 0x0

00002d38 <or1k_icache_disable>:
    2d38:	b5 a0 00 11 	l.mfspr r13,r0,0x11
    2d3c:	9d 80 ff ff 	l.addi r12,r0,-1
    2d40:	ad 8c 00 10 	l.xori r12,r12,16
    2d44:	e1 8d 60 03 	l.and r12,r13,r12
    2d48:	c0 00 60 11 	l.mtspr r0,r12,0x11
    2d4c:	44 00 48 00 	l.jr r9
    2d50:	15 00 00 00 	l.nop 0x0

00002d54 <or1k_icache_flush>:
    2d54:	44 00 48 00 	l.jr r9
    2d58:	c0 80 18 02 	l.mtspr r0,r3,0x2002

00002d5c <or1k_dcache_enable>:
    2d5c:	b5 a0 00 11 	l.mfspr r13,r0,0x11
    2d60:	a9 ad 00 08 	l.ori r13,r13,0x8
    2d64:	c0 00 68 11 	l.mtspr r0,r13,0x11
    2d68:	15 00 00 00 	l.nop 0x0
    2d6c:	15 00 00 00 	l.nop 0x0
    2d70:	15 00 00 00 	l.nop 0x0
    2d74:	15 00 00 00 	l.nop 0x0
    2d78:	15 00 00 00 	l.nop 0x0
    2d7c:	44 00 48 00 	l.jr r9
    2d80:	15 00 00 00 	l.nop 0x0

00002d84 <or1k_dcache_disable>:
    2d84:	b5 a0 00 11 	l.mfspr r13,r0,0x11
    2d88:	9d 80 ff ff 	l.addi r12,r0,-1
    2d8c:	ad 8c 00 08 	l.xori r12,r12,8
    2d90:	e1 8d 60 03 	l.and r12,r13,r12
    2d94:	c0 00 60 11 	l.mtspr r0,r12,0x11
    2d98:	44 00 48 00 	l.jr r9
    2d9c:	15 00 00 00 	l.nop 0x0

00002da0 <or1k_dcache_flush>:
    2da0:	44 00 48 00 	l.jr r9
    2da4:	c0 60 18 03 	l.mtspr r0,r3,0x1803

00002da8 <_or1k_exception_handler>:
    2da8:	d4 01 10 08 	l.sw 8(r1),r2
    2dac:	d4 01 28 14 	l.sw 20(r1),r5
    2db0:	d4 01 30 18 	l.sw 24(r1),r6
    2db4:	d4 01 38 1c 	l.sw 28(r1),r7
    2db8:	d4 01 40 20 	l.sw 32(r1),r8
    2dbc:	d4 01 48 24 	l.sw 36(r1),r9
    2dc0:	d4 01 50 28 	l.sw 40(r1),r10
    2dc4:	d4 01 58 2c 	l.sw 44(r1),r11
    2dc8:	d4 01 60 30 	l.sw 48(r1),r12
    2dcc:	d4 01 68 34 	l.sw 52(r1),r13
    2dd0:	d4 01 70 38 	l.sw 56(r1),r14
    2dd4:	d4 01 78 3c 	l.sw 60(r1),r15
    2dd8:	d4 01 80 40 	l.sw 64(r1),r16
    2ddc:	d4 01 88 44 	l.sw 68(r1),r17
    2de0:	d4 01 90 48 	l.sw 72(r1),r18
    2de4:	d4 01 98 4c 	l.sw 76(r1),r19
    2de8:	d4 01 a0 50 	l.sw 80(r1),r20
    2dec:	d4 01 a8 54 	l.sw 84(r1),r21
    2df0:	d4 01 b0 58 	l.sw 88(r1),r22
    2df4:	d4 01 b8 5c 	l.sw 92(r1),r23
    2df8:	d4 01 c0 60 	l.sw 96(r1),r24
    2dfc:	d4 01 c8 64 	l.sw 100(r1),r25
    2e00:	d4 01 d0 68 	l.sw 104(r1),r26
    2e04:	d4 01 d8 6c 	l.sw 108(r1),r27
    2e08:	d4 01 e0 70 	l.sw 112(r1),r28
    2e0c:	d4 01 e8 74 	l.sw 116(r1),r29
    2e10:	d4 01 f0 78 	l.sw 120(r1),r30
    2e14:	d4 01 f8 7c 	l.sw 124(r1),r31
    2e18:	b5 c0 00 20 	l.mfspr r14,r0,0x20
    2e1c:	d4 01 70 80 	l.sw 128(r1),r14
    2e20:	b5 c0 00 40 	l.mfspr r14,r0,0x40
    2e24:	d4 01 70 84 	l.sw 132(r1),r14
    2e28:	1a 80 00 00 	l.movhi r20,0x0
    2e2c:	aa 94 44 7c 	l.ori r20,r20,0x447c
    2e30:	86 94 00 00 	l.lwz r20,0(r20)
    2e34:	1a a0 00 00 	l.movhi r21,0x0
    2e38:	aa b5 49 d8 	l.ori r21,r21,0x49d8
    2e3c:	d4 15 a0 00 	l.sw 0(r21),r20
    2e40:	a5 a3 ff 00 	l.andi r13,r3,0xff00
    2e44:	b9 ad 00 46 	l.srli r13,r13,0x6
    2e48:	9d ad ff f8 	l.addi r13,r13,-8
    2e4c:	19 c0 00 00 	l.movhi r14,0x0
    2e50:	a9 ce 49 f4 	l.ori r14,r14,0x49f4
    2e54:	e1 ce 68 00 	l.add r14,r14,r13
    2e58:	85 ae 00 00 	l.lwz r13,0(r14)
    2e5c:	e4 2d 00 00 	l.sfne r13,r0
    2e60:	0c 00 00 34 	l.bnf 2f30 <exception_exit>
    2e64:	15 00 00 00 	l.nop 0x0
    2e68:	48 00 68 00 	l.jalr r13
    2e6c:	e0 64 20 04 	l.or r3,r4,r4
    2e70:	1a 80 00 00 	l.movhi r20,0x0
    2e74:	aa 94 49 dc 	l.ori r20,r20,0x49dc
    2e78:	86 94 00 00 	l.lwz r20,0(r20)
    2e7c:	1a a0 00 00 	l.movhi r21,0x0
    2e80:	aa b5 49 d8 	l.ori r21,r21,0x49d8
    2e84:	d4 15 a0 00 	l.sw 0(r21),r20
    2e88:	18 40 00 00 	l.movhi r2,0x0
    2e8c:	a8 42 49 e0 	l.ori r2,r2,0x49e0
    2e90:	84 62 00 00 	l.lwz r3,0(r2)
    2e94:	9c 63 ff ff 	l.addi r3,r3,-1
    2e98:	d4 02 18 00 	l.sw 0(r2),r3
    2e9c:	84 41 00 80 	l.lwz r2,128(r1)
    2ea0:	c0 00 10 20 	l.mtspr r0,r2,0x20
    2ea4:	84 41 00 84 	l.lwz r2,132(r1)
    2ea8:	c0 00 10 40 	l.mtspr r0,r2,0x40
    2eac:	84 41 00 08 	l.lwz r2,8(r1)
    2eb0:	84 61 00 0c 	l.lwz r3,12(r1)
    2eb4:	84 81 00 10 	l.lwz r4,16(r1)
    2eb8:	84 a1 00 14 	l.lwz r5,20(r1)
    2ebc:	84 c1 00 18 	l.lwz r6,24(r1)
    2ec0:	84 e1 00 1c 	l.lwz r7,28(r1)
    2ec4:	85 01 00 20 	l.lwz r8,32(r1)
    2ec8:	85 21 00 24 	l.lwz r9,36(r1)
    2ecc:	85 41 00 28 	l.lwz r10,40(r1)
    2ed0:	85 61 00 2c 	l.lwz r11,44(r1)
    2ed4:	85 81 00 30 	l.lwz r12,48(r1)
    2ed8:	85 a1 00 34 	l.lwz r13,52(r1)
    2edc:	85 c1 00 38 	l.lwz r14,56(r1)
    2ee0:	85 e1 00 3c 	l.lwz r15,60(r1)
    2ee4:	86 01 00 40 	l.lwz r16,64(r1)
    2ee8:	86 21 00 44 	l.lwz r17,68(r1)
    2eec:	86 41 00 48 	l.lwz r18,72(r1)
    2ef0:	86 61 00 4c 	l.lwz r19,76(r1)
    2ef4:	86 81 00 50 	l.lwz r20,80(r1)
    2ef8:	86 a1 00 54 	l.lwz r21,84(r1)
    2efc:	86 c1 00 58 	l.lwz r22,88(r1)
    2f00:	86 e1 00 5c 	l.lwz r23,92(r1)
    2f04:	87 01 00 60 	l.lwz r24,96(r1)
    2f08:	87 21 00 64 	l.lwz r25,100(r1)
    2f0c:	87 41 00 68 	l.lwz r26,104(r1)
    2f10:	87 61 00 6c 	l.lwz r27,108(r1)
    2f14:	87 81 00 70 	l.lwz r28,112(r1)
    2f18:	87 a1 00 74 	l.lwz r29,116(r1)
    2f1c:	87 c1 00 78 	l.lwz r30,120(r1)
    2f20:	87 e1 00 7c 	l.lwz r31,124(r1)
    2f24:	84 21 00 04 	l.lwz r1,4(r1)
    2f28:	24 00 00 00 	l.rfe
    2f2c:	15 00 00 00 	l.nop 0x0

00002f30 <exception_exit>:
    2f30:	07 ff fd 39 	l.jal 2414 <exit>
    2f34:	e0 64 20 04 	l.or r3,r4,r4

00002f38 <or1k_interrupt_handler_add>:
    2f38:	d7 e1 17 fc 	l.sw -4(r1),r2
    2f3c:	18 40 00 00 	l.movhi r2,0x0
    2f40:	b8 63 00 02 	l.slli r3,r3,0x2
    2f44:	a8 42 48 cc 	l.ori r2,r2,0x48cc
    2f48:	d7 e1 0f f8 	l.sw -8(r1),r1
    2f4c:	e0 c3 10 00 	l.add r6,r3,r2
    2f50:	18 40 00 00 	l.movhi r2,0x0
    2f54:	9c 21 ff f8 	l.addi r1,r1,-8
    2f58:	a8 42 49 4c 	l.ori r2,r2,0x494c
    2f5c:	d4 06 20 00 	l.sw 0(r6),r4
    2f60:	e0 63 10 00 	l.add r3,r3,r2
    2f64:	d4 03 28 00 	l.sw 0(r3),r5
    2f68:	9c 21 00 08 	l.addi r1,r1,8
    2f6c:	84 21 ff f8 	l.lwz r1,-8(r1)
    2f70:	44 00 48 00 	l.jr r9
    2f74:	84 41 ff fc 	l.lwz r2,-4(r1)

00002f78 <or1k_interrupts_enable>:
    2f78:	d7 e1 0f fc 	l.sw -4(r1),r1
    2f7c:	9c 80 00 11 	l.addi r4,r0,17
    2f80:	9c 21 ff fc 	l.addi r1,r1,-4
    2f84:	b4 64 00 00 	l.mfspr r3,r4,0x0
    2f88:	a8 63 00 04 	l.ori r3,r3,0x4
    2f8c:	c0 04 18 00 	l.mtspr r4,r3,0x0
    2f90:	9c 21 00 04 	l.addi r1,r1,4
    2f94:	44 00 48 00 	l.jr r9
    2f98:	84 21 ff fc 	l.lwz r1,-4(r1)

00002f9c <or1k_interrupts_disable>:
    2f9c:	d7 e1 0f f8 	l.sw -8(r1),r1
    2fa0:	d7 e1 17 fc 	l.sw -4(r1),r2
    2fa4:	9c 60 00 11 	l.addi r3,r0,17
    2fa8:	9c 21 ff f8 	l.addi r1,r1,-8
    2fac:	b5 63 00 00 	l.mfspr r11,r3,0x0
    2fb0:	9c 40 ff fb 	l.addi r2,r0,-5
    2fb4:	e0 8b 10 03 	l.and r4,r11,r2
    2fb8:	c0 03 20 00 	l.mtspr r3,r4,0x0
    2fbc:	9c 21 00 08 	l.addi r1,r1,8
    2fc0:	b9 6b 00 42 	l.srli r11,r11,0x2
    2fc4:	84 21 ff f8 	l.lwz r1,-8(r1)
    2fc8:	a5 6b 00 01 	l.andi r11,r11,0x1
    2fcc:	44 00 48 00 	l.jr r9
    2fd0:	84 41 ff fc 	l.lwz r2,-4(r1)

00002fd4 <or1k_interrupts_restore>:
    2fd4:	d7 e1 0f f8 	l.sw -8(r1),r1
    2fd8:	d7 e1 17 fc 	l.sw -4(r1),r2
    2fdc:	9c 80 00 11 	l.addi r4,r0,17
    2fe0:	9c 21 ff f8 	l.addi r1,r1,-8
    2fe4:	b4 84 00 00 	l.mfspr r4,r4,0x0
    2fe8:	9c 40 ff fb 	l.addi r2,r0,-5
    2fec:	bc 23 00 00 	l.sfnei r3,0
    2ff0:	e0 84 10 03 	l.and r4,r4,r2
    2ff4:	10 00 00 03 	l.bf 3000 <or1k_interrupts_restore+0x2c>
    2ff8:	9c a0 00 04 	l.addi r5,r0,4
    2ffc:	a8 a3 00 00 	l.ori r5,r3,0x0
    3000:	e0 65 20 04 	l.or r3,r5,r4
    3004:	9c 80 00 11 	l.addi r4,r0,17
    3008:	c0 04 18 00 	l.mtspr r4,r3,0x0
    300c:	9c 21 00 08 	l.addi r1,r1,8
    3010:	84 21 ff f8 	l.lwz r1,-8(r1)
    3014:	44 00 48 00 	l.jr r9
    3018:	84 41 ff fc 	l.lwz r2,-4(r1)

0000301c <_or1k_interrupt_handler>:
    301c:	9c 21 ff fc 	l.addi r1,r1,-4
    3020:	d4 01 48 00 	l.sw 0(r1),r9
    3024:	b6 80 48 02 	l.mfspr r20,r0,0x4802
    3028:	1a 00 00 00 	l.movhi r16,0x0
    302c:	aa 10 48 cc 	l.ori r16,r16,0x48cc
    3030:	1a 40 00 00 	l.movhi r18,0x0
    3034:	aa 52 49 4c 	l.ori r18,r18,0x494c
    3038:	e0 94 00 0f 	l.ff1 r4,r20
    303c:	e4 24 00 00 	l.sfne r4,r0
    3040:	0c 00 00 10 	l.bnf 3080 <_or1k_interrupt_handler+0x64>
    3044:	15 00 00 00 	l.nop 0x0
    3048:	9e c4 ff ff 	l.addi r22,r4,-1
    304c:	b8 d6 00 02 	l.slli r6,r22,0x2
    3050:	e1 c6 80 00 	l.add r14,r6,r16
    3054:	e1 a6 90 00 	l.add r13,r6,r18
    3058:	85 ce 00 00 	l.lwz r14,0(r14)
    305c:	e4 2e 00 00 	l.sfne r14,r0
    3060:	0c 00 00 04 	l.bnf 3070 <_or1k_interrupt_handler+0x54>
    3064:	15 00 00 00 	l.nop 0x0
    3068:	48 00 70 00 	l.jalr r14
    306c:	84 6d 00 00 	l.lwz r3,0(r13)
    3070:	a8 c0 00 01 	l.ori r6,r0,0x1
    3074:	e0 c6 b0 08 	l.sll r6,r6,r22
    3078:	03 ff ff f0 	l.j 3038 <_or1k_interrupt_handler+0x1c>
    307c:	e2 94 30 05 	l.xor r20,r20,r6
    3080:	85 21 00 00 	l.lwz r9,0(r1)
    3084:	c1 20 a0 02 	l.mtspr r0,r20,0x4802
    3088:	44 00 48 00 	l.jr r9
    308c:	9c 21 00 04 	l.addi r1,r1,4

00003090 <or1k_interrupt_enable>:
    3090:	9c 21 ff fc 	l.addi r1,r1,-4
    3094:	d4 01 20 00 	l.sw 0(r1),r4
    3098:	a8 80 00 01 	l.ori r4,r0,0x1
    309c:	e0 84 18 08 	l.sll r4,r4,r3
    30a0:	b4 60 48 00 	l.mfspr r3,r0,0x4800
    30a4:	e0 63 20 04 	l.or r3,r3,r4
    30a8:	c1 20 18 00 	l.mtspr r0,r3,0x4800
    30ac:	84 81 00 00 	l.lwz r4,0(r1)
    30b0:	44 00 48 00 	l.jr r9
    30b4:	9c 21 00 04 	l.addi r1,r1,4

000030b8 <or1k_interrupt_disable>:
    30b8:	9c 21 ff fc 	l.addi r1,r1,-4
    30bc:	d4 01 20 00 	l.sw 0(r1),r4
    30c0:	a8 80 00 01 	l.ori r4,r0,0x1
    30c4:	e0 84 18 08 	l.sll r4,r4,r3
    30c8:	ac 84 ff ff 	l.xori r4,r4,-1
    30cc:	b4 60 48 00 	l.mfspr r3,r0,0x4800
    30d0:	e0 63 20 03 	l.and r3,r3,r4
    30d4:	c1 20 18 00 	l.mtspr r0,r3,0x4800
    30d8:	84 81 00 00 	l.lwz r4,0(r1)
    30dc:	44 00 48 00 	l.jr r9
    30e0:	9c 21 00 04 	l.addi r1,r1,4

000030e4 <_or1k_libc_impure_init>:
    30e4:	d7 e1 17 d8 	l.sw -40(r1),r2
    30e8:	18 40 00 00 	l.movhi r2,0x0
    30ec:	d7 e1 f7 f8 	l.sw -8(r1),r30
    30f0:	a8 42 40 50 	l.ori r2,r2,0x4050
    30f4:	9f c0 04 24 	l.addi r30,r0,1060
    30f8:	d7 e1 4f fc 	l.sw -4(r1),r9
    30fc:	d7 e1 77 dc 	l.sw -36(r1),r14
    3100:	d7 e1 97 e0 	l.sw -32(r1),r18
    3104:	d7 e1 a7 e4 	l.sw -28(r1),r20
    3108:	d7 e1 b7 e8 	l.sw -24(r1),r22
    310c:	d7 e1 c7 ec 	l.sw -20(r1),r24
    3110:	d7 e1 d7 f0 	l.sw -16(r1),r26
    3114:	d7 e1 e7 f4 	l.sw -12(r1),r28
    3118:	84 62 00 00 	l.lwz r3,0(r2)
    311c:	a8 be 00 00 	l.ori r5,r30,0x0
    3120:	d7 e1 0f d4 	l.sw -44(r1),r1
    3124:	9c 80 00 00 	l.addi r4,r0,0
    3128:	9c 21 ff d4 	l.addi r1,r1,-44
    312c:	04 00 01 86 	l.jal 3744 <memset>
    3130:	1b 40 00 00 	l.movhi r26,0x0
    3134:	84 c2 00 00 	l.lwz r6,0(r2)
    3138:	1b 80 00 00 	l.movhi r28,0x0
    313c:	9c 86 03 54 	l.addi r4,r6,852
    3140:	9c 66 03 bc 	l.addi r3,r6,956
    3144:	9c a6 02 ec 	l.addi r5,r6,748
    3148:	ab 5a 3a 38 	l.ori r26,r26,0x3a38
    314c:	9f 00 33 0e 	l.addi r24,r0,13070
    3150:	9e c0 ab cd 	l.addi r22,r0,-21555
    3154:	9e 80 12 34 	l.addi r20,r0,4660
    3158:	9e 40 e6 6d 	l.addi r18,r0,-6547
    315c:	9d c0 de ec 	l.addi r14,r0,-8468
    3160:	ab 9c 44 7c 	l.ori r28,r28,0x447c
    3164:	d4 06 20 08 	l.sw 8(r6),r4
    3168:	d4 06 18 0c 	l.sw 12(r6),r3
    316c:	9c 80 00 0b 	l.addi r4,r0,11
    3170:	9c 60 00 05 	l.addi r3,r0,5
    3174:	d4 06 28 04 	l.sw 4(r6),r5
    3178:	d4 06 d0 34 	l.sw 52(r6),r26
    317c:	dc 06 c0 ac 	l.sh 172(r6),r24
    3180:	dc 06 b0 ae 	l.sh 174(r6),r22
    3184:	dc 06 a0 b0 	l.sh 176(r6),r20
    3188:	dc 06 90 b2 	l.sh 178(r6),r18
    318c:	dc 06 70 b4 	l.sh 180(r6),r14
    3190:	dc 06 18 b6 	l.sh 182(r6),r3
    3194:	dc 06 20 b8 	l.sh 184(r6),r4
    3198:	84 7c 00 00 	l.lwz r3,0(r28)
    319c:	a8 be 00 00 	l.ori r5,r30,0x0
    31a0:	9c e0 00 00 	l.addi r7,r0,0
    31a4:	9d 00 00 01 	l.addi r8,r0,1
    31a8:	d4 06 38 a4 	l.sw 164(r6),r7
    31ac:	d4 06 40 a8 	l.sw 168(r6),r8
    31b0:	04 00 01 65 	l.jal 3744 <memset>
    31b4:	9c 80 00 00 	l.addi r4,r0,0
    31b8:	84 7c 00 00 	l.lwz r3,0(r28)
    31bc:	84 42 00 00 	l.lwz r2,0(r2)
    31c0:	9c 83 03 bc 	l.addi r4,r3,956
    31c4:	9d 00 00 05 	l.addi r8,r0,5
    31c8:	d4 03 20 0c 	l.sw 12(r3),r4
    31cc:	9c 80 00 0b 	l.addi r4,r0,11
    31d0:	dc 03 40 b6 	l.sh 182(r3),r8
    31d4:	dc 03 20 b8 	l.sh 184(r3),r4
    31d8:	18 80 00 00 	l.movhi r4,0x0
    31dc:	9c c3 02 ec 	l.addi r6,r3,748
    31e0:	a8 84 49 dc 	l.ori r4,r4,0x49dc
    31e4:	9c a3 03 54 	l.addi r5,r3,852
    31e8:	d4 04 10 00 	l.sw 0(r4),r2
    31ec:	18 80 00 00 	l.movhi r4,0x0
    31f0:	9c e0 00 00 	l.addi r7,r0,0
    31f4:	a8 84 49 d8 	l.ori r4,r4,0x49d8
    31f8:	9d 00 00 01 	l.addi r8,r0,1
    31fc:	dc 03 c0 ac 	l.sh 172(r3),r24
    3200:	dc 03 b0 ae 	l.sh 174(r3),r22
    3204:	dc 03 a0 b0 	l.sh 176(r3),r20
    3208:	dc 03 90 b2 	l.sh 178(r3),r18
    320c:	dc 03 70 b4 	l.sh 180(r3),r14
    3210:	d4 03 38 a4 	l.sw 164(r3),r7
    3214:	d4 03 40 a8 	l.sw 168(r3),r8
    3218:	d4 03 d0 34 	l.sw 52(r3),r26
    321c:	d4 04 10 00 	l.sw 0(r4),r2
    3220:	d4 03 30 04 	l.sw 4(r3),r6
    3224:	d4 03 28 08 	l.sw 8(r3),r5
    3228:	9c 21 00 2c 	l.addi r1,r1,44
    322c:	85 21 ff fc 	l.lwz r9,-4(r1)
    3230:	84 21 ff d4 	l.lwz r1,-44(r1)
    3234:	84 41 ff d8 	l.lwz r2,-40(r1)
    3238:	85 c1 ff dc 	l.lwz r14,-36(r1)
    323c:	86 41 ff e0 	l.lwz r18,-32(r1)
    3240:	86 81 ff e4 	l.lwz r20,-28(r1)
    3244:	86 c1 ff e8 	l.lwz r22,-24(r1)
    3248:	87 01 ff ec 	l.lwz r24,-20(r1)
    324c:	87 41 ff f0 	l.lwz r26,-16(r1)
    3250:	87 81 ff f4 	l.lwz r28,-12(r1)
    3254:	44 00 48 00 	l.jr r9
    3258:	87 c1 ff f8 	l.lwz r30,-8(r1)

0000325c <_or1k_libc_getreent>:
    325c:	18 60 00 00 	l.movhi r3,0x0
    3260:	d7 e1 0f fc 	l.sw -4(r1),r1
    3264:	a8 63 49 d8 	l.ori r3,r3,0x49d8
    3268:	9c 21 ff fc 	l.addi r1,r1,-4
    326c:	85 63 00 00 	l.lwz r11,0(r3)
    3270:	9c 21 00 04 	l.addi r1,r1,4
    3274:	44 00 48 00 	l.jr r9
    3278:	84 21 ff fc 	l.lwz r1,-4(r1)

0000327c <_or1k_reent_init>:
    327c:	d7 e1 0f fc 	l.sw -4(r1),r1
    3280:	9c 21 ff fc 	l.addi r1,r1,-4
    3284:	9c 21 00 04 	l.addi r1,r1,4
    3288:	44 00 48 00 	l.jr r9
    328c:	84 21 ff fc 	l.lwz r1,-4(r1)

00003290 <_or1k_init>:
    3290:	d7 e1 4f fc 	l.sw -4(r1),r9
    3294:	d7 e1 17 f8 	l.sw -8(r1),r2
    3298:	d7 e1 0f f4 	l.sw -12(r1),r1
    329c:	9c 21 ff f4 	l.addi r1,r1,-12
    32a0:	07 ff ff f7 	l.jal 327c <_or1k_reent_init>
    32a4:	9c 40 00 00 	l.addi r2,r0,0
    32a8:	18 80 00 00 	l.movhi r4,0x0
    32ac:	18 60 00 00 	l.movhi r3,0x0
    32b0:	a8 84 30 1c 	l.ori r4,r4,0x301c
    32b4:	a8 63 49 f4 	l.ori r3,r3,0x49f4
    32b8:	d4 03 20 18 	l.sw 24(r3),r4
    32bc:	18 60 00 00 	l.movhi r3,0x0
    32c0:	a8 63 49 e0 	l.ori r3,r3,0x49e0
    32c4:	d4 03 10 00 	l.sw 0(r3),r2
    32c8:	9c 21 00 0c 	l.addi r1,r1,12
    32cc:	85 21 ff fc 	l.lwz r9,-4(r1)
    32d0:	84 21 ff f4 	l.lwz r1,-12(r1)
    32d4:	44 00 48 00 	l.jr r9
    32d8:	84 41 ff f8 	l.lwz r2,-8(r1)

000032dc <or1k_critical_begin>:
    32dc:	d7 e1 4f fc 	l.sw -4(r1),r9
    32e0:	d7 e1 17 f8 	l.sw -8(r1),r2
    32e4:	d7 e1 0f f4 	l.sw -12(r1),r1
    32e8:	07 ff ff 2d 	l.jal 2f9c <or1k_interrupts_disable>
    32ec:	9c 21 ff f4 	l.addi r1,r1,-12
    32f0:	04 00 00 b7 	l.jal 35cc <or1k_timer_disable>
    32f4:	a8 4b 00 00 	l.ori r2,r11,0x0
    32f8:	9c 21 00 0c 	l.addi r1,r1,12
    32fc:	e0 42 10 00 	l.add r2,r2,r2
    3300:	85 21 ff fc 	l.lwz r9,-4(r1)
    3304:	84 21 ff f4 	l.lwz r1,-12(r1)
    3308:	e1 62 58 04 	l.or r11,r2,r11
    330c:	44 00 48 00 	l.jr r9
    3310:	84 41 ff f8 	l.lwz r2,-8(r1)

00003314 <or1k_critical_end>:
    3314:	d7 e1 4f fc 	l.sw -4(r1),r9
    3318:	d7 e1 17 f8 	l.sw -8(r1),r2
    331c:	d7 e1 0f f4 	l.sw -12(r1),r1
    3320:	a8 43 00 00 	l.ori r2,r3,0x0
    3324:	9c 21 ff f4 	l.addi r1,r1,-12
    3328:	04 00 00 b7 	l.jal 3604 <or1k_timer_restore>
    332c:	a4 63 00 01 	l.andi r3,r3,0x1
    3330:	b8 62 00 41 	l.srli r3,r2,0x1
    3334:	07 ff ff 28 	l.jal 2fd4 <or1k_interrupts_restore>
    3338:	a4 63 00 01 	l.andi r3,r3,0x1
    333c:	9c 21 00 0c 	l.addi r1,r1,12
    3340:	85 21 ff fc 	l.lwz r9,-4(r1)
    3344:	84 21 ff f4 	l.lwz r1,-12(r1)
    3348:	44 00 48 00 	l.jr r9
    334c:	84 41 ff f8 	l.lwz r2,-8(r1)

00003350 <or1k_exception_handler_add>:
    3350:	9c 63 ff fe 	l.addi r3,r3,-2
    3354:	d7 e1 17 fc 	l.sw -4(r1),r2
    3358:	18 40 00 00 	l.movhi r2,0x0
    335c:	b8 63 00 02 	l.slli r3,r3,0x2
    3360:	a8 42 49 f4 	l.ori r2,r2,0x49f4
    3364:	d7 e1 0f f8 	l.sw -8(r1),r1
    3368:	e0 63 10 00 	l.add r3,r3,r2
    336c:	9c 21 ff f8 	l.addi r1,r1,-8
    3370:	d4 03 20 00 	l.sw 0(r3),r4
    3374:	9c 21 00 08 	l.addi r1,r1,8
    3378:	84 21 ff f8 	l.lwz r1,-8(r1)
    337c:	44 00 48 00 	l.jr r9
    3380:	84 41 ff fc 	l.lwz r2,-4(r1)

00003384 <_or1k_timer_interrupt_handler>:
    3384:	18 a0 00 00 	l.movhi r5,0x0
    3388:	d7 e1 0f f8 	l.sw -8(r1),r1
    338c:	a8 a5 49 cc 	l.ori r5,r5,0x49cc
    3390:	d7 e1 17 fc 	l.sw -4(r1),r2
    3394:	84 65 00 00 	l.lwz r3,0(r5)
    3398:	9c 21 ff f8 	l.addi r1,r1,-8
    339c:	9c 63 00 01 	l.addi r3,r3,1
    33a0:	9c 80 50 00 	l.addi r4,r0,20480
    33a4:	d4 05 18 00 	l.sw 0(r5),r3
    33a8:	b4 64 00 00 	l.mfspr r3,r4,0x0
    33ac:	18 40 2f ff 	l.movhi r2,0x2fff
    33b0:	a8 42 ff ff 	l.ori r2,r2,0xffff
    33b4:	e0 63 10 03 	l.and r3,r3,r2
    33b8:	18 40 60 00 	l.movhi r2,0x6000
    33bc:	e0 63 10 04 	l.or r3,r3,r2
    33c0:	c0 04 18 00 	l.mtspr r4,r3,0x0
    33c4:	9c 21 00 08 	l.addi r1,r1,8
    33c8:	84 21 ff f8 	l.lwz r1,-8(r1)
    33cc:	44 00 48 00 	l.jr r9
    33d0:	84 41 ff fc 	l.lwz r2,-4(r1)

000033d4 <or1k_timer_init>:
    33d4:	d7 e1 97 f8 	l.sw -8(r1),r18
    33d8:	d7 e1 4f fc 	l.sw -4(r1),r9
    33dc:	d7 e1 0f ec 	l.sw -20(r1),r1
    33e0:	d7 e1 17 f0 	l.sw -16(r1),r2
    33e4:	d7 e1 77 f4 	l.sw -12(r1),r14
    33e8:	9e 40 00 01 	l.addi r18,r0,1
    33ec:	9c 21 ff ec 	l.addi r1,r1,-20
    33f0:	b4 b2 00 00 	l.mfspr r5,r18,0x0
    33f4:	b8 a5 00 4a 	l.srli r5,r5,0xa
    33f8:	e0 a5 90 03 	l.and r5,r5,r18
    33fc:	bc 05 00 00 	l.sfeqi r5,0
    3400:	10 00 00 20 	l.bf 3480 <or1k_timer_init+0xac>
    3404:	18 40 00 00 	l.movhi r2,0x0
    3408:	a8 83 00 00 	l.ori r4,r3,0x0
    340c:	a8 42 36 f0 	l.ori r2,r2,0x36f0
    3410:	19 c0 00 00 	l.movhi r14,0x0
    3414:	84 62 00 00 	l.lwz r3,0(r2)
    3418:	04 00 01 25 	l.jal 38ac <__udivsi3>
    341c:	18 40 0f ff 	l.movhi r2,0xfff
    3420:	a8 42 ff ff 	l.ori r2,r2,0xffff
    3424:	a9 ce 49 cc 	l.ori r14,r14,0x49cc
    3428:	e1 6b 10 03 	l.and r11,r11,r2
    342c:	9c 40 50 00 	l.addi r2,r0,20480
    3430:	d4 0e 58 04 	l.sw 4(r14),r11
    3434:	c0 02 58 00 	l.mtspr r2,r11,0x0
    3438:	18 80 00 00 	l.movhi r4,0x0
    343c:	9c 40 00 00 	l.addi r2,r0,0
    3440:	9c 60 00 05 	l.addi r3,r0,5
    3444:	a8 84 33 84 	l.ori r4,r4,0x3384
    3448:	d4 0e 10 00 	l.sw 0(r14),r2
    344c:	07 ff ff c1 	l.jal 3350 <or1k_exception_handler_add>
    3450:	15 00 00 00 	l.nop 0x0
    3454:	9c 60 50 01 	l.addi r3,r0,20481
    3458:	d4 0e 90 08 	l.sw 8(r14),r18
    345c:	c0 03 10 00 	l.mtspr r3,r2,0x0
    3460:	a9 62 00 00 	l.ori r11,r2,0x0
    3464:	9c 21 00 14 	l.addi r1,r1,20
    3468:	85 21 ff fc 	l.lwz r9,-4(r1)
    346c:	84 21 ff ec 	l.lwz r1,-20(r1)
    3470:	84 41 ff f0 	l.lwz r2,-16(r1)
    3474:	85 c1 ff f4 	l.lwz r14,-12(r1)
    3478:	44 00 48 00 	l.jr r9
    347c:	86 41 ff f8 	l.lwz r18,-8(r1)
    3480:	03 ff ff f9 	l.j 3464 <or1k_timer_init+0x90>
    3484:	9d 60 ff ff 	l.addi r11,r0,-1

00003488 <or1k_timer_set_period>:
    3488:	a8 83 00 00 	l.ori r4,r3,0x0
    348c:	18 60 00 00 	l.movhi r3,0x0
    3490:	d7 e1 17 f8 	l.sw -8(r1),r2
    3494:	a8 63 36 f0 	l.ori r3,r3,0x36f0
    3498:	18 40 0f ff 	l.movhi r2,0xfff
    349c:	d7 e1 4f fc 	l.sw -4(r1),r9
    34a0:	d7 e1 0f f4 	l.sw -12(r1),r1
    34a4:	84 63 00 00 	l.lwz r3,0(r3)
    34a8:	9c 21 ff f4 	l.addi r1,r1,-12
    34ac:	04 00 01 00 	l.jal 38ac <__udivsi3>
    34b0:	a8 42 ff ff 	l.ori r2,r2,0xffff
    34b4:	9c 80 50 00 	l.addi r4,r0,20480
    34b8:	e1 6b 10 03 	l.and r11,r11,r2
    34bc:	b4 64 00 00 	l.mfspr r3,r4,0x0
    34c0:	18 40 f0 00 	l.movhi r2,0xf000
    34c4:	e0 63 10 03 	l.and r3,r3,r2
    34c8:	e0 63 58 04 	l.or r3,r3,r11
    34cc:	c0 04 18 00 	l.mtspr r4,r3,0x0
    34d0:	18 60 00 00 	l.movhi r3,0x0
    34d4:	a8 63 49 cc 	l.ori r3,r3,0x49cc
    34d8:	d4 03 58 04 	l.sw 4(r3),r11
    34dc:	9c 21 00 0c 	l.addi r1,r1,12
    34e0:	85 21 ff fc 	l.lwz r9,-4(r1)
    34e4:	84 21 ff f4 	l.lwz r1,-12(r1)
    34e8:	44 00 48 00 	l.jr r9
    34ec:	84 41 ff f8 	l.lwz r2,-8(r1)

000034f0 <or1k_timer_set_handler>:
    34f0:	d7 e1 4f fc 	l.sw -4(r1),r9
    34f4:	d7 e1 0f f8 	l.sw -8(r1),r1
    34f8:	a8 83 00 00 	l.ori r4,r3,0x0
    34fc:	9c 21 ff f8 	l.addi r1,r1,-8
    3500:	07 ff ff 94 	l.jal 3350 <or1k_exception_handler_add>
    3504:	9c 60 00 05 	l.addi r3,r0,5
    3508:	9c 21 00 08 	l.addi r1,r1,8
    350c:	85 21 ff fc 	l.lwz r9,-4(r1)
    3510:	44 00 48 00 	l.jr r9
    3514:	84 21 ff f8 	l.lwz r1,-8(r1)

00003518 <or1k_timer_set_mode>:
    3518:	18 80 00 00 	l.movhi r4,0x0
    351c:	d7 e1 0f f8 	l.sw -8(r1),r1
    3520:	a8 84 49 cc 	l.ori r4,r4,0x49cc
    3524:	d7 e1 17 fc 	l.sw -4(r1),r2
    3528:	9c a0 50 00 	l.addi r5,r0,20480
    352c:	9c 21 ff f8 	l.addi r1,r1,-8
    3530:	d4 04 18 08 	l.sw 8(r4),r3
    3534:	b4 85 00 00 	l.mfspr r4,r5,0x0
    3538:	b8 c4 00 5e 	l.srli r6,r4,0x1e
    353c:	bc 06 00 00 	l.sfeqi r6,0
    3540:	10 00 00 07 	l.bf 355c <or1k_timer_set_mode+0x44>
    3544:	b8 63 00 1e 	l.slli r3,r3,0x1e
    3548:	18 40 3f ff 	l.movhi r2,0x3fff
    354c:	a8 42 ff ff 	l.ori r2,r2,0xffff
    3550:	e0 84 10 03 	l.and r4,r4,r2
    3554:	e0 64 18 04 	l.or r3,r4,r3
    3558:	c0 05 18 00 	l.mtspr r5,r3,0x0
    355c:	9c 21 00 08 	l.addi r1,r1,8
    3560:	84 21 ff f8 	l.lwz r1,-8(r1)
    3564:	44 00 48 00 	l.jr r9
    3568:	84 41 ff fc 	l.lwz r2,-4(r1)

0000356c <or1k_timer_enable>:
    356c:	d7 e1 0f f8 	l.sw -8(r1),r1
    3570:	d7 e1 17 fc 	l.sw -4(r1),r2
    3574:	9c a0 50 00 	l.addi r5,r0,20480
    3578:	9c 21 ff f8 	l.addi r1,r1,-8
    357c:	b4 65 00 00 	l.mfspr r3,r5,0x0
    3580:	18 40 3f ff 	l.movhi r2,0x3fff
    3584:	a8 42 ff ff 	l.ori r2,r2,0xffff
    3588:	e0 83 10 03 	l.and r4,r3,r2
    358c:	18 60 00 00 	l.movhi r3,0x0
    3590:	18 40 20 00 	l.movhi r2,0x2000
    3594:	a8 63 49 cc 	l.ori r3,r3,0x49cc
    3598:	84 63 00 08 	l.lwz r3,8(r3)
    359c:	b8 63 00 1e 	l.slli r3,r3,0x1e
    35a0:	e0 63 10 04 	l.or r3,r3,r2
    35a4:	e0 63 20 04 	l.or r3,r3,r4
    35a8:	c0 05 18 00 	l.mtspr r5,r3,0x0
    35ac:	9c 80 00 11 	l.addi r4,r0,17
    35b0:	b4 64 00 00 	l.mfspr r3,r4,0x0
    35b4:	a8 63 00 02 	l.ori r3,r3,0x2
    35b8:	c0 04 18 00 	l.mtspr r4,r3,0x0
    35bc:	9c 21 00 08 	l.addi r1,r1,8
    35c0:	84 21 ff f8 	l.lwz r1,-8(r1)
    35c4:	44 00 48 00 	l.jr r9
    35c8:	84 41 ff fc 	l.lwz r2,-4(r1)

000035cc <or1k_timer_disable>:
    35cc:	d7 e1 0f f8 	l.sw -8(r1),r1
    35d0:	d7 e1 17 fc 	l.sw -4(r1),r2
    35d4:	9c 60 00 11 	l.addi r3,r0,17
    35d8:	9c 21 ff f8 	l.addi r1,r1,-8
    35dc:	b5 63 00 00 	l.mfspr r11,r3,0x0
    35e0:	9c 40 ff fd 	l.addi r2,r0,-3
    35e4:	e0 8b 10 03 	l.and r4,r11,r2
    35e8:	c0 03 20 00 	l.mtspr r3,r4,0x0
    35ec:	9c 21 00 08 	l.addi r1,r1,8
    35f0:	b9 6b 00 41 	l.srli r11,r11,0x1
    35f4:	84 21 ff f8 	l.lwz r1,-8(r1)
    35f8:	a5 6b 00 01 	l.andi r11,r11,0x1
    35fc:	44 00 48 00 	l.jr r9
    3600:	84 41 ff fc 	l.lwz r2,-4(r1)

00003604 <or1k_timer_restore>:
    3604:	d7 e1 0f fc 	l.sw -4(r1),r1
    3608:	9c 80 00 11 	l.addi r4,r0,17
    360c:	9c 21 ff fc 	l.addi r1,r1,-4
    3610:	b4 64 00 00 	l.mfspr r3,r4,0x0
    3614:	a8 63 00 02 	l.ori r3,r3,0x2
    3618:	c0 04 18 00 	l.mtspr r4,r3,0x0
    361c:	9c 21 00 04 	l.addi r1,r1,4
    3620:	44 00 48 00 	l.jr r9
    3624:	84 21 ff fc 	l.lwz r1,-4(r1)

00003628 <or1k_timer_pause>:
    3628:	d7 e1 0f f8 	l.sw -8(r1),r1
    362c:	d7 e1 17 fc 	l.sw -4(r1),r2
    3630:	9c 80 50 00 	l.addi r4,r0,20480
    3634:	9c 21 ff f8 	l.addi r1,r1,-8
    3638:	b4 64 00 00 	l.mfspr r3,r4,0x0
    363c:	18 40 3f ff 	l.movhi r2,0x3fff
    3640:	a8 42 ff ff 	l.ori r2,r2,0xffff
    3644:	e0 63 10 03 	l.and r3,r3,r2
    3648:	c0 04 18 00 	l.mtspr r4,r3,0x0
    364c:	9c 21 00 08 	l.addi r1,r1,8
    3650:	84 21 ff f8 	l.lwz r1,-8(r1)
    3654:	44 00 48 00 	l.jr r9
    3658:	84 41 ff fc 	l.lwz r2,-4(r1)

0000365c <or1k_timer_reset>:
    365c:	d7 e1 0f f8 	l.sw -8(r1),r1
    3660:	d7 e1 17 fc 	l.sw -4(r1),r2
    3664:	9c 80 50 00 	l.addi r4,r0,20480
    3668:	9c 21 ff f8 	l.addi r1,r1,-8
    366c:	b4 64 00 00 	l.mfspr r3,r4,0x0
    3670:	18 40 ef ff 	l.movhi r2,0xefff
    3674:	a8 42 ff ff 	l.ori r2,r2,0xffff
    3678:	e0 63 10 03 	l.and r3,r3,r2
    367c:	c0 04 18 00 	l.mtspr r4,r3,0x0
    3680:	9c 80 00 00 	l.addi r4,r0,0
    3684:	9c 60 50 01 	l.addi r3,r0,20481
    3688:	c0 03 20 00 	l.mtspr r3,r4,0x0
    368c:	9c 21 00 08 	l.addi r1,r1,8
    3690:	84 21 ff f8 	l.lwz r1,-8(r1)
    3694:	44 00 48 00 	l.jr r9
    3698:	84 41 ff fc 	l.lwz r2,-4(r1)

0000369c <or1k_timer_get_ticks>:
    369c:	18 60 00 00 	l.movhi r3,0x0
    36a0:	d7 e1 0f fc 	l.sw -4(r1),r1
    36a4:	a8 63 49 cc 	l.ori r3,r3,0x49cc
    36a8:	9c 21 ff fc 	l.addi r1,r1,-4
    36ac:	85 63 00 00 	l.lwz r11,0(r3)
    36b0:	9c 21 00 04 	l.addi r1,r1,4
    36b4:	44 00 48 00 	l.jr r9
    36b8:	84 21 ff fc 	l.lwz r1,-4(r1)

000036bc <or1k_timer_reset_ticks>:
    36bc:	18 60 00 00 	l.movhi r3,0x0
    36c0:	d7 e1 17 fc 	l.sw -4(r1),r2
    36c4:	a8 63 49 cc 	l.ori r3,r3,0x49cc
    36c8:	9c 40 00 00 	l.addi r2,r0,0
    36cc:	d7 e1 0f f8 	l.sw -8(r1),r1
    36d0:	9c 21 ff f8 	l.addi r1,r1,-8
    36d4:	d4 03 10 00 	l.sw 0(r3),r2
    36d8:	9c 21 00 08 	l.addi r1,r1,8
    36dc:	84 21 ff f8 	l.lwz r1,-8(r1)
    36e0:	44 00 48 00 	l.jr r9
    36e4:	84 41 ff fc 	l.lwz r2,-4(r1)

000036e8 <_or1k_board_mem_base>:
    36e8:	00 00 00 00 	l.j 36e8 <_or1k_board_mem_base>

000036ec <_or1k_board_mem_size>:
    36ec:	08 00 00 00 	*unknown*

000036f0 <_or1k_board_clk_freq>:
    36f0:	03 f9 40 aa 	l.j ffe53998 <_end+0xffe4ef2c>

000036f4 <_or1k_board_uart_base>:
    36f4:	90 00 00 00 	l.lbs r0,0(r0)

000036f8 <_or1k_board_uart_baud>:
    36f8:	00 01 c2 00 	l.j 73ef8 <_end+0x6f48c>

000036fc <_or1k_board_uart_IRQ>:
    36fc:	00 00 00 02 	l.j 3704 <_or1k_board_exit+0x4>

00003700 <_or1k_board_exit>:
    3700:	15 00 00 0c 	l.nop 0xc
    3704:	00 00 00 00 	l.j 3704 <_or1k_board_exit+0x4>
    3708:	15 00 00 00 	l.nop 0x0

0000370c <_or1k_board_init_early>:
    370c:	44 00 48 00 	l.jr r9
    3710:	15 00 00 00 	l.nop 0x0

00003714 <_or1k_board_init>:
    3714:	44 00 48 00 	l.jr r9
    3718:	15 00 00 00 	l.nop 0x0

0000371c <__errno>:
    371c:	d7 e1 4f fc 	l.sw -4(r1),r9
    3720:	d7 e1 0f f8 	l.sw -8(r1),r1
    3724:	04 00 00 06 	l.jal 373c <__getreent>
    3728:	9c 21 ff f8 	l.addi r1,r1,-8
    372c:	9c 21 00 08 	l.addi r1,r1,8
    3730:	85 21 ff fc 	l.lwz r9,-4(r1)
    3734:	44 00 48 00 	l.jr r9
    3738:	84 21 ff f8 	l.lwz r1,-8(r1)

0000373c <__getreent>:
    373c:	03 ff fe c8 	l.j 325c <_or1k_libc_getreent>
    3740:	15 00 00 00 	l.nop 0x0

00003744 <memset>:
    3744:	a4 c3 00 03 	l.andi r6,r3,0x3
    3748:	d7 e1 0f f8 	l.sw -8(r1),r1
    374c:	d7 e1 17 fc 	l.sw -4(r1),r2
    3750:	bc 06 00 00 	l.sfeqi r6,0
    3754:	10 00 00 54 	l.bf 38a4 <memset+0x160>
    3758:	9c 21 ff f8 	l.addi r1,r1,-8
    375c:	bc 25 00 00 	l.sfnei r5,0
    3760:	0c 00 00 4c 	l.bnf 3890 <memset+0x14c>
    3764:	9c a5 ff ff 	l.addi r5,r5,-1
    3768:	b9 a4 00 18 	l.slli r13,r4,0x18
    376c:	a8 e3 00 00 	l.ori r7,r3,0x0
    3770:	a8 c3 00 00 	l.ori r6,r3,0x0
    3774:	00 00 00 05 	l.j 3788 <memset+0x44>
    3778:	b9 ad 00 98 	l.srai r13,r13,0x18
    377c:	bc 05 00 00 	l.sfeqi r5,0
    3780:	10 00 00 44 	l.bf 3890 <memset+0x14c>
    3784:	a8 ac 00 00 	l.ori r5,r12,0x0
    3788:	9c c6 00 01 	l.addi r6,r6,1
    378c:	d8 07 68 00 	l.sb 0(r7),r13
    3790:	a5 06 00 03 	l.andi r8,r6,0x3
    3794:	9d 85 ff ff 	l.addi r12,r5,-1
    3798:	bc 28 00 00 	l.sfnei r8,0
    379c:	13 ff ff f8 	l.bf 377c <memset+0x38>
    37a0:	9c e7 00 01 	l.addi r7,r7,1
    37a4:	bc a5 00 03 	l.sfleui r5,3
    37a8:	10 00 00 30 	l.bf 3868 <memset+0x124>
    37ac:	bc 05 00 00 	l.sfeqi r5,0
    37b0:	a4 e4 00 ff 	l.andi r7,r4,0xff
    37b4:	bc a5 00 0f 	l.sfleui r5,15
    37b8:	b9 07 00 08 	l.slli r8,r7,0x8
    37bc:	e0 e8 38 04 	l.or r7,r8,r7
    37c0:	b9 07 00 10 	l.slli r8,r7,0x10
    37c4:	10 00 00 1b 	l.bf 3830 <memset+0xec>
    37c8:	e0 e8 38 04 	l.or r7,r8,r7
    37cc:	9e 25 ff f0 	l.addi r17,r5,-16
    37d0:	9d 06 00 04 	l.addi r8,r6,4
    37d4:	ba 31 00 44 	l.srli r17,r17,0x4
    37d8:	9d e6 00 08 	l.addi r15,r6,8
    37dc:	9d a6 00 0c 	l.addi r13,r6,12
    37e0:	b9 71 00 04 	l.slli r11,r17,0x4
    37e4:	a9 86 00 00 	l.ori r12,r6,0x0
    37e8:	9d 6b 00 14 	l.addi r11,r11,20
    37ec:	e1 66 58 00 	l.add r11,r6,r11
    37f0:	d4 0c 38 00 	l.sw 0(r12),r7
    37f4:	d4 08 38 00 	l.sw 0(r8),r7
    37f8:	9d 08 00 10 	l.addi r8,r8,16
    37fc:	d4 0f 38 00 	l.sw 0(r15),r7
    3800:	d4 0d 38 00 	l.sw 0(r13),r7
    3804:	e4 28 58 00 	l.sfne r8,r11
    3808:	9d 8c 00 10 	l.addi r12,r12,16
    380c:	9d ef 00 10 	l.addi r15,r15,16
    3810:	13 ff ff f8 	l.bf 37f0 <memset+0xac>
    3814:	9d ad 00 10 	l.addi r13,r13,16
    3818:	9e 31 00 01 	l.addi r17,r17,1
    381c:	a4 a5 00 0f 	l.andi r5,r5,0xf
    3820:	ba 31 00 04 	l.slli r17,r17,0x4
    3824:	bc a5 00 03 	l.sfleui r5,3
    3828:	10 00 00 0f 	l.bf 3864 <memset+0x120>
    382c:	e0 c6 88 00 	l.add r6,r6,r17
    3830:	a9 86 00 00 	l.ori r12,r6,0x0
    3834:	a9 05 00 00 	l.ori r8,r5,0x0
    3838:	9d 08 ff fc 	l.addi r8,r8,-4
    383c:	d4 0c 38 00 	l.sw 0(r12),r7
    3840:	bc 48 00 03 	l.sfgtui r8,3
    3844:	13 ff ff fd 	l.bf 3838 <memset+0xf4>
    3848:	9d 8c 00 04 	l.addi r12,r12,4
    384c:	9c e5 ff fc 	l.addi r7,r5,-4
    3850:	9c 40 ff fc 	l.addi r2,r0,-4
    3854:	a4 a5 00 03 	l.andi r5,r5,0x3
    3858:	e0 e7 10 03 	l.and r7,r7,r2
    385c:	9c e7 00 04 	l.addi r7,r7,4
    3860:	e0 c6 38 00 	l.add r6,r6,r7
    3864:	bc 05 00 00 	l.sfeqi r5,0
    3868:	10 00 00 0a 	l.bf 3890 <memset+0x14c>
    386c:	15 00 00 00 	l.nop 0x0
    3870:	b8 84 00 18 	l.slli r4,r4,0x18
    3874:	e0 a6 28 00 	l.add r5,r6,r5
    3878:	b8 84 00 98 	l.srai r4,r4,0x18
    387c:	d8 06 20 00 	l.sb 0(r6),r4
    3880:	9c c6 00 01 	l.addi r6,r6,1
    3884:	e4 26 28 00 	l.sfne r6,r5
    3888:	13 ff ff fd 	l.bf 387c <memset+0x138>
    388c:	15 00 00 00 	l.nop 0x0
    3890:	9c 21 00 08 	l.addi r1,r1,8
    3894:	a9 63 00 00 	l.ori r11,r3,0x0
    3898:	84 21 ff f8 	l.lwz r1,-8(r1)
    389c:	44 00 48 00 	l.jr r9
    38a0:	84 41 ff fc 	l.lwz r2,-4(r1)
    38a4:	03 ff ff c0 	l.j 37a4 <memset+0x60>
    38a8:	a8 c3 00 00 	l.ori r6,r3,0x0

000038ac <__udivsi3>:
    38ac:	9c 21 ff fc 	l.addi r1,r1,-4
    38b0:	d4 01 48 00 	l.sw 0(r1),r9
    38b4:	9d 60 00 00 	l.addi r11,r0,0
    38b8:	9d 04 00 00 	l.addi r8,r4,0
    38bc:	9c a3 00 00 	l.addi r5,r3,0
    38c0:	e4 28 58 00 	l.sfne r8,r11
    38c4:	0c 00 00 36 	l.bnf 399c <__udivsi3+0xf0>
    38c8:	9c e0 00 00 	l.addi r7,r0,0
    38cc:	e4 48 28 00 	l.sfgtu r8,r5
    38d0:	10 00 00 32 	l.bf 3998 <__udivsi3+0xec>
    38d4:	e4 08 28 00 	l.sfeq r8,r5
    38d8:	10 00 00 2e 	l.bf 3990 <__udivsi3+0xe4>
    38dc:	e4 8b 40 00 	l.sfltu r11,r8
    38e0:	0c 00 00 0d 	l.bnf 3914 <__udivsi3+0x68>
    38e4:	9d a0 00 20 	l.addi r13,r0,32
    38e8:	19 20 80 00 	l.movhi r9,0x8000
    38ec:	9c c0 ff ff 	l.addi r6,r0,-1
    38f0:	e0 65 48 03 	l.and r3,r5,r9
    38f4:	b8 87 00 01 	l.slli r4,r7,0x1
    38f8:	9d e5 00 00 	l.addi r15,r5,0
    38fc:	b8 63 00 5f 	l.srli r3,r3,0x1f
    3900:	e1 ad 30 00 	l.add r13,r13,r6
    3904:	e0 e4 18 04 	l.or r7,r4,r3
    3908:	e4 87 40 00 	l.sfltu r7,r8
    390c:	13 ff ff f9 	l.bf 38f0 <__udivsi3+0x44>
    3910:	b8 a5 00 01 	l.slli r5,r5,0x1
    3914:	b8 e7 00 41 	l.srli r7,r7,0x1
    3918:	9d ad 00 01 	l.addi r13,r13,1
    391c:	9d 20 00 00 	l.addi r9,r0,0
    3920:	e4 89 68 00 	l.sfltu r9,r13
    3924:	0c 00 00 1e 	l.bnf 399c <__udivsi3+0xf0>
    3928:	9c af 00 00 	l.addi r5,r15,0
    392c:	19 e0 80 00 	l.movhi r15,0x8000
    3930:	9e 20 00 00 	l.addi r17,r0,0
    3934:	e0 65 78 03 	l.and r3,r5,r15
    3938:	b8 87 00 01 	l.slli r4,r7,0x1
    393c:	b8 63 00 5f 	l.srli r3,r3,0x1f
    3940:	e0 e4 18 04 	l.or r7,r4,r3
    3944:	e0 c7 40 02 	l.sub r6,r7,r8
    3948:	e0 66 78 03 	l.and r3,r6,r15
    394c:	b8 63 00 5f 	l.srli r3,r3,0x1f
    3950:	9c 80 00 00 	l.addi r4,r0,0
    3954:	e4 23 20 00 	l.sfne r3,r4
    3958:	10 00 00 03 	l.bf 3964 <__udivsi3+0xb8>
    395c:	b8 6b 00 01 	l.slli r3,r11,0x1
    3960:	9c 80 00 01 	l.addi r4,r0,1
    3964:	b8 a5 00 01 	l.slli r5,r5,0x1
    3968:	e4 24 88 00 	l.sfne r4,r17
    396c:	0c 00 00 03 	l.bnf 3978 <__udivsi3+0xcc>
    3970:	e1 63 20 04 	l.or r11,r3,r4
    3974:	9c e6 00 00 	l.addi r7,r6,0
    3978:	9d 29 00 01 	l.addi r9,r9,1
    397c:	e4 89 68 00 	l.sfltu r9,r13
    3980:	13 ff ff ed 	l.bf 3934 <__udivsi3+0x88>
    3984:	15 00 00 00 	l.nop 0x0
    3988:	00 00 00 05 	l.j 399c <__udivsi3+0xf0>
    398c:	15 00 00 00 	l.nop 0x0
    3990:	00 00 00 03 	l.j 399c <__udivsi3+0xf0>
    3994:	9d 60 00 01 	l.addi r11,r0,1
    3998:	9c e5 00 00 	l.addi r7,r5,0
    399c:	85 21 00 00 	l.lwz r9,0(r1)
    39a0:	44 00 48 00 	l.jr r9
    39a4:	9c 21 00 04 	l.addi r1,r1,4

000039a8 <__do_global_ctors_aux>:
    39a8:	d7 e1 17 f8 	l.sw -8(r1),r2
    39ac:	18 40 00 00 	l.movhi r2,0x0
    39b0:	d7 e1 4f fc 	l.sw -4(r1),r9
    39b4:	a8 42 40 34 	l.ori r2,r2,0x4034
    39b8:	d7 e1 0f f4 	l.sw -12(r1),r1
    39bc:	84 62 ff fc 	l.lwz r3,-4(r2)
    39c0:	9c 21 ff f4 	l.addi r1,r1,-12
    39c4:	bc 23 ff ff 	l.sfnei r3,-1
    39c8:	0c 00 00 08 	l.bnf 39e8 <__do_global_ctors_aux+0x40>
    39cc:	9c 42 ff fc 	l.addi r2,r2,-4
    39d0:	48 00 18 00 	l.jalr r3
    39d4:	9c 42 ff fc 	l.addi r2,r2,-4
    39d8:	84 62 00 00 	l.lwz r3,0(r2)
    39dc:	bc 23 ff ff 	l.sfnei r3,-1
    39e0:	13 ff ff fc 	l.bf 39d0 <__do_global_ctors_aux+0x28>
    39e4:	15 00 00 00 	l.nop 0x0
    39e8:	9c 21 00 0c 	l.addi r1,r1,12
    39ec:	85 21 ff fc 	l.lwz r9,-4(r1)
    39f0:	84 21 ff f4 	l.lwz r1,-12(r1)
    39f4:	44 00 48 00 	l.jr r9
    39f8:	84 41 ff f8 	l.lwz r2,-8(r1)

000039fc <call___do_global_ctors_aux>:
    39fc:	d7 e1 4f fc 	l.sw -4(r1),r9
    3a00:	d7 e1 0f f8 	l.sw -8(r1),r1
    3a04:	9c 21 ff f8 	l.addi r1,r1,-8
    3a08:	9c 21 00 08 	l.addi r1,r1,8
    3a0c:	85 21 ff fc 	l.lwz r9,-4(r1)
    3a10:	44 00 48 00 	l.jr r9
    3a14:	84 21 ff f8 	l.lwz r1,-8(r1)

Disassembly of section .fini:

00003a18 <_fini>:
    3a18:	9c 21 ff fc 	l.addi r1,r1,-4
    3a1c:	d4 01 48 00 	l.sw 0(r1),r9
    3a20:	07 ff f9 fa 	l.jal 2208 <__do_global_dtors_aux>
    3a24:	15 00 00 00 	l.nop 0x0
    3a28:	85 21 00 00 	l.lwz r9,0(r1)
    3a2c:	44 00 48 00 	l.jr r9
    3a30:	9c 21 00 04 	l.addi r1,r1,4

Disassembly of section .rodata:

00003a34 <_global_impure_ptr>:
    3a34:	00 00 40 54 	l.j 13b84 <_end+0xf118>
    3a38:	Address 0x0000000000003a38 is out of bounds.


Disassembly of section .eh_frame:

00004000 <__EH_FRAME_BEGIN__>:
    4000:	00 00 00 10 	l.j 4040 <__JCR_END__>
    4004:	00 00 00 00 	l.j 4004 <__EH_FRAME_BEGIN__+0x4>
    4008:	01 7a 52 00 	l.j 5e98808 <_end+0x5e93d9c>
    400c:	04 7c 09 01 	l.jal 1f06410 <_end+0x1f019a4>
    4010:	1b 0d 01 00 	*unknown*
    4014:	00 00 00 14 	l.j 4064 <impure_data+0x10>
    4018:	00 00 00 18 	l.j 4078 <impure_data+0x24>
    401c:	ff ff e3 7c 	*unknown*
    4020:	00 00 00 4c 	l.j 4150 <impure_data+0xfc>
    4024:	00 41 82 01 	l.j 1064828 <_end+0x105fdbc>
    4028:	41 0d 02 00 	*unknown*

0000402c <__FRAME_END__>:
    402c:	00 00 00 00 	l.j 402c <__FRAME_END__>

Disassembly of section .ctors:

00004030 <__CTOR_LIST__>:
    4030:	ff ff ff ff 	*unknown*

00004034 <__CTOR_END__>:
    4034:	00 00 00 00 	l.j 4034 <__CTOR_END__>

Disassembly of section .dtors:

00004038 <__DTOR_LIST__>:
    4038:	ff ff ff ff 	*unknown*

0000403c <__DTOR_END__>:
    403c:	00 00 00 00 	l.j 403c <__DTOR_END__>

Disassembly of section .jcr:

00004040 <__JCR_END__>:
    4040:	00 00 00 00 	l.j 4040 <__JCR_END__>

Disassembly of section .data:

00004044 <_or1k_stack_size>:
    4044:	00 00 20 00 	l.j c044 <_end+0x75d8>

00004048 <_or1k_exception_stack_size>:
    4048:	00 00 20 00 	l.j c048 <_end+0x75dc>

0000404c <__dso_handle>:
    404c:	00 00 00 00 	l.j 404c <__dso_handle>

00004050 <_impure_ptr>:
    4050:	00 00 40 54 	l.j 141a0 <_end+0xf734>

00004054 <impure_data>:
    4054:	00 00 00 00 	l.j 4054 <impure_data>
    4058:	00 00 43 40 	l.j 14d58 <_end+0x102ec>
    405c:	00 00 43 a8 	l.j 14efc <_end+0x10490>
    4060:	00 00 44 10 	l.j 150a0 <_end+0x10634>
	...
    4088:	00 00 3a 38 	l.j 12968 <_end+0xdefc>
	...
    40fc:	00 00 00 01 	l.j 4100 <impure_data+0xac>
    4100:	33 0e ab cd 	*unknown*
    4104:	12 34 e6 6d 	l.bf f8d3dab8 <_end+0xf8d3904c>
    4108:	de ec 00 05 	l.sh -18427(r12),r0
    410c:	00 0b 00 00 	l.j 2c410c <_end+0x2bf6a0>
	...

00004478 <environ>:
    4478:	00 00 48 c4 	l.j 16788 <_end+0x11d1c>

0000447c <_or1k_exception_impure_ptr>:
    447c:	00 00 44 80 	l.j 1567c <_end+0x10c10>

00004480 <_or1k_exception_impure_data>:
    4480:	00 00 00 00 	l.j 4480 <_or1k_exception_impure_data>
    4484:	00 00 47 6c 	l.j 16234 <_end+0x117c8>
    4488:	00 00 47 d4 	l.j 163d8 <_end+0x1196c>
    448c:	00 00 48 3c 	l.j 1657c <_end+0x11b10>
	...
    44b4:	00 00 3a 38 	l.j 12d94 <_end+0xe328>
	...
    4528:	00 00 00 01 	l.j 452c <_or1k_exception_impure_data+0xac>
    452c:	33 0e ab cd 	*unknown*
    4530:	12 34 e6 6d 	l.bf f8d3dee4 <_end+0xf8d39478>
    4534:	de ec 00 05 	l.sh -18427(r12),r0
    4538:	00 0b 00 00 	l.j 2c4538 <_end+0x2bfacc>
	...

Disassembly of section .bss:

000048a4 <__bss_start>:
    48a4:	00 00 00 00 	l.j 48a4 <__bss_start>

000048a8 <dtor_idx.2863>:
    48a8:	00 00 00 00 	l.j 48a8 <dtor_idx.2863>

000048ac <object.2876>:
	...

000048c4 <__env>:
    48c4:	00 00 00 00 	l.j 48c4 <__env>

000048c8 <_or1k_uart_read_cb>:
    48c8:	00 00 00 00 	l.j 48c8 <_or1k_uart_read_cb>

000048cc <_or1k_interrupt_handler_table>:
	...

0000494c <_or1k_interrupt_handler_data_ptr_table>:
	...

000049cc <_or1k_reent>:
	...

000049d8 <_or1k_current_impure_ptr>:
    49d8:	00 00 00 00 	l.j 49d8 <_or1k_current_impure_ptr>

000049dc <_or1k_impure_ptr>:
    49dc:	00 00 00 00 	l.j 49dc <_or1k_impure_ptr>

000049e0 <_or1k_exception_level>:
    49e0:	00 00 00 00 	l.j 49e0 <_or1k_exception_level>

000049e4 <_or1k_exception_stack_top>:
    49e4:	00 00 00 00 	l.j 49e4 <_or1k_exception_stack_top>

000049e8 <_or1k_stack_top>:
    49e8:	00 00 00 00 	l.j 49e8 <_or1k_stack_top>

000049ec <_or1k_exception_stack_bottom>:
    49ec:	00 00 00 00 	l.j 49ec <_or1k_exception_stack_bottom>

000049f0 <_or1k_stack_bottom>:
    49f0:	00 00 00 00 	l.j 49f0 <_or1k_stack_bottom>

000049f4 <_or1k_exception_handler_table>:
	...

Disassembly of section .comment:

00000000 <.comment>:
   0:	47 43 43 3a 	*unknown*
   4:	20 28 47 4e 	*unknown*
   8:	55 29 20 34 	*unknown*
   c:	2e 39 2e 32 	*unknown*
	...

Disassembly of section .debug_aranges:

00000000 <.debug_aranges>:
   0:	00 00 00 1c 	l.j 70 <_or1k_reset-0x90>
   4:	00 02 00 00 	l.j 80004 <_end+0x7b598>
   8:	00 00 04 00 	l.j 1008 <_or1k_reset+0xf08>
   c:	00 00 00 00 	l.j c <_or1k_reset-0xf4>
  10:	00 00 23 e4 	l.j 8fa0 <_end+0x4534>
  14:	00 00 00 30 	l.j d4 <_or1k_reset-0x2c>
	...
  20:	00 00 00 1c 	l.j 90 <_or1k_reset-0x70>
  24:	00 02 00 00 	l.j 80024 <_end+0x7b5b8>
  28:	01 0a 04 00 	l.j 4281028 <_end+0x427c5bc>
  2c:	00 00 00 00 	l.j 2c <_or1k_reset-0xd4>
  30:	00 00 24 14 	l.j 9080 <_end+0x4614>
  34:	00 00 00 48 	l.j 154 <_or1k_reset+0x54>
	...
  40:	00 00 00 14 	l.j 90 <_or1k_reset-0x70>
  44:	00 02 00 00 	l.j 80044 <_end+0x7b5d8>
  48:	09 ee 04 00 	*unknown*
	...
  58:	00 00 00 1c 	l.j c8 <_or1k_reset-0x38>
  5c:	00 02 00 00 	l.j 8005c <_end+0x7b5f0>
  60:	12 82 04 00 	l.bf fa081060 <_end+0xfa07c5f4>
  64:	00 00 00 00 	l.j 64 <_or1k_reset-0x9c>
  68:	00 00 24 5c 	l.j 91d8 <_end+0x476c>
  6c:	00 00 01 40 	l.j 56c <_or1k_reset+0x46c>
	...
  78:	00 00 00 1c 	l.j e8 <_or1k_reset-0x18>
  7c:	00 02 00 00 	l.j 8007c <_end+0x7b610>
  80:	1b b6 04 00 	*unknown*
  84:	00 00 00 00 	l.j 84 <_or1k_reset-0x7c>
  88:	00 00 25 9c 	l.j 96f8 <_end+0x4c8c>
  8c:	00 00 01 e8 	l.j 82c <_or1k_reset+0x72c>
	...
  98:	00 00 00 1c 	l.j 108 <_or1k_reset+0x8>
  9c:	00 02 00 00 	l.j 8009c <_end+0x7b630>
  a0:	25 0f 04 00 	*unknown*
  a4:	00 00 00 00 	l.j a4 <_or1k_reset-0x5c>
  a8:	00 00 27 84 	l.j 9eb8 <_end+0x544c>
  ac:	00 00 02 a8 	l.j b4c <_or1k_reset+0xa4c>
	...
  b8:	00 00 00 1c 	l.j 128 <_or1k_reset+0x28>
  bc:	00 02 00 00 	l.j 800bc <_end+0x7b650>
  c0:	34 4d 04 00 	*unknown*
  c4:	00 00 00 00 	l.j c4 <_or1k_reset-0x3c>
  c8:	00 00 2a 2c 	l.j a978 <_end+0x5f0c>
  cc:	00 00 01 b4 	l.j 79c <_or1k_reset+0x69c>
	...
  d8:	00 00 00 1c 	l.j 148 <_or1k_reset+0x48>
  dc:	00 02 00 00 	l.j 800dc <_end+0x7b670>
  e0:	36 5b 04 00 	*unknown*
  e4:	00 00 00 00 	l.j e4 <_or1k_reset-0x1c>
  e8:	00 00 2f 38 	l.j bdc8 <_end+0x735c>
  ec:	00 00 00 e4 	l.j 47c <_or1k_reset+0x37c>
	...
  f8:	00 00 00 1c 	l.j 168 <_or1k_reset+0x68>
  fc:	00 02 00 00 	l.j 800fc <_end+0x7b690>
 100:	39 4f 04 00 	*unknown*
 104:	00 00 00 00 	l.j 104 <_or1k_reset+0x4>
 108:	00 00 30 e4 	l.j c498 <_end+0x7a2c>
 10c:	00 00 01 ac 	l.j 7bc <_or1k_reset+0x6bc>
	...
 118:	00 00 00 1c 	l.j 188 <_or1k_reset+0x88>
 11c:	00 02 00 00 	l.j 8011c <_end+0x7b6b0>
 120:	42 e1 04 00 	*unknown*
 124:	00 00 00 00 	l.j 124 <_or1k_reset+0x24>
 128:	00 00 32 90 	l.j cb68 <_end+0x80fc>
 12c:	00 00 00 c0 	l.j 42c <_or1k_reset+0x32c>
	...
 138:	00 00 00 1c 	l.j 1a8 <_or1k_reset+0xa8>
 13c:	00 02 00 00 	l.j 8013c <_end+0x7b6d0>
 140:	44 ff 04 00 	*unknown*
 144:	00 00 00 00 	l.j 144 <_or1k_reset+0x44>
 148:	00 00 33 50 	l.j ce88 <_end+0x841c>
 14c:	00 00 00 34 	l.j 21c <_or1k_reset+0x11c>
	...
 158:	00 00 00 1c 	l.j 1c8 <_or1k_reset+0xc8>
 15c:	00 02 00 00 	l.j 8015c <_end+0x7b6f0>
 160:	45 e8 04 00 	*unknown*
 164:	00 00 00 00 	l.j 164 <_or1k_reset+0x64>
 168:	00 00 33 84 	l.j cf78 <_end+0x850c>
 16c:	00 00 03 64 	l.j efc <_or1k_reset+0xdfc>
	...
 178:	00 00 00 1c 	l.j 1e8 <_or1k_reset+0xe8>
 17c:	00 02 00 00 	l.j 8017c <_end+0x7b710>
 180:	4c 94 04 00 	*unknown*
 184:	00 00 00 00 	l.j 184 <_or1k_reset+0x84>
 188:	00 00 37 1c 	l.j ddf8 <_end+0x938c>
 18c:	00 00 00 20 	l.j 20c <_or1k_reset+0x10c>
	...
 198:	00 00 00 1c 	l.j 208 <_or1k_reset+0x108>
 19c:	00 02 00 00 	l.j 8019c <_end+0x7b730>
 1a0:	55 2b 04 00 	*unknown*
 1a4:	00 00 00 00 	l.j 1a4 <_or1k_reset+0xa4>
 1a8:	00 00 37 44 	l.j deb8 <_end+0x944c>
 1ac:	00 00 01 68 	l.j 74c <_or1k_reset+0x64c>
	...

Disassembly of section .debug_info:

00000000 <.debug_info>:
       0:	00 00 01 06 	l.j 418 <_or1k_reset+0x318>
       4:	00 04 00 00 	l.j 100004 <_end+0xfb598>
       8:	00 00 04 01 	l.j 100c <_or1k_reset+0xf0c>
       c:	00 00 00 30 	l.j cc <_or1k_reset-0x34>
      10:	01 00 00 00 	l.j 4000010 <_end+0x3ffb5a4>
      14:	ee 00 00 00 	*unknown*
      18:	9b 00 00 23 	l.lhs r24,35(r0)
      1c:	e4 00 00 00 	l.sfeq r0,r0
      20:	30 00 00 00 	*unknown*
      24:	00 02 04 07 	l.j 81040 <_end+0x7c5d4>
      28:	00 00 00 5e 	l.j 1a0 <_or1k_reset+0xa0>
      2c:	03 04 05 69 	l.j fc1015d0 <_end+0xfc0fcb64>
      30:	6e 74 00 02 	l.lwa r19,2(r20)
      34:	04 05 00 00 	l.jal 140034 <_end+0x13b5c8>
      38:	00 05 02 01 	l.j 14083c <_end+0x13bdd0>
      3c:	06 00 00 00 	l.jal f800003c <_end+0xf7ffb5d0>
      40:	7b 02 01 08 	*unknown*
      44:	00 00 00 79 	l.j 228 <_or1k_reset+0x128>
      48:	02 02 05 00 	l.j f8081448 <_end+0xf807c9dc>
      4c:	00 01 2f 02 	l.j 4bc54 <_end+0x471e8>
      50:	02 07 00 00 	l.j f81c0050 <_end+0xf81bb5e4>
      54:	00 0e 02 08 	l.j 380874 <_end+0x37be08>
      58:	05 00 00 00 	l.jal 4000058 <_end+0x3ffb5ec>
      5c:	00 02 08 07 	l.j 82078 <_end+0x7d60c>
      60:	00 00 00 59 	l.j 1c4 <_or1k_reset+0xc4>
      64:	02 04 07 00 	l.j f8101c64 <_end+0xf80fd1f8>
      68:	00 00 63 02 	l.j 18c70 <_end+0x14204>
      6c:	04 07 00 00 	l.jal 1c006c <_end+0x1bb600>
      70:	01 45 04 04 	l.j 5141080 <_end+0x513c614>
      74:	05 04 00 00 	l.jal 4100074 <_end+0x40fb608>
      78:	00 7a 06 02 	l.j 1e81880 <_end+0x1e7ce14>
      7c:	01 06 00 00 	l.j 418007c <_end+0x417b610>
      80:	00 82 07 00 	l.j 2081c80 <_end+0x207d214>
      84:	00 00 21 04 	l.j 8494 <_end+0x3a28>
      88:	02 05 00 00 	l.j f8140088 <_end+0xf813b61c>
      8c:	00 a1 08 00 	l.j 284208c <_end+0x283d620>
      90:	00 01 39 00 	l.j 4e490 <_end+0x49a24>
      94:	08 00 00 00 	*unknown*
      98:	e2 01 08 00 	l.add r16,r1,r1
      9c:	00 00 70 02 	l.j 1c0a4 <_end+0x17638>
      a0:	00 09 00 00 	l.j 2400a0 <_end+0x23b634>
      a4:	01 3e 03 45 	l.j 4f80db8 <_end+0x4f7c34c>
      a8:	00 00 00 2c 	l.j 158 <_or1k_reset+0x58>
      ac:	00 00 23 e4 	l.j 903c <_end+0x45d0>
      b0:	00 00 00 30 	l.j 170 <_or1k_reset+0x70>
      b4:	01 9c 00 00 	l.j 67000b4 <_end+0x66fb648>
      b8:	00 e9 0a 66 	l.j 3a42a50 <_end+0x3a3dfe4>
      bc:	6e 00 01 3d 	l.lwa r16,317(r0)
      c0:	00 00 00 74 	l.j 290 <_or1k_reset+0x190>
      c4:	00 00 00 00 	l.j c4 <_or1k_reset-0x3c>
      c8:	0b 00 00 24 	*unknown*
      cc:	04 00 00 00 	l.jal cc <_or1k_reset-0x34>
      d0:	e9 0c 01 56 	*unknown*
      d4:	01 30 0c 01 	l.j 4c030d8 <_end+0x4bfe66c>
      d8:	55 01 30 0c 	*unknown*
      dc:	01 54 03 f3 	l.j 55010a8 <_end+0x54fc63c>
      e0:	01 53 0c 01 	l.j 54c30e4 <_end+0x54be678>
      e4:	53 01 30 00 	*unknown*
      e8:	00 0d 00 00 	l.j 3400e8 <_end+0x33b67c>
      ec:	00 87 02 0d 	l.j 21c0920 <_end+0x21bbeb4>
      f0:	00 00 00 2c 	l.j 1a0 <_or1k_reset+0xa0>
      f4:	0e 00 00 00 	l.bnf f80000f4 <_end+0xf7ffb688>
      f8:	2c 0e 00 00 	*unknown*
      fc:	00 74 0e 00 	l.j 1d038fc <_end+0x1cfee90>
     100:	00 00 72 0e 	l.j 1c938 <_end+0x17ecc>
     104:	00 00 00 72 	l.j 2cc <_or1k_reset+0x1cc>
     108:	00 00 00 00 	l.j 108 <_or1k_reset+0x8>
     10c:	08 e0 00 04 	*unknown*
     110:	00 00 00 b5 	l.j 3e4 <_or1k_reset+0x2e4>
     114:	04 01 00 00 	l.jal 40114 <_end+0x3b6a8>
     118:	00 30 01 00 	l.j c00518 <_end+0xbfbaac>
     11c:	00 02 a8 00 	l.j aa11c <_end+0xa56b0>
     120:	00 00 9b 00 	l.j 26d20 <_end+0x222b4>
     124:	00 24 14 00 	l.j 905124 <_end+0x9006b8>
     128:	00 00 48 00 	l.j 12128 <_end+0xd6bc>
     12c:	00 00 d6 02 	l.j 35934 <_end+0x30ec8>
     130:	04 07 00 00 	l.jal 1c0130 <_end+0x1bb6c4>
     134:	00 5e 03 04 	l.j 1780d44 <_end+0x177c2d8>
     138:	05 69 6e 74 	l.jal 5a5bb08 <_end+0x5a5709c>
     13c:	00 02 04 05 	l.j 81150 <_end+0x7c6e4>
     140:	00 00 00 05 	l.j 154 <_or1k_reset+0x54>
     144:	02 01 06 00 	l.j f8041944 <_end+0xf803ced8>
     148:	00 00 7b 02 	l.j 1ed50 <_end+0x1a2e4>
     14c:	01 08 00 00 	l.j 420014c <_end+0x41fb6e0>
     150:	00 79 02 02 	l.j 1e40958 <_end+0x1e3beec>
     154:	05 00 00 01 	l.jal 4000158 <_end+0x3ffb6ec>
     158:	2f 02 02 07 	*unknown*
     15c:	00 00 00 0e 	l.j 194 <_or1k_reset+0x94>
     160:	02 08 05 00 	l.j f8201560 <_end+0xf81fcaf4>
     164:	00 00 00 02 	l.j 16c <_or1k_reset+0x6c>
     168:	08 07 00 00 	*unknown*
     16c:	00 59 04 00 	l.j 164116c <_end+0x163c700>
     170:	00 03 6d 02 	l.j db578 <_end+0xd6b0c>
     174:	07 00 00 00 	l.jal fc000174 <_end+0xfbffb708>
     178:	2c 04 00 00 	*unknown*
     17c:	03 4b 03 10 	l.j fd2c0dbc <_end+0xfd2bc350>
     180:	00 00 00 33 	l.j 24c <_or1k_reset+0x14c>
     184:	04 00 00 04 	l.jal 194 <_or1k_reset+0x94>
     188:	27 03 27 00 	*unknown*
     18c:	00 00 33 05 	l.j cda0 <_end+0x8334>
     190:	00 00 03 17 	l.j dec <_or1k_reset+0xcec>
     194:	04 01 61 00 	l.jal 58594 <_end+0x53b28>
     198:	00 00 91 02 	l.j 245a0 <_end+0x1fb34>
     19c:	04 07 00 00 	l.jal 1c019c <_end+0x1bb730>
     1a0:	00 63 06 04 	l.j 18c19b0 <_end+0x18bcf44>
     1a4:	03 4a 00 00 	l.j fd2801a4 <_end+0xfd27b738>
     1a8:	00 b7 07 00 	l.j 2dc1da8 <_end+0x2dbd33c>
     1ac:	00 03 11 03 	l.j c45b8 <_end+0xbfb4c>
     1b0:	4c 00 00 00 	l.maci r0,0
     1b4:	85 07 00 00 	l.lwz r8,0(r7)
     1b8:	02 84 03 4d 	l.j fa100eec <_end+0xfa0fc480>
     1bc:	00 00 00 b7 	l.j 498 <_or1k_reset+0x398>
     1c0:	00 08 00 00 	l.j 2001c0 <_end+0x1fb754>
     1c4:	00 41 00 00 	l.j 10401c4 <_end+0x103b758>
     1c8:	00 c7 09 00 	l.j 31c25c8 <_end+0x31bdb5c>
     1cc:	00 00 c7 03 	l.j 31dd8 <_end+0x2d36c>
     1d0:	00 02 04 07 	l.j 811ec <_end+0x7c780>
     1d4:	00 00 01 45 	l.j 6e8 <_or1k_reset+0x5e8>
     1d8:	0a 08 03 47 	*unknown*
     1dc:	00 00 00 ef 	l.j 598 <_or1k_reset+0x498>
     1e0:	0b 00 00 04 	*unknown*
     1e4:	11 03 49 00 	l.bf 40d25e4 <_end+0x40cdb78>
     1e8:	00 00 2c 00 	l.j b1e8 <_end+0x677c>
     1ec:	0b 00 00 04 	*unknown*
     1f0:	19 03 4e 00 	*unknown*
     1f4:	00 00 98 04 	l.j 26204 <_end+0x21798>
     1f8:	00 04 00 00 	l.j 1001f8 <_end+0xfb78c>
     1fc:	03 ab 03 4f 	l.j feac0f38 <_end+0xfeabc4cc>
     200:	00 00 00 ce 	l.j 538 <_or1k_reset+0x438>
     204:	04 00 00 02 	l.jal 20c <_or1k_reset+0x10c>
     208:	47 03 53 00 	*unknown*
     20c:	00 00 64 0c 	l.j 1923c <_end+0x147d0>
     210:	04 04 00 00 	l.jal 100210 <_end+0xfb7a4>
     214:	04 54 05 16 	l.jal 150166c <_end+0x14fcc00>
     218:	00 00 00 25 	l.j 2ac <_or1k_reset+0x1ac>
     21c:	0d 00 00 02 	l.bnf 4000224 <_end+0x3ffb7b8>
     220:	58 18 05 2d 	*unknown*
     224:	00 00 01 65 	l.j 7b8 <_or1k_reset+0x6b8>
     228:	0b 00 00 03 	*unknown*
     22c:	cc 05 2f 00 	l.swa 1792(r5),r5
     230:	00 01 65 00 	l.j 59630 <_end+0x54bc4>
     234:	0e 5f 6b 00 	l.bnf f97dae34 <_end+0xf97d63c8>
     238:	05 30 00 00 	l.jal 4c00238 <_end+0x4bfb7cc>
     23c:	00 2c 04 0b 	l.j b01268 <_end+0xafc7fc>
     240:	00 00 04 03 	l.j 124c <_or1k_reset+0x114c>
     244:	05 30 00 00 	l.jal 4c00244 <_end+0x4bfb7d8>
     248:	00 2c 08 0b 	l.j b02274 <_end+0xafd808>
     24c:	00 00 02 41 	l.j b50 <_or1k_reset+0xa50>
     250:	05 30 00 00 	l.jal 4c00250 <_end+0x4bfb7e4>
     254:	00 2c 0c 0b 	l.j b03280 <_end+0xafe814>
     258:	00 00 04 a0 	l.j 14d8 <_or1k_reset+0x13d8>
     25c:	05 30 00 00 	l.jal 4c0025c <_end+0x4bfb7f0>
     260:	00 2c 10 0e 	l.j b04298 <_end+0xaff82c>
     264:	5f 78 00 05 	*unknown*
     268:	31 00 00 01 	*unknown*
     26c:	6b 14 00 0f 	*unknown*
     270:	04 00 00 01 	l.jal 274 <_or1k_reset+0x174>
     274:	12 08 00 00 	l.bf f8200274 <_end+0xf81fb808>
     278:	01 07 00 00 	l.j 41c0278 <_end+0x41bb80c>
     27c:	01 7b 09 00 	l.j 5ec267c <_end+0x5ebdc10>
     280:	00 00 c7 00 	l.j 31e80 <_end+0x2d414>
     284:	00 0d 00 00 	l.j 340284 <_end+0x33b818>
     288:	02 7f 24 05 	l.j f9fc929c <_end+0xf9fc4830>
     28c:	35 00 00 01 	*unknown*
     290:	f4 0b 00 00 	*unknown*
     294:	01 b3 05 37 	l.j 6cc1770 <_end+0x6cbcd04>
     298:	00 00 00 2c 	l.j 348 <_or1k_reset+0x248>
     29c:	00 0b 00 00 	l.j 2c029c <_end+0x2bb830>
     2a0:	04 2f 05 38 	l.jal bc1780 <_end+0xbbcd14>
     2a4:	00 00 00 2c 	l.j 354 <_or1k_reset+0x254>
     2a8:	04 0b 00 00 	l.jal 2c02a8 <_end+0x2bb83c>
     2ac:	01 c2 05 39 	l.j 7081790 <_end+0x707cd24>
     2b0:	00 00 00 2c 	l.j 360 <_or1k_reset+0x260>
     2b4:	08 0b 00 00 	*unknown*
     2b8:	05 19 05 3a 	l.jal 46417a0 <_end+0x463cd34>
     2bc:	00 00 00 2c 	l.j 36c <_or1k_reset+0x26c>
     2c0:	0c 0b 00 00 	l.bnf 2c02c0 <_end+0x2bb854>
     2c4:	03 37 05 3b 	l.j fcdc17b0 <_end+0xfcdbcd44>
     2c8:	00 00 00 2c 	l.j 378 <_or1k_reset+0x278>
     2cc:	10 0b 00 00 	l.bf 2c02cc <_end+0x2bb860>
     2d0:	03 26 05 3c 	l.j fc9817c0 <_end+0xfc97cd54>
     2d4:	00 00 00 2c 	l.j 384 <_or1k_reset+0x284>
     2d8:	14 0b 00 00 	*unknown*
     2dc:	04 a5 05 3d 	l.jal 29417d0 <_end+0x293cd64>
     2e0:	00 00 00 2c 	l.j 390 <_or1k_reset+0x290>
     2e4:	18 0b 00 00 	*unknown*
     2e8:	03 8d 05 3e 	l.j fe3417e0 <_end+0xfe33cd74>
     2ec:	00 00 00 2c 	l.j 39c <_or1k_reset+0x29c>
     2f0:	1c 0b 00 00 	*unknown*
     2f4:	04 e0 05 3f 	l.jal 38017f0 <_end+0x37fcd84>
     2f8:	00 00 00 2c 	l.j 3a8 <_or1k_reset+0x2a8>
     2fc:	20 00 10 00 	l.sys 0x1000
     300:	00 01 d1 01 	l.j 74704 <_end+0x6fc98>
     304:	08 05 48 00 	*unknown*
     308:	00 02 34 0b 	l.j 8d334 <_end+0x888c8>
     30c:	00 00 02 34 	l.j bdc <_or1k_reset+0xadc>
     310:	05 49 00 00 	l.jal 5240310 <_end+0x523b8a4>
     314:	02 34 00 0b 	l.j f8d00340 <_end+0xf8cfb8d4>
     318:	00 00 01 4e 	l.j 850 <_or1k_reset+0x750>
     31c:	05 4a 00 00 	l.jal 528031c <_end+0x527b8b0>
     320:	02 34 80 11 	l.j f8d20364 <_end+0xf8d1b8f8>
     324:	00 00 04 4b 	l.j 1450 <_or1k_reset+0x1350>
     328:	05 4c 00 00 	l.jal 5300328 <_end+0x52fb8bc>
     32c:	01 07 01 00 	l.j 41c072c <_end+0x41bbcc0>
     330:	11 00 00 01 	l.bf 4000334 <_end+0x3ffb8c8>
     334:	f6 05 4f 00 	*unknown*
     338:	00 01 07 01 	l.j 41f3c <_end+0x3d4d0>
     33c:	04 00 08 00 	l.jal 233c <frame_dummy+0x40>
     340:	00 01 05 00 	l.j 41740 <_end+0x3ccd4>
     344:	00 02 44 09 	l.j 91368 <_end+0x8c8fc>
     348:	00 00 00 c7 	l.j 664 <_or1k_reset+0x564>
     34c:	1f 00 10 00 	*unknown*
     350:	00 01 3d 01 	l.j 4f754 <_end+0x4ace8>
     354:	90 05 5b 00 	l.lbs r0,23296(r5)
     358:	00 02 82 0b 	l.j a0b84 <_end+0x9c118>
     35c:	00 00 03 cc 	l.j 128c <_or1k_reset+0x118c>
     360:	05 5c 00 00 	l.jal 5700360 <_end+0x56fb8f4>
     364:	02 82 00 0b 	l.j fa080390 <_end+0xfa07b924>
     368:	00 00 03 e4 	l.j 12f8 <_or1k_reset+0x11f8>
     36c:	05 5d 00 00 	l.jal 574036c <_end+0x573b900>
     370:	00 2c 04 0b 	l.j b0139c <_end+0xafc930>
     374:	00 00 02 3c 	l.j c64 <_or1k_reset+0xb64>
     378:	05 5f 00 00 	l.jal 57c0378 <_end+0x57bb90c>
     37c:	02 88 08 0b 	l.j fa2023a8 <_end+0xfa1fd93c>
     380:	00 00 01 d1 	l.j ac4 <_or1k_reset+0x9c4>
     384:	05 60 00 00 	l.jal 5800384 <_end+0x57fb918>
     388:	01 f4 88 00 	l.j 7d22388 <_end+0x7d1d91c>
     38c:	0f 04 00 00 	l.bnf fc10038c <_end+0xfc0fb920>
     390:	02 44 08 00 	l.j f9102390 <_end+0xf90fd924>
     394:	00 02 98 00 	l.j a6394 <_end+0xa1928>
     398:	00 02 98 09 	l.j a63bc <_end+0xa1950>
     39c:	00 00 00 c7 	l.j 6b8 <_or1k_reset+0x5b8>
     3a0:	1f 00 0f 04 	*unknown*
     3a4:	00 00 02 9e 	l.j e1c <_or1k_reset+0xd1c>
     3a8:	12 0d 00 00 	l.bf f83403a8 <_end+0xf833b93c>
     3ac:	03 97 08 05 	l.j fe5c23c0 <_end+0xfe5bd954>
     3b0:	73 00 00 02 	*unknown*
     3b4:	c4 0b 00 00 	*unknown*
     3b8:	09 f8 05 74 	*unknown*
     3bc:	00 00 02 c4 	l.j ecc <_or1k_reset+0xdcc>
     3c0:	00 0b 00 00 	l.j 2c03c0 <_end+0x2bb954>
     3c4:	07 8b 05 75 	l.jal fe2c1998 <_end+0xfe2bcf2c>
     3c8:	00 00 00 2c 	l.j 478 <_or1k_reset+0x378>
     3cc:	04 00 0f 04 	l.jal 3fdc <_global_impure_ptr+0x5a8>
     3d0:	00 00 00 41 	l.j 4d4 <_or1k_reset+0x3d4>
     3d4:	0d 00 00 03 	l.bnf 40003e0 <_end+0x3ffb974>
     3d8:	b6 68 05 b3 	l.mfspr r19,r8,0x5b3
     3dc:	00 00 03 f4 	l.j 13ac <_or1k_reset+0x12ac>
     3e0:	0e 5f 70 00 	l.bnf f97dc3e0 <_end+0xf97d7974>
     3e4:	05 b4 00 00 	l.jal 6d003e4 <_end+0x6cfb978>
     3e8:	02 c4 00 0e 	l.j fb100420 <_end+0xfb0fb9b4>
     3ec:	5f 72 00 05 	*unknown*
     3f0:	b5 00 00 00 	l.mfspr r8,r0,0x0
     3f4:	2c 04 0e 5f 	*unknown*
     3f8:	77 00 05 b6 	*unknown*
     3fc:	00 00 00 2c 	l.j 4ac <_or1k_reset+0x3ac>
     400:	08 0b 00 00 	*unknown*
     404:	01 ef 05 b7 	l.j 7bc1ae0 <_end+0x7bbd074>
     408:	00 00 00 48 	l.j 528 <_or1k_reset+0x428>
     40c:	0c 0b 00 00 	l.bnf 2c040c <_end+0x2bb9a0>
     410:	02 9b 05 b8 	l.j fa6c1af0 <_end+0xfa6bd084>
     414:	00 00 00 48 	l.j 534 <_or1k_reset+0x434>
     418:	0e 0e 5f 62 	l.bnf f83981a0 <_end+0xf8393734>
     41c:	66 00 05 b9 	*unknown*
     420:	00 00 02 9f 	l.j e9c <_or1k_reset+0xd9c>
     424:	10 0b 00 00 	l.bf 2c0424 <_end+0x2bb9b8>
     428:	01 8d 05 ba 	l.j 6341b10 <_end+0x633d0a4>
     42c:	00 00 00 2c 	l.j 4dc <_or1k_reset+0x3dc>
     430:	18 0b 00 00 	*unknown*
     434:	01 df 05 c1 	l.j 77c1b38 <_end+0x77bd0cc>
     438:	00 00 01 05 	l.j 84c <_or1k_reset+0x74c>
     43c:	1c 0b 00 00 	*unknown*
     440:	02 6f 05 c3 	l.j f9bc1b4c <_end+0xf9bbd0e0>
     444:	00 00 05 57 	l.j 19a0 <_or1k_reset+0x18a0>
     448:	20 0b 00 00 	*unknown*
     44c:	09 a6 05 c5 	*unknown*
     450:	00 00 05 86 	l.j 1a68 <_or1k_reset+0x1968>
     454:	24 0b 00 00 	*unknown*
     458:	04 21 05 c8 	l.jal 841b78 <_end+0x83d10c>
     45c:	00 00 05 aa 	l.j 1b04 <_or1k_reset+0x1a04>
     460:	28 0b 00 00 	*unknown*
     464:	04 fa 05 c9 	l.jal 3e81b88 <_end+0x3e7d11c>
     468:	00 00 05 c4 	l.j 1b78 <_or1k_reset+0x1a78>
     46c:	2c 0e 5f 75 	*unknown*
     470:	62 00 05 cc 	*unknown*
     474:	00 00 02 9f 	l.j ef0 <_or1k_reset+0xdf0>
     478:	30 0e 5f 75 	*unknown*
     47c:	70 00 05 cd 	*unknown*
     480:	00 00 02 c4 	l.j f90 <_or1k_reset+0xe90>
     484:	38 0e 5f 75 	*unknown*
     488:	72 00 05 ce 	*unknown*
     48c:	00 00 00 2c 	l.j 53c <_or1k_reset+0x43c>
     490:	3c 0b 00 00 	*unknown*
     494:	01 bc 05 d1 	l.j 6f01bd8 <_end+0x6efd16c>
     498:	00 00 05 ca 	l.j 1bc0 <_or1k_reset+0x1ac0>
     49c:	40 0b 00 00 	*unknown*
     4a0:	04 d2 05 d2 	l.jal 3481be8 <_end+0x347d17c>
     4a4:	00 00 05 da 	l.j 1c0c <_or1k_reset+0x1b0c>
     4a8:	43 0e 5f 6c 	*unknown*
     4ac:	62 00 05 d5 	*unknown*
     4b0:	00 00 02 9f 	l.j f2c <_or1k_reset+0xe2c>
     4b4:	44 0b 00 00 	*unknown*
     4b8:	08 18 05 d8 	*unknown*
     4bc:	00 00 00 2c 	l.j 56c <_or1k_reset+0x46c>
     4c0:	4c 0b 00 00 	l.maci r11,0
     4c4:	02 0d 05 d9 	l.j f8341c28 <_end+0xf833d1bc>
     4c8:	00 00 00 6f 	l.j 684 <_or1k_reset+0x584>
     4cc:	50 0b 00 00 	*unknown*
     4d0:	0c 6a 05 dc 	l.bnf 1a81c40 <_end+0x1a7d1d4>
     4d4:	00 00 04 12 	l.j 151c <_or1k_reset+0x141c>
     4d8:	54 0b 00 00 	*unknown*
     4dc:	06 4f 05 e0 	l.jal f93c1c5c <_end+0xf93bd1f0>
     4e0:	00 00 00 fa 	l.j 8c8 <_or1k_reset+0x7c8>
     4e4:	58 0b 00 00 	*unknown*
     4e8:	03 be 05 e2 	l.j fef81c70 <_end+0xfef7d204>
     4ec:	00 00 00 ef 	l.j 8a8 <_or1k_reset+0x7a8>
     4f0:	5c 0b 00 00 	*unknown*
     4f4:	03 1e 05 e3 	l.j fc781c80 <_end+0xfc77d214>
     4f8:	00 00 00 2c 	l.j 5a8 <_or1k_reset+0x4a8>
     4fc:	64 00 13 00 	*unknown*
     500:	00 00 2c 00 	l.j b500 <_end+0x6a94>
     504:	00 04 12 14 	l.j 104d54 <_end+0x1002e8>
     508:	00 00 04 12 	l.j 1550 <_or1k_reset+0x1450>
     50c:	14 00 00 01 	*unknown*
     510:	05 14 00 00 	l.jal 4500510 <_end+0x44fbaa4>
     514:	05 4a 14 00 	l.jal 5285514 <_end+0x5280aa8>
     518:	00 00 2c 00 	l.j b518 <_end+0x6aac>
     51c:	0f 04 00 00 	l.bnf fc10051c <_end+0xfc0fbab0>
     520:	04 18 15 00 	l.jal 605920 <_end+0x600eb4>
     524:	00 0c 0b 04 	l.j 303134 <_end+0x2fe6c8>
     528:	24 05 02 39 	*unknown*
     52c:	00 00 05 4a 	l.j 1a54 <_or1k_reset+0x1954>
     530:	16 00 00 06 	*unknown*
     534:	bd 05 02 3b 	*unknown*
     538:	00 00 00 2c 	l.j 5e8 <_or1k_reset+0x4e8>
     53c:	00 16 00 00 	l.j 58053c <_end+0x57bad0>
     540:	01 fe 05 02 	l.j 7f81948 <_end+0x7f7cedc>
     544:	40 00 00 06 	*unknown*
     548:	31 04 16 00 	*unknown*
     54c:	00 02 8b 05 	l.j a3160 <_end+0x9e6f4>
     550:	02 40 00 00 	l.j f9000550 <_end+0xf8ffbae4>
     554:	06 31 08 16 	l.jal f8c425ac <_end+0xf8c3db40>
     558:	00 00 02 50 	l.j e98 <_or1k_reset+0xd98>
     55c:	05 02 40 00 	l.jal 409055c <_end+0x408baf0>
     560:	00 06 31 0c 	l.j 18c990 <_end+0x187f24>
     564:	16 00 00 03 	*unknown*
     568:	df 05 02 42 	l.sh -15806(r5),r0
     56c:	00 00 00 2c 	l.j 61c <_or1k_reset+0x51c>
     570:	10 16 00 00 	l.bf 580570 <_end+0x57bb04>
     574:	01 62 05 02 	l.j 588197c <_end+0x587cf10>
     578:	43 00 00 08 	*unknown*
     57c:	13 14 16 00 	l.bf fc505d7c <_end+0xfc501310>
     580:	00 04 7c 05 	l.j 11f594 <_end+0x11ab28>
     584:	02 45 00 00 	l.j f9140584 <_end+0xf913bb18>
     588:	00 2c 30 16 	l.j b0c5e0 <_end+0xb07b74>
     58c:	00 00 03 e9 	l.j 1530 <_or1k_reset+0x1430>
     590:	05 02 46 00 	l.jal 4091d90 <_end+0x408d324>
     594:	00 05 7b 34 	l.j 15f264 <_end+0x15a7f8>
     598:	16 00 00 03 	*unknown*
     59c:	40 05 02 48 	*unknown*
     5a0:	00 00 00 2c 	l.j 650 <_or1k_reset+0x550>
     5a4:	38 16 00 00 	*unknown*
     5a8:	03 f9 05 02 	l.j ffe419b0 <_end+0xffe3cf44>
     5ac:	4a 00 00 08 	*unknown*
     5b0:	2e 3c 16 00 	*unknown*
     5b4:	00 03 09 05 	l.j c29c8 <_end+0xbdf5c>
     5b8:	02 4d 00 00 	l.j f93405b8 <_end+0xf933bb4c>
     5bc:	01 65 40 16 	l.j 5950614 <_end+0x594bba8>
     5c0:	00 00 02 75 	l.j f94 <_or1k_reset+0xe94>
     5c4:	05 02 4e 00 	l.jal 4093dc4 <_end+0x408f358>
     5c8:	00 00 2c 44 	l.j b6d8 <_end+0x6c6c>
     5cc:	16 00 00 05 	*unknown*
     5d0:	14 05 02 4f 	*unknown*
     5d4:	00 00 01 65 	l.j b68 <_or1k_reset+0xa68>
     5d8:	48 16 00 00 	*unknown*
     5dc:	03 63 05 02 	l.j fd8c19e4 <_end+0xfd8bcf78>
     5e0:	50 00 00 08 	*unknown*
     5e4:	34 4c 16 00 	*unknown*
     5e8:	00 02 93 05 	l.j a51fc <_end+0xa0790>
     5ec:	02 53 00 00 	l.j f94c05ec <_end+0xf94bbb80>
     5f0:	00 2c 50 16 	l.j b14648 <_end+0xb0fbdc>
     5f4:	00 00 02 05 	l.j e08 <_or1k_reset+0xd08>
     5f8:	05 02 54 00 	l.jal 40955f8 <_end+0x4090b8c>
     5fc:	00 05 4a 54 	l.j 152f4c <_end+0x14e4e0>
     600:	16 00 00 03 	*unknown*
     604:	7f 05 02 77 	*unknown*
     608:	00 00 07 f1 	l.j 25cc <__call_exitprocs+0x30>
     60c:	58 17 00 00 	*unknown*
     610:	01 3d 05 02 	l.j 4f41a18 <_end+0x4f3cfac>
     614:	7b 00 00 02 	*unknown*
     618:	82 01 48 17 	*unknown*
     61c:	00 00 02 e7 	l.j 11b8 <_or1k_reset+0x10b8>
     620:	05 02 7c 00 	l.jal 409f620 <_end+0x409abb4>
     624:	00 02 44 01 	l.j 91628 <_end+0x8cbbc>
     628:	4c 17 00 00 	l.maci r23,0
     62c:	04 c8 05 02 	l.jal 3201a34 <_end+0x31fcfc8>
     630:	80 00 00 08 	*unknown*
     634:	45 02 dc 17 	*unknown*
     638:	00 00 01 e7 	l.j dd4 <_or1k_reset+0xcd4>
     63c:	05 02 85 00 	l.jal 40a1a3c <_end+0x409cfd0>
     640:	00 05 f6 02 	l.j 17de48 <_end+0x1793dc>
     644:	e0 17 00 00 	l.add r0,r23,r0
     648:	01 cc 05 02 	l.j 7301a50 <_end+0x72fcfe4>
     64c:	86 00 00 08 	l.lwz r16,8(r0)
     650:	51 02 ec 00 	*unknown*
     654:	0f 04 00 00 	l.bnf fc100654 <_end+0xfc0fbbe8>
     658:	05 50 02 01 	l.jal 5400e5c <_end+0x53fc3f0>
     65c:	06 00 00 00 	l.jal f800065c <_end+0xf7ffbbf0>
     660:	82 0f 04 00 	*unknown*
     664:	00 03 f4 13 	l.j fd6b0 <_end+0xf8c44>
     668:	00 00 00 2c 	l.j 718 <_or1k_reset+0x618>
     66c:	00 00 05 7b 	l.j 1c58 <_or1k_reset+0x1b58>
     670:	14 00 00 04 	*unknown*
     674:	12 14 00 00 	l.bf f8500674 <_end+0xf84fbc08>
     678:	01 05 14 00 	l.j 4145678 <_end+0x4140c0c>
     67c:	00 05 7b 14 	l.j 15f2cc <_end+0x15a860>
     680:	00 00 00 2c 	l.j 730 <_or1k_reset+0x630>
     684:	00 0f 04 00 	l.j 3c1684 <_end+0x3bcc18>
     688:	00 05 81 18 	l.j 160ae8 <_end+0x15c07c>
     68c:	00 00 05 50 	l.j 1bcc <_or1k_reset+0x1acc>
     690:	0f 04 00 00 	l.bnf fc100690 <_end+0xfc0fbc24>
     694:	05 5d 13 00 	l.jal 5745294 <_end+0x5740828>
     698:	00 00 7a 00 	l.j 1ee98 <_end+0x1a42c>
     69c:	00 05 aa 14 	l.j 16aeec <_end+0x166480>
     6a0:	00 00 04 12 	l.j 16e8 <_or1k_reset+0x15e8>
     6a4:	14 00 00 01 	*unknown*
     6a8:	05 14 00 00 	l.jal 45006a8 <_end+0x44fbc3c>
     6ac:	00 7a 14 00 	l.j 1e856ac <_end+0x1e80c40>
     6b0:	00 00 2c 00 	l.j b6b0 <_end+0x6c44>
     6b4:	0f 04 00 00 	l.bnf fc1006b4 <_end+0xfc0fbc48>
     6b8:	05 8c 13 00 	l.jal 63052b8 <_end+0x630084c>
     6bc:	00 00 2c 00 	l.j b6bc <_end+0x6c50>
     6c0:	00 05 c4 14 	l.j 171710 <_end+0x16cca4>
     6c4:	00 00 04 12 	l.j 170c <_or1k_reset+0x160c>
     6c8:	14 00 00 01 	*unknown*
     6cc:	05 00 0f 04 	l.jal 40042dc <_end+0x3fff870>
     6d0:	00 00 05 b0 	l.j 1d90 <_or1k_reset+0x1c90>
     6d4:	08 00 00 00 	*unknown*
     6d8:	41 00 00 05 	*unknown*
     6dc:	da 09 00 00 	l.sb -32768(r9),r0
     6e0:	00 c7 02 00 	l.j 31c0ee0 <_end+0x31bc474>
     6e4:	08 00 00 00 	*unknown*
     6e8:	41 00 00 05 	*unknown*
     6ec:	ea 09 00 00 	*unknown*
     6f0:	00 c7 00 00 	l.j 31c06f0 <_end+0x31bbc84>
     6f4:	05 00 00 03 	l.jal 4000700 <_end+0x3ffbc94>
     6f8:	a4 05 01 1d 	l.andi r0,r5,0x11d
     6fc:	00 00 02 ca 	l.j 1224 <_or1k_reset+0x1124>
     700:	19 00 00 04 	l.movhi r8,0x4
     704:	af 0c 05 01 	l.xori r24,r12,1281
     708:	21 00 00 06 	l.trap 0x6
     70c:	2b 16 00 00 	*unknown*
     710:	03 cc 05 01 	l.j ff301b14 <_end+0xff2fd0a8>
     714:	23 00 00 06 	*unknown*
     718:	2b 00 16 00 	*unknown*
     71c:	00 02 a1 05 	l.j a8b30 <_end+0xa40c4>
     720:	01 24 00 00 	l.j 4900720 <_end+0x48fbcb4>
     724:	00 2c 04 16 	l.j b0177c <_end+0xafcd10>
     728:	00 00 03 9e 	l.j 15a0 <_or1k_reset+0x14a0>
     72c:	05 01 25 00 	l.jal 4049b2c <_end+0x40450c0>
     730:	00 06 31 08 	l.j 18cb50 <_end+0x1880e4>
     734:	00 0f 04 00 	l.j 3c1734 <_end+0x3bccc8>
     738:	00 05 f6 0f 	l.j 17df74 <_end+0x179508>
     73c:	04 00 00 05 	l.jal 750 <_or1k_reset+0x650>
     740:	ea 19 00 00 	*unknown*
     744:	01 5a 0e 05 	l.j 5683f58 <_end+0x567f4ec>
     748:	01 3d 00 00 	l.j 4f40748 <_end+0x4f3bcdc>
     74c:	06 6c 16 00 	l.jal f9b05f4c <_end+0xf9b014e0>
     750:	00 04 0b 05 	l.j 103364 <_end+0xfe8f8>
     754:	01 3e 00 00 	l.j 4f80754 <_end+0x4f7bce8>
     758:	06 6c 00 16 	l.jal f9b007b0 <_end+0xf9afbd44>
     75c:	00 00 04 38 	l.j 183c <_or1k_reset+0x173c>
     760:	05 01 3f 00 	l.jal 4050360 <_end+0x404b8f4>
     764:	00 06 6c 06 	l.j 19b77c <_end+0x196d10>
     768:	16 00 00 0e 	*unknown*
     76c:	6d 05 01 40 	l.lwa r8,320(r5)
     770:	00 00 00 4f 	l.j 8ac <_or1k_reset+0x7ac>
     774:	0c 00 08 00 	l.bnf 2774 <__call_exitprocs+0x1d8>
     778:	00 00 4f 00 	l.j 14378 <_end+0xf90c>
     77c:	00 06 7c 09 	l.j 19f7a0 <_end+0x19ad34>
     780:	00 00 00 c7 	l.j a9c <_or1k_reset+0x99c>
     784:	02 00 1a cc 	l.j f80072b4 <_end+0xf8002848>
     788:	05 02 58 00 	l.jal 4096788 <_end+0x4091d1c>
     78c:	00 07 7d 16 	l.j 1dfbe4 <_end+0x1db178>
     790:	00 00 04 93 	l.j 19dc <_or1k_reset+0x18dc>
     794:	05 02 5a 00 	l.jal 4096f94 <_end+0x4092528>
     798:	00 00 91 00 	l.j 24b98 <_end+0x2012c>
     79c:	16 00 00 04 	*unknown*
     7a0:	3e 05 02 5b 	*unknown*
     7a4:	00 00 05 4a 	l.j 1ccc <_or1k_reset+0x1bcc>
     7a8:	04 16 00 00 	l.jal 5807a8 <_end+0x57bd3c>
     7ac:	02 fc 05 02 	l.j fbf01bb4 <_end+0xfbefd148>
     7b0:	5c 00 00 07 	*unknown*
     7b4:	7d 08 16 00 	*unknown*
     7b8:	00 04 eb 05 	l.j 13b3cc <_end+0x136960>
     7bc:	02 5d 00 00 	l.j f97407bc <_end+0xf973bd50>
     7c0:	01 7b 24 16 	l.j 5ec9818 <_end+0x5ec4dac>
     7c4:	00 00 02 60 	l.j 1144 <_or1k_reset+0x1044>
     7c8:	05 02 5e 00 	l.jal 4097fc8 <_end+0x409355c>
     7cc:	00 00 2c 48 	l.j b8ec <_end+0x6e80>
     7d0:	16 00 00 03 	*unknown*
     7d4:	c7 05 02 5f 	*unknown*
     7d8:	00 00 00 5d 	l.j 94c <_or1k_reset+0x84c>
     7dc:	4c 16 00 00 	l.maci r22,0
     7e0:	05 01 05 02 	l.jal 4041be8 <_end+0x403d17c>
     7e4:	60 00 00 06 	*unknown*
     7e8:	37 54 16 00 	*unknown*
     7ec:	00 03 d2 05 	l.j f5000 <_end+0xf0594>
     7f0:	02 61 00 00 	l.j f98407f0 <_end+0xf983bd84>
     7f4:	00 ef 64 16 	l.j 3bd984c <_end+0x3bd4de0>
     7f8:	00 00 05 06 	l.j 1c10 <_or1k_reset+0x1b10>
     7fc:	05 02 62 00 	l.jal 4098ffc <_end+0x4094590>
     800:	00 00 ef 6c 	l.j 3c5b0 <_end+0x37b44>
     804:	16 00 00 01 	*unknown*
     808:	a5 05 02 63 	l.andi r8,r5,0x263
     80c:	00 00 00 ef 	l.j bc8 <_or1k_reset+0xac8>
     810:	74 16 00 00 	*unknown*
     814:	04 be 05 02 	l.jal 2f81c1c <_end+0x2f7d1b0>
     818:	64 00 00 07 	*unknown*
     81c:	8d 7c 16 00 	l.lbz r11,5632(r28)
     820:	00 02 f0 05 	l.j bc834 <_end+0xb7dc8>
     824:	02 65 00 00 	l.j f9940824 <_end+0xf993bdb8>
     828:	07 9d 84 16 	l.jal fe761880 <_end+0xfe75ce14>
     82c:	00 00 04 5c 	l.j 199c <_or1k_reset+0x189c>
     830:	05 02 66 00 	l.jal 409a030 <_end+0x40955c4>
     834:	00 00 2c 9c 	l.j baa4 <_end+0x7038>
     838:	16 00 00 02 	*unknown*
     83c:	26 05 02 67 	*unknown*
     840:	00 00 00 ef 	l.j bfc <_or1k_reset+0xafc>
     844:	a0 16 00 00 	l.addic r0,r22,0
     848:	01 96 05 02 	l.j 6581c50 <_end+0x657d1e4>
     84c:	68 00 00 00 	*unknown*
     850:	ef a8 16 00 	*unknown*
     854:	00 02 15 05 	l.j 85c68 <_end+0x811fc>
     858:	02 69 00 00 	l.j f9a40858 <_end+0xf9a3bdec>
     85c:	00 ef b0 16 	l.j 3bec8b4 <_end+0x3be7e48>
     860:	00 00 01 6d 	l.j e14 <_or1k_reset+0xd14>
     864:	05 02 6a 00 	l.jal 409b064 <_end+0x40965f8>
     868:	00 00 ef b8 	l.j 3c748 <_end+0x37cdc>
     86c:	16 00 00 01 	*unknown*
     870:	7c 05 02 6b 	*unknown*
     874:	00 00 00 ef 	l.j c30 <_or1k_reset+0xb30>
     878:	c0 16 00 00 	l.mtspr r22,r0,0x0
     87c:	03 84 05 02 	l.j fe101c84 <_end+0xfe0fd218>
     880:	6c 00 00 00 	l.lwa r0,0(r0)
     884:	2c c8 00 08 	*unknown*
     888:	00 00 05 50 	l.j 1dc8 <_or1k_reset+0x1cc8>
     88c:	00 00 07 8d 	l.j 26c0 <__call_exitprocs+0x124>
     890:	09 00 00 00 	*unknown*
     894:	c7 19 00 08 	*unknown*
     898:	00 00 05 50 	l.j 1dd8 <_or1k_reset+0x1cd8>
     89c:	00 00 07 9d 	l.j 2710 <__call_exitprocs+0x174>
     8a0:	09 00 00 00 	*unknown*
     8a4:	c7 07 00 08 	*unknown*
     8a8:	00 00 05 50 	l.j 1de8 <_or1k_reset+0x1ce8>
     8ac:	00 00 07 ad 	l.j 2760 <__call_exitprocs+0x1c4>
     8b0:	09 00 00 00 	*unknown*
     8b4:	c7 17 00 1a 	*unknown*
     8b8:	f0 05 02 71 	*unknown*
     8bc:	00 00 07 d1 	l.j 2800 <_write_r+0x7c>
     8c0:	16 00 00 03 	*unknown*
     8c4:	30 05 02 74 	*unknown*
     8c8:	00 00 07 d1 	l.j 280c <_write_r+0x88>
     8cc:	00 16 00 00 	l.j 5808cc <_end+0x57be60>
     8d0:	04 b5 05 02 	l.jal 2d41cd8 <_end+0x2d3d26c>
     8d4:	75 00 00 07 	*unknown*
     8d8:	e1 78 00 08 	l.sll r11,r24,r0
     8dc:	00 00 02 c4 	l.j 13ec <_or1k_reset+0x12ec>
     8e0:	00 00 07 e1 	l.j 2864 <_execve_r+0x10>
     8e4:	09 00 00 00 	*unknown*
     8e8:	c7 1d 00 08 	*unknown*
     8ec:	00 00 00 91 	l.j b30 <_or1k_reset+0xa30>
     8f0:	00 00 07 f1 	l.j 28b4 <_fstat_r+0x14>
     8f4:	09 00 00 00 	*unknown*
     8f8:	c7 1d 00 1b 	*unknown*
     8fc:	f0 05 02 56 	*unknown*
     900:	00 00 08 13 	l.j 294c <_link_r+0xc>
     904:	1c 00 00 0c 	*unknown*
     908:	0b 05 02 6d 	*unknown*
     90c:	00 00 06 7c 	l.j 22fc <frame_dummy>
     910:	1c 00 00 04 	*unknown*
     914:	d8 05 02 76 	l.sb 630(r5),r0
     918:	00 00 07 ad 	l.j 27cc <_write_r+0x48>
     91c:	00 08 00 00 	l.j 20091c <_end+0x1fbeb0>
     920:	05 50 00 00 	l.jal 5400920 <_end+0x53fbeb4>
     924:	08 23 09 00 	*unknown*
     928:	00 00 c7 18 	l.j 32588 <_end+0x2db1c>
     92c:	00 1d 00 00 	l.j 74092c <_end+0x73bec0>
     930:	08 2e 14 00 	*unknown*
     934:	00 04 12 00 	l.j 105134 <_end+0x1006c8>
     938:	0f 04 00 00 	l.bnf fc100938 <_end+0xfc0fbecc>
     93c:	08 23 0f 04 	*unknown*
     940:	00 00 01 65 	l.j ed4 <_or1k_reset+0xdd4>
     944:	1d 00 00 08 	*unknown*
     948:	45 14 00 00 	*unknown*
     94c:	00 2c 00 0f 	l.j b00988 <_end+0xafbf1c>
     950:	04 00 00 08 	l.jal 970 <_or1k_reset+0x870>
     954:	4b 0f 04 00 	*unknown*
     958:	00 08 3a 08 	l.j 20f178 <_end+0x20a70c>
     95c:	00 00 05 ea 	l.j 2104 <_or1k_start+0xd4>
     960:	00 00 08 61 	l.j 2ae4 <_or1k_uart_init+0x90>
     964:	09 00 00 00 	*unknown*
     968:	c7 02 00 1e 	*unknown*
     96c:	00 00 06 92 	l.j 23b4 <main+0x1c>
     970:	06 55 00 00 	l.jal f9540970 <_end+0xf953bf04>
     974:	24 14 00 00 	*unknown*
     978:	00 48 01 9c 	l.j 1200fe8 <_end+0x11fc57c>
     97c:	00 00 08 af 	l.j 2c38 <_or1k_cache_init+0x30>
     980:	1f 00 00 04 	*unknown*
     984:	8e 01 3a 00 	l.lbz r16,14848(r1)
     988:	00 00 2c 00 	l.j b988 <_end+0x6f1c>
     98c:	00 00 2c 20 	l.j ba0c <_end+0x6fa0>
     990:	00 00 24 30 	l.j 9a50 <_end+0x4fe4>
     994:	00 00 08 c0 	l.j 2c94 <_or1k_cache_init+0x8c>
     998:	00 00 08 9e 	l.j 2c10 <_or1k_cache_init+0x8>
     99c:	21 01 54 01 	*unknown*
     9a0:	30 21 01 53 	*unknown*
     9a4:	02 72 00 00 	l.j f9c809a4 <_end+0xf9c7bf38>
     9a8:	22 00 00 24 	*unknown*
     9ac:	5c 00 00 08 	*unknown*
     9b0:	d6 21 01 53 	l.sw -30381(r1),r0
     9b4:	02 72 00 00 	l.j f9c809b4 <_end+0xf9c7bf48>
     9b8:	00 23 00 00 	l.j 8c09b8 <_end+0x8bbf4c>
     9bc:	04 69 05 02 	l.jal 1a41dc4 <_end+0x1a3d358>
     9c0:	fb 00 00 08 	*unknown*
     9c4:	bb 18 00 00 	l.slli r24,r24,0x0
     9c8:	04 12 24 00 	l.jal 4899c8 <_end+0x484f5c>
     9cc:	00 03 52 07 	l.j d51e8 <_end+0xd077c>
     9d0:	0c 00 00 08 	l.bnf 9f0 <_or1k_reset+0x8f0>
     9d4:	d6 14 00 00 	l.sw -32768(r20),r0
     9d8:	00 2c 14 00 	l.j b059d8 <_end+0xb00f6c>
     9dc:	00 01 05 00 	l.j 41ddc <_end+0x3d370>
     9e0:	25 00 00 06 	*unknown*
     9e4:	91 08 12 14 	l.lbs r8,4628(r8)
     9e8:	00 00 00 2c 	l.j a98 <_or1k_reset+0x998>
     9ec:	00 00 00 00 	l.j 9ec <_or1k_reset+0x8ec>
     9f0:	08 90 00 04 	*unknown*
     9f4:	00 00 02 9b 	l.j 1460 <_or1k_reset+0x1360>
     9f8:	04 01 00 00 	l.jal 409f8 <_end+0x3bf8c>
     9fc:	00 30 01 00 	l.j c00dfc <_end+0xbfc390>
     a00:	00 05 23 00 	l.j 149600 <_end+0x144b94>
     a04:	00 05 63 00 	l.j 159604 <_end+0x154b98>
     a08:	00 02 5e 02 	l.j 98210 <_end+0x937a4>
     a0c:	04 05 00 00 	l.jal 140a0c <_end+0x13bfa0>
     a10:	00 05 02 04 	l.j 141220 <_end+0x13c7b4>
     a14:	07 00 00 00 	l.jal fc000a14 <_end+0xfbffbfa8>
     a18:	5e 03 04 05 	*unknown*
     a1c:	69 6e 74 00 	*unknown*
     a20:	02 01 06 00 	l.j f8042220 <_end+0xf803d7b4>
     a24:	00 00 7b 02 	l.j 1f62c <_end+0x1abc0>
     a28:	01 08 00 00 	l.j 4200a28 <_end+0x41fbfbc>
     a2c:	00 79 02 02 	l.j 1e41234 <_end+0x1e3c7c8>
     a30:	05 00 00 01 	l.jal 4000a34 <_end+0x3ffbfc8>
     a34:	2f 02 02 07 	*unknown*
     a38:	00 00 00 0e 	l.j a70 <_or1k_reset+0x970>
     a3c:	02 08 05 00 	l.j f8201e3c <_end+0xf81fd3d0>
     a40:	00 00 00 02 	l.j a48 <_or1k_reset+0x948>
     a44:	08 07 00 00 	*unknown*
     a48:	00 59 04 00 	l.j 1641a48 <_end+0x163cfdc>
     a4c:	00 03 6d 01 	l.j dbe50 <_end+0xd73e4>
     a50:	07 00 00 00 	l.jal fc000a50 <_end+0xfbffbfe4>
     a54:	2b 04 00 00 	*unknown*
     a58:	03 4b 02 10 	l.j fd2c1298 <_end+0xfd2bc82c>
     a5c:	00 00 00 1d 	l.j ad0 <_or1k_reset+0x9d0>
     a60:	04 00 00 04 	l.jal a70 <_or1k_reset+0x970>
     a64:	27 02 27 00 	*unknown*
     a68:	00 00 1d 05 	l.j 7e7c <_end+0x3410>
     a6c:	00 00 03 17 	l.j 16c8 <_or1k_reset+0x15c8>
     a70:	03 01 61 00 	l.j fc058e70 <_end+0xfc054404>
     a74:	00 00 89 02 	l.j 22e7c <_end+0x1e410>
     a78:	04 07 00 00 	l.jal 1c0a78 <_end+0x1bc00c>
     a7c:	00 63 06 04 	l.j 18c228c <_end+0x18bd820>
     a80:	02 4a 00 00 	l.j f9280a80 <_end+0xf927c014>
     a84:	00 af 07 00 	l.j 2bc2684 <_end+0x2bbdc18>
     a88:	00 03 11 02 	l.j c4e90 <_end+0xc0424>
     a8c:	4c 00 00 00 	l.maci r0,0
     a90:	7d 07 00 00 	*unknown*
     a94:	02 84 02 4d 	l.j fa1013c8 <_end+0xfa0fc95c>
     a98:	00 00 00 af 	l.j d54 <_or1k_reset+0xc54>
     a9c:	00 08 00 00 	l.j 200a9c <_end+0x1fc030>
     aa0:	00 39 00 00 	l.j e40aa0 <_end+0xe3c034>
     aa4:	00 bf 09 00 	l.j 2fc2ea4 <_end+0x2fbe438>
     aa8:	00 00 bf 03 	l.j 306b4 <_end+0x2bc48>
     aac:	00 02 04 07 	l.j 81ac8 <_end+0x7d05c>
     ab0:	00 00 01 45 	l.j fc4 <_or1k_reset+0xec4>
     ab4:	0a 08 02 47 	*unknown*
     ab8:	00 00 00 e7 	l.j e54 <_or1k_reset+0xd54>
     abc:	0b 00 00 04 	*unknown*
     ac0:	11 02 49 00 	l.bf 4092ec0 <_end+0x408e454>
     ac4:	00 00 2b 00 	l.j b6c4 <_end+0x6c58>
     ac8:	0b 00 00 04 	*unknown*
     acc:	19 02 4e 00 	*unknown*
     ad0:	00 00 90 04 	l.j 24ae0 <_end+0x20074>
     ad4:	00 04 00 00 	l.j 100ad4 <_end+0xfc068>
     ad8:	03 ab 02 4f 	l.j feac1414 <_end+0xfeabc9a8>
     adc:	00 00 00 c6 	l.j df4 <_or1k_reset+0xcf4>
     ae0:	04 00 00 02 	l.jal ae8 <_or1k_reset+0x9e8>
     ae4:	47 02 53 00 	*unknown*
     ae8:	00 00 5c 0c 	l.j 17b18 <_end+0x130ac>
     aec:	04 04 00 00 	l.jal 100aec <_end+0xfc080>
     af0:	04 54 04 16 	l.jal 1501b48 <_end+0x14fd0dc>
     af4:	00 00 00 24 	l.j b84 <_or1k_reset+0xa84>
     af8:	0d 00 00 02 	l.bnf 4000b00 <_end+0x3ffc094>
     afc:	58 18 04 2d 	*unknown*
     b00:	00 00 01 5d 	l.j 1074 <_or1k_reset+0xf74>
     b04:	0b 00 00 03 	*unknown*
     b08:	cc 04 2f 00 	l.swa 1792(r4),r5
     b0c:	00 01 5d 00 	l.j 57f0c <_end+0x534a0>
     b10:	0e 5f 6b 00 	l.bnf f97db710 <_end+0xf97d6ca4>
     b14:	04 30 00 00 	l.jal c00b14 <_end+0xbfc0a8>
     b18:	00 2b 04 0b 	l.j ac1b44 <_end+0xabd0d8>
     b1c:	00 00 04 03 	l.j 1b28 <_or1k_reset+0x1a28>
     b20:	04 30 00 00 	l.jal c00b20 <_end+0xbfc0b4>
     b24:	00 2b 08 0b 	l.j ac2b50 <_end+0xabe0e4>
     b28:	00 00 02 41 	l.j 142c <_or1k_reset+0x132c>
     b2c:	04 30 00 00 	l.jal c00b2c <_end+0xbfc0c0>
     b30:	00 2b 0c 0b 	l.j ac3b5c <_end+0xabf0f0>
     b34:	00 00 04 a0 	l.j 1db4 <_or1k_reset+0x1cb4>
     b38:	04 30 00 00 	l.jal c00b38 <_end+0xbfc0cc>
     b3c:	00 2b 10 0e 	l.j ac4b74 <_end+0xac0108>
     b40:	5f 78 00 04 	*unknown*
     b44:	31 00 00 01 	*unknown*
     b48:	63 14 00 0f 	*unknown*
     b4c:	04 00 00 01 	l.jal b50 <_or1k_reset+0xa50>
     b50:	0a 08 00 00 	*unknown*
     b54:	00 ff 00 00 	l.j 3fc0b54 <_end+0x3fbc0e8>
     b58:	01 73 09 00 	l.j 5cc2f58 <_end+0x5cbe4ec>
     b5c:	00 00 bf 00 	l.j 3075c <_end+0x2bcf0>
     b60:	00 0d 00 00 	l.j 340b60 <_end+0x33c0f4>
     b64:	02 7f 24 04 	l.j f9fc9b74 <_end+0xf9fc5108>
     b68:	35 00 00 01 	*unknown*
     b6c:	ec 0b 00 00 	*unknown*
     b70:	01 b3 04 37 	l.j 6cc1c4c <_end+0x6cbd1e0>
     b74:	00 00 00 2b 	l.j c20 <_or1k_reset+0xb20>
     b78:	00 0b 00 00 	l.j 2c0b78 <_end+0x2bc10c>
     b7c:	04 2f 04 38 	l.jal bc1c5c <_end+0xbbd1f0>
     b80:	00 00 00 2b 	l.j c2c <_or1k_reset+0xb2c>
     b84:	04 0b 00 00 	l.jal 2c0b84 <_end+0x2bc118>
     b88:	01 c2 04 39 	l.j 7081c6c <_end+0x707d200>
     b8c:	00 00 00 2b 	l.j c38 <_or1k_reset+0xb38>
     b90:	08 0b 00 00 	*unknown*
     b94:	05 19 04 3a 	l.jal 4641c7c <_end+0x463d210>
     b98:	00 00 00 2b 	l.j c44 <_or1k_reset+0xb44>
     b9c:	0c 0b 00 00 	l.bnf 2c0b9c <_end+0x2bc130>
     ba0:	03 37 04 3b 	l.j fcdc1c8c <_end+0xfcdbd220>
     ba4:	00 00 00 2b 	l.j c50 <_or1k_reset+0xb50>
     ba8:	10 0b 00 00 	l.bf 2c0ba8 <_end+0x2bc13c>
     bac:	03 26 04 3c 	l.j fc981c9c <_end+0xfc97d230>
     bb0:	00 00 00 2b 	l.j c5c <_or1k_reset+0xb5c>
     bb4:	14 0b 00 00 	*unknown*
     bb8:	04 a5 04 3d 	l.jal 2941cac <_end+0x293d240>
     bbc:	00 00 00 2b 	l.j c68 <_or1k_reset+0xb68>
     bc0:	18 0b 00 00 	*unknown*
     bc4:	03 8d 04 3e 	l.j fe341cbc <_end+0xfe33d250>
     bc8:	00 00 00 2b 	l.j c74 <_or1k_reset+0xb74>
     bcc:	1c 0b 00 00 	*unknown*
     bd0:	04 e0 04 3f 	l.jal 3801ccc <_end+0x37fd260>
     bd4:	00 00 00 2b 	l.j c80 <_or1k_reset+0xb80>
     bd8:	20 00 10 00 	l.sys 0x1000
     bdc:	00 01 d1 01 	l.j 74fe0 <_end+0x70574>
     be0:	08 04 48 00 	*unknown*
     be4:	00 02 2c 0b 	l.j 8bc10 <_end+0x871a4>
     be8:	00 00 02 34 	l.j 14b8 <_or1k_reset+0x13b8>
     bec:	04 49 00 00 	l.jal 1240bec <_end+0x123c180>
     bf0:	02 2c 00 0b 	l.j f8b00c1c <_end+0xf8afc1b0>
     bf4:	00 00 01 4e 	l.j 112c <_or1k_reset+0x102c>
     bf8:	04 4a 00 00 	l.jal 1280bf8 <_end+0x127c18c>
     bfc:	02 2c 80 11 	l.j f8b20c40 <_end+0xf8b1c1d4>
     c00:	00 00 04 4b 	l.j 1d2c <_or1k_reset+0x1c2c>
     c04:	04 4c 00 00 	l.jal 1300c04 <_end+0x12fc198>
     c08:	00 ff 01 00 	l.j 3fc1008 <_end+0x3fbc59c>
     c0c:	11 00 00 01 	l.bf 4000c10 <_end+0x3ffc1a4>
     c10:	f6 04 4f 00 	*unknown*
     c14:	00 00 ff 01 	l.j 40818 <_end+0x3bdac>
     c18:	04 00 08 00 	l.jal 2c18 <_or1k_cache_init+0x10>
     c1c:	00 00 fd 00 	l.j 4001c <_end+0x3b5b0>
     c20:	00 02 3c 09 	l.j 8fc44 <_end+0x8b1d8>
     c24:	00 00 00 bf 	l.j f20 <_or1k_reset+0xe20>
     c28:	1f 00 10 00 	*unknown*
     c2c:	00 01 3d 01 	l.j 50030 <_end+0x4b5c4>
     c30:	90 04 5b 00 	l.lbs r0,23296(r4)
     c34:	00 02 7a 0b 	l.j 9f460 <_end+0x9a9f4>
     c38:	00 00 03 cc 	l.j 1b68 <_or1k_reset+0x1a68>
     c3c:	04 5c 00 00 	l.jal 1700c3c <_end+0x16fc1d0>
     c40:	02 7a 00 0b 	l.j f9e80c6c <_end+0xf9e7c200>
     c44:	00 00 03 e4 	l.j 1bd4 <_or1k_reset+0x1ad4>
     c48:	04 5d 00 00 	l.jal 1740c48 <_end+0x173c1dc>
     c4c:	00 2b 04 0b 	l.j ac1c78 <_end+0xabd20c>
     c50:	00 00 02 3c 	l.j 1540 <_or1k_reset+0x1440>
     c54:	04 5f 00 00 	l.jal 17c0c54 <_end+0x17bc1e8>
     c58:	02 80 08 0b 	l.j fa002c84 <_end+0xf9ffe218>
     c5c:	00 00 01 d1 	l.j 13a0 <_or1k_reset+0x12a0>
     c60:	04 60 00 00 	l.jal 1800c60 <_end+0x17fc1f4>
     c64:	01 ec 88 00 	l.j 7b22c64 <_end+0x7b1e1f8>
     c68:	0f 04 00 00 	l.bnf fc100c68 <_end+0xfc0fc1fc>
     c6c:	02 3c 08 00 	l.j f8f02c6c <_end+0xf8efe200>
     c70:	00 02 90 00 	l.j a4c70 <_end+0xa0204>
     c74:	00 02 90 09 	l.j a4c98 <_end+0xa022c>
     c78:	00 00 00 bf 	l.j f74 <_or1k_reset+0xe74>
     c7c:	1f 00 0f 04 	*unknown*
     c80:	00 00 02 96 	l.j 16d8 <_or1k_reset+0x15d8>
     c84:	12 0d 00 00 	l.bf f8340c84 <_end+0xf833c218>
     c88:	03 97 08 04 	l.j fe5c2c98 <_end+0xfe5be22c>
     c8c:	73 00 00 02 	*unknown*
     c90:	bc 0b 00 00 	l.sfeqi r11,0
     c94:	09 f8 04 74 	*unknown*
     c98:	00 00 02 bc 	l.j 1788 <_or1k_reset+0x1688>
     c9c:	00 0b 00 00 	l.j 2c0c9c <_end+0x2bc230>
     ca0:	07 8b 04 75 	l.jal fe2c1e74 <_end+0xfe2bd408>
     ca4:	00 00 00 2b 	l.j d50 <_or1k_reset+0xc50>
     ca8:	04 00 0f 04 	l.jal 48b8 <object.2876+0xc>
     cac:	00 00 00 39 	l.j d90 <_or1k_reset+0xc90>
     cb0:	0d 00 00 03 	l.bnf 4000cbc <_end+0x3ffc250>
     cb4:	b6 68 04 b3 	l.mfspr r19,r8,0x4b3
     cb8:	00 00 03 ec 	l.j 1c68 <_or1k_reset+0x1b68>
     cbc:	0e 5f 70 00 	l.bnf f97dccbc <_end+0xf97d8250>
     cc0:	04 b4 00 00 	l.jal 2d00cc0 <_end+0x2cfc254>
     cc4:	02 bc 00 0e 	l.j faf00cfc <_end+0xfaefc290>
     cc8:	5f 72 00 04 	*unknown*
     ccc:	b5 00 00 00 	l.mfspr r8,r0,0x0
     cd0:	2b 04 0e 5f 	*unknown*
     cd4:	77 00 04 b6 	*unknown*
     cd8:	00 00 00 2b 	l.j d84 <_or1k_reset+0xc84>
     cdc:	08 0b 00 00 	*unknown*
     ce0:	01 ef 04 b7 	l.j 7bc1fbc <_end+0x7bbd550>
     ce4:	00 00 00 40 	l.j de4 <_or1k_reset+0xce4>
     ce8:	0c 0b 00 00 	l.bnf 2c0ce8 <_end+0x2bc27c>
     cec:	02 9b 04 b8 	l.j fa6c1fcc <_end+0xfa6bd560>
     cf0:	00 00 00 40 	l.j df0 <_or1k_reset+0xcf0>
     cf4:	0e 0e 5f 62 	l.bnf f8398a7c <_end+0xf8394010>
     cf8:	66 00 04 b9 	*unknown*
     cfc:	00 00 02 97 	l.j 1758 <_or1k_reset+0x1658>
     d00:	10 0b 00 00 	l.bf 2c0d00 <_end+0x2bc294>
     d04:	01 8d 04 ba 	l.j 6341fec <_end+0x633d580>
     d08:	00 00 00 2b 	l.j db4 <_or1k_reset+0xcb4>
     d0c:	18 0b 00 00 	*unknown*
     d10:	01 df 04 c1 	l.j 77c2014 <_end+0x77bd5a8>
     d14:	00 00 00 fd 	l.j 1108 <_or1k_reset+0x1008>
     d18:	1c 0b 00 00 	*unknown*
     d1c:	02 6f 04 c3 	l.j f9bc2028 <_end+0xf9bbd5bc>
     d20:	00 00 05 4f 	l.j 225c <__do_global_dtors_aux+0x54>
     d24:	20 0b 00 00 	*unknown*
     d28:	09 a6 04 c5 	*unknown*
     d2c:	00 00 05 7e 	l.j 2324 <frame_dummy+0x28>
     d30:	24 0b 00 00 	*unknown*
     d34:	04 21 04 c8 	l.jal 842054 <_end+0x83d5e8>
     d38:	00 00 05 a2 	l.j 23c0 <main+0x28>
     d3c:	28 0b 00 00 	*unknown*
     d40:	04 fa 04 c9 	l.jal 3e82064 <_end+0x3e7d5f8>
     d44:	00 00 05 bc 	l.j 2434 <exit+0x20>
     d48:	2c 0e 5f 75 	*unknown*
     d4c:	62 00 04 cc 	*unknown*
     d50:	00 00 02 97 	l.j 17ac <_or1k_reset+0x16ac>
     d54:	30 0e 5f 75 	*unknown*
     d58:	70 00 04 cd 	*unknown*
     d5c:	00 00 02 bc 	l.j 184c <_or1k_reset+0x174c>
     d60:	38 0e 5f 75 	*unknown*
     d64:	72 00 04 ce 	*unknown*
     d68:	00 00 00 2b 	l.j e14 <_or1k_reset+0xd14>
     d6c:	3c 0b 00 00 	*unknown*
     d70:	01 bc 04 d1 	l.j 6f020b4 <_end+0x6efd648>
     d74:	00 00 05 c2 	l.j 247c <__register_exitproc+0x20>
     d78:	40 0b 00 00 	*unknown*
     d7c:	04 d2 04 d2 	l.jal 34820c4 <_end+0x347d658>
     d80:	00 00 05 d2 	l.j 24c8 <__register_exitproc+0x6c>
     d84:	43 0e 5f 6c 	*unknown*
     d88:	62 00 04 d5 	*unknown*
     d8c:	00 00 02 97 	l.j 17e8 <_or1k_reset+0x16e8>
     d90:	44 0b 00 00 	*unknown*
     d94:	08 18 04 d8 	*unknown*
     d98:	00 00 00 2b 	l.j e44 <_or1k_reset+0xd44>
     d9c:	4c 0b 00 00 	l.maci r11,0
     da0:	02 0d 04 d9 	l.j f8342104 <_end+0xf833d698>
     da4:	00 00 00 67 	l.j f40 <_or1k_reset+0xe40>
     da8:	50 0b 00 00 	*unknown*
     dac:	0c 6a 04 dc 	l.bnf 1a8211c <_end+0x1a7d6b0>
     db0:	00 00 04 0a 	l.j 1dd8 <_or1k_reset+0x1cd8>
     db4:	54 0b 00 00 	*unknown*
     db8:	06 4f 04 e0 	l.jal f93c2138 <_end+0xf93bd6cc>
     dbc:	00 00 00 f2 	l.j 1184 <_or1k_reset+0x1084>
     dc0:	58 0b 00 00 	*unknown*
     dc4:	03 be 04 e2 	l.j fef8214c <_end+0xfef7d6e0>
     dc8:	00 00 00 e7 	l.j 1164 <_or1k_reset+0x1064>
     dcc:	5c 0b 00 00 	*unknown*
     dd0:	03 1e 04 e3 	l.j fc78215c <_end+0xfc77d6f0>
     dd4:	00 00 00 2b 	l.j e80 <_or1k_reset+0xd80>
     dd8:	64 00 13 00 	*unknown*
     ddc:	00 00 2b 00 	l.j b9dc <_end+0x6f70>
     de0:	00 04 0a 14 	l.j 103630 <_end+0xfebc4>
     de4:	00 00 04 0a 	l.j 1e0c <_or1k_reset+0x1d0c>
     de8:	14 00 00 00 	*unknown*
     dec:	fd 14 00 00 	*unknown*
     df0:	05 42 14 00 	l.jal 5085df0 <_end+0x5081384>
     df4:	00 00 2b 00 	l.j b9f4 <_end+0x6f88>
     df8:	0f 04 00 00 	l.bnf fc100df8 <_end+0xfc0fc38c>
     dfc:	04 10 15 00 	l.jal 4061fc <_end+0x401790>
     e00:	00 0c 0b 04 	l.j 303a10 <_end+0x2fefa4>
     e04:	24 04 02 39 	*unknown*
     e08:	00 00 05 42 	l.j 2310 <frame_dummy+0x14>
     e0c:	16 00 00 06 	*unknown*
     e10:	bd 04 02 3b 	*unknown*
     e14:	00 00 00 2b 	l.j ec0 <_or1k_reset+0xdc0>
     e18:	00 16 00 00 	l.j 580e18 <_end+0x57c3ac>
     e1c:	01 fe 04 02 	l.j 7f81e24 <_end+0x7f7d3b8>
     e20:	40 00 00 06 	*unknown*
     e24:	29 04 16 00 	*unknown*
     e28:	00 02 8b 04 	l.j a3a38 <_end+0x9efcc>
     e2c:	02 40 00 00 	l.j f9000e2c <_end+0xf8ffc3c0>
     e30:	06 29 08 16 	l.jal f8a42e88 <_end+0xf8a3e41c>
     e34:	00 00 02 50 	l.j 1774 <_or1k_reset+0x1674>
     e38:	04 02 40 00 	l.jal 90e38 <_end+0x8c3cc>
     e3c:	00 06 29 0c 	l.j 18b26c <_end+0x186800>
     e40:	16 00 00 03 	*unknown*
     e44:	df 04 02 42 	l.sh -15806(r4),r0
     e48:	00 00 00 2b 	l.j ef4 <_or1k_reset+0xdf4>
     e4c:	10 16 00 00 	l.bf 580e4c <_end+0x57c3e0>
     e50:	01 62 04 02 	l.j 5881e58 <_end+0x587d3ec>
     e54:	43 00 00 08 	*unknown*
     e58:	0b 14 16 00 	*unknown*
     e5c:	00 04 7c 04 	l.j 11fe6c <_end+0x11b400>
     e60:	02 45 00 00 	l.j f9140e60 <_end+0xf913c3f4>
     e64:	00 2b 30 16 	l.j accebc <_end+0xac8450>
     e68:	00 00 03 e9 	l.j 1e0c <_or1k_reset+0x1d0c>
     e6c:	04 02 46 00 	l.jal 9266c <_end+0x8dc00>
     e70:	00 05 73 34 	l.j 15db40 <_end+0x1590d4>
     e74:	16 00 00 03 	*unknown*
     e78:	40 04 02 48 	*unknown*
     e7c:	00 00 00 2b 	l.j f28 <_or1k_reset+0xe28>
     e80:	38 16 00 00 	*unknown*
     e84:	03 f9 04 02 	l.j ffe41e8c <_end+0xffe3d420>
     e88:	4a 00 00 08 	*unknown*
     e8c:	26 3c 16 00 	*unknown*
     e90:	00 03 09 04 	l.j c32a0 <_end+0xbe834>
     e94:	02 4d 00 00 	l.j f9340e94 <_end+0xf933c428>
     e98:	01 5d 40 16 	l.j 5750ef0 <_end+0x574c484>
     e9c:	00 00 02 75 	l.j 1870 <_or1k_reset+0x1770>
     ea0:	04 02 4e 00 	l.jal 946a0 <_end+0x8fc34>
     ea4:	00 00 2b 44 	l.j bbb4 <_end+0x7148>
     ea8:	16 00 00 05 	*unknown*
     eac:	14 04 02 4f 	*unknown*
     eb0:	00 00 01 5d 	l.j 1424 <_or1k_reset+0x1324>
     eb4:	48 16 00 00 	*unknown*
     eb8:	03 63 04 02 	l.j fd8c1ec0 <_end+0xfd8bd454>
     ebc:	50 00 00 08 	*unknown*
     ec0:	2c 4c 16 00 	*unknown*
     ec4:	00 02 93 04 	l.j a5ad4 <_end+0xa1068>
     ec8:	02 53 00 00 	l.j f94c0ec8 <_end+0xf94bc45c>
     ecc:	00 2b 50 16 	l.j ad4f24 <_end+0xad04b8>
     ed0:	00 00 02 05 	l.j 16e4 <_or1k_reset+0x15e4>
     ed4:	04 02 54 00 	l.jal 95ed4 <_end+0x91468>
     ed8:	00 05 42 54 	l.j 151828 <_end+0x14cdbc>
     edc:	16 00 00 03 	*unknown*
     ee0:	7f 04 02 77 	*unknown*
     ee4:	00 00 07 e9 	l.j 2e88 <_or1k_exception_handler+0xe0>
     ee8:	58 17 00 00 	*unknown*
     eec:	01 3d 04 02 	l.j 4f41ef4 <_end+0x4f3d488>
     ef0:	7b 00 00 02 	*unknown*
     ef4:	7a 01 48 17 	*unknown*
     ef8:	00 00 02 e7 	l.j 1a94 <_or1k_reset+0x1994>
     efc:	04 02 7c 00 	l.jal 9fefc <_end+0x9b490>
     f00:	00 02 3c 01 	l.j 8ff04 <_end+0x8b498>
     f04:	4c 17 00 00 	l.maci r23,0
     f08:	04 c8 04 02 	l.jal 3201f10 <_end+0x31fd4a4>
     f0c:	80 00 00 08 	*unknown*
     f10:	3d 02 dc 17 	*unknown*
     f14:	00 00 01 e7 	l.j 16b0 <_or1k_reset+0x15b0>
     f18:	04 02 85 00 	l.jal a2318 <_end+0x9d8ac>
     f1c:	00 05 ee 02 	l.j 17c724 <_end+0x177cb8>
     f20:	e0 17 00 00 	l.add r0,r23,r0
     f24:	01 cc 04 02 	l.j 7301f2c <_end+0x72fd4c0>
     f28:	86 00 00 08 	l.lwz r16,8(r0)
     f2c:	49 02 ec 00 	*unknown*
     f30:	0f 04 00 00 	l.bnf fc100f30 <_end+0xfc0fc4c4>
     f34:	05 48 02 01 	l.jal 5201738 <_end+0x51fcccc>
     f38:	06 00 00 00 	l.jal f8000f38 <_end+0xf7ffc4cc>
     f3c:	82 0f 04 00 	*unknown*
     f40:	00 03 ec 13 	l.j fbf8c <_end+0xf7520>
     f44:	00 00 00 2b 	l.j ff0 <_or1k_reset+0xef0>
     f48:	00 00 05 73 	l.j 2514 <__register_exitproc+0xb8>
     f4c:	14 00 00 04 	*unknown*
     f50:	0a 14 00 00 	*unknown*
     f54:	00 fd 14 00 	l.j 3f45f54 <_end+0x3f414e8>
     f58:	00 05 73 14 	l.j 15dba8 <_end+0x15913c>
     f5c:	00 00 00 2b 	l.j 1008 <_or1k_reset+0xf08>
     f60:	00 0f 04 00 	l.j 3c1f60 <_end+0x3bd4f4>
     f64:	00 05 79 18 	l.j 15f3c4 <_end+0x15a958>
     f68:	00 00 05 48 	l.j 2488 <__register_exitproc+0x2c>
     f6c:	0f 04 00 00 	l.bnf fc100f6c <_end+0xfc0fc500>
     f70:	05 55 13 00 	l.jal 5545b70 <_end+0x5541104>
     f74:	00 00 72 00 	l.j 1d774 <_end+0x18d08>
     f78:	00 05 a2 14 	l.j 1697c8 <_end+0x164d5c>
     f7c:	00 00 04 0a 	l.j 1fa4 <_or1k_reset+0x1ea4>
     f80:	14 00 00 00 	*unknown*
     f84:	fd 14 00 00 	*unknown*
     f88:	00 72 14 00 	l.j 1c85f88 <_end+0x1c8151c>
     f8c:	00 00 2b 00 	l.j bb8c <_end+0x7120>
     f90:	0f 04 00 00 	l.bnf fc100f90 <_end+0xfc0fc524>
     f94:	05 84 13 00 	l.jal 6105b94 <_end+0x6101128>
     f98:	00 00 2b 00 	l.j bb98 <_end+0x712c>
     f9c:	00 05 bc 14 	l.j 16ffec <_end+0x16b580>
     fa0:	00 00 04 0a 	l.j 1fc8 <_or1k_reset+0x1ec8>
     fa4:	14 00 00 00 	*unknown*
     fa8:	fd 00 0f 04 	*unknown*
     fac:	00 00 05 a8 	l.j 264c <__call_exitprocs+0xb0>
     fb0:	08 00 00 00 	*unknown*
     fb4:	39 00 00 05 	*unknown*
     fb8:	d2 09 00 00 	*unknown*
     fbc:	00 bf 02 00 	l.j 2fc17bc <_end+0x2fbcd50>
     fc0:	08 00 00 00 	*unknown*
     fc4:	39 00 00 05 	*unknown*
     fc8:	e2 09 00 00 	l.add r16,r9,r0
     fcc:	00 bf 00 00 	l.j 2fc0fcc <_end+0x2fbc560>
     fd0:	05 00 00 03 	l.jal 4000fdc <_end+0x3ffc570>
     fd4:	a4 04 01 1d 	l.andi r0,r4,0x11d
     fd8:	00 00 02 c2 	l.j 1ae0 <_or1k_reset+0x19e0>
     fdc:	19 00 00 04 	l.movhi r8,0x4
     fe0:	af 0c 04 01 	l.xori r24,r12,1025
     fe4:	21 00 00 06 	l.trap 0x6
     fe8:	23 16 00 00 	*unknown*
     fec:	03 cc 04 01 	l.j ff301ff0 <_end+0xff2fd584>
     ff0:	23 00 00 06 	*unknown*
     ff4:	23 00 16 00 	*unknown*
     ff8:	00 02 a1 04 	l.j a9408 <_end+0xa499c>
     ffc:	01 24 00 00 	l.j 4900ffc <_end+0x48fc590>
    1000:	00 2b 04 16 	l.j ac2058 <_end+0xabd5ec>
    1004:	00 00 03 9e 	l.j 1e7c <_or1k_reset+0x1d7c>
    1008:	04 01 25 00 	l.jal 4a408 <_end+0x4599c>
    100c:	00 06 29 08 	l.j 18b42c <_end+0x1869c0>
    1010:	00 0f 04 00 	l.j 3c2010 <_end+0x3bd5a4>
    1014:	00 05 ee 0f 	l.j 17c850 <_end+0x177de4>
    1018:	04 00 00 05 	l.jal 102c <_or1k_reset+0xf2c>
    101c:	e2 19 00 00 	l.add r16,r25,r0
    1020:	01 5a 0e 04 	l.j 5684830 <_end+0x567fdc4>
    1024:	01 3d 00 00 	l.j 4f41024 <_end+0x4f3c5b8>
    1028:	06 64 16 00 	l.jal f9906828 <_end+0xf9901dbc>
    102c:	00 04 0b 04 	l.j 103c3c <_end+0xff1d0>
    1030:	01 3e 00 00 	l.j 4f81030 <_end+0x4f7c5c4>
    1034:	06 64 00 16 	l.jal f990108c <_end+0xf98fc620>
    1038:	00 00 04 38 	l.j 2118 <_or1k_start+0xe8>
    103c:	04 01 3f 00 	l.jal 50c3c <_end+0x4c1d0>
    1040:	00 06 64 06 	l.j 19a058 <_end+0x1955ec>
    1044:	16 00 00 0e 	*unknown*
    1048:	6d 04 01 40 	l.lwa r8,320(r4)
    104c:	00 00 00 47 	l.j 1168 <_or1k_reset+0x1068>
    1050:	0c 00 08 00 	l.bnf 3050 <_or1k_interrupt_handler+0x34>
    1054:	00 00 47 00 	l.j 12c54 <_end+0xe1e8>
    1058:	00 06 74 09 	l.j 19e07c <_end+0x199610>
    105c:	00 00 00 bf 	l.j 1358 <_or1k_reset+0x1258>
    1060:	02 00 1a cc 	l.j f8007b90 <_end+0xf8003124>
    1064:	04 02 58 00 	l.jal 97064 <_end+0x925f8>
    1068:	00 07 75 16 	l.j 1de4c0 <_end+0x1d9a54>
    106c:	00 00 04 93 	l.j 22b8 <__do_global_dtors_aux+0xb0>
    1070:	04 02 5a 00 	l.jal 97870 <_end+0x92e04>
    1074:	00 00 89 00 	l.j 23474 <_end+0x1ea08>
    1078:	16 00 00 04 	*unknown*
    107c:	3e 04 02 5b 	*unknown*
    1080:	00 00 05 42 	l.j 2588 <__register_exitproc+0x12c>
    1084:	04 16 00 00 	l.jal 581084 <_end+0x57c618>
    1088:	02 fc 04 02 	l.j fbf02090 <_end+0xfbefd624>
    108c:	5c 00 00 07 	*unknown*
    1090:	75 08 16 00 	*unknown*
    1094:	00 04 eb 04 	l.j 13bca4 <_end+0x137238>
    1098:	02 5d 00 00 	l.j f9741098 <_end+0xf973c62c>
    109c:	01 73 24 16 	l.j 5cca0f4 <_end+0x5cc5688>
    10a0:	00 00 02 60 	l.j 1a20 <_or1k_reset+0x1920>
    10a4:	04 02 5e 00 	l.jal 988a4 <_end+0x93e38>
    10a8:	00 00 2b 48 	l.j bdc8 <_end+0x735c>
    10ac:	16 00 00 03 	*unknown*
    10b0:	c7 04 02 5f 	*unknown*
    10b4:	00 00 00 55 	l.j 1208 <_or1k_reset+0x1108>
    10b8:	4c 16 00 00 	l.maci r22,0
    10bc:	05 01 04 02 	l.jal 40420c4 <_end+0x403d658>
    10c0:	60 00 00 06 	*unknown*
    10c4:	2f 54 16 00 	*unknown*
    10c8:	00 03 d2 04 	l.j f58d8 <_end+0xf0e6c>
    10cc:	02 61 00 00 	l.j f98410cc <_end+0xf983c660>
    10d0:	00 e7 64 16 	l.j 39da128 <_end+0x39d56bc>
    10d4:	00 00 05 06 	l.j 24ec <__register_exitproc+0x90>
    10d8:	04 02 62 00 	l.jal 998d8 <_end+0x94e6c>
    10dc:	00 00 e7 6c 	l.j 3ae8c <_end+0x36420>
    10e0:	16 00 00 01 	*unknown*
    10e4:	a5 04 02 63 	l.andi r8,r4,0x263
    10e8:	00 00 00 e7 	l.j 1484 <_or1k_reset+0x1384>
    10ec:	74 16 00 00 	*unknown*
    10f0:	04 be 04 02 	l.jal 2f820f8 <_end+0x2f7d68c>
    10f4:	64 00 00 07 	*unknown*
    10f8:	85 7c 16 00 	l.lwz r11,5632(r28)
    10fc:	00 02 f0 04 	l.j bd10c <_end+0xb86a0>
    1100:	02 65 00 00 	l.j f9941100 <_end+0xf993c694>
    1104:	07 95 84 16 	l.jal fe56215c <_end+0xfe55d6f0>
    1108:	00 00 04 5c 	l.j 2278 <__do_global_dtors_aux+0x70>
    110c:	04 02 66 00 	l.jal 9a90c <_end+0x95ea0>
    1110:	00 00 2b 9c 	l.j bf80 <_end+0x7514>
    1114:	16 00 00 02 	*unknown*
    1118:	26 04 02 67 	*unknown*
    111c:	00 00 00 e7 	l.j 14b8 <_or1k_reset+0x13b8>
    1120:	a0 16 00 00 	l.addic r0,r22,0
    1124:	01 96 04 02 	l.j 658212c <_end+0x657d6c0>
    1128:	68 00 00 00 	*unknown*
    112c:	e7 a8 16 00 	*unknown*
    1130:	00 02 15 04 	l.j 86540 <_end+0x81ad4>
    1134:	02 69 00 00 	l.j f9a41134 <_end+0xf9a3c6c8>
    1138:	00 e7 b0 16 	l.j 39ed190 <_end+0x39e8724>
    113c:	00 00 01 6d 	l.j 16f0 <_or1k_reset+0x15f0>
    1140:	04 02 6a 00 	l.jal 9b940 <_end+0x96ed4>
    1144:	00 00 e7 b8 	l.j 3b024 <_end+0x365b8>
    1148:	16 00 00 01 	*unknown*
    114c:	7c 04 02 6b 	*unknown*
    1150:	00 00 00 e7 	l.j 14ec <_or1k_reset+0x13ec>
    1154:	c0 16 00 00 	l.mtspr r22,r0,0x0
    1158:	03 84 04 02 	l.j fe102160 <_end+0xfe0fd6f4>
    115c:	6c 00 00 00 	l.lwa r0,0(r0)
    1160:	2b c8 00 08 	*unknown*
    1164:	00 00 05 48 	l.j 2684 <__call_exitprocs+0xe8>
    1168:	00 00 07 85 	l.j 2f7c <or1k_interrupts_enable+0x4>
    116c:	09 00 00 00 	*unknown*
    1170:	bf 19 00 08 	*unknown*
    1174:	00 00 05 48 	l.j 2694 <__call_exitprocs+0xf8>
    1178:	00 00 07 95 	l.j 2fcc <or1k_interrupts_disable+0x30>
    117c:	09 00 00 00 	*unknown*
    1180:	bf 07 00 08 	*unknown*
    1184:	00 00 05 48 	l.j 26a4 <__call_exitprocs+0x108>
    1188:	00 00 07 a5 	l.j 301c <_or1k_interrupt_handler>
    118c:	09 00 00 00 	*unknown*
    1190:	bf 17 00 1a 	*unknown*
    1194:	f0 04 02 71 	*unknown*
    1198:	00 00 07 c9 	l.j 30bc <or1k_interrupt_disable+0x4>
    119c:	16 00 00 03 	*unknown*
    11a0:	30 04 02 74 	*unknown*
    11a4:	00 00 07 c9 	l.j 30c8 <or1k_interrupt_disable+0x10>
    11a8:	00 16 00 00 	l.j 5811a8 <_end+0x57c73c>
    11ac:	04 b5 04 02 	l.jal 2d421b4 <_end+0x2d3d748>
    11b0:	75 00 00 07 	*unknown*
    11b4:	d9 78 00 08 	l.sb 22536(r24),r0
    11b8:	00 00 02 bc 	l.j 1ca8 <_or1k_reset+0x1ba8>
    11bc:	00 00 07 d9 	l.j 3120 <_or1k_libc_impure_init+0x3c>
    11c0:	09 00 00 00 	*unknown*
    11c4:	bf 1d 00 08 	*unknown*
    11c8:	00 00 00 89 	l.j 13ec <_or1k_reset+0x12ec>
    11cc:	00 00 07 e9 	l.j 3170 <_or1k_libc_impure_init+0x8c>
    11d0:	09 00 00 00 	*unknown*
    11d4:	bf 1d 00 1b 	*unknown*
    11d8:	f0 04 02 56 	*unknown*
    11dc:	00 00 08 0b 	l.j 3208 <_or1k_libc_impure_init+0x124>
    11e0:	1c 00 00 0c 	*unknown*
    11e4:	0b 04 02 6d 	*unknown*
    11e8:	00 00 06 74 	l.j 2bb8 <or1k_uart_set_read_cb+0x44>
    11ec:	1c 00 00 04 	*unknown*
    11f0:	d8 04 02 76 	l.sb 630(r4),r0
    11f4:	00 00 07 a5 	l.j 3088 <_or1k_interrupt_handler+0x6c>
    11f8:	00 08 00 00 	l.j 2011f8 <_end+0x1fc78c>
    11fc:	05 48 00 00 	l.jal 52011fc <_end+0x51fc790>
    1200:	08 1b 09 00 	*unknown*
    1204:	00 00 bf 18 	l.j 30e64 <_end+0x2c3f8>
    1208:	00 1d 00 00 	l.j 741208 <_end+0x73c79c>
    120c:	08 26 14 00 	*unknown*
    1210:	00 04 0a 00 	l.j 103a10 <_end+0xfefa4>
    1214:	0f 04 00 00 	l.bnf fc101214 <_end+0xfc0fc7a8>
    1218:	08 1b 0f 04 	*unknown*
    121c:	00 00 01 5d 	l.j 1790 <_or1k_reset+0x1690>
    1220:	1d 00 00 08 	*unknown*
    1224:	3d 14 00 00 	*unknown*
    1228:	00 2b 00 0f 	l.j ac1264 <_end+0xabc7f8>
    122c:	04 00 00 08 	l.jal 124c <_or1k_reset+0x114c>
    1230:	43 0f 04 00 	*unknown*
    1234:	00 08 32 08 	l.j 20da54 <_end+0x208fe8>
    1238:	00 00 05 e2 	l.j 29c0 <_read_r+0x14>
    123c:	00 00 08 59 	l.j 33a0 <_or1k_timer_interrupt_handler+0x1c>
    1240:	09 00 00 00 	*unknown*
    1244:	bf 02 00 1e 	*unknown*
    1248:	00 00 0c 64 	l.j 43d8 <impure_data+0x384>
    124c:	05 17 00 00 	l.jal 45c124c <_end+0x45bc7e0>
    1250:	04 10 05 03 	l.jal 40265c <_end+0x3fdbf0>
    1254:	00 00 40 54 	l.j 113a4 <_end+0xc938>
    1258:	1f 00 00 0c 	*unknown*
    125c:	86 04 02 fa 	l.lwz r16,762(r4)
    1260:	00 00 04 0a 	l.j 2288 <__do_global_dtors_aux+0x80>
    1264:	05 03 00 00 	l.jal 40c1264 <_end+0x40bc7f8>
    1268:	40 50 1f 00 	*unknown*
    126c:	00 04 69 04 	l.j 11b67c <_end+0x116c10>
    1270:	02 fb 00 00 	l.j fbec1270 <_end+0xfbebc804>
    1274:	08 8e 05 03 	*unknown*
    1278:	00 00 3a 34 	l.j fb48 <_end+0xb0dc>
    127c:	18 00 00 04 	l.movhi r0,0x4
    1280:	0a 00 00 00 	*unknown*
    1284:	09 30 00 04 	*unknown*
    1288:	00 00 04 1b 	l.j 22f4 <call___do_global_dtors_aux+0x14>
    128c:	04 01 00 00 	l.jal 4128c <_end+0x3c820>
    1290:	00 30 01 00 	l.j c01690 <_end+0xbfcc24>
    1294:	00 05 a9 00 	l.j 16b694 <_end+0x166c28>
    1298:	00 00 9b 00 	l.j 27e98 <_end+0x2342c>
    129c:	00 24 5c 00 	l.j 91829c <_end+0x913830>
    12a0:	00 01 40 00 	l.j 512a0 <_end+0x4c834>
    12a4:	00 03 64 02 	l.j da2ac <_end+0xd5840>
    12a8:	04 05 00 00 	l.jal 1412a8 <_end+0x13c83c>
    12ac:	00 05 03 00 	l.j 141eac <_end+0x13d440>
    12b0:	00 08 c2 02 	l.j 231ab8 <_end+0x22d04c>
    12b4:	d4 00 00 00 	l.sw 0(r0),r0
    12b8:	37 02 04 07 	*unknown*
    12bc:	00 00 00 5e 	l.j 1434 <_or1k_reset+0x1334>
    12c0:	04 04 05 69 	l.jal 102864 <_end+0xfddf8>
    12c4:	6e 74 00 02 	l.lwa r19,2(r20)
    12c8:	01 06 00 00 	l.j 41812c8 <_end+0x417c85c>
    12cc:	00 7b 02 01 	l.j 1ec1ad0 <_end+0x1ebd064>
    12d0:	08 00 00 00 	*unknown*
    12d4:	79 02 02 05 	*unknown*
    12d8:	00 00 01 2f 	l.j 1794 <_or1k_reset+0x1694>
    12dc:	02 02 07 00 	l.j f8082edc <_end+0xf807e470>
    12e0:	00 00 0e 02 	l.j 4ae8 <_end+0x7c>
    12e4:	08 05 00 00 	*unknown*
    12e8:	00 00 02 08 	l.j 1b08 <_or1k_reset+0x1a08>
    12ec:	07 00 00 00 	l.jal fc0012ec <_end+0xfbffc880>
    12f0:	59 03 00 00 	*unknown*
    12f4:	03 6d 03 07 	l.j fdb41f10 <_end+0xfdb3d4a4>
    12f8:	00 00 00 3e 	l.j 13f0 <_or1k_reset+0x12f0>
    12fc:	03 00 00 03 	l.j fc001308 <_end+0xfbffc89c>
    1300:	4b 04 10 00 	*unknown*
    1304:	00 00 25 03 	l.j a710 <_end+0x5ca4>
    1308:	00 00 04 27 	l.j 23a4 <main+0xc>
    130c:	04 27 00 00 	l.jal 9c130c <_end+0x9bc8a0>
    1310:	00 25 05 00 	l.j 942710 <_end+0x93dca4>
    1314:	00 03 17 02 	l.j c6f1c <_end+0xc24b0>
    1318:	01 61 00 00 	l.j 5841318 <_end+0x583c8ac>
    131c:	00 9c 02 04 	l.j 2701b2c <_end+0x26fd0c0>
    1320:	07 00 00 00 	l.jal fc001320 <_end+0xfbffc8b4>
    1324:	63 06 04 04 	*unknown*
    1328:	4a 00 00 00 	*unknown*
    132c:	c2 07 00 00 	l.mtspr r7,r0,0x8000
    1330:	03 11 04 4c 	l.j fc442460 <_end+0xfc43d9f4>
    1334:	00 00 00 90 	l.j 1574 <_or1k_reset+0x1474>
    1338:	07 00 00 02 	l.jal fc001340 <_end+0xfbffc8d4>
    133c:	84 04 4d 00 	l.lwz r0,19712(r4)
    1340:	00 00 c2 00 	l.j 31b40 <_end+0x2d0d4>
    1344:	08 00 00 00 	*unknown*
    1348:	4c 00 00 00 	l.maci r0,0
    134c:	d2 09 00 00 	*unknown*
    1350:	00 d2 03 00 	l.j 3481f50 <_end+0x347d4e4>
    1354:	02 04 07 00 	l.j f8102f54 <_end+0xf80fe4e8>
    1358:	00 01 45 0a 	l.j 52780 <_end+0x4dd14>
    135c:	08 04 47 00 	*unknown*
    1360:	00 00 fa 0b 	l.j 3fb8c <_end+0x3b120>
    1364:	00 00 04 11 	l.j 23a8 <main+0x10>
    1368:	04 49 00 00 	l.jal 1241368 <_end+0x123c8fc>
    136c:	00 3e 00 0b 	l.j f81398 <_end+0xf7c92c>
    1370:	00 00 04 19 	l.j 23d4 <main+0x3c>
    1374:	04 4e 00 00 	l.jal 1381374 <_end+0x137c908>
    1378:	00 a3 04 00 	l.j 28c2378 <_end+0x28bd90c>
    137c:	03 00 00 03 	l.j fc001388 <_end+0xfbffc91c>
    1380:	ab 04 4f 00 	l.ori r24,r4,0x4f00
    1384:	00 00 d9 03 	l.j 37790 <_end+0x32d24>
    1388:	00 00 02 47 	l.j 1ca4 <_or1k_reset+0x1ba4>
    138c:	04 53 00 00 	l.jal 14c138c <_end+0x14bc920>
    1390:	00 6f 0c 04 	l.j 1bc43a0 <_end+0x1bbf934>
    1394:	03 00 00 04 	l.j fc0013a4 <_end+0xfbffc938>
    1398:	54 05 16 00 	*unknown*
    139c:	00 00 37 0d 	l.j efd0 <_end+0xa564>
    13a0:	00 00 02 58 	l.j 1d00 <_or1k_reset+0x1c00>
    13a4:	18 05 2d 00 	*unknown*
    13a8:	00 01 70 0b 	l.j 5d3d4 <_end+0x58968>
    13ac:	00 00 03 cc 	l.j 22dc <__do_global_dtors_aux+0xd4>
    13b0:	05 2f 00 00 	l.jal 4bc13b0 <_end+0x4bbc944>
    13b4:	01 70 00 0e 	l.j 5c013ec <_end+0x5bfc980>
    13b8:	5f 6b 00 05 	*unknown*
    13bc:	30 00 00 00 	*unknown*
    13c0:	3e 04 0b 00 	*unknown*
    13c4:	00 04 03 05 	l.j 101fd8 <_end+0xfd56c>
    13c8:	30 00 00 00 	*unknown*
    13cc:	3e 08 0b 00 	*unknown*
    13d0:	00 02 41 05 	l.j 917e4 <_end+0x8cd78>
    13d4:	30 00 00 00 	*unknown*
    13d8:	3e 0c 0b 00 	*unknown*
    13dc:	00 04 a0 05 	l.j 1293f0 <_end+0x124984>
    13e0:	30 00 00 00 	*unknown*
    13e4:	3e 10 0e 5f 	*unknown*
    13e8:	78 00 05 31 	*unknown*
    13ec:	00 00 01 76 	l.j 19c4 <_or1k_reset+0x18c4>
    13f0:	14 00 0f 04 	*unknown*
    13f4:	00 00 01 1d 	l.j 1868 <_or1k_reset+0x1768>
    13f8:	08 00 00 01 	*unknown*
    13fc:	12 00 00 01 	l.bf f8001400 <_end+0xf7ffc994>
    1400:	86 09 00 00 	l.lwz r16,0(r9)
    1404:	00 d2 00 00 	l.j 3481404 <_end+0x347c998>
    1408:	0d 00 00 02 	l.bnf 4001410 <_end+0x3ffc9a4>
    140c:	7f 24 05 35 	*unknown*
    1410:	00 00 01 ff 	l.j 1c0c <_or1k_reset+0x1b0c>
    1414:	0b 00 00 01 	*unknown*
    1418:	b3 05 37 00 	l.muli r24,r5,14080
    141c:	00 00 3e 00 	l.j 10c1c <_end+0xc1b0>
    1420:	0b 00 00 04 	*unknown*
    1424:	2f 05 38 00 	*unknown*
    1428:	00 00 3e 04 	l.j 10c38 <_end+0xc1cc>
    142c:	0b 00 00 01 	*unknown*
    1430:	c2 05 39 00 	l.mtspr r5,r7,0x8100
    1434:	00 00 3e 08 	l.j 10c54 <_end+0xc1e8>
    1438:	0b 00 00 05 	*unknown*
    143c:	19 05 3a 00 	*unknown*
    1440:	00 00 3e 0c 	l.j 10c70 <_end+0xc204>
    1444:	0b 00 00 03 	*unknown*
    1448:	37 05 3b 00 	*unknown*
    144c:	00 00 3e 10 	l.j 10c8c <_end+0xc220>
    1450:	0b 00 00 03 	*unknown*
    1454:	26 05 3c 00 	*unknown*
    1458:	00 00 3e 14 	l.j 10ca8 <_end+0xc23c>
    145c:	0b 00 00 04 	*unknown*
    1460:	a5 05 3d 00 	l.andi r8,r5,0x3d00
    1464:	00 00 3e 18 	l.j 10cc4 <_end+0xc258>
    1468:	0b 00 00 03 	*unknown*
    146c:	8d 05 3e 00 	l.lbz r8,15872(r5)
    1470:	00 00 3e 1c 	l.j 10ce0 <_end+0xc274>
    1474:	0b 00 00 04 	*unknown*
    1478:	e0 05 3f 00 	*unknown*
    147c:	00 00 3e 20 	l.j 10cfc <_end+0xc290>
    1480:	00 10 00 00 	l.j 401480 <_end+0x3fca14>
    1484:	01 d1 01 08 	l.j 74418a4 <_end+0x743ce38>
    1488:	05 48 00 00 	l.jal 5201488 <_end+0x51fca1c>
    148c:	02 3f 0b 00 	l.j f8fc408c <_end+0xf8fbf620>
    1490:	00 02 34 05 	l.j 8e4a4 <_end+0x89a38>
    1494:	49 00 00 02 	*unknown*
    1498:	3f 00 0b 00 	*unknown*
    149c:	00 01 4e 05 	l.j 54cb0 <_end+0x50244>
    14a0:	4a 00 00 02 	*unknown*
    14a4:	3f 80 11 00 	*unknown*
    14a8:	00 04 4b 05 	l.j 1140bc <_end+0x10f650>
    14ac:	4c 00 00 01 	l.maci r0,1
    14b0:	12 01 00 11 	l.bf f80414f4 <_end+0xf803ca88>
    14b4:	00 00 01 f6 	l.j 1c8c <_or1k_reset+0x1b8c>
    14b8:	05 4f 00 00 	l.jal 53c14b8 <_end+0x53bca4c>
    14bc:	01 12 01 04 	l.j 44818cc <_end+0x447ce60>
    14c0:	00 08 00 00 	l.j 2014c0 <_end+0x1fca54>
    14c4:	01 10 00 00 	l.j 44014c4 <_end+0x43fca58>
    14c8:	02 4f 09 00 	l.j f93c38c8 <_end+0xf93bee5c>
    14cc:	00 00 d2 1f 	l.j 35d48 <_end+0x312dc>
    14d0:	00 10 00 00 	l.j 4014d0 <_end+0x3fca64>
    14d4:	01 3d 01 90 	l.j 4f41b14 <_end+0x4f3d0a8>
    14d8:	05 5b 00 00 	l.jal 56c14d8 <_end+0x56bca6c>
    14dc:	02 8d 0b 00 	l.j fa3440dc <_end+0xfa33f670>
    14e0:	00 03 cc 05 	l.j f44f4 <_end+0xefa88>
    14e4:	5c 00 00 02 	*unknown*
    14e8:	8d 00 0b 00 	l.lbz r8,2816(r0)
    14ec:	00 03 e4 05 	l.j fa500 <_end+0xf5a94>
    14f0:	5d 00 00 00 	*unknown*
    14f4:	3e 04 0b 00 	*unknown*
    14f8:	00 02 3c 05 	l.j 9050c <_end+0x8baa0>
    14fc:	5f 00 00 02 	*unknown*
    1500:	93 08 0b 00 	l.lbs r24,2816(r8)
    1504:	00 01 d1 05 	l.j 75918 <_end+0x70eac>
    1508:	60 00 00 01 	*unknown*
    150c:	ff 88 00 0f 	*unknown*
    1510:	04 00 00 02 	l.jal 1518 <_or1k_reset+0x1418>
    1514:	4f 08 00 00 	*unknown*
    1518:	02 a3 00 00 	l.j fa8c1518 <_end+0xfa8bcaac>
    151c:	02 a3 09 00 	l.j fa8c391c <_end+0xfa8beeb0>
    1520:	00 00 d2 1f 	l.j 35d9c <_end+0x31330>
    1524:	00 0f 04 00 	l.j 3c2524 <_end+0x3bdab8>
    1528:	00 02 a9 12 	l.j ab970 <_end+0xa6f04>
    152c:	0d 00 00 03 	l.bnf 4001538 <_end+0x3ffcacc>
    1530:	97 08 05 73 	l.lhz r24,1395(r8)
    1534:	00 00 02 cf 	l.j 2070 <_or1k_start+0x40>
    1538:	0b 00 00 09 	*unknown*
    153c:	f8 05 74 00 	*unknown*
    1540:	00 02 cf 00 	l.j b5140 <_end+0xb06d4>
    1544:	0b 00 00 07 	*unknown*
    1548:	8b 05 75 00 	l.lws r24,29952(r5)
    154c:	00 00 3e 04 	l.j 10d5c <_end+0xc2f0>
    1550:	00 0f 04 00 	l.j 3c2550 <_end+0x3bdae4>
    1554:	00 00 4c 0d 	l.j 14588 <_end+0xfb1c>
    1558:	00 00 03 b6 	l.j 2430 <exit+0x1c>
    155c:	68 05 b3 00 	*unknown*
    1560:	00 03 ff 0e 	l.j 101198 <_end+0xfc72c>
    1564:	5f 70 00 05 	*unknown*
    1568:	b4 00 00 02 	l.mfspr r0,r0,0x2
    156c:	cf 00 0e 5f 	l.swa -14753(r0),r1
    1570:	72 00 05 b5 	*unknown*
    1574:	00 00 00 3e 	l.j 166c <_or1k_reset+0x156c>
    1578:	04 0e 5f 77 	l.jal 399354 <_end+0x3948e8>
    157c:	00 05 b6 00 	l.j 16ed7c <_end+0x16a310>
    1580:	00 00 3e 08 	l.j 10da0 <_end+0xc334>
    1584:	0b 00 00 01 	*unknown*
    1588:	ef 05 b7 00 	*unknown*
    158c:	00 00 53 0c 	l.j 161bc <_end+0x11750>
    1590:	0b 00 00 02 	*unknown*
    1594:	9b 05 b8 00 	l.lhs r24,-18432(r5)
    1598:	00 00 53 0e 	l.j 161d0 <_end+0x11764>
    159c:	0e 5f 62 66 	l.bnf f97d9f34 <_end+0xf97d54c8>
    15a0:	00 05 b9 00 	l.j 16f9a0 <_end+0x16af34>
    15a4:	00 02 aa 10 	l.j abde4 <_end+0xa7378>
    15a8:	0b 00 00 01 	*unknown*
    15ac:	8d 05 ba 00 	l.lbz r8,-17920(r5)
    15b0:	00 00 3e 18 	l.j 10e10 <_end+0xc3a4>
    15b4:	0b 00 00 01 	*unknown*
    15b8:	df 05 c1 00 	l.sh -16128(r5),r24
    15bc:	00 01 10 1c 	l.j 4562c <_end+0x40bc0>
    15c0:	0b 00 00 02 	*unknown*
    15c4:	6f 05 c3 00 	l.lwa r24,-15616(r5)
    15c8:	00 05 62 20 	l.j 159e48 <_end+0x1553dc>
    15cc:	0b 00 00 09 	*unknown*
    15d0:	a6 05 c5 00 	l.andi r16,r5,0xc500
    15d4:	00 05 91 24 	l.j 165a64 <_end+0x160ff8>
    15d8:	0b 00 00 04 	*unknown*
    15dc:	21 05 c8 00 	*unknown*
    15e0:	00 05 b5 28 	l.j 16ea80 <_end+0x16a014>
    15e4:	0b 00 00 04 	*unknown*
    15e8:	fa 05 c9 00 	*unknown*
    15ec:	00 05 cf 2c 	l.j 17529c <_end+0x170830>
    15f0:	0e 5f 75 62 	l.bnf f97deb78 <_end+0xf97da10c>
    15f4:	00 05 cc 00 	l.j 1745f4 <_end+0x16fb88>
    15f8:	00 02 aa 30 	l.j abeb8 <_end+0xa744c>
    15fc:	0e 5f 75 70 	l.bnf f97debbc <_end+0xf97da150>
    1600:	00 05 cd 00 	l.j 174a00 <_end+0x16ff94>
    1604:	00 02 cf 38 	l.j b52e4 <_end+0xb0878>
    1608:	0e 5f 75 72 	l.bnf f97debd0 <_end+0xf97da164>
    160c:	00 05 ce 00 	l.j 174e0c <_end+0x1703a0>
    1610:	00 00 3e 3c 	l.j 10f00 <_end+0xc494>
    1614:	0b 00 00 01 	*unknown*
    1618:	bc 05 d1 00 	l.sfeqi r5,-12032
    161c:	00 05 d5 40 	l.j 176b1c <_end+0x1720b0>
    1620:	0b 00 00 04 	*unknown*
    1624:	d2 05 d2 00 	*unknown*
    1628:	00 05 e5 43 	l.j 17ab34 <_end+0x1760c8>
    162c:	0e 5f 6c 62 	l.bnf f97dc7b4 <_end+0xf97d7d48>
    1630:	00 05 d5 00 	l.j 176a30 <_end+0x171fc4>
    1634:	00 02 aa 44 	l.j abf44 <_end+0xa74d8>
    1638:	0b 00 00 08 	*unknown*
    163c:	18 05 d8 00 	*unknown*
    1640:	00 00 3e 4c 	l.j 10f70 <_end+0xc504>
    1644:	0b 00 00 02 	*unknown*
    1648:	0d 05 d9 00 	l.bnf 4177a48 <_end+0x4172fdc>
    164c:	00 00 7a 50 	l.j 1ff8c <_end+0x1b520>
    1650:	0b 00 00 0c 	*unknown*
    1654:	6a 05 dc 00 	*unknown*
    1658:	00 04 1d 54 	l.j 108ba8 <_end+0x10413c>
    165c:	0b 00 00 06 	*unknown*
    1660:	4f 05 e0 00 	*unknown*
    1664:	00 01 05 58 	l.j 42bc4 <_end+0x3e158>
    1668:	0b 00 00 03 	*unknown*
    166c:	be 05 e2 00 	*unknown*
    1670:	00 00 fa 5c 	l.j 3ffe0 <_end+0x3b574>
    1674:	0b 00 00 03 	*unknown*
    1678:	1e 05 e3 00 	*unknown*
    167c:	00 00 3e 64 	l.j 1100c <_end+0xc5a0>
    1680:	00 13 00 00 	l.j 4c1680 <_end+0x4bcc14>
    1684:	00 3e 00 00 	l.j f81684 <_end+0xf7cc18>
    1688:	04 1d 14 00 	l.jal 746688 <_end+0x741c1c>
    168c:	00 04 1d 14 	l.j 108adc <_end+0x104070>
    1690:	00 00 01 10 	l.j 1ad0 <_or1k_reset+0x19d0>
    1694:	14 00 00 05 	*unknown*
    1698:	55 14 00 00 	*unknown*
    169c:	00 3e 00 0f 	l.j f816d8 <_end+0xf7cc6c>
    16a0:	04 00 00 04 	l.jal 16b0 <_or1k_reset+0x15b0>
    16a4:	23 15 00 00 	*unknown*
    16a8:	0c 0b 04 24 	l.bnf 2c2738 <_end+0x2bdccc>
    16ac:	05 02 39 00 	l.jal 408faac <_end+0x408b040>
    16b0:	00 05 55 16 	l.j 156b08 <_end+0x15209c>
    16b4:	00 00 06 bd 	l.j 31a8 <_or1k_libc_impure_init+0xc4>
    16b8:	05 02 3b 00 	l.jal 40902b8 <_end+0x408b84c>
    16bc:	00 00 3e 00 	l.j 10ebc <_end+0xc450>
    16c0:	16 00 00 01 	*unknown*
    16c4:	fe 05 02 40 	*unknown*
    16c8:	00 00 06 3c 	l.j 2fb8 <or1k_interrupts_disable+0x1c>
    16cc:	04 16 00 00 	l.jal 5816cc <_end+0x57cc60>
    16d0:	02 8b 05 02 	l.j fa2c2ad8 <_end+0xfa2be06c>
    16d4:	40 00 00 06 	*unknown*
    16d8:	3c 08 16 00 	*unknown*
    16dc:	00 02 50 05 	l.j 956f0 <_end+0x90c84>
    16e0:	02 40 00 00 	l.j f90016e0 <_end+0xf8ffcc74>
    16e4:	06 3c 0c 16 	l.jal f8f0473c <_end+0xf8effcd0>
    16e8:	00 00 03 df 	l.j 2664 <__call_exitprocs+0xc8>
    16ec:	05 02 42 00 	l.jal 4091eec <_end+0x408d480>
    16f0:	00 00 3e 10 	l.j 10f30 <_end+0xc4c4>
    16f4:	16 00 00 01 	*unknown*
    16f8:	62 05 02 43 	*unknown*
    16fc:	00 00 08 1e 	l.j 3774 <memset+0x30>
    1700:	14 16 00 00 	*unknown*
    1704:	04 7c 05 02 	l.jal 1f02b0c <_end+0x1efe0a0>
    1708:	45 00 00 00 	*unknown*
    170c:	3e 30 16 00 	*unknown*
    1710:	00 03 e9 05 	l.j fbb24 <_end+0xf70b8>
    1714:	02 46 00 00 	l.j f9181714 <_end+0xf917cca8>
    1718:	05 86 34 16 	l.jal 618e770 <_end+0x6189d04>
    171c:	00 00 03 40 	l.j 241c <exit+0x8>
    1720:	05 02 48 00 	l.jal 4093720 <_end+0x408ecb4>
    1724:	00 00 3e 38 	l.j 11004 <_end+0xc598>
    1728:	16 00 00 03 	*unknown*
    172c:	f9 05 02 4a 	*unknown*
    1730:	00 00 08 39 	l.j 3814 <memset+0xd0>
    1734:	3c 16 00 00 	*unknown*
    1738:	03 09 05 02 	l.j fc242b40 <_end+0xfc23e0d4>
    173c:	4d 00 00 01 	*unknown*
    1740:	70 40 16 00 	*unknown*
    1744:	00 02 75 05 	l.j 9eb58 <_end+0x9a0ec>
    1748:	02 4e 00 00 	l.j f9381748 <_end+0xf937ccdc>
    174c:	00 3e 44 16 	l.j f927a4 <_end+0xf8dd38>
    1750:	00 00 05 14 	l.j 2ba0 <or1k_uart_set_read_cb+0x2c>
    1754:	05 02 4f 00 	l.jal 4095354 <_end+0x40908e8>
    1758:	00 01 70 48 	l.j 5d878 <_end+0x58e0c>
    175c:	16 00 00 03 	*unknown*
    1760:	63 05 02 50 	*unknown*
    1764:	00 00 08 3f 	l.j 3860 <memset+0x11c>
    1768:	4c 16 00 00 	l.maci r22,0
    176c:	02 93 05 02 	l.j fa4c2b74 <_end+0xfa4be108>
    1770:	53 00 00 00 	*unknown*
    1774:	3e 50 16 00 	*unknown*
    1778:	00 02 05 05 	l.j 82b8c <_end+0x7e120>
    177c:	02 54 00 00 	l.j f950177c <_end+0xf94fcd10>
    1780:	05 55 54 16 	l.jal 55567d8 <_end+0x5551d6c>
    1784:	00 00 03 7f 	l.j 2580 <__register_exitproc+0x124>
    1788:	05 02 77 00 	l.jal 409f388 <_end+0x409a91c>
    178c:	00 07 fc 58 	l.j 2008ec <_end+0x1fbe80>
    1790:	17 00 00 01 	*unknown*
    1794:	3d 05 02 7b 	*unknown*
    1798:	00 00 02 8d 	l.j 21cc <register_tm_clones+0x30>
    179c:	01 48 17 00 	l.j 520739c <_end+0x5202930>
    17a0:	00 02 e7 05 	l.j bb3b4 <_end+0xb6948>
    17a4:	02 7c 00 00 	l.j f9f017a4 <_end+0xf9efcd38>
    17a8:	02 4f 01 4c 	l.j f93c1cd8 <_end+0xf93bd26c>
    17ac:	17 00 00 04 	*unknown*
    17b0:	c8 05 02 80 	*unknown*
    17b4:	00 00 08 50 	l.j 38f4 <__udivsi3+0x48>
    17b8:	02 dc 17 00 	l.j fb7073b8 <_end+0xfb70294c>
    17bc:	00 01 e7 05 	l.j 7b3d0 <_end+0x76964>
    17c0:	02 85 00 00 	l.j fa1417c0 <_end+0xfa13cd54>
    17c4:	06 01 02 e0 	l.jal f8042344 <_end+0xf803d8d8>
    17c8:	17 00 00 01 	*unknown*
    17cc:	cc 05 02 86 	l.swa 646(r5),r0
    17d0:	00 00 08 5c 	l.j 3940 <__udivsi3+0x94>
    17d4:	02 ec 00 0f 	l.j fbb01810 <_end+0xfbafcda4>
    17d8:	04 00 00 05 	l.jal 17ec <_or1k_reset+0x16ec>
    17dc:	5b 02 01 06 	*unknown*
    17e0:	00 00 00 82 	l.j 19e8 <_or1k_reset+0x18e8>
    17e4:	0f 04 00 00 	l.bnf fc1017e4 <_end+0xfc0fcd78>
    17e8:	03 ff 13 00 	l.j fffc63e8 <_end+0xfffc197c>
    17ec:	00 00 3e 00 	l.j 10fec <_end+0xc580>
    17f0:	00 05 86 14 	l.j 163040 <_end+0x15e5d4>
    17f4:	00 00 04 1d 	l.j 2868 <_execve_r+0x14>
    17f8:	14 00 00 01 	*unknown*
    17fc:	10 14 00 00 	l.bf 5017fc <_end+0x4fcd90>
    1800:	05 86 14 00 	l.jal 6186800 <_end+0x6181d94>
    1804:	00 00 3e 00 	l.j 11004 <_end+0xc598>
    1808:	0f 04 00 00 	l.bnf fc101808 <_end+0xfc0fcd9c>
    180c:	05 8c 18 00 	l.jal 630780c <_end+0x6302da0>
    1810:	00 05 5b 0f 	l.j 15844c <_end+0x1539e0>
    1814:	04 00 00 05 	l.jal 1828 <_or1k_reset+0x1728>
    1818:	68 13 00 00 	*unknown*
    181c:	00 85 00 00 	l.j 214181c <_end+0x213cdb0>
    1820:	05 b5 14 00 	l.jal 6d46820 <_end+0x6d41db4>
    1824:	00 04 1d 14 	l.j 108c74 <_end+0x104208>
    1828:	00 00 01 10 	l.j 1c68 <_or1k_reset+0x1b68>
    182c:	14 00 00 00 	*unknown*
    1830:	85 14 00 00 	l.lwz r8,0(r20)
    1834:	00 3e 00 0f 	l.j f81870 <_end+0xf7ce04>
    1838:	04 00 00 05 	l.jal 184c <_or1k_reset+0x174c>
    183c:	97 13 00 00 	l.lhz r24,0(r19)
    1840:	00 3e 00 00 	l.j f81840 <_end+0xf7cdd4>
    1844:	05 cf 14 00 	l.jal 73c6844 <_end+0x73c1dd8>
    1848:	00 04 1d 14 	l.j 108c98 <_end+0x10422c>
    184c:	00 00 01 10 	l.j 1c8c <_or1k_reset+0x1b8c>
    1850:	00 0f 04 00 	l.j 3c2850 <_end+0x3bdde4>
    1854:	00 05 bb 08 	l.j 170474 <_end+0x16ba08>
    1858:	00 00 00 4c 	l.j 1988 <_or1k_reset+0x1888>
    185c:	00 00 05 e5 	l.j 2ff0 <or1k_interrupts_restore+0x1c>
    1860:	09 00 00 00 	*unknown*
    1864:	d2 02 00 08 	*unknown*
    1868:	00 00 00 4c 	l.j 1998 <_or1k_reset+0x1898>
    186c:	00 00 05 f5 	l.j 3040 <_or1k_interrupt_handler+0x24>
    1870:	09 00 00 00 	*unknown*
    1874:	d2 00 00 05 	*unknown*
    1878:	00 00 03 a4 	l.j 2708 <__call_exitprocs+0x16c>
    187c:	05 01 1d 00 	l.jal 4048c7c <_end+0x4044210>
    1880:	00 02 d5 19 	l.j b6ce4 <_end+0xb2278>
    1884:	00 00 04 af 	l.j 2b40 <_or1k_uart_write+0x14>
    1888:	0c 05 01 21 	l.bnf 141d0c <_end+0x13d2a0>
    188c:	00 00 06 36 	l.j 3164 <_or1k_libc_impure_init+0x80>
    1890:	16 00 00 03 	*unknown*
    1894:	cc 05 01 23 	l.swa 291(r5),r0
    1898:	00 00 06 36 	l.j 3170 <_or1k_libc_impure_init+0x8c>
    189c:	00 16 00 00 	l.j 58189c <_end+0x57ce30>
    18a0:	02 a1 05 01 	l.j fa842ca4 <_end+0xfa83e238>
    18a4:	24 00 00 00 	l.rfe
    18a8:	3e 04 16 00 	*unknown*
    18ac:	00 03 9e 05 	l.j e90c0 <_end+0xe4654>
    18b0:	01 25 00 00 	l.j 49418b0 <_end+0x493ce44>
    18b4:	06 3c 08 00 	l.jal f8f038b4 <_end+0xf8efee48>
    18b8:	0f 04 00 00 	l.bnf fc1018b8 <_end+0xfc0fce4c>
    18bc:	06 01 0f 04 	l.jal f80454cc <_end+0xf8040a60>
    18c0:	00 00 05 f5 	l.j 3094 <or1k_interrupt_enable+0x4>
    18c4:	19 00 00 01 	l.movhi r8,0x1
    18c8:	5a 0e 05 01 	*unknown*
    18cc:	3d 00 00 06 	*unknown*
    18d0:	77 16 00 00 	*unknown*
    18d4:	04 0b 05 01 	l.jal 2c2cd8 <_end+0x2be26c>
    18d8:	3e 00 00 06 	*unknown*
    18dc:	77 00 16 00 	*unknown*
    18e0:	00 04 38 05 	l.j 10f8f4 <_end+0x10ae88>
    18e4:	01 3f 00 00 	l.j 4fc18e4 <_end+0x4fbce78>
    18e8:	06 77 06 16 	l.jal f9dc3140 <_end+0xf9dbe6d4>
    18ec:	00 00 0e 6d 	l.j 52a0 <_end+0x834>
    18f0:	05 01 40 00 	l.jal 40518f0 <_end+0x404ce84>
    18f4:	00 00 5a 0c 	l.j 18124 <_end+0x136b8>
    18f8:	00 08 00 00 	l.j 2018f8 <_end+0x1fce8c>
    18fc:	00 5a 00 00 	l.j 16818fc <_end+0x167ce90>
    1900:	06 87 09 00 	l.jal fa1c3d00 <_end+0xfa1bf294>
    1904:	00 00 d2 02 	l.j 3610c <_end+0x316a0>
    1908:	00 1a cc 05 	l.j 6b491c <_end+0x6afeb0>
    190c:	02 58 00 00 	l.j f960190c <_end+0xf95fcea0>
    1910:	07 88 16 00 	l.jal fe207110 <_end+0xfe2026a4>
    1914:	00 04 93 05 	l.j 126528 <_end+0x121abc>
    1918:	02 5a 00 00 	l.j f9681918 <_end+0xf967ceac>
    191c:	00 9c 00 16 	l.j 2701974 <_end+0x26fcf08>
    1920:	00 00 04 3e 	l.j 2a18 <_unlink_r+0xc>
    1924:	05 02 5b 00 	l.jal 4098524 <_end+0x4093ab8>
    1928:	00 05 55 04 	l.j 156d38 <_end+0x1522cc>
    192c:	16 00 00 02 	*unknown*
    1930:	fc 05 02 5c 	*unknown*
    1934:	00 00 07 88 	l.j 3754 <memset+0x10>
    1938:	08 16 00 00 	*unknown*
    193c:	04 eb 05 02 	l.jal 3ac2d44 <_end+0x3abe2d8>
    1940:	5d 00 00 01 	*unknown*
    1944:	86 24 16 00 	l.lwz r17,5632(r4)
    1948:	00 02 60 05 	l.j 9995c <_end+0x94ef0>
    194c:	02 5e 00 00 	l.j f978194c <_end+0xf977cee0>
    1950:	00 3e 48 16 	l.j f939a8 <_end+0xf8ef3c>
    1954:	00 00 03 c7 	l.j 2870 <_execve_r+0x1c>
    1958:	05 02 5f 00 	l.jal 4099558 <_end+0x4094aec>
    195c:	00 00 68 4c 	l.j 1ba8c <_end+0x17020>
    1960:	16 00 00 05 	*unknown*
    1964:	01 05 02 60 	l.j 41422e4 <_end+0x413d878>
    1968:	00 00 06 42 	l.j 3270 <_or1k_libc_getreent+0x14>
    196c:	54 16 00 00 	*unknown*
    1970:	03 d2 05 02 	l.j ff482d78 <_end+0xff47e30c>
    1974:	61 00 00 00 	*unknown*
    1978:	fa 64 16 00 	*unknown*
    197c:	00 05 06 05 	l.j 143190 <_end+0x13e724>
    1980:	02 62 00 00 	l.j f9881980 <_end+0xf987cf14>
    1984:	00 fa 6c 16 	l.j 3e9c9dc <_end+0x3e97f70>
    1988:	00 00 01 a5 	l.j 201c <_init+0x18>
    198c:	05 02 63 00 	l.jal 409a58c <_end+0x4095b20>
    1990:	00 00 fa 74 	l.j 40360 <_end+0x3b8f4>
    1994:	16 00 00 04 	*unknown*
    1998:	be 05 02 64 	*unknown*
    199c:	00 00 07 98 	l.j 37fc <memset+0xb8>
    19a0:	7c 16 00 00 	*unknown*
    19a4:	02 f0 05 02 	l.j fbc02dac <_end+0xfbbfe340>
    19a8:	65 00 00 07 	*unknown*
    19ac:	a8 84 16 00 	l.ori r4,r4,0x1600
    19b0:	00 04 5c 05 	l.j 1189c4 <_end+0x113f58>
    19b4:	02 66 00 00 	l.j f99819b4 <_end+0xf997cf48>
    19b8:	00 3e 9c 16 	l.j fa8a10 <_end+0xfa3fa4>
    19bc:	00 00 02 26 	l.j 2254 <__do_global_dtors_aux+0x4c>
    19c0:	05 02 67 00 	l.jal 409b5c0 <_end+0x4096b54>
    19c4:	00 00 fa a0 	l.j 40444 <_end+0x3b9d8>
    19c8:	16 00 00 01 	*unknown*
    19cc:	96 05 02 68 	l.lhz r16,616(r5)
    19d0:	00 00 00 fa 	l.j 1db8 <_or1k_reset+0x1cb8>
    19d4:	a8 16 00 00 	l.ori r0,r22,0x0
    19d8:	02 15 05 02 	l.j f8542de0 <_end+0xf853e374>
    19dc:	69 00 00 00 	*unknown*
    19e0:	fa b0 16 00 	*unknown*
    19e4:	00 01 6d 05 	l.j 5cdf8 <_end+0x5838c>
    19e8:	02 6a 00 00 	l.j f9a819e8 <_end+0xf9a7cf7c>
    19ec:	00 fa b8 16 	l.j 3eafa44 <_end+0x3eaafd8>
    19f0:	00 00 01 7c 	l.j 1fe0 <_or1k_reset+0x1ee0>
    19f4:	05 02 6b 00 	l.jal 409c5f4 <_end+0x4097b88>
    19f8:	00 00 fa c0 	l.j 404f8 <_end+0x3ba8c>
    19fc:	16 00 00 03 	*unknown*
    1a00:	84 05 02 6c 	l.lwz r0,620(r5)
    1a04:	00 00 00 3e 	l.j 1afc <_or1k_reset+0x19fc>
    1a08:	c8 00 08 00 	lf.add.s r0,r0,r1
    1a0c:	00 05 5b 00 	l.j 15860c <_end+0x153ba0>
    1a10:	00 07 98 09 	l.j 1e7a34 <_end+0x1e2fc8>
    1a14:	00 00 00 d2 	l.j 1d5c <_or1k_reset+0x1c5c>
    1a18:	19 00 08 00 	l.movhi r8,0x800
    1a1c:	00 05 5b 00 	l.j 15861c <_end+0x153bb0>
    1a20:	00 07 a8 09 	l.j 1eba44 <_end+0x1e6fd8>
    1a24:	00 00 00 d2 	l.j 1d6c <_or1k_reset+0x1c6c>
    1a28:	07 00 08 00 	l.jal fc003a28 <_end+0xfbffefbc>
    1a2c:	00 05 5b 00 	l.j 15862c <_end+0x153bc0>
    1a30:	00 07 b8 09 	l.j 1efa54 <_end+0x1eafe8>
    1a34:	00 00 00 d2 	l.j 1d7c <_or1k_reset+0x1c7c>
    1a38:	17 00 1a f0 	*unknown*
    1a3c:	05 02 71 00 	l.jal 409de3c <_end+0x40993d0>
    1a40:	00 07 dc 16 	l.j 1f8a98 <_end+0x1f402c>
    1a44:	00 00 03 30 	l.j 2704 <__call_exitprocs+0x168>
    1a48:	05 02 74 00 	l.jal 409ea48 <_end+0x4099fdc>
    1a4c:	00 07 dc 00 	l.j 1f8a4c <_end+0x1f3fe0>
    1a50:	16 00 00 04 	*unknown*
    1a54:	b5 05 02 75 	l.mfspr r8,r5,0x275
    1a58:	00 00 07 ec 	l.j 3a08 <call___do_global_ctors_aux+0xc>
    1a5c:	78 00 08 00 	*unknown*
    1a60:	00 02 cf 00 	l.j b5660 <_end+0xb0bf4>
    1a64:	00 07 ec 09 	l.j 1fca88 <_end+0x1f801c>
    1a68:	00 00 00 d2 	l.j 1db0 <_or1k_reset+0x1cb0>
    1a6c:	1d 00 08 00 	*unknown*
    1a70:	00 00 9c 00 	l.j 28a70 <_end+0x24004>
    1a74:	00 07 fc 09 	l.j 200a98 <_end+0x1fc02c>
    1a78:	00 00 00 d2 	l.j 1dc0 <_or1k_reset+0x1cc0>
    1a7c:	1d 00 1b f0 	*unknown*
    1a80:	05 02 56 00 	l.jal 4097280 <_end+0x4092814>
    1a84:	00 08 1e 1c 	l.j 2092f4 <_end+0x204888>
    1a88:	00 00 0c 0b 	l.j 4ab4 <_end+0x48>
    1a8c:	05 02 6d 00 	l.jal 409ce8c <_end+0x4098420>
    1a90:	00 06 87 1c 	l.j 1a3700 <_end+0x19ec94>
    1a94:	00 00 04 d8 	l.j 2df4 <_or1k_exception_handler+0x4c>
    1a98:	05 02 76 00 	l.jal 409f298 <_end+0x409a82c>
    1a9c:	00 07 b8 00 	l.j 1efa9c <_end+0x1eb030>
    1aa0:	08 00 00 05 	*unknown*
    1aa4:	5b 00 00 08 	*unknown*
    1aa8:	2e 09 00 00 	*unknown*
    1aac:	00 d2 18 00 	l.j 3487aac <_end+0x3483040>
    1ab0:	1d 00 00 08 	*unknown*
    1ab4:	39 14 00 00 	*unknown*
    1ab8:	04 1d 00 0f 	l.jal 741af4 <_end+0x73d088>
    1abc:	04 00 00 08 	l.jal 1adc <_or1k_reset+0x19dc>
    1ac0:	2e 0f 04 00 	*unknown*
    1ac4:	00 01 70 1d 	l.j 5db38 <_end+0x590cc>
    1ac8:	00 00 08 50 	l.j 3c08 <_global_impure_ptr+0x1d4>
    1acc:	14 00 00 00 	*unknown*
    1ad0:	3e 00 0f 04 	*unknown*
    1ad4:	00 00 08 56 	l.j 3c2c <_global_impure_ptr+0x1f8>
    1ad8:	0f 04 00 00 	l.bnf fc101ad8 <_end+0xfc0fd06c>
    1adc:	08 45 08 00 	*unknown*
    1ae0:	00 05 f5 00 	l.j 17eee0 <_end+0x17a474>
    1ae4:	00 08 6c 09 	l.j 21cb08 <_end+0x21809c>
    1ae8:	00 00 00 d2 	l.j 1e30 <_or1k_reset+0x1d30>
    1aec:	02 00 1e 00 	l.j f80092ec <_end+0xf8004880>
    1af0:	00 00 21 04 	l.j 9f00 <_end+0x5494>
    1af4:	06 05 00 00 	l.jal f8141af4 <_end+0xf813d088>
    1af8:	08 8b 1f 00 	*unknown*
    1afc:	00 01 39 00 	l.j 4fefc <_end+0x4b490>
    1b00:	1f 00 00 00 	*unknown*
    1b04:	e2 01 1f 00 	*unknown*
    1b08:	00 00 70 02 	l.j 1db10 <_end+0x190a4>
    1b0c:	00 20 00 00 	l.j 801b0c <_end+0x7fd0a0>
    1b10:	00 87 01 42 	l.j 21c2018 <_end+0x21bd5ac>
    1b14:	00 00 00 3e 	l.j 1c0c <_or1k_reset+0x1b0c>
    1b18:	00 00 24 5c 	l.j ac88 <_end+0x621c>
    1b1c:	00 00 01 40 	l.j 201c <_init+0x18>
    1b20:	01 9c 00 00 	l.j 6701b20 <_end+0x66fd0b4>
    1b24:	09 0b 21 00 	*unknown*
    1b28:	00 01 49 01 	l.j 53f2c <_end+0x4f4c0>
    1b2c:	42 00 00 00 	*unknown*
    1b30:	3e 00 00 00 	*unknown*
    1b34:	4a 22 66 6e 	*unknown*
    1b38:	00 01 42 00 	l.j 52338 <_end+0x4d8cc>
    1b3c:	00 02 a3 00 	l.j aa73c <_end+0xa5cd0>
    1b40:	00 00 97 22 	l.j 277c8 <_end+0x22d5c>
    1b44:	61 72 67 00 	*unknown*
    1b48:	01 42 00 00 	l.j 5081b48 <_end+0x507d0dc>
    1b4c:	01 10 00 00 	l.j 4401b4c <_end+0x43fd0e0>
    1b50:	00 ce 22 64 	l.j 338a4e0 <_end+0x3385a74>
    1b54:	00 01 42 00 	l.j 52354 <_end+0x4d8e8>
    1b58:	00 01 10 00 	l.j 45b58 <_end+0x410ec>
    1b5c:	00 01 26 23 	l.j 4b3e8 <_end+0x4697c>
    1b60:	00 00 01 da 	l.j 22c8 <__do_global_dtors_aux+0xc0>
    1b64:	01 49 00 00 	l.j 5241b64 <_end+0x523d0f8>
    1b68:	09 0b 00 00 	*unknown*
    1b6c:	01 7e 24 70 	l.j 5f8ad2c <_end+0x5f862c0>
    1b70:	00 01 4a 00 	l.j 54370 <_end+0x4f904>
    1b74:	00 02 8d 00 	l.j a4f74 <_end+0xa0508>
    1b78:	00 01 94 25 	l.j 66c0c <_end+0x621a0>
    1b7c:	00 00 25 14 	l.j afcc <_end+0x6560>
    1b80:	00 00 09 22 	l.j 4008 <__EH_FRAME_BEGIN__+0x8>
    1b84:	26 01 53 03 	*unknown*
    1b88:	0a 01 90 00 	*unknown*
    1b8c:	00 0f 04 00 	l.j 3c2b8c <_end+0x3be120>
    1b90:	00 01 ff 27 	l.j 8182c <_end+0x7cdc0>
    1b94:	00 00 04 69 	l.j 2d38 <or1k_icache_disable>
    1b98:	05 02 fb 00 	l.jal 40c0798 <_end+0x40bbd2c>
    1b9c:	00 09 1d 18 	l.j 248ffc <_end+0x244590>
    1ba0:	00 00 04 1d 	l.j 2c14 <_or1k_cache_init+0xc>
    1ba4:	28 00 00 04 	*unknown*
    1ba8:	b7 07 61 00 	l.mfspr r24,r7,0x6100
    1bac:	00 01 10 14 	l.j 45bfc <_end+0x41190>
    1bb0:	00 00 00 2c 	l.j 1c60 <_or1k_reset+0x1b60>
    1bb4:	00 00 00 00 	l.j 1bb4 <_or1k_reset+0x1ab4>
    1bb8:	09 55 00 04 	*unknown*
    1bbc:	00 00 06 2a 	l.j 3464 <or1k_timer_init+0x90>
    1bc0:	04 01 00 00 	l.jal 41bc0 <_end+0x3d154>
    1bc4:	00 30 01 00 	l.j c01fc4 <_end+0xbfd558>
    1bc8:	00 05 ff 00 	l.j 1817c8 <_end+0x17cd5c>
    1bcc:	00 00 9b 00 	l.j 287cc <_end+0x23d60>
    1bd0:	00 25 9c 00 	l.j 968bd0 <_end+0x964164>
    1bd4:	00 01 e8 00 	l.j 7bbd4 <_end+0x77168>
    1bd8:	00 05 21 02 	l.j 149fe0 <_end+0x145574>
    1bdc:	04 07 00 00 	l.jal 1c1bdc <_end+0x1bd170>
    1be0:	00 5e 03 04 	l.j 17827f0 <_end+0x177dd84>
    1be4:	05 69 6e 74 	l.jal 5a5d5b4 <_end+0x5a58b48>
    1be8:	00 02 04 05 	l.j 82bfc <_end+0x7e190>
    1bec:	00 00 00 05 	l.j 1c00 <_or1k_reset+0x1b00>
    1bf0:	02 01 06 00 	l.j f80433f0 <_end+0xf803e984>
    1bf4:	00 00 7b 02 	l.j 207fc <_end+0x1bd90>
    1bf8:	01 08 00 00 	l.j 4201bf8 <_end+0x41fd18c>
    1bfc:	00 79 02 02 	l.j 1e42404 <_end+0x1e3d998>
    1c00:	05 00 00 01 	l.jal 4001c04 <_end+0x3ffd198>
    1c04:	2f 02 02 07 	*unknown*
    1c08:	00 00 00 0e 	l.j 1c40 <_or1k_reset+0x1b40>
    1c0c:	02 08 05 00 	l.j f820300c <_end+0xf81fe5a0>
    1c10:	00 00 00 02 	l.j 1c18 <_or1k_reset+0x1b18>
    1c14:	08 07 00 00 	*unknown*
    1c18:	00 59 04 00 	l.j 1642c18 <_end+0x163e1ac>
    1c1c:	00 03 6d 02 	l.j dd024 <_end+0xd85b8>
    1c20:	07 00 00 00 	l.jal fc001c20 <_end+0xfbffd1b4>
    1c24:	2c 04 00 00 	*unknown*
    1c28:	03 4b 03 10 	l.j fd2c2868 <_end+0xfd2bddfc>
    1c2c:	00 00 00 33 	l.j 1cf8 <_or1k_reset+0x1bf8>
    1c30:	04 00 00 04 	l.jal 1c40 <_or1k_reset+0x1b40>
    1c34:	27 03 27 00 	*unknown*
    1c38:	00 00 33 05 	l.j e84c <_end+0x9de0>
    1c3c:	00 00 03 17 	l.j 2898 <_fork_r+0x24>
    1c40:	04 01 61 00 	l.jal 5a040 <_end+0x555d4>
    1c44:	00 00 91 02 	l.j 2604c <_end+0x215e0>
    1c48:	04 07 00 00 	l.jal 1c1c48 <_end+0x1bd1dc>
    1c4c:	00 63 06 04 	l.j 18c345c <_end+0x18be9f0>
    1c50:	03 4a 00 00 	l.j fd281c50 <_end+0xfd27d1e4>
    1c54:	00 b7 07 00 	l.j 2dc3854 <_end+0x2dbede8>
    1c58:	00 03 11 03 	l.j c6064 <_end+0xc15f8>
    1c5c:	4c 00 00 00 	l.maci r0,0
    1c60:	85 07 00 00 	l.lwz r8,0(r7)
    1c64:	02 84 03 4d 	l.j fa102998 <_end+0xfa0fdf2c>
    1c68:	00 00 00 b7 	l.j 1f44 <_or1k_reset+0x1e44>
    1c6c:	00 08 00 00 	l.j 201c6c <_end+0x1fd200>
    1c70:	00 41 00 00 	l.j 1041c70 <_end+0x103d204>
    1c74:	00 c7 09 00 	l.j 31c4074 <_end+0x31bf608>
    1c78:	00 00 c7 03 	l.j 33884 <_end+0x2ee18>
    1c7c:	00 02 04 07 	l.j 82c98 <_end+0x7e22c>
    1c80:	00 00 01 45 	l.j 2194 <deregister_tm_clones+0x54>
    1c84:	0a 08 03 47 	*unknown*
    1c88:	00 00 00 ef 	l.j 2044 <_or1k_start+0x14>
    1c8c:	0b 00 00 04 	*unknown*
    1c90:	11 03 49 00 	l.bf 40d4090 <_end+0x40cf624>
    1c94:	00 00 2c 00 	l.j cc94 <_end+0x8228>
    1c98:	0b 00 00 04 	*unknown*
    1c9c:	19 03 4e 00 	*unknown*
    1ca0:	00 00 98 04 	l.j 27cb0 <_end+0x23244>
    1ca4:	00 04 00 00 	l.j 101ca4 <_end+0xfd238>
    1ca8:	03 ab 03 4f 	l.j feac29e4 <_end+0xfeabdf78>
    1cac:	00 00 00 ce 	l.j 1fe4 <_or1k_reset+0x1ee4>
    1cb0:	04 00 00 02 	l.jal 1cb8 <_or1k_reset+0x1bb8>
    1cb4:	47 03 53 00 	*unknown*
    1cb8:	00 00 64 0c 	l.j 1ace8 <_end+0x1627c>
    1cbc:	04 04 00 00 	l.jal 101cbc <_end+0xfd250>
    1cc0:	04 54 05 16 	l.jal 1503118 <_end+0x14fe6ac>
    1cc4:	00 00 00 25 	l.j 1d58 <_or1k_reset+0x1c58>
    1cc8:	0d 00 00 02 	l.bnf 4001cd0 <_end+0x3ffd264>
    1ccc:	58 18 05 2d 	*unknown*
    1cd0:	00 00 01 65 	l.j 2264 <__do_global_dtors_aux+0x5c>
    1cd4:	0b 00 00 03 	*unknown*
    1cd8:	cc 05 2f 00 	l.swa 1792(r5),r5
    1cdc:	00 01 65 00 	l.j 5b0dc <_end+0x56670>
    1ce0:	0e 5f 6b 00 	l.bnf f97dc8e0 <_end+0xf97d7e74>
    1ce4:	05 30 00 00 	l.jal 4c01ce4 <_end+0x4bfd278>
    1ce8:	00 2c 04 0b 	l.j b02d14 <_end+0xafe2a8>
    1cec:	00 00 04 03 	l.j 2cf8 <_or1k_cache_init+0xf0>
    1cf0:	05 30 00 00 	l.jal 4c01cf0 <_end+0x4bfd284>
    1cf4:	00 2c 08 0b 	l.j b03d20 <_end+0xaff2b4>
    1cf8:	00 00 02 41 	l.j 25fc <__call_exitprocs+0x60>
    1cfc:	05 30 00 00 	l.jal 4c01cfc <_end+0x4bfd290>
    1d00:	00 2c 0c 0b 	l.j b04d2c <_end+0xb002c0>
    1d04:	00 00 04 a0 	l.j 2f84 <or1k_interrupts_enable+0xc>
    1d08:	05 30 00 00 	l.jal 4c01d08 <_end+0x4bfd29c>
    1d0c:	00 2c 10 0e 	l.j b05d44 <_end+0xb012d8>
    1d10:	5f 78 00 05 	*unknown*
    1d14:	31 00 00 01 	*unknown*
    1d18:	6b 14 00 0f 	*unknown*
    1d1c:	04 00 00 01 	l.jal 1d20 <_or1k_reset+0x1c20>
    1d20:	12 08 00 00 	l.bf f8201d20 <_end+0xf81fd2b4>
    1d24:	01 07 00 00 	l.j 41c1d24 <_end+0x41bd2b8>
    1d28:	01 7b 09 00 	l.j 5ec4128 <_end+0x5ebf6bc>
    1d2c:	00 00 c7 00 	l.j 3392c <_end+0x2eec0>
    1d30:	00 0d 00 00 	l.j 341d30 <_end+0x33d2c4>
    1d34:	02 7f 24 05 	l.j f9fcad48 <_end+0xf9fc62dc>
    1d38:	35 00 00 01 	*unknown*
    1d3c:	f4 0b 00 00 	*unknown*
    1d40:	01 b3 05 37 	l.j 6cc321c <_end+0x6cbe7b0>
    1d44:	00 00 00 2c 	l.j 1df4 <_or1k_reset+0x1cf4>
    1d48:	00 0b 00 00 	l.j 2c1d48 <_end+0x2bd2dc>
    1d4c:	04 2f 05 38 	l.jal bc322c <_end+0xbbe7c0>
    1d50:	00 00 00 2c 	l.j 1e00 <_or1k_reset+0x1d00>
    1d54:	04 0b 00 00 	l.jal 2c1d54 <_end+0x2bd2e8>
    1d58:	01 c2 05 39 	l.j 708323c <_end+0x707e7d0>
    1d5c:	00 00 00 2c 	l.j 1e0c <_or1k_reset+0x1d0c>
    1d60:	08 0b 00 00 	*unknown*
    1d64:	05 19 05 3a 	l.jal 464324c <_end+0x463e7e0>
    1d68:	00 00 00 2c 	l.j 1e18 <_or1k_reset+0x1d18>
    1d6c:	0c 0b 00 00 	l.bnf 2c1d6c <_end+0x2bd300>
    1d70:	03 37 05 3b 	l.j fcdc325c <_end+0xfcdbe7f0>
    1d74:	00 00 00 2c 	l.j 1e24 <_or1k_reset+0x1d24>
    1d78:	10 0b 00 00 	l.bf 2c1d78 <_end+0x2bd30c>
    1d7c:	03 26 05 3c 	l.j fc98326c <_end+0xfc97e800>
    1d80:	00 00 00 2c 	l.j 1e30 <_or1k_reset+0x1d30>
    1d84:	14 0b 00 00 	*unknown*
    1d88:	04 a5 05 3d 	l.jal 294327c <_end+0x293e810>
    1d8c:	00 00 00 2c 	l.j 1e3c <_or1k_reset+0x1d3c>
    1d90:	18 0b 00 00 	*unknown*
    1d94:	03 8d 05 3e 	l.j fe34328c <_end+0xfe33e820>
    1d98:	00 00 00 2c 	l.j 1e48 <_or1k_reset+0x1d48>
    1d9c:	1c 0b 00 00 	*unknown*
    1da0:	04 e0 05 3f 	l.jal 380329c <_end+0x37fe830>
    1da4:	00 00 00 2c 	l.j 1e54 <_or1k_reset+0x1d54>
    1da8:	20 00 10 00 	l.sys 0x1000
    1dac:	00 01 d1 01 	l.j 761b0 <_end+0x71744>
    1db0:	08 05 48 00 	*unknown*
    1db4:	00 02 34 0b 	l.j 8ede0 <_end+0x8a374>
    1db8:	00 00 02 34 	l.j 2688 <__call_exitprocs+0xec>
    1dbc:	05 49 00 00 	l.jal 5241dbc <_end+0x523d350>
    1dc0:	02 34 00 0b 	l.j f8d01dec <_end+0xf8cfd380>
    1dc4:	00 00 01 4e 	l.j 22fc <frame_dummy>
    1dc8:	05 4a 00 00 	l.jal 5281dc8 <_end+0x527d35c>
    1dcc:	02 34 80 11 	l.j f8d21e10 <_end+0xf8d1d3a4>
    1dd0:	00 00 04 4b 	l.j 2efc <_or1k_exception_handler+0x154>
    1dd4:	05 4c 00 00 	l.jal 5301dd4 <_end+0x52fd368>
    1dd8:	01 07 01 00 	l.j 41c21d8 <_end+0x41bd76c>
    1ddc:	11 00 00 01 	l.bf 4001de0 <_end+0x3ffd374>
    1de0:	f6 05 4f 00 	*unknown*
    1de4:	00 01 07 01 	l.j 439e8 <_end+0x3ef7c>
    1de8:	04 00 08 00 	l.jal 3de8 <_global_impure_ptr+0x3b4>
    1dec:	00 01 05 00 	l.j 431ec <_end+0x3e780>
    1df0:	00 02 44 09 	l.j 92e14 <_end+0x8e3a8>
    1df4:	00 00 00 c7 	l.j 2110 <_or1k_start+0xe0>
    1df8:	1f 00 10 00 	*unknown*
    1dfc:	00 01 3d 01 	l.j 51200 <_end+0x4c794>
    1e00:	90 05 5b 00 	l.lbs r0,23296(r5)
    1e04:	00 02 82 0b 	l.j a2630 <_end+0x9dbc4>
    1e08:	00 00 03 cc 	l.j 2d38 <or1k_icache_disable>
    1e0c:	05 5c 00 00 	l.jal 5701e0c <_end+0x56fd3a0>
    1e10:	02 82 00 0b 	l.j fa081e3c <_end+0xfa07d3d0>
    1e14:	00 00 03 e4 	l.j 2da4 <or1k_dcache_flush+0x4>
    1e18:	05 5d 00 00 	l.jal 5741e18 <_end+0x573d3ac>
    1e1c:	00 2c 04 0b 	l.j b02e48 <_end+0xafe3dc>
    1e20:	00 00 02 3c 	l.j 2710 <__call_exitprocs+0x174>
    1e24:	05 5f 00 00 	l.jal 57c1e24 <_end+0x57bd3b8>
    1e28:	02 88 08 0b 	l.j fa203e54 <_end+0xfa1ff3e8>
    1e2c:	00 00 01 d1 	l.j 2570 <__register_exitproc+0x114>
    1e30:	05 60 00 00 	l.jal 5801e30 <_end+0x57fd3c4>
    1e34:	01 f4 88 00 	l.j 7d23e34 <_end+0x7d1f3c8>
    1e38:	0f 04 00 00 	l.bnf fc101e38 <_end+0xfc0fd3cc>
    1e3c:	02 44 08 00 	l.j f9103e3c <_end+0xf90ff3d0>
    1e40:	00 02 98 00 	l.j a7e40 <_end+0xa33d4>
    1e44:	00 02 98 09 	l.j a7e68 <_end+0xa33fc>
    1e48:	00 00 00 c7 	l.j 2164 <deregister_tm_clones+0x24>
    1e4c:	1f 00 0f 04 	*unknown*
    1e50:	00 00 02 9e 	l.j 28c8 <_getpid_r+0x8>
    1e54:	12 0d 00 00 	l.bf f8341e54 <_end+0xf833d3e8>
    1e58:	03 97 08 05 	l.j fe5c3e6c <_end+0xfe5bf400>
    1e5c:	73 00 00 02 	*unknown*
    1e60:	c4 0b 00 00 	*unknown*
    1e64:	09 f8 05 74 	*unknown*
    1e68:	00 00 02 c4 	l.j 2978 <_lseek_r+0x18>
    1e6c:	00 0b 00 00 	l.j 2c1e6c <_end+0x2bd400>
    1e70:	07 8b 05 75 	l.jal fe2c3444 <_end+0xfe2be9d8>
    1e74:	00 00 00 2c 	l.j 1f24 <_or1k_reset+0x1e24>
    1e78:	04 00 0f 04 	l.jal 5a88 <_end+0x101c>
    1e7c:	00 00 00 41 	l.j 1f80 <_or1k_reset+0x1e80>
    1e80:	0d 00 00 03 	l.bnf 4001e8c <_end+0x3ffd420>
    1e84:	b6 68 05 b3 	l.mfspr r19,r8,0x5b3
    1e88:	00 00 03 f4 	l.j 2e58 <_or1k_exception_handler+0xb0>
    1e8c:	0e 5f 70 00 	l.bnf f97dde8c <_end+0xf97d9420>
    1e90:	05 b4 00 00 	l.jal 6d01e90 <_end+0x6cfd424>
    1e94:	02 c4 00 0e 	l.j fb101ecc <_end+0xfb0fd460>
    1e98:	5f 72 00 05 	*unknown*
    1e9c:	b5 00 00 00 	l.mfspr r8,r0,0x0
    1ea0:	2c 04 0e 5f 	*unknown*
    1ea4:	77 00 05 b6 	*unknown*
    1ea8:	00 00 00 2c 	l.j 1f58 <_or1k_reset+0x1e58>
    1eac:	08 0b 00 00 	*unknown*
    1eb0:	01 ef 05 b7 	l.j 7bc358c <_end+0x7bbeb20>
    1eb4:	00 00 00 48 	l.j 1fd4 <_or1k_reset+0x1ed4>
    1eb8:	0c 0b 00 00 	l.bnf 2c1eb8 <_end+0x2bd44c>
    1ebc:	02 9b 05 b8 	l.j fa6c359c <_end+0xfa6beb30>
    1ec0:	00 00 00 48 	l.j 1fe0 <_or1k_reset+0x1ee0>
    1ec4:	0e 0e 5f 62 	l.bnf f8399c4c <_end+0xf83951e0>
    1ec8:	66 00 05 b9 	*unknown*
    1ecc:	00 00 02 9f 	l.j 2948 <_link_r+0x8>
    1ed0:	10 0b 00 00 	l.bf 2c1ed0 <_end+0x2bd464>
    1ed4:	01 8d 05 ba 	l.j 63435bc <_end+0x633eb50>
    1ed8:	00 00 00 2c 	l.j 1f88 <_or1k_reset+0x1e88>
    1edc:	18 0b 00 00 	*unknown*
    1ee0:	01 df 05 c1 	l.j 77c35e4 <_end+0x77beb78>
    1ee4:	00 00 01 05 	l.j 22f8 <call___do_global_dtors_aux+0x18>
    1ee8:	1c 0b 00 00 	*unknown*
    1eec:	02 6f 05 c3 	l.j f9bc35f8 <_end+0xf9bbeb8c>
    1ef0:	00 00 05 57 	l.j 344c <or1k_timer_init+0x78>
    1ef4:	20 0b 00 00 	*unknown*
    1ef8:	09 a6 05 c5 	*unknown*
    1efc:	00 00 05 86 	l.j 3514 <or1k_timer_set_handler+0x24>
    1f00:	24 0b 00 00 	*unknown*
    1f04:	04 21 05 c8 	l.jal 843624 <_end+0x83ebb8>
    1f08:	00 00 05 aa 	l.j 35b0 <or1k_timer_enable+0x44>
    1f0c:	28 0b 00 00 	*unknown*
    1f10:	04 fa 05 c9 	l.jal 3e83634 <_end+0x3e7ebc8>
    1f14:	00 00 05 c4 	l.j 3624 <or1k_timer_restore+0x20>
    1f18:	2c 0e 5f 75 	*unknown*
    1f1c:	62 00 05 cc 	*unknown*
    1f20:	00 00 02 9f 	l.j 299c <_open+0x10>
    1f24:	30 0e 5f 75 	*unknown*
    1f28:	70 00 05 cd 	*unknown*
    1f2c:	00 00 02 c4 	l.j 2a3c <_or1k_uart_interrupt_handler+0x10>
    1f30:	38 0e 5f 75 	*unknown*
    1f34:	72 00 05 ce 	*unknown*
    1f38:	00 00 00 2c 	l.j 1fe8 <_or1k_reset+0x1ee8>
    1f3c:	3c 0b 00 00 	*unknown*
    1f40:	01 bc 05 d1 	l.j 6f03684 <_end+0x6efec18>
    1f44:	00 00 05 ca 	l.j 366c <or1k_timer_reset+0x10>
    1f48:	40 0b 00 00 	*unknown*
    1f4c:	04 d2 05 d2 	l.jal 3483694 <_end+0x347ec28>
    1f50:	00 00 05 da 	l.j 36b8 <or1k_timer_get_ticks+0x1c>
    1f54:	43 0e 5f 6c 	*unknown*
    1f58:	62 00 05 d5 	*unknown*
    1f5c:	00 00 02 9f 	l.j 29d8 <_readlink_r+0xc>
    1f60:	44 0b 00 00 	*unknown*
    1f64:	08 18 05 d8 	*unknown*
    1f68:	00 00 00 2c 	l.j 2018 <_init+0x14>
    1f6c:	4c 0b 00 00 	l.maci r11,0
    1f70:	02 0d 05 d9 	l.j f83436d4 <_end+0xf833ec68>
    1f74:	00 00 00 6f 	l.j 2130 <_or1k_start+0x100>
    1f78:	50 0b 00 00 	*unknown*
    1f7c:	0c 6a 05 dc 	l.bnf 1a836ec <_end+0x1a7ec80>
    1f80:	00 00 04 12 	l.j 2fc8 <or1k_interrupts_disable+0x2c>
    1f84:	54 0b 00 00 	*unknown*
    1f88:	06 4f 05 e0 	l.jal f93c3708 <_end+0xf93bec9c>
    1f8c:	00 00 00 fa 	l.j 2374 <frame_dummy+0x78>
    1f90:	58 0b 00 00 	*unknown*
    1f94:	03 be 05 e2 	l.j fef8371c <_end+0xfef7ecb0>
    1f98:	00 00 00 ef 	l.j 2354 <frame_dummy+0x58>
    1f9c:	5c 0b 00 00 	*unknown*
    1fa0:	03 1e 05 e3 	l.j fc78372c <_end+0xfc77ecc0>
    1fa4:	00 00 00 2c 	l.j 2054 <_or1k_start+0x24>
    1fa8:	64 00 13 00 	*unknown*
    1fac:	00 00 2c 00 	l.j cfac <_end+0x8540>
    1fb0:	00 04 12 14 	l.j 106800 <_end+0x101d94>
    1fb4:	00 00 04 12 	l.j 2ffc <or1k_interrupts_restore+0x28>
    1fb8:	14 00 00 01 	*unknown*
    1fbc:	05 14 00 00 	l.jal 4501fbc <_end+0x44fd550>
    1fc0:	05 4a 14 00 	l.jal 5286fc0 <_end+0x5282554>
    1fc4:	00 00 2c 00 	l.j cfc4 <_end+0x8558>
    1fc8:	0f 04 00 00 	l.bnf fc101fc8 <_end+0xfc0fd55c>
    1fcc:	04 18 15 00 	l.jal 6073cc <_end+0x602960>
    1fd0:	00 0c 0b 04 	l.j 304be0 <_end+0x300174>
    1fd4:	24 05 02 39 	*unknown*
    1fd8:	00 00 05 4a 	l.j 3500 <or1k_timer_set_handler+0x10>
    1fdc:	16 00 00 06 	*unknown*
    1fe0:	bd 05 02 3b 	*unknown*
    1fe4:	00 00 00 2c 	l.j 2094 <_or1k_start+0x64>
    1fe8:	00 16 00 00 	l.j 581fe8 <_end+0x57d57c>
    1fec:	01 fe 05 02 	l.j 7f833f4 <_end+0x7f7e988>
    1ff0:	40 00 00 06 	*unknown*
    1ff4:	31 04 16 00 	*unknown*
    1ff8:	00 02 8b 05 	l.j a4c0c <_end+0xa01a0>
    1ffc:	02 40 00 00 	l.j f9001ffc <_end+0xf8ffd590>
    2000:	06 31 08 16 	l.jal f8c44058 <_end+0xf8c3f5ec>
    2004:	00 00 02 50 	l.j 2944 <_link_r+0x4>
    2008:	05 02 40 00 	l.jal 4092008 <_end+0x408d59c>
    200c:	00 06 31 0c 	l.j 18e43c <_end+0x1899d0>
    2010:	16 00 00 03 	*unknown*
    2014:	df 05 02 42 	l.sh -15806(r5),r0
    2018:	00 00 00 2c 	l.j 20c8 <_or1k_start+0x98>
    201c:	10 16 00 00 	l.bf 58201c <_end+0x57d5b0>
    2020:	01 62 05 02 	l.j 5883428 <_end+0x587e9bc>
    2024:	43 00 00 08 	*unknown*
    2028:	13 14 16 00 	l.bf fc507828 <_end+0xfc502dbc>
    202c:	00 04 7c 05 	l.j 121040 <_end+0x11c5d4>
    2030:	02 45 00 00 	l.j f9142030 <_end+0xf913d5c4>
    2034:	00 2c 30 16 	l.j b0e08c <_end+0xb09620>
    2038:	00 00 03 e9 	l.j 2fdc <or1k_interrupts_restore+0x8>
    203c:	05 02 46 00 	l.jal 409383c <_end+0x408edd0>
    2040:	00 05 7b 34 	l.j 160d10 <_end+0x15c2a4>
    2044:	16 00 00 03 	*unknown*
    2048:	40 05 02 48 	*unknown*
    204c:	00 00 00 2c 	l.j 20fc <_or1k_start+0xcc>
    2050:	38 16 00 00 	*unknown*
    2054:	03 f9 05 02 	l.j ffe4345c <_end+0xffe3e9f0>
    2058:	4a 00 00 08 	*unknown*
    205c:	2e 3c 16 00 	*unknown*
    2060:	00 03 09 05 	l.j c4474 <_end+0xbfa08>
    2064:	02 4d 00 00 	l.j f9342064 <_end+0xf933d5f8>
    2068:	01 65 40 16 	l.j 59520c0 <_end+0x594d654>
    206c:	00 00 02 75 	l.j 2a40 <_or1k_uart_interrupt_handler+0x14>
    2070:	05 02 4e 00 	l.jal 4095870 <_end+0x4090e04>
    2074:	00 00 2c 44 	l.j d184 <_end+0x8718>
    2078:	16 00 00 05 	*unknown*
    207c:	14 05 02 4f 	*unknown*
    2080:	00 00 01 65 	l.j 2614 <__call_exitprocs+0x78>
    2084:	48 16 00 00 	*unknown*
    2088:	03 63 05 02 	l.j fd8c3490 <_end+0xfd8bea24>
    208c:	50 00 00 08 	*unknown*
    2090:	34 4c 16 00 	*unknown*
    2094:	00 02 93 05 	l.j a6ca8 <_end+0xa223c>
    2098:	02 53 00 00 	l.j f94c2098 <_end+0xf94bd62c>
    209c:	00 2c 50 16 	l.j b160f4 <_end+0xb11688>
    20a0:	00 00 02 05 	l.j 28b4 <_fstat_r+0x14>
    20a4:	05 02 54 00 	l.jal 40970a4 <_end+0x4092638>
    20a8:	00 05 4a 54 	l.j 1549f8 <_end+0x14ff8c>
    20ac:	16 00 00 03 	*unknown*
    20b0:	7f 05 02 77 	*unknown*
    20b4:	00 00 07 f1 	l.j 4078 <impure_data+0x24>
    20b8:	58 17 00 00 	*unknown*
    20bc:	01 3d 05 02 	l.j 4f434c4 <_end+0x4f3ea58>
    20c0:	7b 00 00 02 	*unknown*
    20c4:	82 01 48 17 	*unknown*
    20c8:	00 00 02 e7 	l.j 2c64 <_or1k_cache_init+0x5c>
    20cc:	05 02 7c 00 	l.jal 40a10cc <_end+0x409c660>
    20d0:	00 02 44 01 	l.j 930d4 <_end+0x8e668>
    20d4:	4c 17 00 00 	l.maci r23,0
    20d8:	04 c8 05 02 	l.jal 32034e0 <_end+0x31fea74>
    20dc:	80 00 00 08 	*unknown*
    20e0:	45 02 dc 17 	*unknown*
    20e4:	00 00 01 e7 	l.j 2880 <_fork_r+0xc>
    20e8:	05 02 85 00 	l.jal 40a34e8 <_end+0x409ea7c>
    20ec:	00 05 f6 02 	l.j 17f8f4 <_end+0x17ae88>
    20f0:	e0 17 00 00 	l.add r0,r23,r0
    20f4:	01 cc 05 02 	l.j 73034fc <_end+0x72fea90>
    20f8:	86 00 00 08 	l.lwz r16,8(r0)
    20fc:	51 02 ec 00 	*unknown*
    2100:	0f 04 00 00 	l.bnf fc102100 <_end+0xfc0fd694>
    2104:	05 50 02 01 	l.jal 5402908 <_end+0x53fde9c>
    2108:	06 00 00 00 	l.jal f8002108 <_end+0xf7ffd69c>
    210c:	82 0f 04 00 	*unknown*
    2110:	00 03 f4 13 	l.j ff15c <_end+0xfa6f0>
    2114:	00 00 00 2c 	l.j 21c4 <register_tm_clones+0x28>
    2118:	00 00 05 7b 	l.j 3704 <_or1k_board_exit+0x4>
    211c:	14 00 00 04 	*unknown*
    2120:	12 14 00 00 	l.bf f8502120 <_end+0xf84fd6b4>
    2124:	01 05 14 00 	l.j 4147124 <_end+0x41426b8>
    2128:	00 05 7b 14 	l.j 160d78 <_end+0x15c30c>
    212c:	00 00 00 2c 	l.j 21dc <register_tm_clones+0x40>
    2130:	00 0f 04 00 	l.j 3c3130 <_end+0x3be6c4>
    2134:	00 05 81 18 	l.j 162594 <_end+0x15db28>
    2138:	00 00 05 50 	l.j 3678 <or1k_timer_reset+0x1c>
    213c:	0f 04 00 00 	l.bnf fc10213c <_end+0xfc0fd6d0>
    2140:	05 5d 13 00 	l.jal 5746d40 <_end+0x57422d4>
    2144:	00 00 7a 00 	l.j 20944 <_end+0x1bed8>
    2148:	00 05 aa 14 	l.j 16c998 <_end+0x167f2c>
    214c:	00 00 04 12 	l.j 3194 <_or1k_libc_impure_init+0xb0>
    2150:	14 00 00 01 	*unknown*
    2154:	05 14 00 00 	l.jal 4502154 <_end+0x44fd6e8>
    2158:	00 7a 14 00 	l.j 1e87158 <_end+0x1e826ec>
    215c:	00 00 2c 00 	l.j d15c <_end+0x86f0>
    2160:	0f 04 00 00 	l.bnf fc102160 <_end+0xfc0fd6f4>
    2164:	05 8c 13 00 	l.jal 6306d64 <_end+0x63022f8>
    2168:	00 00 2c 00 	l.j d168 <_end+0x86fc>
    216c:	00 05 c4 14 	l.j 1731bc <_end+0x16e750>
    2170:	00 00 04 12 	l.j 31b8 <_or1k_libc_impure_init+0xd4>
    2174:	14 00 00 01 	*unknown*
    2178:	05 00 0f 04 	l.jal 4005d88 <_end+0x400131c>
    217c:	00 00 05 b0 	l.j 383c <memset+0xf8>
    2180:	08 00 00 00 	*unknown*
    2184:	41 00 00 05 	*unknown*
    2188:	da 09 00 00 	l.sb -32768(r9),r0
    218c:	00 c7 02 00 	l.j 31c298c <_end+0x31bdf20>
    2190:	08 00 00 00 	*unknown*
    2194:	41 00 00 05 	*unknown*
    2198:	ea 09 00 00 	*unknown*
    219c:	00 c7 00 00 	l.j 31c219c <_end+0x31bd730>
    21a0:	05 00 00 03 	l.jal 40021ac <_end+0x3ffd740>
    21a4:	a4 05 01 1d 	l.andi r0,r5,0x11d
    21a8:	00 00 02 ca 	l.j 2cd0 <_or1k_cache_init+0xc8>
    21ac:	19 00 00 04 	l.movhi r8,0x4
    21b0:	af 0c 05 01 	l.xori r24,r12,1281
    21b4:	21 00 00 06 	l.trap 0x6
    21b8:	2b 16 00 00 	*unknown*
    21bc:	03 cc 05 01 	l.j ff3035c0 <_end+0xff2feb54>
    21c0:	23 00 00 06 	*unknown*
    21c4:	2b 00 16 00 	*unknown*
    21c8:	00 02 a1 05 	l.j aa5dc <_end+0xa5b70>
    21cc:	01 24 00 00 	l.j 49021cc <_end+0x48fd760>
    21d0:	00 2c 04 16 	l.j b03228 <_end+0xafe7bc>
    21d4:	00 00 03 9e 	l.j 304c <_or1k_interrupt_handler+0x30>
    21d8:	05 01 25 00 	l.jal 404b5d8 <_end+0x4046b6c>
    21dc:	00 06 31 08 	l.j 18e5fc <_end+0x189b90>
    21e0:	00 0f 04 00 	l.j 3c31e0 <_end+0x3be774>
    21e4:	00 05 f6 0f 	l.j 17fa20 <_end+0x17afb4>
    21e8:	04 00 00 05 	l.jal 21fc <register_tm_clones+0x60>
    21ec:	ea 19 00 00 	*unknown*
    21f0:	01 5a 0e 05 	l.j 5685a04 <_end+0x5680f98>
    21f4:	01 3d 00 00 	l.j 4f421f4 <_end+0x4f3d788>
    21f8:	06 6c 16 00 	l.jal f9b079f8 <_end+0xf9b02f8c>
    21fc:	00 04 0b 05 	l.j 104e10 <_end+0x1003a4>
    2200:	01 3e 00 00 	l.j 4f82200 <_end+0x4f7d794>
    2204:	06 6c 00 16 	l.jal f9b0225c <_end+0xf9afd7f0>
    2208:	00 00 04 38 	l.j 32e8 <or1k_critical_begin+0xc>
    220c:	05 01 3f 00 	l.jal 4051e0c <_end+0x404d3a0>
    2210:	00 06 6c 06 	l.j 19d228 <_end+0x1987bc>
    2214:	16 00 00 0e 	*unknown*
    2218:	6d 05 01 40 	l.lwa r8,320(r5)
    221c:	00 00 00 4f 	l.j 2358 <frame_dummy+0x5c>
    2220:	0c 00 08 00 	l.bnf 4220 <impure_data+0x1cc>
    2224:	00 00 4f 00 	l.j 15e24 <_end+0x113b8>
    2228:	00 06 7c 09 	l.j 1a124c <_end+0x19c7e0>
    222c:	00 00 00 c7 	l.j 2548 <__register_exitproc+0xec>
    2230:	02 00 1a cc 	l.j f8008d60 <_end+0xf80042f4>
    2234:	05 02 58 00 	l.jal 4098234 <_end+0x40937c8>
    2238:	00 07 7d 16 	l.j 1e1690 <_end+0x1dcc24>
    223c:	00 00 04 93 	l.j 3488 <or1k_timer_set_period>
    2240:	05 02 5a 00 	l.jal 4098a40 <_end+0x4093fd4>
    2244:	00 00 91 00 	l.j 26644 <_end+0x21bd8>
    2248:	16 00 00 04 	*unknown*
    224c:	3e 05 02 5b 	*unknown*
    2250:	00 00 05 4a 	l.j 3778 <memset+0x34>
    2254:	04 16 00 00 	l.jal 582254 <_end+0x57d7e8>
    2258:	02 fc 05 02 	l.j fbf03660 <_end+0xfbefebf4>
    225c:	5c 00 00 07 	*unknown*
    2260:	7d 08 16 00 	*unknown*
    2264:	00 04 eb 05 	l.j 13ce78 <_end+0x13840c>
    2268:	02 5d 00 00 	l.j f9742268 <_end+0xf973d7fc>
    226c:	01 7b 24 16 	l.j 5ecb2c4 <_end+0x5ec6858>
    2270:	00 00 02 60 	l.j 2bf0 <_or1k_outbyte+0x10>
    2274:	05 02 5e 00 	l.jal 4099a74 <_end+0x4095008>
    2278:	00 00 2c 48 	l.j d398 <_end+0x892c>
    227c:	16 00 00 03 	*unknown*
    2280:	c7 05 02 5f 	*unknown*
    2284:	00 00 00 5d 	l.j 23f8 <atexit+0x14>
    2288:	4c 16 00 00 	l.maci r22,0
    228c:	05 01 05 02 	l.jal 4043694 <_end+0x403ec28>
    2290:	60 00 00 06 	*unknown*
    2294:	37 54 16 00 	*unknown*
    2298:	00 03 d2 05 	l.j f6aac <_end+0xf2040>
    229c:	02 61 00 00 	l.j f984229c <_end+0xf983d830>
    22a0:	00 ef 64 16 	l.j 3bdb2f8 <_end+0x3bd688c>
    22a4:	00 00 05 06 	l.j 36bc <or1k_timer_reset_ticks>
    22a8:	05 02 62 00 	l.jal 409aaa8 <_end+0x409603c>
    22ac:	00 00 ef 6c 	l.j 3e05c <_end+0x395f0>
    22b0:	16 00 00 01 	*unknown*
    22b4:	a5 05 02 63 	l.andi r8,r5,0x263
    22b8:	00 00 00 ef 	l.j 2674 <__call_exitprocs+0xd8>
    22bc:	74 16 00 00 	*unknown*
    22c0:	04 be 05 02 	l.jal 2f836c8 <_end+0x2f7ec5c>
    22c4:	64 00 00 07 	*unknown*
    22c8:	8d 7c 16 00 	l.lbz r11,5632(r28)
    22cc:	00 02 f0 05 	l.j be2e0 <_end+0xb9874>
    22d0:	02 65 00 00 	l.j f99422d0 <_end+0xf993d864>
    22d4:	07 9d 84 16 	l.jal fe76332c <_end+0xfe75e8c0>
    22d8:	00 00 04 5c 	l.j 3448 <or1k_timer_init+0x74>
    22dc:	05 02 66 00 	l.jal 409badc <_end+0x4097070>
    22e0:	00 00 2c 9c 	l.j d550 <_end+0x8ae4>
    22e4:	16 00 00 02 	*unknown*
    22e8:	26 05 02 67 	*unknown*
    22ec:	00 00 00 ef 	l.j 26a8 <__call_exitprocs+0x10c>
    22f0:	a0 16 00 00 	l.addic r0,r22,0
    22f4:	01 96 05 02 	l.j 65836fc <_end+0x657ec90>
    22f8:	68 00 00 00 	*unknown*
    22fc:	ef a8 16 00 	*unknown*
    2300:	00 02 15 05 	l.j 87714 <_end+0x82ca8>
    2304:	02 69 00 00 	l.j f9a42304 <_end+0xf9a3d898>
    2308:	00 ef b0 16 	l.j 3bee360 <_end+0x3be98f4>
    230c:	00 00 01 6d 	l.j 28c0 <_getpid_r>
    2310:	05 02 6a 00 	l.jal 409cb10 <_end+0x40980a4>
    2314:	00 00 ef b8 	l.j 3e1f4 <_end+0x39788>
    2318:	16 00 00 01 	*unknown*
    231c:	7c 05 02 6b 	*unknown*
    2320:	00 00 00 ef 	l.j 26dc <__call_exitprocs+0x140>
    2324:	c0 16 00 00 	l.mtspr r22,r0,0x0
    2328:	03 84 05 02 	l.j fe103730 <_end+0xfe0fecc4>
    232c:	6c 00 00 00 	l.lwa r0,0(r0)
    2330:	2c c8 00 08 	*unknown*
    2334:	00 00 05 50 	l.j 3874 <memset+0x130>
    2338:	00 00 07 8d 	l.j 416c <impure_data+0x118>
    233c:	09 00 00 00 	*unknown*
    2340:	c7 19 00 08 	*unknown*
    2344:	00 00 05 50 	l.j 3884 <memset+0x140>
    2348:	00 00 07 9d 	l.j 41bc <impure_data+0x168>
    234c:	09 00 00 00 	*unknown*
    2350:	c7 07 00 08 	*unknown*
    2354:	00 00 05 50 	l.j 3894 <memset+0x150>
    2358:	00 00 07 ad 	l.j 420c <impure_data+0x1b8>
    235c:	09 00 00 00 	*unknown*
    2360:	c7 17 00 1a 	*unknown*
    2364:	f0 05 02 71 	*unknown*
    2368:	00 00 07 d1 	l.j 42ac <impure_data+0x258>
    236c:	16 00 00 03 	*unknown*
    2370:	30 05 02 74 	*unknown*
    2374:	00 00 07 d1 	l.j 42b8 <impure_data+0x264>
    2378:	00 16 00 00 	l.j 582378 <_end+0x57d90c>
    237c:	04 b5 05 02 	l.jal 2d43784 <_end+0x2d3ed18>
    2380:	75 00 00 07 	*unknown*
    2384:	e1 78 00 08 	l.sll r11,r24,r0
    2388:	00 00 02 c4 	l.j 2e98 <_or1k_exception_handler+0xf0>
    238c:	00 00 07 e1 	l.j 4310 <impure_data+0x2bc>
    2390:	09 00 00 00 	*unknown*
    2394:	c7 1d 00 08 	*unknown*
    2398:	00 00 00 91 	l.j 25dc <__call_exitprocs+0x40>
    239c:	00 00 07 f1 	l.j 4360 <impure_data+0x30c>
    23a0:	09 00 00 00 	*unknown*
    23a4:	c7 1d 00 1b 	*unknown*
    23a8:	f0 05 02 56 	*unknown*
    23ac:	00 00 08 13 	l.j 43f8 <impure_data+0x3a4>
    23b0:	1c 00 00 0c 	*unknown*
    23b4:	0b 05 02 6d 	*unknown*
    23b8:	00 00 06 7c 	l.j 3da8 <_global_impure_ptr+0x374>
    23bc:	1c 00 00 04 	*unknown*
    23c0:	d8 05 02 76 	l.sb 630(r5),r0
    23c4:	00 00 07 ad 	l.j 4278 <impure_data+0x224>
    23c8:	00 08 00 00 	l.j 2023c8 <_end+0x1fd95c>
    23cc:	05 50 00 00 	l.jal 54023cc <_end+0x53fd960>
    23d0:	08 23 09 00 	*unknown*
    23d4:	00 00 c7 18 	l.j 34034 <_end+0x2f5c8>
    23d8:	00 1d 00 00 	l.j 7423d8 <_end+0x73d96c>
    23dc:	08 2e 14 00 	*unknown*
    23e0:	00 04 12 00 	l.j 106be0 <_end+0x102174>
    23e4:	0f 04 00 00 	l.bnf fc1023e4 <_end+0xfc0fd978>
    23e8:	08 23 0f 04 	*unknown*
    23ec:	00 00 01 65 	l.j 2980 <_lseek_r+0x20>
    23f0:	1d 00 00 08 	*unknown*
    23f4:	45 14 00 00 	*unknown*
    23f8:	00 2c 00 0f 	l.j b02434 <_end+0xafd9c8>
    23fc:	04 00 00 08 	l.jal 241c <exit+0x8>
    2400:	4b 0f 04 00 	*unknown*
    2404:	00 08 3a 08 	l.j 210c24 <_end+0x20c1b8>
    2408:	00 00 05 ea 	l.j 3bb0 <_global_impure_ptr+0x17c>
    240c:	00 00 08 61 	l.j 4590 <_or1k_exception_impure_data+0x110>
    2410:	09 00 00 00 	*unknown*
    2414:	c7 02 00 1e 	*unknown*
    2418:	00 00 03 52 	l.j 3160 <_or1k_libc_impure_init+0x7c>
    241c:	01 42 00 00 	l.j 508241c <_end+0x507d9b0>
    2420:	25 9c 00 00 	*unknown*
    2424:	01 e8 01 9c 	l.j 7a02a94 <_end+0x79fe028>
    2428:	00 00 09 22 	l.j 48b0 <object.2876+0x4>
    242c:	1f 00 00 04 	*unknown*
    2430:	8e 01 42 00 	l.lbz r16,16896(r1)
    2434:	00 00 2c 00 	l.j d434 <_end+0x89c8>
    2438:	00 01 b2 20 	l.j 6ecb8 <_end+0x6a24c>
    243c:	64 00 01 42 	*unknown*
    2440:	00 00 01 05 	l.j 2854 <_execve_r>
    2444:	00 00 01 e9 	l.j 2be8 <_or1k_outbyte+0x8>
    2448:	21 70 00 01 	*unknown*
    244c:	45 00 00 02 	*unknown*
    2450:	82 00 00 02 	*unknown*
    2454:	20 22 00 00 	*unknown*
    2458:	05 f1 01 46 	l.jal 7c42970 <_end+0x7c3df04>
    245c:	00 00 09 22 	l.j 48e4 <_or1k_interrupt_handler_table+0x18>
    2460:	00 00 02 49 	l.j 2d84 <or1k_dcache_disable>
    2464:	22 00 00 01 	*unknown*
    2468:	da 01 47 00 	l.sb -30976(r1),r8
    246c:	00 09 28 00 	l.j 24c46c <_end+0x247a00>
    2470:	00 02 73 21 	l.j 9f0f4 <_end+0x9a688>
    2474:	6e 00 01 48 	l.lwa r16,328(r0)
    2478:	00 00 00 2c 	l.j 2528 <__register_exitproc+0xcc>
    247c:	00 00 02 a5 	l.j 2f10 <_or1k_exception_handler+0x168>
    2480:	21 69 00 01 	*unknown*
    2484:	49 00 00 00 	*unknown*
    2488:	2c 00 00 02 	*unknown*
    248c:	c3 21 66 6e 	l.mtspr r1,r12,0xce6e
    2490:	00 01 4a 00 	l.j 54c90 <_end+0x50224>
    2494:	00 02 98 00 	l.j a8494 <_end+0xa3a28>
    2498:	00 03 16 23 	l.j c7d24 <_end+0xc32b8>
    249c:	00 00 05 f7 	l.j 3c78 <_global_impure_ptr+0x244>
    24a0:	01 51 24 00 	l.j 544b4a0 <_end+0x5446a34>
    24a4:	00 00 00 00 	l.j 24a4 <__register_exitproc+0x48>
    24a8:	00 09 11 21 	l.j 24692c <_end+0x241ec0>
    24ac:	69 6e 64 00 	*unknown*
    24b0:	01 5e 00 00 	l.j 57824b0 <_end+0x577da44>
    24b4:	00 2c 00 00 	l.j b024b4 <_end+0xafda48>
    24b8:	03 3f 25 00 	l.j fcfcb8b8 <_end+0xfcfc6e4c>
    24bc:	00 27 58 26 	l.j 9d8554 <_end+0x9d3ae8>
    24c0:	01 53 02 8a 	l.j 54c2ee8 <_end+0x54be47c>
    24c4:	00 00 00 27 	l.j 2560 <__register_exitproc+0x104>
    24c8:	00 00 26 f4 	l.j c098 <_end+0x762c>
    24cc:	00 00 09 4b 	l.j 49f8 <_or1k_exception_handler_table+0x4>
    24d0:	26 01 53 02 	*unknown*
    24d4:	84 00 00 00 	l.lwz r0,0(r0)
    24d8:	0f 04 00 00 	l.bnf fc1024d8 <_end+0xfc0fda6c>
    24dc:	02 82 0f 04 	l.j fa0860ec <_end+0xfa081680>
    24e0:	00 00 01 f4 	l.j 2cb0 <_or1k_cache_init+0xa8>
    24e4:	28 00 00 06 	*unknown*
    24e8:	47 01 0e 00 	*unknown*
    24ec:	00 00 2c 00 	l.j d4ec <_end+0x8a80>
    24f0:	29 00 00 04 	*unknown*
    24f4:	69 05 02 fb 	*unknown*
    24f8:	00 00 09 46 	l.j 4a10 <_or1k_exception_handler_table+0x1c>
    24fc:	18 00 00 04 	l.movhi r0,0x4
    2500:	12 2a 00 00 	l.bf f8a82500 <_end+0xf8a7da94>
    2504:	05 ec 06 56 	l.jal 7b03e5c <_end+0x7aff3f0>
    2508:	14 00 00 01 	*unknown*
    250c:	05 00 00 00 	l.jal 400250c <_end+0x3ffdaa0>
    2510:	00 0f 3a 00 	l.j 3d0d10 <_end+0x3cc2a4>
    2514:	04 00 00 08 	l.jal 2534 <__register_exitproc+0xd8>
    2518:	49 04 01 00 	*unknown*
    251c:	00 08 99 01 	l.j 228920 <_end+0x223eb4>
    2520:	00 00 06 e4 	l.j 40b0 <impure_data+0x5c>
    2524:	00 00 07 aa 	l.j 43cc <impure_data+0x378>
    2528:	00 00 27 84 	l.j c338 <_end+0x78cc>
    252c:	00 00 02 a8 	l.j 2fcc <or1k_interrupts_disable+0x30>
    2530:	00 00 06 e3 	l.j 40bc <impure_data+0x68>
    2534:	02 04 05 69 	l.j f8103ad8 <_end+0xf80ff06c>
    2538:	6e 74 00 03 	l.lwa r19,3(r20)
    253c:	04 05 00 00 	l.jal 14253c <_end+0x13dad0>
    2540:	00 05 04 00 	l.j 143540 <_end+0x13ead4>
    2544:	00 08 c2 02 	l.j 232d4c <_end+0x22e2e0>
    2548:	d4 00 00 00 	l.sw 0(r0),r0
    254c:	3e 03 04 07 	*unknown*
    2550:	00 00 00 5e 	l.j 26c8 <__call_exitprocs+0x12c>
    2554:	03 01 06 00 	l.j fc043d54 <_end+0xfc03f2e8>
    2558:	00 00 7b 03 	l.j 21164 <_end+0x1c6f8>
    255c:	01 08 00 00 	l.j 420255c <_end+0x41fdaf0>
    2560:	00 79 03 02 	l.j 1e43168 <_end+0x1e3e6fc>
    2564:	05 00 00 01 	l.jal 4002568 <_end+0x3ffdafc>
    2568:	2f 03 02 07 	*unknown*
    256c:	00 00 00 0e 	l.j 25a4 <__call_exitprocs+0x8>
    2570:	03 08 05 00 	l.j fc203970 <_end+0xfc1fef04>
    2574:	00 00 00 03 	l.j 2580 <__register_exitproc+0x124>
    2578:	08 07 00 00 	*unknown*
    257c:	00 59 04 00 	l.j 164357c <_end+0x163eb10>
    2580:	00 03 6d 03 	l.j dd98c <_end+0xd8f20>
    2584:	07 00 00 00 	l.jal fc002584 <_end+0xfbffdb18>
    2588:	25 04 00 00 	*unknown*
    258c:	03 4b 04 10 	l.j fd2c35cc <_end+0xfd2beb60>
    2590:	00 00 00 2c 	l.j 2640 <__call_exitprocs+0xa4>
    2594:	04 00 00 07 	l.jal 25b0 <__call_exitprocs+0x14>
    2598:	30 04 14 00 	*unknown*
    259c:	00 00 53 04 	l.j 171ac <_end+0x12740>
    25a0:	00 00 08 5c 	l.j 4710 <_or1k_exception_impure_data+0x290>
    25a4:	04 18 00 00 	l.jal 6025a4 <_end+0x5fdb38>
    25a8:	00 5a 04 00 	l.j 16835a8 <_end+0x167eb3c>
    25ac:	00 06 55 04 	l.j 1979bc <_end+0x192f50>
    25b0:	1b 00 00 00 	l.movhi r24,0x0
    25b4:	5a 04 00 00 	*unknown*
    25b8:	04 27 04 27 	l.jal 9c3654 <_end+0x9bebe8>
    25bc:	00 00 00 2c 	l.j 266c <__call_exitprocs+0xd0>
    25c0:	04 00 00 08 	l.jal 25e0 <__call_exitprocs+0x44>
    25c4:	c0 04 37 00 	l.mtspr r4,r6,0x700
    25c8:	00 00 2c 05 	l.j d5dc <_end+0x8b70>
    25cc:	00 00 03 17 	l.j 3228 <_or1k_libc_impure_init+0x144>
    25d0:	02 01 61 00 	l.j f805a9d0 <_end+0xf8055f64>
    25d4:	00 00 c8 03 	l.j 345e0 <_end+0x2fb74>
    25d8:	04 07 00 00 	l.jal 1c25d8 <_end+0x1bdb6c>
    25dc:	00 63 06 04 	l.j 18c3dec <_end+0x18bf380>
    25e0:	04 4a 00 00 	l.jal 12825e0 <_end+0x127db74>
    25e4:	00 ee 07 00 	l.j 3b841e4 <_end+0x3b7f778>
    25e8:	00 03 11 04 	l.j c69f8 <_end+0xc1f8c>
    25ec:	4c 00 00 00 	l.maci r0,0
    25f0:	bc 07 00 00 	l.sfeqi r7,0
    25f4:	02 84 04 4d 	l.j fa103728 <_end+0xfa0fecbc>
    25f8:	00 00 00 ee 	l.j 29b0 <_read_r+0x4>
    25fc:	00 08 00 00 	l.j 2025fc <_end+0x1fdb90>
    2600:	00 4c 00 00 	l.j 1302600 <_end+0x12fdb94>
    2604:	00 fe 09 00 	l.j 3f84a04 <_end+0x3f7ff98>
    2608:	00 00 fe 03 	l.j 41e14 <_end+0x3d3a8>
    260c:	00 03 04 07 	l.j c3628 <_end+0xbebbc>
    2610:	00 00 01 45 	l.j 2b24 <_or1k_uart_init+0xd0>
    2614:	0a 08 04 47 	*unknown*
    2618:	00 00 01 26 	l.j 2ab0 <_or1k_uart_init+0x5c>
    261c:	0b 00 00 04 	*unknown*
    2620:	11 04 49 00 	l.bf 4114a20 <_end+0x410ffb4>
    2624:	00 00 25 00 	l.j ba24 <_end+0x6fb8>
    2628:	0b 00 00 04 	*unknown*
    262c:	19 04 4e 00 	*unknown*
    2630:	00 00 cf 04 	l.j 36240 <_end+0x317d4>
    2634:	00 04 00 00 	l.j 102634 <_end+0xfdbc8>
    2638:	03 ab 04 4f 	l.j feac3774 <_end+0xfeabed08>
    263c:	00 00 01 05 	l.j 2a50 <_or1k_uart_interrupt_handler+0x24>
    2640:	04 00 00 02 	l.jal 2648 <__call_exitprocs+0xac>
    2644:	47 04 53 00 	*unknown*
    2648:	00 00 6f 0c 	l.j 1e278 <_end+0x1980c>
    264c:	04 04 00 00 	l.jal 10264c <_end+0xfdbe0>
    2650:	04 54 05 16 	l.jal 1503aa8 <_end+0x14ff03c>
    2654:	00 00 00 3e 	l.j 274c <__call_exitprocs+0x1b0>
    2658:	0d 00 00 02 	l.bnf 4002660 <_end+0x3ffdbf4>
    265c:	58 18 05 2d 	*unknown*
    2660:	00 00 01 9c 	l.j 2cd0 <_or1k_cache_init+0xc8>
    2664:	0b 00 00 03 	*unknown*
    2668:	cc 05 2f 00 	l.swa 1792(r5),r5
    266c:	00 01 9c 00 	l.j 6966c <_end+0x64c00>
    2670:	0e 5f 6b 00 	l.bnf f97dd270 <_end+0xf97d8804>
    2674:	05 30 00 00 	l.jal 4c02674 <_end+0x4bfdc08>
    2678:	00 25 04 0b 	l.j 9436a4 <_end+0x93ec38>
    267c:	00 00 04 03 	l.j 3688 <or1k_timer_reset+0x2c>
    2680:	05 30 00 00 	l.jal 4c02680 <_end+0x4bfdc14>
    2684:	00 25 08 0b 	l.j 9446b0 <_end+0x93fc44>
    2688:	00 00 02 41 	l.j 2f8c <or1k_interrupts_enable+0x14>
    268c:	05 30 00 00 	l.jal 4c0268c <_end+0x4bfdc20>
    2690:	00 25 0c 0b 	l.j 9456bc <_end+0x940c50>
    2694:	00 00 04 a0 	l.j 3914 <__udivsi3+0x68>
    2698:	05 30 00 00 	l.jal 4c02698 <_end+0x4bfdc2c>
    269c:	00 25 10 0e 	l.j 9466d4 <_end+0x941c68>
    26a0:	5f 78 00 05 	*unknown*
    26a4:	31 00 00 01 	*unknown*
    26a8:	a2 14 00 0f 	l.addic r16,r20,15
    26ac:	04 00 00 01 	l.jal 26b0 <__call_exitprocs+0x114>
    26b0:	49 08 00 00 	*unknown*
    26b4:	01 3e 00 00 	l.j 4f826b4 <_end+0x4f7dc48>
    26b8:	01 b2 09 00 	l.j 6c84ab8 <_end+0x6c8004c>
    26bc:	00 00 fe 00 	l.j 41ebc <_end+0x3d450>
    26c0:	00 0d 00 00 	l.j 3426c0 <_end+0x33dc54>
    26c4:	02 7f 24 05 	l.j f9fcb6d8 <_end+0xf9fc6c6c>
    26c8:	35 00 00 02 	*unknown*
    26cc:	2b 0b 00 00 	*unknown*
    26d0:	01 b3 05 37 	l.j 6cc3bac <_end+0x6cbf140>
    26d4:	00 00 00 25 	l.j 2768 <__call_exitprocs+0x1cc>
    26d8:	00 0b 00 00 	l.j 2c26d8 <_end+0x2bdc6c>
    26dc:	04 2f 05 38 	l.jal bc3bbc <_end+0xbbf150>
    26e0:	00 00 00 25 	l.j 2774 <__call_exitprocs+0x1d8>
    26e4:	04 0b 00 00 	l.jal 2c26e4 <_end+0x2bdc78>
    26e8:	01 c2 05 39 	l.j 7083bcc <_end+0x707f160>
    26ec:	00 00 00 25 	l.j 2780 <__call_exitprocs+0x1e4>
    26f0:	08 0b 00 00 	*unknown*
    26f4:	05 19 05 3a 	l.jal 4643bdc <_end+0x463f170>
    26f8:	00 00 00 25 	l.j 278c <_write_r+0x8>
    26fc:	0c 0b 00 00 	l.bnf 2c26fc <_end+0x2bdc90>
    2700:	03 37 05 3b 	l.j fcdc3bec <_end+0xfcdbf180>
    2704:	00 00 00 25 	l.j 2798 <_write_r+0x14>
    2708:	10 0b 00 00 	l.bf 2c2708 <_end+0x2bdc9c>
    270c:	03 26 05 3c 	l.j fc983bfc <_end+0xfc97f190>
    2710:	00 00 00 25 	l.j 27a4 <_write_r+0x20>
    2714:	14 0b 00 00 	*unknown*
    2718:	04 a5 05 3d 	l.jal 2943c0c <_end+0x293f1a0>
    271c:	00 00 00 25 	l.j 27b0 <_write_r+0x2c>
    2720:	18 0b 00 00 	*unknown*
    2724:	03 8d 05 3e 	l.j fe343c1c <_end+0xfe33f1b0>
    2728:	00 00 00 25 	l.j 27bc <_write_r+0x38>
    272c:	1c 0b 00 00 	*unknown*
    2730:	04 e0 05 3f 	l.jal 3803c2c <_end+0x37ff1c0>
    2734:	00 00 00 25 	l.j 27c8 <_write_r+0x44>
    2738:	20 00 10 00 	l.sys 0x1000
    273c:	00 01 d1 01 	l.j 76b40 <_end+0x720d4>
    2740:	08 05 48 00 	*unknown*
    2744:	00 02 6b 0b 	l.j 9d370 <_end+0x98904>
    2748:	00 00 02 34 	l.j 3018 <or1k_interrupts_restore+0x44>
    274c:	05 49 00 00 	l.jal 524274c <_end+0x523dce0>
    2750:	02 6b 00 0b 	l.j f9ac277c <_end+0xf9abdd10>
    2754:	00 00 01 4e 	l.j 2c8c <_or1k_cache_init+0x84>
    2758:	05 4a 00 00 	l.jal 5282758 <_end+0x527dcec>
    275c:	02 6b 80 11 	l.j f9ae27a0 <_end+0xf9addd34>
    2760:	00 00 04 4b 	l.j 388c <memset+0x148>
    2764:	05 4c 00 00 	l.jal 5302764 <_end+0x52fdcf8>
    2768:	01 3e 01 00 	l.j 4f82b68 <_end+0x4f7e0fc>
    276c:	11 00 00 01 	l.bf 4002770 <_end+0x3ffdd04>
    2770:	f6 05 4f 00 	*unknown*
    2774:	00 01 3e 01 	l.j 51f78 <_end+0x4d50c>
    2778:	04 00 08 00 	l.jal 4778 <_or1k_exception_impure_data+0x2f8>
    277c:	00 01 3c 00 	l.j 5177c <_end+0x4cd10>
    2780:	00 02 7b 09 	l.j a13a4 <_end+0x9c938>
    2784:	00 00 00 fe 	l.j 2b7c <or1k_uart_set_read_cb+0x8>
    2788:	1f 00 10 00 	*unknown*
    278c:	00 01 3d 01 	l.j 51b90 <_end+0x4d124>
    2790:	90 05 5b 00 	l.lbs r0,23296(r5)
    2794:	00 02 b9 0b 	l.j b0bc0 <_end+0xac154>
    2798:	00 00 03 cc 	l.j 36c8 <or1k_timer_reset_ticks+0xc>
    279c:	05 5c 00 00 	l.jal 570279c <_end+0x56fdd30>
    27a0:	02 b9 00 0b 	l.j fae427cc <_end+0xfae3dd60>
    27a4:	00 00 03 e4 	l.j 3734 <__errno+0x18>
    27a8:	05 5d 00 00 	l.jal 57427a8 <_end+0x573dd3c>
    27ac:	00 25 04 0b 	l.j 9437d8 <_end+0x93ed6c>
    27b0:	00 00 02 3c 	l.j 30a0 <or1k_interrupt_enable+0x10>
    27b4:	05 5f 00 00 	l.jal 57c27b4 <_end+0x57bdd48>
    27b8:	02 bf 08 0b 	l.j fafc47e4 <_end+0xfafbfd78>
    27bc:	00 00 01 d1 	l.j 2f00 <_or1k_exception_handler+0x158>
    27c0:	05 60 00 00 	l.jal 58027c0 <_end+0x57fdd54>
    27c4:	02 2b 88 00 	l.j f8ae47c4 <_end+0xf8adfd58>
    27c8:	0f 04 00 00 	l.bnf fc1027c8 <_end+0xfc0fdd5c>
    27cc:	02 7b 08 00 	l.j f9ec47cc <_end+0xf9ebfd60>
    27d0:	00 02 cf 00 	l.j b63d0 <_end+0xb1964>
    27d4:	00 02 cf 09 	l.j b63f8 <_end+0xb198c>
    27d8:	00 00 00 fe 	l.j 2bd0 <or1k_uart_set_read_cb+0x5c>
    27dc:	1f 00 0f 04 	*unknown*
    27e0:	00 00 02 d5 	l.j 3334 <or1k_critical_end+0x20>
    27e4:	12 0d 00 00 	l.bf f83427e4 <_end+0xf833dd78>
    27e8:	03 97 08 05 	l.j fe5c47fc <_end+0xfe5bfd90>
    27ec:	73 00 00 02 	*unknown*
    27f0:	fb 0b 00 00 	*unknown*
    27f4:	09 f8 05 74 	*unknown*
    27f8:	00 00 02 fb 	l.j 33e4 <or1k_timer_init+0x10>
    27fc:	00 0b 00 00 	l.j 2c27fc <_end+0x2bdd90>
    2800:	07 8b 05 75 	l.jal fe2c3dd4 <_end+0xfe2bf368>
    2804:	00 00 00 25 	l.j 2898 <_fork_r+0x24>
    2808:	04 00 0f 04 	l.jal 6418 <_end+0x19ac>
    280c:	00 00 00 4c 	l.j 293c <_kill_r+0x1c>
    2810:	0d 00 00 03 	l.bnf 400281c <_end+0x3ffddb0>
    2814:	b6 68 05 b3 	l.mfspr r19,r8,0x5b3
    2818:	00 00 04 2b 	l.j 38c4 <__udivsi3+0x18>
    281c:	0e 5f 70 00 	l.bnf f97de81c <_end+0xf97d9db0>
    2820:	05 b4 00 00 	l.jal 6d02820 <_end+0x6cfddb4>
    2824:	02 fb 00 0e 	l.j fbec285c <_end+0xfbebddf0>
    2828:	5f 72 00 05 	*unknown*
    282c:	b5 00 00 00 	l.mfspr r8,r0,0x0
    2830:	25 04 0e 5f 	*unknown*
    2834:	77 00 05 b6 	*unknown*
    2838:	00 00 00 25 	l.j 28cc <_getpid_r+0xc>
    283c:	08 0b 00 00 	*unknown*
    2840:	01 ef 05 b7 	l.j 7bc3f1c <_end+0x7bbf4b0>
    2844:	00 00 00 53 	l.j 2990 <_open+0x4>
    2848:	0c 0b 00 00 	l.bnf 2c2848 <_end+0x2bdddc>
    284c:	02 9b 05 b8 	l.j fa6c3f2c <_end+0xfa6bf4c0>
    2850:	00 00 00 53 	l.j 299c <_open+0x10>
    2854:	0e 0e 5f 62 	l.bnf f839a5dc <_end+0xf8395b70>
    2858:	66 00 05 b9 	*unknown*
    285c:	00 00 02 d6 	l.j 33b4 <_or1k_timer_interrupt_handler+0x30>
    2860:	10 0b 00 00 	l.bf 2c2860 <_end+0x2bddf4>
    2864:	01 8d 05 ba 	l.j 6343f4c <_end+0x633f4e0>
    2868:	00 00 00 25 	l.j 28fc <_gettimeofday+0x1c>
    286c:	18 0b 00 00 	*unknown*
    2870:	01 df 05 c1 	l.j 77c3f74 <_end+0x77bf508>
    2874:	00 00 01 3c 	l.j 2d64 <or1k_dcache_enable+0x8>
    2878:	1c 0b 00 00 	*unknown*
    287c:	02 6f 05 c3 	l.j f9bc3f88 <_end+0xf9bbf51c>
    2880:	00 00 05 8e 	l.j 3eb8 <_global_impure_ptr+0x484>
    2884:	20 0b 00 00 	*unknown*
    2888:	09 a6 05 c5 	*unknown*
    288c:	00 00 05 bd 	l.j 3f80 <_global_impure_ptr+0x54c>
    2890:	24 0b 00 00 	*unknown*
    2894:	04 21 05 c8 	l.jal 843fb4 <_end+0x83f548>
    2898:	00 00 05 e1 	l.j 401c <__EH_FRAME_BEGIN__+0x1c>
    289c:	28 0b 00 00 	*unknown*
    28a0:	04 fa 05 c9 	l.jal 3e83fc4 <_end+0x3e7f558>
    28a4:	00 00 05 fb 	l.j 4090 <impure_data+0x3c>
    28a8:	2c 0e 5f 75 	*unknown*
    28ac:	62 00 05 cc 	*unknown*
    28b0:	00 00 02 d6 	l.j 3408 <or1k_timer_init+0x34>
    28b4:	30 0e 5f 75 	*unknown*
    28b8:	70 00 05 cd 	*unknown*
    28bc:	00 00 02 fb 	l.j 34a8 <or1k_timer_set_period+0x20>
    28c0:	38 0e 5f 75 	*unknown*
    28c4:	72 00 05 ce 	*unknown*
    28c8:	00 00 00 25 	l.j 295c <_link_r+0x1c>
    28cc:	3c 0b 00 00 	*unknown*
    28d0:	01 bc 05 d1 	l.j 6f04014 <_end+0x6eff5a8>
    28d4:	00 00 06 01 	l.j 40d8 <impure_data+0x84>
    28d8:	40 0b 00 00 	*unknown*
    28dc:	04 d2 05 d2 	l.jal 3484024 <_end+0x347f5b8>
    28e0:	00 00 06 11 	l.j 4124 <impure_data+0xd0>
    28e4:	43 0e 5f 6c 	*unknown*
    28e8:	62 00 05 d5 	*unknown*
    28ec:	00 00 02 d6 	l.j 3444 <or1k_timer_init+0x70>
    28f0:	44 0b 00 00 	*unknown*
    28f4:	08 18 05 d8 	*unknown*
    28f8:	00 00 00 25 	l.j 298c <_open>
    28fc:	4c 0b 00 00 	l.maci r11,0
    2900:	02 0d 05 d9 	l.j f8344064 <_end+0xf833f5f8>
    2904:	00 00 00 7a 	l.j 2aec <_or1k_uart_init+0x98>
    2908:	50 0b 00 00 	*unknown*
    290c:	0c 6a 05 dc 	l.bnf 1a8407c <_end+0x1a7f610>
    2910:	00 00 04 49 	l.j 3a34 <_global_impure_ptr>
    2914:	54 0b 00 00 	*unknown*
    2918:	06 4f 05 e0 	l.jal f93c4098 <_end+0xf93bf62c>
    291c:	00 00 01 31 	l.j 2de0 <_or1k_exception_handler+0x38>
    2920:	58 0b 00 00 	*unknown*
    2924:	03 be 05 e2 	l.j fef840ac <_end+0xfef7f640>
    2928:	00 00 01 26 	l.j 2dc0 <_or1k_exception_handler+0x18>
    292c:	5c 0b 00 00 	*unknown*
    2930:	03 1e 05 e3 	l.j fc7840bc <_end+0xfc77f650>
    2934:	00 00 00 25 	l.j 29c8 <_read_r+0x1c>
    2938:	64 00 13 00 	*unknown*
    293c:	00 00 25 00 	l.j bd3c <_end+0x72d0>
    2940:	00 04 49 14 	l.j 114d90 <_end+0x110324>
    2944:	00 00 04 49 	l.j 3a68 <_global_impure_ptr+0x34>
    2948:	14 00 00 01 	*unknown*
    294c:	3c 14 00 00 	*unknown*
    2950:	05 81 14 00 	l.jal 6047950 <_end+0x6042ee4>
    2954:	00 00 25 00 	l.j bd54 <_end+0x72e8>
    2958:	0f 04 00 00 	l.bnf fc102958 <_end+0xfc0fdeec>
    295c:	04 4f 15 00 	l.jal 13c7d5c <_end+0x13c32f0>
    2960:	00 0c 0b 04 	l.j 305570 <_end+0x300b04>
    2964:	24 05 02 39 	*unknown*
    2968:	00 00 05 81 	l.j 3f6c <_global_impure_ptr+0x538>
    296c:	16 00 00 06 	*unknown*
    2970:	bd 05 02 3b 	*unknown*
    2974:	00 00 00 25 	l.j 2a08 <_stat_r+0x1c>
    2978:	00 16 00 00 	l.j 582978 <_end+0x57df0c>
    297c:	01 fe 05 02 	l.j 7f83d84 <_end+0x7f7f318>
    2980:	40 00 00 06 	*unknown*
    2984:	68 04 16 00 	*unknown*
    2988:	00 02 8b 05 	l.j a559c <_end+0xa0b30>
    298c:	02 40 00 00 	l.j f900298c <_end+0xf8ffdf20>
    2990:	06 68 08 16 	l.jal f9a049e8 <_end+0xf99fff7c>
    2994:	00 00 02 50 	l.j 32d4 <_or1k_init+0x44>
    2998:	05 02 40 00 	l.jal 4092998 <_end+0x408df2c>
    299c:	00 06 68 0c 	l.j 19c9cc <_end+0x197f60>
    29a0:	16 00 00 03 	*unknown*
    29a4:	df 05 02 42 	l.sh -15806(r5),r0
    29a8:	00 00 00 25 	l.j 2a3c <_or1k_uart_interrupt_handler+0x10>
    29ac:	10 16 00 00 	l.bf 5829ac <_end+0x57df40>
    29b0:	01 62 05 02 	l.j 5883db8 <_end+0x587f34c>
    29b4:	43 00 00 08 	*unknown*
    29b8:	4a 14 16 00 	*unknown*
    29bc:	00 04 7c 05 	l.j 1219d0 <_end+0x11cf64>
    29c0:	02 45 00 00 	l.j f91429c0 <_end+0xf913df54>
    29c4:	00 25 30 16 	l.j 94ea1c <_end+0x949fb0>
    29c8:	00 00 03 e9 	l.j 396c <__udivsi3+0xc0>
    29cc:	05 02 46 00 	l.jal 40941cc <_end+0x408f760>
    29d0:	00 05 b2 34 	l.j 16f2a0 <_end+0x16a834>
    29d4:	16 00 00 03 	*unknown*
    29d8:	40 05 02 48 	*unknown*
    29dc:	00 00 00 25 	l.j 2a70 <_or1k_uart_init+0x1c>
    29e0:	38 16 00 00 	*unknown*
    29e4:	03 f9 05 02 	l.j ffe43dec <_end+0xffe3f380>
    29e8:	4a 00 00 08 	*unknown*
    29ec:	65 3c 16 00 	*unknown*
    29f0:	00 03 09 05 	l.j c4e04 <_end+0xc0398>
    29f4:	02 4d 00 00 	l.j f93429f4 <_end+0xf933df88>
    29f8:	01 9c 40 16 	l.j 6712a50 <_end+0x670dfe4>
    29fc:	00 00 02 75 	l.j 33d0 <_or1k_timer_interrupt_handler+0x4c>
    2a00:	05 02 4e 00 	l.jal 4096200 <_end+0x4091794>
    2a04:	00 00 25 44 	l.j bf14 <_end+0x74a8>
    2a08:	16 00 00 05 	*unknown*
    2a0c:	14 05 02 4f 	*unknown*
    2a10:	00 00 01 9c 	l.j 3080 <_or1k_interrupt_handler+0x64>
    2a14:	48 16 00 00 	*unknown*
    2a18:	03 63 05 02 	l.j fd8c3e20 <_end+0xfd8bf3b4>
    2a1c:	50 00 00 08 	*unknown*
    2a20:	6b 4c 16 00 	*unknown*
    2a24:	00 02 93 05 	l.j a7638 <_end+0xa2bcc>
    2a28:	02 53 00 00 	l.j f94c2a28 <_end+0xf94bdfbc>
    2a2c:	00 25 50 16 	l.j 956a84 <_end+0x952018>
    2a30:	00 00 02 05 	l.j 3244 <_or1k_libc_impure_init+0x160>
    2a34:	05 02 54 00 	l.jal 4097a34 <_end+0x4092fc8>
    2a38:	00 05 81 54 	l.j 162f88 <_end+0x15e51c>
    2a3c:	16 00 00 03 	*unknown*
    2a40:	7f 05 02 77 	*unknown*
    2a44:	00 00 08 28 	l.j 4ae4 <_end+0x78>
    2a48:	58 17 00 00 	*unknown*
    2a4c:	01 3d 05 02 	l.j 4f43e54 <_end+0x4f3f3e8>
    2a50:	7b 00 00 02 	*unknown*
    2a54:	b9 01 48 17 	*unknown*
    2a58:	00 00 02 e7 	l.j 35f4 <or1k_timer_disable+0x28>
    2a5c:	05 02 7c 00 	l.jal 40a1a5c <_end+0x409cff0>
    2a60:	00 02 7b 01 	l.j a1664 <_end+0x9cbf8>
    2a64:	4c 17 00 00 	l.maci r23,0
    2a68:	04 c8 05 02 	l.jal 3203e70 <_end+0x31ff404>
    2a6c:	80 00 00 08 	*unknown*
    2a70:	7c 02 dc 17 	*unknown*
    2a74:	00 00 01 e7 	l.j 3210 <_or1k_libc_impure_init+0x12c>
    2a78:	05 02 85 00 	l.jal 40a3e78 <_end+0x409f40c>
    2a7c:	00 06 2d 02 	l.j 18de84 <_end+0x189418>
    2a80:	e0 17 00 00 	l.add r0,r23,r0
    2a84:	01 cc 05 02 	l.j 7303e8c <_end+0x72ff420>
    2a88:	86 00 00 08 	l.lwz r16,8(r0)
    2a8c:	88 02 ec 00 	l.lws r0,-5120(r2)
    2a90:	0f 04 00 00 	l.bnf fc102a90 <_end+0xfc0fe024>
    2a94:	05 87 03 01 	l.jal 61c3698 <_end+0x61bec2c>
    2a98:	06 00 00 00 	l.jal f8002a98 <_end+0xf7ffe02c>
    2a9c:	82 0f 04 00 	*unknown*
    2aa0:	00 04 2b 13 	l.j 10d6ec <_end+0x108c80>
    2aa4:	00 00 00 25 	l.j 2b38 <_or1k_uart_write+0xc>
    2aa8:	00 00 05 b2 	l.j 4170 <impure_data+0x11c>
    2aac:	14 00 00 04 	*unknown*
    2ab0:	49 14 00 00 	*unknown*
    2ab4:	01 3c 14 00 	l.j 4f07ab4 <_end+0x4f03048>
    2ab8:	00 05 b2 14 	l.j 16f308 <_end+0x16a89c>
    2abc:	00 00 00 25 	l.j 2b50 <_or1k_uart_write+0x24>
    2ac0:	00 0f 04 00 	l.j 3c3ac0 <_end+0x3bf054>
    2ac4:	00 05 b8 18 	l.j 170b24 <_end+0x16c0b8>
    2ac8:	00 00 05 87 	l.j 40e4 <impure_data+0x90>
    2acc:	0f 04 00 00 	l.bnf fc102acc <_end+0xfc0fe060>
    2ad0:	05 94 13 00 	l.jal 65076d0 <_end+0x6502c64>
    2ad4:	00 00 a6 00 	l.j 2c2d4 <_end+0x27868>
    2ad8:	00 05 e1 14 	l.j 17af28 <_end+0x1764bc>
    2adc:	00 00 04 49 	l.j 3c00 <_global_impure_ptr+0x1cc>
    2ae0:	14 00 00 01 	*unknown*
    2ae4:	3c 14 00 00 	*unknown*
    2ae8:	00 a6 14 00 	l.j 2987ae8 <_end+0x298307c>
    2aec:	00 00 25 00 	l.j beec <_end+0x7480>
    2af0:	0f 04 00 00 	l.bnf fc102af0 <_end+0xfc0fe084>
    2af4:	05 c3 13 00 	l.jal 70c76f4 <_end+0x70c2c88>
    2af8:	00 00 25 00 	l.j bef8 <_end+0x748c>
    2afc:	00 05 fb 14 	l.j 18174c <_end+0x17cce0>
    2b00:	00 00 04 49 	l.j 3c24 <_global_impure_ptr+0x1f0>
    2b04:	14 00 00 01 	*unknown*
    2b08:	3c 00 0f 04 	*unknown*
    2b0c:	00 00 05 e7 	l.j 42a8 <impure_data+0x254>
    2b10:	08 00 00 00 	*unknown*
    2b14:	4c 00 00 06 	l.maci r0,6
    2b18:	11 09 00 00 	l.bf 4242b18 <_end+0x423e0ac>
    2b1c:	00 fe 02 00 	l.j 3f8331c <_end+0x3f7e8b0>
    2b20:	08 00 00 00 	*unknown*
    2b24:	4c 00 00 06 	l.maci r0,6
    2b28:	21 09 00 00 	*unknown*
    2b2c:	00 fe 00 00 	l.j 3f82b2c <_end+0x3f7e0c0>
    2b30:	05 00 00 03 	l.jal 4002b3c <_end+0x3ffe0d0>
    2b34:	a4 05 01 1d 	l.andi r0,r5,0x11d
    2b38:	00 00 03 01 	l.j 373c <__getreent>
    2b3c:	19 00 00 04 	l.movhi r8,0x4
    2b40:	af 0c 05 01 	l.xori r24,r12,1281
    2b44:	21 00 00 06 	l.trap 0x6
    2b48:	62 16 00 00 	*unknown*
    2b4c:	03 cc 05 01 	l.j ff303f50 <_end+0xff2ff4e4>
    2b50:	23 00 00 06 	*unknown*
    2b54:	62 00 16 00 	*unknown*
    2b58:	00 02 a1 05 	l.j aaf6c <_end+0xa6500>
    2b5c:	01 24 00 00 	l.j 4902b5c <_end+0x48fe0f0>
    2b60:	00 25 04 16 	l.j 943bb8 <_end+0x93f14c>
    2b64:	00 00 03 9e 	l.j 39dc <__do_global_ctors_aux+0x34>
    2b68:	05 01 25 00 	l.jal 404bf68 <_end+0x40474fc>
    2b6c:	00 06 68 08 	l.j 19cb8c <_end+0x198120>
    2b70:	00 0f 04 00 	l.j 3c3b70 <_end+0x3bf104>
    2b74:	00 06 2d 0f 	l.j 18dfb0 <_end+0x189544>
    2b78:	04 00 00 06 	l.jal 2b90 <or1k_uart_set_read_cb+0x1c>
    2b7c:	21 19 00 00 	*unknown*
    2b80:	01 5a 0e 05 	l.j 5686394 <_end+0x5681928>
    2b84:	01 3d 00 00 	l.j 4f42b84 <_end+0x4f3e118>
    2b88:	06 a3 16 00 	l.jal fa8c8388 <_end+0xfa8c391c>
    2b8c:	00 04 0b 05 	l.j 1057a0 <_end+0x100d34>
    2b90:	01 3e 00 00 	l.j 4f82b90 <_end+0x4f7e124>
    2b94:	06 a3 00 16 	l.jal fa8c2bec <_end+0xfa8be180>
    2b98:	00 00 04 38 	l.j 3c78 <_global_impure_ptr+0x244>
    2b9c:	05 01 3f 00 	l.jal 405279c <_end+0x404dd30>
    2ba0:	00 06 a3 06 	l.j 1ab7b8 <_end+0x1a6d4c>
    2ba4:	16 00 00 0e 	*unknown*
    2ba8:	6d 05 01 40 	l.lwa r8,320(r5)
    2bac:	00 00 00 5a 	l.j 2d14 <or1k_icache_enable+0x4>
    2bb0:	0c 00 08 00 	l.bnf 4bb0 <_end+0x144>
    2bb4:	00 00 5a 00 	l.j 193b4 <_end+0x14948>
    2bb8:	00 06 b3 09 	l.j 1af7dc <_end+0x1aad70>
    2bbc:	00 00 00 fe 	l.j 2fb4 <or1k_interrupts_disable+0x18>
    2bc0:	02 00 1a cc 	l.j f80096f0 <_end+0xf8004c84>
    2bc4:	05 02 58 00 	l.jal 4098bc4 <_end+0x4094158>
    2bc8:	00 07 b4 16 	l.j 1efc20 <_end+0x1eb1b4>
    2bcc:	00 00 04 93 	l.j 3e18 <_global_impure_ptr+0x3e4>
    2bd0:	05 02 5a 00 	l.jal 40993d0 <_end+0x4094964>
    2bd4:	00 00 c8 00 	l.j 34bd4 <_end+0x30168>
    2bd8:	16 00 00 04 	*unknown*
    2bdc:	3e 05 02 5b 	*unknown*
    2be0:	00 00 05 81 	l.j 41e4 <impure_data+0x190>
    2be4:	04 16 00 00 	l.jal 582be4 <_end+0x57e178>
    2be8:	02 fc 05 02 	l.j fbf03ff0 <_end+0xfbeff584>
    2bec:	5c 00 00 07 	*unknown*
    2bf0:	b4 08 16 00 	l.mfspr r0,r8,0x1600
    2bf4:	00 04 eb 05 	l.j 13d808 <_end+0x138d9c>
    2bf8:	02 5d 00 00 	l.j f9742bf8 <_end+0xf973e18c>
    2bfc:	01 b2 24 16 	l.j 6c8bc54 <_end+0x6c871e8>
    2c00:	00 00 02 60 	l.j 3580 <or1k_timer_enable+0x14>
    2c04:	05 02 5e 00 	l.jal 409a404 <_end+0x4095998>
    2c08:	00 00 25 48 	l.j c128 <_end+0x76bc>
    2c0c:	16 00 00 03 	*unknown*
    2c10:	c7 05 02 5f 	*unknown*
    2c14:	00 00 00 68 	l.j 2db4 <_or1k_exception_handler+0xc>
    2c18:	4c 16 00 00 	l.maci r22,0
    2c1c:	05 01 05 02 	l.jal 4044024 <_end+0x403f5b8>
    2c20:	60 00 00 06 	*unknown*
    2c24:	6e 54 16 00 	l.lwa r18,5632(r20)
    2c28:	00 03 d2 05 	l.j f743c <_end+0xf29d0>
    2c2c:	02 61 00 00 	l.j f9842c2c <_end+0xf983e1c0>
    2c30:	01 26 64 16 	l.j 499bc88 <_end+0x499721c>
    2c34:	00 00 05 06 	l.j 404c <__dso_handle>
    2c38:	05 02 62 00 	l.jal 409b438 <_end+0x40969cc>
    2c3c:	00 01 26 6c 	l.j 4c5ec <_end+0x47b80>
    2c40:	16 00 00 01 	*unknown*
    2c44:	a5 05 02 63 	l.andi r8,r5,0x263
    2c48:	00 00 01 26 	l.j 30e0 <or1k_interrupt_disable+0x28>
    2c4c:	74 16 00 00 	*unknown*
    2c50:	04 be 05 02 	l.jal 2f84058 <_end+0x2f7f5ec>
    2c54:	64 00 00 07 	*unknown*
    2c58:	c4 7c 16 00 	*unknown*
    2c5c:	00 02 f0 05 	l.j bec70 <_end+0xba204>
    2c60:	02 65 00 00 	l.j f9942c60 <_end+0xf993e1f4>
    2c64:	07 d4 84 16 	l.jal ff523cbc <_end+0xff51f250>
    2c68:	00 00 04 5c 	l.j 3dd8 <_global_impure_ptr+0x3a4>
    2c6c:	05 02 66 00 	l.jal 409c46c <_end+0x4097a00>
    2c70:	00 00 25 9c 	l.j c2e0 <_end+0x7874>
    2c74:	16 00 00 02 	*unknown*
    2c78:	26 05 02 67 	*unknown*
    2c7c:	00 00 01 26 	l.j 3114 <_or1k_libc_impure_init+0x30>
    2c80:	a0 16 00 00 	l.addic r0,r22,0
    2c84:	01 96 05 02 	l.j 658408c <_end+0x657f620>
    2c88:	68 00 00 01 	*unknown*
    2c8c:	26 a8 16 00 	*unknown*
    2c90:	00 02 15 05 	l.j 880a4 <_end+0x83638>
    2c94:	02 69 00 00 	l.j f9a42c94 <_end+0xf9a3e228>
    2c98:	01 26 b0 16 	l.j 49aecf0 <_end+0x49aa284>
    2c9c:	00 00 01 6d 	l.j 3250 <_or1k_libc_impure_init+0x16c>
    2ca0:	05 02 6a 00 	l.jal 409d4a0 <_end+0x4098a34>
    2ca4:	00 01 26 b8 	l.j 4c784 <_end+0x47d18>
    2ca8:	16 00 00 01 	*unknown*
    2cac:	7c 05 02 6b 	*unknown*
    2cb0:	00 00 01 26 	l.j 3148 <_or1k_libc_impure_init+0x64>
    2cb4:	c0 16 00 00 	l.mtspr r22,r0,0x0
    2cb8:	03 84 05 02 	l.j fe1040c0 <_end+0xfe0ff654>
    2cbc:	6c 00 00 00 	l.lwa r0,0(r0)
    2cc0:	25 c8 00 08 	*unknown*
    2cc4:	00 00 05 87 	l.j 42e0 <impure_data+0x28c>
    2cc8:	00 00 07 c4 	l.j 4bd8 <_end+0x16c>
    2ccc:	09 00 00 00 	*unknown*
    2cd0:	fe 19 00 08 	*unknown*
    2cd4:	00 00 05 87 	l.j 42f0 <impure_data+0x29c>
    2cd8:	00 00 07 d4 	l.j 4c28 <_end+0x1bc>
    2cdc:	09 00 00 00 	*unknown*
    2ce0:	fe 07 00 08 	*unknown*
    2ce4:	00 00 05 87 	l.j 4300 <impure_data+0x2ac>
    2ce8:	00 00 07 e4 	l.j 4c78 <_end+0x20c>
    2cec:	09 00 00 00 	*unknown*
    2cf0:	fe 17 00 1a 	*unknown*
    2cf4:	f0 05 02 71 	*unknown*
    2cf8:	00 00 08 08 	l.j 4d18 <_end+0x2ac>
    2cfc:	16 00 00 03 	*unknown*
    2d00:	30 05 02 74 	*unknown*
    2d04:	00 00 08 08 	l.j 4d24 <_end+0x2b8>
    2d08:	00 16 00 00 	l.j 582d08 <_end+0x57e29c>
    2d0c:	04 b5 05 02 	l.jal 2d44114 <_end+0x2d3f6a8>
    2d10:	75 00 00 08 	*unknown*
    2d14:	18 78 00 08 	*unknown*
    2d18:	00 00 02 fb 	l.j 3904 <__udivsi3+0x58>
    2d1c:	00 00 08 18 	l.j 4d7c <_end+0x310>
    2d20:	09 00 00 00 	*unknown*
    2d24:	fe 1d 00 08 	*unknown*
    2d28:	00 00 00 c8 	l.j 3048 <_or1k_interrupt_handler+0x2c>
    2d2c:	00 00 08 28 	l.j 4dcc <_end+0x360>
    2d30:	09 00 00 00 	*unknown*
    2d34:	fe 1d 00 1b 	*unknown*
    2d38:	f0 05 02 56 	*unknown*
    2d3c:	00 00 08 4a 	l.j 4e64 <_end+0x3f8>
    2d40:	1c 00 00 0c 	*unknown*
    2d44:	0b 05 02 6d 	*unknown*
    2d48:	00 00 06 b3 	l.j 4814 <_or1k_exception_impure_data+0x394>
    2d4c:	1c 00 00 04 	*unknown*
    2d50:	d8 05 02 76 	l.sb 630(r5),r0
    2d54:	00 00 07 e4 	l.j 4ce4 <_end+0x278>
    2d58:	00 08 00 00 	l.j 202d58 <_end+0x1fe2ec>
    2d5c:	05 87 00 00 	l.jal 61c2d5c <_end+0x61be2f0>
    2d60:	08 5a 09 00 	*unknown*
    2d64:	00 00 fe 18 	l.j 425c4 <_end+0x3db58>
    2d68:	00 1d 00 00 	l.j 742d68 <_end+0x73e2fc>
    2d6c:	08 65 14 00 	*unknown*
    2d70:	00 04 49 00 	l.j 115170 <_end+0x110704>
    2d74:	0f 04 00 00 	l.bnf fc102d74 <_end+0xfc0fe308>
    2d78:	08 5a 0f 04 	*unknown*
    2d7c:	00 00 01 9c 	l.j 33ec <or1k_timer_init+0x18>
    2d80:	1d 00 00 08 	*unknown*
    2d84:	7c 14 00 00 	*unknown*
    2d88:	00 25 00 0f 	l.j 942dc4 <_end+0x93e358>
    2d8c:	04 00 00 08 	l.jal 2dac <_or1k_exception_handler+0x4>
    2d90:	82 0f 04 00 	*unknown*
    2d94:	00 08 71 08 	l.j 21f1b4 <_end+0x21a748>
    2d98:	00 00 06 21 	l.j 461c <_or1k_exception_impure_data+0x19c>
    2d9c:	00 00 08 98 	l.j 4ffc <_end+0x590>
    2da0:	09 00 00 00 	*unknown*
    2da4:	fe 02 00 04 	*unknown*
    2da8:	00 00 07 4c 	l.j 4ad8 <_end+0x6c>
    2dac:	06 1c 00 00 	l.jal f8702dac <_end+0xf86fe340>
    2db0:	00 2c 04 00 	l.j b03db0 <_end+0xaff344>
    2db4:	00 07 f8 07 	l.j 200dd0 <_end+0x1fc364>
    2db8:	7c 00 00 00 	l.cust4
    2dbc:	2c 04 00 00 	*unknown*
    2dc0:	08 f1 07 8e 	*unknown*
    2dc4:	00 00 00 5a 	l.j 2f2c <_or1k_exception_handler+0x184>
    2dc8:	04 00 00 03 	l.jal 2dd4 <_or1k_exception_handler+0x2c>
    2dcc:	4c 07 ab 00 	l.maci r7,-21760
    2dd0:	00 00 7a 04 	l.j 215e0 <_end+0x1cb74>
    2dd4:	00 00 07 32 	l.j 4a9c <_end+0x30>
    2dd8:	07 ac 00 00 	l.jal feb02dd8 <_end+0xfeafe36c>
    2ddc:	00 85 04 00 	l.j 2143ddc <_end+0x213f370>
    2de0:	00 08 5e 07 	l.j 21a5fc <_end+0x215b90>
    2de4:	ad 00 00 00 	l.xori r8,r0,0
    2de8:	90 04 00 00 	l.lbs r0,0(r4)
    2dec:	06 57 07 ae 	l.jal f95c4ca4 <_end+0xf95c0238>
    2df0:	00 00 00 9b 	l.j 305c <_or1k_interrupt_handler+0x40>
    2df4:	04 00 00 06 	l.jal 2e0c <_or1k_exception_handler+0x64>
    2df8:	78 07 cc 00 	*unknown*
    2dfc:	00 00 c8 04 	l.j 34e0c <_end+0x303a0>
    2e00:	00 00 06 70 	l.j 47c0 <_or1k_exception_impure_data+0x340>
    2e04:	07 d1 00 00 	l.jal ff442e04 <_end+0xff43e398>
    2e08:	00 5a 05 00 	l.j 1684208 <_end+0x167f79c>
    2e0c:	00 07 4e 07 	l.j 1d6628 <_end+0x1d1bbc>
    2e10:	01 12 00 00 	l.j 4482e10 <_end+0x447e3a4>
    2e14:	08 98 0d 00 	*unknown*
    2e18:	00 06 ad 3c 	l.j 1ae308 <_end+0x1a989c>
    2e1c:	08 1b 00 00 	*unknown*
    2e20:	09 e0 0b 00 	*unknown*
    2e24:	00 06 7f 08 	l.j 1a2a44 <_end+0x19dfd8>
    2e28:	1d 00 00 08 	*unknown*
    2e2c:	c4 00 0b 00 	*unknown*
    2e30:	00 08 0f 08 	l.j 206a50 <_end+0x201fe4>
    2e34:	1e 00 00 08 	*unknown*
    2e38:	ae 02 0b 00 	l.xori r16,r2,2816
    2e3c:	00 08 64 08 	l.j 21be5c <_end+0x2173f0>
    2e40:	1f 00 00 08 	*unknown*
    2e44:	e5 04 0b 00 	*unknown*
    2e48:	00 08 06 08 	l.j 204668 <_end+0x1ffbfc>
    2e4c:	20 00 00 08 	l.sys 0x8
    2e50:	f0 08 0b 00 	*unknown*
    2e54:	00 07 5f 08 	l.j 1daa74 <_end+0x1d6008>
    2e58:	21 00 00 08 	l.trap 0x8
    2e5c:	cf 0a 0b 00 	l.swa -15616(r10),r1
    2e60:	00 06 97 08 	l.j 1a8a80 <_end+0x1a4014>
    2e64:	22 00 00 08 	*unknown*
    2e68:	da 0c 0b 00 	l.sb -32000(r12),r1
    2e6c:	00 08 c9 08 	l.j 23528c <_end+0x230820>
    2e70:	23 00 00 08 	*unknown*
    2e74:	c4 0e 0b 00 	*unknown*
    2e78:	00 07 89 08 	l.j 1e5298 <_end+0x1e082c>
    2e7c:	24 00 00 08 	*unknown*
    2e80:	b9 10 0b 00 	*unknown*
    2e84:	00 08 41 08 	l.j 2132a4 <_end+0x20e838>
    2e88:	32 00 00 08 	*unknown*
    2e8c:	a3 14 0b 00 	l.addic r24,r20,2816
    2e90:	00 07 38 08 	l.j 1d0eb0 <_end+0x1cc444>
    2e94:	33 00 00 00 	*unknown*
    2e98:	2c 18 0b 00 	*unknown*
    2e9c:	00 08 de 08 	l.j 23a6bc <_end+0x235c50>
    2ea0:	34 00 00 08 	*unknown*
    2ea4:	a3 1c 0b 00 	l.addic r24,r28,2816
    2ea8:	00 07 42 08 	l.j 1d36c8 <_end+0x1cec5c>
    2eac:	35 00 00 00 	*unknown*
    2eb0:	2c 20 0b 00 	*unknown*
    2eb4:	00 08 21 08 	l.j 20b2d4 <_end+0x206868>
    2eb8:	36 00 00 08 	*unknown*
    2ebc:	a3 24 0b 00 	l.addic r25,r4,2816
    2ec0:	00 06 ca 08 	l.j 1b56e0 <_end+0x1b0c74>
    2ec4:	37 00 00 00 	*unknown*
    2ec8:	2c 28 0b 00 	*unknown*
    2ecc:	00 08 16 08 	l.j 2086ec <_end+0x203c80>
    2ed0:	38 00 00 00 	*unknown*
    2ed4:	2c 2c 0b 00 	*unknown*
    2ed8:	00 06 a3 08 	l.j 1abaf8 <_end+0x1a708c>
    2edc:	39 00 00 00 	*unknown*
    2ee0:	2c 30 0b 00 	*unknown*
    2ee4:	00 08 52 08 	l.j 217704 <_end+0x212c98>
    2ee8:	3a 00 00 09 	*unknown*
    2eec:	e0 34 00 08 	l.sll r1,r20,r0
    2ef0:	00 00 00 2c 	l.j 2fa0 <or1k_interrupts_disable+0x4>
    2ef4:	00 00 09 f0 	l.j 56b4 <_end+0xc48>
    2ef8:	09 00 00 00 	*unknown*
    2efc:	fe 01 00 0d 	*unknown*
    2f00:	00 00 07 9b 	l.j 4d6c <_end+0x300>
    2f04:	08 09 33 00 	*unknown*
    2f08:	00 0a 15 0b 	l.j 288334 <_end+0x2838c8>
    2f0c:	00 00 07 a3 	l.j 4d98 <_end+0x32c>
    2f10:	09 34 00 00 	*unknown*
    2f14:	08 a3 00 0b 	*unknown*
    2f18:	00 00 08 7a 	l.j 5100 <_end+0x694>
    2f1c:	09 35 00 00 	*unknown*
    2f20:	08 fb 04 00 	*unknown*
    2f24:	1e 00 00 07 	*unknown*
    2f28:	91 0a 9f 00 	l.lbs r8,-24832(r10)
    2f2c:	00 00 b1 00 	l.j 2f32c <_end+0x2a8c0>
    2f30:	00 27 84 00 	l.j 9e3f30 <_end+0x9df4c4>
    2f34:	00 00 98 01 	l.j 28f38 <_end+0x244cc>
    2f38:	9c 00 00 0a 	l.addi r0,r0,10
    2f3c:	a9 1f 00 00 	l.ori r8,r31,0x0
    2f40:	05 a3 01 1f 	l.jal 68c33bc <_end+0x68be950>
    2f44:	00 00 04 49 	l.j 4068 <impure_data+0x14>
    2f48:	00 00 03 68 	l.j 3ce8 <_global_impure_ptr+0x2b4>
    2f4c:	20 66 64 00 	*unknown*
    2f50:	01 1f 00 00 	l.j 47c2f50 <_end+0x47be4e4>
    2f54:	00 25 00 00 	l.j 942f54 <_end+0x93e4e8>
    2f58:	03 89 20 62 	l.j fe24b0e0 <_end+0xfe246674>
    2f5c:	75 66 00 01 	*unknown*
    2f60:	1f 00 00 0a 	*unknown*
    2f64:	a9 00 00 03 	l.ori r8,r0,0x3
    2f68:	aa 1f 00 00 	l.ori r16,r31,0x0
    2f6c:	07 ff 01 1f 	l.jal fffc33e8 <_end+0xfffbe97c>
    2f70:	00 00 00 33 	l.j 303c <_or1k_interrupt_handler+0x20>
    2f74:	00 00 03 cb 	l.j 3ea0 <_global_impure_ptr+0x46c>
    2f78:	21 69 00 01 	*unknown*
    2f7c:	21 00 00 00 	l.trap 0x0
    2f80:	25 00 00 03 	*unknown*
    2f84:	f4 21 62 00 	*unknown*
    2f88:	01 22 00 00 	l.j 4882f88 <_end+0x487e51c>
    2f8c:	05 81 00 00 	l.jal 6042f8c <_end+0x603e520>
    2f90:	03 aa 22 00 	l.j fea8b790 <_end+0xfea86d24>
    2f94:	00 27 c0 00 	l.j 9f2f94 <_end+0x9ee528>
    2f98:	00 0f 14 23 	l.j 3c8024 <_end+0x3c35b8>
    2f9c:	00 00 27 e8 	l.j cf3c <_end+0x84d0>
    2fa0:	00 00 0f 14 	l.j 6bf0 <_end+0x2184>
    2fa4:	00 00 0a 9f 	l.j 5a20 <_end+0xfb4>
    2fa8:	24 01 53 01 	*unknown*
    2fac:	3d 00 22 00 	*unknown*
    2fb0:	00 27 f0 00 	l.j 9fefb0 <_end+0x9fa544>
    2fb4:	00 0f 14 00 	l.j 3c7fb4 <_end+0x3c3548>
    2fb8:	0f 04 00 00 	l.bnf fc102fb8 <_end+0xfc0fe54c>
    2fbc:	0a af 25 26 	*unknown*
    2fc0:	00 00 06 91 	l.j 4a04 <_or1k_exception_handler_table+0x10>
    2fc4:	01 2e 00 00 	l.j 4b82fc4 <_end+0x4b7e558>
    2fc8:	28 1c 00 00 	*unknown*
    2fcc:	00 18 01 9c 	l.j 60363c <_end+0x5febd0>
    2fd0:	00 00 0a dd 	l.j 5b44 <_end+0x10d8>
    2fd4:	20 72 63 00 	*unknown*
    2fd8:	01 2e 00 00 	l.j 4b82fd8 <_end+0x4b7e56c>
    2fdc:	00 25 00 00 	l.j 942fdc <_end+0x93e570>
    2fe0:	04 3d 22 00 	l.jal f4b7e0 <_end+0xf46d74>
    2fe4:	00 28 2c 00 	l.j a0dfe4 <_end+0xa09578>
    2fe8:	00 0f 25 00 	l.j 3cc3e8 <_end+0x3c797c>
    2fec:	1e 00 00 07 	*unknown*
    2ff0:	76 0a 8c 00 	*unknown*
    2ff4:	00 00 25 00 	l.j c3f4 <_end+0x7988>
    2ff8:	00 28 34 00 	l.j a0fff8 <_end+0xa0b58c>
    2ffc:	00 00 20 01 	l.j b000 <_end+0x6594>
    3000:	9c 00 00 0b 	l.addi r0,r0,11
    3004:	13 27 00 00 	l.bf fc9c3004 <_end+0xfc9be598>
    3008:	05 a3 01 35 	l.jal 68c34dc <_end+0x68bea70>
    300c:	00 00 04 49 	l.j 4130 <impure_data+0xdc>
    3010:	01 53 1f 00 	l.j 54cac10 <_end+0x54c61a4>
    3014:	00 08 3a 01 	l.j 211818 <_end+0x20cdac>
    3018:	35 00 00 00 	*unknown*
    301c:	25 00 00 04 	*unknown*
    3020:	5e 00 1e 00 	*unknown*
    3024:	00 06 66 0a 	l.j 19c84c <_end+0x197de0>
    3028:	8d 00 00 00 	l.lbz r8,0(r0)
    302c:	25 00 00 28 	*unknown*
    3030:	54 00 00 00 	*unknown*
    3034:	20 01 9c 00 	*unknown*
    3038:	00 0b 63 27 	l.j 2dbcd4 <_end+0x2d7268>
    303c:	00 00 05 a3 	l.j 46c8 <_or1k_exception_impure_data+0x248>
    3040:	01 3f 00 00 	l.j 4fc3040 <_end+0x4fbe5d4>
    3044:	04 49 01 53 	l.jal 1243590 <_end+0x123eb24>
    3048:	1f 00 00 07 	*unknown*
    304c:	5a 01 3f 00 	*unknown*
    3050:	00 05 b2 00 	l.j 16f850 <_end+0x16ade4>
    3054:	00 04 7f 27 	l.j 122cf0 <_end+0x11e284>
    3058:	00 00 08 d9 	l.j 53bc <_end+0x950>
    305c:	01 3f 00 00 	l.j 4fc305c <_end+0x4fbe5f0>
    3060:	0b 63 01 55 	*unknown*
    3064:	28 65 6e 76 	*unknown*
    3068:	00 01 40 00 	l.j 53068 <_end+0x4e5fc>
    306c:	00 0b 63 01 	l.j 2dbc70 <_end+0x2d7204>
    3070:	56 00 0f 04 	*unknown*
    3074:	00 00 0b 69 	l.j 5e18 <_end+0x13ac>
    3078:	18 00 00 05 	l.movhi r0,0x5
    307c:	81 1e 00 00 	*unknown*
    3080:	06 d4 0a 8f 	l.jal fb505abc <_end+0xfb501050>
    3084:	00 00 00 25 	l.j 3118 <_or1k_libc_impure_init+0x34>
    3088:	00 00 28 74 	l.j d258 <_end+0x87ec>
    308c:	00 00 00 2c 	l.j 313c <_or1k_libc_impure_init+0x58>
    3090:	01 9c 00 00 	l.j 6703090 <_end+0x66fe624>
    3094:	0b a0 1f 00 	*unknown*
    3098:	00 05 a3 01 	l.j 16bc9c <_end+0x167230>
    309c:	47 00 00 04 	*unknown*
    30a0:	49 00 00 04 	*unknown*
    30a4:	a0 22 00 00 	l.addic r1,r2,0
    30a8:	28 84 00 00 	*unknown*
    30ac:	0f 2c 00 1e 	l.bnf fcb03124 <_end+0xfcafe6b8>
    30b0:	00 00 08 82 	l.j 52b8 <_end+0x84c>
    30b4:	0a 90 00 00 	*unknown*
    30b8:	00 25 00 00 	l.j 9430b8 <_end+0x93e64c>
    30bc:	28 a0 00 00 	*unknown*
    30c0:	00 20 01 9c 	l.j 803730 <_end+0x7fecc4>
    30c4:	00 00 0b e2 	l.j 604c <_end+0x15e0>
    30c8:	27 00 00 05 	*unknown*
    30cc:	a3 01 4e 00 	l.addic r24,r1,19968
    30d0:	00 04 49 01 	l.j 1154d4 <_end+0x110a68>
    30d4:	53 1f 00 00 	*unknown*
    30d8:	08 3a 01 4e 	*unknown*
    30dc:	00 00 00 25 	l.j 3170 <_or1k_libc_impure_init+0x8c>
    30e0:	00 00 04 c1 	l.j 43e4 <impure_data+0x390>
    30e4:	28 73 74 00 	*unknown*
    30e8:	01 4e 00 00 	l.j 53830e8 <_end+0x537e67c>
    30ec:	0b e2 01 55 	*unknown*
    30f0:	00 0f 04 00 	l.j 3c40f0 <_end+0x3bf684>
    30f4:	00 09 07 1e 	l.j 244d6c <_end+0x240300>
    30f8:	00 00 07 7f 	l.j 4ef4 <_end+0x488>
    30fc:	0a 91 00 00 	*unknown*
    3100:	00 25 00 00 	l.j 943100 <_end+0x93e694>
    3104:	28 c0 00 00 	*unknown*
    3108:	00 20 01 9c 	l.j 803778 <_end+0x7fed0c>
    310c:	00 00 0c 0f 	l.j 6148 <_end+0x16dc>
    3110:	27 00 00 05 	*unknown*
    3114:	a3 01 55 00 	l.addic r24,r1,21760
    3118:	00 04 49 01 	l.j 11551c <_end+0x110ab0>
    311c:	53 00 1e 00 	*unknown*
    3120:	00 08 6c 01 	l.j 21e124 <_end+0x2196b8>
    3124:	5c 00 00 00 	*unknown*
    3128:	25 00 00 28 	*unknown*
    312c:	e0 00 00 00 	l.add r0,r0,r0
    3130:	20 01 9c 00 	*unknown*
    3134:	00 0c 52 27 	l.j 3179d0 <_end+0x312f64>
    3138:	00 00 05 a3 	l.j 47c4 <_or1k_exception_impure_data+0x344>
    313c:	01 5c 00 00 	l.j 570313c <_end+0x56fe6d0>
    3140:	04 49 01 53 	l.jal 124368c <_end+0x123ec20>
    3144:	1f 00 00 07 	*unknown*
    3148:	9a 01 5c 00 	l.lhs r16,23552(r1)
    314c:	00 0c 52 00 	l.j 31794c <_end+0x312ee0>
    3150:	00 04 e2 27 	l.j 13b9ec <_end+0x136f80>
    3154:	00 00 06 b2 	l.j 4c1c <_end+0x1b0>
    3158:	01 5c 00 00 	l.j 5703158 <_end+0x56fe6ec>
    315c:	01 3c 01 55 	l.j 4f036b0 <_end+0x4efec44>
    3160:	00 0f 04 00 	l.j 3c4160 <_end+0x3bf6f4>
    3164:	00 09 f0 1e 	l.j 27f1dc <_end+0x27a770>
    3168:	00 00 08 30 	l.j 5228 <_end+0x7bc>
    316c:	0a 92 00 00 	*unknown*
    3170:	00 25 00 00 	l.j 943170 <_end+0x93e704>
    3174:	29 00 00 00 	*unknown*
    3178:	00 20 01 9c 	l.j 8037e8 <_end+0x7fed7c>
    317c:	00 00 0c 8e 	l.j 63b4 <_end+0x1948>
    3180:	27 00 00 05 	*unknown*
    3184:	a3 01 63 00 	l.addic r24,r1,25344
    3188:	00 04 49 01 	l.j 11558c <_end+0x110b20>
    318c:	53 1f 00 00 	*unknown*
    3190:	02 9c 01 63 	l.j fa70371c <_end+0xfa6fecb0>
    3194:	00 00 00 25 	l.j 3228 <_or1k_libc_impure_init+0x144>
    3198:	00 00 05 03 	l.j 45a4 <_or1k_exception_impure_data+0x124>
    319c:	00 1e 00 00 	l.j 78319c <_end+0x77e730>
    31a0:	06 dc 0a 93 	l.jal fb705bec <_end+0xfb701180>
    31a4:	00 00 00 25 	l.j 3238 <_or1k_libc_impure_init+0x154>
    31a8:	00 00 29 20 	l.j d628 <_end+0x8bbc>
    31ac:	00 00 00 20 	l.j 322c <_or1k_libc_impure_init+0x148>
    31b0:	01 9c 00 00 	l.j 67031b0 <_end+0x66fe744>
    31b4:	0c d1 27 00 	l.bnf 344cdb4 <_end+0x3448348>
    31b8:	00 05 a3 01 	l.j 16bdbc <_end+0x167350>
    31bc:	6a 00 00 04 	*unknown*
    31c0:	49 01 53 20 	*unknown*
    31c4:	70 69 64 00 	*unknown*
    31c8:	01 6a 00 00 	l.j 5a831c8 <_end+0x5a7e75c>
    31cc:	00 25 00 00 	l.j 9431cc <_end+0x93e760>
    31d0:	05 24 28 73 	l.jal 490d39c <_end+0x4908930>
    31d4:	69 67 00 01 	*unknown*
    31d8:	6a 00 00 00 	*unknown*
    31dc:	25 01 55 00 	*unknown*
    31e0:	1e 00 00 08 	*unknown*
    31e4:	d1 0a 94 00 	*unknown*
    31e8:	00 00 25 00 	l.j c5e8 <_end+0x7b7c>
    31ec:	00 29 40 00 	l.j a531ec <_end+0xa4e780>
    31f0:	00 00 20 01 	l.j b1f4 <_end+0x6788>
    31f4:	9c 00 00 0d 	l.addi r0,r0,13
    31f8:	14 27 00 00 	*unknown*
    31fc:	05 a3 01 71 	l.jal 68c37c0 <_end+0x68bed54>
    3200:	00 00 04 49 	l.j 4324 <impure_data+0x2d0>
    3204:	01 53 1f 00 	l.j 54cae04 <_end+0x54c6398>
    3208:	00 06 5d 01 	l.j 19a60c <_end+0x195ba0>
    320c:	71 00 00 05 	*unknown*
    3210:	b2 00 00 05 	l.muli r16,r0,5
    3214:	45 28 6e 65 	*unknown*
    3218:	77 00 01 71 	*unknown*
    321c:	00 00 05 b2 	l.j 48e4 <_or1k_interrupt_handler_table+0x18>
    3220:	01 55 00 1e 	l.j 5543298 <_end+0x553e82c>
    3224:	00 00 07 27 	l.j 4ec0 <_end+0x454>
    3228:	0a 95 00 00 	*unknown*
    322c:	00 7a 00 00 	l.j 1e8322c <_end+0x1e7e7c0>
    3230:	29 60 00 00 	*unknown*
    3234:	00 2c 01 9c 	l.j b038a4 <_end+0xafee38>
    3238:	00 00 0d 73 	l.j 6804 <_end+0x1d98>
    323c:	1f 00 00 05 	*unknown*
    3240:	a3 01 78 00 	l.addic r24,r1,30720
    3244:	00 04 49 00 	l.j 115644 <_end+0x110bd8>
    3248:	00 05 66 1f 	l.j 15cac4 <_end+0x158058>
    324c:	00 00 02 9c 	l.j 3cbc <_global_impure_ptr+0x288>
    3250:	01 78 00 00 	l.j 5e03250 <_end+0x5dfe7e4>
    3254:	00 25 00 00 	l.j 943254 <_end+0x93e7e8>
    3258:	05 87 20 70 	l.jal 61cb418 <_end+0x61c69ac>
    325c:	74 72 00 01 	*unknown*
    3260:	78 00 00 00 	l.cust3
    3264:	7a 00 00 05 	*unknown*
    3268:	a8 20 64 69 	l.ori r1,r0,0x6469
    326c:	72 00 01 78 	*unknown*
    3270:	00 00 00 25 	l.j 3304 <or1k_critical_begin+0x28>
    3274:	00 00 05 c9 	l.j 4998 <_or1k_interrupt_handler_data_ptr_table+0x4c>
    3278:	22 00 00 29 	*unknown*
    327c:	70 00 00 0f 	*unknown*
    3280:	2c 00 1e 00 	*unknown*
    3284:	00 06 c4 01 	l.j 1b4288 <_end+0x1af81c>
    3288:	7f 00 00 00 	*unknown*
    328c:	25 00 00 29 	*unknown*
    3290:	8c 00 00 00 	l.lbz r0,0(r0)
    3294:	20 01 9c 00 	*unknown*
    3298:	00 0d c3 27 	l.j 373f34 <_end+0x36f4c8>
    329c:	00 00 05 a3 	l.j 4928 <_or1k_interrupt_handler_table+0x5c>
    32a0:	01 7f 00 00 	l.j 5fc32a0 <_end+0x5fbe834>
    32a4:	04 49 01 53 	l.jal 12437f0 <_end+0x123ed84>
    32a8:	1f 00 00 02 	*unknown*
    32ac:	9c 01 7f 00 	l.addi r0,r1,32512
    32b0:	00 05 81 00 	l.j 1636b0 <_end+0x15ec44>
    32b4:	00 05 ea 27 	l.j 17db50 <_end+0x1790e4>
    32b8:	00 00 01 f0 	l.j 3a78 <_global_impure_ptr+0x44>
    32bc:	01 7f 00 00 	l.j 5fc32bc <_end+0x5fbe850>
    32c0:	00 25 01 55 	l.j 943814 <_end+0x93eda8>
    32c4:	27 00 00 0c 	*unknown*
    32c8:	4f 01 7f 00 	*unknown*
    32cc:	00 00 25 01 	l.j c6d0 <_end+0x7c64>
    32d0:	56 00 1e 00 	*unknown*
    32d4:	00 08 4a 0a 	l.j 215afc <_end+0x211090>
    32d8:	98 00 00 00 	l.lhs r0,0(r0)
    32dc:	b1 00 00 29 	l.muli r8,r0,41
    32e0:	ac 00 00 00 	l.xori r0,r0,0
    32e4:	20 01 9c 00 	*unknown*
    32e8:	00 0e 13 27 	l.j 387f84 <_end+0x383518>
    32ec:	00 00 05 a3 	l.j 4978 <_or1k_interrupt_handler_data_ptr_table+0x2c>
    32f0:	01 86 00 00 	l.j 61832f0 <_end+0x617e884>
    32f4:	04 49 01 53 	l.jal 1243840 <_end+0x123edd4>
    32f8:	1f 00 00 02 	*unknown*
    32fc:	9c 01 86 00 	l.addi r0,r1,-31232
    3300:	00 00 25 00 	l.j c700 <_end+0x7c94>
    3304:	00 06 0b 28 	l.j 185fa4 <_end+0x181538>
    3308:	70 74 72 00 	*unknown*
    330c:	01 86 00 00 	l.j 618330c <_end+0x617e8a0>
    3310:	01 3c 01 55 	l.j 4f03864 <_end+0x4efedf8>
    3314:	28 6c 65 6e 	*unknown*
    3318:	00 01 86 00 	l.j 64b18 <_end+0x600ac>
    331c:	00 00 33 01 	l.j ff20 <_end+0xb4b4>
    3320:	56 00 1e 00 	*unknown*
    3324:	00 07 ec 01 	l.j 1fe328 <_end+0x1f98bc>
    3328:	8d 00 00 00 	l.lbz r8,0(r0)
    332c:	25 00 00 29 	*unknown*
    3330:	cc 00 00 00 	l.swa 0(r0),r0
    3334:	20 01 9c 00 	*unknown*
    3338:	00 0e 63 27 	l.j 39bfd4 <_end+0x397568>
    333c:	00 00 05 a3 	l.j 49c8 <_or1k_interrupt_handler_data_ptr_table+0x7c>
    3340:	01 8d 00 00 	l.j 6343340 <_end+0x633e8d4>
    3344:	04 49 01 53 	l.jal 1243890 <_end+0x123ee24>
    3348:	1f 00 00 06 	*unknown*
    334c:	9e 01 8d 00 	l.addi r16,r1,-29440
    3350:	00 05 b2 00 	l.j 16fb50 <_end+0x16b0e4>
    3354:	00 06 2c 28 	l.j 18e3f4 <_end+0x189988>
    3358:	62 75 66 00 	*unknown*
    335c:	01 8d 00 00 	l.j 634335c <_end+0x633e8f0>
    3360:	05 81 01 55 	l.jal 60438b4 <_end+0x603ee48>
    3364:	27 00 00 07 	*unknown*
    3368:	1f 01 8d 00 	*unknown*
    336c:	00 00 33 01 	l.j ff70 <_end+0xb504>
    3370:	56 00 1e 00 	*unknown*
    3374:	00 07 66 0a 	l.j 1dcb9c <_end+0x1d8130>
    3378:	9b 00 00 00 	l.lhs r24,0(r0)
    337c:	25 00 00 29 	*unknown*
    3380:	ec 00 00 00 	*unknown*
    3384:	20 01 9c 00 	*unknown*
    3388:	00 0e a6 27 	l.j 3acc24 <_end+0x3a81b8>
    338c:	00 00 05 a3 	l.j 4a18 <_or1k_exception_handler_table+0x24>
    3390:	01 94 00 00 	l.j 6503390 <_end+0x64fe924>
    3394:	04 49 01 53 	l.jal 12438e0 <_end+0x123ee74>
    3398:	1f 00 00 06 	*unknown*
    339c:	9e 01 94 00 	l.addi r16,r1,-27648
    33a0:	00 05 b2 00 	l.j 16fba0 <_end+0x16b134>
    33a4:	00 06 4d 28 	l.j 196844 <_end+0x191dd8>
    33a8:	62 75 66 00 	*unknown*
    33ac:	01 94 00 00 	l.j 65033ac <_end+0x64fe940>
    33b0:	0b e2 01 55 	*unknown*
    33b4:	00 1e 00 00 	l.j 7833b4 <_end+0x77e948>
    33b8:	08 e7 0a 9d 	*unknown*
    33bc:	00 00 00 25 	l.j 3450 <or1k_timer_init+0x7c>
    33c0:	00 00 2a 0c 	l.j dbf0 <_end+0x9184>
    33c4:	00 00 00 20 	l.j 3444 <or1k_timer_init+0x70>
    33c8:	01 9c 00 00 	l.j 67033c8 <_end+0x66fe95c>
    33cc:	0e dc 27 00 	l.bnf fb70cfcc <_end+0xfb708560>
    33d0:	00 05 a3 01 	l.j 16bfd4 <_end+0x167568>
    33d4:	9b 00 00 04 	l.lhs r24,4(r0)
    33d8:	49 01 53 1f 	*unknown*
    33dc:	00 00 06 9e 	l.j 4e54 <_end+0x3e8>
    33e0:	01 9b 00 00 	l.j 66c33e0 <_end+0x66be974>
    33e4:	05 b2 00 00 	l.jal 6c833e4 <_end+0x6c7e978>
    33e8:	06 6e 00 08 	l.jal f9b83408 <_end+0xf9b7e99c>
    33ec:	00 00 05 81 	l.j 49f0 <_or1k_stack_bottom>
    33f0:	00 00 0e ec 	l.j 6fa0 <_end+0x2534>
    33f4:	09 00 00 00 	*unknown*
    33f8:	fe 00 00 29 	*unknown*
    33fc:	00 00 08 2a 	l.j 54a4 <_end+0xa38>
    3400:	01 3b 00 00 	l.j 4ec3400 <_end+0x4ebe994>
    3404:	0e dc 05 03 	l.bnf fb704810 <_end+0xfb6ffda4>
    3408:	00 00 48 c4 	l.j 15718 <_end+0x10cac>
    340c:	29 00 00 07 	*unknown*
    3410:	6e 01 3c 00 	l.lwa r16,15360(r1)
    3414:	00 0f 0e 05 	l.j 3c6c28 <_end+0x3c21bc>
    3418:	03 00 00 44 	l.j fc003528 <_end+0xfbffeabc>
    341c:	78 0f 04 00 	*unknown*
    3420:	00 05 81 2a 	l.j 1638c8 <_end+0x15ee5c>
    3424:	00 00 08 8b 	l.j 5650 <_end+0xbe4>
    3428:	01 1c 00 00 	l.j 4703428 <_end+0x46fe9bc>
    342c:	0f 25 14 00 	l.bnf fc94842c <_end+0xfc9439c0>
    3430:	00 05 87 00 	l.j 165030 <_end+0x1605c4>
    3434:	2b 00 00 06 	*unknown*
    3438:	86 0b 1f 2c 	l.lwz r16,7980(r11)
    343c:	00 00 06 bc 	l.j 4f2c <_end+0x4c0>
    3440:	0c 0f 00 00 	l.bnf 3c3440 <_end+0x3be9d4>
    3444:	0f 37 0f 04 	l.bnf fcdc7054 <_end+0xfcdc25e8>
    3448:	00 00 00 25 	l.j 34dc <or1k_timer_set_period+0x54>
    344c:	00 00 00 02 	l.j 3454 <or1k_timer_init+0x80>
    3450:	0a 00 04 00 	*unknown*
    3454:	00 0a 9f 04 	l.j 2ab064 <_end+0x2a65f8>
    3458:	01 00 00 08 	l.j 4003478 <_end+0x3ffea0c>
    345c:	99 01 00 00 	l.lhs r8,0(r1)
    3460:	09 34 00 00 	*unknown*
    3464:	07 aa 00 00 	l.jal fea83464 <_end+0xfea7e9f8>
    3468:	2a 2c 00 00 	*unknown*
    346c:	01 b4 00 00 	l.j 6d0346c <_end+0x6cfea00>
    3470:	09 3c 02 01 	*unknown*
    3474:	06 00 00 00 	l.jal f8003474 <_end+0xf7ffea08>
    3478:	7b 03 00 00 	*unknown*
    347c:	08 f7 02 1d 	*unknown*
    3480:	00 00 00 37 	l.j 355c <or1k_timer_set_mode+0x44>
    3484:	02 01 08 00 	l.j f8045484 <_end+0xf8040a18>
    3488:	00 00 79 02 	l.j 21890 <_end+0x1ce24>
    348c:	02 05 00 00 	l.j f814348c <_end+0xf813ea20>
    3490:	01 2f 03 00 	l.j 4bc4090 <_end+0x4bbf624>
    3494:	00 09 86 02 	l.j 264c9c <_end+0x260230>
    3498:	2b 00 00 00 	*unknown*
    349c:	50 02 02 07 	*unknown*
    34a0:	00 00 00 0e 	l.j 34d8 <or1k_timer_set_period+0x50>
    34a4:	02 04 05 00 	l.j f81048a4 <_end+0xf80ffe38>
    34a8:	00 00 05 03 	l.j 48b4 <object.2876+0x8>
    34ac:	00 00 09 91 	l.j 5af0 <_end+0x1084>
    34b0:	02 41 00 00 	l.j f90434b0 <_end+0xf903ea44>
    34b4:	00 69 02 04 	l.j 1a43cc4 <_end+0x1a3f258>
    34b8:	07 00 00 00 	l.jal fc0034b8 <_end+0xfbffea4c>
    34bc:	5e 02 08 05 	*unknown*
    34c0:	00 00 00 00 	l.j 34c0 <or1k_timer_set_period+0x38>
    34c4:	02 08 07 00 	l.j f82050c4 <_end+0xf8200658>
    34c8:	00 00 59 03 	l.j 198d4 <_end+0x14e68>
    34cc:	00 00 08 f9 	l.j 58b0 <_end+0xe44>
    34d0:	03 14 00 00 	l.j fc5034d0 <_end+0xfc4fea64>
    34d4:	00 2c 03 00 	l.j b040d4 <_end+0xaff668>
    34d8:	00 09 88 03 	l.j 2654e4 <_end+0x260a78>
    34dc:	1a 00 00 00 	l.movhi r16,0x0
    34e0:	45 03 00 00 	*unknown*
    34e4:	09 93 03 20 	*unknown*
    34e8:	00 00 00 5e 	l.j 3660 <or1k_timer_reset+0x4>
    34ec:	04 04 05 69 	l.jal 104a90 <_end+0x100024>
    34f0:	6e 74 00 02 	l.lwa r19,2(r20)
    34f4:	04 07 00 00 	l.jal 1c34f4 <_end+0x1bea88>
    34f8:	00 63 03 00 	l.j 18c40f8 <_end+0x18bf68c>
    34fc:	00 0a 06 04 	l.j 284d0c <_end+0x2802a0>
    3500:	55 00 00 00 	*unknown*
    3504:	b8 05 04 00 	*unknown*
    3508:	00 00 be 06 	l.j 32d20 <_end+0x2e2b4>
    350c:	00 00 00 c9 	l.j 3830 <memset+0xec>
    3510:	07 00 00 00 	l.jal fc003510 <_end+0xfbffeaa4>
    3514:	c9 00 08 04 	*unknown*
    3518:	09 00 00 09 	*unknown*
    351c:	17 01 5d 00 	*unknown*
    3520:	00 2a 2c 00 	l.j a8e520 <_end+0xa89ab4>
    3524:	00 00 28 01 	l.j d528 <_end+0x8abc>
    3528:	9c 00 00 00 	l.addi r0,r0,0
    352c:	fb 0a 00 00 	*unknown*
    3530:	0c 6b 01 5d 	l.bnf 1ac3aa4 <_end+0x1abf038>
    3534:	00 00 00 94 	l.j 3784 <memset+0x40>
    3538:	00 00 06 8f 	l.j 4f74 <_end+0x508>
    353c:	0b 69 69 72 	*unknown*
    3540:	00 01 5f 00 	l.j 5b140 <_end+0x566d4>
    3544:	00 00 7e 00 	l.j 22d44 <_end+0x1e2d8>
    3548:	0c 00 00 09 	l.bnf 356c <or1k_timer_enable>
    354c:	c3 01 6d 00 	l.mtspr r1,r13,0xc500
    3550:	00 00 9f 00 	l.j 2b150 <_end+0x266e4>
    3554:	00 2a 54 00 	l.j a98554 <_end+0xa93ae8>
    3558:	00 00 d8 01 	l.j 3955c <_end+0x34af0>
    355c:	9c 00 00 01 	l.addi r0,r0,1
    3560:	24 0d 00 00 	*unknown*
    3564:	09 fe 01 6f 	*unknown*
    3568:	00 00 00 89 	l.j 378c <memset+0x48>
    356c:	00 00 06 b0 	l.j 502c <_end+0x5c0>
    3570:	00 09 00 00 	l.j 243570 <_end+0x23eb04>
    3574:	09 9c 01 90 	*unknown*
    3578:	00 00 2b 2c 	l.j e228 <_end+0x97bc>
    357c:	00 00 00 48 	l.j 369c <or1k_timer_get_ticks>
    3580:	01 9c 00 00 	l.j 6703580 <_end+0x66feb14>
    3584:	01 47 0e 63 	l.j 51c6f10 <_end+0x51c24a4>
    3588:	00 01 90 00 	l.j 67588 <_end+0x62b1c>
    358c:	00 01 47 00 	l.j 5518c <_end+0x50720>
    3590:	00 06 c3 00 	l.j 1b4190 <_end+0x1af724>
    3594:	02 01 06 00 	l.j f8044d94 <_end+0xf8040328>
    3598:	00 00 82 09 	l.j 23dbc <_end+0x1f350>
    359c:	00 00 09 ad 	l.j 5c50 <_end+0x11e4>
    35a0:	01 99 00 00 	l.j 66435a0 <_end+0x663eb34>
    35a4:	2b 74 00 00 	*unknown*
    35a8:	00 6c 01 9c 	l.j 1b03c18 <_end+0x1aff1ac>
    35ac:	00 00 01 97 	l.j 3c08 <_global_impure_ptr+0x1d4>
    35b0:	0e 63 62 00 	l.bnf f98dbdb0 <_end+0xf98d7344>
    35b4:	01 99 00 00 	l.j 66435b4 <_end+0x663eb48>
    35b8:	01 a2 00 00 	l.j 68835b8 <_end+0x687eb4c>
    35bc:	06 e4 0f 00 	l.jal fb9071bc <_end+0xfb902750>
    35c0:	00 2b c4 00 	l.j af45c0 <_end+0xaefb54>
    35c4:	00 01 e5 00 	l.j 7c9c4 <_end+0x77f58>
    35c8:	00 01 8d 10 	l.j 66a08 <_end+0x61f9c>
    35cc:	01 55 01 30 	l.j 5543a8c <_end+0x553f020>
    35d0:	10 01 54 05 	l.bf 585e4 <_end+0x53b78>
    35d4:	03 00 00 2a 	l.j fc00367c <_end+0xfbffec10>
    35d8:	2c 00 11 00 	*unknown*
    35dc:	00 2b cc 00 	l.j af65dc <_end+0xaf1b70>
    35e0:	00 02 00 00 	l.j 835e0 <_end+0x7eb74>
    35e4:	06 00 00 01 	l.jal f80035e8 <_end+0xf7ffeb7c>
    35e8:	a2 07 00 00 	l.addic r16,r7,0
    35ec:	01 47 00 05 	l.j 51c3600 <_end+0x51beb94>
    35f0:	04 00 00 01 	l.jal 35f4 <or1k_timer_disable+0x28>
    35f4:	97 12 00 00 	l.lhz r24,0(r18)
    35f8:	09 d3 05 19 	*unknown*
    35fc:	00 00 00 94 	l.j 384c <memset+0x108>
    3600:	12 00 00 09 	l.bf f8003624 <_end+0xf7ffebb8>
    3604:	e8 05 1b 00 	*unknown*
    3608:	00 00 94 12 	l.j 28650 <_end+0x23be4>
    360c:	00 00 09 01 	l.j 5a10 <_end+0xfa4>
    3610:	05 1c 00 00 	l.jal 4703610 <_end+0x46feba4>
    3614:	00 94 12 00 	l.j 2507e14 <_end+0x25033a8>
    3618:	00 0a 22 05 	l.j 28be2c <_end+0x2873c0>
    361c:	1d 00 00 00 	*unknown*
    3620:	94 13 00 00 	l.lhz r0,0(r19)
    3624:	0a 37 01 57 	*unknown*
    3628:	00 00 01 a2 	l.j 3cb0 <_global_impure_ptr+0x27c>
    362c:	05 03 00 00 	l.jal 40c362c <_end+0x40bebc0>
    3630:	48 c8 14 00 	*unknown*
    3634:	00 0a 4a 04 	l.j 295e44 <_end+0x2913d8>
    3638:	60 00 00 02 	*unknown*
    363c:	00 07 00 00 	l.j 1c363c <_end+0x1bebd0>
    3640:	00 94 07 00 	l.j 2505240 <_end+0x25007d4>
    3644:	00 00 ad 07 	l.j 2ea60 <_end+0x29ff4>
    3648:	00 00 00 c9 	l.j 396c <__udivsi3+0xc0>
    364c:	00 15 00 00 	l.j 54364c <_end+0x53ebe0>
    3650:	09 70 04 6c 	*unknown*
    3654:	07 00 00 00 	l.jal fc003654 <_end+0xfbffebe8>
    3658:	9f 00 00 00 	l.addi r24,r0,0
    365c:	00 02 f0 00 	l.j bf65c <_end+0xbabf0>
    3660:	04 00 00 0b 	l.jal 368c <or1k_timer_reset+0x30>
    3664:	d0 04 01 00 	*unknown*
    3668:	00 08 99 01 	l.j 229a6c <_end+0x225000>
    366c:	00 00 0a 65 	l.j 6000 <_end+0x1594>
    3670:	00 00 07 aa 	l.j 5518 <_end+0xaac>
    3674:	00 00 2f 38 	l.j f354 <_end+0xa8e8>
    3678:	00 00 00 e4 	l.j 3a08 <call___do_global_ctors_aux+0xc>
    367c:	00 00 0b 08 	l.j 629c <_end+0x1830>
    3680:	02 01 06 00 	l.j f8044e80 <_end+0xf8040414>
    3684:	00 00 7b 02 	l.j 2228c <_end+0x1d820>
    3688:	01 08 00 00 	l.j 4203688 <_end+0x41fec1c>
    368c:	00 79 02 02 	l.j 1e43e94 <_end+0x1e3f428>
    3690:	05 00 00 01 	l.jal 4003694 <_end+0x3ffec28>
    3694:	2f 02 02 07 	*unknown*
    3698:	00 00 00 0e 	l.j 36d0 <or1k_timer_reset_ticks+0x14>
    369c:	02 04 05 00 	l.j f8104a9c <_end+0xf8100030>
    36a0:	00 00 05 03 	l.j 4aac <_end+0x40>
    36a4:	00 00 09 91 	l.j 5ce8 <_end+0x127c>
    36a8:	03 41 00 00 	l.j fd0436a8 <_end+0xfd03ec3c>
    36ac:	00 53 02 04 	l.j 14c3ebc <_end+0x14bf450>
    36b0:	07 00 00 00 	l.jal fc0036b0 <_end+0xfbffec44>
    36b4:	5e 02 08 05 	*unknown*
    36b8:	00 00 00 00 	l.j 36b8 <or1k_timer_get_ticks+0x1c>
    36bc:	02 08 07 00 	l.j f82052bc <_end+0xf8200850>
    36c0:	00 00 59 03 	l.j 19acc <_end+0x15060>
    36c4:	00 00 09 93 	l.j 5d10 <_end+0x12a4>
    36c8:	04 20 00 00 	l.jal 8036c8 <_end+0x7fec5c>
    36cc:	00 48 04 04 	l.j 12046dc <_end+0x11ffc70>
    36d0:	05 69 6e 74 	l.jal 5a5f0a0 <_end+0x5a5a634>
    36d4:	00 02 04 07 	l.j 846f0 <_end+0x7fc84>
    36d8:	00 00 00 63 	l.j 3864 <memset+0x120>
    36dc:	03 00 00 0a 	l.j fc003704 <_end+0xfbffec98>
    36e0:	06 02 55 00 	l.jal f8098ae0 <_end+0xf8094074>
    36e4:	00 00 8c 05 	l.j 266f8 <_end+0x21c8c>
    36e8:	04 00 00 00 	l.jal 36e8 <_or1k_board_mem_base>
    36ec:	92 06 00 00 	l.lbs r16,0(r6)
    36f0:	00 9d 07 00 	l.j 27452f0 <_end+0x2740884>
    36f4:	00 00 9d 00 	l.j 2aaf4 <_end+0x26088>
    36f8:	08 04 02 04 	*unknown*
    36fc:	07 00 00 01 	l.jal fc003700 <_end+0xfbffec94>
    3700:	45 09 00 00 	*unknown*
    3704:	00 9d 00 00 	l.j 2743704 <_end+0x273ec98>
    3708:	00 b6 0a 00 	l.j 2d85f08 <_end+0x2d8149c>
    370c:	00 00 9f 1f 	l.j 2b388 <_end+0x2691c>
    3710:	00 02 01 06 	l.j 83b28 <_end+0x7f0bc>
    3714:	00 00 00 82 	l.j 391c <__udivsi3+0x70>
    3718:	03 00 00 0b 	l.j fc003744 <_end+0xfbffecd8>
    371c:	63 05 22 00 	*unknown*
    3720:	00 00 c8 09 	l.j 35744 <_end+0x30cd8>
    3724:	00 00 00 81 	l.j 3928 <__udivsi3+0x7c>
    3728:	00 00 00 d8 	l.j 3a88 <_global_impure_ptr+0x54>
    372c:	0a 00 00 00 	*unknown*
    3730:	9f 1f 00 03 	l.addi r24,r31,3
    3734:	00 00 0b 3b 	l.j 6420 <_end+0x19b4>
    3738:	05 23 00 00 	l.jal 48c3738 <_end+0x48beccc>
    373c:	00 a6 0b 00 	l.j 298633c <_end+0x29818d0>
    3740:	00 0b 30 02 	l.j 2cf748 <_end+0x2cacdc>
    3744:	01 02 00 00 	l.j 4083744 <_end+0x407ecd8>
    3748:	00 68 03 00 	l.j 1a04348 <_end+0x19ff8dc>
    374c:	00 01 0d 0c 	l.j 46b7c <_end+0x42110>
    3750:	73 70 72 00 	*unknown*
    3754:	02 01 02 00 	l.j f8043f54 <_end+0xf803f4e8>
    3758:	00 00 68 0d 	l.j 1d78c <_end+0x18d20>
    375c:	00 00 04 1b 	l.j 47c8 <_or1k_exception_impure_data+0x348>
    3760:	02 01 03 00 	l.j f8044360 <_end+0xf803f8f4>
    3764:	00 00 68 00 	l.j 1d764 <_end+0x18cf8>
    3768:	0e 00 00 0b 	l.bnf f8003794 <_end+0xf7ffed28>
    376c:	25 02 f5 03 	*unknown*
    3770:	00 00 01 30 	l.j 3c30 <_global_impure_ptr+0x1fc>
    3774:	0f 73 70 72 	l.bnf fdcdf93c <_end+0xfdcdaed0>
    3778:	00 02 f5 00 	l.j c0b78 <_end+0xbc10c>
    377c:	00 00 68 10 	l.j 1d7bc <_end+0x18d50>
    3780:	00 00 04 1b 	l.j 47ec <_or1k_exception_impure_data+0x36c>
    3784:	02 f5 00 00 	l.j fbd43784 <_end+0xfbd3ed18>
    3788:	00 68 00 11 	l.j 1a037cc <_end+0x19fed60>
    378c:	00 00 0a 4a 	l.j 60b4 <_end+0x1648>
    3790:	01 20 00 00 	l.j 4803790 <_end+0x47fed24>
    3794:	2f 38 00 00 	*unknown*
    3798:	00 40 01 9c 	l.j 1003e08 <_end+0xfff39c>
    379c:	00 00 01 6e 	l.j 3d54 <_global_impure_ptr+0x320>
    37a0:	12 69 64 00 	l.bf f9a5c7a0 <_end+0xf9a57d34>
    37a4:	01 20 00 00 	l.j 48037a4 <_end+0x47fed38>
    37a8:	00 68 00 00 	l.j 1a037a8 <_end+0x19fed3c>
    37ac:	07 05 13 00 	l.jal fc1483ac <_end+0xfc143940>
    37b0:	00 0e 81 01 	l.j 3a3bb4 <_end+0x39f148>
    37b4:	21 00 00 00 	l.trap 0x0
    37b8:	81 01 54 13 	*unknown*
    37bc:	00 00 0a a8 	l.j 625c <_end+0x17f0>
    37c0:	01 22 00 00 	l.j 48837c0 <_end+0x487ed54>
    37c4:	00 9d 01 55 	l.j 2743d18 <_end+0x273f2ac>
    37c8:	00 11 00 00 	l.j 4437c8 <_end+0x43ed5c>
    37cc:	0b 0e 01 2e 	*unknown*
    37d0:	00 00 2f 78 	l.j f5b0 <_end+0xab44>
    37d4:	00 00 00 24 	l.j 3864 <memset+0x120>
    37d8:	01 9c 00 00 	l.j 67037d8 <_end+0x66fed6c>
    37dc:	01 da 14 73 	l.j 76889a8 <_end+0x7683f3c>
    37e0:	72 00 01 30 	*unknown*
    37e4:	00 00 00 68 	l.j 3984 <__udivsi3+0xd8>
    37e8:	00 00 07 26 	l.j 5480 <_end+0xa14>
    37ec:	15 00 00 00 	l.nop 0x0
    37f0:	e3 00 00 2f 	*unknown*
    37f4:	7c 00 00 00 	l.cust4
    37f8:	20 01 30 00 	*unknown*
    37fc:	00 01 ba 16 	l.j 72054 <_end+0x6d5e8>
    3800:	00 00 00 f4 	l.j 3bd0 <_global_impure_ptr+0x19c>
    3804:	11 17 00 00 	l.bf 45c3804 <_end+0x45bed98>
    3808:	00 20 18 00 	l.j 809808 <_end+0x804d9c>
    380c:	00 01 00 00 	l.j 4380c <_end+0x3eda0>
    3810:	00 07 48 00 	l.j 1d5810 <_end+0x1d0da4>
    3814:	00 19 00 00 	l.j 643814 <_end+0x63eda8>
    3818:	01 0d 00 00 	l.j 4343818 <_end+0x433edac>
    381c:	2f 8c 00 00 	*unknown*
    3820:	00 04 01 32 	l.j 103ce8 <_end+0xff27c>
    3824:	1a 00 00 01 	l.movhi r16,0x1
    3828:	24 00 00 07 	*unknown*
    382c:	26 16 00 00 	*unknown*
    3830:	01 19 11 00 	l.j 4647c30 <_end+0x46431c4>
    3834:	00 1b 00 00 	l.j 6c3834 <_end+0x6bedc8>
    3838:	0a d8 01 36 	*unknown*
    383c:	00 00 00 68 	l.j 39dc <__do_global_ctors_aux+0x34>
    3840:	00 00 2f 9c 	l.j f6b0 <_end+0xac44>
    3844:	00 00 00 38 	l.j 3924 <__udivsi3+0x78>
    3848:	01 9c 00 00 	l.j 6703848 <_end+0x66feddc>
    384c:	02 56 1c 00 	l.j f958a84c <_end+0xf9585de0>
    3850:	00 0a a2 01 	l.j 2ac054 <_end+0x2a75e8>
    3854:	38 00 00 00 	*unknown*
    3858:	68 1d 00 00 	*unknown*
    385c:	0b a1 01 38 	*unknown*
    3860:	00 00 00 68 	l.j 3a00 <call___do_global_ctors_aux+0x4>
    3864:	00 00 07 5b 	l.j 55d0 <_end+0xb64>
    3868:	15 00 00 00 	l.nop 0x0
    386c:	e3 00 00 2f 	*unknown*
    3870:	a4 00 00 00 	l.andi r0,r0,0x0
    3874:	38 01 39 00 	*unknown*
    3878:	00 02 36 16 	l.j 910d0 <_end+0x8c664>
    387c:	00 00 00 f4 	l.j 3c4c <_global_impure_ptr+0x218>
    3880:	11 17 00 00 	l.bf 45c3880 <_end+0x45bee14>
    3884:	00 38 18 00 	l.j e09884 <_end+0xe04e18>
    3888:	00 01 00 00 	l.j 43888 <_end+0x3ee1c>
    388c:	00 07 7e 00 	l.j 1e308c <_end+0x1de620>
    3890:	00 19 00 00 	l.j 643890 <_end+0x63ee24>
    3894:	01 0d 00 00 	l.j 4343894 <_end+0x433ee28>
    3898:	2f b8 00 00 	*unknown*
    389c:	00 04 01 3b 	l.j 103d88 <_end+0xff31c>
    38a0:	1a 00 00 01 	l.movhi r16,0x1
    38a4:	24 00 00 07 	*unknown*
    38a8:	5b 16 00 00 	*unknown*
    38ac:	01 19 11 00 	l.j 4647cac <_end+0x4643240>
    38b0:	00 11 00 00 	l.j 4438b0 <_end+0x43ee44>
    38b4:	0b 82 01 40 	*unknown*
    38b8:	00 00 2f d4 	l.j f808 <_end+0xad9c>
    38bc:	00 00 00 48 	l.j 39dc <__do_global_ctors_aux+0x34>
    38c0:	01 9c 00 00 	l.j 67038c0 <_end+0x66fee54>
    38c4:	02 d1 1e 00 	l.j fb44b0c4 <_end+0xfb446658>
    38c8:	00 0b 9a 01 	l.j 2ea0cc <_end+0x2e5660>
    38cc:	40 00 00 00 	*unknown*
    38d0:	68 00 00 07 	*unknown*
    38d4:	91 14 73 72 	l.lbs r8,29554(r20)
    38d8:	00 01 42 00 	l.j 540d8 <_end+0x4f66c>
    38dc:	00 00 68 00 	l.j 1d8dc <_end+0x18e70>
    38e0:	00 07 b2 15 	l.j 1f0134 <_end+0x1eb6c8>
    38e4:	00 00 00 e3 	l.j 3c70 <_global_impure_ptr+0x23c>
    38e8:	00 00 2f dc 	l.j f858 <_end+0xadec>
    38ec:	00 00 00 50 	l.j 3a2c <_fini+0x14>
    38f0:	01 42 00 00 	l.j 50838f0 <_end+0x507ee84>
    38f4:	02 b1 16 00 	l.j fac490f4 <_end+0xfac44688>
    38f8:	00 00 f4 11 	l.j 4093c <_end+0x3bed0>
    38fc:	17 00 00 00 	*unknown*
    3900:	50 18 00 00 	*unknown*
    3904:	01 00 00 00 	l.j 4003904 <_end+0x3ffee98>
    3908:	07 d5 00 00 	l.jal ff543908 <_end+0xff53ee9c>
    390c:	19 00 00 01 	l.movhi r8,0x1
    3910:	0d 00 00 30 	l.bnf 40039d0 <_end+0x3ffef64>
    3914:	04 00 00 00 	l.jal 3914 <__udivsi3+0x68>
    3918:	08 01 44 1a 	*unknown*
    391c:	00 00 01 24 	l.j 3dac <_global_impure_ptr+0x378>
    3920:	00 00 07 b2 	l.j 57e8 <_end+0xd7c>
    3924:	16 00 00 01 	*unknown*
    3928:	19 11 00 00 	*unknown*
    392c:	1f 00 00 0a 	*unknown*
    3930:	f0 01 1c 00 	*unknown*
    3934:	00 00 bd 05 	l.j 32d48 <_end+0x2e2dc>
    3938:	03 00 00 48 	l.j fc003a58 <_end+0xfbffefec>
    393c:	cc 1f 00 00 	l.swa 0(r31),r0
    3940:	0a b1 01 1d 	*unknown*
    3944:	00 00 00 d8 	l.j 3ca4 <_global_impure_ptr+0x270>
    3948:	05 03 00 00 	l.jal 40c3948 <_end+0x40beedc>
    394c:	49 4c 00 00 	*unknown*
    3950:	00 09 8e 00 	l.j 267150 <_end+0x2626e4>
    3954:	04 00 00 0d 	l.jal 3988 <__udivsi3+0xdc>
    3958:	72 04 01 00 	*unknown*
    395c:	00 08 99 01 	l.j 229d60 <_end+0x2252f4>
    3960:	00 00 0b a7 	l.j 67fc <_end+0x1d90>
    3964:	00 00 07 aa 	l.j 580c <_end+0xda0>
    3968:	00 00 30 e4 	l.j fcf8 <_end+0xb28c>
    396c:	00 00 01 ac 	l.j 401c <__EH_FRAME_BEGIN__+0x1c>
    3970:	00 00 0d 1a 	l.j 6dd8 <_end+0x236c>
    3974:	02 04 05 00 	l.j f8104d74 <_end+0xf8100308>
    3978:	00 00 05 02 	l.j 4d80 <_end+0x314>
    397c:	04 07 00 00 	l.jal 1c397c <_end+0x1bef10>
    3980:	00 5e 03 04 	l.j 1784590 <_end+0x177fb24>
    3984:	05 69 6e 74 	l.jal 5a5f354 <_end+0x5a5a8e8>
    3988:	00 02 01 06 	l.j 83da0 <_end+0x7f334>
    398c:	00 00 00 7b 	l.j 3b78 <_global_impure_ptr+0x144>
    3990:	02 01 08 00 	l.j f8045990 <_end+0xf8040f24>
    3994:	00 00 79 02 	l.j 21d9c <_end+0x1d330>
    3998:	02 05 00 00 	l.j f8143998 <_end+0xf813ef2c>
    399c:	01 2f 02 02 	l.j 4bc41a4 <_end+0x4bbf738>
    39a0:	07 00 00 00 	l.jal fc0039a0 <_end+0xfbffef34>
    39a4:	0e 04 00 00 	l.bnf f81039a4 <_end+0xf80fef38>
    39a8:	09 91 02 41 	*unknown*
    39ac:	00 00 00 2c 	l.j 3a5c <_global_impure_ptr+0x28>
    39b0:	02 08 05 00 	l.j f8204db0 <_end+0xf8200344>
    39b4:	00 00 00 02 	l.j 39bc <__do_global_ctors_aux+0x14>
    39b8:	08 07 00 00 	*unknown*
    39bc:	00 59 04 00 	l.j 16449bc <_end+0x163ff50>
    39c0:	00 03 6d 03 	l.j dedcc <_end+0xda360>
    39c4:	07 00 00 00 	l.jal fc0039c4 <_end+0xfbffef58>
    39c8:	33 04 00 00 	*unknown*
    39cc:	03 4b 04 10 	l.j fd2c4a0c <_end+0xfd2bffa0>
    39d0:	00 00 00 25 	l.j 3a64 <_global_impure_ptr+0x30>
    39d4:	04 00 00 04 	l.jal 39e4 <__do_global_ctors_aux+0x3c>
    39d8:	27 04 27 00 	*unknown*
    39dc:	00 00 25 05 	l.j cdf0 <_end+0x8384>
    39e0:	00 00 03 17 	l.j 463c <_or1k_exception_impure_data+0x1bc>
    39e4:	05 01 61 00 	l.jal 405bde4 <_end+0x4057378>
    39e8:	00 00 9c 02 	l.j 2a9f0 <_end+0x25f84>
    39ec:	04 07 00 00 	l.jal 1c39ec <_end+0x1bef80>
    39f0:	00 63 06 04 	l.j 18c5200 <_end+0x18c0794>
    39f4:	04 4a 00 00 	l.jal 12839f4 <_end+0x127ef88>
    39f8:	00 c2 07 00 	l.j 30855f8 <_end+0x3080b8c>
    39fc:	00 03 11 04 	l.j c7e0c <_end+0xc33a0>
    3a00:	4c 00 00 00 	l.maci r0,0
    3a04:	90 07 00 00 	l.lbs r0,0(r7)
    3a08:	02 84 04 4d 	l.j fa104b3c <_end+0xfa1000d0>
    3a0c:	00 00 00 c2 	l.j 3d14 <_global_impure_ptr+0x2e0>
    3a10:	00 08 00 00 	l.j 203a10 <_end+0x1fefa4>
    3a14:	00 41 00 00 	l.j 1043a14 <_end+0x103efa8>
    3a18:	00 d2 09 00 	l.j 3485e18 <_end+0x34813ac>
    3a1c:	00 00 d2 03 	l.j 38228 <_end+0x337bc>
    3a20:	00 02 04 07 	l.j 84a3c <_end+0x7ffd0>
    3a24:	00 00 01 45 	l.j 3f38 <_global_impure_ptr+0x504>
    3a28:	0a 08 04 47 	*unknown*
    3a2c:	00 00 00 fa 	l.j 3e14 <_global_impure_ptr+0x3e0>
    3a30:	0b 00 00 04 	*unknown*
    3a34:	11 04 49 00 	l.bf 4115e34 <_end+0x41113c8>
    3a38:	00 00 33 00 	l.j 10638 <_end+0xbbcc>
    3a3c:	0b 00 00 04 	*unknown*
    3a40:	19 04 4e 00 	*unknown*
    3a44:	00 00 a3 04 	l.j 2c654 <_end+0x27be8>
    3a48:	00 04 00 00 	l.j 103a48 <_end+0xfefdc>
    3a4c:	03 ab 04 4f 	l.j feac4b88 <_end+0xfeac011c>
    3a50:	00 00 00 d9 	l.j 3db4 <_global_impure_ptr+0x380>
    3a54:	04 00 00 02 	l.jal 3a5c <_global_impure_ptr+0x28>
    3a58:	47 04 53 00 	*unknown*
    3a5c:	00 00 6f 0c 	l.j 1f68c <_end+0x1ac20>
    3a60:	04 04 00 00 	l.jal 103a60 <_end+0xfeff4>
    3a64:	04 54 06 16 	l.jal 15052bc <_end+0x1500850>
    3a68:	00 00 00 2c 	l.j 3b18 <_global_impure_ptr+0xe4>
    3a6c:	0d 00 00 02 	l.bnf 4003a74 <_end+0x3fff008>
    3a70:	58 18 06 2d 	*unknown*
    3a74:	00 00 01 70 	l.j 4034 <__CTOR_END__>
    3a78:	0b 00 00 03 	*unknown*
    3a7c:	cc 06 2f 00 	l.swa 1792(r6),r5
    3a80:	00 01 70 00 	l.j 5fa80 <_end+0x5b014>
    3a84:	0e 5f 6b 00 	l.bnf f97de684 <_end+0xf97d9c18>
    3a88:	06 30 00 00 	l.jal f8c03a88 <_end+0xf8bff01c>
    3a8c:	00 33 04 0b 	l.j cc4ab8 <_end+0xcc004c>
    3a90:	00 00 04 03 	l.j 4a9c <_end+0x30>
    3a94:	06 30 00 00 	l.jal f8c03a94 <_end+0xf8bff028>
    3a98:	00 33 08 0b 	l.j cc5ac4 <_end+0xcc1058>
    3a9c:	00 00 02 41 	l.j 43a0 <impure_data+0x34c>
    3aa0:	06 30 00 00 	l.jal f8c03aa0 <_end+0xf8bff034>
    3aa4:	00 33 0c 0b 	l.j cc6ad0 <_end+0xcc2064>
    3aa8:	00 00 04 a0 	l.j 4d28 <_end+0x2bc>
    3aac:	06 30 00 00 	l.jal f8c03aac <_end+0xf8bff040>
    3ab0:	00 33 10 0e 	l.j cc7ae8 <_end+0xcc307c>
    3ab4:	5f 78 00 06 	*unknown*
    3ab8:	31 00 00 01 	*unknown*
    3abc:	76 14 00 0f 	*unknown*
    3ac0:	04 00 00 01 	l.jal 3ac4 <_global_impure_ptr+0x90>
    3ac4:	1d 08 00 00 	*unknown*
    3ac8:	01 12 00 00 	l.j 4483ac8 <_end+0x447f05c>
    3acc:	01 86 09 00 	l.j 6185ecc <_end+0x6181460>
    3ad0:	00 00 d2 00 	l.j 382d0 <_end+0x33864>
    3ad4:	00 0d 00 00 	l.j 343ad4 <_end+0x33f068>
    3ad8:	02 7f 24 06 	l.j f9fccaf0 <_end+0xf9fc8084>
    3adc:	35 00 00 01 	*unknown*
    3ae0:	ff 0b 00 00 	*unknown*
    3ae4:	01 b3 06 37 	l.j 6cc53c0 <_end+0x6cc0954>
    3ae8:	00 00 00 33 	l.j 3bb4 <_global_impure_ptr+0x180>
    3aec:	00 0b 00 00 	l.j 2c3aec <_end+0x2bf080>
    3af0:	04 2f 06 38 	l.jal bc53d0 <_end+0xbc0964>
    3af4:	00 00 00 33 	l.j 3bc0 <_global_impure_ptr+0x18c>
    3af8:	04 0b 00 00 	l.jal 2c3af8 <_end+0x2bf08c>
    3afc:	01 c2 06 39 	l.j 70853e0 <_end+0x7080974>
    3b00:	00 00 00 33 	l.j 3bcc <_global_impure_ptr+0x198>
    3b04:	08 0b 00 00 	*unknown*
    3b08:	05 19 06 3a 	l.jal 46453f0 <_end+0x4640984>
    3b0c:	00 00 00 33 	l.j 3bd8 <_global_impure_ptr+0x1a4>
    3b10:	0c 0b 00 00 	l.bnf 2c3b10 <_end+0x2bf0a4>
    3b14:	03 37 06 3b 	l.j fcdc5400 <_end+0xfcdc0994>
    3b18:	00 00 00 33 	l.j 3be4 <_global_impure_ptr+0x1b0>
    3b1c:	10 0b 00 00 	l.bf 2c3b1c <_end+0x2bf0b0>
    3b20:	03 26 06 3c 	l.j fc985410 <_end+0xfc9809a4>
    3b24:	00 00 00 33 	l.j 3bf0 <_global_impure_ptr+0x1bc>
    3b28:	14 0b 00 00 	*unknown*
    3b2c:	04 a5 06 3d 	l.jal 2945420 <_end+0x29409b4>
    3b30:	00 00 00 33 	l.j 3bfc <_global_impure_ptr+0x1c8>
    3b34:	18 0b 00 00 	*unknown*
    3b38:	03 8d 06 3e 	l.j fe345430 <_end+0xfe3409c4>
    3b3c:	00 00 00 33 	l.j 3c08 <_global_impure_ptr+0x1d4>
    3b40:	1c 0b 00 00 	*unknown*
    3b44:	04 e0 06 3f 	l.jal 3805440 <_end+0x38009d4>
    3b48:	00 00 00 33 	l.j 3c14 <_global_impure_ptr+0x1e0>
    3b4c:	20 00 10 00 	l.sys 0x1000
    3b50:	00 01 d1 01 	l.j 77f54 <_end+0x734e8>
    3b54:	08 06 48 00 	*unknown*
    3b58:	00 02 3f 0b 	l.j 93784 <_end+0x8ed18>
    3b5c:	00 00 02 34 	l.j 442c <impure_data+0x3d8>
    3b60:	06 49 00 00 	l.jal f9243b60 <_end+0xf923f0f4>
    3b64:	02 3f 00 0b 	l.j f8fc3b90 <_end+0xf8fbf124>
    3b68:	00 00 01 4e 	l.j 40a0 <impure_data+0x4c>
    3b6c:	06 4a 00 00 	l.jal f9283b6c <_end+0xf927f100>
    3b70:	02 3f 80 11 	l.j f8fe3bb4 <_end+0xf8fdf148>
    3b74:	00 00 04 4b 	l.j 4ca0 <_end+0x234>
    3b78:	06 4c 00 00 	l.jal f9303b78 <_end+0xf92ff10c>
    3b7c:	01 12 01 00 	l.j 4483f7c <_end+0x447f510>
    3b80:	11 00 00 01 	l.bf 4003b84 <_end+0x3fff118>
    3b84:	f6 06 4f 00 	*unknown*
    3b88:	00 01 12 01 	l.j 4838c <_end+0x43920>
    3b8c:	04 00 08 00 	l.jal 5b8c <_end+0x1120>
    3b90:	00 01 10 00 	l.j 47b90 <_end+0x43124>
    3b94:	00 02 4f 09 	l.j 977b8 <_end+0x92d4c>
    3b98:	00 00 00 d2 	l.j 3ee0 <_global_impure_ptr+0x4ac>
    3b9c:	1f 00 10 00 	*unknown*
    3ba0:	00 01 3d 01 	l.j 52fa4 <_end+0x4e538>
    3ba4:	90 06 5b 00 	l.lbs r0,23296(r6)
    3ba8:	00 02 8d 0b 	l.j a6fd4 <_end+0xa2568>
    3bac:	00 00 03 cc 	l.j 4adc <_end+0x70>
    3bb0:	06 5c 00 00 	l.jal f9703bb0 <_end+0xf96ff144>
    3bb4:	02 8d 00 0b 	l.j fa343be0 <_end+0xfa33f174>
    3bb8:	00 00 03 e4 	l.j 4b48 <_end+0xdc>
    3bbc:	06 5d 00 00 	l.jal f9743bbc <_end+0xf973f150>
    3bc0:	00 33 04 0b 	l.j cc4bec <_end+0xcc0180>
    3bc4:	00 00 02 3c 	l.j 44b4 <_or1k_exception_impure_data+0x34>
    3bc8:	06 5f 00 00 	l.jal f97c3bc8 <_end+0xf97bf15c>
    3bcc:	02 93 08 0b 	l.j fa4c5bf8 <_end+0xfa4c118c>
    3bd0:	00 00 01 d1 	l.j 4314 <impure_data+0x2c0>
    3bd4:	06 60 00 00 	l.jal f9803bd4 <_end+0xf97ff168>
    3bd8:	01 ff 88 00 	l.j 7fe5bd8 <_end+0x7fe116c>
    3bdc:	0f 04 00 00 	l.bnf fc103bdc <_end+0xfc0ff170>
    3be0:	02 4f 08 00 	l.j f93c5be0 <_end+0xf93c1174>
    3be4:	00 02 a3 00 	l.j ac7e4 <_end+0xa7d78>
    3be8:	00 02 a3 09 	l.j ac80c <_end+0xa7da0>
    3bec:	00 00 00 d2 	l.j 3f34 <_global_impure_ptr+0x500>
    3bf0:	1f 00 0f 04 	*unknown*
    3bf4:	00 00 02 a9 	l.j 4698 <_or1k_exception_impure_data+0x218>
    3bf8:	12 0d 00 00 	l.bf f8343bf8 <_end+0xf833f18c>
    3bfc:	03 97 08 06 	l.j fe5c5c14 <_end+0xfe5c11a8>
    3c00:	73 00 00 02 	*unknown*
    3c04:	cf 0b 00 00 	l.swa -16384(r11),r0
    3c08:	09 f8 06 74 	*unknown*
    3c0c:	00 00 02 cf 	l.j 4748 <_or1k_exception_impure_data+0x2c8>
    3c10:	00 0b 00 00 	l.j 2c3c10 <_end+0x2bf1a4>
    3c14:	07 8b 06 75 	l.jal fe2c55e8 <_end+0xfe2c0b7c>
    3c18:	00 00 00 33 	l.j 3ce4 <_global_impure_ptr+0x2b0>
    3c1c:	04 00 0f 04 	l.jal 782c <_end+0x2dc0>
    3c20:	00 00 00 41 	l.j 3d24 <_global_impure_ptr+0x2f0>
    3c24:	0d 00 00 03 	l.bnf 4003c30 <_end+0x3fff1c4>
    3c28:	b6 68 06 b3 	l.mfspr r19,r8,0x6b3
    3c2c:	00 00 03 ff 	l.j 4c28 <_end+0x1bc>
    3c30:	0e 5f 70 00 	l.bnf f97dfc30 <_end+0xf97db1c4>
    3c34:	06 b4 00 00 	l.jal fad03c34 <_end+0xfacff1c8>
    3c38:	02 cf 00 0e 	l.j fb3c3c70 <_end+0xfb3bf204>
    3c3c:	5f 72 00 06 	*unknown*
    3c40:	b5 00 00 00 	l.mfspr r8,r0,0x0
    3c44:	33 04 0e 5f 	*unknown*
    3c48:	77 00 06 b6 	*unknown*
    3c4c:	00 00 00 33 	l.j 3d18 <_global_impure_ptr+0x2e4>
    3c50:	08 0b 00 00 	*unknown*
    3c54:	01 ef 06 b7 	l.j 7bc5730 <_end+0x7bc0cc4>
    3c58:	00 00 00 48 	l.j 3d78 <_global_impure_ptr+0x344>
    3c5c:	0c 0b 00 00 	l.bnf 2c3c5c <_end+0x2bf1f0>
    3c60:	02 9b 06 b8 	l.j fa6c5740 <_end+0xfa6c0cd4>
    3c64:	00 00 00 48 	l.j 3d84 <_global_impure_ptr+0x350>
    3c68:	0e 0e 5f 62 	l.bnf f839b9f0 <_end+0xf8396f84>
    3c6c:	66 00 06 b9 	*unknown*
    3c70:	00 00 02 aa 	l.j 4718 <_or1k_exception_impure_data+0x298>
    3c74:	10 0b 00 00 	l.bf 2c3c74 <_end+0x2bf208>
    3c78:	01 8d 06 ba 	l.j 6345760 <_end+0x6340cf4>
    3c7c:	00 00 00 33 	l.j 3d48 <_global_impure_ptr+0x314>
    3c80:	18 0b 00 00 	*unknown*
    3c84:	01 df 06 c1 	l.j 77c5788 <_end+0x77c0d1c>
    3c88:	00 00 01 10 	l.j 40c8 <impure_data+0x74>
    3c8c:	1c 0b 00 00 	*unknown*
    3c90:	02 6f 06 c3 	l.j f9bc579c <_end+0xf9bc0d30>
    3c94:	00 00 05 62 	l.j 521c <_end+0x7b0>
    3c98:	20 0b 00 00 	*unknown*
    3c9c:	09 a6 06 c5 	*unknown*
    3ca0:	00 00 05 91 	l.j 52e4 <_end+0x878>
    3ca4:	24 0b 00 00 	*unknown*
    3ca8:	04 21 06 c8 	l.jal 8457c8 <_end+0x840d5c>
    3cac:	00 00 05 b5 	l.j 5380 <_end+0x914>
    3cb0:	28 0b 00 00 	*unknown*
    3cb4:	04 fa 06 c9 	l.jal 3e857d8 <_end+0x3e80d6c>
    3cb8:	00 00 05 cf 	l.j 53f4 <_end+0x988>
    3cbc:	2c 0e 5f 75 	*unknown*
    3cc0:	62 00 06 cc 	*unknown*
    3cc4:	00 00 02 aa 	l.j 476c <_or1k_exception_impure_data+0x2ec>
    3cc8:	30 0e 5f 75 	*unknown*
    3ccc:	70 00 06 cd 	*unknown*
    3cd0:	00 00 02 cf 	l.j 480c <_or1k_exception_impure_data+0x38c>
    3cd4:	38 0e 5f 75 	*unknown*
    3cd8:	72 00 06 ce 	*unknown*
    3cdc:	00 00 00 33 	l.j 3da8 <_global_impure_ptr+0x374>
    3ce0:	3c 0b 00 00 	*unknown*
    3ce4:	01 bc 06 d1 	l.j 6f05828 <_end+0x6f00dbc>
    3ce8:	00 00 05 d5 	l.j 543c <_end+0x9d0>
    3cec:	40 0b 00 00 	*unknown*
    3cf0:	04 d2 06 d2 	l.jal 3485838 <_end+0x3480dcc>
    3cf4:	00 00 05 e5 	l.j 5488 <_end+0xa1c>
    3cf8:	43 0e 5f 6c 	*unknown*
    3cfc:	62 00 06 d5 	*unknown*
    3d00:	00 00 02 aa 	l.j 47a8 <_or1k_exception_impure_data+0x328>
    3d04:	44 0b 00 00 	*unknown*
    3d08:	08 18 06 d8 	*unknown*
    3d0c:	00 00 00 33 	l.j 3dd8 <_global_impure_ptr+0x3a4>
    3d10:	4c 0b 00 00 	l.maci r11,0
    3d14:	02 0d 06 d9 	l.j f8345878 <_end+0xf8340e0c>
    3d18:	00 00 00 7a 	l.j 3f00 <_global_impure_ptr+0x4cc>
    3d1c:	50 0b 00 00 	*unknown*
    3d20:	0c 6a 06 dc 	l.bnf 1a85890 <_end+0x1a80e24>
    3d24:	00 00 04 1d 	l.j 4d98 <_end+0x32c>
    3d28:	54 0b 00 00 	*unknown*
    3d2c:	06 4f 06 e0 	l.jal f93c58ac <_end+0xf93c0e40>
    3d30:	00 00 01 05 	l.j 4144 <impure_data+0xf0>
    3d34:	58 0b 00 00 	*unknown*
    3d38:	03 be 06 e2 	l.j fef858c0 <_end+0xfef80e54>
    3d3c:	00 00 00 fa 	l.j 4124 <impure_data+0xd0>
    3d40:	5c 0b 00 00 	*unknown*
    3d44:	03 1e 06 e3 	l.j fc7858d0 <_end+0xfc780e64>
    3d48:	00 00 00 33 	l.j 3e14 <_global_impure_ptr+0x3e0>
    3d4c:	64 00 13 00 	*unknown*
    3d50:	00 00 33 00 	l.j 10950 <_end+0xbee4>
    3d54:	00 04 1d 14 	l.j 10b1a4 <_end+0x106738>
    3d58:	00 00 04 1d 	l.j 4dcc <_end+0x360>
    3d5c:	14 00 00 01 	*unknown*
    3d60:	10 14 00 00 	l.bf 503d60 <_end+0x4ff2f4>
    3d64:	05 55 14 00 	l.jal 5548d64 <_end+0x55442f8>
    3d68:	00 00 33 00 	l.j 10968 <_end+0xbefc>
    3d6c:	0f 04 00 00 	l.bnf fc103d6c <_end+0xfc0ff300>
    3d70:	04 23 15 00 	l.jal 8c9170 <_end+0x8c4704>
    3d74:	00 0c 0b 04 	l.j 306984 <_end+0x301f18>
    3d78:	24 06 02 39 	*unknown*
    3d7c:	00 00 05 55 	l.j 52d0 <_end+0x864>
    3d80:	16 00 00 06 	*unknown*
    3d84:	bd 06 02 3b 	*unknown*
    3d88:	00 00 00 33 	l.j 3e54 <_global_impure_ptr+0x420>
    3d8c:	00 16 00 00 	l.j 583d8c <_end+0x57f320>
    3d90:	01 fe 06 02 	l.j 7f85598 <_end+0x7f80b2c>
    3d94:	40 00 00 06 	*unknown*
    3d98:	3c 04 16 00 	*unknown*
    3d9c:	00 02 8b 06 	l.j a69b4 <_end+0xa1f48>
    3da0:	02 40 00 00 	l.j f9003da0 <_end+0xf8fff334>
    3da4:	06 3c 08 16 	l.jal f8f05dfc <_end+0xf8f01390>
    3da8:	00 00 02 50 	l.j 46e8 <_or1k_exception_impure_data+0x268>
    3dac:	06 02 40 00 	l.jal f8093dac <_end+0xf808f340>
    3db0:	00 06 3c 0c 	l.j 192de0 <_end+0x18e374>
    3db4:	16 00 00 03 	*unknown*
    3db8:	df 06 02 42 	l.sh -15806(r6),r0
    3dbc:	00 00 00 33 	l.j 3e88 <_global_impure_ptr+0x454>
    3dc0:	10 16 00 00 	l.bf 583dc0 <_end+0x57f354>
    3dc4:	01 62 06 02 	l.j 58855cc <_end+0x5880b60>
    3dc8:	43 00 00 08 	*unknown*
    3dcc:	1e 14 16 00 	*unknown*
    3dd0:	00 04 7c 06 	l.j 122de8 <_end+0x11e37c>
    3dd4:	02 45 00 00 	l.j f9143dd4 <_end+0xf913f368>
    3dd8:	00 33 30 16 	l.j ccfe30 <_end+0xccb3c4>
    3ddc:	00 00 03 e9 	l.j 4d80 <_end+0x314>
    3de0:	06 02 46 00 	l.jal f80955e0 <_end+0xf8090b74>
    3de4:	00 05 86 34 	l.j 1656b4 <_end+0x160c48>
    3de8:	16 00 00 03 	*unknown*
    3dec:	40 06 02 48 	*unknown*
    3df0:	00 00 00 33 	l.j 3ebc <_global_impure_ptr+0x488>
    3df4:	38 16 00 00 	*unknown*
    3df8:	03 f9 06 02 	l.j ffe45600 <_end+0xffe40b94>
    3dfc:	4a 00 00 08 	*unknown*
    3e00:	39 3c 16 00 	*unknown*
    3e04:	00 03 09 06 	l.j c621c <_end+0xc17b0>
    3e08:	02 4d 00 00 	l.j f9343e08 <_end+0xf933f39c>
    3e0c:	01 70 40 16 	l.j 5c13e64 <_end+0x5c0f3f8>
    3e10:	00 00 02 75 	l.j 47e4 <_or1k_exception_impure_data+0x364>
    3e14:	06 02 4e 00 	l.jal f8097614 <_end+0xf8092ba8>
    3e18:	00 00 33 44 	l.j 10b28 <_end+0xc0bc>
    3e1c:	16 00 00 05 	*unknown*
    3e20:	14 06 02 4f 	*unknown*
    3e24:	00 00 01 70 	l.j 43e4 <impure_data+0x390>
    3e28:	48 16 00 00 	*unknown*
    3e2c:	03 63 06 02 	l.j fd8c5634 <_end+0xfd8c0bc8>
    3e30:	50 00 00 08 	*unknown*
    3e34:	3f 4c 16 00 	*unknown*
    3e38:	00 02 93 06 	l.j a8a50 <_end+0xa3fe4>
    3e3c:	02 53 00 00 	l.j f94c3e3c <_end+0xf94bf3d0>
    3e40:	00 33 50 16 	l.j cd7e98 <_end+0xcd342c>
    3e44:	00 00 02 05 	l.j 4658 <_or1k_exception_impure_data+0x1d8>
    3e48:	06 02 54 00 	l.jal f8098e48 <_end+0xf80943dc>
    3e4c:	00 05 55 54 	l.j 15939c <_end+0x154930>
    3e50:	16 00 00 03 	*unknown*
    3e54:	7f 06 02 77 	*unknown*
    3e58:	00 00 07 fc 	l.j 5e48 <_end+0x13dc>
    3e5c:	58 17 00 00 	*unknown*
    3e60:	01 3d 06 02 	l.j 4f45668 <_end+0x4f40bfc>
    3e64:	7b 00 00 02 	*unknown*
    3e68:	8d 01 48 17 	l.lbz r8,18455(r1)
    3e6c:	00 00 02 e7 	l.j 4a08 <_or1k_exception_handler_table+0x14>
    3e70:	06 02 7c 00 	l.jal f80a2e70 <_end+0xf809e404>
    3e74:	00 02 4f 01 	l.j 97a78 <_end+0x9300c>
    3e78:	4c 17 00 00 	l.maci r23,0
    3e7c:	04 c8 06 02 	l.jal 3205684 <_end+0x3200c18>
    3e80:	80 00 00 08 	*unknown*
    3e84:	50 02 dc 17 	*unknown*
    3e88:	00 00 01 e7 	l.j 4624 <_or1k_exception_impure_data+0x1a4>
    3e8c:	06 02 85 00 	l.jal f80a528c <_end+0xf80a0820>
    3e90:	00 06 01 02 	l.j 184298 <_end+0x17f82c>
    3e94:	e0 17 00 00 	l.add r0,r23,r0
    3e98:	01 cc 06 02 	l.j 73056a0 <_end+0x7300c34>
    3e9c:	86 00 00 08 	l.lwz r16,8(r0)
    3ea0:	5c 02 ec 00 	*unknown*
    3ea4:	0f 04 00 00 	l.bnf fc103ea4 <_end+0xfc0ff438>
    3ea8:	05 5b 02 01 	l.jal 56c46ac <_end+0x56bfc40>
    3eac:	06 00 00 00 	l.jal f8003eac <_end+0xf7fff440>
    3eb0:	82 0f 04 00 	*unknown*
    3eb4:	00 03 ff 13 	l.j 103b00 <_end+0xff094>
    3eb8:	00 00 00 33 	l.j 3f84 <_global_impure_ptr+0x550>
    3ebc:	00 00 05 86 	l.j 54d4 <_end+0xa68>
    3ec0:	14 00 00 04 	*unknown*
    3ec4:	1d 14 00 00 	*unknown*
    3ec8:	01 10 14 00 	l.j 4408ec8 <_end+0x440445c>
    3ecc:	00 05 86 14 	l.j 16571c <_end+0x160cb0>
    3ed0:	00 00 00 33 	l.j 3f9c <_global_impure_ptr+0x568>
    3ed4:	00 0f 04 00 	l.j 3c4ed4 <_end+0x3c0468>
    3ed8:	00 05 8c 18 	l.j 166f38 <_end+0x1624cc>
    3edc:	00 00 05 5b 	l.j 5448 <_end+0x9dc>
    3ee0:	0f 04 00 00 	l.bnf fc103ee0 <_end+0xfc0ff474>
    3ee4:	05 68 13 00 	l.jal 5a08ae4 <_end+0x5a04078>
    3ee8:	00 00 85 00 	l.j 252e8 <_end+0x2087c>
    3eec:	00 05 b5 14 	l.j 17133c <_end+0x16c8d0>
    3ef0:	00 00 04 1d 	l.j 4f64 <_end+0x4f8>
    3ef4:	14 00 00 01 	*unknown*
    3ef8:	10 14 00 00 	l.bf 503ef8 <_end+0x4ff48c>
    3efc:	00 85 14 00 	l.j 2148efc <_end+0x2144490>
    3f00:	00 00 33 00 	l.j 10b00 <_end+0xc094>
    3f04:	0f 04 00 00 	l.bnf fc103f04 <_end+0xfc0ff498>
    3f08:	05 97 13 00 	l.jal 65c8b08 <_end+0x65c409c>
    3f0c:	00 00 33 00 	l.j 10b0c <_end+0xc0a0>
    3f10:	00 05 cf 14 	l.j 177b60 <_end+0x1730f4>
    3f14:	00 00 04 1d 	l.j 4f88 <_end+0x51c>
    3f18:	14 00 00 01 	*unknown*
    3f1c:	10 00 0f 04 	l.bf 7b2c <_end+0x30c0>
    3f20:	00 00 05 bb 	l.j 560c <_end+0xba0>
    3f24:	08 00 00 00 	*unknown*
    3f28:	41 00 00 05 	*unknown*
    3f2c:	e5 09 00 00 	*unknown*
    3f30:	00 d2 02 00 	l.j 3484730 <_end+0x347fcc4>
    3f34:	08 00 00 00 	*unknown*
    3f38:	41 00 00 05 	*unknown*
    3f3c:	f5 09 00 00 	*unknown*
    3f40:	00 d2 00 00 	l.j 3483f40 <_end+0x347f4d4>
    3f44:	05 00 00 03 	l.jal 4003f50 <_end+0x3fff4e4>
    3f48:	a4 06 01 1d 	l.andi r0,r6,0x11d
    3f4c:	00 00 02 d5 	l.j 4aa0 <_end+0x34>
    3f50:	19 00 00 04 	l.movhi r8,0x4
    3f54:	af 0c 06 01 	l.xori r24,r12,1537
    3f58:	21 00 00 06 	l.trap 0x6
    3f5c:	36 16 00 00 	*unknown*
    3f60:	03 cc 06 01 	l.j ff305764 <_end+0xff300cf8>
    3f64:	23 00 00 06 	*unknown*
    3f68:	36 00 16 00 	*unknown*
    3f6c:	00 02 a1 06 	l.j ac384 <_end+0xa7918>
    3f70:	01 24 00 00 	l.j 4903f70 <_end+0x48ff504>
    3f74:	00 33 04 16 	l.j cc4fcc <_end+0xcc0560>
    3f78:	00 00 03 9e 	l.j 4df0 <_end+0x384>
    3f7c:	06 01 25 00 	l.jal f804d37c <_end+0xf8048910>
    3f80:	00 06 3c 08 	l.j 192fa0 <_end+0x18e534>
    3f84:	00 0f 04 00 	l.j 3c4f84 <_end+0x3c0518>
    3f88:	00 06 01 0f 	l.j 1843c4 <_end+0x17f958>
    3f8c:	04 00 00 05 	l.jal 3fa0 <_global_impure_ptr+0x56c>
    3f90:	f5 19 00 00 	*unknown*
    3f94:	01 5a 0e 06 	l.j 56877ac <_end+0x5682d40>
    3f98:	01 3d 00 00 	l.j 4f43f98 <_end+0x4f3f52c>
    3f9c:	06 77 16 00 	l.jal f9dc979c <_end+0xf9dc4d30>
    3fa0:	00 04 0b 06 	l.j 106bb8 <_end+0x10214c>
    3fa4:	01 3e 00 00 	l.j 4f83fa4 <_end+0x4f7f538>
    3fa8:	06 77 00 16 	l.jal f9dc4000 <_end+0xf9dbf594>
    3fac:	00 00 04 38 	l.j 508c <_end+0x620>
    3fb0:	06 01 3f 00 	l.jal f8053bb0 <_end+0xf804f144>
    3fb4:	00 06 77 06 	l.j 1a1bcc <_end+0x19d160>
    3fb8:	16 00 00 0e 	*unknown*
    3fbc:	6d 06 01 40 	l.lwa r8,320(r6)
    3fc0:	00 00 00 4f 	l.j 40fc <impure_data+0xa8>
    3fc4:	0c 00 08 00 	l.bnf 5fc4 <_end+0x1558>
    3fc8:	00 00 4f 00 	l.j 17bc8 <_end+0x1315c>
    3fcc:	00 06 87 09 	l.j 1a5bf0 <_end+0x1a1184>
    3fd0:	00 00 00 d2 	l.j 4318 <impure_data+0x2c4>
    3fd4:	02 00 1a cc 	l.j f800ab04 <_end+0xf8006098>
    3fd8:	06 02 58 00 	l.jal f8099fd8 <_end+0xf809556c>
    3fdc:	00 07 88 16 	l.j 1e6034 <_end+0x1e15c8>
    3fe0:	00 00 04 93 	l.j 522c <_end+0x7c0>
    3fe4:	06 02 5a 00 	l.jal f809a7e4 <_end+0xf8095d78>
    3fe8:	00 00 9c 00 	l.j 2afe8 <_end+0x2657c>
    3fec:	16 00 00 04 	*unknown*
    3ff0:	3e 06 02 5b 	*unknown*
    3ff4:	00 00 05 55 	l.j 5548 <_end+0xadc>
    3ff8:	04 16 00 00 	l.jal 583ff8 <_end+0x57f58c>
    3ffc:	02 fc 06 02 	l.j fbf05804 <_end+0xfbf00d98>
    4000:	5c 00 00 07 	*unknown*
    4004:	88 08 16 00 	l.lws r0,5632(r8)
    4008:	00 04 eb 06 	l.j 13ec20 <_end+0x13a1b4>
    400c:	02 5d 00 00 	l.j f974400c <_end+0xf973f5a0>
    4010:	01 86 24 16 	l.j 618d068 <_end+0x61885fc>
    4014:	00 00 02 60 	l.j 4994 <_or1k_interrupt_handler_data_ptr_table+0x48>
    4018:	06 02 5e 00 	l.jal f809b818 <_end+0xf8096dac>
    401c:	00 00 33 48 	l.j 10d3c <_end+0xc2d0>
    4020:	16 00 00 03 	*unknown*
    4024:	c7 06 02 5f 	*unknown*
    4028:	00 00 00 68 	l.j 41c8 <impure_data+0x174>
    402c:	4c 16 00 00 	l.maci r22,0
    4030:	05 01 06 02 	l.jal 4045838 <_end+0x4040dcc>
    4034:	60 00 00 06 	*unknown*
    4038:	42 54 16 00 	*unknown*
    403c:	00 03 d2 06 	l.j f8854 <_end+0xf3de8>
    4040:	02 61 00 00 	l.j f9844040 <_end+0xf983f5d4>
    4044:	00 fa 64 16 	l.j 3e9d09c <_end+0x3e98630>
    4048:	00 00 05 06 	l.j 5460 <_end+0x9f4>
    404c:	06 02 62 00 	l.jal f809c84c <_end+0xf8097de0>
    4050:	00 00 fa 6c 	l.j 42a00 <_end+0x3df94>
    4054:	16 00 00 01 	*unknown*
    4058:	a5 06 02 63 	l.andi r8,r6,0x263
    405c:	00 00 00 fa 	l.j 4444 <impure_data+0x3f0>
    4060:	74 16 00 00 	*unknown*
    4064:	04 be 06 02 	l.jal 2f8586c <_end+0x2f80e00>
    4068:	64 00 00 07 	*unknown*
    406c:	98 7c 16 00 	l.lhs r3,5632(r28)
    4070:	00 02 f0 06 	l.j c0088 <_end+0xbb61c>
    4074:	02 65 00 00 	l.j f9944074 <_end+0xf993f608>
    4078:	07 a8 84 16 	l.jal fea250d0 <_end+0xfea20664>
    407c:	00 00 04 5c 	l.j 51ec <_end+0x780>
    4080:	06 02 66 00 	l.jal f809d880 <_end+0xf8098e14>
    4084:	00 00 33 9c 	l.j 10ef4 <_end+0xc488>
    4088:	16 00 00 02 	*unknown*
    408c:	26 06 02 67 	*unknown*
    4090:	00 00 00 fa 	l.j 4478 <environ>
    4094:	a0 16 00 00 	l.addic r0,r22,0
    4098:	01 96 06 02 	l.j 65858a0 <_end+0x6580e34>
    409c:	68 00 00 00 	*unknown*
    40a0:	fa a8 16 00 	*unknown*
    40a4:	00 02 15 06 	l.j 894bc <_end+0x84a50>
    40a8:	02 69 00 00 	l.j f9a440a8 <_end+0xf9a3f63c>
    40ac:	00 fa b0 16 	l.j 3eb0104 <_end+0x3eab698>
    40b0:	00 00 01 6d 	l.j 4664 <_or1k_exception_impure_data+0x1e4>
    40b4:	06 02 6a 00 	l.jal f809e8b4 <_end+0xf8099e48>
    40b8:	00 00 fa b8 	l.j 42b98 <_end+0x3e12c>
    40bc:	16 00 00 01 	*unknown*
    40c0:	7c 06 02 6b 	*unknown*
    40c4:	00 00 00 fa 	l.j 44ac <_or1k_exception_impure_data+0x2c>
    40c8:	c0 16 00 00 	l.mtspr r22,r0,0x0
    40cc:	03 84 06 02 	l.j fe1058d4 <_end+0xfe100e68>
    40d0:	6c 00 00 00 	l.lwa r0,0(r0)
    40d4:	33 c8 00 08 	*unknown*
    40d8:	00 00 05 5b 	l.j 5644 <_end+0xbd8>
    40dc:	00 00 07 98 	l.j 5f3c <_end+0x14d0>
    40e0:	09 00 00 00 	*unknown*
    40e4:	d2 19 00 08 	*unknown*
    40e8:	00 00 05 5b 	l.j 5654 <_end+0xbe8>
    40ec:	00 00 07 a8 	l.j 5f8c <_end+0x1520>
    40f0:	09 00 00 00 	*unknown*
    40f4:	d2 07 00 08 	*unknown*
    40f8:	00 00 05 5b 	l.j 5664 <_end+0xbf8>
    40fc:	00 00 07 b8 	l.j 5fdc <_end+0x1570>
    4100:	09 00 00 00 	*unknown*
    4104:	d2 17 00 1a 	*unknown*
    4108:	f0 06 02 71 	*unknown*
    410c:	00 00 07 dc 	l.j 607c <_end+0x1610>
    4110:	16 00 00 03 	*unknown*
    4114:	30 06 02 74 	*unknown*
    4118:	00 00 07 dc 	l.j 6088 <_end+0x161c>
    411c:	00 16 00 00 	l.j 58411c <_end+0x57f6b0>
    4120:	04 b5 06 02 	l.jal 2d45928 <_end+0x2d40ebc>
    4124:	75 00 00 07 	*unknown*
    4128:	ec 78 00 08 	*unknown*
    412c:	00 00 02 cf 	l.j 4c68 <_end+0x1fc>
    4130:	00 00 07 ec 	l.j 60e0 <_end+0x1674>
    4134:	09 00 00 00 	*unknown*
    4138:	d2 1d 00 08 	*unknown*
    413c:	00 00 00 9c 	l.j 43ac <impure_data+0x358>
    4140:	00 00 07 fc 	l.j 6130 <_end+0x16c4>
    4144:	09 00 00 00 	*unknown*
    4148:	d2 1d 00 1b 	*unknown*
    414c:	f0 06 02 56 	*unknown*
    4150:	00 00 08 1e 	l.j 61c8 <_end+0x175c>
    4154:	1c 00 00 0c 	*unknown*
    4158:	0b 06 02 6d 	*unknown*
    415c:	00 00 06 87 	l.j 5b78 <_end+0x110c>
    4160:	1c 00 00 04 	*unknown*
    4164:	d8 06 02 76 	l.sb 630(r6),r0
    4168:	00 00 07 b8 	l.j 6048 <_end+0x15dc>
    416c:	00 08 00 00 	l.j 20416c <_end+0x1ff700>
    4170:	05 5b 00 00 	l.jal 56c4170 <_end+0x56bf704>
    4174:	08 2e 09 00 	*unknown*
    4178:	00 00 d2 18 	l.j 389d8 <_end+0x33f6c>
    417c:	00 1d 00 00 	l.j 74417c <_end+0x73f710>
    4180:	08 39 14 00 	*unknown*
    4184:	00 04 1d 00 	l.j 10b584 <_end+0x106b18>
    4188:	0f 04 00 00 	l.bnf fc104188 <_end+0xfc0ff71c>
    418c:	08 2e 0f 04 	*unknown*
    4190:	00 00 01 70 	l.j 4750 <_or1k_exception_impure_data+0x2d0>
    4194:	1d 00 00 08 	*unknown*
    4198:	50 14 00 00 	*unknown*
    419c:	00 33 00 0f 	l.j cc41d8 <_end+0xcbf76c>
    41a0:	04 00 00 08 	l.jal 41c0 <impure_data+0x16c>
    41a4:	56 0f 04 00 	*unknown*
    41a8:	00 08 45 08 	l.j 2155c8 <_end+0x210b5c>
    41ac:	00 00 05 f5 	l.j 5980 <_end+0xf14>
    41b0:	00 00 08 6c 	l.j 6360 <_end+0x18f4>
    41b4:	09 00 00 00 	*unknown*
    41b8:	d2 02 00 04 	*unknown*
    41bc:	00 00 09 93 	l.j 6808 <_end+0x1d9c>
    41c0:	07 20 00 00 	l.jal fc8041c0 <_end+0xfc7ff754>
    41c4:	00 56 0d 00 	l.j 15875c4 <_end+0x1582b58>
    41c8:	00 0c 06 0c 	l.j 3059f8 <_end+0x300f8c>
    41cc:	08 2f 00 00 	*unknown*
    41d0:	08 a8 0b 00 	*unknown*
    41d4:	00 0c 99 08 	l.j 32a5f4 <_end+0x325b88>
    41d8:	31 00 00 08 	*unknown*
    41dc:	a8 00 0b 00 	l.ori r0,r0,0xb00
    41e0:	00 0b f4 08 	l.j 301200 <_end+0x2fc794>
    41e4:	34 00 00 08 	*unknown*
    41e8:	6c 04 0b 00 	l.lwa r0,2816(r4)
    41ec:	00 0c 44 08 	l.j 31520c <_end+0x3107a0>
    41f0:	35 00 00 08 	*unknown*
    41f4:	6c 08 00 1e 	l.lwa r0,30(r8)
    41f8:	00 00 08 6c 	l.j 63a8 <_end+0x193c>
    41fc:	1f 00 00 0c 	*unknown*
    4200:	2d 01 44 00 	*unknown*
    4204:	00 30 e4 00 	l.j c3d204 <_end+0xc38798>
    4208:	00 01 78 01 	l.j 6220c <_end+0x5d7a0>
    420c:	9c 00 00 08 	l.addi r0,r0,8
    4210:	f1 20 00 00 	*unknown*
    4214:	31 34 00 00 	*unknown*
    4218:	09 78 00 00 	*unknown*
    421c:	08 db 21 01 	*unknown*
    4220:	55 02 8e 00 	*unknown*
    4224:	21 01 54 01 	*unknown*
    4228:	30 00 22 00 	*unknown*
    422c:	00 31 b8 00 	l.j c7222c <_end+0xc6d7c0>
    4230:	00 09 78 21 	l.j 2622b4 <_end+0x25d848>
    4234:	01 55 02 8e 	l.j 5544c6c <_end+0x5540200>
    4238:	00 21 01 54 	l.j 844788 <_end+0x83fd1c>
    423c:	01 30 00 00 	l.j 4c0423c <_end+0x4bff7d0>
    4240:	23 00 00 0b 	*unknown*
    4244:	e0 01 6a 00 	*unknown*
    4248:	00 04 1d 00 	l.j 10b648 <_end+0x106bdc>
    424c:	00 32 5c 00 	l.j c9b24c <_end+0xc967e0>
    4250:	00 00 20 01 	l.j c254 <_end+0x77e8>
    4254:	9c 24 00 00 	l.addi r1,r4,0
    4258:	0c 70 01 75 	l.bnf 1c0482c <_end+0x1bffdc0>
    425c:	00 00 32 7c 	l.j 10c4c <_end+0xc1e0>
    4260:	00 00 00 14 	l.j 42b0 <impure_data+0x25c>
    4264:	01 9c 25 00 	l.j 670d664 <_end+0x6708bf8>
    4268:	00 0c 54 01 	l.j 31926c <_end+0x314800>
    426c:	34 00 00 04 	*unknown*
    4270:	23 05 03 00 	*unknown*
    4274:	00 44 80 26 	l.j 112430c <_end+0x111f8a0>
    4278:	00 00 0c 86 	l.j 7490 <_end+0x2a24>
    427c:	06 02 fa 00 	l.jal f80c2a7c <_end+0xf80be010>
    4280:	00 04 1d 27 	l.j 10b71c <_end+0x106cb0>
    4284:	00 00 0c 06 	l.j 729c <_end+0x2830>
    4288:	01 71 00 00 	l.j 5c44288 <_end+0x5c3f81c>
    428c:	08 77 05 03 	*unknown*
    4290:	00 00 49 cc 	l.j 169c0 <_end+0x11f54>
    4294:	27 00 00 0c 	*unknown*
    4298:	81 01 31 00 	*unknown*
    429c:	00 04 1d 05 	l.j 10b6b0 <_end+0x106c44>
    42a0:	03 00 00 49 	l.j fc0043c4 <_end+0xfbfff958>
    42a4:	dc 27 00 00 	l.sh 2048(r7),r0
    42a8:	0c 12 01 37 	l.bnf 484784 <_end+0x47fd18>
    42ac:	00 00 04 1d 	l.j 5320 <_end+0x8b4>
    42b0:	05 03 00 00 	l.jal 40c42b0 <_end+0x40bf844>
    42b4:	44 7c 27 00 	*unknown*
    42b8:	00 0c aa 01 	l.j 32eabc <_end+0x32a050>
    42bc:	3a 00 00 04 	*unknown*
    42c0:	1d 05 03 00 	*unknown*
    42c4:	00 49 d8 28 	l.j 127a364 <_end+0x12758f8>
    42c8:	00 00 0c 92 	l.j 7510 <_end+0x2aa4>
    42cc:	00 00 01 10 	l.j 470c <_or1k_exception_impure_data+0x28c>
    42d0:	14 00 00 01 	*unknown*
    42d4:	10 14 00 00 	l.bf 5042d4 <_end+0x4ff868>
    42d8:	00 33 14 00 	l.j cc92d8 <_end+0xcc486c>
    42dc:	00 00 d2 00 	l.j 38adc <_end+0x34070>
    42e0:	00 00 00 02 	l.j 42e8 <impure_data+0x294>
    42e4:	1a 00 04 00 	l.movhi r16,0x400
    42e8:	00 0f 8f 04 	l.j 3e7ef8 <_end+0x3e348c>
    42ec:	01 00 00 08 	l.j 400430c <_end+0x3fff8a0>
    42f0:	99 01 00 00 	l.lhs r8,0(r1)
    42f4:	0d 6f 00 00 	l.bnf 5bc42f4 <_end+0x5bbf888>
    42f8:	07 aa 00 00 	l.jal fea842f8 <_end+0xfea7f88c>
    42fc:	32 90 00 00 	*unknown*
    4300:	00 c0 00 00 	l.j 3004300 <_end+0x2fff894>
    4304:	0e ee 02 01 	l.bnf fbb84b08 <_end+0xfbb8009c>
    4308:	06 00 00 00 	l.jal f8004308 <_end+0xf7fff89c>
    430c:	7b 02 01 08 	*unknown*
    4310:	00 00 00 79 	l.j 44f4 <_or1k_exception_impure_data+0x74>
    4314:	02 02 05 00 	l.j f8085714 <_end+0xf8080ca8>
    4318:	00 01 2f 02 	l.j 4ff20 <_end+0x4b4b4>
    431c:	02 07 00 00 	l.j f81c431c <_end+0xf81bf8b0>
    4320:	00 0e 02 04 	l.j 384b30 <_end+0x3800c4>
    4324:	05 00 00 00 	l.jal 4004324 <_end+0x3fff8b8>
    4328:	05 03 00 00 	l.jal 40c4328 <_end+0x40bf8bc>
    432c:	09 91 02 41 	*unknown*
    4330:	00 00 00 53 	l.j 447c <_or1k_exception_impure_ptr>
    4334:	02 04 07 00 	l.j f8105f34 <_end+0xf81014c8>
    4338:	00 00 5e 02 	l.j 1bb40 <_end+0x170d4>
    433c:	08 05 00 00 	*unknown*
    4340:	00 00 02 08 	l.j 4b60 <_end+0xf4>
    4344:	07 00 00 00 	l.jal fc004344 <_end+0xfbfff8d8>
    4348:	59 03 00 00 	*unknown*
    434c:	09 93 03 20 	*unknown*
    4350:	00 00 00 48 	l.j 4470 <impure_data+0x41c>
    4354:	04 04 05 69 	l.jal 1058f8 <_end+0x100e8c>
    4358:	6e 74 00 02 	l.lwa r19,2(r20)
    435c:	04 07 00 00 	l.jal 1c435c <_end+0x1bf8f0>
    4360:	00 63 02 04 	l.j 18c4b70 <_end+0x18c0104>
    4364:	07 00 00 01 	l.jal fc004368 <_end+0xfbfff8fc>
    4368:	45 05 04 00 	*unknown*
    436c:	00 00 8e 06 	l.j 27b84 <_end+0x23118>
    4370:	02 01 06 00 	l.j f8045b70 <_end+0xf8041104>
    4374:	00 00 82 03 	l.j 24b80 <_end+0x20114>
    4378:	00 00 0d 35 	l.j 784c <_end+0x2de0>
    437c:	04 d7 00 00 	l.jal 35c437c <_end+0x35bf910>
    4380:	00 88 03 00 	l.j 2204f80 <_end+0x2200514>
    4384:	00 0d a6 05 	l.j 36db98 <_end+0x36912c>
    4388:	1a 00 00 00 	l.movhi r16,0x0
    438c:	ac 07 00 00 	l.xori r0,r7,0
    4390:	00 96 00 00 	l.j 2584390 <_end+0x257f924>
    4394:	00 bc 08 00 	l.j 2f06394 <_end+0x2f01928>
    4398:	00 00 81 1d 	l.j 2480c <_end+0x1fda0>
    439c:	00 09 00 00 	l.j 24439c <_end+0x23f930>
    43a0:	0d 51 01 29 	l.bnf 5444844 <_end+0x543fdd8>
    43a4:	00 00 32 90 	l.j 10de4 <_end+0xc378>
    43a8:	00 00 00 4c 	l.j 44d8 <_or1k_exception_impure_data+0x58>
    43ac:	01 9c 00 00 	l.j 67043ac <_end+0x66ff940>
    43b0:	00 db 0a 00 	l.j 36c6bb0 <_end+0x36c2144>
    43b4:	00 32 a8 00 	l.j cae3b4 <_end+0xca9948>
    43b8:	00 01 da 00 	l.j 7abb8 <_end+0x7614c>
    43bc:	0b 00 00 0d 	*unknown*
    43c0:	e8 01 57 00 	*unknown*
    43c4:	00 00 68 00 	l.j 1e3c4 <_end+0x19958>
    43c8:	00 32 dc 00 	l.j cbb3c8 <_end+0xcb695c>
    43cc:	00 00 38 01 	l.j 123d0 <_end+0xd964>
    43d0:	9c 00 00 01 	l.addi r0,r0,1
    43d4:	25 0c 69 65 	*unknown*
    43d8:	65 00 01 58 	*unknown*
    43dc:	00 00 00 68 	l.j 457c <_or1k_exception_impure_data+0xfc>
    43e0:	00 00 07 e8 	l.j 6380 <_end+0x1914>
    43e4:	0c 74 65 65 	l.bnf 1d1d978 <_end+0x1d18f0c>
    43e8:	00 01 59 00 	l.j 5a7e8 <_end+0x55d7c>
    43ec:	00 00 68 00 	l.j 1e3ec <_end+0x19980>
    43f0:	00 07 fb 0a 	l.j 203018 <_end+0x1fe5ac>
    43f4:	00 00 32 f0 	l.j 10fb4 <_end+0xc548>
    43f8:	00 00 01 e7 	l.j 4b94 <_end+0x128>
    43fc:	0a 00 00 32 	*unknown*
    4400:	f8 00 00 01 	*unknown*
    4404:	f2 00 0d 00 	*unknown*
    4408:	00 0d 06 01 	l.j 345c0c <_end+0x3411a0>
    440c:	5d 00 00 33 	*unknown*
    4410:	14 00 00 00 	*unknown*
    4414:	3c 01 9c 00 	*unknown*
    4418:	00 01 74 0e 	l.j 61450 <_end+0x5c9e4>
    441c:	00 00 0d e0 	l.j 7b9c <_end+0x3130>
    4420:	01 5d 00 00 	l.j 5744420 <_end+0x573f9b4>
    4424:	00 68 00 00 	l.j 1a04424 <_end+0x19ff9b8>
    4428:	08 0e 0f 00 	*unknown*
    442c:	00 33 30 00 	l.j cd042c <_end+0xccb9c0>
    4430:	00 01 fe 00 	l.j 83c30 <_end+0x7f1c4>
    4434:	00 01 5f 10 	l.j 5c074 <_end+0x57608>
    4438:	01 53 04 72 	l.j 54c5600 <_end+0x54c0b94>
    443c:	00 31 1a 00 	l.j c4ac3c <_end+0xc461d0>
    4440:	11 00 00 33 	l.bf 400450c <_end+0x3fffaa0>
    4444:	3c 00 00 02 	*unknown*
    4448:	10 10 01 53 	l.bf 404994 <_end+0x3fff28>
    444c:	06 72 00 31 	l.jal f9c84510 <_end+0xf9c7faa4>
    4450:	25 31 1a 00 	*unknown*
    4454:	00 12 00 00 	l.j 484454 <_end+0x47f9e8>
    4458:	0d c5 01 23 	l.bnf 71448e4 <_end+0x713fe78>
    445c:	00 00 01 85 	l.j 4a70 <_end+0x4>
    4460:	05 03 00 00 	l.jal 40c4460 <_end+0x40bf9f4>
    4464:	49 e8 05 04 	*unknown*
    4468:	00 00 00 68 	l.j 4608 <_or1k_exception_impure_data+0x188>
    446c:	12 00 00 0d 	l.bf f80044a0 <_end+0xf7fffa34>
    4470:	5c 01 24 00 	*unknown*
    4474:	00 01 85 05 	l.j 65888 <_end+0x60e1c>
    4478:	03 00 00 49 	l.j fc00459c <_end+0xfbfffb30>
    447c:	f0 12 00 00 	*unknown*
    4480:	0c d6 01 26 	l.bnf 3584918 <_end+0x357feac>
    4484:	00 00 01 85 	l.j 4a98 <_end+0x2c>
    4488:	05 03 00 00 	l.jal 40c4488 <_end+0x40bfa1c>
    448c:	49 e4 12 00 	*unknown*
    4490:	00 0d 18 01 	l.j 34a494 <_end+0x345a28>
    4494:	27 00 00 01 	*unknown*
    4498:	85 05 03 00 	l.lwz r8,768(r5)
    449c:	00 49 ec 13 	l.j 127f4e8 <_end+0x127aa7c>
    44a0:	00 00 0d fc 	l.j 7c90 <_end+0x3224>
    44a4:	05 1f 00 00 	l.jal 47c44a4 <_end+0x47bfa38>
    44a8:	00 a1 12 00 	l.j 2848ca8 <_end+0x284423c>
    44ac:	00 0c f0 01 	l.j 3404b0 <_end+0x33ba44>
    44b0:	20 00 00 00 	l.sys 0x0
    44b4:	68 05 03 00 	*unknown*
    44b8:	00 49 e0 14 	l.j 127c508 <_end+0x1277a9c>
    44bc:	00 00 0c 70 	l.j 767c <_end+0x2c10>
    44c0:	05 41 00 00 	l.jal 50444c0 <_end+0x503fa54>
    44c4:	01 e7 15 00 	l.j 79c98c4 <_end+0x79c4e58>
    44c8:	16 00 00 0a 	*unknown*
    44cc:	d8 04 92 00 	l.sb 512(r4),r18
    44d0:	00 00 68 17 	l.j 1e52c <_end+0x19ac0>
    44d4:	00 00 0c c3 	l.j 77e0 <_end+0x2d74>
    44d8:	04 02 16 00 	l.jal 89cd8 <_end+0x8526c>
    44dc:	00 00 68 18 	l.j 1e53c <_end+0x19ad0>
    44e0:	00 00 0d d5 	l.j 7c34 <_end+0x31c8>
    44e4:	04 02 21 00 	l.jal 8c8e4 <_end+0x87e78>
    44e8:	00 02 10 19 	l.j 8854c <_end+0x83ae0>
    44ec:	00 00 00 68 	l.j 468c <_or1k_exception_impure_data+0x20c>
    44f0:	00 1a 00 00 	l.j 6844f0 <_end+0x67fa84>
    44f4:	0b 82 04 af 	*unknown*
    44f8:	19 00 00 00 	l.movhi r8,0x0
    44fc:	68 00 00 00 	*unknown*
    4500:	00 00 e5 00 	l.j 3d900 <_end+0x38e94>
    4504:	04 00 00 11 	l.jal 4548 <_or1k_exception_impure_data+0xc8>
    4508:	0a 04 01 00 	*unknown*
    450c:	00 08 99 01 	l.j 22a910 <_end+0x225ea4>
    4510:	00 00 0e 1a 	l.j 7d78 <_end+0x330c>
    4514:	00 00 07 aa 	l.j 63bc <_end+0x1950>
    4518:	00 00 00 68 	l.j 46b8 <_or1k_exception_impure_data+0x238>
    451c:	00 00 00 00 	l.j 451c <_or1k_exception_impure_data+0x9c>
    4520:	00 00 10 81 	l.j 8724 <_end+0x3cb8>
    4524:	02 01 06 00 	l.j f8045d24 <_end+0xf80412b8>
    4528:	00 00 7b 02 	l.j 23130 <_end+0x1e6c4>
    452c:	01 08 00 00 	l.j 420452c <_end+0x41ffac0>
    4530:	00 79 02 02 	l.j 1e44d38 <_end+0x1e402cc>
    4534:	05 00 00 01 	l.jal 4004538 <_end+0x3fffacc>
    4538:	2f 02 02 07 	*unknown*
    453c:	00 00 00 0e 	l.j 4574 <_or1k_exception_impure_data+0xf4>
    4540:	02 04 05 00 	l.j f8105940 <_end+0xf8100ed4>
    4544:	00 00 05 02 	l.j 594c <_end+0xee0>
    4548:	04 07 00 00 	l.jal 1c4548 <_end+0x1bfadc>
    454c:	00 5e 02 08 	l.j 1784d6c <_end+0x1780300>
    4550:	05 00 00 00 	l.jal 4004550 <_end+0x3fffae4>
    4554:	00 02 08 07 	l.j 86570 <_end+0x81b04>
    4558:	00 00 00 59 	l.j 46bc <_or1k_exception_impure_data+0x23c>
    455c:	03 04 05 69 	l.j fc105b00 <_end+0xfc101094>
    4560:	6e 74 00 02 	l.lwa r19,2(r20)
    4564:	04 07 00 00 	l.jal 1c4564 <_end+0x1bfaf8>
    4568:	00 63 04 00 	l.j 18c5568 <_end+0x18c0afc>
    456c:	00 0d 35 02 	l.j 351974 <_end+0x34cf08>
    4570:	d7 00 00 00 	l.sw -16384(r0),r0
    4574:	76 05 04 00 	*unknown*
    4578:	00 00 7c 06 	l.j 23590 <_end+0x1eb24>
    457c:	02 04 07 00 	l.j f810617c <_end+0xf8101710>
    4580:	00 01 45 02 	l.j 55988 <_end+0x50f1c>
    4584:	01 06 00 00 	l.j 4184584 <_end+0x417fb18>
    4588:	00 82 04 00 	l.j 2085588 <_end+0x2080b1c>
    458c:	00 0d a6 03 	l.j 36dd98 <_end+0x36932c>
    4590:	1a 00 00 00 	l.movhi r16,0x0
    4594:	96 07 00 00 	l.lhz r16,0(r7)
    4598:	00 6b 00 00 	l.j 1ac4598 <_end+0x1abfb2c>
    459c:	00 a6 08 00 	l.j 298659c <_end+0x2981b30>
    45a0:	00 00 7d 1d 	l.j 23a14 <_end+0x1efa8>
    45a4:	00 09 00 00 	l.j 2445a4 <_end+0x23fb38>
    45a8:	0e 57 01 0b 	l.bnf f95c49d4 <_end+0xf95bff68>
    45ac:	00 00 33 50 	l.j 112ec <_end+0xc880>
    45b0:	00 00 00 34 	l.j 4680 <_or1k_exception_impure_data+0x200>
    45b4:	01 9c 00 00 	l.j 67045b4 <_end+0x66ffb48>
    45b8:	00 d7 0a 69 	l.j 35c6f5c <_end+0x35c24f0>
    45bc:	64 00 01 0b 	*unknown*
    45c0:	00 00 00 5d 	l.j 4734 <_or1k_exception_impure_data+0x2b4>
    45c4:	00 00 08 3a 	l.j 66ac <_end+0x1c40>
    45c8:	0b 00 00 0e 	*unknown*
    45cc:	81 01 0b 00 	*unknown*
    45d0:	00 00 6b 01 	l.j 1f1d4 <_end+0x1a768>
    45d4:	54 00 0c 00 	*unknown*
    45d8:	00 0d fc 01 	l.j 3835dc <_end+0x37eb70>
    45dc:	08 00 00 00 	*unknown*
    45e0:	8b 05 03 00 	l.lws r24,768(r5)
    45e4:	00 49 f4 00 	l.j 12815e4 <_end+0x127cb78>
    45e8:	00 00 06 a8 	l.j 6088 <_end+0x161c>
    45ec:	00 04 00 00 	l.j 1045ec <_end+0xffb80>
    45f0:	11 ac 04 01 	l.bf 6b055f4 <_end+0x6b00b88>
    45f4:	00 00 08 99 	l.j 6858 <_end+0x1dec>
    45f8:	01 00 00 0f 	l.j 4004634 <_end+0x3fffbc8>
    45fc:	0f 00 00 07 	l.bnf fc004618 <_end+0xfbfffbac>
    4600:	aa 00 00 33 	l.ori r16,r0,0x33
    4604:	84 00 00 03 	l.lwz r0,3(r0)
    4608:	64 00 00 11 	*unknown*
    460c:	59 02 01 06 	*unknown*
    4610:	00 00 00 7b 	l.j 47fc <_or1k_exception_impure_data+0x37c>
    4614:	02 01 08 00 	l.j f8046614 <_end+0xf8041ba8>
    4618:	00 00 79 02 	l.j 22a20 <_end+0x1dfb4>
    461c:	02 05 00 00 	l.j f814461c <_end+0xf813fbb0>
    4620:	01 2f 02 02 	l.j 4bc4e28 <_end+0x4bc03bc>
    4624:	07 00 00 00 	l.jal fc004624 <_end+0xfbfffbb8>
    4628:	0e 02 04 05 	l.bnf f808563c <_end+0xf8080bd0>
    462c:	00 00 00 05 	l.j 4640 <_or1k_exception_impure_data+0x1c0>
    4630:	03 00 00 09 	l.j fc004654 <_end+0xfbfffbe8>
    4634:	91 03 41 00 	l.lbs r8,16640(r3)
    4638:	00 00 53 02 	l.j 19240 <_end+0x147d4>
    463c:	04 07 00 00 	l.jal 1c463c <_end+0x1bfbd0>
    4640:	00 5e 02 08 	l.j 1784e60 <_end+0x17803f4>
    4644:	05 00 00 00 	l.jal 4004644 <_end+0x3fffbd8>
    4648:	00 02 08 07 	l.j 86664 <_end+0x81bf8>
    464c:	00 00 00 59 	l.j 47b0 <_or1k_exception_impure_data+0x330>
    4650:	03 00 00 09 	l.j fc004674 <_end+0xfbfffc08>
    4654:	93 04 20 00 	l.lbs r24,8192(r4)
    4658:	00 00 48 04 	l.j 16668 <_end+0x11bfc>
    465c:	04 05 69 6e 	l.jal 15ec14 <_end+0x15a1a8>
    4660:	74 00 02 04 	*unknown*
    4664:	07 00 00 00 	l.jal fc004664 <_end+0xfbfffbf8>
    4668:	63 03 00 00 	*unknown*
    466c:	0d 35 02 d7 	l.bnf 4d451c8 <_end+0x4d4075c>
    4670:	00 00 00 8c 	l.j 48a0 <_or1k_exception_impure_data+0x420>
    4674:	05 04 00 00 	l.jal 4104674 <_end+0x40ffc08>
    4678:	00 92 06 02 	l.j 2485e80 <_end+0x2481414>
    467c:	04 07 00 00 	l.jal 1c467c <_end+0x1bfc10>
    4680:	01 45 02 01 	l.j 5144e84 <_end+0x5140418>
    4684:	06 00 00 00 	l.jal f8004684 <_end+0xf7fffc18>
    4688:	82 07 00 00 	*unknown*
    468c:	0c 06 0c 05 	l.bnf 1876a0 <_end+0x182c34>
    4690:	2f 00 00 00 	*unknown*
    4694:	d2 08 00 00 	*unknown*
    4698:	0c 99 05 31 	l.bnf 2645b5c <_end+0x26410f0>
    469c:	00 00 00 d2 	l.j 49e4 <_or1k_exception_stack_top>
    46a0:	00 08 00 00 	l.j 2046a0 <_end+0x1ffc34>
    46a4:	0b f4 05 34 	*unknown*
    46a8:	00 00 00 68 	l.j 4848 <_or1k_exception_impure_data+0x3c8>
    46ac:	04 08 00 00 	l.jal 2046ac <_end+0x1ffc40>
    46b0:	0c 44 05 35 	l.bnf 1105b84 <_end+0x1101118>
    46b4:	00 00 00 68 	l.j 4854 <_or1k_exception_impure_data+0x3d4>
    46b8:	08 00 09 00 	*unknown*
    46bc:	00 00 68 0a 	l.j 1e6e4 <_end+0x19c78>
    46c0:	00 00 0b 30 	l.j 7380 <_end+0x2914>
    46c4:	02 01 02 00 	l.j f8044ec4 <_end+0xf8040458>
    46c8:	00 00 68 03 	l.j 1e6d4 <_end+0x19c68>
    46cc:	00 00 01 01 	l.j 4ad0 <_end+0x64>
    46d0:	0b 73 70 72 	*unknown*
    46d4:	00 02 01 02 	l.j 84adc <_end+0x80070>
    46d8:	00 00 00 68 	l.j 4878 <_or1k_exception_impure_data+0x3f8>
    46dc:	0c 00 00 04 	l.bnf 46ec <_or1k_exception_impure_data+0x26c>
    46e0:	1b 02 01 03 	*unknown*
    46e4:	00 00 00 68 	l.j 4884 <_or1k_exception_impure_data+0x404>
    46e8:	00 0d 00 00 	l.j 3446e8 <_end+0x33fc7c>
    46ec:	0b 25 02 f5 	*unknown*
    46f0:	03 00 00 01 	l.j fc0046f4 <_end+0xfbfffc88>
    46f4:	24 0e 73 70 	*unknown*
    46f8:	72 00 02 f5 	*unknown*
    46fc:	00 00 00 68 	l.j 489c <_or1k_exception_impure_data+0x41c>
    4700:	0f 00 00 04 	l.bnf fc004710 <_end+0xfbfffca4>
    4704:	1b 02 f5 00 	*unknown*
    4708:	00 00 68 00 	l.j 1e708 <_end+0x19c9c>
    470c:	10 00 00 0e 	l.bf 4744 <_or1k_exception_impure_data+0x2c4>
    4710:	d3 01 20 00 	*unknown*
    4714:	00 33 84 00 	l.j ce5714 <_end+0xce0ca8>
    4718:	00 00 50 01 	l.j 1871c <_end+0x13cb0>
    471c:	9c 00 00 01 	l.addi r0,r0,1
    4720:	93 11 00 00 	l.lbs r24,0(r17)
    4724:	0f 03 01 23 	l.bnf fc0c4bb0 <_end+0xfc0c0144>
    4728:	00 00 00 68 	l.j 48c8 <_or1k_uart_read_cb>
    472c:	00 00 08 75 	l.j 6900 <_end+0x1e94>
    4730:	12 00 00 00 	l.bf f8004730 <_end+0xf7fffcc4>
    4734:	d7 00 00 33 	l.sw -16333(r0),r0
    4738:	a0 00 00 00 	l.addic r0,r0,0
    473c:	78 01 23 00 	*unknown*
    4740:	00 01 72 13 	l.j 60f8c <_end+0x5c520>
    4744:	00 00 00 e8 	l.j 4ae4 <_end+0x78>
    4748:	50 00 14 00 	*unknown*
    474c:	00 00 78 15 	l.j 227a0 <_end+0x1dd34>
    4750:	00 00 00 f4 	l.j 4b20 <_end+0xb4>
    4754:	00 00 08 b0 	l.j 6a14 <_end+0x1fa8>
    4758:	00 00 16 00 	l.j 9f58 <_end+0x54ec>
    475c:	00 01 01 00 	l.j 44b5c <_end+0x400f0>
    4760:	00 33 c0 00 	l.j cf4760 <_end+0xcefcf4>
    4764:	00 00 04 01 	l.j 5768 <_end+0xcfc>
    4768:	27 17 00 00 	*unknown*
    476c:	01 18 00 00 	l.j 460476c <_end+0x45ffd00>
    4770:	08 c3 13 00 	*unknown*
    4774:	00 01 0d 50 	l.j 47cb4 <_end+0x43248>
    4778:	00 00 00 18 	l.j 47d8 <_or1k_exception_impure_data+0x358>
    477c:	00 00 0e b2 	l.j 8244 <_end+0x37d8>
    4780:	01 32 00 00 	l.j 4c84780 <_end+0x4c7fd14>
    4784:	00 73 00 00 	l.j 1cc4784 <_end+0x1cbfd18>
    4788:	33 d4 00 00 	*unknown*
    478c:	00 b4 01 9c 	l.j 2d04dfc <_end+0x2d00390>
    4790:	00 00 02 62 	l.j 5118 <_end+0x6ac>
    4794:	19 68 7a 00 	*unknown*
    4798:	01 32 00 00 	l.j 4c84798 <_end+0x4c7fd2c>
    479c:	00 7a 00 00 	l.j 1e8479c <_end+0x1e7fd30>
    47a0:	08 e7 1a 75 	*unknown*
    47a4:	70 72 00 01 	*unknown*
    47a8:	34 00 00 00 	*unknown*
    47ac:	68 11 00 00 	*unknown*
    47b0:	0b ff 01 3a 	*unknown*
    47b4:	00 00 00 68 	l.j 4954 <_or1k_interrupt_handler_data_ptr_table+0x8>
    47b8:	00 00 09 1e 	l.j 6c30 <_end+0x21c4>
    47bc:	12 00 00 00 	l.bf f80047bc <_end+0xf7fffd50>
    47c0:	d7 00 00 33 	l.sw -16333(r0),r0
    47c4:	e8 00 00 00 	*unknown*
    47c8:	90 01 34 00 	l.lbs r0,13312(r1)
    47cc:	00 01 fd 1b 	l.j 83c38 <_end+0x7f1cc>
    47d0:	00 00 00 e8 	l.j 4b70 <_end+0x104>
    47d4:	01 14 00 00 	l.j 45047d4 <_end+0x44ffd68>
    47d8:	00 90 15 00 	l.j 2409bd8 <_end+0x240516c>
    47dc:	00 00 f4 00 	l.j 417dc <_end+0x3cd70>
    47e0:	00 09 31 00 	l.j 250be0 <_end+0x24c174>
    47e4:	00 12 00 00 	l.j 4847e4 <_end+0x47fd78>
    47e8:	01 01 00 00 	l.j 40447e8 <_end+0x403fd7c>
    47ec:	34 2c 00 00 	*unknown*
    47f0:	00 a8 01 3c 	l.j 2a04ce0 <_end+0x2a00274>
    47f4:	00 00 02 23 	l.j 5080 <_end+0x614>
    47f8:	17 00 00 01 	*unknown*
    47fc:	18 00 00 09 	l.movhi r0,0x9
    4800:	44 17 00 00 	*unknown*
    4804:	01 0d 00 00 	l.j 4344804 <_end+0x433fd98>
    4808:	09 57 00 12 	*unknown*
    480c:	00 00 01 01 	l.j 4c10 <_end+0x1a4>
    4810:	00 00 34 54 	l.j 11960 <_end+0xcef4>
    4814:	00 00 00 c0 	l.j 4b14 <_end+0xa8>
    4818:	01 46 00 00 	l.j 5184818 <_end+0x517fdac>
    481c:	02 49 17 00 	l.j f924a41c <_end+0xf92459b0>
    4820:	00 01 18 00 	l.j 4a820 <_end+0x45db4>
    4824:	00 09 6d 17 	l.j 25fc80 <_end+0x25b214>
    4828:	00 00 01 0d 	l.j 4c5c <_end+0x1f0>
    482c:	00 00 09 81 	l.j 6e30 <_end+0x23c4>
    4830:	00 1c 00 00 	l.j 704830 <_end+0x6ffdc4>
    4834:	34 54 00 00 	*unknown*
    4838:	06 99 1d 01 	l.jal fa64bc3c <_end+0xfa6471d0>
    483c:	54 05 03 00 	*unknown*
    4840:	00 33 84 1d 	l.j ce58b4 <_end+0xce0e48>
    4844:	01 53 01 35 	l.j 54c4d18 <_end+0x54c02ac>
    4848:	00 00 1e 00 	l.j c048 <_end+0x75dc>
    484c:	00 0f 6f 01 	l.j 3e0450 <_end+0x3db9e4>
    4850:	4c 00 00 34 	l.maci r0,52
    4854:	88 00 00 00 	l.lws r0,0(r0)
    4858:	68 01 9c 00 	*unknown*
    485c:	00 02 ec 19 	l.j bf8c0 <_end+0xbae54>
    4860:	68 7a 00 01 	*unknown*
    4864:	4c 00 00 00 	l.maci r0,0
    4868:	68 00 00 09 	*unknown*
    486c:	97 1f 00 00 	l.lhz r24,0(r31)
    4870:	0b ff 01 4e 	*unknown*
    4874:	00 00 00 68 	l.j 4a14 <_or1k_exception_handler_table+0x20>
    4878:	01 5b 11 00 	l.j 56c8c78 <_end+0x56c420c>
    487c:	00 0f 03 01 	l.j 3c5480 <_end+0x3c0a14>
    4880:	4f 00 00 00 	*unknown*
    4884:	68 00 00 09 	*unknown*
    4888:	c3 12 00 00 	l.mtspr r18,r0,0xc000
    488c:	00 d7 00 00 	l.j 35c488c <_end+0x35bfe20>
    4890:	34 b4 00 00 	*unknown*
    4894:	00 d8 01 4f 	l.j 3604dd0 <_end+0x3600364>
    4898:	00 00 02 cb 	l.j 53c4 <_end+0x958>
    489c:	13 00 00 00 	l.bf fc00489c <_end+0xfbfffe30>
    48a0:	e8 50 00 14 	*unknown*
    48a4:	00 00 00 d8 	l.j 4c04 <_end+0x198>
    48a8:	15 00 00 00 	l.nop 0x0
    48ac:	f4 00 00 09 	*unknown*
    48b0:	e6 00 00 16 	*unknown*
    48b4:	00 00 01 01 	l.j 4cb8 <_end+0x24c>
    48b8:	00 00 34 cc 	l.j 11be8 <_end+0xd17c>
    48bc:	00 00 00 04 	l.j 48cc <_or1k_interrupt_handler_table>
    48c0:	01 51 17 00 	l.j 544a4c0 <_end+0x5445a54>
    48c4:	00 01 18 00 	l.j 4a8c4 <_end+0x45e58>
    48c8:	00 09 c3 13 	l.j 275514 <_end+0x270aa8>
    48cc:	00 00 01 0d 	l.j 4d00 <_end+0x294>
    48d0:	50 00 00 00 	*unknown*
    48d4:	10 00 00 0e 	l.bf 490c <_or1k_interrupt_handler_table+0x40>
    48d8:	72 01 56 00 	*unknown*
    48dc:	00 34 f0 00 	l.j d408dc <_end+0xd3be70>
    48e0:	00 00 28 01 	l.j e8e4 <_end+0x9e78>
    48e4:	9c 00 00 03 	l.addi r0,r0,3
    48e8:	27 20 00 00 	*unknown*
    48ec:	0e 81 01 56 	l.bnf fa044e44 <_end+0xfa0403d8>
    48f0:	00 00 00 8c 	l.j 4b20 <_end+0xb4>
    48f4:	00 00 09 f9 	l.j 70d8 <_end+0x266c>
    48f8:	1c 00 00 35 	*unknown*
    48fc:	08 00 00 06 	*unknown*
    4900:	99 1d 01 54 	l.lhs r8,340(r29)
    4904:	03 f3 01 53 	l.j ffcc4e50 <_end+0xffcc03e4>
    4908:	1d 01 53 01 	*unknown*
    490c:	35 00 00 10 	*unknown*
    4910:	00 00 0e 9e 	l.j 8388 <_end+0x391c>
    4914:	01 5c 00 00 	l.j 5704914 <_end+0x56ffea8>
    4918:	35 18 00 00 	*unknown*
    491c:	00 54 01 9c 	l.j 1504f8c <_end+0x1500520>
    4920:	00 00 03 9f 	l.j 579c <_end+0xd30>
    4924:	20 00 00 0c 	l.sys 0xc
    4928:	4f 01 5c 00 	*unknown*
    492c:	00 00 68 00 	l.j 1e92c <_end+0x19ec0>
    4930:	00 0a 25 21 	l.j 28ddb4 <_end+0x289348>
    4934:	00 00 0f 03 	l.j 8540 <_end+0x3ad4>
    4938:	01 61 00 00 	l.j 5844938 <_end+0x583fecc>
    493c:	00 68 12 00 	l.j 1a0913c <_end+0x1a046d0>
    4940:	00 00 d7 00 	l.j 3a540 <_end+0x35ad4>
    4944:	00 35 28 00 	l.j d4e944 <_end+0xd49ed8>
    4948:	00 00 f0 01 	l.j 4094c <_end+0x3bee0>
    494c:	61 00 00 03 	*unknown*
    4950:	80 13 00 00 	*unknown*
    4954:	00 e8 50 00 	l.j 3a18954 <_end+0x3a13ee8>
    4958:	14 00 00 00 	*unknown*
    495c:	f0 15 00 00 	*unknown*
    4960:	00 f4 00 00 	l.j 3d04960 <_end+0x3cffef4>
    4964:	0a 38 00 00 	*unknown*
    4968:	16 00 00 01 	*unknown*
    496c:	01 00 00 35 	l.j 4004a40 <_end+0x3ffffd4>
    4970:	58 00 00 00 	*unknown*
    4974:	04 01 65 22 	l.jal 5ddfc <_end+0x59390>
    4978:	00 00 01 18 	l.j 4dd8 <_end+0x36c>
    497c:	17 00 00 01 	*unknown*
    4980:	0d 00 00 0a 	l.bnf 40049a8 <_end+0x3ffff3c>
    4984:	4b 00 00 10 	*unknown*
    4988:	00 00 0e f1 	l.j 854c <_end+0x3ae0>
    498c:	01 70 00 00 	l.j 5c0498c <_end+0x5bfff20>
    4990:	35 6c 00 00 	*unknown*
    4994:	00 60 01 9c 	l.j 1805004 <_end+0x1800598>
    4998:	00 00 04 68 	l.j 5b38 <_end+0x10cc>
    499c:	11 00 00 0f 	l.bf 40049d8 <_end+0x3ffff6c>
    49a0:	03 01 72 00 	l.j fc0611a0 <_end+0xfc05c734>
    49a4:	00 00 68 00 	l.j 1e9a4 <_end+0x19f38>
    49a8:	00 0a 61 23 	l.j 29ce34 <_end+0x2983c8>
    49ac:	73 72 00 01 	*unknown*
    49b0:	77 00 00 00 	*unknown*
    49b4:	68 00 00 0a 	*unknown*
    49b8:	7a 12 00 00 	*unknown*
    49bc:	00 d7 00 00 	l.j 35c49bc <_end+0x35bff50>
    49c0:	35 74 00 00 	*unknown*
    49c4:	01 08 01 72 	l.j 4204f8c <_end+0x4200520>
    49c8:	00 00 03 fb 	l.j 59b4 <_end+0xf48>
    49cc:	13 00 00 00 	l.bf fc0049cc <_end+0xfbffff60>
    49d0:	e8 50 00 14 	*unknown*
    49d4:	00 00 01 08 	l.j 4df4 <_end+0x388>
    49d8:	15 00 00 00 	l.nop 0x0
    49dc:	f4 00 00 0a 	*unknown*
    49e0:	9c 00 00 24 	l.addi r0,r0,36
    49e4:	00 00 01 01 	l.j 4de8 <_end+0x37c>
    49e8:	00 00 35 a8 	l.j 12088 <_end+0xd61c>
    49ec:	00 00 00 04 	l.j 49fc <_or1k_exception_handler_table+0x8>
    49f0:	01 75 00 00 	l.j 5d449f0 <_end+0x5d3ff84>
    49f4:	04 1b 22 00 	l.jal 6cd1f4 <_end+0x6c8788>
    49f8:	00 01 18 13 	l.j 4aa44 <_end+0x45fd8>
    49fc:	00 00 01 0d 	l.j 4e30 <_end+0x3c4>
    4a00:	50 00 00 24 	*unknown*
    4a04:	00 00 00 d7 	l.j 4d60 <_end+0x2f4>
    4a08:	00 00 35 ac 	l.j 120b8 <_end+0xd64c>
    4a0c:	00 00 00 08 	l.j 4a2c <_or1k_exception_handler_table+0x38>
    4a10:	01 77 00 00 	l.j 5dc4a10 <_end+0x5dbffa4>
    4a14:	04 48 1b 00 	l.jal 120b614 <_end+0x1206ba8>
    4a18:	00 00 e8 11 	l.j 3ea5c <_end+0x39ff0>
    4a1c:	25 00 00 35 	*unknown*
    4a20:	ac 00 00 00 	l.xori r0,r0,0
    4a24:	08 15 00 00 	*unknown*
    4a28:	00 f4 00 00 	l.j 3d04a28 <_end+0x3cfffbc>
    4a2c:	0a af 00 00 	*unknown*
    4a30:	16 00 00 01 	*unknown*
    4a34:	01 00 00 35 	l.j 4004b08 <_end+0x400009c>
    4a38:	b8 00 00 00 	l.slli r0,r0,0x0
    4a3c:	04 01 79 17 	l.jal 62e98 <_end+0x5e42c>
    4a40:	00 00 01 18 	l.j 4ea0 <_end+0x434>
    4a44:	00 00 0a 7a 	l.j 742c <_end+0x29c0>
    4a48:	1b 00 00 01 	l.movhi r24,0x1
    4a4c:	0d 11 00 00 	l.bnf 4444a4c <_end+0x443ffe0>
    4a50:	26 00 00 0c 	*unknown*
    4a54:	c3 01 83 00 	l.mtspr r1,r16,0xc300
    4a58:	00 00 68 00 	l.j 1ea58 <_end+0x19fec>
    4a5c:	00 35 cc 00 	l.j d77a5c <_end+0xd72ff0>
    4a60:	00 00 38 01 	l.j 12a64 <_end+0xdff8>
    4a64:	9c 00 00 04 	l.addi r0,r0,4
    4a68:	e3 21 00 00 	l.add r25,r1,r0
    4a6c:	0a a2 01 85 	*unknown*
    4a70:	00 00 00 68 	l.j 4c10 <_end+0x1a4>
    4a74:	23 73 72 00 	*unknown*
    4a78:	01 86 00 00 	l.j 6184a78 <_end+0x618000c>
    4a7c:	00 68 00 00 	l.j 1a04a7c <_end+0x1a00010>
    4a80:	0a c2 12 00 	*unknown*
    4a84:	00 00 d7 00 	l.j 3a684 <_end+0x35c18>
    4a88:	00 35 d4 00 	l.j d79a88 <_end+0xd7501c>
    4a8c:	00 01 20 01 	l.j 4ca90 <_end+0x48024>
    4a90:	85 00 00 04 	l.lwz r8,4(r0)
    4a94:	c3 1b 00 00 	l.mtspr r27,r0,0xc000
    4a98:	00 e8 11 14 	l.j 3a08ee8 <_end+0x3a0447c>
    4a9c:	00 00 01 20 	l.j 4f1c <_end+0x4b0>
    4aa0:	15 00 00 00 	l.nop 0x0
    4aa4:	f4 00 00 0a 	*unknown*
    4aa8:	e5 00 00 16 	*unknown*
    4aac:	00 00 01 01 	l.j 4eb0 <_end+0x444>
    4ab0:	00 00 35 e8 	l.j 12250 <_end+0xd7e4>
    4ab4:	00 00 00 04 	l.j 4ac4 <_end+0x58>
    4ab8:	01 87 17 00 	l.j 61ca6b8 <_end+0x61c5c4c>
    4abc:	00 01 18 00 	l.j 4aabc <_end+0x46050>
    4ac0:	00 0a c2 1b 	l.j 2b532c <_end+0x2b08c0>
    4ac4:	00 00 01 0d 	l.j 4ef8 <_end+0x48c>
    4ac8:	11 00 00 10 	l.bf 4004b08 <_end+0x400009c>
    4acc:	00 00 0d d5 	l.j 8220 <_end+0x37b4>
    4ad0:	01 8c 00 00 	l.j 6304ad0 <_end+0x6300064>
    4ad4:	36 04 00 00 	*unknown*
    4ad8:	00 24 01 9c 	l.j 905148 <_end+0x9006dc>
    4adc:	00 00 05 5e 	l.j 6054 <_end+0x15e8>
    4ae0:	20 00 00 0f 	l.sys 0xf
    4ae4:	08 01 8c 00 	*unknown*
    4ae8:	00 00 68 00 	l.j 1eae8 <_end+0x1a07c>
    4aec:	00 0a f8 23 	l.j 2c2b78 <_end+0x2be10c>
    4af0:	73 72 00 01 	*unknown*
    4af4:	8e 00 00 00 	l.lbz r16,0(r0)
    4af8:	68 00 00 0b 	*unknown*
    4afc:	19 12 00 00 	*unknown*
    4b00:	00 d7 00 00 	l.j 35c4b00 <_end+0x35c0094>
    4b04:	36 08 00 00 	*unknown*
    4b08:	01 38 01 8e 	l.j 4e05140 <_end+0x4e006d4>
    4b0c:	00 00 05 3e 	l.j 6004 <_end+0x1598>
    4b10:	1b 00 00 00 	l.movhi r24,0x0
    4b14:	e8 11 14 00 	*unknown*
    4b18:	00 01 38 15 	l.j 52b6c <_end+0x4e100>
    4b1c:	00 00 00 f4 	l.j 4eec <_end+0x480>
    4b20:	00 00 0b 3b 	l.j 780c <_end+0x2da0>
    4b24:	00 00 16 00 	l.j a324 <_end+0x58b8>
    4b28:	00 01 01 00 	l.j 44f28 <_end+0x404bc>
    4b2c:	00 36 18 00 	l.j d8ab2c <_end+0xd860c0>
    4b30:	00 00 04 01 	l.j 5b34 <_end+0x10c8>
    4b34:	90 17 00 00 	l.lbs r0,0(r23)
    4b38:	01 18 00 00 	l.j 4604b38 <_end+0x46000cc>
    4b3c:	0b 19 1b 00 	*unknown*
    4b40:	00 01 0d 11 	l.j 47f84 <_end+0x43518>
    4b44:	00 00 10 00 	l.j 8b44 <_end+0x40d8>
    4b48:	00 0e c2 01 	l.j 3b534c <_end+0x3b08e0>
    4b4c:	94 00 00 36 	l.lhz r0,54(r0)
    4b50:	28 00 00 00 	*unknown*
    4b54:	34 01 9c 00 	*unknown*
    4b58:	00 05 cd 11 	l.j 177f9c <_end+0x173530>
    4b5c:	00 00 0f 03 	l.j 8768 <_end+0x3cfc>
    4b60:	01 96 00 00 	l.j 6584b60 <_end+0x65800f4>
    4b64:	00 68 00 00 	l.j 1a04b64 <_end+0x1a000f8>
    4b68:	0b 4e 12 00 	*unknown*
    4b6c:	00 00 d7 00 	l.j 3a76c <_end+0x35d00>
    4b70:	00 36 30 00 	l.j d90b70 <_end+0xd8c104>
    4b74:	00 01 50 01 	l.j 58b78 <_end+0x5410c>
    4b78:	96 00 00 05 	l.lhz r16,5(r0)
    4b7c:	ac 13 00 00 	l.xori r0,r19,0
    4b80:	00 e8 50 00 	l.j 3a18b80 <_end+0x3a14114>
    4b84:	14 00 00 01 	*unknown*
    4b88:	50 15 00 00 	*unknown*
    4b8c:	00 f4 00 00 	l.j 3d04b8c <_end+0x3d00120>
    4b90:	0b 69 00 00 	*unknown*
    4b94:	16 00 00 01 	*unknown*
    4b98:	01 00 00 36 	l.j 4004c70 <_end+0x4000204>
    4b9c:	48 00 00 00 	l.jalr r0
    4ba0:	04 01 98 17 	l.jal 6abfc <_end+0x66190>
    4ba4:	00 00 01 18 	l.j 5004 <_end+0x598>
    4ba8:	00 00 0b 4e 	l.j 78e0 <_end+0x2e74>
    4bac:	13 00 00 01 	l.bf fc004bb0 <_end+0xfc000144>
    4bb0:	0d 50 00 00 	l.bnf 5404bb0 <_end+0x5400144>
    4bb4:	00 10 00 00 	l.j 404bb4 <_end+0x400148>
    4bb8:	0f 47 01 9c 	l.bnf fd1c5228 <_end+0xfd1c07bc>
    4bbc:	00 00 36 5c 	l.j 1252c <_end+0xdac0>
    4bc0:	00 00 00 40 	l.j 4cc0 <_end+0x254>
    4bc4:	01 9c 00 00 	l.j 6704bc4 <_end+0x6700158>
    4bc8:	06 5d 11 00 	l.jal f9748fc8 <_end+0xf974455c>
    4bcc:	00 0f 03 01 	l.j 3c57d0 <_end+0x3c0d64>
    4bd0:	9e 00 00 00 	l.addi r16,r0,0
    4bd4:	68 00 00 0b 	*unknown*
    4bd8:	7c 12 00 00 	*unknown*
    4bdc:	00 d7 00 00 	l.j 35c4bdc <_end+0x35c0170>
    4be0:	36 64 00 00 	*unknown*
    4be4:	01 68 01 9e 	l.j 5a0525c <_end+0x5a007f0>
    4be8:	00 00 06 1b 	l.j 6454 <_end+0x19e8>
    4bec:	13 00 00 00 	l.bf fc004bec <_end+0xfc000180>
    4bf0:	e8 50 00 14 	*unknown*
    4bf4:	00 00 01 68 	l.j 5194 <_end+0x728>
    4bf8:	15 00 00 00 	l.nop 0x0
    4bfc:	f4 00 00 0b 	*unknown*
    4c00:	98 00 00 24 	l.lhs r0,36(r0)
    4c04:	00 00 01 01 	l.j 5008 <_end+0x59c>
    4c08:	00 00 36 7c 	l.j 125f8 <_end+0xdb8c>
    4c0c:	00 00 00 04 	l.j 4c1c <_end+0x1b0>
    4c10:	01 a0 00 00 	l.j 6804c10 <_end+0x68001a4>
    4c14:	06 3f 17 00 	l.jal f8fca814 <_end+0xf8fc5da8>
    4c18:	00 01 18 00 	l.j 4ac18 <_end+0x461ac>
    4c1c:	00 0b 7c 13 	l.j 2e3c68 <_end+0x2df1fc>
    4c20:	00 00 01 0d 	l.j 5054 <_end+0x5e8>
    4c24:	50 00 00 16 	*unknown*
    4c28:	00 00 01 01 	l.j 502c <_end+0x5c0>
    4c2c:	00 00 36 80 	l.j 1262c <_end+0xdbc0>
    4c30:	00 00 00 0c 	l.j 4c60 <_end+0x1f4>
    4c34:	01 a1 1b 00 	l.j 684b834 <_end+0x6846dc8>
    4c38:	00 01 18 00 	l.j 4ac38 <_end+0x461cc>
    4c3c:	13 00 00 01 	l.bf fc004c40 <_end+0xfc0001d4>
    4c40:	0d 50 01 00 	l.bnf 5405040 <_end+0x54005d4>
    4c44:	00 27 00 00 	l.j 9c4c44 <_end+0x9c01d8>
    4c48:	0e 89 01 ab 	l.bnf fa2452f4 <_end+0xfa240888>
    4c4c:	00 00 00 53 	l.j 4d98 <_end+0x32c>
    4c50:	00 00 36 9c 	l.j 126c0 <_end+0xdc54>
    4c54:	00 00 00 20 	l.j 4cd4 <_end+0x268>
    4c58:	01 9c 28 00 	l.j 670ec58 <_end+0x670a1ec>
    4c5c:	00 0f 58 01 	l.j 3dac60 <_end+0x3d61f4>
    4c60:	b7 00 00 36 	l.mfspr r24,r0,0x36
    4c64:	bc 00 00 00 	l.sfeqi r0,0
    4c68:	2c 01 9c 29 	*unknown*
    4c6c:	00 00 0c 06 	l.j 7c84 <_end+0x3218>
    4c70:	05 3d 00 00 	l.jal 4f44c70 <_end+0x4f40204>
    4c74:	00 a1 29 00 	l.j 284f074 <_end+0x284a608>
    4c78:	00 09 d3 06 	l.j 279890 <_end+0x274e24>
    4c7c:	19 00 00 00 	l.movhi r8,0x0
    4c80:	68 2a 00 00 	*unknown*
    4c84:	0e 57 02 e3 	l.bnf f95c5810 <_end+0xf95c0da4>
    4c88:	2b 00 00 00 	*unknown*
    4c8c:	73 2b 00 00 	*unknown*
    4c90:	00 81 00 00 	l.j 2044c90 <_end+0x2040224>
    4c94:	00 00 08 93 	l.j 6ee0 <_end+0x2474>
    4c98:	00 04 00 00 	l.j 104c98 <_end+0x10022c>
    4c9c:	14 1f 04 01 	*unknown*
    4ca0:	00 00 00 30 	l.j 4d60 <_end+0x2f4>
    4ca4:	01 00 00 0f 	l.j 4004ce0 <_end+0x4000274>
    4ca8:	d6 00 00 0f 	l.sw -32753(r0),r0
    4cac:	90 00 00 37 	l.lbs r0,55(r0)
    4cb0:	1c 00 00 00 	*unknown*
    4cb4:	20 00 00 14 	l.sys 0x14
    4cb8:	a5 02 04 05 	l.andi r8,r2,0x405
    4cbc:	69 6e 74 00 	*unknown*
    4cc0:	03 04 05 00 	l.j fc1060c0 <_end+0xfc101654>
    4cc4:	00 00 05 03 	l.j 60d0 <_end+0x1664>
    4cc8:	04 07 00 00 	l.jal 1c4cc8 <_end+0x1c025c>
    4ccc:	00 5e 03 01 	l.j 17858d0 <_end+0x1780e64>
    4cd0:	06 00 00 00 	l.jal f8004cd0 <_end+0xf8000264>
    4cd4:	7b 03 01 08 	*unknown*
    4cd8:	00 00 00 79 	l.j 4ebc <_end+0x450>
    4cdc:	03 02 05 00 	l.j fc0860dc <_end+0xfc081670>
    4ce0:	00 01 2f 03 	l.j 508ec <_end+0x4be80>
    4ce4:	02 07 00 00 	l.j f81c4ce4 <_end+0xf81c0278>
    4ce8:	00 0e 03 08 	l.j 385908 <_end+0x380e9c>
    4cec:	05 00 00 00 	l.jal 4004cec <_end+0x4000280>
    4cf0:	00 03 08 07 	l.j c6d0c <_end+0xc22a0>
    4cf4:	00 00 00 59 	l.j 4e58 <_end+0x3ec>
    4cf8:	04 00 00 03 	l.jal 4d04 <_end+0x298>
    4cfc:	6d 02 07 00 	l.lwa r8,1792(r2)
    4d00:	00 00 25 04 	l.j e110 <_end+0x96a4>
    4d04:	00 00 03 4b 	l.j 5a30 <_end+0xfc4>
    4d08:	03 10 00 00 	l.j fc404d08 <_end+0xfc40029c>
    4d0c:	00 2c 04 00 	l.j b05d0c <_end+0xb012a0>
    4d10:	00 04 27 03 	l.j 10e91c <_end+0x109eb0>
    4d14:	27 00 00 00 	*unknown*
    4d18:	2c 05 00 00 	*unknown*
    4d1c:	03 17 04 01 	l.j fc5c5d20 <_end+0xfc5c12b4>
    4d20:	61 00 00 00 	*unknown*
    4d24:	91 03 04 07 	l.lbs r8,1031(r3)
    4d28:	00 00 00 63 	l.j 4eb4 <_end+0x448>
    4d2c:	06 04 03 4a 	l.jal f8105a54 <_end+0xf8100fe8>
    4d30:	00 00 00 b7 	l.j 500c <_end+0x5a0>
    4d34:	07 00 00 03 	l.jal fc004d40 <_end+0xfc0002d4>
    4d38:	11 03 4c 00 	l.bf 40d7d38 <_end+0x40d32cc>
    4d3c:	00 00 85 07 	l.j 26158 <_end+0x216ec>
    4d40:	00 00 02 84 	l.j 5750 <_end+0xce4>
    4d44:	03 4d 00 00 	l.j fd344d44 <_end+0xfd3402d8>
    4d48:	00 b7 00 08 	l.j 2dc4d68 <_end+0x2dc02fc>
    4d4c:	00 00 00 41 	l.j 4e50 <_end+0x3e4>
    4d50:	00 00 00 c7 	l.j 506c <_end+0x600>
    4d54:	09 00 00 00 	*unknown*
    4d58:	c7 03 00 03 	*unknown*
    4d5c:	04 07 00 00 	l.jal 1c4d5c <_end+0x1c02f0>
    4d60:	01 45 0a 08 	l.j 5147580 <_end+0x5142b14>
    4d64:	03 47 00 00 	l.j fd1c4d64 <_end+0xfd1c02f8>
    4d68:	00 ef 0b 00 	l.j 3bc7968 <_end+0x3bc2efc>
    4d6c:	00 04 11 03 	l.j 109178 <_end+0x10470c>
    4d70:	49 00 00 00 	*unknown*
    4d74:	25 00 0b 00 	*unknown*
    4d78:	00 04 19 03 	l.j 10b184 <_end+0x106718>
    4d7c:	4e 00 00 00 	*unknown*
    4d80:	98 04 00 04 	l.lhs r0,4(r4)
    4d84:	00 00 03 ab 	l.j 5c30 <_end+0x11c4>
    4d88:	03 4f 00 00 	l.j fd3c4d88 <_end+0xfd3c031c>
    4d8c:	00 ce 04 00 	l.j 3385d8c <_end+0x3381320>
    4d90:	00 02 47 03 	l.j 9699c <_end+0x91f30>
    4d94:	53 00 00 00 	*unknown*
    4d98:	64 0c 04 04 	*unknown*
    4d9c:	00 00 04 54 	l.j 5eec <_end+0x1480>
    4da0:	05 16 00 00 	l.jal 4584da0 <_end+0x4580334>
    4da4:	00 33 0d 00 	l.j cc81a4 <_end+0xcc3738>
    4da8:	00 02 58 18 	l.j 9ae08 <_end+0x9639c>
    4dac:	05 2d 00 00 	l.jal 4b44dac <_end+0x4b40340>
    4db0:	01 65 0b 00 	l.j 59479b0 <_end+0x5942f44>
    4db4:	00 03 cc 05 	l.j f7dc8 <_end+0xf335c>
    4db8:	2f 00 00 01 	*unknown*
    4dbc:	65 00 0e 5f 	*unknown*
    4dc0:	6b 00 05 30 	*unknown*
    4dc4:	00 00 00 25 	l.j 4e58 <_end+0x3ec>
    4dc8:	04 0b 00 00 	l.jal 2c4dc8 <_end+0x2c035c>
    4dcc:	04 03 05 30 	l.jal c628c <_end+0xc1820>
    4dd0:	00 00 00 25 	l.j 4e64 <_end+0x3f8>
    4dd4:	08 0b 00 00 	*unknown*
    4dd8:	02 41 05 30 	l.j f9046298 <_end+0xf904182c>
    4ddc:	00 00 00 25 	l.j 4e70 <_end+0x404>
    4de0:	0c 0b 00 00 	l.bnf 2c4de0 <_end+0x2c0374>
    4de4:	04 a0 05 30 	l.jal 28062a4 <_end+0x2801838>
    4de8:	00 00 00 25 	l.j 4e7c <_end+0x410>
    4dec:	10 0e 5f 78 	l.bf 39cbcc <_end+0x398160>
    4df0:	00 05 31 00 	l.j 1511f0 <_end+0x14c784>
    4df4:	00 01 6b 14 	l.j 5fa44 <_end+0x5afd8>
    4df8:	00 0f 04 00 	l.j 3c5df8 <_end+0x3c138c>
    4dfc:	00 01 12 08 	l.j 4961c <_end+0x44bb0>
    4e00:	00 00 01 07 	l.j 521c <_end+0x7b0>
    4e04:	00 00 01 7b 	l.j 53f0 <_end+0x984>
    4e08:	09 00 00 00 	*unknown*
    4e0c:	c7 00 00 0d 	*unknown*
    4e10:	00 00 02 7f 	l.j 580c <_end+0xda0>
    4e14:	24 05 35 00 	*unknown*
    4e18:	00 01 f4 0b 	l.j 81e44 <_end+0x7d3d8>
    4e1c:	00 00 01 b3 	l.j 54e8 <_end+0xa7c>
    4e20:	05 37 00 00 	l.jal 4dc4e20 <_end+0x4dc03b4>
    4e24:	00 25 00 0b 	l.j 944e50 <_end+0x9403e4>
    4e28:	00 00 04 2f 	l.j 5ee4 <_end+0x1478>
    4e2c:	05 38 00 00 	l.jal 4e04e2c <_end+0x4e003c0>
    4e30:	00 25 04 0b 	l.j 945e5c <_end+0x9413f0>
    4e34:	00 00 01 c2 	l.j 553c <_end+0xad0>
    4e38:	05 39 00 00 	l.jal 4e44e38 <_end+0x4e403cc>
    4e3c:	00 25 08 0b 	l.j 946e68 <_end+0x9423fc>
    4e40:	00 00 05 19 	l.j 62a4 <_end+0x1838>
    4e44:	05 3a 00 00 	l.jal 4e84e44 <_end+0x4e803d8>
    4e48:	00 25 0c 0b 	l.j 947e74 <_end+0x943408>
    4e4c:	00 00 03 37 	l.j 5b28 <_end+0x10bc>
    4e50:	05 3b 00 00 	l.jal 4ec4e50 <_end+0x4ec03e4>
    4e54:	00 25 10 0b 	l.j 948e80 <_end+0x944414>
    4e58:	00 00 03 26 	l.j 5af0 <_end+0x1084>
    4e5c:	05 3c 00 00 	l.jal 4f04e5c <_end+0x4f003f0>
    4e60:	00 25 14 0b 	l.j 949e8c <_end+0x945420>
    4e64:	00 00 04 a5 	l.j 60f8 <_end+0x168c>
    4e68:	05 3d 00 00 	l.jal 4f44e68 <_end+0x4f403fc>
    4e6c:	00 25 18 0b 	l.j 94ae98 <_end+0x94642c>
    4e70:	00 00 03 8d 	l.j 5ca4 <_end+0x1238>
    4e74:	05 3e 00 00 	l.jal 4f84e74 <_end+0x4f80408>
    4e78:	00 25 1c 0b 	l.j 94bea4 <_end+0x947438>
    4e7c:	00 00 04 e0 	l.j 61fc <_end+0x1790>
    4e80:	05 3f 00 00 	l.jal 4fc4e80 <_end+0x4fc0414>
    4e84:	00 25 20 00 	l.j 94ce84 <_end+0x948418>
    4e88:	10 00 00 01 	l.bf 4e8c <_end+0x420>
    4e8c:	d1 01 08 05 	*unknown*
    4e90:	48 00 00 02 	*unknown*
    4e94:	34 0b 00 00 	*unknown*
    4e98:	02 34 05 49 	l.j f8d063bc <_end+0xf8d01950>
    4e9c:	00 00 02 34 	l.j 576c <_end+0xd00>
    4ea0:	00 0b 00 00 	l.j 2c4ea0 <_end+0x2c0434>
    4ea4:	01 4e 05 4a 	l.j 53863cc <_end+0x5381960>
    4ea8:	00 00 02 34 	l.j 5778 <_end+0xd0c>
    4eac:	80 11 00 00 	*unknown*
    4eb0:	04 4b 05 4c 	l.jal 12c63e0 <_end+0x12c1974>
    4eb4:	00 00 01 07 	l.j 52d0 <_end+0x864>
    4eb8:	01 00 11 00 	l.j 40092b8 <_end+0x400484c>
    4ebc:	00 01 f6 05 	l.j 826d0 <_end+0x7dc64>
    4ec0:	4f 00 00 01 	*unknown*
    4ec4:	07 01 04 00 	l.jal fc045ec4 <_end+0xfc041458>
    4ec8:	08 00 00 01 	*unknown*
    4ecc:	05 00 00 02 	l.jal 4004ed4 <_end+0x4000468>
    4ed0:	44 09 00 00 	*unknown*
    4ed4:	00 c7 1f 00 	l.j 31ccad4 <_end+0x31c8068>
    4ed8:	10 00 00 01 	l.bf 4edc <_end+0x470>
    4edc:	3d 01 90 05 	*unknown*
    4ee0:	5b 00 00 02 	*unknown*
    4ee4:	82 0b 00 00 	*unknown*
    4ee8:	03 cc 05 5c 	l.j ff306458 <_end+0xff3019ec>
    4eec:	00 00 02 82 	l.j 58f4 <_end+0xe88>
    4ef0:	00 0b 00 00 	l.j 2c4ef0 <_end+0x2c0484>
    4ef4:	03 e4 05 5d 	l.j ff906468 <_end+0xff9019fc>
    4ef8:	00 00 00 25 	l.j 4f8c <_end+0x520>
    4efc:	04 0b 00 00 	l.jal 2c4efc <_end+0x2c0490>
    4f00:	02 3c 05 5f 	l.j f8f0647c <_end+0xf8f01a10>
    4f04:	00 00 02 88 	l.j 5924 <_end+0xeb8>
    4f08:	08 0b 00 00 	*unknown*
    4f0c:	01 d1 05 60 	l.j 744648c <_end+0x7441a20>
    4f10:	00 00 01 f4 	l.j 56e0 <_end+0xc74>
    4f14:	88 00 0f 04 	l.lws r0,3844(r0)
    4f18:	00 00 02 44 	l.j 5828 <_end+0xdbc>
    4f1c:	08 00 00 02 	*unknown*
    4f20:	98 00 00 02 	l.lhs r0,2(r0)
    4f24:	98 09 00 00 	l.lhs r0,0(r9)
    4f28:	00 c7 1f 00 	l.j 31ccb28 <_end+0x31c80bc>
    4f2c:	0f 04 00 00 	l.bnf fc104f2c <_end+0xfc1004c0>
    4f30:	02 9e 12 0d 	l.j fa789764 <_end+0xfa784cf8>
    4f34:	00 00 03 97 	l.j 5d90 <_end+0x1324>
    4f38:	08 05 73 00 	*unknown*
    4f3c:	00 02 c4 0b 	l.j b5f68 <_end+0xb14fc>
    4f40:	00 00 09 f8 	l.j 7720 <_end+0x2cb4>
    4f44:	05 74 00 00 	l.jal 5d04f44 <_end+0x5d004d8>
    4f48:	02 c4 00 0b 	l.j fb104f74 <_end+0xfb100508>
    4f4c:	00 00 07 8b 	l.j 6d78 <_end+0x230c>
    4f50:	05 75 00 00 	l.jal 5d44f50 <_end+0x5d404e4>
    4f54:	00 25 04 00 	l.j 945f54 <_end+0x9414e8>
    4f58:	0f 04 00 00 	l.bnf fc104f58 <_end+0xfc1004ec>
    4f5c:	00 41 0d 00 	l.j 104835c <_end+0x10438f0>
    4f60:	00 03 b6 68 	l.j f2900 <_end+0xede94>
    4f64:	05 b3 00 00 	l.jal 6cc4f64 <_end+0x6cc04f8>
    4f68:	03 f4 0e 5f 	l.j ffd088e4 <_end+0xffd03e78>
    4f6c:	70 00 05 b4 	*unknown*
    4f70:	00 00 02 c4 	l.j 5a80 <_end+0x1014>
    4f74:	00 0e 5f 72 	l.j 39cd3c <_end+0x3982d0>
    4f78:	00 05 b5 00 	l.j 172378 <_end+0x16d90c>
    4f7c:	00 00 25 04 	l.j e38c <_end+0x9920>
    4f80:	0e 5f 77 00 	l.bnf f97e2b80 <_end+0xf97de114>
    4f84:	05 b6 00 00 	l.jal 6d84f84 <_end+0x6d80518>
    4f88:	00 25 08 0b 	l.j 946fb4 <_end+0x942548>
    4f8c:	00 00 01 ef 	l.j 5748 <_end+0xcdc>
    4f90:	05 b7 00 00 	l.jal 6dc4f90 <_end+0x6dc0524>
    4f94:	00 48 0c 0b 	l.j 1207fc0 <_end+0x1203554>
    4f98:	00 00 02 9b 	l.j 5a04 <_end+0xf98>
    4f9c:	05 b8 00 00 	l.jal 6e04f9c <_end+0x6e00530>
    4fa0:	00 48 0e 0e 	l.j 12087d8 <_end+0x1203d6c>
    4fa4:	5f 62 66 00 	*unknown*
    4fa8:	05 b9 00 00 	l.jal 6e44fa8 <_end+0x6e4053c>
    4fac:	02 9f 10 0b 	l.j fa7c8fd8 <_end+0xfa7c456c>
    4fb0:	00 00 01 8d 	l.j 55e4 <_end+0xb78>
    4fb4:	05 ba 00 00 	l.jal 6e84fb4 <_end+0x6e80548>
    4fb8:	00 25 18 0b 	l.j 94afe4 <_end+0x946578>
    4fbc:	00 00 01 df 	l.j 5738 <_end+0xccc>
    4fc0:	05 c1 00 00 	l.jal 7044fc0 <_end+0x7040554>
    4fc4:	01 05 1c 0b 	l.j 414bff0 <_end+0x4147584>
    4fc8:	00 00 02 6f 	l.j 5984 <_end+0xf18>
    4fcc:	05 c3 00 00 	l.jal 70c4fcc <_end+0x70c0560>
    4fd0:	05 57 20 0b 	l.jal 55ccffc <_end+0x55c8590>
    4fd4:	00 00 09 a6 	l.j 766c <_end+0x2c00>
    4fd8:	05 c5 00 00 	l.jal 7144fd8 <_end+0x714056c>
    4fdc:	05 86 24 0b 	l.jal 618e008 <_end+0x618959c>
    4fe0:	00 00 04 21 	l.j 6064 <_end+0x15f8>
    4fe4:	05 c8 00 00 	l.jal 7204fe4 <_end+0x7200578>
    4fe8:	05 aa 28 0b 	l.jal 6a8f014 <_end+0x6a8a5a8>
    4fec:	00 00 04 fa 	l.j 63d4 <_end+0x1968>
    4ff0:	05 c9 00 00 	l.jal 7244ff0 <_end+0x7240584>
    4ff4:	05 c4 2c 0e 	l.jal 711002c <_end+0x710b5c0>
    4ff8:	5f 75 62 00 	*unknown*
    4ffc:	05 cc 00 00 	l.jal 7304ffc <_end+0x7300590>
    5000:	02 9f 30 0e 	l.j fa7d1038 <_end+0xfa7cc5cc>
    5004:	5f 75 70 00 	*unknown*
    5008:	05 cd 00 00 	l.jal 7345008 <_end+0x734059c>
    500c:	02 c4 38 0e 	l.j fb113044 <_end+0xfb10e5d8>
    5010:	5f 75 72 00 	*unknown*
    5014:	05 ce 00 00 	l.jal 7385014 <_end+0x73805a8>
    5018:	00 25 3c 0b 	l.j 954044 <_end+0x94f5d8>
    501c:	00 00 01 bc 	l.j 570c <_end+0xca0>
    5020:	05 d1 00 00 	l.jal 7445020 <_end+0x74405b4>
    5024:	05 ca 40 0b 	l.jal 7295050 <_end+0x72905e4>
    5028:	00 00 04 d2 	l.j 6370 <_end+0x1904>
    502c:	05 d2 00 00 	l.jal 748502c <_end+0x74805c0>
    5030:	05 da 43 0e 	l.jal 7695c68 <_end+0x76911fc>
    5034:	5f 6c 62 00 	*unknown*
    5038:	05 d5 00 00 	l.jal 7545038 <_end+0x75405cc>
    503c:	02 9f 44 0b 	l.j fa7d6068 <_end+0xfa7d15fc>
    5040:	00 00 08 18 	l.j 70a0 <_end+0x2634>
    5044:	05 d8 00 00 	l.jal 7605044 <_end+0x76005d8>
    5048:	00 25 4c 0b 	l.j 958074 <_end+0x953608>
    504c:	00 00 02 0d 	l.j 5880 <_end+0xe14>
    5050:	05 d9 00 00 	l.jal 7645050 <_end+0x76405e4>
    5054:	00 6f 50 0b 	l.j 1bd9080 <_end+0x1bd4614>
    5058:	00 00 0c 6a 	l.j 8200 <_end+0x3794>
    505c:	05 dc 00 00 	l.jal 770505c <_end+0x77005f0>
    5060:	04 12 54 0b 	l.jal 49a08c <_end+0x495620>
    5064:	00 00 06 4f 	l.j 69a0 <_end+0x1f34>
    5068:	05 e0 00 00 	l.jal 7805068 <_end+0x78005fc>
    506c:	00 fa 58 0b 	l.j 3e9b098 <_end+0x3e9662c>
    5070:	00 00 03 be 	l.j 5f68 <_end+0x14fc>
    5074:	05 e2 00 00 	l.jal 7885074 <_end+0x7880608>
    5078:	00 ef 5c 0b 	l.j 3bdc0a4 <_end+0x3bd7638>
    507c:	00 00 03 1e 	l.j 5cf4 <_end+0x1288>
    5080:	05 e3 00 00 	l.jal 78c5080 <_end+0x78c0614>
    5084:	00 25 64 00 	l.j 95e084 <_end+0x959618>
    5088:	13 00 00 00 	l.bf fc005088 <_end+0xfc00061c>
    508c:	25 00 00 04 	*unknown*
    5090:	12 14 00 00 	l.bf f8505090 <_end+0xf8500624>
    5094:	04 12 14 00 	l.jal 48a094 <_end+0x485628>
    5098:	00 01 05 14 	l.j 464e8 <_end+0x41a7c>
    509c:	00 00 05 4a 	l.j 65c4 <_end+0x1b58>
    50a0:	14 00 00 00 	*unknown*
    50a4:	25 00 0f 04 	*unknown*
    50a8:	00 00 04 18 	l.j 6108 <_end+0x169c>
    50ac:	15 00 00 0c 	l.nop 0xc
    50b0:	0b 04 24 05 	*unknown*
    50b4:	02 39 00 00 	l.j f8e450b4 <_end+0xf8e40648>
    50b8:	05 4a 16 00 	l.jal 528a8b8 <_end+0x5285e4c>
    50bc:	00 06 bd 05 	l.j 1b44d0 <_end+0x1afa64>
    50c0:	02 3b 00 00 	l.j f8ec50c0 <_end+0xf8ec0654>
    50c4:	00 25 00 16 	l.j 94511c <_end+0x9406b0>
    50c8:	00 00 01 fe 	l.j 58c0 <_end+0xe54>
    50cc:	05 02 40 00 	l.jal 40950cc <_end+0x4090660>
    50d0:	00 06 31 04 	l.j 1914e0 <_end+0x18ca74>
    50d4:	16 00 00 02 	*unknown*
    50d8:	8b 05 02 40 	l.lws r24,576(r5)
    50dc:	00 00 06 31 	l.j 69a0 <_end+0x1f34>
    50e0:	08 16 00 00 	*unknown*
    50e4:	02 50 05 02 	l.j f94064ec <_end+0xf9401a80>
    50e8:	40 00 00 06 	*unknown*
    50ec:	31 0c 16 00 	*unknown*
    50f0:	00 03 df 05 	l.j fcd04 <_end+0xf8298>
    50f4:	02 42 00 00 	l.j f90850f4 <_end+0xf9080688>
    50f8:	00 25 10 16 	l.j 949150 <_end+0x9446e4>
    50fc:	00 00 01 62 	l.j 5684 <_end+0xc18>
    5100:	05 02 43 00 	l.jal 4095d00 <_end+0x4091294>
    5104:	00 08 13 14 	l.j 209d54 <_end+0x2052e8>
    5108:	16 00 00 04 	*unknown*
    510c:	7c 05 02 45 	*unknown*
    5110:	00 00 00 25 	l.j 51a4 <_end+0x738>
    5114:	30 16 00 00 	*unknown*
    5118:	03 e9 05 02 	l.j ffa46520 <_end+0xffa41ab4>
    511c:	46 00 00 05 	*unknown*
    5120:	7b 34 16 00 	*unknown*
    5124:	00 03 40 05 	l.j d5138 <_end+0xd06cc>
    5128:	02 48 00 00 	l.j f9205128 <_end+0xf92006bc>
    512c:	00 25 38 16 	l.j 953184 <_end+0x94e718>
    5130:	00 00 03 f9 	l.j 6114 <_end+0x16a8>
    5134:	05 02 4a 00 	l.jal 4097934 <_end+0x4092ec8>
    5138:	00 08 2e 3c 	l.j 210a28 <_end+0x20bfbc>
    513c:	16 00 00 03 	*unknown*
    5140:	09 05 02 4d 	*unknown*
    5144:	00 00 01 65 	l.j 56d8 <_end+0xc6c>
    5148:	40 16 00 00 	*unknown*
    514c:	02 75 05 02 	l.j f9d46554 <_end+0xf9d41ae8>
    5150:	4e 00 00 00 	*unknown*
    5154:	25 44 16 00 	*unknown*
    5158:	00 05 14 05 	l.j 14a16c <_end+0x145700>
    515c:	02 4f 00 00 	l.j f93c515c <_end+0xf93c06f0>
    5160:	01 65 48 16 	l.j 59571b8 <_end+0x595274c>
    5164:	00 00 03 63 	l.j 5ef0 <_end+0x1484>
    5168:	05 02 50 00 	l.jal 4099168 <_end+0x40946fc>
    516c:	00 08 34 4c 	l.j 21229c <_end+0x20d830>
    5170:	16 00 00 02 	*unknown*
    5174:	93 05 02 53 	l.lbs r24,595(r5)
    5178:	00 00 00 25 	l.j 520c <_end+0x7a0>
    517c:	50 16 00 00 	*unknown*
    5180:	02 05 05 02 	l.j f8146588 <_end+0xf8141b1c>
    5184:	54 00 00 05 	*unknown*
    5188:	4a 54 16 00 	*unknown*
    518c:	00 03 7f 05 	l.j e4da0 <_end+0xe0334>
    5190:	02 77 00 00 	l.j f9dc5190 <_end+0xf9dc0724>
    5194:	07 f1 58 17 	l.jal ffc5b1f0 <_end+0xffc56784>
    5198:	00 00 01 3d 	l.j 568c <_end+0xc20>
    519c:	05 02 7b 00 	l.jal 40a3d9c <_end+0x409f330>
    51a0:	00 02 82 01 	l.j a59a4 <_end+0xa0f38>
    51a4:	48 17 00 00 	*unknown*
    51a8:	02 e7 05 02 	l.j fb9c65b0 <_end+0xfb9c1b44>
    51ac:	7c 00 00 02 	*unknown*
    51b0:	44 01 4c 17 	*unknown*
    51b4:	00 00 04 c8 	l.j 64d4 <_end+0x1a68>
    51b8:	05 02 80 00 	l.jal 40a51b8 <_end+0x40a074c>
    51bc:	00 08 45 02 	l.j 2165c4 <_end+0x211b58>
    51c0:	dc 17 00 00 	l.sh 0(r23),r0
    51c4:	01 e7 05 02 	l.j 79c65cc <_end+0x79c1b60>
    51c8:	85 00 00 05 	l.lwz r8,5(r0)
    51cc:	f6 02 e0 17 	*unknown*
    51d0:	00 00 01 cc 	l.j 5900 <_end+0xe94>
    51d4:	05 02 86 00 	l.jal 40a69d4 <_end+0x40a1f68>
    51d8:	00 08 51 02 	l.j 2195e0 <_end+0x214b74>
    51dc:	ec 00 0f 04 	*unknown*
    51e0:	00 00 05 50 	l.j 6720 <_end+0x1cb4>
    51e4:	03 01 06 00 	l.j fc0469e4 <_end+0xfc041f78>
    51e8:	00 00 82 0f 	l.j 25a24 <_end+0x20fb8>
    51ec:	04 00 00 03 	l.jal 51f8 <_end+0x78c>
    51f0:	f4 13 00 00 	*unknown*
    51f4:	00 25 00 00 	l.j 9451f4 <_end+0x940788>
    51f8:	05 7b 14 00 	l.jal 5eca1f8 <_end+0x5ec578c>
    51fc:	00 04 12 14 	l.j 109a4c <_end+0x104fe0>
    5200:	00 00 01 05 	l.j 5614 <_end+0xba8>
    5204:	14 00 00 05 	*unknown*
    5208:	7b 14 00 00 	*unknown*
    520c:	00 25 00 0f 	l.j 945248 <_end+0x9407dc>
    5210:	04 00 00 05 	l.jal 5224 <_end+0x7b8>
    5214:	81 18 00 00 	*unknown*
    5218:	05 50 0f 04 	l.jal 5408e28 <_end+0x54043bc>
    521c:	00 00 05 5d 	l.j 6790 <_end+0x1d24>
    5220:	13 00 00 00 	l.bf fc005220 <_end+0xfc0007b4>
    5224:	7a 00 00 05 	*unknown*
    5228:	aa 14 00 00 	l.ori r16,r20,0x0
    522c:	04 12 14 00 	l.jal 48a22c <_end+0x4857c0>
    5230:	00 01 05 14 	l.j 46680 <_end+0x41c14>
    5234:	00 00 00 7a 	l.j 541c <_end+0x9b0>
    5238:	14 00 00 00 	*unknown*
    523c:	25 00 0f 04 	*unknown*
    5240:	00 00 05 8c 	l.j 6870 <_end+0x1e04>
    5244:	13 00 00 00 	l.bf fc005244 <_end+0xfc0007d8>
    5248:	25 00 00 05 	*unknown*
    524c:	c4 14 00 00 	*unknown*
    5250:	04 12 14 00 	l.jal 48a250 <_end+0x4857e4>
    5254:	00 01 05 00 	l.j 46654 <_end+0x41be8>
    5258:	0f 04 00 00 	l.bnf fc105258 <_end+0xfc1007ec>
    525c:	05 b0 08 00 	l.jal 6c0725c <_end+0x6c027f0>
    5260:	00 00 41 00 	l.j 15660 <_end+0x10bf4>
    5264:	00 05 da 09 	l.j 17ba88 <_end+0x17701c>
    5268:	00 00 00 c7 	l.j 5584 <_end+0xb18>
    526c:	02 00 08 00 	l.j f800726c <_end+0xf8002800>
    5270:	00 00 41 00 	l.j 15670 <_end+0x10c04>
    5274:	00 05 ea 09 	l.j 17fa98 <_end+0x17b02c>
    5278:	00 00 00 c7 	l.j 5594 <_end+0xb28>
    527c:	00 00 05 00 	l.j 667c <_end+0x1c10>
    5280:	00 03 a4 05 	l.j ee294 <_end+0xe9828>
    5284:	01 1d 00 00 	l.j 4745284 <_end+0x4740818>
    5288:	02 ca 19 00 	l.j fb28b688 <_end+0xfb286c1c>
    528c:	00 04 af 0c 	l.j 130ebc <_end+0x12c450>
    5290:	05 01 21 00 	l.jal 404d690 <_end+0x4048c24>
    5294:	00 06 2b 16 	l.j 18feec <_end+0x18b480>
    5298:	00 00 03 cc 	l.j 61c8 <_end+0x175c>
    529c:	05 01 23 00 	l.jal 404de9c <_end+0x4049430>
    52a0:	00 06 2b 00 	l.j 18fea0 <_end+0x18b434>
    52a4:	16 00 00 02 	*unknown*
    52a8:	a1 05 01 24 	l.addic r8,r5,292
    52ac:	00 00 00 25 	l.j 5340 <_end+0x8d4>
    52b0:	04 16 00 00 	l.jal 5852b0 <_end+0x580844>
    52b4:	03 9e 05 01 	l.j fe7866b8 <_end+0xfe781c4c>
    52b8:	25 00 00 06 	*unknown*
    52bc:	31 08 00 0f 	*unknown*
    52c0:	04 00 00 05 	l.jal 52d4 <_end+0x868>
    52c4:	f6 0f 04 00 	*unknown*
    52c8:	00 05 ea 19 	l.j 17fb2c <_end+0x17b0c0>
    52cc:	00 00 01 5a 	l.j 5834 <_end+0xdc8>
    52d0:	0e 05 01 3d 	l.bnf f81457c4 <_end+0xf8140d58>
    52d4:	00 00 06 6c 	l.j 6c84 <_end+0x2218>
    52d8:	16 00 00 04 	*unknown*
    52dc:	0b 05 01 3e 	*unknown*
    52e0:	00 00 06 6c 	l.j 6c90 <_end+0x2224>
    52e4:	00 16 00 00 	l.j 5852e4 <_end+0x580878>
    52e8:	04 38 05 01 	l.jal e066ec <_end+0xe01c80>
    52ec:	3f 00 00 06 	*unknown*
    52f0:	6c 06 16 00 	l.lwa r0,5632(r6)
    52f4:	00 0e 6d 05 	l.j 3a0708 <_end+0x39bc9c>
    52f8:	01 40 00 00 	l.j 50052f8 <_end+0x500088c>
    52fc:	00 4f 0c 00 	l.j 13c82fc <_end+0x13c3890>
    5300:	08 00 00 00 	*unknown*
    5304:	4f 00 00 06 	*unknown*
    5308:	7c 09 00 00 	*unknown*
    530c:	00 c7 02 00 	l.j 31c5b0c <_end+0x31c10a0>
    5310:	1a cc 05 02 	*unknown*
    5314:	58 00 00 07 	*unknown*
    5318:	7d 16 00 00 	*unknown*
    531c:	04 93 05 02 	l.jal 24c6724 <_end+0x24c1cb8>
    5320:	5a 00 00 00 	*unknown*
    5324:	91 00 16 00 	l.lbs r8,5632(r0)
    5328:	00 04 3e 05 	l.j 114b3c <_end+0x1100d0>
    532c:	02 5b 00 00 	l.j f96c532c <_end+0xf96c08c0>
    5330:	05 4a 04 16 	l.jal 5286388 <_end+0x528191c>
    5334:	00 00 02 fc 	l.j 5f24 <_end+0x14b8>
    5338:	05 02 5c 00 	l.jal 409c338 <_end+0x40978cc>
    533c:	00 07 7d 08 	l.j 1e475c <_end+0x1dfcf0>
    5340:	16 00 00 04 	*unknown*
    5344:	eb 05 02 5d 	*unknown*
    5348:	00 00 01 7b 	l.j 5934 <_end+0xec8>
    534c:	24 16 00 00 	*unknown*
    5350:	02 60 05 02 	l.j f9806758 <_end+0xf9801cec>
    5354:	5e 00 00 00 	*unknown*
    5358:	25 48 16 00 	*unknown*
    535c:	00 03 c7 05 	l.j f6f70 <_end+0xf2504>
    5360:	02 5f 00 00 	l.j f97c5360 <_end+0xf97c08f4>
    5364:	00 5d 4c 16 	l.j 17583bc <_end+0x1753950>
    5368:	00 00 05 01 	l.j 676c <_end+0x1d00>
    536c:	05 02 60 00 	l.jal 409d36c <_end+0x4098900>
    5370:	00 06 37 54 	l.j 1930c0 <_end+0x18e654>
    5374:	16 00 00 03 	*unknown*
    5378:	d2 05 02 61 	*unknown*
    537c:	00 00 00 ef 	l.j 5738 <_end+0xccc>
    5380:	64 16 00 00 	*unknown*
    5384:	05 06 05 02 	l.jal 418678c <_end+0x4181d20>
    5388:	62 00 00 00 	*unknown*
    538c:	ef 6c 16 00 	*unknown*
    5390:	00 01 a5 05 	l.j 6e7a4 <_end+0x69d38>
    5394:	02 63 00 00 	l.j f98c5394 <_end+0xf98c0928>
    5398:	00 ef 74 16 	l.j 3be23f0 <_end+0x3bdd984>
    539c:	00 00 04 be 	l.j 6694 <_end+0x1c28>
    53a0:	05 02 64 00 	l.jal 409e3a0 <_end+0x4099934>
    53a4:	00 07 8d 7c 	l.j 1e8994 <_end+0x1e3f28>
    53a8:	16 00 00 02 	*unknown*
    53ac:	f0 05 02 65 	*unknown*
    53b0:	00 00 07 9d 	l.j 7224 <_end+0x27b8>
    53b4:	84 16 00 00 	l.lwz r0,0(r22)
    53b8:	04 5c 05 02 	l.jal 17067c0 <_end+0x1701d54>
    53bc:	66 00 00 00 	*unknown*
    53c0:	25 9c 16 00 	*unknown*
    53c4:	00 02 26 05 	l.j 8ebd8 <_end+0x8a16c>
    53c8:	02 67 00 00 	l.j f99c53c8 <_end+0xf99c095c>
    53cc:	00 ef a0 16 	l.j 3bed424 <_end+0x3be89b8>
    53d0:	00 00 01 96 	l.j 5a28 <_end+0xfbc>
    53d4:	05 02 68 00 	l.jal 409f3d4 <_end+0x409a968>
    53d8:	00 00 ef a8 	l.j 41278 <_end+0x3c80c>
    53dc:	16 00 00 02 	*unknown*
    53e0:	15 05 02 69 	*unknown*
    53e4:	00 00 00 ef 	l.j 57a0 <_end+0xd34>
    53e8:	b0 16 00 00 	l.muli r0,r22,0
    53ec:	01 6d 05 02 	l.j 5b467f4 <_end+0x5b41d88>
    53f0:	6a 00 00 00 	*unknown*
    53f4:	ef b8 16 00 	*unknown*
    53f8:	00 01 7c 05 	l.j 6440c <_end+0x5f9a0>
    53fc:	02 6b 00 00 	l.j f9ac53fc <_end+0xf9ac0990>
    5400:	00 ef c0 16 	l.j 3bf5458 <_end+0x3bf09ec>
    5404:	00 00 03 84 	l.j 6214 <_end+0x17a8>
    5408:	05 02 6c 00 	l.jal 40a0408 <_end+0x409b99c>
    540c:	00 00 25 c8 	l.j eb2c <_end+0xa0c0>
    5410:	00 08 00 00 	l.j 205410 <_end+0x2009a4>
    5414:	05 50 00 00 	l.jal 5405414 <_end+0x54009a8>
    5418:	07 8d 09 00 	l.jal fe347818 <_end+0xfe342dac>
    541c:	00 00 c7 19 	l.j 37080 <_end+0x32614>
    5420:	00 08 00 00 	l.j 205420 <_end+0x2009b4>
    5424:	05 50 00 00 	l.jal 5405424 <_end+0x54009b8>
    5428:	07 9d 09 00 	l.jal fe747828 <_end+0xfe742dbc>
    542c:	00 00 c7 07 	l.j 37048 <_end+0x325dc>
    5430:	00 08 00 00 	l.j 205430 <_end+0x2009c4>
    5434:	05 50 00 00 	l.jal 5405434 <_end+0x54009c8>
    5438:	07 ad 09 00 	l.jal feb47838 <_end+0xfeb42dcc>
    543c:	00 00 c7 17 	l.j 37098 <_end+0x3262c>
    5440:	00 1a f0 05 	l.j 6c1454 <_end+0x6bc9e8>
    5444:	02 71 00 00 	l.j f9c45444 <_end+0xf9c409d8>
    5448:	07 d1 16 00 	l.jal ff44ac48 <_end+0xff4461dc>
    544c:	00 03 30 05 	l.j d1460 <_end+0xcc9f4>
    5450:	02 74 00 00 	l.j f9d05450 <_end+0xf9d009e4>
    5454:	07 d1 00 16 	l.jal ff4454ac <_end+0xff440a40>
    5458:	00 00 04 b5 	l.j 672c <_end+0x1cc0>
    545c:	05 02 75 00 	l.jal 40a285c <_end+0x409ddf0>
    5460:	00 07 e1 78 	l.j 1fda40 <_end+0x1f8fd4>
    5464:	00 08 00 00 	l.j 205464 <_end+0x2009f8>
    5468:	02 c4 00 00 	l.j fb105468 <_end+0xfb1009fc>
    546c:	07 e1 09 00 	l.jal ff84786c <_end+0xff842e00>
    5470:	00 00 c7 1d 	l.j 370e4 <_end+0x32678>
    5474:	00 08 00 00 	l.j 205474 <_end+0x200a08>
    5478:	00 91 00 00 	l.j 2445478 <_end+0x2440a0c>
    547c:	07 f1 09 00 	l.jal ffc4787c <_end+0xffc42e10>
    5480:	00 00 c7 1d 	l.j 370f4 <_end+0x32688>
    5484:	00 1b f0 05 	l.j 701498 <_end+0x6fca2c>
    5488:	02 56 00 00 	l.j f9585488 <_end+0xf9580a1c>
    548c:	08 13 1c 00 	*unknown*
    5490:	00 0c 0b 05 	l.j 3080a4 <_end+0x303638>
    5494:	02 6d 00 00 	l.j f9b45494 <_end+0xf9b40a28>
    5498:	06 7c 1c 00 	l.jal f9f0c498 <_end+0xf9f07a2c>
    549c:	00 04 d8 05 	l.j 13b4b0 <_end+0x136a44>
    54a0:	02 76 00 00 	l.j f9d854a0 <_end+0xf9d80a34>
    54a4:	07 ad 00 08 	l.jal feb454c4 <_end+0xfeb40a58>
    54a8:	00 00 05 50 	l.j 69e8 <_end+0x1f7c>
    54ac:	00 00 08 23 	l.j 7538 <_end+0x2acc>
    54b0:	09 00 00 00 	*unknown*
    54b4:	c7 18 00 1d 	*unknown*
    54b8:	00 00 08 2e 	l.j 7570 <_end+0x2b04>
    54bc:	14 00 00 04 	*unknown*
    54c0:	12 00 0f 04 	l.bf f80090d0 <_end+0xf8004664>
    54c4:	00 00 08 23 	l.j 7550 <_end+0x2ae4>
    54c8:	0f 04 00 00 	l.bnf fc1054c8 <_end+0xfc100a5c>
    54cc:	01 65 1d 00 	l.j 594c8cc <_end+0x5947e60>
    54d0:	00 08 45 14 	l.j 216920 <_end+0x211eb4>
    54d4:	00 00 00 25 	l.j 5568 <_end+0xafc>
    54d8:	00 0f 04 00 	l.j 3c64d8 <_end+0x3c1a6c>
    54dc:	00 08 4b 0f 	l.j 218118 <_end+0x2136ac>
    54e0:	04 00 00 08 	l.jal 5500 <_end+0xa94>
    54e4:	3a 08 00 00 	*unknown*
    54e8:	05 ea 00 00 	l.jal 7a854e8 <_end+0x7a80a7c>
    54ec:	08 61 09 00 	*unknown*
    54f0:	00 00 c7 02 	l.j 370f8 <_end+0x3268c>
    54f4:	00 1e 00 00 	l.j 7854f4 <_end+0x780a88>
    54f8:	06 bc 06 0f 	l.jal faf06d34 <_end+0xfaf022c8>
    54fc:	00 00 08 84 	l.j 770c <_end+0x2ca0>
    5500:	00 00 37 1c 	l.j 13170 <_end+0xe704>
    5504:	00 00 00 20 	l.j 5584 <_end+0xb18>
    5508:	01 9c 00 00 	l.j 6705508 <_end+0x6700a9c>
    550c:	08 84 1f 00 	*unknown*
    5510:	00 37 2c 00 	l.j dd0510 <_end+0xdcbaa4>
    5514:	00 08 8a 00 	l.j 227d14 <_end+0x2232a8>
    5518:	0f 04 00 00 	l.bnf fc105518 <_end+0xfc100aac>
    551c:	00 25 20 00 	l.j 94d51c <_end+0x948ab0>
    5520:	00 0f 85 05 	l.j 3e6934 <_end+0x3e1ec8>
    5524:	03 03 00 00 	l.j fc0c5524 <_end+0xfc0c0ab8>
    5528:	04 12 00 00 	l.jal 485528 <_end+0x480abc>
    552c:	00 01 13 00 	l.j 4a12c <_end+0x456c0>
    5530:	04 00 00 15 	l.jal 5584 <_end+0xb18>
    5534:	bd 04 01 00 	*unknown*
    5538:	00 00 30 01 	l.j 1153c <_end+0xcad0>
    553c:	00 00 10 15 	l.j 9590 <_end+0x4b24>
    5540:	00 00 10 6a 	l.j 96e8 <_end+0x4c7c>
    5544:	00 00 37 44 	l.j 13254 <_end+0xe7e8>
    5548:	00 00 01 68 	l.j 5ae8 <_end+0x107c>
    554c:	00 00 15 c6 	l.j ac64 <_end+0x61f8>
    5550:	02 04 05 00 	l.j f8106950 <_end+0xf8101ee4>
    5554:	00 00 05 03 	l.j 6960 <_end+0x1ef4>
    5558:	00 00 08 c2 	l.j 7860 <_end+0x2df4>
    555c:	02 d4 00 00 	l.j fb50555c <_end+0xfb500af0>
    5560:	00 37 02 04 	l.j dc5d70 <_end+0xdc1304>
    5564:	07 00 00 00 	l.jal fc005564 <_end+0xfc000af8>
    5568:	5e 04 04 05 	*unknown*
    556c:	69 6e 74 00 	*unknown*
    5570:	02 01 06 00 	l.j f8046d70 <_end+0xf8042304>
    5574:	00 00 7b 02 	l.j 2417c <_end+0x1f710>
    5578:	01 08 00 00 	l.j 4205578 <_end+0x4200b0c>
    557c:	00 79 02 02 	l.j 1e45d84 <_end+0x1e41318>
    5580:	05 00 00 01 	l.jal 4005584 <_end+0x4000b18>
    5584:	2f 02 02 07 	*unknown*
    5588:	00 00 00 0e 	l.j 55c0 <_end+0xb54>
    558c:	02 08 05 00 	l.j f820698c <_end+0xf8201f20>
    5590:	00 00 00 02 	l.j 5598 <_end+0xb2c>
    5594:	08 07 00 00 	*unknown*
    5598:	00 59 02 04 	l.j 1645da8 <_end+0x164133c>
    559c:	07 00 00 00 	l.jal fc00559c <_end+0xfc000b30>
    55a0:	63 02 04 07 	*unknown*
    55a4:	00 00 01 45 	l.j 5ab8 <_end+0x104c>
    55a8:	05 04 06 04 	l.jal 4106db8 <_end+0x410234c>
    55ac:	00 00 00 85 	l.j 57c0 <_end+0xd54>
    55b0:	02 01 06 00 	l.j f8046db0 <_end+0xf8042344>
    55b4:	00 00 82 07 	l.j 25dd0 <_end+0x21364>
    55b8:	00 00 0c 92 	l.j 8800 <_end+0x3d94>
    55bc:	03 19 00 00 	l.j fc6455bc <_end+0xfc640b50>
    55c0:	00 7d 00 00 	l.j 1f455c0 <_end+0x1f40b54>
    55c4:	37 44 00 00 	*unknown*
    55c8:	01 68 01 9c 	l.j 5a05c38 <_end+0x5a011cc>
    55cc:	00 00 01 10 	l.j 5a0c <_end+0xfa0>
    55d0:	08 6d 00 01 	*unknown*
    55d4:	2d 00 00 00 	*unknown*
    55d8:	7d 01 53 09 	*unknown*
    55dc:	63 00 01 2d 	*unknown*
    55e0:	00 00 00 3e 	l.j 56d8 <_end+0xc6c>
    55e4:	00 00 0b ab 	l.j 8490 <_end+0x3a24>
    55e8:	09 6e 00 01 	*unknown*
    55ec:	2d 00 00 00 	*unknown*
    55f0:	2c 00 00 0b 	*unknown*
    55f4:	d7 0a 73 00 	l.sw -15616(r10),r14
    55f8:	01 32 00 00 	l.j 4c855f8 <_end+0x4c80b8c>
    55fc:	00 7f 00 00 	l.j 1fc55fc <_end+0x1fc0b90>
    5600:	0c 2c 0a 69 	l.bnf b07fa4 <_end+0xb03538>
    5604:	00 01 35 00 	l.j 52a04 <_end+0x4df98>
    5608:	00 00 6f 00 	l.j 21208 <_end+0x1c79c>
    560c:	00 0c 90 0b 	l.j 329638 <_end+0x324bcc>
    5610:	00 00 10 63 	l.j 979c <_end+0x4d30>
    5614:	01 36 00 00 	l.j 4d85614 <_end+0x4d80ba8>
    5618:	00 37 00 00 	l.j dc5618 <_end+0xdc0bac>
    561c:	0c a5 0b 00 	l.bnf 294821c <_end+0x29437b0>
    5620:	00 10 56 01 	l.j 41ae24 <_end+0x4163b8>
    5624:	37 00 00 01 	*unknown*
    5628:	10 00 00 0c 	l.bf 5658 <_end+0xbec>
    562c:	c4 0a 64 00 	*unknown*
    5630:	01 38 00 00 	l.j 4e05630 <_end+0x4e00bc4>
    5634:	00 6f 00 00 	l.j 1bc5634 <_end+0x1bc0bc8>
    5638:	0d 2c 00 06 	l.bnf 4b05650 <_end+0x4b00be4>
    563c:	04 00 00 00 	l.jal 563c <_end+0xbd0>
    5640:	Address 0x0000000000005640 is out of bounds.


Disassembly of section .debug_abbrev:

00000000 <.debug_abbrev>:
       0:	01 11 01 25 	l.j 4440494 <_end+0x443ba28>
       4:	0e 13 0b 03 	l.bnf f84c2c10 <_end+0xf84be1a4>
       8:	0e 1b 0e 11 	l.bnf f86c384c <_end+0xf86bede0>
       c:	01 12 06 10 	l.j 448184c <_end+0x447cde0>
      10:	17 00 00 02 	*unknown*
      14:	24 00 0b 0b 	*unknown*
      18:	3e 0b 03 0e 	*unknown*
      1c:	00 00 03 24 	l.j cac <_or1k_reset+0xbac>
      20:	00 0b 0b 3e 	l.j 2c2d18 <_end+0x2be2ac>
      24:	0b 03 08 00 	*unknown*
      28:	00 04 0f 00 	l.j 103c28 <_end+0xff1bc>
      2c:	0b 0b 00 00 	*unknown*
      30:	05 0f 00 0b 	l.jal 43c005c <_end+0x43bb5f0>
      34:	0b 49 13 00 	*unknown*
      38:	00 06 15 00 	l.j 185438 <_end+0x1809cc>
      3c:	27 19 00 00 	*unknown*
      40:	07 04 01 03 	l.jal fc10044c <_end+0xfc0fb9e0>
      44:	0e 0b 0b 3a 	l.bnf f82c2d2c <_end+0xf82be2c0>
      48:	0b 3b 0b 01 	*unknown*
      4c:	13 00 00 08 	l.bf fc00006c <_end+0xfbffb600>
      50:	28 00 03 0e 	*unknown*
      54:	1c 0d 00 00 	*unknown*
      58:	09 2e 01 3f 	*unknown*
      5c:	19 03 0e 3a 	*unknown*
      60:	0b 3b 0b 27 	*unknown*
      64:	19 49 13 11 	*unknown*
      68:	01 12 06 40 	l.j 4481968 <_end+0x447cefc>
      6c:	18 97 42 19 	*unknown*
      70:	01 13 00 00 	l.j 44c0070 <_end+0x44bb604>
      74:	0a 05 00 03 	*unknown*
      78:	08 3a 0b 3b 	*unknown*
      7c:	0b 49 13 02 	*unknown*
      80:	17 00 00 0b 	*unknown*
      84:	89 82 01 01 	l.lws r12,257(r2)
      88:	11 01 31 13 	l.bf 404c4d4 <_end+0x4047a68>
      8c:	00 00 0c 8a 	l.j 32b4 <_or1k_init+0x24>
      90:	82 01 00 02 	*unknown*
      94:	18 91 42 18 	*unknown*
      98:	00 00 0d 2e 	l.j 3550 <or1k_timer_set_mode+0x38>
      9c:	01 3f 19 03 	l.j 4fc64a8 <_end+0x4fc1a3c>
      a0:	0e 3a 0b 3b 	l.bnf f8e82d8c <_end+0xf8e7e320>
      a4:	0b 27 19 49 	*unknown*
      a8:	13 3c 19 00 	l.bf fcf064a8 <_end+0xfcf01a3c>
      ac:	00 0e 05 00 	l.j 3814ac <_end+0x37ca40>
      b0:	49 13 00 00 	*unknown*
      b4:	00 01 11 01 	l.j 444b8 <_end+0x3fa4c>
      b8:	25 0e 13 0b 	*unknown*
      bc:	03 0e 1b 0e 	l.j fc386cf4 <_end+0xfc382288>
      c0:	11 01 12 06 	l.bf 40448d8 <_end+0x403fe6c>
      c4:	10 17 00 00 	l.bf 5c00c4 <_end+0x5bb658>
      c8:	02 24 00 0b 	l.j f89000f4 <_end+0xf88fb688>
      cc:	0b 3e 0b 03 	*unknown*
      d0:	0e 00 00 03 	l.bnf f80000dc <_end+0xf7ffb670>
      d4:	24 00 0b 0b 	*unknown*
      d8:	3e 0b 03 08 	*unknown*
      dc:	00 00 04 16 	l.j 1134 <_or1k_reset+0x1034>
      e0:	00 03 0e 3a 	l.j c39c8 <_end+0xbef5c>
      e4:	0b 3b 0b 49 	*unknown*
      e8:	13 00 00 05 	l.bf fc0000fc <_end+0xfbffb690>
      ec:	16 00 03 0e 	*unknown*
      f0:	3a 0b 3b 05 	*unknown*
      f4:	49 13 00 00 	*unknown*
      f8:	06 17 01 0b 	l.jal f85c0524 <_end+0xf85bbab8>
      fc:	0b 3a 0b 3b 	*unknown*
     100:	0b 01 13 00 	*unknown*
     104:	00 07 0d 00 	l.j 1c3504 <_end+0x1bea98>
     108:	03 0e 3a 0b 	l.j fc38e934 <_end+0xfc389ec8>
     10c:	3b 0b 49 13 	*unknown*
     110:	00 00 08 01 	l.j 2114 <_or1k_start+0xe4>
     114:	01 49 13 01 	l.j 5244d18 <_end+0x52402ac>
     118:	13 00 00 09 	l.bf fc00013c <_end+0xfbffb6d0>
     11c:	21 00 49 13 	l.trap 0x4913
     120:	2f 0b 00 00 	*unknown*
     124:	0a 13 01 0b 	*unknown*
     128:	0b 3a 0b 3b 	*unknown*
     12c:	0b 01 13 00 	*unknown*
     130:	00 0b 0d 00 	l.j 2c3530 <_end+0x2beac4>
     134:	03 0e 3a 0b 	l.j fc38e960 <_end+0xfc389ef4>
     138:	3b 0b 49 13 	*unknown*
     13c:	38 0b 00 00 	*unknown*
     140:	0c 0f 00 0b 	l.bnf 3c016c <_end+0x3bb700>
     144:	0b 00 00 0d 	*unknown*
     148:	13 01 03 0e 	l.bf fc040d80 <_end+0xfc03c314>
     14c:	0b 0b 3a 0b 	*unknown*
     150:	3b 0b 01 13 	*unknown*
     154:	00 00 0e 0d 	l.j 3988 <__udivsi3+0xdc>
     158:	00 03 08 3a 	l.j c2240 <_end+0xbd7d4>
     15c:	0b 3b 0b 49 	*unknown*
     160:	13 38 0b 00 	l.bf fce02d60 <_end+0xfcdfe2f4>
     164:	00 0f 0f 00 	l.j 3c3d64 <_end+0x3bf2f8>
     168:	0b 0b 49 13 	*unknown*
     16c:	00 00 10 13 	l.j 41b8 <impure_data+0x164>
     170:	01 03 0e 0b 	l.j 40c399c <_end+0x40bef30>
     174:	05 3a 0b 3b 	l.jal 4e82e60 <_end+0x4e7e3f4>
     178:	0b 01 13 00 	*unknown*
     17c:	00 11 0d 00 	l.j 44357c <_end+0x43eb10>
     180:	03 0e 3a 0b 	l.j fc38e9ac <_end+0xfc389f40>
     184:	3b 0b 49 13 	*unknown*
     188:	38 05 00 00 	*unknown*
     18c:	12 15 00 27 	l.bf f8540228 <_end+0xf853b7bc>
     190:	19 00 00 13 	l.movhi r8,0x13
     194:	15 01 27 19 	*unknown*
     198:	49 13 01 13 	*unknown*
     19c:	00 00 14 05 	l.j 51b0 <_end+0x744>
     1a0:	00 49 13 00 	l.j 1244da0 <_end+0x1240334>
     1a4:	00 15 13 01 	l.j 544da8 <_end+0x54033c>
     1a8:	03 0e 0b 05 	l.j fc382dbc <_end+0xfc37e350>
     1ac:	3a 0b 3b 05 	*unknown*
     1b0:	01 13 00 00 	l.j 44c01b0 <_end+0x44bb744>
     1b4:	16 0d 00 03 	*unknown*
     1b8:	0e 3a 0b 3b 	l.bnf f8e82ea4 <_end+0xf8e7e438>
     1bc:	05 49 13 38 	l.jal 5244e9c <_end+0x5240430>
     1c0:	0b 00 00 17 	*unknown*
     1c4:	0d 00 03 0e 	l.bnf 4000dfc <_end+0x3ffc390>
     1c8:	3a 0b 3b 05 	*unknown*
     1cc:	49 13 38 05 	*unknown*
     1d0:	00 00 18 26 	l.j 6268 <_end+0x17fc>
     1d4:	00 49 13 00 	l.j 1244dd4 <_end+0x1240368>
     1d8:	00 19 13 01 	l.j 644ddc <_end+0x640370>
     1dc:	03 0e 0b 0b 	l.j fc382e08 <_end+0xfc37e39c>
     1e0:	3a 0b 3b 05 	*unknown*
     1e4:	01 13 00 00 	l.j 44c01e4 <_end+0x44bb778>
     1e8:	1a 13 01 0b 	*unknown*
     1ec:	0b 3a 0b 3b 	*unknown*
     1f0:	05 01 13 00 	l.jal 4044df0 <_end+0x4040384>
     1f4:	00 1b 17 01 	l.j 6c5df8 <_end+0x6c138c>
     1f8:	0b 0b 3a 0b 	*unknown*
     1fc:	3b 05 01 13 	*unknown*
     200:	00 00 1c 0d 	l.j 7234 <_end+0x27c8>
     204:	00 03 0e 3a 	l.j c3aec <_end+0xbf080>
     208:	0b 3b 05 49 	*unknown*
     20c:	13 00 00 1d 	l.bf fc000280 <_end+0xfbffb814>
     210:	15 01 27 19 	*unknown*
     214:	01 13 00 00 	l.j 44c0214 <_end+0x44bb7a8>
     218:	1e 2e 01 3f 	*unknown*
     21c:	19 03 0e 3a 	*unknown*
     220:	0b 3b 0b 27 	*unknown*
     224:	19 11 01 12 	*unknown*
     228:	06 40 18 96 	l.jal f9006480 <_end+0xf9001a14>
     22c:	42 19 01 13 	*unknown*
     230:	00 00 1f 05 	l.j 7e44 <_end+0x33d8>
     234:	00 03 0e 3a 	l.j c3b1c <_end+0xbf0b0>
     238:	0b 3b 0b 49 	*unknown*
     23c:	13 02 17 00 	l.bf fc085e3c <_end+0xfc0813d0>
     240:	00 20 89 82 	l.j 822848 <_end+0x81dddc>
     244:	01 01 11 01 	l.j 4044648 <_end+0x403fbdc>
     248:	31 13 01 13 	*unknown*
     24c:	00 00 21 8a 	l.j 8874 <_end+0x3e08>
     250:	82 01 00 02 	*unknown*
     254:	18 91 42 18 	*unknown*
     258:	00 00 22 89 	l.j 8c7c <_end+0x4210>
     25c:	82 01 01 11 	*unknown*
     260:	01 31 13 00 	l.j 4c44e60 <_end+0x4c403f4>
     264:	00 23 34 00 	l.j 8cd264 <_end+0x8c87f8>
     268:	03 0e 3a 0b 	l.j fc38ea94 <_end+0xfc38a028>
     26c:	3b 05 49 13 	*unknown*
     270:	3f 19 3c 19 	*unknown*
     274:	00 00 24 2e 	l.j 932c <_end+0x48c0>
     278:	01 3f 19 03 	l.j 4fc6684 <_end+0x4fc1c18>
     27c:	0e 3a 0b 3b 	l.bnf f8e82f68 <_end+0xf8e7e4fc>
     280:	0b 27 19 3c 	*unknown*
     284:	19 01 13 00 	*unknown*
     288:	00 25 2e 01 	l.j 94ba8c <_end+0x947020>
     28c:	3f 19 03 0e 	*unknown*
     290:	3a 0b 3b 0b 	*unknown*
     294:	27 19 3c 19 	*unknown*
     298:	00 00 00 01 	l.j 29c <_or1k_reset+0x19c>
     29c:	11 01 25 0e 	l.bf 40496d4 <_end+0x4044c68>
     2a0:	13 0b 03 0e 	l.bf fc2c0ed8 <_end+0xfc2bc46c>
     2a4:	1b 0e 10 17 	*unknown*
     2a8:	00 00 02 24 	l.j b38 <_or1k_reset+0xa38>
     2ac:	00 0b 0b 3e 	l.j 2c2fa4 <_end+0x2be538>
     2b0:	0b 03 0e 00 	*unknown*
     2b4:	00 03 24 00 	l.j c92b4 <_end+0xc4848>
     2b8:	0b 0b 3e 0b 	*unknown*
     2bc:	03 08 00 00 	l.j fc2002bc <_end+0xfc1fb850>
     2c0:	04 16 00 03 	l.jal 5802cc <_end+0x57b860>
     2c4:	0e 3a 0b 3b 	l.bnf f8e82fb0 <_end+0xf8e7e544>
     2c8:	0b 49 13 00 	*unknown*
     2cc:	00 05 16 00 	l.j 145acc <_end+0x141060>
     2d0:	03 0e 3a 0b 	l.j fc38eafc <_end+0xfc38a090>
     2d4:	3b 05 49 13 	*unknown*
     2d8:	00 00 06 17 	l.j 1b34 <_or1k_reset+0x1a34>
     2dc:	01 0b 0b 3a 	l.j 42c2fc4 <_end+0x42be558>
     2e0:	0b 3b 0b 01 	*unknown*
     2e4:	13 00 00 07 	l.bf fc000300 <_end+0xfbffb894>
     2e8:	0d 00 03 0e 	l.bnf 4000f20 <_end+0x3ffc4b4>
     2ec:	3a 0b 3b 0b 	*unknown*
     2f0:	49 13 00 00 	*unknown*
     2f4:	08 01 01 49 	*unknown*
     2f8:	13 01 13 00 	l.bf fc044ef8 <_end+0xfc04048c>
     2fc:	00 09 21 00 	l.j 2486fc <_end+0x243c90>
     300:	49 13 2f 0b 	*unknown*
     304:	00 00 0a 13 	l.j 2b50 <_or1k_uart_write+0x24>
     308:	01 0b 0b 3a 	l.j 42c2ff0 <_end+0x42be584>
     30c:	0b 3b 0b 01 	*unknown*
     310:	13 00 00 0b 	l.bf fc00033c <_end+0xfbffb8d0>
     314:	0d 00 03 0e 	l.bnf 4000f4c <_end+0x3ffc4e0>
     318:	3a 0b 3b 0b 	*unknown*
     31c:	49 13 38 0b 	*unknown*
     320:	00 00 0c 0f 	l.j 335c <or1k_exception_handler_add+0xc>
     324:	00 0b 0b 00 	l.j 2c2f24 <_end+0x2be4b8>
     328:	00 0d 13 01 	l.j 344f2c <_end+0x3404c0>
     32c:	03 0e 0b 0b 	l.j fc382f58 <_end+0xfc37e4ec>
     330:	3a 0b 3b 0b 	*unknown*
     334:	01 13 00 00 	l.j 44c0334 <_end+0x44bb8c8>
     338:	0e 0d 00 03 	l.bnf f8340344 <_end+0xf833b8d8>
     33c:	08 3a 0b 3b 	*unknown*
     340:	0b 49 13 38 	*unknown*
     344:	0b 00 00 0f 	*unknown*
     348:	0f 00 0b 0b 	l.bnf fc002f74 <_end+0xfbffe508>
     34c:	49 13 00 00 	*unknown*
     350:	10 13 01 03 	l.bf 4c075c <_end+0x4bbcf0>
     354:	0e 0b 05 3a 	l.bnf f82c183c <_end+0xf82bcdd0>
     358:	0b 3b 0b 01 	*unknown*
     35c:	13 00 00 11 	l.bf fc0003a0 <_end+0xfbffb934>
     360:	0d 00 03 0e 	l.bnf 4000f98 <_end+0x3ffc52c>
     364:	3a 0b 3b 0b 	*unknown*
     368:	49 13 38 05 	*unknown*
     36c:	00 00 12 15 	l.j 4bc0 <_end+0x154>
     370:	00 27 19 00 	l.j 9c6770 <_end+0x9c1d04>
     374:	00 13 15 01 	l.j 4c5778 <_end+0x4c0d0c>
     378:	27 19 49 13 	*unknown*
     37c:	01 13 00 00 	l.j 44c037c <_end+0x44bb910>
     380:	14 05 00 49 	*unknown*
     384:	13 00 00 15 	l.bf fc0003d8 <_end+0xfbffb96c>
     388:	13 01 03 0e 	l.bf fc040fc0 <_end+0xfc03c554>
     38c:	0b 05 3a 0b 	*unknown*
     390:	3b 05 01 13 	*unknown*
     394:	00 00 16 0d 	l.j 5bc8 <_end+0x115c>
     398:	00 03 0e 3a 	l.j c3c80 <_end+0xbf214>
     39c:	0b 3b 05 49 	*unknown*
     3a0:	13 38 0b 00 	l.bf fce02fa0 <_end+0xfcdfe534>
     3a4:	00 17 0d 00 	l.j 5c37a4 <_end+0x5bed38>
     3a8:	03 0e 3a 0b 	l.j fc38ebd4 <_end+0xfc38a168>
     3ac:	3b 05 49 13 	*unknown*
     3b0:	38 05 00 00 	*unknown*
     3b4:	18 26 00 49 	*unknown*
     3b8:	13 00 00 19 	l.bf fc00041c <_end+0xfbffb9b0>
     3bc:	13 01 03 0e 	l.bf fc040ff4 <_end+0xfc03c588>
     3c0:	0b 0b 3a 0b 	*unknown*
     3c4:	3b 05 01 13 	*unknown*
     3c8:	00 00 1a 13 	l.j 6c14 <_end+0x21a8>
     3cc:	01 0b 0b 3a 	l.j 42c30b4 <_end+0x42be648>
     3d0:	0b 3b 05 01 	*unknown*
     3d4:	13 00 00 1b 	l.bf fc000440 <_end+0xfbffb9d4>
     3d8:	17 01 0b 0b 	*unknown*
     3dc:	3a 0b 3b 05 	*unknown*
     3e0:	01 13 00 00 	l.j 44c03e0 <_end+0x44bb974>
     3e4:	1c 0d 00 03 	*unknown*
     3e8:	0e 3a 0b 3b 	l.bnf f8e830d4 <_end+0xf8e7e668>
     3ec:	05 49 13 00 	l.jal 5244fec <_end+0x5240580>
     3f0:	00 1d 15 01 	l.j 7457f4 <_end+0x740d88>
     3f4:	27 19 01 13 	*unknown*
     3f8:	00 00 1e 34 	l.j 7cc8 <_end+0x325c>
     3fc:	00 03 0e 3a 	l.j c3ce4 <_end+0xbf278>
     400:	0b 3b 0b 49 	*unknown*
     404:	13 02 18 00 	l.bf fc086404 <_end+0xfc081998>
     408:	00 1f 34 00 	l.j 7cd408 <_end+0x7c899c>
     40c:	03 0e 3a 0b 	l.j fc38ec38 <_end+0xfc38a1cc>
     410:	3b 05 49 13 	*unknown*
     414:	3f 19 02 18 	*unknown*
     418:	00 00 00 01 	l.j 41c <_or1k_reset+0x31c>
     41c:	11 01 25 0e 	l.bf 4049854 <_end+0x4044de8>
     420:	13 0b 03 0e 	l.bf fc2c1058 <_end+0xfc2bc5ec>
     424:	1b 0e 11 01 	*unknown*
     428:	12 06 10 17 	l.bf f8184484 <_end+0xf817fa18>
     42c:	00 00 02 24 	l.j cbc <_or1k_reset+0xbbc>
     430:	00 0b 0b 3e 	l.j 2c3128 <_end+0x2be6bc>
     434:	0b 03 0e 00 	*unknown*
     438:	00 03 16 00 	l.j c5c38 <_end+0xc11cc>
     43c:	03 0e 3a 0b 	l.j fc38ec68 <_end+0xfc38a1fc>
     440:	3b 0b 49 13 	*unknown*
     444:	00 00 04 24 	l.j 14d4 <_or1k_reset+0x13d4>
     448:	00 0b 0b 3e 	l.j 2c3140 <_end+0x2be6d4>
     44c:	0b 03 08 00 	*unknown*
     450:	00 05 16 00 	l.j 145c50 <_end+0x1411e4>
     454:	03 0e 3a 0b 	l.j fc38ec80 <_end+0xfc38a214>
     458:	3b 05 49 13 	*unknown*
     45c:	00 00 06 17 	l.j 1cb8 <_or1k_reset+0x1bb8>
     460:	01 0b 0b 3a 	l.j 42c3148 <_end+0x42be6dc>
     464:	0b 3b 0b 01 	*unknown*
     468:	13 00 00 07 	l.bf fc000484 <_end+0xfbffba18>
     46c:	0d 00 03 0e 	l.bnf 40010a4 <_end+0x3ffc638>
     470:	3a 0b 3b 0b 	*unknown*
     474:	49 13 00 00 	*unknown*
     478:	08 01 01 49 	*unknown*
     47c:	13 01 13 00 	l.bf fc04507c <_end+0xfc040610>
     480:	00 09 21 00 	l.j 248880 <_end+0x243e14>
     484:	49 13 2f 0b 	*unknown*
     488:	00 00 0a 13 	l.j 2cd4 <_or1k_cache_init+0xcc>
     48c:	01 0b 0b 3a 	l.j 42c3174 <_end+0x42be708>
     490:	0b 3b 0b 01 	*unknown*
     494:	13 00 00 0b 	l.bf fc0004c0 <_end+0xfbffba54>
     498:	0d 00 03 0e 	l.bnf 40010d0 <_end+0x3ffc664>
     49c:	3a 0b 3b 0b 	*unknown*
     4a0:	49 13 38 0b 	*unknown*
     4a4:	00 00 0c 0f 	l.j 34e0 <or1k_timer_set_period+0x58>
     4a8:	00 0b 0b 00 	l.j 2c30a8 <_end+0x2be63c>
     4ac:	00 0d 13 01 	l.j 3450b0 <_end+0x340644>
     4b0:	03 0e 0b 0b 	l.j fc3830dc <_end+0xfc37e670>
     4b4:	3a 0b 3b 0b 	*unknown*
     4b8:	01 13 00 00 	l.j 44c04b8 <_end+0x44bba4c>
     4bc:	0e 0d 00 03 	l.bnf f83404c8 <_end+0xf833ba5c>
     4c0:	08 3a 0b 3b 	*unknown*
     4c4:	0b 49 13 38 	*unknown*
     4c8:	0b 00 00 0f 	*unknown*
     4cc:	0f 00 0b 0b 	l.bnf fc0030f8 <_end+0xfbffe68c>
     4d0:	49 13 00 00 	*unknown*
     4d4:	10 13 01 03 	l.bf 4c08e0 <_end+0x4bbe74>
     4d8:	0e 0b 05 3a 	l.bnf f82c19c0 <_end+0xf82bcf54>
     4dc:	0b 3b 0b 01 	*unknown*
     4e0:	13 00 00 11 	l.bf fc000524 <_end+0xfbffbab8>
     4e4:	0d 00 03 0e 	l.bnf 400111c <_end+0x3ffc6b0>
     4e8:	3a 0b 3b 0b 	*unknown*
     4ec:	49 13 38 05 	*unknown*
     4f0:	00 00 12 15 	l.j 4d44 <_end+0x2d8>
     4f4:	00 27 19 00 	l.j 9c68f4 <_end+0x9c1e88>
     4f8:	00 13 15 01 	l.j 4c58fc <_end+0x4c0e90>
     4fc:	27 19 49 13 	*unknown*
     500:	01 13 00 00 	l.j 44c0500 <_end+0x44bba94>
     504:	14 05 00 49 	*unknown*
     508:	13 00 00 15 	l.bf fc00055c <_end+0xfbffbaf0>
     50c:	13 01 03 0e 	l.bf fc041144 <_end+0xfc03c6d8>
     510:	0b 05 3a 0b 	*unknown*
     514:	3b 05 01 13 	*unknown*
     518:	00 00 16 0d 	l.j 5d4c <_end+0x12e0>
     51c:	00 03 0e 3a 	l.j c3e04 <_end+0xbf398>
     520:	0b 3b 05 49 	*unknown*
     524:	13 38 0b 00 	l.bf fce03124 <_end+0xfcdfe6b8>
     528:	00 17 0d 00 	l.j 5c3928 <_end+0x5beebc>
     52c:	03 0e 3a 0b 	l.j fc38ed58 <_end+0xfc38a2ec>
     530:	3b 05 49 13 	*unknown*
     534:	38 05 00 00 	*unknown*
     538:	18 26 00 49 	*unknown*
     53c:	13 00 00 19 	l.bf fc0005a0 <_end+0xfbffbb34>
     540:	13 01 03 0e 	l.bf fc041178 <_end+0xfc03c70c>
     544:	0b 0b 3a 0b 	*unknown*
     548:	3b 05 01 13 	*unknown*
     54c:	00 00 1a 13 	l.j 6d98 <_end+0x232c>
     550:	01 0b 0b 3a 	l.j 42c3238 <_end+0x42be7cc>
     554:	0b 3b 05 01 	*unknown*
     558:	13 00 00 1b 	l.bf fc0005c4 <_end+0xfbffbb58>
     55c:	17 01 0b 0b 	*unknown*
     560:	3a 0b 3b 05 	*unknown*
     564:	01 13 00 00 	l.j 44c0564 <_end+0x44bbaf8>
     568:	1c 0d 00 03 	*unknown*
     56c:	0e 3a 0b 3b 	l.bnf f8e83258 <_end+0xf8e7e7ec>
     570:	05 49 13 00 	l.jal 5245170 <_end+0x5240704>
     574:	00 1d 15 01 	l.j 745978 <_end+0x740f0c>
     578:	27 19 01 13 	*unknown*
     57c:	00 00 1e 04 	l.j 7d8c <_end+0x3320>
     580:	01 03 0e 0b 	l.j 40c3dac <_end+0x40bf340>
     584:	0b 3a 0b 3b 	*unknown*
     588:	0b 01 13 00 	*unknown*
     58c:	00 1f 28 00 	l.j 7ca58c <_end+0x7c5b20>
     590:	03 0e 1c 0d 	l.j fc3875c4 <_end+0xfc382b58>
     594:	00 00 20 2e 	l.j 864c <_end+0x3be0>
     598:	01 3f 19 03 	l.j 4fc69a4 <_end+0x4fc1f38>
     59c:	0e 3a 0b 3b 	l.bnf f8e83288 <_end+0xf8e7e81c>
     5a0:	0b 27 19 49 	*unknown*
     5a4:	13 11 01 12 	l.bf fc4409ec <_end+0xfc43bf80>
     5a8:	06 40 18 97 	l.jal f9006804 <_end+0xf9001d98>
     5ac:	42 19 01 13 	*unknown*
     5b0:	00 00 21 05 	l.j 89c4 <_end+0x3f58>
     5b4:	00 03 0e 3a 	l.j c3e9c <_end+0xbf430>
     5b8:	0b 3b 0b 49 	*unknown*
     5bc:	13 02 17 00 	l.bf fc0861bc <_end+0xfc081750>
     5c0:	00 22 05 00 	l.j 8819c0 <_end+0x87cf54>
     5c4:	03 08 3a 0b 	l.j fc20edf0 <_end+0xfc20a384>
     5c8:	3b 0b 49 13 	*unknown*
     5cc:	02 17 00 00 	l.j f85c05cc <_end+0xf85bbb60>
     5d0:	23 34 00 03 	*unknown*
     5d4:	0e 3a 0b 3b 	l.bnf f8e832c0 <_end+0xf8e7e854>
     5d8:	0b 49 13 02 	*unknown*
     5dc:	17 00 00 24 	*unknown*
     5e0:	34 00 03 08 	*unknown*
     5e4:	3a 0b 3b 0b 	*unknown*
     5e8:	49 13 02 17 	*unknown*
     5ec:	00 00 25 89 	l.j 9c10 <_end+0x51a4>
     5f0:	82 01 01 11 	*unknown*
     5f4:	01 31 13 00 	l.j 4c451f4 <_end+0x4c40788>
     5f8:	00 26 8a 82 	l.j 9a3000 <_end+0x99e594>
     5fc:	01 00 02 18 	l.j 4000e5c <_end+0x3ffc3f0>
     600:	91 42 18 00 	l.lbs r10,6144(r2)
     604:	00 27 34 00 	l.j 9cd604 <_end+0x9c8b98>
     608:	03 0e 3a 0b 	l.j fc38ee34 <_end+0xfc38a3c8>
     60c:	3b 05 49 13 	*unknown*
     610:	3f 19 3c 19 	*unknown*
     614:	00 00 28 2e 	l.j a6cc <_end+0x5c60>
     618:	01 3f 19 03 	l.j 4fc6a24 <_end+0x4fc1fb8>
     61c:	0e 3a 0b 3b 	l.bnf f8e83308 <_end+0xf8e7e89c>
     620:	0b 27 19 49 	*unknown*
     624:	13 3c 19 00 	l.bf fcf06a24 <_end+0xfcf01fb8>
     628:	00 00 01 11 	l.j a6c <_or1k_reset+0x96c>
     62c:	01 25 0e 13 	l.j 4943e78 <_end+0x493f40c>
     630:	0b 03 0e 1b 	*unknown*
     634:	0e 11 01 12 	l.bnf f8440a7c <_end+0xf843c010>
     638:	06 10 17 00 	l.jal f8406238 <_end+0xf84017cc>
     63c:	00 02 24 00 	l.j 8963c <_end+0x84bd0>
     640:	0b 0b 3e 0b 	*unknown*
     644:	03 0e 00 00 	l.j fc380644 <_end+0xfc37bbd8>
     648:	03 24 00 0b 	l.j fc900674 <_end+0xfc8fbc08>
     64c:	0b 3e 0b 03 	*unknown*
     650:	08 00 00 04 	*unknown*
     654:	16 00 03 0e 	*unknown*
     658:	3a 0b 3b 0b 	*unknown*
     65c:	49 13 00 00 	*unknown*
     660:	05 16 00 03 	l.jal 458066c <_end+0x457bc00>
     664:	0e 3a 0b 3b 	l.bnf f8e83350 <_end+0xf8e7e8e4>
     668:	05 49 13 00 	l.jal 5245268 <_end+0x52407fc>
     66c:	00 06 17 01 	l.j 186270 <_end+0x181804>
     670:	0b 0b 3a 0b 	*unknown*
     674:	3b 0b 01 13 	*unknown*
     678:	00 00 07 0d 	l.j 22ac <__do_global_dtors_aux+0xa4>
     67c:	00 03 0e 3a 	l.j c3f64 <_end+0xbf4f8>
     680:	0b 3b 0b 49 	*unknown*
     684:	13 00 00 08 	l.bf fc0006a4 <_end+0xfbffbc38>
     688:	01 01 49 13 	l.j 4052ad4 <_end+0x404e068>
     68c:	01 13 00 00 	l.j 44c068c <_end+0x44bbc20>
     690:	09 21 00 49 	*unknown*
     694:	13 2f 0b 00 	l.bf fcbc3294 <_end+0xfcbbe828>
     698:	00 0a 13 01 	l.j 28529c <_end+0x280830>
     69c:	0b 0b 3a 0b 	*unknown*
     6a0:	3b 0b 01 13 	*unknown*
     6a4:	00 00 0b 0d 	l.j 32d8 <_or1k_init+0x48>
     6a8:	00 03 0e 3a 	l.j c3f90 <_end+0xbf524>
     6ac:	0b 3b 0b 49 	*unknown*
     6b0:	13 38 0b 00 	l.bf fce032b0 <_end+0xfcdfe844>
     6b4:	00 0c 0f 00 	l.j 3042b4 <_end+0x2ff848>
     6b8:	0b 0b 00 00 	*unknown*
     6bc:	0d 13 01 03 	l.bnf 44c0ac8 <_end+0x44bc05c>
     6c0:	0e 0b 0b 3a 	l.bnf f82c33a8 <_end+0xf82be93c>
     6c4:	0b 3b 0b 01 	*unknown*
     6c8:	13 00 00 0e 	l.bf fc000700 <_end+0xfbffbc94>
     6cc:	0d 00 03 08 	l.bnf 40012ec <_end+0x3ffc880>
     6d0:	3a 0b 3b 0b 	*unknown*
     6d4:	49 13 38 0b 	*unknown*
     6d8:	00 00 0f 0f 	l.j 4314 <impure_data+0x2c0>
     6dc:	00 0b 0b 49 	l.j 2c3400 <_end+0x2be994>
     6e0:	13 00 00 10 	l.bf fc000720 <_end+0xfbffbcb4>
     6e4:	13 01 03 0e 	l.bf fc04131c <_end+0xfc03c8b0>
     6e8:	0b 05 3a 0b 	*unknown*
     6ec:	3b 0b 01 13 	*unknown*
     6f0:	00 00 11 0d 	l.j 4b24 <_end+0xb8>
     6f4:	00 03 0e 3a 	l.j c3fdc <_end+0xbf570>
     6f8:	0b 3b 0b 49 	*unknown*
     6fc:	13 38 05 00 	l.bf fce01afc <_end+0xfcdfd090>
     700:	00 12 15 00 	l.j 485b00 <_end+0x481094>
     704:	27 19 00 00 	*unknown*
     708:	13 15 01 27 	l.bf fc540ba4 <_end+0xfc53c138>
     70c:	19 49 13 01 	*unknown*
     710:	13 00 00 14 	l.bf fc000760 <_end+0xfbffbcf4>
     714:	05 00 49 13 	l.jal 4012b60 <_end+0x400e0f4>
     718:	00 00 15 13 	l.j 5b64 <_end+0x10f8>
     71c:	01 03 0e 0b 	l.j 40c3f48 <_end+0x40bf4dc>
     720:	05 3a 0b 3b 	l.jal 4e8340c <_end+0x4e7e9a0>
     724:	05 01 13 00 	l.jal 4045324 <_end+0x40408b8>
     728:	00 16 0d 00 	l.j 583b28 <_end+0x57f0bc>
     72c:	03 0e 3a 0b 	l.j fc38ef58 <_end+0xfc38a4ec>
     730:	3b 05 49 13 	*unknown*
     734:	38 0b 00 00 	*unknown*
     738:	17 0d 00 03 	*unknown*
     73c:	0e 3a 0b 3b 	l.bnf f8e83428 <_end+0xf8e7e9bc>
     740:	05 49 13 38 	l.jal 5245420 <_end+0x52409b4>
     744:	05 00 00 18 	l.jal 40007a4 <_end+0x3ffbd38>
     748:	26 00 49 13 	*unknown*
     74c:	00 00 19 13 	l.j 6b98 <_end+0x212c>
     750:	01 03 0e 0b 	l.j 40c3f7c <_end+0x40bf510>
     754:	0b 3a 0b 3b 	*unknown*
     758:	05 01 13 00 	l.jal 4045358 <_end+0x40408ec>
     75c:	00 1a 13 01 	l.j 685360 <_end+0x6808f4>
     760:	0b 0b 3a 0b 	*unknown*
     764:	3b 05 01 13 	*unknown*
     768:	00 00 1b 17 	l.j 73c4 <_end+0x2958>
     76c:	01 0b 0b 3a 	l.j 42c3454 <_end+0x42be9e8>
     770:	0b 3b 05 01 	*unknown*
     774:	13 00 00 1c 	l.bf fc0007e4 <_end+0xfbffbd78>
     778:	0d 00 03 0e 	l.bnf 40013b0 <_end+0x3ffc944>
     77c:	3a 0b 3b 05 	*unknown*
     780:	49 13 00 00 	*unknown*
     784:	1d 15 01 27 	*unknown*
     788:	19 01 13 00 	*unknown*
     78c:	00 1e 2e 01 	l.j 78bf90 <_end+0x787524>
     790:	3f 19 03 0e 	*unknown*
     794:	3a 0b 3b 0b 	*unknown*
     798:	27 19 11 01 	*unknown*
     79c:	12 06 40 18 	l.bf f81907fc <_end+0xf818bd90>
     7a0:	96 42 19 01 	l.lhz r18,6401(r2)
     7a4:	13 00 00 1f 	l.bf fc000820 <_end+0xfbffbdb4>
     7a8:	05 00 03 0e 	l.jal 40013e0 <_end+0x3ffc974>
     7ac:	3a 0b 3b 0b 	*unknown*
     7b0:	49 13 02 17 	*unknown*
     7b4:	00 00 20 05 	l.j 87c8 <_end+0x3d5c>
     7b8:	00 03 08 3a 	l.j c28a0 <_end+0xbde34>
     7bc:	0b 3b 0b 49 	*unknown*
     7c0:	13 02 17 00 	l.bf fc0863c0 <_end+0xfc081954>
     7c4:	00 21 34 00 	l.j 84d7c4 <_end+0x848d58>
     7c8:	03 08 3a 0b 	l.j fc20eff4 <_end+0xfc20a588>
     7cc:	3b 0b 49 13 	*unknown*
     7d0:	02 17 00 00 	l.j f85c07d0 <_end+0xf85bbd64>
     7d4:	22 34 00 03 	*unknown*
     7d8:	0e 3a 0b 3b 	l.bnf f8e834c4 <_end+0xf8e7ea58>
     7dc:	0b 49 13 02 	*unknown*
     7e0:	17 00 00 23 	*unknown*
     7e4:	0a 00 03 0e 	*unknown*
     7e8:	3a 0b 3b 0b 	*unknown*
     7ec:	00 00 24 0b 	l.j 9818 <_end+0x4dac>
     7f0:	01 55 17 01 	l.j 55463f4 <_end+0x5541988>
     7f4:	13 00 00 25 	l.bf fc000888 <_end+0xfbffbe1c>
     7f8:	89 82 01 01 	l.lws r12,257(r2)
     7fc:	11 01 00 00 	l.bf 40407fc <_end+0x403bd90>
     800:	26 8a 82 01 	*unknown*
     804:	00 02 18 91 	l.j 86a48 <_end+0x81fdc>
     808:	42 18 00 00 	*unknown*
     80c:	27 89 82 01 	*unknown*
     810:	01 11 01 31 	l.j 4440cd4 <_end+0x443c268>
     814:	13 00 00 28 	l.bf fc0008b4 <_end+0xfbffbe48>
     818:	34 00 03 0e 	*unknown*
     81c:	3a 0b 3b 0b 	*unknown*
     820:	49 13 1c 0b 	*unknown*
     824:	00 00 29 34 	l.j acf4 <_end+0x6288>
     828:	00 03 0e 3a 	l.j c4110 <_end+0xbf6a4>
     82c:	0b 3b 05 49 	*unknown*
     830:	13 3f 19 3c 	l.bf fcfc6d20 <_end+0xfcfc22b4>
     834:	19 00 00 2a 	l.movhi r8,0x2a
     838:	2e 01 3f 19 	*unknown*
     83c:	03 0e 3a 0b 	l.j fc38f068 <_end+0xfc38a5fc>
     840:	3b 0b 27 19 	*unknown*
     844:	3c 19 00 00 	*unknown*
     848:	00 01 11 01 	l.j 44c4c <_end+0x401e0>
     84c:	25 0e 13 0b 	*unknown*
     850:	03 0e 1b 0e 	l.j fc387488 <_end+0xfc382a1c>
     854:	11 01 12 06 	l.bf 404506c <_end+0x4040600>
     858:	10 17 00 00 	l.bf 5c0858 <_end+0x5bbdec>
     85c:	02 24 00 0b 	l.j f8900888 <_end+0xf88fbe1c>
     860:	0b 3e 0b 03 	*unknown*
     864:	08 00 00 03 	*unknown*
     868:	24 00 0b 0b 	*unknown*
     86c:	3e 0b 03 0e 	*unknown*
     870:	00 00 04 16 	l.j 18c8 <_or1k_reset+0x17c8>
     874:	00 03 0e 3a 	l.j c415c <_end+0xbf6f0>
     878:	0b 3b 0b 49 	*unknown*
     87c:	13 00 00 05 	l.bf fc000890 <_end+0xfbffbe24>
     880:	16 00 03 0e 	*unknown*
     884:	3a 0b 3b 05 	*unknown*
     888:	49 13 00 00 	*unknown*
     88c:	06 17 01 0b 	l.jal f85c0cb8 <_end+0xf85bc24c>
     890:	0b 3a 0b 3b 	*unknown*
     894:	0b 01 13 00 	*unknown*
     898:	00 07 0d 00 	l.j 1c3c98 <_end+0x1bf22c>
     89c:	03 0e 3a 0b 	l.j fc38f0c8 <_end+0xfc38a65c>
     8a0:	3b 0b 49 13 	*unknown*
     8a4:	00 00 08 01 	l.j 28a8 <_fstat_r+0x8>
     8a8:	01 49 13 01 	l.j 52454ac <_end+0x5240a40>
     8ac:	13 00 00 09 	l.bf fc0008d0 <_end+0xfbffbe64>
     8b0:	21 00 49 13 	l.trap 0x4913
     8b4:	2f 0b 00 00 	*unknown*
     8b8:	0a 13 01 0b 	*unknown*
     8bc:	0b 3a 0b 3b 	*unknown*
     8c0:	0b 01 13 00 	*unknown*
     8c4:	00 0b 0d 00 	l.j 2c3cc4 <_end+0x2bf258>
     8c8:	03 0e 3a 0b 	l.j fc38f0f4 <_end+0xfc38a688>
     8cc:	3b 0b 49 13 	*unknown*
     8d0:	38 0b 00 00 	*unknown*
     8d4:	0c 0f 00 0b 	l.bnf 3c0900 <_end+0x3bbe94>
     8d8:	0b 00 00 0d 	*unknown*
     8dc:	13 01 03 0e 	l.bf fc041514 <_end+0xfc03caa8>
     8e0:	0b 0b 3a 0b 	*unknown*
     8e4:	3b 0b 01 13 	*unknown*
     8e8:	00 00 0e 0d 	l.j 411c <impure_data+0xc8>
     8ec:	00 03 08 3a 	l.j c29d4 <_end+0xbdf68>
     8f0:	0b 3b 0b 49 	*unknown*
     8f4:	13 38 0b 00 	l.bf fce034f4 <_end+0xfcdfea88>
     8f8:	00 0f 0f 00 	l.j 3c44f8 <_end+0x3bfa8c>
     8fc:	0b 0b 49 13 	*unknown*
     900:	00 00 10 13 	l.j 494c <_or1k_interrupt_handler_data_ptr_table>
     904:	01 03 0e 0b 	l.j 40c4130 <_end+0x40bf6c4>
     908:	05 3a 0b 3b 	l.jal 4e835f4 <_end+0x4e7eb88>
     90c:	0b 01 13 00 	*unknown*
     910:	00 11 0d 00 	l.j 443d10 <_end+0x43f2a4>
     914:	03 0e 3a 0b 	l.j fc38f140 <_end+0xfc38a6d4>
     918:	3b 0b 49 13 	*unknown*
     91c:	38 05 00 00 	*unknown*
     920:	12 15 00 27 	l.bf f85409bc <_end+0xf853bf50>
     924:	19 00 00 13 	l.movhi r8,0x13
     928:	15 01 27 19 	*unknown*
     92c:	49 13 01 13 	*unknown*
     930:	00 00 14 05 	l.j 5944 <_end+0xed8>
     934:	00 49 13 00 	l.j 1245534 <_end+0x1240ac8>
     938:	00 15 13 01 	l.j 54553c <_end+0x540ad0>
     93c:	03 0e 0b 05 	l.j fc383550 <_end+0xfc37eae4>
     940:	3a 0b 3b 05 	*unknown*
     944:	01 13 00 00 	l.j 44c0944 <_end+0x44bbed8>
     948:	16 0d 00 03 	*unknown*
     94c:	0e 3a 0b 3b 	l.bnf f8e83638 <_end+0xf8e7ebcc>
     950:	05 49 13 38 	l.jal 5245630 <_end+0x5240bc4>
     954:	0b 00 00 17 	*unknown*
     958:	0d 00 03 0e 	l.bnf 4001590 <_end+0x3ffcb24>
     95c:	3a 0b 3b 05 	*unknown*
     960:	49 13 38 05 	*unknown*
     964:	00 00 18 26 	l.j 69fc <_end+0x1f90>
     968:	00 49 13 00 	l.j 1245568 <_end+0x1240afc>
     96c:	00 19 13 01 	l.j 645570 <_end+0x640b04>
     970:	03 0e 0b 0b 	l.j fc38359c <_end+0xfc37eb30>
     974:	3a 0b 3b 05 	*unknown*
     978:	01 13 00 00 	l.j 44c0978 <_end+0x44bbf0c>
     97c:	1a 13 01 0b 	*unknown*
     980:	0b 3a 0b 3b 	*unknown*
     984:	05 01 13 00 	l.jal 4045584 <_end+0x4040b18>
     988:	00 1b 17 01 	l.j 6c658c <_end+0x6c1b20>
     98c:	0b 0b 3a 0b 	*unknown*
     990:	3b 05 01 13 	*unknown*
     994:	00 00 1c 0d 	l.j 79c8 <_end+0x2f5c>
     998:	00 03 0e 3a 	l.j c4280 <_end+0xbf814>
     99c:	0b 3b 05 49 	*unknown*
     9a0:	13 00 00 1d 	l.bf fc000a14 <_end+0xfbffbfa8>
     9a4:	15 01 27 19 	*unknown*
     9a8:	01 13 00 00 	l.j 44c09a8 <_end+0x44bbf3c>
     9ac:	1e 2e 01 3f 	*unknown*
     9b0:	19 03 0e 3a 	*unknown*
     9b4:	0b 3b 0b 27 	*unknown*
     9b8:	19 49 13 11 	*unknown*
     9bc:	01 12 06 40 	l.j 44822bc <_end+0x447d850>
     9c0:	18 97 42 19 	*unknown*
     9c4:	01 13 00 00 	l.j 44c09c4 <_end+0x44bbf58>
     9c8:	1f 05 00 03 	*unknown*
     9cc:	0e 3a 0b 3b 	l.bnf f8e836b8 <_end+0xf8e7ec4c>
     9d0:	0b 49 13 02 	*unknown*
     9d4:	17 00 00 20 	*unknown*
     9d8:	05 00 03 08 	l.jal 40015f8 <_end+0x3ffcb8c>
     9dc:	3a 0b 3b 0b 	*unknown*
     9e0:	49 13 02 17 	*unknown*
     9e4:	00 00 21 34 	l.j 8eb4 <_end+0x4448>
     9e8:	00 03 08 3a 	l.j c2ad0 <_end+0xbe064>
     9ec:	0b 3b 0b 49 	*unknown*
     9f0:	13 02 17 00 	l.bf fc0865f0 <_end+0xfc081b84>
     9f4:	00 22 89 82 	l.j 8a2ffc <_end+0x89e590>
     9f8:	01 00 11 01 	l.j 4004dfc <_end+0x4000390>
     9fc:	31 13 00 00 	*unknown*
     a00:	23 89 82 01 	*unknown*
     a04:	01 11 01 31 	l.j 4440ec8 <_end+0x443c45c>
     a08:	13 01 13 00 	l.bf fc045608 <_end+0xfc040b9c>
     a0c:	00 24 8a 82 	l.j 923414 <_end+0x91e9a8>
     a10:	01 00 02 18 	l.j 4001270 <_end+0x3ffc804>
     a14:	91 42 18 00 	l.lbs r10,6144(r2)
     a18:	00 25 26 00 	l.j 94a218 <_end+0x9457ac>
     a1c:	00 00 26 2e 	l.j a2d4 <_end+0x5868>
     a20:	01 3f 19 03 	l.j 4fc6e2c <_end+0x4fc23c0>
     a24:	0e 3a 0b 3b 	l.bnf f8e83710 <_end+0xf8e7eca4>
     a28:	0b 27 19 11 	*unknown*
     a2c:	01 12 06 40 	l.j 448232c <_end+0x447d8c0>
     a30:	18 97 42 19 	*unknown*
     a34:	01 13 00 00 	l.j 44c0a34 <_end+0x44bbfc8>
     a38:	27 05 00 03 	*unknown*
     a3c:	0e 3a 0b 3b 	l.bnf f8e83728 <_end+0xf8e7ecbc>
     a40:	0b 49 13 02 	*unknown*
     a44:	18 00 00 28 	l.movhi r0,0x28
     a48:	05 00 03 08 	l.jal 4001668 <_end+0x3ffcbfc>
     a4c:	3a 0b 3b 0b 	*unknown*
     a50:	49 13 02 18 	*unknown*
     a54:	00 00 29 34 	l.j af24 <_end+0x64b8>
     a58:	00 03 0e 3a 	l.j c4340 <_end+0xbf8d4>
     a5c:	0b 3b 0b 49 	*unknown*
     a60:	13 3f 19 02 	l.bf fcfc6e68 <_end+0xfcfc23fc>
     a64:	18 00 00 2a 	l.movhi r0,0x2a
     a68:	2e 01 3f 19 	*unknown*
     a6c:	03 0e 3a 0b 	l.j fc38f298 <_end+0xfc38a82c>
     a70:	3b 0b 27 19 	*unknown*
     a74:	3c 19 01 13 	*unknown*
     a78:	00 00 2b 2e 	l.j b730 <_end+0x6cc4>
     a7c:	00 3f 19 03 	l.j fc6e88 <_end+0xfc241c>
     a80:	0e 3a 0b 3b 	l.bnf f8e8376c <_end+0xf8e7ed00>
     a84:	0b 27 19 3c 	*unknown*
     a88:	19 00 00 2c 	l.movhi r8,0x2c
     a8c:	2e 00 3f 19 	*unknown*
     a90:	03 0e 3a 0b 	l.j fc38f2bc <_end+0xfc38a850>
     a94:	3b 0b 27 19 	*unknown*
     a98:	49 13 3c 19 	*unknown*
     a9c:	00 00 00 01 	l.j aa0 <_or1k_reset+0x9a0>
     aa0:	11 01 25 0e 	l.bf 4049ed8 <_end+0x404546c>
     aa4:	13 0b 03 0e 	l.bf fc2c16dc <_end+0xfc2bcc70>
     aa8:	1b 0e 11 01 	*unknown*
     aac:	12 06 10 17 	l.bf f8184b08 <_end+0xf818009c>
     ab0:	00 00 02 24 	l.j 1340 <_or1k_reset+0x1240>
     ab4:	00 0b 0b 3e 	l.j 2c37ac <_end+0x2bed40>
     ab8:	0b 03 0e 00 	*unknown*
     abc:	00 03 16 00 	l.j c62bc <_end+0xc1850>
     ac0:	03 0e 3a 0b 	l.j fc38f2ec <_end+0xfc38a880>
     ac4:	3b 0b 49 13 	*unknown*
     ac8:	00 00 04 24 	l.j 1b58 <_or1k_reset+0x1a58>
     acc:	00 0b 0b 3e 	l.j 2c37c4 <_end+0x2bed58>
     ad0:	0b 03 08 00 	*unknown*
     ad4:	00 05 0f 00 	l.j 1446d4 <_end+0x13fc68>
     ad8:	0b 0b 49 13 	*unknown*
     adc:	00 00 06 15 	l.j 2330 <frame_dummy+0x34>
     ae0:	01 27 19 01 	l.j 49c6ee4 <_end+0x49c2478>
     ae4:	13 00 00 07 	l.bf fc000b00 <_end+0xfbffc094>
     ae8:	05 00 49 13 	l.jal 4012f34 <_end+0x400e4c8>
     aec:	00 00 08 0f 	l.j 2b28 <_or1k_uart_init+0xd4>
     af0:	00 0b 0b 00 	l.j 2c36f0 <_end+0x2bec84>
     af4:	00 09 2e 01 	l.j 24c2f8 <_end+0x24788c>
     af8:	3f 19 03 0e 	*unknown*
     afc:	3a 0b 3b 0b 	*unknown*
     b00:	27 19 11 01 	*unknown*
     b04:	12 06 40 18 	l.bf f8190b64 <_end+0xf818c0f8>
     b08:	97 42 19 01 	l.lhz r26,6401(r2)
     b0c:	13 00 00 0a 	l.bf fc000b34 <_end+0xfbffc0c8>
     b10:	05 00 03 0e 	l.jal 4001748 <_end+0x3ffccdc>
     b14:	3a 0b 3b 0b 	*unknown*
     b18:	49 13 02 17 	*unknown*
     b1c:	00 00 0b 34 	l.j 37ec <memset+0xa8>
     b20:	00 03 08 3a 	l.j c2c08 <_end+0xbe19c>
     b24:	0b 3b 0b 49 	*unknown*
     b28:	13 00 00 0c 	l.bf fc000b58 <_end+0xfbffc0ec>
     b2c:	2e 01 3f 19 	*unknown*
     b30:	03 0e 3a 0b 	l.j fc38f35c <_end+0xfc38a8f0>
     b34:	3b 0b 27 19 	*unknown*
     b38:	49 13 11 01 	*unknown*
     b3c:	12 06 40 18 	l.bf f8190b9c <_end+0xf818c130>
     b40:	96 42 19 01 	l.lhz r18,6401(r2)
     b44:	13 00 00 0d 	l.bf fc000b78 <_end+0xfbffc10c>
     b48:	34 00 03 0e 	*unknown*
     b4c:	3a 0b 3b 0b 	*unknown*
     b50:	49 13 02 17 	*unknown*
     b54:	00 00 0e 05 	l.j 4368 <impure_data+0x314>
     b58:	00 03 08 3a 	l.j c2c40 <_end+0xbe1d4>
     b5c:	0b 3b 0b 49 	*unknown*
     b60:	13 02 17 00 	l.bf fc086760 <_end+0xfc081cf4>
     b64:	00 0f 89 82 	l.j 3e316c <_end+0x3de700>
     b68:	01 01 11 01 	l.j 4044f6c <_end+0x4040500>
     b6c:	31 13 01 13 	*unknown*
     b70:	00 00 10 8a 	l.j 4d98 <_end+0x32c>
     b74:	82 01 00 02 	*unknown*
     b78:	18 91 42 18 	*unknown*
     b7c:	00 00 11 89 	l.j 51a0 <_end+0x734>
     b80:	82 01 00 11 	*unknown*
     b84:	01 31 13 00 	l.j 4c45784 <_end+0x4c40d18>
     b88:	00 12 34 00 	l.j 48db88 <_end+0x48911c>
     b8c:	03 0e 3a 0b 	l.j fc38f3b8 <_end+0xfc38a94c>
     b90:	3b 0b 49 13 	*unknown*
     b94:	3f 19 3c 19 	*unknown*
     b98:	00 00 13 34 	l.j 5868 <_end+0xdfc>
     b9c:	00 03 0e 3a 	l.j c4484 <_end+0xbfa18>
     ba0:	0b 3b 0b 49 	*unknown*
     ba4:	13 3f 19 02 	l.bf fcfc6fac <_end+0xfcfc2540>
     ba8:	18 00 00 14 	l.movhi r0,0x14
     bac:	2e 01 3f 19 	*unknown*
     bb0:	03 0e 3a 0b 	l.j fc38f3dc <_end+0xfc38a970>
     bb4:	3b 0b 27 19 	*unknown*
     bb8:	3c 19 01 13 	*unknown*
     bbc:	00 00 15 2e 	l.j 6074 <_end+0x1608>
     bc0:	01 3f 19 03 	l.j 4fc6fcc <_end+0x4fc2560>
     bc4:	0e 3a 0b 3b 	l.bnf f8e838b0 <_end+0xf8e7ee44>
     bc8:	0b 27 19 3c 	*unknown*
     bcc:	19 00 00 00 	l.movhi r8,0x0
     bd0:	01 11 01 25 	l.j 4441064 <_end+0x443c5f8>
     bd4:	0e 13 0b 03 	l.bnf f84c37e0 <_end+0xf84bed74>
     bd8:	0e 1b 0e 11 	l.bnf f86c441c <_end+0xf86bf9b0>
     bdc:	01 12 06 10 	l.j 448241c <_end+0x447d9b0>
     be0:	17 00 00 02 	*unknown*
     be4:	24 00 0b 0b 	*unknown*
     be8:	3e 0b 03 0e 	*unknown*
     bec:	00 00 03 16 	l.j 1844 <_or1k_reset+0x1744>
     bf0:	00 03 0e 3a 	l.j c44d8 <_end+0xbfa6c>
     bf4:	0b 3b 0b 49 	*unknown*
     bf8:	13 00 00 04 	l.bf fc000c08 <_end+0xfbffc19c>
     bfc:	24 00 0b 0b 	*unknown*
     c00:	3e 0b 03 08 	*unknown*
     c04:	00 00 05 0f 	l.j 2040 <_or1k_start+0x10>
     c08:	00 0b 0b 49 	l.j 2c392c <_end+0x2beec0>
     c0c:	13 00 00 06 	l.bf fc000c24 <_end+0xfbffc1b8>
     c10:	15 01 27 19 	*unknown*
     c14:	01 13 00 00 	l.j 44c0c14 <_end+0x44bc1a8>
     c18:	07 05 00 49 	l.jal fc140d3c <_end+0xfc13c2d0>
     c1c:	13 00 00 08 	l.bf fc000c3c <_end+0xfbffc1d0>
     c20:	0f 00 0b 0b 	l.bnf fc00384c <_end+0xfbffede0>
     c24:	00 00 09 01 	l.j 3028 <_or1k_interrupt_handler+0xc>
     c28:	01 49 13 01 	l.j 524582c <_end+0x5240dc0>
     c2c:	13 00 00 0a 	l.bf fc000c54 <_end+0xfbffc1e8>
     c30:	21 00 49 13 	l.trap 0x4913
     c34:	2f 0b 00 00 	*unknown*
     c38:	0b 2e 01 03 	*unknown*
     c3c:	0e 3a 0b 3b 	l.bnf f8e83928 <_end+0xf8e7eebc>
     c40:	05 27 19 49 	l.jal 49c7164 <_end+0x49c26f8>
     c44:	13 20 0b 01 	l.bf fc803848 <_end+0xfc7feddc>
     c48:	13 00 00 0c 	l.bf fc000c78 <_end+0xfbffc20c>
     c4c:	05 00 03 08 	l.jal 400186c <_end+0x3ffce00>
     c50:	3a 0b 3b 05 	*unknown*
     c54:	49 13 00 00 	*unknown*
     c58:	0d 34 00 03 	l.bnf 4d00c64 <_end+0x4cfc1f8>
     c5c:	0e 3a 0b 3b 	l.bnf f8e83948 <_end+0xf8e7eedc>
     c60:	05 49 13 00 	l.jal 5245860 <_end+0x5240df4>
     c64:	00 0e 2e 01 	l.j 38c468 <_end+0x3879fc>
     c68:	03 0e 3a 0b 	l.j fc38f494 <_end+0xfc38aa28>
     c6c:	3b 0b 27 19 	*unknown*
     c70:	20 0b 01 13 	*unknown*
     c74:	00 00 0f 05 	l.j 4888 <_or1k_exception_impure_data+0x408>
     c78:	00 03 08 3a 	l.j c2d60 <_end+0xbe2f4>
     c7c:	0b 3b 0b 49 	*unknown*
     c80:	13 00 00 10 	l.bf fc000cc0 <_end+0xfbffc254>
     c84:	05 00 03 0e 	l.jal 40018bc <_end+0x3ffce50>
     c88:	3a 0b 3b 0b 	*unknown*
     c8c:	49 13 00 00 	*unknown*
     c90:	11 2e 01 3f 	l.bf 4b8118c <_end+0x4b7c720>
     c94:	19 03 0e 3a 	*unknown*
     c98:	0b 3b 0b 27 	*unknown*
     c9c:	19 11 01 12 	*unknown*
     ca0:	06 40 18 97 	l.jal f9006efc <_end+0xf9002490>
     ca4:	42 19 01 13 	*unknown*
     ca8:	00 00 12 05 	l.j 54bc <_end+0xa50>
     cac:	00 03 08 3a 	l.j c2d94 <_end+0xbe328>
     cb0:	0b 3b 0b 49 	*unknown*
     cb4:	13 02 17 00 	l.bf fc0868b4 <_end+0xfc081e48>
     cb8:	00 13 05 00 	l.j 4c20b8 <_end+0x4bd64c>
     cbc:	03 0e 3a 0b 	l.j fc38f4e8 <_end+0xfc38aa7c>
     cc0:	3b 0b 49 13 	*unknown*
     cc4:	02 18 00 00 	l.j f8600cc4 <_end+0xf85fc258>
     cc8:	14 34 00 03 	*unknown*
     ccc:	08 3a 0b 3b 	*unknown*
     cd0:	0b 49 13 02 	*unknown*
     cd4:	17 00 00 15 	*unknown*
     cd8:	1d 01 31 13 	*unknown*
     cdc:	52 01 55 17 	*unknown*
     ce0:	58 0b 59 0b 	*unknown*
     ce4:	01 13 00 00 	l.j 44c0ce4 <_end+0x44bc278>
     ce8:	16 05 00 31 	*unknown*
     cec:	13 1c 0b 00 	l.bf fc7038ec <_end+0xfc6fee80>
     cf0:	00 17 0b 01 	l.j 5c38f4 <_end+0x5bee88>
     cf4:	55 17 00 00 	*unknown*
     cf8:	18 34 00 31 	*unknown*
     cfc:	13 02 17 00 	l.bf fc0868fc <_end+0xfc081e90>
     d00:	00 19 1d 01 	l.j 648104 <_end+0x643698>
     d04:	31 13 11 01 	*unknown*
     d08:	12 06 58 0b 	l.bf f8196d34 <_end+0xf81922c8>
     d0c:	59 0b 00 00 	*unknown*
     d10:	1a 05 00 31 	*unknown*
     d14:	13 02 17 00 	l.bf fc086914 <_end+0xfc081ea8>
     d18:	00 1b 2e 01 	l.j 6cc51c <_end+0x6c7ab0>
     d1c:	3f 19 03 0e 	*unknown*
     d20:	3a 0b 3b 0b 	*unknown*
     d24:	27 19 49 13 	*unknown*
     d28:	11 01 12 06 	l.bf 4045540 <_end+0x4040ad4>
     d2c:	40 18 97 42 	*unknown*
     d30:	19 01 13 00 	*unknown*
     d34:	00 1c 34 00 	l.j 70dd34 <_end+0x7092c8>
     d38:	03 0e 3a 0b 	l.j fc38f564 <_end+0xfc38aaf8>
     d3c:	3b 0b 49 13 	*unknown*
     d40:	00 00 1d 34 	l.j 8210 <_end+0x37a4>
     d44:	00 03 0e 3a 	l.j c462c <_end+0xbfbc0>
     d48:	0b 3b 0b 49 	*unknown*
     d4c:	13 02 17 00 	l.bf fc08694c <_end+0xfc081ee0>
     d50:	00 1e 05 00 	l.j 782150 <_end+0x77d6e4>
     d54:	03 0e 3a 0b 	l.j fc38f580 <_end+0xfc38ab14>
     d58:	3b 0b 49 13 	*unknown*
     d5c:	02 17 00 00 	l.j f85c0d5c <_end+0xf85bc2f0>
     d60:	1f 34 00 03 	*unknown*
     d64:	0e 3a 0b 3b 	l.bnf f8e83a50 <_end+0xf8e7efe4>
     d68:	0b 49 13 3f 	*unknown*
     d6c:	19 02 18 00 	*unknown*
     d70:	00 00 01 11 	l.j 11b4 <_or1k_reset+0x10b4>
     d74:	01 25 0e 13 	l.j 49445c0 <_end+0x493fb54>
     d78:	0b 03 0e 1b 	*unknown*
     d7c:	0e 11 01 12 	l.bnf f84411c4 <_end+0xf843c758>
     d80:	06 10 17 00 	l.jal f8406980 <_end+0xf8401f14>
     d84:	00 02 24 00 	l.j 89d84 <_end+0x85318>
     d88:	0b 0b 3e 0b 	*unknown*
     d8c:	03 0e 00 00 	l.j fc380d8c <_end+0xfc37c320>
     d90:	03 24 00 0b 	l.j fc900dbc <_end+0xfc8fc350>
     d94:	0b 3e 0b 03 	*unknown*
     d98:	08 00 00 04 	*unknown*
     d9c:	16 00 03 0e 	*unknown*
     da0:	3a 0b 3b 0b 	*unknown*
     da4:	49 13 00 00 	*unknown*
     da8:	05 16 00 03 	l.jal 4580db4 <_end+0x457c348>
     dac:	0e 3a 0b 3b 	l.bnf f8e83a98 <_end+0xf8e7f02c>
     db0:	05 49 13 00 	l.jal 52459b0 <_end+0x5240f44>
     db4:	00 06 17 01 	l.j 1869b8 <_end+0x181f4c>
     db8:	0b 0b 3a 0b 	*unknown*
     dbc:	3b 0b 01 13 	*unknown*
     dc0:	00 00 07 0d 	l.j 29f4 <_stat_r+0x8>
     dc4:	00 03 0e 3a 	l.j c46ac <_end+0xbfc40>
     dc8:	0b 3b 0b 49 	*unknown*
     dcc:	13 00 00 08 	l.bf fc000dec <_end+0xfbffc380>
     dd0:	01 01 49 13 	l.j 405321c <_end+0x404e7b0>
     dd4:	01 13 00 00 	l.j 44c0dd4 <_end+0x44bc368>
     dd8:	09 21 00 49 	*unknown*
     ddc:	13 2f 0b 00 	l.bf fcbc39dc <_end+0xfcbbef70>
     de0:	00 0a 13 01 	l.j 2859e4 <_end+0x280f78>
     de4:	0b 0b 3a 0b 	*unknown*
     de8:	3b 0b 01 13 	*unknown*
     dec:	00 00 0b 0d 	l.j 3a20 <_fini+0x8>
     df0:	00 03 0e 3a 	l.j c46d8 <_end+0xbfc6c>
     df4:	0b 3b 0b 49 	*unknown*
     df8:	13 38 0b 00 	l.bf fce039f8 <_end+0xfcdfef8c>
     dfc:	00 0c 0f 00 	l.j 3049fc <_end+0x2fff90>
     e00:	0b 0b 00 00 	*unknown*
     e04:	0d 13 01 03 	l.bnf 44c1210 <_end+0x44bc7a4>
     e08:	0e 0b 0b 3a 	l.bnf f82c3af0 <_end+0xf82bf084>
     e0c:	0b 3b 0b 01 	*unknown*
     e10:	13 00 00 0e 	l.bf fc000e48 <_end+0xfbffc3dc>
     e14:	0d 00 03 08 	l.bnf 4001a34 <_end+0x3ffcfc8>
     e18:	3a 0b 3b 0b 	*unknown*
     e1c:	49 13 38 0b 	*unknown*
     e20:	00 00 0f 0f 	l.j 4a5c <_or1k_exception_handler_table+0x68>
     e24:	00 0b 0b 49 	l.j 2c3b48 <_end+0x2bf0dc>
     e28:	13 00 00 10 	l.bf fc000e68 <_end+0xfbffc3fc>
     e2c:	13 01 03 0e 	l.bf fc041a64 <_end+0xfc03cff8>
     e30:	0b 05 3a 0b 	*unknown*
     e34:	3b 0b 01 13 	*unknown*
     e38:	00 00 11 0d 	l.j 526c <_end+0x800>
     e3c:	00 03 0e 3a 	l.j c4724 <_end+0xbfcb8>
     e40:	0b 3b 0b 49 	*unknown*
     e44:	13 38 05 00 	l.bf fce02244 <_end+0xfcdfd7d8>
     e48:	00 12 15 00 	l.j 486248 <_end+0x4817dc>
     e4c:	27 19 00 00 	*unknown*
     e50:	13 15 01 27 	l.bf fc5412ec <_end+0xfc53c880>
     e54:	19 49 13 01 	*unknown*
     e58:	13 00 00 14 	l.bf fc000ea8 <_end+0xfbffc43c>
     e5c:	05 00 49 13 	l.jal 40132a8 <_end+0x400e83c>
     e60:	00 00 15 13 	l.j 62ac <_end+0x1840>
     e64:	01 03 0e 0b 	l.j 40c4690 <_end+0x40bfc24>
     e68:	05 3a 0b 3b 	l.jal 4e83b54 <_end+0x4e7f0e8>
     e6c:	05 01 13 00 	l.jal 4045a6c <_end+0x4041000>
     e70:	00 16 0d 00 	l.j 584270 <_end+0x57f804>
     e74:	03 0e 3a 0b 	l.j fc38f6a0 <_end+0xfc38ac34>
     e78:	3b 05 49 13 	*unknown*
     e7c:	38 0b 00 00 	*unknown*
     e80:	17 0d 00 03 	*unknown*
     e84:	0e 3a 0b 3b 	l.bnf f8e83b70 <_end+0xf8e7f104>
     e88:	05 49 13 38 	l.jal 5245b68 <_end+0x52410fc>
     e8c:	05 00 00 18 	l.jal 4000eec <_end+0x3ffc480>
     e90:	26 00 49 13 	*unknown*
     e94:	00 00 19 13 	l.j 72e0 <_end+0x2874>
     e98:	01 03 0e 0b 	l.j 40c46c4 <_end+0x40bfc58>
     e9c:	0b 3a 0b 3b 	*unknown*
     ea0:	05 01 13 00 	l.jal 4045aa0 <_end+0x4041034>
     ea4:	00 1a 13 01 	l.j 685aa8 <_end+0x68103c>
     ea8:	0b 0b 3a 0b 	*unknown*
     eac:	3b 05 01 13 	*unknown*
     eb0:	00 00 1b 17 	l.j 7b0c <_end+0x30a0>
     eb4:	01 0b 0b 3a 	l.j 42c3b9c <_end+0x42bf130>
     eb8:	0b 3b 05 01 	*unknown*
     ebc:	13 00 00 1c 	l.bf fc000f2c <_end+0xfbffc4c0>
     ec0:	0d 00 03 0e 	l.bnf 4001af8 <_end+0x3ffd08c>
     ec4:	3a 0b 3b 05 	*unknown*
     ec8:	49 13 00 00 	*unknown*
     ecc:	1d 15 01 27 	*unknown*
     ed0:	19 01 13 00 	*unknown*
     ed4:	00 1e 35 00 	l.j 78e2d4 <_end+0x789868>
     ed8:	49 13 00 00 	*unknown*
     edc:	1f 2e 01 3f 	*unknown*
     ee0:	19 03 0e 3a 	*unknown*
     ee4:	0b 3b 0b 27 	*unknown*
     ee8:	19 11 01 12 	*unknown*
     eec:	06 40 18 97 	l.jal f9007148 <_end+0xf90026dc>
     ef0:	42 19 01 13 	*unknown*
     ef4:	00 00 20 89 	l.j 9118 <_end+0x46ac>
     ef8:	82 01 01 11 	*unknown*
     efc:	01 31 13 01 	l.j 4c45b00 <_end+0x4c41094>
     f00:	13 00 00 21 	l.bf fc000f84 <_end+0xfbffc518>
     f04:	8a 82 01 00 	l.lws r20,256(r2)
     f08:	02 18 91 42 	l.j f8625410 <_end+0xf86209a4>
     f0c:	18 00 00 22 	l.movhi r0,0x22
     f10:	89 82 01 01 	l.lws r12,257(r2)
     f14:	11 01 31 13 	l.bf 404d360 <_end+0x40488f4>
     f18:	00 00 23 2e 	l.j 9bd0 <_end+0x5164>
     f1c:	00 3f 19 03 	l.j fc7328 <_end+0xfc28bc>
     f20:	0e 3a 0b 3b 	l.bnf f8e83c0c <_end+0xf8e7f1a0>
     f24:	0b 27 19 49 	*unknown*
     f28:	13 11 01 12 	l.bf fc441370 <_end+0xfc43c904>
     f2c:	06 40 18 97 	l.jal f9007188 <_end+0xf900271c>
     f30:	42 19 00 00 	*unknown*
     f34:	24 2e 00 3f 	*unknown*
     f38:	19 03 0e 3a 	*unknown*
     f3c:	0b 3b 0b 27 	*unknown*
     f40:	19 11 01 12 	*unknown*
     f44:	06 40 18 97 	l.jal f90071a0 <_end+0xf9002734>
     f48:	42 19 00 00 	*unknown*
     f4c:	25 34 00 03 	*unknown*
     f50:	0e 3a 0b 3b 	l.bnf f8e83c3c <_end+0xf8e7f1d0>
     f54:	0b 49 13 02 	*unknown*
     f58:	18 00 00 26 	l.movhi r0,0x26
     f5c:	34 00 03 0e 	*unknown*
     f60:	3a 0b 3b 05 	*unknown*
     f64:	49 13 3f 19 	*unknown*
     f68:	3c 19 00 00 	*unknown*
     f6c:	27 34 00 03 	*unknown*
     f70:	0e 3a 0b 3b 	l.bnf f8e83c5c <_end+0xf8e7f1f0>
     f74:	0b 49 13 3f 	*unknown*
     f78:	19 02 18 00 	*unknown*
     f7c:	00 28 2e 01 	l.j a0c780 <_end+0xa07d14>
     f80:	3f 19 03 0e 	*unknown*
     f84:	27 19 49 13 	*unknown*
     f88:	34 19 3c 19 	*unknown*
     f8c:	00 00 00 01 	l.j f90 <_or1k_reset+0xe90>
     f90:	11 01 25 0e 	l.bf 404a3c8 <_end+0x404595c>
     f94:	13 0b 03 0e 	l.bf fc2c1bcc <_end+0xfc2bd160>
     f98:	1b 0e 11 01 	*unknown*
     f9c:	12 06 10 17 	l.bf f8184ff8 <_end+0xf818058c>
     fa0:	00 00 02 24 	l.j 1830 <_or1k_reset+0x1730>
     fa4:	00 0b 0b 3e 	l.j 2c3c9c <_end+0x2bf230>
     fa8:	0b 03 0e 00 	*unknown*
     fac:	00 03 16 00 	l.j c67ac <_end+0xc1d40>
     fb0:	03 0e 3a 0b 	l.j fc38f7dc <_end+0xfc38ad70>
     fb4:	3b 0b 49 13 	*unknown*
     fb8:	00 00 04 24 	l.j 2048 <_or1k_start+0x18>
     fbc:	00 0b 0b 3e 	l.j 2c3cb4 <_end+0x2bf248>
     fc0:	0b 03 08 00 	*unknown*
     fc4:	00 05 0f 00 	l.j 144bc4 <_end+0x140158>
     fc8:	0b 0b 49 13 	*unknown*
     fcc:	00 00 06 15 	l.j 2820 <_exit+0x4>
     fd0:	00 27 19 00 	l.j 9c73d0 <_end+0x9c2964>
     fd4:	00 07 01 01 	l.j 1c13d8 <_end+0x1bc96c>
     fd8:	49 13 01 13 	*unknown*
     fdc:	00 00 08 21 	l.j 3060 <_or1k_interrupt_handler+0x44>
     fe0:	00 49 13 2f 	l.j 1245c9c <_end+0x1241230>
     fe4:	0b 00 00 09 	*unknown*
     fe8:	2e 01 3f 19 	*unknown*
     fec:	03 0e 3a 0b 	l.j fc38f818 <_end+0xfc38adac>
     ff0:	3b 0b 11 01 	*unknown*
     ff4:	12 06 40 18 	l.bf f8191054 <_end+0xf818c5e8>
     ff8:	97 42 19 01 	l.lhz r26,6401(r2)
     ffc:	13 00 00 0a 	l.bf fc001024 <_end+0xfbffc5b8>
    1000:	89 82 01 00 	l.lws r12,256(r2)
    1004:	11 01 31 13 	l.bf 404d450 <_end+0x40489e4>
    1008:	00 00 0b 2e 	l.j 3cc0 <_global_impure_ptr+0x28c>
    100c:	01 3f 19 03 	l.j 4fc7418 <_end+0x4fc29ac>
    1010:	0e 3a 0b 3b 	l.bnf f8e83cfc <_end+0xf8e7f290>
    1014:	0b 49 13 11 	*unknown*
    1018:	01 12 06 40 	l.j 4482918 <_end+0x447deac>
    101c:	18 97 42 19 	*unknown*
    1020:	01 13 00 00 	l.j 44c1020 <_end+0x44bc5b4>
    1024:	0c 34 00 03 	l.bnf d01030 <_end+0xcfc5c4>
    1028:	08 3a 0b 3b 	*unknown*
    102c:	0b 49 13 02 	*unknown*
    1030:	17 00 00 0d 	*unknown*
    1034:	2e 01 3f 19 	*unknown*
    1038:	03 0e 3a 0b 	l.j fc38f864 <_end+0xfc38adf8>
    103c:	3b 0b 27 19 	*unknown*
    1040:	11 01 12 06 	l.bf 4045858 <_end+0x4040dec>
    1044:	40 18 97 42 	*unknown*
    1048:	19 01 13 00 	*unknown*
    104c:	00 0e 05 00 	l.j 38244c <_end+0x37d9e0>
    1050:	03 0e 3a 0b 	l.j fc38f87c <_end+0xfc38ae10>
    1054:	3b 0b 49 13 	*unknown*
    1058:	02 17 00 00 	l.j f85c1058 <_end+0xf85bc5ec>
    105c:	0f 89 82 01 	l.bnf fe261860 <_end+0xfe25cdf4>
    1060:	01 11 01 31 	l.j 4441524 <_end+0x443cab8>
    1064:	13 01 13 00 	l.bf fc045c64 <_end+0xfc0411f8>
    1068:	00 10 8a 82 	l.j 423a70 <_end+0x41f004>
    106c:	01 00 02 18 	l.j 40018cc <_end+0x3ffce60>
    1070:	91 42 18 00 	l.lbs r10,6144(r2)
    1074:	00 11 89 82 	l.j 46367c <_end+0x45ec10>
    1078:	01 01 11 01 	l.j 404547c <_end+0x4040a10>
    107c:	31 13 00 00 	*unknown*
    1080:	12 34 00 03 	l.bf f8d0108c <_end+0xf8cfc620>
    1084:	0e 3a 0b 3b 	l.bnf f8e83d70 <_end+0xf8e7f304>
    1088:	0b 49 13 3f 	*unknown*
    108c:	19 02 18 00 	*unknown*
    1090:	00 13 34 00 	l.j 4ce090 <_end+0x4c9624>
    1094:	03 0e 3a 0b 	l.j fc38f8c0 <_end+0xfc38ae54>
    1098:	3b 0b 49 13 	*unknown*
    109c:	3f 19 3c 19 	*unknown*
    10a0:	00 00 14 2e 	l.j 6158 <_end+0x16ec>
    10a4:	01 3f 19 03 	l.j 4fc74b0 <_end+0x4fc2a44>
    10a8:	0e 3a 0b 3b 	l.bnf f8e83d94 <_end+0xf8e7f328>
    10ac:	0b 3c 19 01 	*unknown*
    10b0:	13 00 00 15 	l.bf fc001104 <_end+0xfbffc698>
    10b4:	18 00 00 00 	l.movhi r0,0x0
    10b8:	16 2e 00 3f 	*unknown*
    10bc:	19 03 0e 3a 	*unknown*
    10c0:	0b 3b 0b 27 	*unknown*
    10c4:	19 49 13 3c 	*unknown*
    10c8:	19 00 00 17 	l.movhi r8,0x17
    10cc:	2e 00 3f 19 	*unknown*
    10d0:	03 0e 3a 0b 	l.j fc38f8fc <_end+0xfc38ae90>
    10d4:	3b 05 27 19 	*unknown*
    10d8:	49 13 3c 19 	*unknown*
    10dc:	00 00 18 2e 	l.j 7194 <_end+0x2728>
    10e0:	01 3f 19 03 	l.j 4fc74ec <_end+0x4fc2a80>
    10e4:	0e 3a 0b 3b 	l.bnf f8e83dd0 <_end+0xf8e7f364>
    10e8:	05 27 19 3c 	l.jal 49c75d8 <_end+0x49c2b6c>
    10ec:	19 01 13 00 	*unknown*
    10f0:	00 19 05 00 	l.j 6424f0 <_end+0x63da84>
    10f4:	49 13 00 00 	*unknown*
    10f8:	1a 2e 01 3f 	*unknown*
    10fc:	19 03 0e 3a 	*unknown*
    1100:	0b 3b 0b 27 	*unknown*
    1104:	19 3c 19 00 	*unknown*
    1108:	00 00 01 11 	l.j 154c <_or1k_reset+0x144c>
    110c:	01 25 0e 13 	l.j 4944958 <_end+0x493feec>
    1110:	0b 03 0e 1b 	*unknown*
    1114:	0e 55 17 11 	l.bnf f9546d58 <_end+0xf95422ec>
    1118:	01 10 17 00 	l.j 4406d18 <_end+0x44022ac>
    111c:	00 02 24 00 	l.j 8a11c <_end+0x856b0>
    1120:	0b 0b 3e 0b 	*unknown*
    1124:	03 0e 00 00 	l.j fc381124 <_end+0xfc37c6b8>
    1128:	03 24 00 0b 	l.j fc901154 <_end+0xfc8fc6e8>
    112c:	0b 3e 0b 03 	*unknown*
    1130:	08 00 00 04 	*unknown*
    1134:	16 00 03 0e 	*unknown*
    1138:	3a 0b 3b 0b 	*unknown*
    113c:	49 13 00 00 	*unknown*
    1140:	05 0f 00 0b 	l.jal 43c116c <_end+0x43bc700>
    1144:	0b 49 13 00 	*unknown*
    1148:	00 06 15 00 	l.j 186548 <_end+0x181adc>
    114c:	27 19 00 00 	*unknown*
    1150:	07 01 01 49 	l.jal fc041674 <_end+0xfc03cc08>
    1154:	13 01 13 00 	l.bf fc045d54 <_end+0xfc0412e8>
    1158:	00 08 21 00 	l.j 209558 <_end+0x204aec>
    115c:	49 13 2f 0b 	*unknown*
    1160:	00 00 09 2e 	l.j 3618 <or1k_timer_restore+0x14>
    1164:	01 3f 19 03 	l.j 4fc7570 <_end+0x4fc2b04>
    1168:	0e 3a 0b 3b 	l.bnf f8e83e54 <_end+0xf8e7f3e8>
    116c:	0b 27 19 11 	*unknown*
    1170:	01 12 06 40 	l.j 4482a70 <_end+0x447e004>
    1174:	18 97 42 19 	*unknown*
    1178:	01 13 00 00 	l.j 44c1178 <_end+0x44bc70c>
    117c:	0a 05 00 03 	*unknown*
    1180:	08 3a 0b 3b 	*unknown*
    1184:	0b 49 13 02 	*unknown*
    1188:	17 00 00 0b 	*unknown*
    118c:	05 00 03 0e 	l.jal 4001dc4 <_end+0x3ffd358>
    1190:	3a 0b 3b 0b 	*unknown*
    1194:	49 13 02 18 	*unknown*
    1198:	00 00 0c 34 	l.j 4268 <impure_data+0x214>
    119c:	00 03 0e 3a 	l.j c4a84 <_end+0xc0018>
    11a0:	0b 3b 0b 49 	*unknown*
    11a4:	13 3f 19 02 	l.bf fcfc75ac <_end+0xfcfc2b40>
    11a8:	18 00 00 00 	l.movhi r0,0x0
    11ac:	01 11 01 25 	l.j 4441640 <_end+0x443cbd4>
    11b0:	0e 13 0b 03 	l.bnf f84c3dbc <_end+0xf84bf350>
    11b4:	0e 1b 0e 11 	l.bnf f86c49f8 <_end+0xf86bff8c>
    11b8:	01 12 06 10 	l.j 44829f8 <_end+0x447df8c>
    11bc:	17 00 00 02 	*unknown*
    11c0:	24 00 0b 0b 	*unknown*
    11c4:	3e 0b 03 0e 	*unknown*
    11c8:	00 00 03 16 	l.j 1e20 <_or1k_reset+0x1d20>
    11cc:	00 03 0e 3a 	l.j c4ab4 <_end+0xc0048>
    11d0:	0b 3b 0b 49 	*unknown*
    11d4:	13 00 00 04 	l.bf fc0011e4 <_end+0xfbffc778>
    11d8:	24 00 0b 0b 	*unknown*
    11dc:	3e 0b 03 08 	*unknown*
    11e0:	00 00 05 0f 	l.j 261c <__call_exitprocs+0x80>
    11e4:	00 0b 0b 49 	l.j 2c3f08 <_end+0x2bf49c>
    11e8:	13 00 00 06 	l.bf fc001200 <_end+0xfbffc794>
    11ec:	15 00 27 19 	l.nop 0x2719
    11f0:	00 00 07 13 	l.j 2e3c <_or1k_exception_handler+0x94>
    11f4:	01 03 0e 0b 	l.j 40c4a20 <_end+0x40bffb4>
    11f8:	0b 3a 0b 3b 	*unknown*
    11fc:	0b 01 13 00 	*unknown*
    1200:	00 08 0d 00 	l.j 204600 <_end+0x1ffb94>
    1204:	03 0e 3a 0b 	l.j fc38fa30 <_end+0xfc38afc4>
    1208:	3b 0b 49 13 	*unknown*
    120c:	38 0b 00 00 	*unknown*
    1210:	09 35 00 49 	*unknown*
    1214:	13 00 00 0a 	l.bf fc00123c <_end+0xfbffc7d0>
    1218:	2e 01 03 0e 	*unknown*
    121c:	3a 0b 3b 05 	*unknown*
    1220:	27 19 49 13 	*unknown*
    1224:	20 0b 01 13 	*unknown*
    1228:	00 00 0b 05 	l.j 3e3c <_global_impure_ptr+0x408>
    122c:	00 03 08 3a 	l.j c3314 <_end+0xbe8a8>
    1230:	0b 3b 05 49 	*unknown*
    1234:	13 00 00 0c 	l.bf fc001264 <_end+0xfbffc7f8>
    1238:	34 00 03 0e 	*unknown*
    123c:	3a 0b 3b 05 	*unknown*
    1240:	49 13 00 00 	*unknown*
    1244:	0d 2e 01 03 	l.bnf 4b81650 <_end+0x4b7cbe4>
    1248:	0e 3a 0b 3b 	l.bnf f8e83f34 <_end+0xf8e7f4c8>
    124c:	0b 27 19 20 	*unknown*
    1250:	0b 01 13 00 	*unknown*
    1254:	00 0e 05 00 	l.j 382654 <_end+0x37dbe8>
    1258:	03 08 3a 0b 	l.j fc20fa84 <_end+0xfc20b018>
    125c:	3b 0b 49 13 	*unknown*
    1260:	00 00 0f 05 	l.j 4e74 <_end+0x408>
    1264:	00 03 0e 3a 	l.j c4b4c <_end+0xc00e0>
    1268:	0b 3b 0b 49 	*unknown*
    126c:	13 00 00 10 	l.bf fc0012ac <_end+0xfbffc840>
    1270:	2e 01 3f 19 	*unknown*
    1274:	03 0e 3a 0b 	l.j fc38faa0 <_end+0xfc38b034>
    1278:	3b 0b 27 19 	*unknown*
    127c:	11 01 12 06 	l.bf 4045a94 <_end+0x4041028>
    1280:	40 18 97 42 	*unknown*
    1284:	19 01 13 00 	*unknown*
    1288:	00 11 34 00 	l.j 44e288 <_end+0x44981c>
    128c:	03 0e 3a 0b 	l.j fc38fab8 <_end+0xfc38b04c>
    1290:	3b 0b 49 13 	*unknown*
    1294:	02 17 00 00 	l.j f85c1294 <_end+0xf85bc828>
    1298:	12 1d 01 31 	l.bf f874175c <_end+0xf873ccf0>
    129c:	13 52 01 55 	l.bf fd4817f0 <_end+0xfd47cd84>
    12a0:	17 58 0b 59 	*unknown*
    12a4:	0b 01 13 00 	*unknown*
    12a8:	00 13 05 00 	l.j 4c26a8 <_end+0x4bdc3c>
    12ac:	31 13 1c 05 	*unknown*
    12b0:	00 00 14 0b 	l.j 62dc <_end+0x1870>
    12b4:	01 55 17 00 	l.j 5546eb4 <_end+0x5542448>
    12b8:	00 15 34 00 	l.j 54e2b8 <_end+0x54984c>
    12bc:	31 13 02 17 	*unknown*
    12c0:	00 00 16 1d 	l.j 6b34 <_end+0x20c8>
    12c4:	01 31 13 11 	l.j 4c45f08 <_end+0x4c4149c>
    12c8:	01 12 06 58 	l.j 4482c28 <_end+0x447e1bc>
    12cc:	0b 59 0b 00 	*unknown*
    12d0:	00 17 05 00 	l.j 5c26d0 <_end+0x5bdc64>
    12d4:	31 13 02 17 	*unknown*
    12d8:	00 00 18 2e 	l.j 7390 <_end+0x2924>
    12dc:	01 3f 19 03 	l.j 4fc76e8 <_end+0x4fc2c7c>
    12e0:	0e 3a 0b 3b 	l.bnf f8e83fcc <_end+0xf8e7f560>
    12e4:	0b 27 19 49 	*unknown*
    12e8:	13 11 01 12 	l.bf fc441730 <_end+0xfc43ccc4>
    12ec:	06 40 18 96 	l.jal f9007544 <_end+0xf9002ad8>
    12f0:	42 19 01 13 	*unknown*
    12f4:	00 00 19 05 	l.j 7708 <_end+0x2c9c>
    12f8:	00 03 08 3a 	l.j c33e0 <_end+0xbe974>
    12fc:	0b 3b 0b 49 	*unknown*
    1300:	13 02 17 00 	l.bf fc086f00 <_end+0xfc082494>
    1304:	00 1a 34 00 	l.j 68e304 <_end+0x689898>
    1308:	03 08 3a 0b 	l.j fc20fb34 <_end+0xfc20b0c8>
    130c:	3b 0b 49 13 	*unknown*
    1310:	00 00 1b 05 	l.j 7f24 <_end+0x34b8>
    1314:	00 31 13 1c 	l.j c45f84 <_end+0xc41518>
    1318:	0b 00 00 1c 	*unknown*
    131c:	89 82 01 01 	l.lws r12,257(r2)
    1320:	11 01 31 13 	l.bf 404d76c <_end+0x4048d00>
    1324:	00 00 1d 8a 	l.j 894c <_end+0x3ee0>
    1328:	82 01 00 02 	*unknown*
    132c:	18 91 42 18 	*unknown*
    1330:	00 00 1e 2e 	l.j 8be8 <_end+0x417c>
    1334:	01 3f 19 03 	l.j 4fc7740 <_end+0x4fc2cd4>
    1338:	0e 3a 0b 3b 	l.bnf f8e84024 <_end+0xf8e7f5b8>
    133c:	0b 27 19 11 	*unknown*
    1340:	01 12 06 40 	l.j 4482c40 <_end+0x447e1d4>
    1344:	18 96 42 19 	*unknown*
    1348:	01 13 00 00 	l.j 44c1348 <_end+0x44bc8dc>
    134c:	1f 34 00 03 	*unknown*
    1350:	0e 3a 0b 3b 	l.bnf f8e8403c <_end+0xf8e7f5d0>
    1354:	0b 49 13 02 	*unknown*
    1358:	18 00 00 20 	l.movhi r0,0x20
    135c:	05 00 03 0e 	l.jal 4001f94 <_end+0x3ffd528>
    1360:	3a 0b 3b 0b 	*unknown*
    1364:	49 13 02 17 	*unknown*
    1368:	00 00 21 34 	l.j 9838 <_end+0x4dcc>
    136c:	00 03 0e 3a 	l.j c4c54 <_end+0xc01e8>
    1370:	0b 3b 0b 49 	*unknown*
    1374:	13 00 00 22 	l.bf fc0013fc <_end+0xfbffc990>
    1378:	05 00 31 13 	l.jal 400d7c4 <_end+0x4008d58>
    137c:	00 00 23 34 	l.j a04c <_end+0x55e0>
    1380:	00 03 08 3a 	l.j c3468 <_end+0xbe9fc>
    1384:	0b 3b 0b 49 	*unknown*
    1388:	13 02 17 00 	l.bf fc086f88 <_end+0xfc08251c>
    138c:	00 24 1d 01 	l.j 908790 <_end+0x903d24>
    1390:	31 13 11 01 	*unknown*
    1394:	12 06 58 0b 	l.bf f81973c0 <_end+0xf8192954>
    1398:	59 0b 01 13 	*unknown*
    139c:	00 00 25 0b 	l.j a7c8 <_end+0x5d5c>
    13a0:	01 11 01 12 	l.j 44417e8 <_end+0x443cd7c>
    13a4:	06 00 00 26 	l.jal f800143c <_end+0xf7ffc9d0>
    13a8:	2e 01 3f 19 	*unknown*
    13ac:	03 0e 3a 0b 	l.j fc38fbd8 <_end+0xfc38b16c>
    13b0:	3b 0b 27 19 	*unknown*
    13b4:	49 13 11 01 	*unknown*
    13b8:	12 06 40 18 	l.bf f8191418 <_end+0xf818c9ac>
    13bc:	97 42 19 01 	l.lhz r26,6401(r2)
    13c0:	13 00 00 27 	l.bf fc00145c <_end+0xfbffc9f0>
    13c4:	2e 00 3f 19 	*unknown*
    13c8:	03 0e 3a 0b 	l.j fc38fbf4 <_end+0xfc38b188>
    13cc:	3b 0b 27 19 	*unknown*
    13d0:	49 13 11 01 	*unknown*
    13d4:	12 06 40 18 	l.bf f8191434 <_end+0xf818c9c8>
    13d8:	97 42 19 00 	l.lhz r26,6400(r2)
    13dc:	00 28 2e 00 	l.j a0cbdc <_end+0xa08170>
    13e0:	3f 19 03 0e 	*unknown*
    13e4:	3a 0b 3b 0b 	*unknown*
    13e8:	27 19 11 01 	*unknown*
    13ec:	12 06 40 18 	l.bf f819144c <_end+0xf818c9e0>
    13f0:	97 42 19 00 	l.lhz r26,6400(r2)
    13f4:	00 29 34 00 	l.j a4e3f4 <_end+0xa49988>
    13f8:	03 0e 3a 0b 	l.j fc38fc24 <_end+0xfc38b1b8>
    13fc:	3b 0b 49 13 	*unknown*
    1400:	3f 19 3c 19 	*unknown*
    1404:	00 00 2a 2e 	l.j bcbc <_end+0x7250>
    1408:	01 3f 19 03 	l.j 4fc7814 <_end+0x4fc2da8>
    140c:	0e 3a 0b 3b 	l.bnf f8e840f8 <_end+0xf8e7f68c>
    1410:	0b 27 19 3c 	*unknown*
    1414:	19 00 00 2b 	l.movhi r8,0x2b
    1418:	05 00 49 13 	l.jal 4013864 <_end+0x400edf8>
    141c:	00 00 00 01 	l.j 1420 <_or1k_reset+0x1320>
    1420:	11 01 25 0e 	l.bf 404a858 <_end+0x4045dec>
    1424:	13 0b 03 0e 	l.bf fc2c205c <_end+0xfc2bd5f0>
    1428:	1b 0e 11 01 	*unknown*
    142c:	12 06 10 17 	l.bf f8185488 <_end+0xf8180a1c>
    1430:	00 00 02 24 	l.j 1cc0 <_or1k_reset+0x1bc0>
    1434:	00 0b 0b 3e 	l.j 2c412c <_end+0x2bf6c0>
    1438:	0b 03 08 00 	*unknown*
    143c:	00 03 24 00 	l.j ca43c <_end+0xc59d0>
    1440:	0b 0b 3e 0b 	*unknown*
    1444:	03 0e 00 00 	l.j fc381444 <_end+0xfc37c9d8>
    1448:	04 16 00 03 	l.jal 581454 <_end+0x57c9e8>
    144c:	0e 3a 0b 3b 	l.bnf f8e84138 <_end+0xf8e7f6cc>
    1450:	0b 49 13 00 	*unknown*
    1454:	00 05 16 00 	l.j 146c54 <_end+0x1421e8>
    1458:	03 0e 3a 0b 	l.j fc38fc84 <_end+0xfc38b218>
    145c:	3b 05 49 13 	*unknown*
    1460:	00 00 06 17 	l.j 2cbc <_or1k_cache_init+0xb4>
    1464:	01 0b 0b 3a 	l.j 42c414c <_end+0x42bf6e0>
    1468:	0b 3b 0b 01 	*unknown*
    146c:	13 00 00 07 	l.bf fc001488 <_end+0xfbffca1c>
    1470:	0d 00 03 0e 	l.bnf 40020a8 <_end+0x3ffd63c>
    1474:	3a 0b 3b 0b 	*unknown*
    1478:	49 13 00 00 	*unknown*
    147c:	08 01 01 49 	*unknown*
    1480:	13 01 13 00 	l.bf fc046080 <_end+0xfc041614>
    1484:	00 09 21 00 	l.j 249884 <_end+0x244e18>
    1488:	49 13 2f 0b 	*unknown*
    148c:	00 00 0a 13 	l.j 3cd8 <_global_impure_ptr+0x2a4>
    1490:	01 0b 0b 3a 	l.j 42c4178 <_end+0x42bf70c>
    1494:	0b 3b 0b 01 	*unknown*
    1498:	13 00 00 0b 	l.bf fc0014c4 <_end+0xfbffca58>
    149c:	0d 00 03 0e 	l.bnf 40020d4 <_end+0x3ffd668>
    14a0:	3a 0b 3b 0b 	*unknown*
    14a4:	49 13 38 0b 	*unknown*
    14a8:	00 00 0c 0f 	l.j 44e4 <_or1k_exception_impure_data+0x64>
    14ac:	00 0b 0b 00 	l.j 2c40ac <_end+0x2bf640>
    14b0:	00 0d 13 01 	l.j 3460b4 <_end+0x341648>
    14b4:	03 0e 0b 0b 	l.j fc3840e0 <_end+0xfc37f674>
    14b8:	3a 0b 3b 0b 	*unknown*
    14bc:	01 13 00 00 	l.j 44c14bc <_end+0x44bca50>
    14c0:	0e 0d 00 03 	l.bnf f83414cc <_end+0xf833ca60>
    14c4:	08 3a 0b 3b 	*unknown*
    14c8:	0b 49 13 38 	*unknown*
    14cc:	0b 00 00 0f 	*unknown*
    14d0:	0f 00 0b 0b 	l.bnf fc0040fc <_end+0xfbfff690>
    14d4:	49 13 00 00 	*unknown*
    14d8:	10 13 01 03 	l.bf 4c18e4 <_end+0x4bce78>
    14dc:	0e 0b 05 3a 	l.bnf f82c29c4 <_end+0xf82bdf58>
    14e0:	0b 3b 0b 01 	*unknown*
    14e4:	13 00 00 11 	l.bf fc001528 <_end+0xfbffcabc>
    14e8:	0d 00 03 0e 	l.bnf 4002120 <_end+0x3ffd6b4>
    14ec:	3a 0b 3b 0b 	*unknown*
    14f0:	49 13 38 05 	*unknown*
    14f4:	00 00 12 15 	l.j 5d48 <_end+0x12dc>
    14f8:	00 27 19 00 	l.j 9c78f8 <_end+0x9c2e8c>
    14fc:	00 13 15 01 	l.j 4c6900 <_end+0x4c1e94>
    1500:	27 19 49 13 	*unknown*
    1504:	01 13 00 00 	l.j 44c1504 <_end+0x44bca98>
    1508:	14 05 00 49 	*unknown*
    150c:	13 00 00 15 	l.bf fc001560 <_end+0xfbffcaf4>
    1510:	13 01 03 0e 	l.bf fc042148 <_end+0xfc03d6dc>
    1514:	0b 05 3a 0b 	*unknown*
    1518:	3b 05 01 13 	*unknown*
    151c:	00 00 16 0d 	l.j 6d50 <_end+0x22e4>
    1520:	00 03 0e 3a 	l.j c4e08 <_end+0xc039c>
    1524:	0b 3b 05 49 	*unknown*
    1528:	13 38 0b 00 	l.bf fce04128 <_end+0xfcdff6bc>
    152c:	00 17 0d 00 	l.j 5c492c <_end+0x5bfec0>
    1530:	03 0e 3a 0b 	l.j fc38fd5c <_end+0xfc38b2f0>
    1534:	3b 05 49 13 	*unknown*
    1538:	38 05 00 00 	*unknown*
    153c:	18 26 00 49 	*unknown*
    1540:	13 00 00 19 	l.bf fc0015a4 <_end+0xfbffcb38>
    1544:	13 01 03 0e 	l.bf fc04217c <_end+0xfc03d710>
    1548:	0b 0b 3a 0b 	*unknown*
    154c:	3b 05 01 13 	*unknown*
    1550:	00 00 1a 13 	l.j 7d9c <_end+0x3330>
    1554:	01 0b 0b 3a 	l.j 42c423c <_end+0x42bf7d0>
    1558:	0b 3b 05 01 	*unknown*
    155c:	13 00 00 1b 	l.bf fc0015c8 <_end+0xfbffcb5c>
    1560:	17 01 0b 0b 	*unknown*
    1564:	3a 0b 3b 05 	*unknown*
    1568:	01 13 00 00 	l.j 44c1568 <_end+0x44bcafc>
    156c:	1c 0d 00 03 	*unknown*
    1570:	0e 3a 0b 3b 	l.bnf f8e8425c <_end+0xf8e7f7f0>
    1574:	05 49 13 00 	l.jal 5246174 <_end+0x5241708>
    1578:	00 1d 15 01 	l.j 74697c <_end+0x741f10>
    157c:	27 19 01 13 	*unknown*
    1580:	00 00 1e 2e 	l.j 8e38 <_end+0x43cc>
    1584:	01 3f 19 03 	l.j 4fc7990 <_end+0x4fc2f24>
    1588:	0e 3a 0b 3b 	l.bnf f8e84274 <_end+0xf8e7f808>
    158c:	0b 27 19 49 	*unknown*
    1590:	13 11 01 12 	l.bf fc4419d8 <_end+0xfc43cf6c>
    1594:	06 40 18 97 	l.jal f90077f0 <_end+0xf9002d84>
    1598:	42 19 01 13 	*unknown*
    159c:	00 00 1f 89 	l.j 93c0 <_end+0x4954>
    15a0:	82 01 00 11 	*unknown*
    15a4:	01 31 13 00 	l.j 4c461a4 <_end+0x4c41738>
    15a8:	00 20 2e 00 	l.j 80cda8 <_end+0x80833c>
    15ac:	3f 19 03 0e 	*unknown*
    15b0:	3a 0b 3b 05 	*unknown*
    15b4:	27 19 49 13 	*unknown*
    15b8:	3c 19 00 00 	*unknown*
    15bc:	00 01 11 01 	l.j 459c0 <_end+0x40f54>
    15c0:	25 0e 13 0b 	*unknown*
    15c4:	03 0e 1b 0e 	l.j fc3881fc <_end+0xfc383790>
    15c8:	11 01 12 06 	l.bf 4045de0 <_end+0x4041374>
    15cc:	10 17 00 00 	l.bf 5c15cc <_end+0x5bcb60>
    15d0:	02 24 00 0b 	l.j f89015fc <_end+0xf88fcb90>
    15d4:	0b 3e 0b 03 	*unknown*
    15d8:	0e 00 00 03 	l.bnf f80015e4 <_end+0xf7ffcb78>
    15dc:	16 00 03 0e 	*unknown*
    15e0:	3a 0b 3b 0b 	*unknown*
    15e4:	49 13 00 00 	*unknown*
    15e8:	04 24 00 0b 	l.jal 901614 <_end+0x8fcba8>
    15ec:	0b 3e 0b 03 	*unknown*
    15f0:	08 00 00 05 	*unknown*
    15f4:	0f 00 0b 0b 	l.bnf fc004220 <_end+0xfbfff7b4>
    15f8:	00 00 06 0f 	l.j 2e34 <_or1k_exception_handler+0x8c>
    15fc:	00 0b 0b 49 	l.j 2c4320 <_end+0x2bf8b4>
    1600:	13 00 00 07 	l.bf fc00161c <_end+0xfbffcbb0>
    1604:	2e 01 3f 19 	*unknown*
    1608:	03 0e 3a 0b 	l.j fc38fe34 <_end+0xfc38b3c8>
    160c:	3b 0b 27 19 	*unknown*
    1610:	49 13 11 01 	*unknown*
    1614:	12 06 40 18 	l.bf f8191674 <_end+0xf818cc08>
    1618:	97 42 19 01 	l.lhz r26,6401(r2)
    161c:	13 00 00 08 	l.bf fc00163c <_end+0xfbffcbd0>
    1620:	05 00 03 08 	l.jal 4002240 <_end+0x3ffd7d4>
    1624:	3a 0b 3b 0b 	*unknown*
    1628:	49 13 02 18 	*unknown*
    162c:	00 00 09 05 	l.j 3a40 <_global_impure_ptr+0xc>
    1630:	00 03 08 3a 	l.j c3718 <_end+0xbecac>
    1634:	0b 3b 0b 49 	*unknown*
    1638:	13 02 17 00 	l.bf fc087238 <_end+0xfc0827cc>
    163c:	00 0a 34 00 	l.j 28e63c <_end+0x289bd0>
    1640:	03 08 3a 0b 	l.j fc20fe6c <_end+0xfc20b400>
    1644:	3b 0b 49 13 	*unknown*
    1648:	02 17 00 00 	l.j f85c1648 <_end+0xf85bcbdc>
    164c:	0b 34 00 03 	*unknown*
    1650:	0e 3a 0b 3b 	l.bnf f8e8433c <_end+0xf8e7f8d0>
    1654:	0b 49 13 02 	*unknown*
    1658:	17 00 00 00 	*unknown*

Disassembly of section .debug_line:

00000000 <.debug_line>:
       0:	00 00 00 d2 	l.j 348 <_or1k_reset+0x248>
       4:	00 02 00 00 	l.j 80004 <_end+0x7b598>
       8:	00 b7 04 01 	l.j 2dc100c <_end+0x2dbc5a0>
       c:	fb 0e 0d 00 	*unknown*
      10:	01 01 01 01 	l.j 4040414 <_end+0x403b9a8>
      14:	00 00 00 01 	l.j 18 <_or1k_reset-0xe8>
      18:	00 00 01 2e 	l.j 4d0 <_or1k_reset+0x3d0>
      1c:	2e 2f 2e 2e 	*unknown*
      20:	2f 2e 2e 2f 	*unknown*
      24:	2e 2e 2f 2e 	*unknown*
      28:	2e 2f 6e 65 	*unknown*
      2c:	77 6c 69 62 	*unknown*
      30:	2d 32 2e 32 	*unknown*
      34:	2e 30 2e 32 	*unknown*
      38:	30 31 35 30 	*unknown*
      3c:	38 32 34 2f 	*unknown*
      40:	6e 65 77 6c 	l.lwa r19,30572(r5)
      44:	69 62 2f 6c 	*unknown*
      48:	69 62 63 2f 	*unknown*
      4c:	73 74 64 6c 	*unknown*
      50:	69 62 00 2f 	*unknown*
      54:	68 6f 6d 65 	*unknown*
      58:	2f 61 6e 64 	*unknown*
      5c:	72 7a 65 6a 	*unknown*
      60:	72 2f 44 69 	*unknown*
      64:	67 69 74 61 	*unknown*
      68:	6c 2f 4f 52 	l.lwa r1,20306(r15)
      6c:	50 53 6f 43 	*unknown*
      70:	2f 6e 65 77 	*unknown*
      74:	6c 69 62 2d 	l.lwa r3,25133(r9)
      78:	32 2e 32 2e 	*unknown*
      7c:	30 2e 32 30 	*unknown*
      80:	31 35 30 38 	*unknown*
      84:	32 34 2f 6e 	*unknown*
      88:	65 77 6c 69 	*unknown*
      8c:	62 2f 6c 69 	*unknown*
      90:	62 63 2f 69 	*unknown*
      94:	6e 63 6c 75 	l.lwa r19,27765(r3)
      98:	64 65 00 00 	*unknown*
      9c:	61 74 65 78 	*unknown*
      a0:	69 74 2e 63 	*unknown*
      a4:	00 01 00 00 	l.j 400a4 <_end+0x3b638>
      a8:	61 74 65 78 	*unknown*
      ac:	69 74 2e 68 	*unknown*
      b0:	00 01 00 00 	l.j 400b0 <_end+0x3b644>
      b4:	73 74 64 6c 	*unknown*
      b8:	69 62 2e 68 	*unknown*
      bc:	00 02 00 00 	l.j 800bc <_end+0x7b650>
      c0:	00 00 05 02 	l.j 14c8 <_or1k_reset+0x13c8>
      c4:	00 00 23 e4 	l.j 9054 <_end+0x45e8>
      c8:	03 3f 01 21 	l.j fcfc054c <_end+0xfcfbbae0>
      cc:	1f 2f 1f 21 	*unknown*
      d0:	2f 02 04 00 	*unknown*
      d4:	01 01 00 00 	l.j 40400d4 <_end+0x403b668>
      d8:	01 84 00 02 	l.j 61000e0 <_end+0x60fb674>
      dc:	00 00 01 67 	l.j 678 <_or1k_reset+0x578>
      e0:	04 01 fb 0e 	l.jal 7ed18 <_end+0x7a2ac>
      e4:	0d 00 01 01 	l.bnf 40004e8 <_end+0x3ffba7c>
      e8:	01 01 00 00 	l.j 40400e8 <_end+0x403b67c>
      ec:	00 01 00 00 	l.j 400ec <_end+0x3b680>
      f0:	01 2e 2e 2f 	l.j 4b8b9ac <_end+0x4b86f40>
      f4:	2e 2e 2f 2e 	*unknown*
      f8:	2e 2f 2e 2e 	*unknown*
      fc:	2f 2e 2e 2f 	*unknown*
     100:	6e 65 77 6c 	l.lwa r19,30572(r5)
     104:	69 62 2d 32 	*unknown*
     108:	2e 32 2e 30 	*unknown*
     10c:	2e 32 30 31 	*unknown*
     110:	35 30 38 32 	*unknown*
     114:	34 2f 6e 65 	*unknown*
     118:	77 6c 69 62 	*unknown*
     11c:	2f 6c 69 62 	*unknown*
     120:	63 2f 73 74 	*unknown*
     124:	64 6c 69 62 	*unknown*
     128:	00 2f 68 6f 	l.j bda2e4 <_end+0xbd5878>
     12c:	6d 65 2f 61 	l.lwa r11,12129(r5)
     130:	6e 64 72 7a 	l.lwa r19,29306(r4)
     134:	65 6a 72 2f 	*unknown*
     138:	44 69 67 69 	*unknown*
     13c:	74 61 6c 2f 	*unknown*
     140:	4f 52 50 53 	*unknown*
     144:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     148:	65 77 6c 69 	*unknown*
     14c:	62 2d 32 2e 	*unknown*
     150:	32 2e 30 2e 	*unknown*
     154:	32 30 31 35 	*unknown*
     158:	30 38 32 34 	*unknown*
     15c:	2f 6e 65 77 	*unknown*
     160:	6c 69 62 2f 	l.lwa r3,25135(r9)
     164:	6c 69 62 63 	l.lwa r3,25187(r9)
     168:	2f 69 6e 63 	*unknown*
     16c:	6c 75 64 65 	l.lwa r3,25701(r21)
     170:	2f 73 79 73 	*unknown*
     174:	00 2f 6f 70 	l.j bdbf34 <_end+0xbd74c8>
     178:	74 2f 6f 72 	*unknown*
     17c:	31 6b 2d 65 	*unknown*
     180:	6c 66 2f 6c 	l.lwa r3,12140(r6)
     184:	69 62 2f 67 	*unknown*
     188:	63 63 2f 6f 	*unknown*
     18c:	72 31 6b 2d 	*unknown*
     190:	65 6c 66 2f 	*unknown*
     194:	34 2e 39 2e 	*unknown*
     198:	32 2f 69 6e 	*unknown*
     19c:	63 6c 75 64 	*unknown*
     1a0:	65 00 2f 68 	*unknown*
     1a4:	6f 6d 65 2f 	l.lwa r27,25903(r13)
     1a8:	61 6e 64 72 	*unknown*
     1ac:	7a 65 6a 72 	*unknown*
     1b0:	2f 44 69 67 	*unknown*
     1b4:	69 74 61 6c 	*unknown*
     1b8:	2f 4f 52 50 	*unknown*
     1bc:	53 6f 43 2f 	*unknown*
     1c0:	6e 65 77 6c 	l.lwa r19,30572(r5)
     1c4:	69 62 2d 32 	*unknown*
     1c8:	2e 32 2e 30 	*unknown*
     1cc:	2e 32 30 31 	*unknown*
     1d0:	35 30 38 32 	*unknown*
     1d4:	34 2f 6e 65 	*unknown*
     1d8:	77 6c 69 62 	*unknown*
     1dc:	2f 6c 69 62 	*unknown*
     1e0:	63 2f 69 6e 	*unknown*
     1e4:	63 6c 75 64 	*unknown*
     1e8:	65 00 00 65 	*unknown*
     1ec:	78 69 74 2e 	*unknown*
     1f0:	63 00 01 00 	*unknown*
     1f4:	00 6c 6f 63 	l.j 1b1bf80 <_end+0x1b17514>
     1f8:	6b 2e 68 00 	*unknown*
     1fc:	02 00 00 5f 	l.j f8000378 <_end+0xf7ffb90c>
     200:	74 79 70 65 	*unknown*
     204:	73 2e 68 00 	*unknown*
     208:	02 00 00 73 	l.j f80003d4 <_end+0xf7ffb968>
     20c:	74 64 64 65 	*unknown*
     210:	66 2e 68 00 	*unknown*
     214:	03 00 00 72 	l.j fc0003dc <_end+0xfbffb970>
     218:	65 65 6e 74 	*unknown*
     21c:	2e 68 00 02 	*unknown*
     220:	00 00 73 74 	l.j 1cff0 <_end+0x18584>
     224:	64 6c 69 62 	*unknown*
     228:	2e 68 00 04 	*unknown*
     22c:	00 00 61 74 	l.j 187fc <_end+0x13d90>
     230:	65 78 69 74 	*unknown*
     234:	2e 68 00 01 	*unknown*
     238:	00 00 75 6e 	l.j 1d7f0 <_end+0x18d84>
     23c:	69 73 74 64 	*unknown*
     240:	2e 68 00 02 	*unknown*
     244:	00 00 00 00 	l.j 244 <_or1k_reset+0x144>
     248:	05 02 00 00 	l.jal 4080248 <_end+0x407b7dc>
     24c:	24 14 03 3b 	*unknown*
     250:	01 42 03 7a 	l.j 5081038 <_end+0x507c5cc>
     254:	20 26 30 75 	*unknown*
     258:	2f 02 02 00 	*unknown*
     25c:	01 01 00 00 	l.j 404025c <_end+0x403b7f0>
     260:	01 02 00 02 	l.j 4080268 <_end+0x407b7fc>
     264:	00 00 00 fc 	l.j 654 <_or1k_reset+0x554>
     268:	04 01 fb 0e 	l.jal 7eea0 <_end+0x7a434>
     26c:	0d 00 01 01 	l.bnf 4000670 <_end+0x3ffbc04>
     270:	01 01 00 00 	l.j 4040270 <_end+0x403b804>
     274:	00 01 00 00 	l.j 40274 <_end+0x3b808>
     278:	01 2f 68 6f 	l.j 4bda434 <_end+0x4bd59c8>
     27c:	6d 65 2f 61 	l.lwa r11,12129(r5)
     280:	6e 64 72 7a 	l.lwa r19,29306(r4)
     284:	65 6a 72 2f 	*unknown*
     288:	44 69 67 69 	*unknown*
     28c:	74 61 6c 2f 	*unknown*
     290:	4f 52 50 53 	*unknown*
     294:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     298:	65 77 6c 69 	*unknown*
     29c:	62 2d 32 2e 	*unknown*
     2a0:	32 2e 30 2e 	*unknown*
     2a4:	32 30 31 35 	*unknown*
     2a8:	30 38 32 34 	*unknown*
     2ac:	2f 6e 65 77 	*unknown*
     2b0:	6c 69 62 2f 	l.lwa r3,25135(r9)
     2b4:	6c 69 62 63 	l.lwa r3,25187(r9)
     2b8:	2f 69 6e 63 	*unknown*
     2bc:	6c 75 64 65 	l.lwa r3,25701(r21)
     2c0:	2f 73 79 73 	*unknown*
     2c4:	00 2f 6f 70 	l.j bdc084 <_end+0xbd7618>
     2c8:	74 2f 6f 72 	*unknown*
     2cc:	31 6b 2d 65 	*unknown*
     2d0:	6c 66 2f 6c 	l.lwa r3,12140(r6)
     2d4:	69 62 2f 67 	*unknown*
     2d8:	63 63 2f 6f 	*unknown*
     2dc:	72 31 6b 2d 	*unknown*
     2e0:	65 6c 66 2f 	*unknown*
     2e4:	34 2e 39 2e 	*unknown*
     2e8:	32 2f 69 6e 	*unknown*
     2ec:	63 6c 75 64 	*unknown*
     2f0:	65 00 2e 2e 	*unknown*
     2f4:	2f 2e 2e 2f 	*unknown*
     2f8:	2e 2e 2f 2e 	*unknown*
     2fc:	2e 2f 2e 2e 	*unknown*
     300:	2f 6e 65 77 	*unknown*
     304:	6c 69 62 2d 	l.lwa r3,25133(r9)
     308:	32 2e 32 2e 	*unknown*
     30c:	30 2e 32 30 	*unknown*
     310:	31 35 30 38 	*unknown*
     314:	32 34 2f 6e 	*unknown*
     318:	65 77 6c 69 	*unknown*
     31c:	62 2f 6c 69 	*unknown*
     320:	62 63 2f 72 	*unknown*
     324:	65 65 6e 74 	*unknown*
     328:	00 00 6c 6f 	l.j 1b4e4 <_end+0x16a78>
     32c:	63 6b 2e 68 	*unknown*
     330:	00 01 00 00 	l.j 40330 <_end+0x3b8c4>
     334:	5f 74 79 70 	*unknown*
     338:	65 73 2e 68 	*unknown*
     33c:	00 01 00 00 	l.j 4033c <_end+0x3b8d0>
     340:	73 74 64 64 	*unknown*
     344:	65 66 2e 68 	*unknown*
     348:	00 02 00 00 	l.j 80348 <_end+0x7b8dc>
     34c:	72 65 65 6e 	*unknown*
     350:	74 2e 68 00 	*unknown*
     354:	01 00 00 69 	l.j 40004f8 <_end+0x3ffba8c>
     358:	6d 70 75 72 	l.lwa r11,30066(r16)
     35c:	65 2e 63 00 	*unknown*
     360:	03 00 00 00 	l.j fc000360 <_end+0xfbffb8f4>
     364:	00 00 01 b9 	l.j a48 <_or1k_reset+0x948>
     368:	00 02 00 00 	l.j 80368 <_end+0x7b8fc>
     36c:	01 5f 04 01 	l.j 57c1370 <_end+0x57bc904>
     370:	fb 0e 0d 00 	*unknown*
     374:	01 01 01 01 	l.j 4040778 <_end+0x403bd0c>
     378:	00 00 00 01 	l.j 37c <_or1k_reset+0x27c>
     37c:	00 00 01 2e 	l.j 834 <_or1k_reset+0x734>
     380:	2e 2f 2e 2e 	*unknown*
     384:	2f 2e 2e 2f 	*unknown*
     388:	2e 2e 2f 2e 	*unknown*
     38c:	2e 2f 6e 65 	*unknown*
     390:	77 6c 69 62 	*unknown*
     394:	2d 32 2e 32 	*unknown*
     398:	2e 30 2e 32 	*unknown*
     39c:	30 31 35 30 	*unknown*
     3a0:	38 32 34 2f 	*unknown*
     3a4:	6e 65 77 6c 	l.lwa r19,30572(r5)
     3a8:	69 62 2f 6c 	*unknown*
     3ac:	69 62 63 2f 	*unknown*
     3b0:	73 74 64 6c 	*unknown*
     3b4:	69 62 00 2f 	*unknown*
     3b8:	6f 70 74 2f 	l.lwa r27,29743(r16)
     3bc:	6f 72 31 6b 	l.lwa r27,12651(r18)
     3c0:	2d 65 6c 66 	*unknown*
     3c4:	2f 6c 69 62 	*unknown*
     3c8:	2f 67 63 63 	*unknown*
     3cc:	2f 6f 72 31 	*unknown*
     3d0:	6b 2d 65 6c 	*unknown*
     3d4:	66 2f 34 2e 	*unknown*
     3d8:	39 2e 32 2f 	*unknown*
     3dc:	69 6e 63 6c 	*unknown*
     3e0:	75 64 65 00 	*unknown*
     3e4:	2f 68 6f 6d 	*unknown*
     3e8:	65 2f 61 6e 	*unknown*
     3ec:	64 72 7a 65 	*unknown*
     3f0:	6a 72 2f 44 	*unknown*
     3f4:	69 67 69 74 	*unknown*
     3f8:	61 6c 2f 4f 	*unknown*
     3fc:	52 50 53 6f 	*unknown*
     400:	43 2f 6e 65 	*unknown*
     404:	77 6c 69 62 	*unknown*
     408:	2d 32 2e 32 	*unknown*
     40c:	2e 30 2e 32 	*unknown*
     410:	30 31 35 30 	*unknown*
     414:	38 32 34 2f 	*unknown*
     418:	6e 65 77 6c 	l.lwa r19,30572(r5)
     41c:	69 62 2f 6c 	*unknown*
     420:	69 62 63 2f 	*unknown*
     424:	69 6e 63 6c 	*unknown*
     428:	75 64 65 2f 	*unknown*
     42c:	73 79 73 00 	*unknown*
     430:	2f 68 6f 6d 	*unknown*
     434:	65 2f 61 6e 	*unknown*
     438:	64 72 7a 65 	*unknown*
     43c:	6a 72 2f 44 	*unknown*
     440:	69 67 69 74 	*unknown*
     444:	61 6c 2f 4f 	*unknown*
     448:	52 50 53 6f 	*unknown*
     44c:	43 2f 6e 65 	*unknown*
     450:	77 6c 69 62 	*unknown*
     454:	2d 32 2e 32 	*unknown*
     458:	2e 30 2e 32 	*unknown*
     45c:	30 31 35 30 	*unknown*
     460:	38 32 34 2f 	*unknown*
     464:	6e 65 77 6c 	l.lwa r19,30572(r5)
     468:	69 62 2f 6c 	*unknown*
     46c:	69 62 63 2f 	*unknown*
     470:	69 6e 63 6c 	*unknown*
     474:	75 64 65 00 	*unknown*
     478:	00 5f 5f 61 	l.j 17d81fc <_end+0x17d3790>
     47c:	74 65 78 69 	*unknown*
     480:	74 2e 63 00 	*unknown*
     484:	01 00 00 73 	l.j 4000650 <_end+0x3ffbbe4>
     488:	74 64 64 65 	*unknown*
     48c:	66 2e 68 00 	*unknown*
     490:	02 00 00 6c 	l.j f8000640 <_end+0xf7ffbbd4>
     494:	6f 63 6b 2e 	l.lwa r27,27438(r3)
     498:	68 00 03 00 	*unknown*
     49c:	00 5f 74 79 	l.j 17dd680 <_end+0x17d8c14>
     4a0:	70 65 73 2e 	*unknown*
     4a4:	68 00 03 00 	*unknown*
     4a8:	00 72 65 65 	l.j 1c99a3c <_end+0x1c94fd0>
     4ac:	6e 74 2e 68 	l.lwa r19,11880(r20)
     4b0:	00 03 00 00 	l.j c04b0 <_end+0xbba44>
     4b4:	61 74 65 78 	*unknown*
     4b8:	69 74 2e 68 	*unknown*
     4bc:	00 01 00 00 	l.j 404bc <_end+0x3ba50>
     4c0:	73 74 64 6c 	*unknown*
     4c4:	69 62 2e 68 	*unknown*
     4c8:	00 04 00 00 	l.j 1004c8 <_end+0xfba5c>
     4cc:	00 00 05 02 	l.j 18d4 <_or1k_reset+0x17d4>
     4d0:	00 00 24 5c 	l.j 9640 <_end+0x4bd4>
     4d4:	03 c7 00 01 	l.j ff1c04d8 <_end+0xff1bba6c>
     4d8:	28 03 78 20 	*unknown*
     4dc:	28 03 78 20 	*unknown*
     4e0:	28 03 78 20 	*unknown*
     4e4:	28 03 78 20 	*unknown*
     4e8:	03 09 3c 03 	l.j fc24f4f4 <_end+0xfc24aa88>
     4ec:	77 20 20 03 	*unknown*
     4f0:	09 3c 30 03 	*unknown*
     4f4:	1e 4a 03 1d 	*unknown*
     4f8:	3c 40 1c 33 	*unknown*
     4fc:	03 47 90 4d 	l.j fd1e4630 <_end+0xfd1dfbc4>
     500:	2f 44 1f 21 	*unknown*
     504:	1f 22 22 21 	*unknown*
     508:	27 03 79 20 	*unknown*
     50c:	27 03 18 3c 	*unknown*
     510:	22 2c 1f 21 	*unknown*
     514:	3e 2f 03 46 	*unknown*
     518:	4a 03 09 3c 	*unknown*
     51c:	02 02 00 01 	l.j f8080520 <_end+0xf807bab4>
     520:	01 00 00 01 	l.j 4000524 <_end+0x3ffbab8>
     524:	be 00 02 00 	*unknown*
     528:	00 01 58 04 	l.j 56538 <_end+0x51acc>
     52c:	01 fb 0e 0d 	l.j 7ec3d60 <_end+0x7ebf2f4>
     530:	00 01 01 01 	l.j 40934 <_end+0x3bec8>
     534:	01 00 00 00 	l.j 4000534 <_end+0x3ffbac8>
     538:	01 00 00 01 	l.j 400053c <_end+0x3ffbad0>
     53c:	2e 2e 2f 2e 	*unknown*
     540:	2e 2f 2e 2e 	*unknown*
     544:	2f 2e 2e 2f 	*unknown*
     548:	2e 2e 2f 6e 	*unknown*
     54c:	65 77 6c 69 	*unknown*
     550:	62 2d 32 2e 	*unknown*
     554:	32 2e 30 2e 	*unknown*
     558:	32 30 31 35 	*unknown*
     55c:	30 38 32 34 	*unknown*
     560:	2f 6e 65 77 	*unknown*
     564:	6c 69 62 2f 	l.lwa r3,25135(r9)
     568:	6c 69 62 63 	l.lwa r3,25187(r9)
     56c:	2f 73 74 64 	*unknown*
     570:	6c 69 62 00 	l.lwa r3,25088(r9)
     574:	2f 68 6f 6d 	*unknown*
     578:	65 2f 61 6e 	*unknown*
     57c:	64 72 7a 65 	*unknown*
     580:	6a 72 2f 44 	*unknown*
     584:	69 67 69 74 	*unknown*
     588:	61 6c 2f 4f 	*unknown*
     58c:	52 50 53 6f 	*unknown*
     590:	43 2f 6e 65 	*unknown*
     594:	77 6c 69 62 	*unknown*
     598:	2d 32 2e 32 	*unknown*
     59c:	2e 30 2e 32 	*unknown*
     5a0:	30 31 35 30 	*unknown*
     5a4:	38 32 34 2f 	*unknown*
     5a8:	6e 65 77 6c 	l.lwa r19,30572(r5)
     5ac:	69 62 2f 6c 	*unknown*
     5b0:	69 62 63 2f 	*unknown*
     5b4:	69 6e 63 6c 	*unknown*
     5b8:	75 64 65 2f 	*unknown*
     5bc:	73 79 73 00 	*unknown*
     5c0:	2f 6f 70 74 	*unknown*
     5c4:	2f 6f 72 31 	*unknown*
     5c8:	6b 2d 65 6c 	*unknown*
     5cc:	66 2f 6c 69 	*unknown*
     5d0:	62 2f 67 63 	*unknown*
     5d4:	63 2f 6f 72 	*unknown*
     5d8:	31 6b 2d 65 	*unknown*
     5dc:	6c 66 2f 34 	l.lwa r3,12084(r6)
     5e0:	2e 39 2e 32 	*unknown*
     5e4:	2f 69 6e 63 	*unknown*
     5e8:	6c 75 64 65 	l.lwa r3,25701(r21)
     5ec:	00 2f 68 6f 	l.j bda7a8 <_end+0xbd5d3c>
     5f0:	6d 65 2f 61 	l.lwa r11,12129(r5)
     5f4:	6e 64 72 7a 	l.lwa r19,29306(r4)
     5f8:	65 6a 72 2f 	*unknown*
     5fc:	44 69 67 69 	*unknown*
     600:	74 61 6c 2f 	*unknown*
     604:	4f 52 50 53 	*unknown*
     608:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     60c:	65 77 6c 69 	*unknown*
     610:	62 2d 32 2e 	*unknown*
     614:	32 2e 30 2e 	*unknown*
     618:	32 30 31 35 	*unknown*
     61c:	30 38 32 34 	*unknown*
     620:	2f 6e 65 77 	*unknown*
     624:	6c 69 62 2f 	l.lwa r3,25135(r9)
     628:	6c 69 62 63 	l.lwa r3,25187(r9)
     62c:	2f 69 6e 63 	*unknown*
     630:	6c 75 64 65 	l.lwa r3,25701(r21)
     634:	00 00 5f 5f 	l.j 183b0 <_end+0x13944>
     638:	63 61 6c 6c 	*unknown*
     63c:	5f 61 74 65 	*unknown*
     640:	78 69 74 2e 	*unknown*
     644:	63 00 01 00 	*unknown*
     648:	00 6c 6f 63 	l.j 1b1c3d4 <_end+0x1b17968>
     64c:	6b 2e 68 00 	*unknown*
     650:	02 00 00 5f 	l.j f80007cc <_end+0xf7ffbd60>
     654:	74 79 70 65 	*unknown*
     658:	73 2e 68 00 	*unknown*
     65c:	02 00 00 73 	l.j f8000828 <_end+0xf7ffbdbc>
     660:	74 64 64 65 	*unknown*
     664:	66 2e 68 00 	*unknown*
     668:	03 00 00 72 	l.j fc000830 <_end+0xfbffbdc4>
     66c:	65 65 6e 74 	*unknown*
     670:	2e 68 00 02 	*unknown*
     674:	00 00 73 74 	l.j 1d444 <_end+0x189d8>
     678:	64 6c 69 62 	*unknown*
     67c:	2e 68 00 04 	*unknown*
     680:	00 00 00 00 	l.j 680 <_or1k_reset+0x580>
     684:	05 02 00 00 	l.jal 4080684 <_end+0x407bc18>
     688:	25 9c 03 c3 	*unknown*
     68c:	00 01 03 c4 	l.j 4159c <_end+0x3cb30>
     690:	00 f2 03 bc 	l.j 3c81580 <_end+0x3c7cb14>
     694:	7f 20 03 c4 	*unknown*
     698:	00 3c 03 4b 	l.j f013c4 <_end+0xefc958>
     69c:	20 22 43 00 	*unknown*
     6a0:	02 04 02 97 	l.j f81010fc <_end+0xf80fc690>
     6a4:	03 79 4a 51 	l.j fde52fe8 <_end+0xfde4e57c>
     6a8:	42 5b 31 03 	*unknown*
     6ac:	71 3c 03 15 	*unknown*
     6b0:	20 59 03 09 	*unknown*
     6b4:	2e 00 02 04 	*unknown*
     6b8:	01 06 4a 06 	l.j 4192ed0 <_end+0x418e464>
     6bc:	03 5d 4a 03 	l.j fd752ec8 <_end+0xfd74e45c>
     6c0:	2c 3c 41 00 	*unknown*
     6c4:	02 04 01 06 	l.j f8100adc <_end+0xf80fc070>
     6c8:	4a 06 44 2f 	*unknown*
     6cc:	03 bf 7f 20 	l.j fefe034c <_end+0xfefdb8e0>
     6d0:	03 ce 00 3c 	l.j ff3807c0 <_end+0xff37bd54>
     6d4:	03 54 c8 59 	l.j fd532838 <_end+0xfd52ddcc>
     6d8:	76 33 03 1b 	*unknown*
     6dc:	2e 21 02 02 	*unknown*
     6e0:	00 01 01 00 	l.j 40ae0 <_end+0x3c074>
     6e4:	00 02 55 00 	l.j 95ae4 <_end+0x91078>
     6e8:	02 00 00 01 	l.j f80006ec <_end+0xf7ffbc80>
     6ec:	de 04 01 fb 	l.sh -32261(r4),r0
     6f0:	0e 0d 00 01 	l.bnf f83406f4 <_end+0xf833bc88>
     6f4:	01 01 01 00 	l.j 4040af4 <_end+0x403c088>
     6f8:	00 00 01 00 	l.j af8 <_or1k_reset+0x9f8>
     6fc:	00 01 2e 2e 	l.j 4bfb4 <_end+0x47548>
     700:	2f 2e 2e 2f 	*unknown*
     704:	2e 2e 2f 2e 	*unknown*
     708:	2e 2f 6e 65 	*unknown*
     70c:	77 6c 69 62 	*unknown*
     710:	2d 32 2e 32 	*unknown*
     714:	2e 30 2e 32 	*unknown*
     718:	30 31 35 30 	*unknown*
     71c:	38 32 34 2f 	*unknown*
     720:	6c 69 62 67 	l.lwa r3,25191(r9)
     724:	6c 6f 73 73 	l.lwa r3,29555(r15)
     728:	2f 6f 72 31 	*unknown*
     72c:	6b 00 2f 6f 	*unknown*
     730:	70 74 2f 6f 	*unknown*
     734:	72 31 6b 2d 	*unknown*
     738:	65 6c 66 2f 	*unknown*
     73c:	6c 69 62 2f 	l.lwa r3,25135(r9)
     740:	67 63 63 2f 	*unknown*
     744:	6f 72 31 6b 	l.lwa r27,12651(r18)
     748:	2d 65 6c 66 	*unknown*
     74c:	2f 34 2e 39 	*unknown*
     750:	2e 32 2f 69 	*unknown*
     754:	6e 63 6c 75 	l.lwa r19,27765(r3)
     758:	64 65 00 2f 	*unknown*
     75c:	68 6f 6d 65 	*unknown*
     760:	2f 61 6e 64 	*unknown*
     764:	72 7a 65 6a 	*unknown*
     768:	72 2f 44 69 	*unknown*
     76c:	67 69 74 61 	*unknown*
     770:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     774:	50 53 6f 43 	*unknown*
     778:	2f 6e 65 77 	*unknown*
     77c:	6c 69 62 2d 	l.lwa r3,25133(r9)
     780:	32 2e 32 2e 	*unknown*
     784:	30 2e 32 30 	*unknown*
     788:	31 35 30 38 	*unknown*
     78c:	32 34 2f 6e 	*unknown*
     790:	65 77 6c 69 	*unknown*
     794:	62 2f 6c 69 	*unknown*
     798:	62 63 2f 69 	*unknown*
     79c:	6e 63 6c 75 	l.lwa r19,27765(r3)
     7a0:	64 65 2f 73 	*unknown*
     7a4:	79 73 00 2f 	*unknown*
     7a8:	68 6f 6d 65 	*unknown*
     7ac:	2f 61 6e 64 	*unknown*
     7b0:	72 7a 65 6a 	*unknown*
     7b4:	72 2f 44 69 	*unknown*
     7b8:	67 69 74 61 	*unknown*
     7bc:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     7c0:	50 53 6f 43 	*unknown*
     7c4:	2f 6e 65 77 	*unknown*
     7c8:	6c 69 62 2d 	l.lwa r3,25133(r9)
     7cc:	32 2e 32 2e 	*unknown*
     7d0:	30 2e 32 30 	*unknown*
     7d4:	31 35 30 38 	*unknown*
     7d8:	32 34 2f 6e 	*unknown*
     7dc:	65 77 6c 69 	*unknown*
     7e0:	62 2f 6c 69 	*unknown*
     7e4:	62 63 2f 69 	*unknown*
     7e8:	6e 63 6c 75 	l.lwa r19,27765(r3)
     7ec:	64 65 2f 6d 	*unknown*
     7f0:	61 63 68 69 	*unknown*
     7f4:	6e 65 00 2f 	l.lwa r19,47(r5)
     7f8:	68 6f 6d 65 	*unknown*
     7fc:	2f 61 6e 64 	*unknown*
     800:	72 7a 65 6a 	*unknown*
     804:	72 2f 44 69 	*unknown*
     808:	67 69 74 61 	*unknown*
     80c:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     810:	50 53 6f 43 	*unknown*
     814:	2f 6e 65 77 	*unknown*
     818:	6c 69 62 2d 	l.lwa r3,25133(r9)
     81c:	32 2e 32 2e 	*unknown*
     820:	30 2e 32 30 	*unknown*
     824:	31 35 30 38 	*unknown*
     828:	32 34 2f 6e 	*unknown*
     82c:	65 77 6c 69 	*unknown*
     830:	62 2f 6c 69 	*unknown*
     834:	62 63 2f 69 	*unknown*
     838:	6e 63 6c 75 	l.lwa r19,27765(r3)
     83c:	64 65 00 00 	*unknown*
     840:	73 79 73 63 	*unknown*
     844:	61 6c 6c 73 	*unknown*
     848:	2e 63 00 01 	*unknown*
     84c:	00 00 73 74 	l.j 1d61c <_end+0x18bb0>
     850:	64 64 65 66 	*unknown*
     854:	2e 68 00 02 	*unknown*
     858:	00 00 6c 6f 	l.j 1ba14 <_end+0x16fa8>
     85c:	63 6b 2e 68 	*unknown*
     860:	00 03 00 00 	l.j c0860 <_end+0xbbdf4>
     864:	5f 74 79 70 	*unknown*
     868:	65 73 2e 68 	*unknown*
     86c:	00 03 00 00 	l.j c086c <_end+0xbbe00>
     870:	72 65 65 6e 	*unknown*
     874:	74 2e 68 00 	*unknown*
     878:	03 00 00 74 	l.j fc000a48 <_end+0xfbffbfdc>
     87c:	79 70 65 73 	*unknown*
     880:	2e 68 00 04 	*unknown*
     884:	00 00 74 79 	l.j 1da68 <_end+0x18ffc>
     888:	70 65 73 2e 	*unknown*
     88c:	68 00 03 00 	*unknown*
     890:	00 73 74 61 	l.j 1cdda14 <_end+0x1cd8fa8>
     894:	74 2e 68 00 	*unknown*
     898:	03 00 00 5f 	l.j fc000a14 <_end+0xfbffbfa8>
     89c:	74 69 6d 65 	*unknown*
     8a0:	76 61 6c 2e 	*unknown*
     8a4:	68 00 03 00 	*unknown*
     8a8:	00 72 65 65 	l.j 1c99e3c <_end+0x1c953d0>
     8ac:	6e 74 2e 68 	l.lwa r19,11880(r20)
     8b0:	00 05 00 00 	l.j 1408b0 <_end+0x13be44>
     8b4:	62 6f 61 72 	*unknown*
     8b8:	64 2e 68 00 	*unknown*
     8bc:	01 00 00 65 	l.j 4000a50 <_end+0x3ffbfe4>
     8c0:	72 72 6e 6f 	*unknown*
     8c4:	2e 68 00 03 	*unknown*
     8c8:	00 00 00 00 	l.j 8c8 <_or1k_reset+0x7c8>
     8cc:	05 02 00 00 	l.jal 40808cc <_end+0x407be60>
     8d0:	27 84 03 1f 	*unknown*
     8d4:	01 5c 1c 20 	l.j 5707954 <_end+0x5702ee8>
     8d8:	32 35 00 02 	*unknown*
     8dc:	04 02 2b 00 	l.jal 8b4dc <_end+0x86a70>
     8e0:	02 04 02 2a 	l.j f8101188 <_end+0xf80fc71c>
     8e4:	3d 4b 3e 2a 	*unknown*
     8e8:	43 86 2f 00 	*unknown*
     8ec:	02 04 01 2f 	l.j f8100da8 <_end+0xf80fc33c>
     8f0:	33 2f 22 2c 	*unknown*
     8f4:	22 36 2f 22 	*unknown*
     8f8:	2c 22 32 2f 	*unknown*
     8fc:	4c 5c 2f 22 	*unknown*
     900:	2c 22 32 2f 	*unknown*
     904:	22 2c 22 32 	*unknown*
     908:	2f 22 2c 22 	*unknown*
     90c:	32 2f 22 2c 	*unknown*
     910:	22 32 2f 22 	*unknown*
     914:	2c 22 32 2f 	*unknown*
     918:	22 2c 22 32 	*unknown*
     91c:	2f 4c 5c 2f 	*unknown*
     920:	22 2c 22 32 	*unknown*
     924:	2f 22 2c 22 	*unknown*
     928:	32 2f 22 2c 	*unknown*
     92c:	22 32 2f 22 	*unknown*
     930:	2c 22 32 2f 	*unknown*
     934:	22 2c 22 02 	*unknown*
     938:	02 00 01 01 	l.j f8000d3c <_end+0xf7ffc2d0>
     93c:	00 00 01 c8 	l.j 105c <_or1k_reset+0xf5c>
     940:	00 02 00 00 	l.j 80940 <_end+0x7bed4>
     944:	01 64 04 01 	l.j 5901948 <_end+0x58fcedc>
     948:	fb 0e 0d 00 	*unknown*
     94c:	01 01 01 01 	l.j 4040d50 <_end+0x403c2e4>
     950:	00 00 00 01 	l.j 954 <_or1k_reset+0x854>
     954:	00 00 01 2e 	l.j e0c <_or1k_reset+0xd0c>
     958:	2e 2f 2e 2e 	*unknown*
     95c:	2f 2e 2e 2f 	*unknown*
     960:	2e 2e 2f 6e 	*unknown*
     964:	65 77 6c 69 	*unknown*
     968:	62 2d 32 2e 	*unknown*
     96c:	32 2e 30 2e 	*unknown*
     970:	32 30 31 35 	*unknown*
     974:	30 38 32 34 	*unknown*
     978:	2f 6c 69 62 	*unknown*
     97c:	67 6c 6f 73 	*unknown*
     980:	73 2f 6f 72 	*unknown*
     984:	31 6b 00 2f 	*unknown*
     988:	68 6f 6d 65 	*unknown*
     98c:	2f 61 6e 64 	*unknown*
     990:	72 7a 65 6a 	*unknown*
     994:	72 2f 44 69 	*unknown*
     998:	67 69 74 61 	*unknown*
     99c:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     9a0:	50 53 6f 43 	*unknown*
     9a4:	2f 6e 65 77 	*unknown*
     9a8:	6c 69 62 2d 	l.lwa r3,25133(r9)
     9ac:	32 2e 32 2e 	*unknown*
     9b0:	30 2e 32 30 	*unknown*
     9b4:	31 35 30 38 	*unknown*
     9b8:	32 34 2f 6e 	*unknown*
     9bc:	65 77 6c 69 	*unknown*
     9c0:	62 2f 6c 69 	*unknown*
     9c4:	62 63 2f 69 	*unknown*
     9c8:	6e 63 6c 75 	l.lwa r19,27765(r3)
     9cc:	64 65 2f 6d 	*unknown*
     9d0:	61 63 68 69 	*unknown*
     9d4:	6e 65 00 2f 	l.lwa r19,47(r5)
     9d8:	68 6f 6d 65 	*unknown*
     9dc:	2f 61 6e 64 	*unknown*
     9e0:	72 7a 65 6a 	*unknown*
     9e4:	72 2f 44 69 	*unknown*
     9e8:	67 69 74 61 	*unknown*
     9ec:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     9f0:	50 53 6f 43 	*unknown*
     9f4:	2f 6e 65 77 	*unknown*
     9f8:	6c 69 62 2d 	l.lwa r3,25133(r9)
     9fc:	32 2e 32 2e 	*unknown*
     a00:	30 2e 32 30 	*unknown*
     a04:	31 35 30 38 	*unknown*
     a08:	32 34 2f 6e 	*unknown*
     a0c:	65 77 6c 69 	*unknown*
     a10:	62 2f 6c 69 	*unknown*
     a14:	62 63 2f 69 	*unknown*
     a18:	6e 63 6c 75 	l.lwa r19,27765(r3)
     a1c:	64 65 2f 73 	*unknown*
     a20:	79 73 00 2e 	*unknown*
     a24:	2e 2f 2e 2e 	*unknown*
     a28:	2f 2e 2e 2f 	*unknown*
     a2c:	2e 2e 2f 6e 	*unknown*
     a30:	65 77 6c 69 	*unknown*
     a34:	62 2d 32 2e 	*unknown*
     a38:	32 2e 30 2e 	*unknown*
     a3c:	32 30 31 35 	*unknown*
     a40:	30 38 32 34 	*unknown*
     a44:	2f 6c 69 62 	*unknown*
     a48:	67 6c 6f 73 	*unknown*
     a4c:	73 2f 6f 72 	*unknown*
     a50:	31 6b 2f 69 	*unknown*
     a54:	6e 63 6c 75 	l.lwa r19,27765(r3)
     a58:	64 65 00 00 	*unknown*
     a5c:	6f 72 31 6b 	l.lwa r27,12651(r18)
     a60:	5f 75 61 72 	*unknown*
     a64:	74 2e 63 00 	*unknown*
     a68:	01 00 00 5f 	l.j 4000be4 <_end+0x3ffc178>
     a6c:	64 65 66 61 	*unknown*
     a70:	75 6c 74 5f 	*unknown*
     a74:	74 79 70 65 	*unknown*
     a78:	73 2e 68 00 	*unknown*
     a7c:	02 00 00 5f 	l.j f8000bf8 <_end+0xf7ffc18c>
     a80:	73 74 64 69 	*unknown*
     a84:	6e 74 2e 68 	l.lwa r19,11880(r20)
     a88:	00 03 00 00 	l.j c0a88 <_end+0xbc01c>
     a8c:	6f 72 31 6b 	l.lwa r27,12651(r18)
     a90:	2d 73 75 70 	*unknown*
     a94:	70 6f 72 74 	*unknown*
     a98:	2e 68 00 04 	*unknown*
     a9c:	00 00 62 6f 	l.j 19458 <_end+0x149ec>
     aa0:	61 72 64 2e 	*unknown*
     aa4:	68 00 01 00 	*unknown*
     aa8:	00 00 00 05 	l.j abc <_or1k_reset+0x9bc>
     aac:	02 00 00 2a 	l.j f8000b54 <_end+0xf7ffc0e8>
     ab0:	2c 03 dd 00 	*unknown*
     ab4:	01 13 1f 21 	l.j 44c8738 <_end+0x44c3ccc>
     ab8:	1f 21 03 0c 	*unknown*
     abc:	3c 3f 24 1c 	*unknown*
     ac0:	24 1c 24 1c 	*unknown*
     ac4:	24 20 36 1d 	*unknown*
     ac8:	23 1d 23 2b 	*unknown*
     acc:	23 39 23 2f 	*unknown*
     ad0:	21 1f 22 1f 	*unknown*
     ad4:	03 0c 20 03 	l.j fc308ae0 <_end+0xfc304074>
     ad8:	74 20 21 03 	*unknown*
     adc:	10 2e 03 70 	l.bf b8189c <_end+0xb7ce30>
     ae0:	20 36 4d 3f 	*unknown*
     ae4:	4d 03 65 66 	*unknown*
     ae8:	03 1e 2e 00 	l.j fc78c2e8 <_end+0xfc78787c>
     aec:	02 04 01 84 	l.j f81010fc <_end+0xf80fc690>
     af0:	5b 2f 3f 25 	*unknown*
     af4:	1b 25 1b 25 	*unknown*
     af8:	1d 23 1d 1e 	*unknown*
     afc:	22 23 23 1d 	*unknown*
     b00:	23 6a 2f 02 	*unknown*
     b04:	05 00 01 01 	l.jal 4000f08 <_end+0x3ffc49c>
     b08:	00 00 02 0e 	l.j 1340 <_or1k_reset+0x1240>
     b0c:	00 02 00 00 	l.j 80b0c <_end+0x7c0a0>
     b10:	01 6e 04 01 	l.j 5b81b14 <_end+0x5b7d0a8>
     b14:	fb 0e 0d 00 	*unknown*
     b18:	01 01 01 01 	l.j 4040f1c <_end+0x403c4b0>
     b1c:	00 00 00 01 	l.j b20 <_or1k_reset+0xa20>
     b20:	00 00 01 2e 	l.j fd8 <_or1k_reset+0xed8>
     b24:	2e 2f 2e 2e 	*unknown*
     b28:	2f 2e 2e 2f 	*unknown*
     b2c:	2e 2e 2f 6e 	*unknown*
     b30:	65 77 6c 69 	*unknown*
     b34:	62 2d 32 2e 	*unknown*
     b38:	32 2e 30 2e 	*unknown*
     b3c:	32 30 31 35 	*unknown*
     b40:	30 38 32 34 	*unknown*
     b44:	2f 6c 69 62 	*unknown*
     b48:	67 6c 6f 73 	*unknown*
     b4c:	73 2f 6f 72 	*unknown*
     b50:	31 6b 00 2e 	*unknown*
     b54:	2e 2f 2e 2e 	*unknown*
     b58:	2f 2e 2e 2f 	*unknown*
     b5c:	2e 2e 2f 6e 	*unknown*
     b60:	65 77 6c 69 	*unknown*
     b64:	62 2d 32 2e 	*unknown*
     b68:	32 2e 30 2e 	*unknown*
     b6c:	32 30 31 35 	*unknown*
     b70:	30 38 32 34 	*unknown*
     b74:	2f 6c 69 62 	*unknown*
     b78:	67 6c 6f 73 	*unknown*
     b7c:	73 2f 6f 72 	*unknown*
     b80:	31 6b 2f 69 	*unknown*
     b84:	6e 63 6c 75 	l.lwa r19,27765(r3)
     b88:	64 65 00 2f 	*unknown*
     b8c:	68 6f 6d 65 	*unknown*
     b90:	2f 61 6e 64 	*unknown*
     b94:	72 7a 65 6a 	*unknown*
     b98:	72 2f 44 69 	*unknown*
     b9c:	67 69 74 61 	*unknown*
     ba0:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     ba4:	50 53 6f 43 	*unknown*
     ba8:	2f 6e 65 77 	*unknown*
     bac:	6c 69 62 2d 	l.lwa r3,25133(r9)
     bb0:	32 2e 32 2e 	*unknown*
     bb4:	30 2e 32 30 	*unknown*
     bb8:	31 35 30 38 	*unknown*
     bbc:	32 34 2f 6e 	*unknown*
     bc0:	65 77 6c 69 	*unknown*
     bc4:	62 2f 6c 69 	*unknown*
     bc8:	62 63 2f 69 	*unknown*
     bcc:	6e 63 6c 75 	l.lwa r19,27765(r3)
     bd0:	64 65 2f 6d 	*unknown*
     bd4:	61 63 68 69 	*unknown*
     bd8:	6e 65 00 2f 	l.lwa r19,47(r5)
     bdc:	68 6f 6d 65 	*unknown*
     be0:	2f 61 6e 64 	*unknown*
     be4:	72 7a 65 6a 	*unknown*
     be8:	72 2f 44 69 	*unknown*
     bec:	67 69 74 61 	*unknown*
     bf0:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     bf4:	50 53 6f 43 	*unknown*
     bf8:	2f 6e 65 77 	*unknown*
     bfc:	6c 69 62 2d 	l.lwa r3,25133(r9)
     c00:	32 2e 32 2e 	*unknown*
     c04:	30 2e 32 30 	*unknown*
     c08:	31 35 30 38 	*unknown*
     c0c:	32 34 2f 6e 	*unknown*
     c10:	65 77 6c 69 	*unknown*
     c14:	62 2f 6c 69 	*unknown*
     c18:	62 63 2f 69 	*unknown*
     c1c:	6e 63 6c 75 	l.lwa r19,27765(r3)
     c20:	64 65 2f 73 	*unknown*
     c24:	79 73 00 00 	*unknown*
     c28:	69 6e 74 65 	*unknown*
     c2c:	72 72 75 70 	*unknown*
     c30:	74 73 2e 63 	*unknown*
     c34:	00 01 00 00 	l.j 40c34 <_end+0x3c1c8>
     c38:	6f 72 31 6b 	l.lwa r27,12651(r18)
     c3c:	2d 73 75 70 	*unknown*
     c40:	70 6f 72 74 	*unknown*
     c44:	2e 68 00 02 	*unknown*
     c48:	00 00 5f 64 	l.j 189d8 <_end+0x13f6c>
     c4c:	65 66 61 75 	*unknown*
     c50:	6c 74 5f 74 	l.lwa r3,24436(r20)
     c54:	79 70 65 73 	*unknown*
     c58:	2e 68 00 03 	*unknown*
     c5c:	00 00 5f 73 	l.j 18a28 <_end+0x13fbc>
     c60:	74 64 69 6e 	*unknown*
     c64:	74 2e 68 00 	*unknown*
     c68:	04 00 00 6f 	l.jal e24 <_or1k_reset+0xd24>
     c6c:	72 31 6b 2d 	*unknown*
     c70:	69 6e 74 65 	*unknown*
     c74:	72 6e 61 6c 	*unknown*
     c78:	73 2e 68 00 	*unknown*
     c7c:	01 00 00 00 	l.j 4000c7c <_end+0x3ffc210>
     c80:	00 05 02 00 	l.j 141480 <_end+0x13ca14>
     c84:	00 2f 38 03 	l.j bcec90 <_end+0xbca224>
     c88:	22 01 25 37 	*unknown*
     c8c:	25 21 03 7a 	*unknown*
     c90:	20 26 1f 21 	*unknown*
     c94:	30 4e 04 02 	*unknown*
     c98:	03 d5 01 20 	l.j ff541118 <_end+0xff53c6ac>
     c9c:	04 01 03 ab 	l.jal 41b48 <_end+0x3d0dc>
     ca0:	7e 20 04 02 	*unknown*
     ca4:	03 d5 01 20 	l.j ff541124 <_end+0xff53c6b8>
     ca8:	04 01 03 ad 	l.jal 41b5c <_end+0x3d0f0>
     cac:	7e 20 04 02 	*unknown*
     cb0:	03 c6 01 20 	l.j ff181130 <_end+0xff17c6c4>
     cb4:	04 01 03 bc 	l.jal 41ba4 <_end+0x3d138>
     cb8:	7e 20 40 04 	*unknown*
     cbc:	02 03 cd 01 	l.j f80f40c0 <_end+0xf80ef654>
     cc0:	2e 04 01 03 	*unknown*
     cc4:	b3 7e 20 04 	l.muli r27,r30,8196
     cc8:	02 03 cd 01 	l.j f80f40cc <_end+0xf80ef660>
     ccc:	20 04 01 03 	*unknown*
     cd0:	b6 7e 20 04 	l.mfspr r19,r30,0x2004
     cd4:	02 03 bd 01 	l.j f80f00d8 <_end+0xf80eb66c>
     cd8:	2e 04 01 03 	*unknown*
     cdc:	c6 7e 20 1f 	*unknown*
     ce0:	21 4e 04 02 	*unknown*
     ce4:	03 c3 01 2e 	l.j ff0c119c <_end+0xff0bc730>
     ce8:	04 01 03 bd 	l.jal 41bdc <_end+0x3d170>
     cec:	7e 20 04 02 	*unknown*
     cf0:	03 c3 01 20 	l.j ff0c1170 <_end+0xff0bc704>
     cf4:	04 01 03 bf 	l.jal 41bf0 <_end+0x3d184>
     cf8:	7e 20 00 02 	*unknown*
     cfc:	04 04 06 66 	l.jal 102694 <_end+0xfdc28>
     d00:	04 02 00 02 	l.jal 80d08 <_end+0x7c29c>
     d04:	04 04 06 03 	l.jal 102510 <_end+0xfdaa4>
     d08:	b4 01 20 04 	l.mfspr r0,r1,0x2004
     d0c:	01 00 02 04 	l.j 400151c <_end+0x3ffcab0>
     d10:	04 03 ce 7e 	l.jal f4708 <_end+0xefc9c>
     d14:	2e 02 04 00 	*unknown*
     d18:	01 01 00 00 	l.j 4040d18 <_end+0x403c2ac>
     d1c:	01 d0 00 02 	l.j 7400d24 <_end+0x73fc2b8>
     d20:	00 00 01 7a 	l.j 1308 <_or1k_reset+0x1208>
     d24:	04 01 fb 0e 	l.jal 7f95c <_end+0x7aef0>
     d28:	0d 00 01 01 	l.bnf 400112c <_end+0x3ffc6c0>
     d2c:	01 01 00 00 	l.j 4040d2c <_end+0x403c2c0>
     d30:	00 01 00 00 	l.j 40d30 <_end+0x3c2c4>
     d34:	01 2e 2e 2f 	l.j 4b8c5f0 <_end+0x4b87b84>
     d38:	2e 2e 2f 2e 	*unknown*
     d3c:	2e 2f 2e 2e 	*unknown*
     d40:	2f 6e 65 77 	*unknown*
     d44:	6c 69 62 2d 	l.lwa r3,25133(r9)
     d48:	32 2e 32 2e 	*unknown*
     d4c:	30 2e 32 30 	*unknown*
     d50:	31 35 30 38 	*unknown*
     d54:	32 34 2f 6c 	*unknown*
     d58:	69 62 67 6c 	*unknown*
     d5c:	6f 73 73 2f 	l.lwa r27,29487(r19)
     d60:	6f 72 31 6b 	l.lwa r27,12651(r18)
     d64:	00 2f 68 6f 	l.j bdaf20 <_end+0xbd64b4>
     d68:	6d 65 2f 61 	l.lwa r11,12129(r5)
     d6c:	6e 64 72 7a 	l.lwa r19,29306(r4)
     d70:	65 6a 72 2f 	*unknown*
     d74:	44 69 67 69 	*unknown*
     d78:	74 61 6c 2f 	*unknown*
     d7c:	4f 52 50 53 	*unknown*
     d80:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     d84:	65 77 6c 69 	*unknown*
     d88:	62 2d 32 2e 	*unknown*
     d8c:	32 2e 30 2e 	*unknown*
     d90:	32 30 31 35 	*unknown*
     d94:	30 38 32 34 	*unknown*
     d98:	2f 6e 65 77 	*unknown*
     d9c:	6c 69 62 2f 	l.lwa r3,25135(r9)
     da0:	6c 69 62 63 	l.lwa r3,25187(r9)
     da4:	2f 69 6e 63 	*unknown*
     da8:	6c 75 64 65 	l.lwa r3,25701(r21)
     dac:	2f 6d 61 63 	*unknown*
     db0:	68 69 6e 65 	*unknown*
     db4:	00 2f 68 6f 	l.j bdaf70 <_end+0xbd6504>
     db8:	6d 65 2f 61 	l.lwa r11,12129(r5)
     dbc:	6e 64 72 7a 	l.lwa r19,29306(r4)
     dc0:	65 6a 72 2f 	*unknown*
     dc4:	44 69 67 69 	*unknown*
     dc8:	74 61 6c 2f 	*unknown*
     dcc:	4f 52 50 53 	*unknown*
     dd0:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     dd4:	65 77 6c 69 	*unknown*
     dd8:	62 2d 32 2e 	*unknown*
     ddc:	32 2e 30 2e 	*unknown*
     de0:	32 30 31 35 	*unknown*
     de4:	30 38 32 34 	*unknown*
     de8:	2f 6e 65 77 	*unknown*
     dec:	6c 69 62 2f 	l.lwa r3,25135(r9)
     df0:	6c 69 62 63 	l.lwa r3,25187(r9)
     df4:	2f 69 6e 63 	*unknown*
     df8:	6c 75 64 65 	l.lwa r3,25701(r21)
     dfc:	2f 73 79 73 	*unknown*
     e00:	00 2f 6f 70 	l.j bdcbc0 <_end+0xbd8154>
     e04:	74 2f 6f 72 	*unknown*
     e08:	31 6b 2d 65 	*unknown*
     e0c:	6c 66 2f 6c 	l.lwa r3,12140(r6)
     e10:	69 62 2f 67 	*unknown*
     e14:	63 63 2f 6f 	*unknown*
     e18:	72 31 6b 2d 	*unknown*
     e1c:	65 6c 66 2f 	*unknown*
     e20:	34 2e 39 2e 	*unknown*
     e24:	32 2f 69 6e 	*unknown*
     e28:	63 6c 75 64 	*unknown*
     e2c:	65 00 00 69 	*unknown*
     e30:	6d 70 75 72 	l.lwa r11,30066(r16)
     e34:	65 2e 63 00 	*unknown*
     e38:	01 00 00 5f 	l.j 4000fb4 <_end+0x3ffc548>
     e3c:	64 65 66 61 	*unknown*
     e40:	75 6c 74 5f 	*unknown*
     e44:	74 79 70 65 	*unknown*
     e48:	73 2e 68 00 	*unknown*
     e4c:	02 00 00 6c 	l.j f8000ffc <_end+0xf7ffc590>
     e50:	6f 63 6b 2e 	l.lwa r27,27438(r3)
     e54:	68 00 03 00 	*unknown*
     e58:	00 5f 74 79 	l.j 17de03c <_end+0x17d95d0>
     e5c:	70 65 73 2e 	*unknown*
     e60:	68 00 03 00 	*unknown*
     e64:	00 73 74 64 	l.j 1cddff4 <_end+0x1cd9588>
     e68:	64 65 66 2e 	*unknown*
     e6c:	68 00 04 00 	*unknown*
     e70:	00 72 65 65 	l.j 1c9a404 <_end+0x1c95998>
     e74:	6e 74 2e 68 	l.lwa r19,11880(r20)
     e78:	00 03 00 00 	l.j c0e78 <_end+0xbc40c>
     e7c:	5f 73 74 64 	*unknown*
     e80:	69 6e 74 2e 	*unknown*
     e84:	68 00 03 00 	*unknown*
     e88:	00 6f 72 31 	l.j 1bdd74c <_end+0x1bd8ce0>
     e8c:	6b 2d 69 6e 	*unknown*
     e90:	74 65 72 6e 	*unknown*
     e94:	61 6c 73 2e 	*unknown*
     e98:	68 00 01 00 	*unknown*
     e9c:	00 00 00 05 	l.j eb0 <_or1k_reset+0xdb0>
     ea0:	02 00 00 30 	l.j f8000f60 <_end+0xf7ffc4f4>
     ea4:	e4 03 c4 00 	*unknown*
     ea8:	01 03 19 20 	l.j 40c7328 <_end+0x40c28bc>
     eac:	03 67 20 03 	l.j fd9c8eb8 <_end+0xfd9c444c>
     eb0:	19 20 03 67 	l.movhi r9,0x367
     eb4:	2e 03 19 82 	*unknown*
     eb8:	03 67 2e 03 	l.j fd9cc6c4 <_end+0xfd9c7c58>
     ebc:	19 20 03 67 	l.movhi r9,0x367
     ec0:	20 03 19 20 	*unknown*
     ec4:	3d 1f 91 1f 	*unknown*
     ec8:	c9 2d 4b 3f 	*unknown*
     ecc:	1d 69 1d 23 	*unknown*
     ed0:	1d 23 23 03 	*unknown*
     ed4:	7a 20 26 03 	*unknown*
     ed8:	7a 20 96 03 	*unknown*
     edc:	7a 20 36 cb 	*unknown*
     ee0:	13 1f 21 1f 	l.bf fc7c935c <_end+0xfc7c48f0>
     ee4:	22 03 0a 4a 	*unknown*
     ee8:	33 02 03 00 	*unknown*
     eec:	01 01 00 00 	l.j 4040eec <_end+0x403c480>
     ef0:	01 8f 00 02 	l.j 63c0ef8 <_end+0x63bc48c>
     ef4:	00 00 01 68 	l.j 1494 <_or1k_reset+0x1394>
     ef8:	04 01 fb 0e 	l.jal 7fb30 <_end+0x7b0c4>
     efc:	0d 00 01 01 	l.bnf 4001300 <_end+0x3ffc894>
     f00:	01 01 00 00 	l.j 4040f00 <_end+0x403c494>
     f04:	00 01 00 00 	l.j 40f04 <_end+0x3c498>
     f08:	01 2e 2e 2f 	l.j 4b8c7c4 <_end+0x4b87d58>
     f0c:	2e 2e 2f 2e 	*unknown*
     f10:	2e 2f 2e 2e 	*unknown*
     f14:	2f 6e 65 77 	*unknown*
     f18:	6c 69 62 2d 	l.lwa r3,25133(r9)
     f1c:	32 2e 32 2e 	*unknown*
     f20:	30 2e 32 30 	*unknown*
     f24:	31 35 30 38 	*unknown*
     f28:	32 34 2f 6c 	*unknown*
     f2c:	69 62 67 6c 	*unknown*
     f30:	6f 73 73 2f 	l.lwa r27,29487(r19)
     f34:	6f 72 31 6b 	l.lwa r27,12651(r18)
     f38:	00 2f 68 6f 	l.j bdb0f4 <_end+0xbd6688>
     f3c:	6d 65 2f 61 	l.lwa r11,12129(r5)
     f40:	6e 64 72 7a 	l.lwa r19,29306(r4)
     f44:	65 6a 72 2f 	*unknown*
     f48:	44 69 67 69 	*unknown*
     f4c:	74 61 6c 2f 	*unknown*
     f50:	4f 52 50 53 	*unknown*
     f54:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     f58:	65 77 6c 69 	*unknown*
     f5c:	62 2d 32 2e 	*unknown*
     f60:	32 2e 30 2e 	*unknown*
     f64:	32 30 31 35 	*unknown*
     f68:	30 38 32 34 	*unknown*
     f6c:	2f 6e 65 77 	*unknown*
     f70:	6c 69 62 2f 	l.lwa r3,25135(r9)
     f74:	6c 69 62 63 	l.lwa r3,25187(r9)
     f78:	2f 69 6e 63 	*unknown*
     f7c:	6c 75 64 65 	l.lwa r3,25701(r21)
     f80:	2f 6d 61 63 	*unknown*
     f84:	68 69 6e 65 	*unknown*
     f88:	00 2f 68 6f 	l.j bdb144 <_end+0xbd66d8>
     f8c:	6d 65 2f 61 	l.lwa r11,12129(r5)
     f90:	6e 64 72 7a 	l.lwa r19,29306(r4)
     f94:	65 6a 72 2f 	*unknown*
     f98:	44 69 67 69 	*unknown*
     f9c:	74 61 6c 2f 	*unknown*
     fa0:	4f 52 50 53 	*unknown*
     fa4:	6f 43 2f 6e 	l.lwa r26,12142(r3)
     fa8:	65 77 6c 69 	*unknown*
     fac:	62 2d 32 2e 	*unknown*
     fb0:	32 2e 30 2e 	*unknown*
     fb4:	32 30 31 35 	*unknown*
     fb8:	30 38 32 34 	*unknown*
     fbc:	2f 6e 65 77 	*unknown*
     fc0:	6c 69 62 2f 	l.lwa r3,25135(r9)
     fc4:	6c 69 62 63 	l.lwa r3,25187(r9)
     fc8:	2f 69 6e 63 	*unknown*
     fcc:	6c 75 64 65 	l.lwa r3,25701(r21)
     fd0:	2f 73 79 73 	*unknown*
     fd4:	00 2e 2e 2f 	l.j b8c890 <_end+0xb87e24>
     fd8:	2e 2e 2f 2e 	*unknown*
     fdc:	2e 2f 2e 2e 	*unknown*
     fe0:	2f 6e 65 77 	*unknown*
     fe4:	6c 69 62 2d 	l.lwa r3,25133(r9)
     fe8:	32 2e 32 2e 	*unknown*
     fec:	30 2e 32 30 	*unknown*
     ff0:	31 35 30 38 	*unknown*
     ff4:	32 34 2f 6c 	*unknown*
     ff8:	69 62 67 6c 	*unknown*
     ffc:	6f 73 73 2f 	l.lwa r27,29487(r19)
    1000:	6f 72 31 6b 	l.lwa r27,12651(r18)
    1004:	2f 69 6e 63 	*unknown*
    1008:	6c 75 64 65 	l.lwa r3,25701(r21)
    100c:	00 00 75 74 	l.j 1e5dc <_end+0x19b70>
    1010:	69 6c 2e 63 	*unknown*
    1014:	00 01 00 00 	l.j 41014 <_end+0x3c5a8>
    1018:	5f 64 65 66 	*unknown*
    101c:	61 75 6c 74 	*unknown*
    1020:	5f 74 79 70 	*unknown*
    1024:	65 73 2e 68 	*unknown*
    1028:	00 02 00 00 	l.j 81028 <_end+0x7c5bc>
    102c:	5f 73 74 64 	*unknown*
    1030:	69 6e 74 2e 	*unknown*
    1034:	68 00 03 00 	*unknown*
    1038:	00 6f 72 31 	l.j 1bdd8fc <_end+0x1bd8e90>
    103c:	6b 2d 73 75 	*unknown*
    1040:	70 70 6f 72 	*unknown*
    1044:	74 2e 68 00 	*unknown*
    1048:	04 00 00 6f 	l.jal 1204 <_or1k_reset+0x1104>
    104c:	72 31 6b 2d 	*unknown*
    1050:	69 6e 74 65 	*unknown*
    1054:	72 6e 61 6c 	*unknown*
    1058:	73 2e 68 00 	*unknown*
    105c:	01 00 00 00 	l.j 400105c <_end+0x3ffc5f0>
    1060:	00 05 02 00 	l.j 141860 <_end+0x13cdf4>
    1064:	00 32 90 03 	l.j ca5070 <_end+0xca0604>
    1068:	28 01 03 1a 	*unknown*
    106c:	4a 35 03 09 	*unknown*
    1070:	58 3e 5a 3d 	*unknown*
    1074:	2f 30 1f 21 	*unknown*
    1078:	5a 59 2f 3d 	*unknown*
    107c:	02 05 00 01 	l.j f8141080 <_end+0xf813c614>
    1080:	01 00 00 00 	l.j 4001080 <_end+0x3ffc614>
    1084:	d4 00 02 00 	l.sw 512(r0),r0
    1088:	00 00 b1 04 	l.j 2d498 <_end+0x28a2c>
    108c:	01 fb 0e 0d 	l.j 7ec48c0 <_end+0x7ebfe54>
    1090:	00 01 01 01 	l.j 41494 <_end+0x3ca28>
    1094:	01 00 00 00 	l.j 4001094 <_end+0x3ffc628>
    1098:	01 00 00 01 	l.j 400109c <_end+0x3ffc630>
    109c:	2e 2e 2f 2e 	*unknown*
    10a0:	2e 2f 2e 2e 	*unknown*
    10a4:	2f 2e 2e 2f 	*unknown*
    10a8:	6e 65 77 6c 	l.lwa r19,30572(r5)
    10ac:	69 62 2d 32 	*unknown*
    10b0:	2e 32 2e 30 	*unknown*
    10b4:	2e 32 30 31 	*unknown*
    10b8:	35 30 38 32 	*unknown*
    10bc:	34 2f 6c 69 	*unknown*
    10c0:	62 67 6c 6f 	*unknown*
    10c4:	73 73 2f 6f 	*unknown*
    10c8:	72 31 6b 00 	*unknown*
    10cc:	2e 2e 2f 2e 	*unknown*
    10d0:	2e 2f 2e 2e 	*unknown*
    10d4:	2f 2e 2e 2f 	*unknown*
    10d8:	6e 65 77 6c 	l.lwa r19,30572(r5)
    10dc:	69 62 2d 32 	*unknown*
    10e0:	2e 32 2e 30 	*unknown*
    10e4:	2e 32 30 31 	*unknown*
    10e8:	35 30 38 32 	*unknown*
    10ec:	34 2f 6c 69 	*unknown*
    10f0:	62 67 6c 6f 	*unknown*
    10f4:	73 73 2f 6f 	*unknown*
    10f8:	72 31 6b 2f 	*unknown*
    10fc:	69 6e 63 6c 	*unknown*
    1100:	75 64 65 00 	*unknown*
    1104:	00 65 78 63 	l.j 195f290 <_end+0x195a824>
    1108:	65 70 74 69 	*unknown*
    110c:	6f 6e 73 2e 	l.lwa r27,29486(r14)
    1110:	63 00 01 00 	*unknown*
    1114:	00 6f 72 31 	l.j 1bdd9d8 <_end+0x1bd8f6c>
    1118:	6b 2d 73 75 	*unknown*
    111c:	70 70 6f 72 	*unknown*
    1120:	74 2e 68 00 	*unknown*
    1124:	02 00 00 6f 	l.j f80012e0 <_end+0xf7ffc874>
    1128:	72 31 6b 2d 	*unknown*
    112c:	69 6e 74 65 	*unknown*
    1130:	72 6e 61 6c 	*unknown*
    1134:	73 2e 68 00 	*unknown*
    1138:	01 00 00 00 	l.j 4001138 <_end+0x3ffc6cc>
    113c:	00 05 02 00 	l.j 14193c <_end+0x13ced0>
    1140:	00 33 50 03 	l.j cd514c <_end+0xcd06e0>
    1144:	0b 01 19 03 	*unknown*
    1148:	79 20 27 03 	*unknown*
    114c:	79 3c 27 03 	*unknown*
    1150:	79 20 27 22 	*unknown*
    1154:	02 04 00 01 	l.j f8101158 <_end+0xf80fc6ec>
    1158:	01 00 00 03 	l.j 4001164 <_end+0x3ffc6f8>
    115c:	48 00 02 00 	*unknown*
    1160:	00 01 74 04 	l.j 5e170 <_end+0x59704>
    1164:	01 fb 0e 0d 	l.j 7ec4998 <_end+0x7ebff2c>
    1168:	00 01 01 01 	l.j 4156c <_end+0x3cb00>
    116c:	01 00 00 00 	l.j 400116c <_end+0x3ffc700>
    1170:	01 00 00 01 	l.j 4001174 <_end+0x3ffc708>
    1174:	2e 2e 2f 2e 	*unknown*
    1178:	2e 2f 2e 2e 	*unknown*
    117c:	2f 2e 2e 2f 	*unknown*
    1180:	6e 65 77 6c 	l.lwa r19,30572(r5)
    1184:	69 62 2d 32 	*unknown*
    1188:	2e 32 2e 30 	*unknown*
    118c:	2e 32 30 31 	*unknown*
    1190:	35 30 38 32 	*unknown*
    1194:	34 2f 6c 69 	*unknown*
    1198:	62 67 6c 6f 	*unknown*
    119c:	73 73 2f 6f 	*unknown*
    11a0:	72 31 6b 00 	*unknown*
    11a4:	2e 2e 2f 2e 	*unknown*
    11a8:	2e 2f 2e 2e 	*unknown*
    11ac:	2f 2e 2e 2f 	*unknown*
    11b0:	6e 65 77 6c 	l.lwa r19,30572(r5)
    11b4:	69 62 2d 32 	*unknown*
    11b8:	2e 32 2e 30 	*unknown*
    11bc:	2e 32 30 31 	*unknown*
    11c0:	35 30 38 32 	*unknown*
    11c4:	34 2f 6c 69 	*unknown*
    11c8:	62 67 6c 6f 	*unknown*
    11cc:	73 73 2f 6f 	*unknown*
    11d0:	72 31 6b 2f 	*unknown*
    11d4:	69 6e 63 6c 	*unknown*
    11d8:	75 64 65 00 	*unknown*
    11dc:	2f 68 6f 6d 	*unknown*
    11e0:	65 2f 61 6e 	*unknown*
    11e4:	64 72 7a 65 	*unknown*
    11e8:	6a 72 2f 44 	*unknown*
    11ec:	69 67 69 74 	*unknown*
    11f0:	61 6c 2f 4f 	*unknown*
    11f4:	52 50 53 6f 	*unknown*
    11f8:	43 2f 6e 65 	*unknown*
    11fc:	77 6c 69 62 	*unknown*
    1200:	2d 32 2e 32 	*unknown*
    1204:	2e 30 2e 32 	*unknown*
    1208:	30 31 35 30 	*unknown*
    120c:	38 32 34 2f 	*unknown*
    1210:	6e 65 77 6c 	l.lwa r19,30572(r5)
    1214:	69 62 2f 6c 	*unknown*
    1218:	69 62 63 2f 	*unknown*
    121c:	69 6e 63 6c 	*unknown*
    1220:	75 64 65 2f 	*unknown*
    1224:	6d 61 63 68 	l.lwa r11,25448(r1)
    1228:	69 6e 65 00 	*unknown*
    122c:	2f 68 6f 6d 	*unknown*
    1230:	65 2f 61 6e 	*unknown*
    1234:	64 72 7a 65 	*unknown*
    1238:	6a 72 2f 44 	*unknown*
    123c:	69 67 69 74 	*unknown*
    1240:	61 6c 2f 4f 	*unknown*
    1244:	52 50 53 6f 	*unknown*
    1248:	43 2f 6e 65 	*unknown*
    124c:	77 6c 69 62 	*unknown*
    1250:	2d 32 2e 32 	*unknown*
    1254:	2e 30 2e 32 	*unknown*
    1258:	30 31 35 30 	*unknown*
    125c:	38 32 34 2f 	*unknown*
    1260:	6e 65 77 6c 	l.lwa r19,30572(r5)
    1264:	69 62 2f 6c 	*unknown*
    1268:	69 62 63 2f 	*unknown*
    126c:	69 6e 63 6c 	*unknown*
    1270:	75 64 65 2f 	*unknown*
    1274:	73 79 73 00 	*unknown*
    1278:	00 74 69 6d 	l.j 1d1b82c <_end+0x1d16dc0>
    127c:	65 72 2e 63 	*unknown*
    1280:	00 01 00 00 	l.j 41280 <_end+0x3c814>
    1284:	6f 72 31 6b 	l.lwa r27,12651(r18)
    1288:	2d 73 75 70 	*unknown*
    128c:	70 6f 72 74 	*unknown*
    1290:	2e 68 00 02 	*unknown*
    1294:	00 00 5f 64 	l.j 19024 <_end+0x145b8>
    1298:	65 66 61 75 	*unknown*
    129c:	6c 74 5f 74 	l.lwa r3,24436(r20)
    12a0:	79 70 65 73 	*unknown*
    12a4:	2e 68 00 03 	*unknown*
    12a8:	00 00 5f 73 	l.j 19074 <_end+0x14608>
    12ac:	74 64 69 6e 	*unknown*
    12b0:	74 2e 68 00 	*unknown*
    12b4:	04 00 00 6f 	l.jal 1470 <_or1k_reset+0x1370>
    12b8:	72 31 6b 2d 	*unknown*
    12bc:	69 6e 74 65 	*unknown*
    12c0:	72 6e 61 6c 	*unknown*
    12c4:	73 2e 68 00 	*unknown*
    12c8:	01 00 00 62 	l.j 4001450 <_end+0x3ffc9e4>
    12cc:	6f 61 72 64 	l.lwa r27,29284(r1)
    12d0:	2e 68 00 01 	*unknown*
    12d4:	00 00 00 00 	l.j 12d4 <_or1k_reset+0x11d4>
    12d8:	05 02 00 00 	l.jal 40812d8 <_end+0x407c86c>
    12dc:	33 84 03 20 	*unknown*
    12e0:	01 13 1f 21 	l.j 44c8f64 <_end+0x44c44f8>
    12e4:	1f 21 1f 21 	*unknown*
    12e8:	04 02 03 e2 	l.jal 82270 <_end+0x7d804>
    12ec:	01 20 04 01 	l.j 48022f0 <_end+0x47fd884>
    12f0:	03 9e 7e 20 	l.j fe7a0b70 <_end+0xfe79c104>
    12f4:	04 02 03 e2 	l.jal 8227c <_end+0x7d810>
    12f8:	01 20 04 01 	l.j 48022fc <_end+0x47fd890>
    12fc:	03 a2 7e 20 	l.j fe8a0b7c <_end+0xfe89c110>
    1300:	04 02 03 d1 	l.jal 82244 <_end+0x7d7d8>
    1304:	01 58 04 01 	l.j 5602308 <_end+0x55fd89c>
    1308:	03 b1 7e 20 	l.j fec60b88 <_end+0xfec5c11c>
    130c:	03 0b 4a 04 	l.j fc2d3b1c <_end+0xfc2cf0b0>
    1310:	02 03 d1 01 	l.j f80f5714 <_end+0xf80f0ca8>
    1314:	58 04 01 03 	*unknown*
    1318:	af 7e 20 04 	l.xori r27,r30,8196
    131c:	02 03 d1 01 	l.j f80f5720 <_end+0xf80f0cb4>
    1320:	20 04 01 03 	*unknown*
    1324:	b1 7e 20 6b 	l.muli r11,r30,8299
    1328:	21 1f 4b 1f 	*unknown*
    132c:	04 02 03 bd 	l.jal 82220 <_end+0x7d7b4>
    1330:	01 20 04 01 	l.j 4802334 <_end+0x47fd8c8>
    1334:	03 c4 7e 20 	l.j ff120bb4 <_end+0xff11c148>
    1338:	04 02 03 bc 	l.jal 82228 <_end+0x7d7bc>
    133c:	01 20 04 01 	l.j 4802340 <_end+0x47fd8d4>
    1340:	03 cb 7e 20 	l.j ff2e0bc0 <_end+0xff2dc154>
    1344:	1d 23 2b 23 	*unknown*
    1348:	04 02 03 b5 	l.jal 8221c <_end+0x7d7b0>
    134c:	01 2e 04 01 	l.j 4b82350 <_end+0x4b7d8e4>
    1350:	03 cc 7e 20 	l.j ff320bd0 <_end+0xff31c164>
    1354:	04 02 03 b4 	l.jal 82224 <_end+0x7d7b8>
    1358:	01 20 04 01 	l.j 480235c <_end+0x47fd8f0>
    135c:	03 d1 7e 20 	l.j ff460bdc <_end+0xff45c170>
    1360:	21 03 6d 74 	*unknown*
    1364:	03 17 2e 13 	l.j fc5ccbb0 <_end+0xfc5c8144>
    1368:	2d 21 2d 2f 	*unknown*
    136c:	1f 21 04 02 	*unknown*
    1370:	03 b6 01 2e 	l.j fed81828 <_end+0xfed7cdbc>
    1374:	04 01 03 ca 	l.jal 4229c <_end+0x3d830>
    1378:	7e 20 04 02 	*unknown*
    137c:	03 b6 01 20 	l.j fed817fc <_end+0xfed7cd90>
    1380:	04 01 03 cc 	l.jal 422b0 <_end+0x3d844>
    1384:	7e 20 04 02 	*unknown*
    1388:	03 a7 01 3c 	l.j fe9c1878 <_end+0xfe9bce0c>
    138c:	04 01 03 db 	l.jal 422f8 <_end+0x3d88c>
    1390:	7e 20 3d 5c 	*unknown*
    1394:	4b 2f 4e 14 	*unknown*
    1398:	1e 22 1e 04 	*unknown*
    139c:	02 03 a7 01 	l.j f80eafa0 <_end+0xf80e6534>
    13a0:	20 04 01 03 	*unknown*
    13a4:	d9 7e 20 22 	l.sb 22562(r30),r4
    13a8:	04 02 03 a5 	l.jal 8223c <_end+0x7d7d0>
    13ac:	01 20 04 01 	l.j 48023b0 <_end+0x47fd944>
    13b0:	03 df 7e 20 	l.j ff7e0c30 <_end+0xff7dc1c4>
    13b4:	4b 04 02 03 	*unknown*
    13b8:	93 01 4a 04 	l.lbs r24,18948(r1)
    13bc:	01 03 f0 7e 	l.j 40fd5b4 <_end+0x40f8b48>
    13c0:	20 03 0a 4a 	*unknown*
    13c4:	04 02 03 93 	l.jal 82210 <_end+0x7d7a4>
    13c8:	01 2e 04 01 	l.j 4b823cc <_end+0x4b7d960>
    13cc:	03 ed 7e 20 	l.j ffb60c4c <_end+0xffb5c1e0>
    13d0:	04 02 03 93 	l.jal 8221c <_end+0x7d7b0>
    13d4:	01 20 04 01 	l.j 48023d8 <_end+0x47fd96c>
    13d8:	03 f0 7e 20 	l.j ffc20c58 <_end+0xffc1c1ec>
    13dc:	04 02 03 83 	l.jal 821e8 <_end+0x7d77c>
    13e0:	01 9e 03 0d 	l.j 6782014 <_end+0x677d5a8>
    13e4:	20 04 01 03 	*unknown*
    13e8:	f4 7e 2e 04 	*unknown*
    13ec:	02 03 ff 00 	l.j f8100fec <_end+0xf80fc580>
    13f0:	20 04 01 03 	*unknown*
    13f4:	83 7f 20 03 	*unknown*
    13f8:	0a 4a 04 02 	*unknown*
    13fc:	03 80 01 2e 	l.j fe0018b4 <_end+0xfdffce48>
    1400:	04 01 03 80 	l.jal 42200 <_end+0x3d794>
    1404:	7f 20 04 02 	*unknown*
    1408:	03 80 01 20 	l.j fe001888 <_end+0xfdffce1c>
    140c:	04 01 03 82 	l.jal 42214 <_end+0x3d7a8>
    1410:	7f 20 04 02 	*unknown*
    1414:	03 f1 00 2e 	l.j ffc414cc <_end+0xffc3ca60>
    1418:	04 01 03 92 	l.jal 42260 <_end+0x3d7f4>
    141c:	7f 20 1f 21 	*unknown*
    1420:	4e 04 02 03 	*unknown*
    1424:	f7 00 20 04 	*unknown*
    1428:	01 03 89 7f 	l.j 40e3a24 <_end+0x40defb8>
    142c:	20 04 02 03 	*unknown*
    1430:	f7 00 20 04 	*unknown*
    1434:	01 03 8b 7f 	l.j 40e4230 <_end+0x40df7c4>
    1438:	20 04 02 03 	*unknown*
    143c:	e8 00 20 04 	*unknown*
    1440:	01 03 9a 7f 	l.j 40e7e3c <_end+0x40e33d0>
    1444:	20 40 04 02 	*unknown*
    1448:	03 ef 00 2e 	l.j ffbc1500 <_end+0xffbbca94>
    144c:	04 01 03 91 	l.jal 42290 <_end+0x3d824>
    1450:	7f 20 04 02 	*unknown*
    1454:	03 ef 00 20 	l.j ffbc14d4 <_end+0xffbbca68>
    1458:	04 01 03 93 	l.jal 422a4 <_end+0x3d838>
    145c:	7f 20 04 02 	*unknown*
    1460:	03 e0 00 3c 	l.j ff801550 <_end+0xff7fcae4>
    1464:	04 01 03 a2 	l.jal 422ec <_end+0x3d880>
    1468:	7f 20 4e 04 	*unknown*
    146c:	02 03 e7 00 	l.j f80fb06c <_end+0xf80f6600>
    1470:	2e 04 01 03 	*unknown*
    1474:	99 7f 20 04 	l.lhs r11,8196(r31)
    1478:	02 03 e7 00 	l.j f80fb078 <_end+0xf80f660c>
    147c:	20 04 01 03 	*unknown*
    1480:	9b 7f 20 04 	l.lhs r27,8196(r31)
    1484:	02 03 d8 00 	l.j f80f7484 <_end+0xf80f2a18>
    1488:	3c 04 01 03 	*unknown*
    148c:	ab 7f 4a 03 	l.ori r27,r31,0x4a03
    1490:	0a 4a 13 1f 	*unknown*
    1494:	21 1f 22 03 	*unknown*
    1498:	0a 4a 13 1f 	*unknown*
    149c:	21 2d 2f 21 	*unknown*
    14a0:	02 04 00 01 	l.j f81014a4 <_end+0xf80fca38>
    14a4:	01 00 00 01 	l.j 40014a8 <_end+0x3ffca3c>
    14a8:	1d 00 02 00 	*unknown*
    14ac:	00 01 06 04 	l.j 42cbc <_end+0x3e250>
    14b0:	01 fb 0e 0d 	l.j 7ec4ce4 <_end+0x7ec0278>
    14b4:	00 01 01 01 	l.j 418b8 <_end+0x3ce4c>
    14b8:	01 00 00 00 	l.j 40014b8 <_end+0x3ffca4c>
    14bc:	01 00 00 01 	l.j 40014c0 <_end+0x3ffca54>
    14c0:	2e 2e 2f 2e 	*unknown*
    14c4:	2e 2f 2e 2e 	*unknown*
    14c8:	2f 2e 2e 2f 	*unknown*
    14cc:	2e 2e 2f 6e 	*unknown*
    14d0:	65 77 6c 69 	*unknown*
    14d4:	62 2d 32 2e 	*unknown*
    14d8:	32 2e 30 2e 	*unknown*
    14dc:	32 30 31 35 	*unknown*
    14e0:	30 38 32 34 	*unknown*
    14e4:	2f 6e 65 77 	*unknown*
    14e8:	6c 69 62 2f 	l.lwa r3,25135(r9)
    14ec:	6c 69 62 63 	l.lwa r3,25187(r9)
    14f0:	2f 65 72 72 	*unknown*
    14f4:	6e 6f 00 2f 	l.lwa r19,47(r15)
    14f8:	68 6f 6d 65 	*unknown*
    14fc:	2f 61 6e 64 	*unknown*
    1500:	72 7a 65 6a 	*unknown*
    1504:	72 2f 44 69 	*unknown*
    1508:	67 69 74 61 	*unknown*
    150c:	6c 2f 4f 52 	l.lwa r1,20306(r15)
    1510:	50 53 6f 43 	*unknown*
    1514:	2f 6e 65 77 	*unknown*
    1518:	6c 69 62 2d 	l.lwa r3,25133(r9)
    151c:	32 2e 32 2e 	*unknown*
    1520:	30 2e 32 30 	*unknown*
    1524:	31 35 30 38 	*unknown*
    1528:	32 34 2f 6e 	*unknown*
    152c:	65 77 6c 69 	*unknown*
    1530:	62 2f 6c 69 	*unknown*
    1534:	62 63 2f 69 	*unknown*
    1538:	6e 63 6c 75 	l.lwa r19,27765(r3)
    153c:	64 65 2f 73 	*unknown*
    1540:	79 73 00 2f 	*unknown*
    1544:	6f 70 74 2f 	l.lwa r27,29743(r16)
    1548:	6f 72 31 6b 	l.lwa r27,12651(r18)
    154c:	2d 65 6c 66 	*unknown*
    1550:	2f 6c 69 62 	*unknown*
    1554:	2f 67 63 63 	*unknown*
    1558:	2f 6f 72 31 	*unknown*
    155c:	6b 2d 65 6c 	*unknown*
    1560:	66 2f 34 2e 	*unknown*
    1564:	39 2e 32 2f 	*unknown*
    1568:	69 6e 63 6c 	*unknown*
    156c:	75 64 65 00 	*unknown*
    1570:	00 65 72 72 	l.j 195df38 <_end+0x19594cc>
    1574:	6e 6f 2e 63 	l.lwa r19,11875(r15)
    1578:	00 01 00 00 	l.j 41578 <_end+0x3cb0c>
    157c:	6c 6f 63 6b 	l.lwa r3,25451(r15)
    1580:	2e 68 00 02 	*unknown*
    1584:	00 00 5f 74 	l.j 19354 <_end+0x148e8>
    1588:	79 70 65 73 	*unknown*
    158c:	2e 68 00 02 	*unknown*
    1590:	00 00 73 74 	l.j 1e360 <_end+0x198f4>
    1594:	64 64 65 66 	*unknown*
    1598:	2e 68 00 03 	*unknown*
    159c:	00 00 72 65 	l.j 1df30 <_end+0x194c4>
    15a0:	65 6e 74 2e 	*unknown*
    15a4:	68 00 02 00 	*unknown*
    15a8:	00 65 72 72 	l.j 195df70 <_end+0x1959504>
    15ac:	6e 6f 2e 68 	l.lwa r19,11880(r15)
    15b0:	00 02 00 00 	l.j 815b0 <_end+0x7cb44>
    15b4:	00 00 05 02 	l.j 29bc <_read_r+0x10>
    15b8:	00 00 37 1c 	l.j f228 <_end+0xa7bc>
    15bc:	03 0b 01 2f 	l.j fc2c1a78 <_end+0xfc2bd00c>
    15c0:	2f 02 04 00 	*unknown*
    15c4:	01 01 00 00 	l.j 40415c4 <_end+0x403cb58>
    15c8:	01 2d 00 02 	l.j 4b415d0 <_end+0x4b3cb64>
    15cc:	00 00 00 e4 	l.j 195c <_or1k_reset+0x185c>
    15d0:	04 01 fb 0e 	l.jal 80208 <_end+0x7b79c>
    15d4:	0d 00 01 01 	l.bnf 40019d8 <_end+0x3ffcf6c>
    15d8:	01 01 00 00 	l.j 40415d8 <_end+0x403cb6c>
    15dc:	00 01 00 00 	l.j 415dc <_end+0x3cb70>
    15e0:	01 2e 2e 2f 	l.j 4b8ce9c <_end+0x4b88430>
    15e4:	2e 2e 2f 2e 	*unknown*
    15e8:	2e 2f 2e 2e 	*unknown*
    15ec:	2f 2e 2e 2f 	*unknown*
    15f0:	6e 65 77 6c 	l.lwa r19,30572(r5)
    15f4:	69 62 2d 32 	*unknown*
    15f8:	2e 32 2e 30 	*unknown*
    15fc:	2e 32 30 31 	*unknown*
    1600:	35 30 38 32 	*unknown*
    1604:	34 2f 6e 65 	*unknown*
    1608:	77 6c 69 62 	*unknown*
    160c:	2f 6c 69 62 	*unknown*
    1610:	63 2f 73 74 	*unknown*
    1614:	72 69 6e 67 	*unknown*
    1618:	00 2f 6f 70 	l.j bdd3d8 <_end+0xbd896c>
    161c:	74 2f 6f 72 	*unknown*
    1620:	31 6b 2d 65 	*unknown*
    1624:	6c 66 2f 6c 	l.lwa r3,12140(r6)
    1628:	69 62 2f 67 	*unknown*
    162c:	63 63 2f 6f 	*unknown*
    1630:	72 31 6b 2d 	*unknown*
    1634:	65 6c 66 2f 	*unknown*
    1638:	34 2e 39 2e 	*unknown*
    163c:	32 2f 69 6e 	*unknown*
    1640:	63 6c 75 64 	*unknown*
    1644:	65 00 2f 68 	*unknown*
    1648:	6f 6d 65 2f 	l.lwa r27,25903(r13)
    164c:	61 6e 64 72 	*unknown*
    1650:	7a 65 6a 72 	*unknown*
    1654:	2f 44 69 67 	*unknown*
    1658:	69 74 61 6c 	*unknown*
    165c:	2f 4f 52 50 	*unknown*
    1660:	53 6f 43 2f 	*unknown*
    1664:	6e 65 77 6c 	l.lwa r19,30572(r5)
    1668:	69 62 2d 32 	*unknown*
    166c:	2e 32 2e 30 	*unknown*
    1670:	2e 32 30 31 	*unknown*
    1674:	35 30 38 32 	*unknown*
    1678:	34 2f 6e 65 	*unknown*
    167c:	77 6c 69 62 	*unknown*
    1680:	2f 6c 69 62 	*unknown*
    1684:	63 2f 69 6e 	*unknown*
    1688:	63 6c 75 64 	*unknown*
    168c:	65 00 00 6d 	*unknown*
    1690:	65 6d 73 65 	*unknown*
    1694:	74 2e 63 00 	*unknown*
    1698:	01 00 00 73 	l.j 4001864 <_end+0x3ffcdf8>
    169c:	74 64 64 65 	*unknown*
    16a0:	66 2e 68 00 	*unknown*
    16a4:	02 00 00 73 	l.j f8001870 <_end+0xf7ffce04>
    16a8:	74 72 69 6e 	*unknown*
    16ac:	67 2e 68 00 	*unknown*
    16b0:	03 00 00 00 	l.j fc0016b0 <_end+0xfbffcc44>
    16b4:	00 05 02 00 	l.j 141eb4 <_end+0x13d448>
    16b8:	00 37 44 03 	l.j dd26c4 <_end+0xdcdc58>
    16bc:	30 01 03 0a 	*unknown*
    16c0:	01 03 76 20 	l.j 40def40 <_end+0x40da4d4>
    16c4:	03 0a 2e 20 	l.j fc28cf44 <_end+0xfc2884d8>
    16c8:	30 ad 2b 22 	*unknown*
    16cc:	1e 44 03 75 	*unknown*
    16d0:	3c 03 18 20 	*unknown*
    16d4:	03 7a 20 2f 	l.j fde89790 <_end+0xfde84d24>
    16d8:	25 ae 21 2f 	*unknown*
    16dc:	21 1b 03 09 	*unknown*
    16e0:	82 03 77 4a 	*unknown*
    16e4:	03 0c 20 1f 	l.j fc309760 <_end+0xfc304cf4>
    16e8:	1e 03 0b 90 	*unknown*
    16ec:	67 2d 40 03 	*unknown*
    16f0:	4a 58 02 02 	*unknown*
    16f4:	Address 0x00000000000016f4 is out of bounds.


Disassembly of section .debug_frame:

00000000 <.debug_frame>:
   0:	00 00 00 0c 	l.j 30 <_or1k_reset-0xd0>
   4:	ff ff ff ff 	*unknown*
   8:	03 00 01 7c 	l.j fc0005f8 <_end+0xfbffbb8c>
   c:	09 0c 01 00 	*unknown*
  10:	00 00 00 24 	l.j a0 <_or1k_reset-0x60>
  14:	00 00 00 00 	l.j 14 <_or1k_reset-0xec>
  18:	00 00 21 40 	l.j 8518 <_end+0x3aac>
  1c:	00 00 00 5c 	l.j 18c <_or1k_reset+0x8c>
  20:	04 00 00 00 	l.jal 20 <_or1k_reset-0xe0>
  24:	08 82 02 04 	*unknown*
  28:	00 00 00 1c 	l.j 98 <_or1k_reset-0x68>
  2c:	89 01 81 03 	l.lws r8,-32509(r1)
  30:	04 00 00 00 	l.jal 30 <_or1k_reset-0xd0>
  34:	08 0e 0c 00 	*unknown*
  38:	00 00 00 24 	l.j c8 <_or1k_reset-0x38>
  3c:	00 00 00 00 	l.j 3c <_or1k_reset-0xc4>
  40:	00 00 21 9c 	l.j 86b0 <_end+0x3c44>
  44:	00 00 00 6c 	l.j 1f4 <_or1k_reset+0xf4>
  48:	04 00 00 00 	l.jal 48 <_or1k_reset-0xb8>
  4c:	08 82 02 04 	*unknown*
  50:	00 00 00 1c 	l.j c0 <_or1k_reset-0x40>
  54:	89 01 81 03 	l.lws r8,-32509(r1)
  58:	04 00 00 00 	l.jal 58 <_or1k_reset-0xa8>
  5c:	18 0e 0c 00 	*unknown*
  60:	00 00 00 2c 	l.j 110 <_or1k_reset+0x10>
  64:	00 00 00 00 	l.j 64 <_or1k_reset-0x9c>
  68:	00 00 22 08 	l.j 8888 <_end+0x3e1c>
  6c:	00 00 00 d8 	l.j 3cc <_or1k_reset+0x2cc>
  70:	04 00 00 00 	l.jal 70 <_or1k_reset-0x90>
  74:	04 92 02 04 	l.jal 2480884 <_end+0x247be18>
  78:	00 00 00 10 	l.j b8 <_or1k_reset-0x48>
  7c:	82 04 89 01 	*unknown*
  80:	04 00 00 00 	l.jal 80 <_or1k_reset-0x80>
  84:	10 81 05 8e 	l.bf 20416bc <_end+0x203cc50>
  88:	03 04 00 00 	l.j fc100088 <_end+0xfc0fb61c>
  8c:	00 08 0e 14 	l.j 2038dc <_end+0x1fee70>
  90:	00 00 00 1c 	l.j 100 <_or1k_reset>
  94:	00 00 00 00 	l.j 94 <_or1k_reset-0x6c>
  98:	00 00 22 e0 	l.j 8c18 <_end+0x41ac>
  9c:	00 00 00 1c 	l.j 10c <_or1k_reset+0xc>
  a0:	04 00 00 00 	l.jal a0 <_or1k_reset-0x60>
  a4:	08 89 01 81 	*unknown*
  a8:	02 04 00 00 	l.j f81000a8 <_end+0xf80fb63c>
  ac:	00 04 0e 08 	l.j 1038cc <_end+0xfee60>
  b0:	00 00 00 1c 	l.j 120 <_or1k_reset+0x20>
  b4:	00 00 00 00 	l.j b4 <_or1k_reset-0x4c>
  b8:	00 00 22 fc 	l.j 8ca8 <_end+0x423c>
  bc:	00 00 00 80 	l.j 2bc <_or1k_reset+0x1bc>
  c0:	04 00 00 00 	l.jal c0 <_or1k_reset-0x40>
  c4:	14 89 01 81 	*unknown*
  c8:	02 04 00 00 	l.j f81000c8 <_end+0xf80fb65c>
  cc:	00 08 0e 08 	l.j 2038ec <_end+0x1fee80>
  d0:	00 00 00 1c 	l.j 140 <_or1k_reset+0x40>
  d4:	00 00 00 00 	l.j d4 <_or1k_reset-0x2c>
  d8:	00 00 23 7c 	l.j 8ec8 <_end+0x445c>
  dc:	00 00 00 1c 	l.j 14c <_or1k_reset+0x4c>
  e0:	04 00 00 00 	l.jal e0 <_or1k_reset-0x20>
  e4:	08 89 01 81 	*unknown*
  e8:	02 04 00 00 	l.j f81000e8 <_end+0xf80fb67c>
  ec:	00 04 0e 08 	l.j 10390c <_end+0xfeea0>
  f0:	00 00 00 0c 	l.j 120 <_or1k_reset+0x20>
  f4:	ff ff ff ff 	*unknown*
  f8:	01 00 04 7c 	l.j 40012e8 <_end+0x3ffc87c>
  fc:	09 0d 01 00 	*unknown*
 100:	00 00 00 14 	l.j 150 <_or1k_reset+0x50>
 104:	00 00 00 f0 	l.j 4c4 <_or1k_reset+0x3c4>
 108:	00 00 23 e4 	l.j 9098 <_end+0x462c>
 10c:	00 00 00 30 	l.j 1cc <_or1k_reset+0xcc>
 110:	45 89 01 81 	*unknown*
 114:	02 41 0e 08 	l.j f9043934 <_end+0xf903eec8>
 118:	00 00 00 0c 	l.j 148 <_or1k_reset+0x48>
 11c:	ff ff ff ff 	*unknown*
 120:	01 00 04 7c 	l.j 4001310 <_end+0x3ffc8a4>
 124:	09 0d 01 00 	*unknown*
 128:	00 00 00 18 	l.j 188 <_or1k_reset+0x88>
 12c:	00 00 01 18 	l.j 58c <_or1k_reset+0x48c>
 130:	00 00 24 14 	l.j 9180 <_end+0x4714>
 134:	00 00 00 48 	l.j 254 <_or1k_reset+0x154>
 138:	44 82 02 89 	*unknown*
 13c:	01 81 03 41 	l.j 6040e40 <_end+0x603c3d4>
 140:	0e 0c 00 00 	l.bnf f8300140 <_end+0xf82fb6d4>
 144:	00 00 00 0c 	l.j 174 <_or1k_reset+0x74>
 148:	ff ff ff ff 	*unknown*
 14c:	01 00 04 7c 	l.j 400133c <_end+0x3ffc8d0>
 150:	09 0d 01 00 	*unknown*
 154:	00 00 00 20 	l.j 1d4 <_or1k_reset+0xd4>
 158:	00 00 01 44 	l.j 668 <_or1k_reset+0x568>
 15c:	00 00 24 5c 	l.j 92cc <_end+0x4860>
 160:	00 00 01 40 	l.j 660 <_or1k_reset+0x560>
 164:	41 82 06 44 	*unknown*
 168:	8e 05 92 04 	l.lbz r16,-28156(r5)
 16c:	47 94 03 96 	*unknown*
 170:	02 89 01 81 	l.j fa240774 <_end+0xfa23bd08>
 174:	07 41 0e 1c 	l.jal fd0439e4 <_end+0xfd03ef78>
 178:	00 00 00 0c 	l.j 1a8 <_or1k_reset+0xa8>
 17c:	ff ff ff ff 	*unknown*
 180:	01 00 04 7c 	l.j 4001370 <_end+0x3ffc904>
 184:	09 0d 01 00 	*unknown*
 188:	00 00 00 28 	l.j 228 <_or1k_reset+0x128>
 18c:	00 00 01 78 	l.j 76c <_or1k_reset+0x66c>
 190:	00 00 25 9c 	l.j 9800 <_end+0x4d94>
 194:	00 00 01 e8 	l.j 934 <_or1k_reset+0x834>
 198:	41 82 0a 44 	*unknown*
 19c:	9e 02 92 08 	l.addi r16,r2,-28152
 1a0:	4a 9a 04 9c 	*unknown*
 1a4:	03 89 01 81 	l.j fe2407a8 <_end+0xfe23bd3c>
 1a8:	0b 8e 09 94 	*unknown*
 1ac:	07 96 06 98 	l.jal fe581c0c <_end+0xfe57d1a0>
 1b0:	05 41 0e 30 	l.jal 5043a70 <_end+0x503f004>
 1b4:	00 00 00 0c 	l.j 1e4 <_or1k_reset+0xe4>
 1b8:	ff ff ff ff 	*unknown*
 1bc:	01 00 04 7c 	l.j 40013ac <_end+0x3ffc940>
 1c0:	09 0d 01 00 	*unknown*
 1c4:	00 00 00 1c 	l.j 234 <_or1k_reset+0x134>
 1c8:	00 00 01 b4 	l.j 898 <_or1k_reset+0x798>
 1cc:	00 00 27 84 	l.j 9fdc <_end+0x5570>
 1d0:	00 00 00 98 	l.j 430 <_or1k_reset+0x330>
 1d4:	46 82 04 8e 	*unknown*
 1d8:	03 92 02 89 	l.j fe480bfc <_end+0xfe47c190>
 1dc:	01 81 05 41 	l.j 60416e0 <_end+0x603cc74>
 1e0:	0e 14 00 00 	l.bnf f85001e0 <_end+0xf84fb774>
 1e4:	00 00 00 14 	l.j 234 <_or1k_reset+0x134>
 1e8:	00 00 01 b4 	l.j 8b8 <_or1k_reset+0x7b8>
 1ec:	00 00 28 1c 	l.j a25c <_end+0x57f0>
 1f0:	00 00 00 18 	l.j 250 <_or1k_reset+0x150>
 1f4:	42 89 01 81 	*unknown*
 1f8:	02 42 0e 08 	l.j f9083a18 <_end+0xf907efac>
 1fc:	00 00 00 14 	l.j 24c <_or1k_reset+0x14c>
 200:	00 00 01 b4 	l.j 8d0 <_or1k_reset+0x7d0>
 204:	00 00 28 34 	l.j a2d4 <_end+0x5868>
 208:	00 00 00 20 	l.j 288 <_or1k_reset+0x188>
 20c:	41 81 01 41 	*unknown*
 210:	0e 04 00 00 	l.bnf f8100210 <_end+0xf80fb7a4>
 214:	00 00 00 14 	l.j 264 <_or1k_reset+0x164>
 218:	00 00 01 b4 	l.j 8e8 <_or1k_reset+0x7e8>
 21c:	00 00 28 54 	l.j a36c <_end+0x5900>
 220:	00 00 00 20 	l.j 2a0 <_or1k_reset+0x1a0>
 224:	41 81 01 41 	*unknown*
 228:	0e 04 00 00 	l.bnf f8100228 <_end+0xf80fb7bc>
 22c:	00 00 00 14 	l.j 27c <_or1k_reset+0x17c>
 230:	00 00 01 b4 	l.j 900 <_or1k_reset+0x800>
 234:	00 00 28 74 	l.j a404 <_end+0x5998>
 238:	00 00 00 2c 	l.j 2e8 <_or1k_reset+0x1e8>
 23c:	42 89 01 81 	*unknown*
 240:	02 42 0e 08 	l.j f9083a60 <_end+0xf907eff4>
 244:	00 00 00 14 	l.j 294 <_or1k_reset+0x194>
 248:	00 00 01 b4 	l.j 918 <_or1k_reset+0x818>
 24c:	00 00 28 a0 	l.j a4cc <_end+0x5a60>
 250:	00 00 00 20 	l.j 2d0 <_or1k_reset+0x1d0>
 254:	41 81 01 41 	*unknown*
 258:	0e 04 00 00 	l.bnf f8100258 <_end+0xf80fb7ec>
 25c:	00 00 00 14 	l.j 2ac <_or1k_reset+0x1ac>
 260:	00 00 01 b4 	l.j 930 <_or1k_reset+0x830>
 264:	00 00 28 c0 	l.j a564 <_end+0x5af8>
 268:	00 00 00 20 	l.j 2e8 <_or1k_reset+0x1e8>
 26c:	41 81 01 41 	*unknown*
 270:	0e 04 00 00 	l.bnf f8100270 <_end+0xf80fb804>
 274:	00 00 00 14 	l.j 2c4 <_or1k_reset+0x1c4>
 278:	00 00 01 b4 	l.j 948 <_or1k_reset+0x848>
 27c:	00 00 28 e0 	l.j a5fc <_end+0x5b90>
 280:	00 00 00 20 	l.j 300 <_or1k_reset+0x200>
 284:	41 81 01 41 	*unknown*
 288:	0e 04 00 00 	l.bnf f8100288 <_end+0xf80fb81c>
 28c:	00 00 00 14 	l.j 2dc <_or1k_reset+0x1dc>
 290:	00 00 01 b4 	l.j 960 <_or1k_reset+0x860>
 294:	00 00 29 00 	l.j a694 <_end+0x5c28>
 298:	00 00 00 20 	l.j 318 <_or1k_reset+0x218>
 29c:	41 81 01 41 	*unknown*
 2a0:	0e 04 00 00 	l.bnf f81002a0 <_end+0xf80fb834>
 2a4:	00 00 00 14 	l.j 2f4 <_or1k_reset+0x1f4>
 2a8:	00 00 01 b4 	l.j 978 <_or1k_reset+0x878>
 2ac:	00 00 29 20 	l.j a72c <_end+0x5cc0>
 2b0:	00 00 00 20 	l.j 330 <_or1k_reset+0x230>
 2b4:	41 81 01 41 	*unknown*
 2b8:	0e 04 00 00 	l.bnf f81002b8 <_end+0xf80fb84c>
 2bc:	00 00 00 14 	l.j 30c <_or1k_reset+0x20c>
 2c0:	00 00 01 b4 	l.j 990 <_or1k_reset+0x890>
 2c4:	00 00 29 40 	l.j a7c4 <_end+0x5d58>
 2c8:	00 00 00 20 	l.j 348 <_or1k_reset+0x248>
 2cc:	41 81 01 41 	*unknown*
 2d0:	0e 04 00 00 	l.bnf f81002d0 <_end+0xf80fb864>
 2d4:	00 00 00 14 	l.j 324 <_or1k_reset+0x224>
 2d8:	00 00 01 b4 	l.j 9a8 <_or1k_reset+0x8a8>
 2dc:	00 00 29 60 	l.j a85c <_end+0x5df0>
 2e0:	00 00 00 2c 	l.j 390 <_or1k_reset+0x290>
 2e4:	42 89 01 81 	*unknown*
 2e8:	02 42 0e 08 	l.j f9083b08 <_end+0xf907f09c>
 2ec:	00 00 00 14 	l.j 33c <_or1k_reset+0x23c>
 2f0:	00 00 01 b4 	l.j 9c0 <_or1k_reset+0x8c0>
 2f4:	00 00 29 8c 	l.j a924 <_end+0x5eb8>
 2f8:	00 00 00 20 	l.j 378 <_or1k_reset+0x278>
 2fc:	41 81 01 41 	*unknown*
 300:	0e 04 00 00 	l.bnf f8100300 <_end+0xf80fb894>
 304:	00 00 00 14 	l.j 354 <_or1k_reset+0x254>
 308:	00 00 01 b4 	l.j 9d8 <_or1k_reset+0x8d8>
 30c:	00 00 29 ac 	l.j a9bc <_end+0x5f50>
 310:	00 00 00 20 	l.j 390 <_or1k_reset+0x290>
 314:	41 81 01 41 	*unknown*
 318:	0e 04 00 00 	l.bnf f8100318 <_end+0xf80fb8ac>
 31c:	00 00 00 14 	l.j 36c <_or1k_reset+0x26c>
 320:	00 00 01 b4 	l.j 9f0 <_or1k_reset+0x8f0>
 324:	00 00 29 cc 	l.j aa54 <_end+0x5fe8>
 328:	00 00 00 20 	l.j 3a8 <_or1k_reset+0x2a8>
 32c:	41 81 01 41 	*unknown*
 330:	0e 04 00 00 	l.bnf f8100330 <_end+0xf80fb8c4>
 334:	00 00 00 14 	l.j 384 <_or1k_reset+0x284>
 338:	00 00 01 b4 	l.j a08 <_or1k_reset+0x908>
 33c:	00 00 29 ec 	l.j aaec <_end+0x6080>
 340:	00 00 00 20 	l.j 3c0 <_or1k_reset+0x2c0>
 344:	41 81 01 41 	*unknown*
 348:	0e 04 00 00 	l.bnf f8100348 <_end+0xf80fb8dc>
 34c:	00 00 00 14 	l.j 39c <_or1k_reset+0x29c>
 350:	00 00 01 b4 	l.j a20 <_or1k_reset+0x920>
 354:	00 00 2a 0c 	l.j ab84 <_end+0x6118>
 358:	00 00 00 20 	l.j 3d8 <_or1k_reset+0x2d8>
 35c:	41 81 01 41 	*unknown*
 360:	0e 04 00 00 	l.bnf f8100360 <_end+0xf80fb8f4>
 364:	00 00 00 0c 	l.j 394 <_or1k_reset+0x294>
 368:	ff ff ff ff 	*unknown*
 36c:	01 00 04 7c 	l.j 400155c <_end+0x3ffcaf0>
 370:	09 0d 01 00 	*unknown*
 374:	00 00 00 14 	l.j 3c4 <_or1k_reset+0x2c4>
 378:	00 00 03 64 	l.j 1108 <_or1k_reset+0x1008>
 37c:	00 00 2a 2c 	l.j ac2c <_end+0x61c0>
 380:	00 00 00 28 	l.j 420 <_or1k_reset+0x320>
 384:	43 81 01 41 	*unknown*
 388:	0e 04 00 00 	l.bnf f8100388 <_end+0xf80fb91c>
 38c:	00 00 00 1c 	l.j 3fc <_or1k_reset+0x2fc>
 390:	00 00 03 64 	l.j 1120 <_or1k_reset+0x1020>
 394:	00 00 2a 54 	l.j ace4 <_end+0x6278>
 398:	00 00 00 d8 	l.j 6f8 <_or1k_reset+0x5f8>
 39c:	41 82 03 44 	*unknown*
 3a0:	8e 02 89 01 	l.lbz r16,-30463(r2)
 3a4:	43 81 04 42 	*unknown*
 3a8:	0e 10 00 00 	l.bnf f84003a8 <_end+0xf83fb93c>
 3ac:	00 00 00 14 	l.j 3fc <_or1k_reset+0x2fc>
 3b0:	00 00 03 64 	l.j 1140 <_or1k_reset+0x1040>
 3b4:	00 00 2b 2c 	l.j b064 <_end+0x65f8>
 3b8:	00 00 00 48 	l.j 4d8 <_or1k_reset+0x3d8>
 3bc:	46 81 01 41 	*unknown*
 3c0:	0e 04 00 00 	l.bnf f81003c0 <_end+0xf80fb954>
 3c4:	00 00 00 18 	l.j 424 <_or1k_reset+0x324>
 3c8:	00 00 03 64 	l.j 1158 <_or1k_reset+0x1058>
 3cc:	00 00 2b 74 	l.j b19c <_end+0x6730>
 3d0:	00 00 00 6c 	l.j 580 <_or1k_reset+0x480>
 3d4:	41 82 02 48 	*unknown*
 3d8:	89 01 81 03 	l.lws r8,-32509(r1)
 3dc:	41 0e 0c 00 	*unknown*
 3e0:	00 00 00 0c 	l.j 410 <_or1k_reset+0x310>
 3e4:	ff ff ff ff 	*unknown*
 3e8:	01 00 04 7c 	l.j 40015d8 <_end+0x3ffcb6c>
 3ec:	09 0d 01 00 	*unknown*
 3f0:	00 00 00 18 	l.j 450 <_or1k_reset+0x350>
 3f4:	00 00 03 e0 	l.j 1374 <_or1k_reset+0x1274>
 3f8:	00 00 2f 38 	l.j c0d8 <_end+0x766c>
 3fc:	00 00 00 40 	l.j 4fc <_or1k_reset+0x3fc>
 400:	41 82 01 46 	*unknown*
 404:	81 02 41 0e 	*unknown*
 408:	08 00 00 00 	*unknown*
 40c:	00 00 00 14 	l.j 45c <_or1k_reset+0x35c>
 410:	00 00 03 e0 	l.j 1390 <_or1k_reset+0x1290>
 414:	00 00 2f 78 	l.j c1f4 <_end+0x7788>
 418:	00 00 00 24 	l.j 4a8 <_or1k_reset+0x3a8>
 41c:	42 81 01 41 	*unknown*
 420:	0e 04 00 00 	l.bnf f8100420 <_end+0xf80fb9b4>
 424:	00 00 00 14 	l.j 474 <_or1k_reset+0x374>
 428:	00 00 03 e0 	l.j 13a8 <_or1k_reset+0x12a8>
 42c:	00 00 2f 9c 	l.j c29c <_end+0x7830>
 430:	00 00 00 38 	l.j 510 <_or1k_reset+0x410>
 434:	43 81 02 82 	*unknown*
 438:	01 41 0e 08 	l.j 5043c58 <_end+0x503f1ec>
 43c:	00 00 00 14 	l.j 48c <_or1k_reset+0x38c>
 440:	00 00 03 e0 	l.j 13c0 <_or1k_reset+0x12c0>
 444:	00 00 2f d4 	l.j c394 <_end+0x7928>
 448:	00 00 00 48 	l.j 568 <_or1k_reset+0x468>
 44c:	43 81 02 82 	*unknown*
 450:	01 41 0e 08 	l.j 5043c70 <_end+0x503f204>
 454:	00 00 00 0c 	l.j 484 <_or1k_reset+0x384>
 458:	ff ff ff ff 	*unknown*
 45c:	01 00 04 7c 	l.j 400164c <_end+0x3ffcbe0>
 460:	09 0d 01 00 	*unknown*
 464:	00 00 00 28 	l.j 504 <_or1k_reset+0x404>
 468:	00 00 04 54 	l.j 15b8 <_or1k_reset+0x14b8>
 46c:	00 00 30 e4 	l.j c7fc <_end+0x7d90>
 470:	00 00 01 78 	l.j a50 <_or1k_reset+0x950>
 474:	41 82 0a 43 	*unknown*
 478:	9e 02 4d 89 	l.addi r16,r2,19849
 47c:	01 8e 09 92 	l.j 6382ac4 <_end+0x637e058>
 480:	08 94 07 96 	*unknown*
 484:	06 98 05 9a 	l.jal fa601aec <_end+0xfa5fd080>
 488:	04 9c 03 81 	l.jal 270128c <_end+0x26fc820>
 48c:	0b 41 0e 2c 	*unknown*
 490:	00 00 00 14 	l.j 4e0 <_or1k_reset+0x3e0>
 494:	00 00 04 54 	l.j 15e4 <_or1k_reset+0x14e4>
 498:	00 00 32 5c 	l.j ce08 <_end+0x839c>
 49c:	00 00 00 20 	l.j 51c <_or1k_reset+0x41c>
 4a0:	43 81 01 41 	*unknown*
 4a4:	0e 04 00 00 	l.bnf f81004a4 <_end+0xf80fba38>
 4a8:	00 00 00 14 	l.j 4f8 <_or1k_reset+0x3f8>
 4ac:	00 00 04 54 	l.j 15fc <_or1k_reset+0x14fc>
 4b0:	00 00 32 7c 	l.j cea0 <_end+0x8434>
 4b4:	00 00 00 14 	l.j 504 <_or1k_reset+0x404>
 4b8:	41 81 01 41 	*unknown*
 4bc:	0e 04 00 00 	l.bnf f81004bc <_end+0xf80fba50>
 4c0:	00 00 00 0c 	l.j 4f0 <_or1k_reset+0x3f0>
 4c4:	ff ff ff ff 	*unknown*
 4c8:	01 00 04 7c 	l.j 40016b8 <_end+0x3ffcc4c>
 4cc:	09 0d 01 00 	*unknown*
 4d0:	00 00 00 18 	l.j 530 <_or1k_reset+0x430>
 4d4:	00 00 04 c0 	l.j 17d4 <_or1k_reset+0x16d4>
 4d8:	00 00 32 90 	l.j cf18 <_end+0x84ac>
 4dc:	00 00 00 4c 	l.j 60c <_or1k_reset+0x50c>
 4e0:	43 89 01 82 	*unknown*
 4e4:	02 81 03 41 	l.j fa0411e8 <_end+0xfa03c77c>
 4e8:	0e 0c 00 00 	l.bnf f83004e8 <_end+0xf82fba7c>
 4ec:	00 00 00 18 	l.j 54c <_or1k_reset+0x44c>
 4f0:	00 00 04 c0 	l.j 17f0 <_or1k_reset+0x16f0>
 4f4:	00 00 32 dc 	l.j d064 <_end+0x85f8>
 4f8:	00 00 00 38 	l.j 5d8 <_or1k_reset+0x4d8>
 4fc:	43 89 01 82 	*unknown*
 500:	02 81 03 42 	l.j fa041208 <_end+0xfa03c79c>
 504:	0e 0c 00 00 	l.bnf f8300504 <_end+0xf82fba98>
 508:	00 00 00 18 	l.j 568 <_or1k_reset+0x468>
 50c:	00 00 04 c0 	l.j 180c <_or1k_reset+0x170c>
 510:	00 00 33 14 	l.j d160 <_end+0x86f4>
 514:	00 00 00 3c 	l.j 604 <_or1k_reset+0x504>
 518:	43 89 01 82 	*unknown*
 51c:	02 81 03 42 	l.j fa041224 <_end+0xfa03c7b8>
 520:	0e 0c 00 00 	l.bnf f8300520 <_end+0xf82fbab4>
 524:	00 00 00 0c 	l.j 554 <_or1k_reset+0x454>
 528:	ff ff ff ff 	*unknown*
 52c:	01 00 04 7c 	l.j 400171c <_end+0x3ffccb0>
 530:	09 0d 01 00 	*unknown*
 534:	00 00 00 18 	l.j 594 <_or1k_reset+0x494>
 538:	00 00 05 24 	l.j 19c8 <_or1k_reset+0x18c8>
 53c:	00 00 33 50 	l.j d27c <_end+0x8810>
 540:	00 00 00 34 	l.j 610 <_or1k_reset+0x510>
 544:	42 82 01 45 	*unknown*
 548:	81 02 41 0e 	*unknown*
 54c:	08 00 00 00 	*unknown*
 550:	00 00 00 0c 	l.j 580 <_or1k_reset+0x480>
 554:	ff ff ff ff 	*unknown*
 558:	01 00 04 7c 	l.j 4001748 <_end+0x3ffccdc>
 55c:	09 0d 01 00 	*unknown*
 560:	00 00 00 14 	l.j 5b0 <_or1k_reset+0x4b0>
 564:	00 00 05 50 	l.j 1aa4 <_or1k_reset+0x19a4>
 568:	00 00 33 84 	l.j d378 <_end+0x890c>
 56c:	00 00 00 50 	l.j 6ac <_or1k_reset+0x5ac>
 570:	45 81 02 82 	*unknown*
 574:	01 41 0e 08 	l.j 5043d94 <_end+0x503f328>
 578:	00 00 00 1c 	l.j 5e8 <_or1k_reset+0x4e8>
 57c:	00 00 05 50 	l.j 1abc <_or1k_reset+0x19bc>
 580:	00 00 33 d4 	l.j d4d0 <_end+0x8a64>
 584:	00 00 00 b4 	l.j 854 <_or1k_reset+0x754>
 588:	45 92 02 89 	*unknown*
 58c:	01 81 05 82 	l.j 6041b94 <_end+0x603d128>
 590:	04 8e 03 42 	l.jal 2381298 <_end+0x237c82c>
 594:	0e 14 00 00 	l.bnf f8500594 <_end+0xf84fbb28>
 598:	00 00 00 18 	l.j 5f8 <_or1k_reset+0x4f8>
 59c:	00 00 05 50 	l.j 1adc <_or1k_reset+0x19dc>
 5a0:	00 00 34 88 	l.j d7c0 <_end+0x8d54>
 5a4:	00 00 00 68 	l.j 744 <_or1k_reset+0x644>
 5a8:	44 82 02 44 	*unknown*
 5ac:	89 01 81 03 	l.lws r8,-32509(r1)
 5b0:	41 0e 0c 00 	*unknown*
 5b4:	00 00 00 14 	l.j 604 <_or1k_reset+0x504>
 5b8:	00 00 05 50 	l.j 1af8 <_or1k_reset+0x19f8>
 5bc:	00 00 34 f0 	l.j d97c <_end+0x8f10>
 5c0:	00 00 00 28 	l.j 660 <_or1k_reset+0x560>
 5c4:	43 89 01 81 	*unknown*
 5c8:	02 41 0e 08 	l.j f9043de8 <_end+0xf903f37c>
 5cc:	00 00 00 14 	l.j 61c <_or1k_reset+0x51c>
 5d0:	00 00 05 50 	l.j 1b10 <_or1k_reset+0x1a10>
 5d4:	00 00 35 18 	l.j da34 <_end+0x8fc8>
 5d8:	00 00 00 54 	l.j 728 <_or1k_reset+0x628>
 5dc:	45 81 02 82 	*unknown*
 5e0:	01 41 0e 08 	l.j 5043e00 <_end+0x503f394>
 5e4:	00 00 00 14 	l.j 634 <_or1k_reset+0x534>
 5e8:	00 00 05 50 	l.j 1b28 <_or1k_reset+0x1a28>
 5ec:	00 00 35 6c 	l.j db9c <_end+0x9130>
 5f0:	00 00 00 60 	l.j 770 <_or1k_reset+0x670>
 5f4:	43 81 02 82 	*unknown*
 5f8:	01 41 0e 08 	l.j 5043e18 <_end+0x503f3ac>
 5fc:	00 00 00 14 	l.j 64c <_or1k_reset+0x54c>
 600:	00 00 05 50 	l.j 1b40 <_or1k_reset+0x1a40>
 604:	00 00 35 cc 	l.j dd34 <_end+0x92c8>
 608:	00 00 00 38 	l.j 6e8 <_or1k_reset+0x5e8>
 60c:	43 81 02 82 	*unknown*
 610:	01 41 0e 08 	l.j 5043e30 <_end+0x503f3c4>
 614:	00 00 00 14 	l.j 664 <_or1k_reset+0x564>
 618:	00 00 05 50 	l.j 1b58 <_or1k_reset+0x1a58>
 61c:	00 00 36 04 	l.j de2c <_end+0x93c0>
 620:	00 00 00 24 	l.j 6b0 <_or1k_reset+0x5b0>
 624:	42 81 01 41 	*unknown*
 628:	0e 04 00 00 	l.bnf f8100628 <_end+0xf80fbbbc>
 62c:	00 00 00 14 	l.j 67c <_or1k_reset+0x57c>
 630:	00 00 05 50 	l.j 1b70 <_or1k_reset+0x1a70>
 634:	00 00 36 28 	l.j ded4 <_end+0x9468>
 638:	00 00 00 34 	l.j 708 <_or1k_reset+0x608>
 63c:	43 81 02 82 	*unknown*
 640:	01 41 0e 08 	l.j 5043e60 <_end+0x503f3f4>
 644:	00 00 00 14 	l.j 694 <_or1k_reset+0x594>
 648:	00 00 05 50 	l.j 1b88 <_or1k_reset+0x1a88>
 64c:	00 00 36 5c 	l.j dfbc <_end+0x9550>
 650:	00 00 00 40 	l.j 750 <_or1k_reset+0x650>
 654:	43 81 02 82 	*unknown*
 658:	01 41 0e 08 	l.j 5043e78 <_end+0x503f40c>
 65c:	00 00 00 14 	l.j 6ac <_or1k_reset+0x5ac>
 660:	00 00 05 50 	l.j 1ba0 <_or1k_reset+0x1aa0>
 664:	00 00 36 9c 	l.j e0d4 <_end+0x9668>
 668:	00 00 00 20 	l.j 6e8 <_or1k_reset+0x5e8>
 66c:	43 81 01 41 	*unknown*
 670:	0e 04 00 00 	l.bnf f8100670 <_end+0xf80fbc04>
 674:	00 00 00 18 	l.j 6d4 <_or1k_reset+0x5d4>
 678:	00 00 05 50 	l.j 1bb8 <_or1k_reset+0x1ab8>
 67c:	00 00 36 bc 	l.j e16c <_end+0x9700>
 680:	00 00 00 2c 	l.j 730 <_or1k_reset+0x630>
 684:	43 82 01 42 	*unknown*
 688:	81 02 41 0e 	*unknown*
 68c:	08 00 00 00 	*unknown*
 690:	00 00 00 0c 	l.j 6c0 <_or1k_reset+0x5c0>
 694:	ff ff ff ff 	*unknown*
 698:	01 00 04 7c 	l.j 4001888 <_end+0x3ffce1c>
 69c:	09 0d 01 00 	*unknown*
 6a0:	00 00 00 14 	l.j 6f0 <_or1k_reset+0x5f0>
 6a4:	00 00 06 90 	l.j 20e4 <_or1k_start+0xb4>
 6a8:	00 00 37 1c 	l.j e318 <_end+0x98ac>
 6ac:	00 00 00 20 	l.j 72c <_or1k_reset+0x62c>
 6b0:	42 89 01 81 	*unknown*
 6b4:	02 42 0e 08 	l.j f9083ed4 <_end+0xf907f468>
 6b8:	00 00 00 0c 	l.j 6e8 <_or1k_reset+0x5e8>
 6bc:	ff ff ff ff 	*unknown*
 6c0:	01 00 04 7c 	l.j 40018b0 <_end+0x3ffce44>
 6c4:	09 0d 01 00 	*unknown*
 6c8:	00 00 00 14 	l.j 718 <_or1k_reset+0x618>
 6cc:	00 00 06 b8 	l.j 21ac <register_tm_clones+0x10>
 6d0:	00 00 37 44 	l.j e3e0 <_end+0x9974>
 6d4:	00 00 01 68 	l.j c74 <_or1k_reset+0xb74>
 6d8:	44 81 02 82 	*unknown*
 6dc:	01 42 0e 08 	l.j 5083efc <_end+0x507f490>
 6e0:	00 00 00 0c 	l.j 710 <_or1k_reset+0x610>
 6e4:	ff ff ff ff 	*unknown*
 6e8:	03 00 01 7c 	l.j fc000cd8 <_end+0xfbffc26c>
 6ec:	09 0c 01 00 	*unknown*
 6f0:	00 00 00 24 	l.j 780 <_or1k_reset+0x680>
 6f4:	00 00 06 e0 	l.j 2274 <__do_global_dtors_aux+0x6c>
 6f8:	00 00 39 a8 	l.j ed98 <_end+0xa32c>
 6fc:	00 00 00 54 	l.j 84c <_or1k_reset+0x74c>
 700:	04 00 00 00 	l.jal 700 <_or1k_reset+0x600>
 704:	04 82 02 04 	l.jal 2080f14 <_end+0x207c4a8>
 708:	00 00 00 14 	l.j 758 <_or1k_reset+0x658>
 70c:	89 01 81 03 	l.lws r8,-32509(r1)
 710:	04 00 00 00 	l.jal 710 <_or1k_reset+0x610>
 714:	04 0e 0c 00 	l.jal 383714 <_end+0x37eca8>
 718:	00 00 00 1c 	l.j 788 <_or1k_reset+0x688>
 71c:	00 00 06 e0 	l.j 229c <__do_global_dtors_aux+0x94>
 720:	00 00 39 fc 	l.j ef10 <_end+0xa4a4>
 724:	00 00 00 1c 	l.j 794 <_or1k_reset+0x694>
 728:	04 00 00 00 	l.jal 728 <_or1k_reset+0x628>
 72c:	08 89 01 81 	*unknown*
 730:	02 04 00 00 	l.j f8100730 <_end+0xf80fbcc4>
 734:	00 04 0e 08 	l.j 103f54 <_end+0xff4e8>

Disassembly of section .debug_str:

00000000 <.debug_str>:
       0:	6c 6f 6e 67 	l.lwa r3,28263(r15)
       4:	20 6c 6f 6e 	*unknown*
       8:	67 20 69 6e 	*unknown*
       c:	74 00 73 68 	*unknown*
      10:	6f 72 74 20 	l.lwa r27,29728(r18)
      14:	75 6e 73 69 	*unknown*
      18:	67 6e 65 64 	*unknown*
      1c:	20 69 6e 74 	*unknown*
      20:	00 5f 5f 61 	l.j 17d7da4 <_end+0x17d3338>
      24:	74 65 78 69 	*unknown*
      28:	74 5f 74 79 	*unknown*
      2c:	70 65 73 00 	*unknown*
      30:	47 4e 55 20 	*unknown*
      34:	43 20 34 2e 	*unknown*
      38:	39 2e 32 20 	*unknown*
      3c:	2d 6d 6e 65 	*unknown*
      40:	77 6c 69 62 	*unknown*
      44:	20 2d 67 20 	*unknown*
      48:	2d 4f 32 20 	*unknown*
      4c:	2d 66 6e 6f 	*unknown*
      50:	2d 62 75 69 	*unknown*
      54:	6c 74 69 6e 	l.lwa r3,26990(r20)
      58:	00 6c 6f 6e 	l.j 1b1be10 <_end+0x1b173a4>
      5c:	67 20 6c 6f 	*unknown*
      60:	6e 67 20 75 	l.lwa r19,8309(r7)
      64:	6e 73 69 67 	l.lwa r19,26983(r19)
      68:	6e 65 64 20 	l.lwa r19,25632(r5)
      6c:	69 6e 74 00 	*unknown*
      70:	5f 5f 65 74 	*unknown*
      74:	5f 63 78 61 	*unknown*
      78:	00 75 6e 73 	l.j 1d5ba44 <_end+0x1d56fd8>
      7c:	69 67 6e 65 	*unknown*
      80:	64 20 63 68 	*unknown*
      84:	61 72 00 5f 	*unknown*
      88:	5f 72 65 67 	*unknown*
      8c:	69 73 74 65 	*unknown*
      90:	72 5f 65 78 	*unknown*
      94:	69 74 70 72 	*unknown*
      98:	6f 63 00 2f 	l.lwa r27,47(r3)
      9c:	68 6f 6d 65 	*unknown*
      a0:	2f 61 6e 64 	*unknown*
      a4:	72 7a 65 6a 	*unknown*
      a8:	72 2f 44 69 	*unknown*
      ac:	67 69 74 61 	*unknown*
      b0:	6c 2f 4f 52 	l.lwa r1,20306(r15)
      b4:	50 53 6f 43 	*unknown*
      b8:	2f 62 75 69 	*unknown*
      bc:	6c 64 2d 6e 	l.lwa r3,11630(r4)
      c0:	65 77 6c 69 	*unknown*
      c4:	62 2f 6f 72 	*unknown*
      c8:	31 6b 2d 65 	*unknown*
      cc:	6c 66 2f 6e 	l.lwa r3,12142(r6)
      d0:	65 77 6c 69 	*unknown*
      d4:	62 2f 6c 69 	*unknown*
      d8:	62 63 2f 73 	*unknown*
      dc:	74 64 6c 69 	*unknown*
      e0:	62 00 5f 5f 	*unknown*
      e4:	65 74 5f 6f 	*unknown*
      e8:	6e 65 78 69 	l.lwa r19,30825(r5)
      ec:	74 00 2e 2e 	*unknown*
      f0:	2f 2e 2e 2f 	*unknown*
      f4:	2e 2e 2f 2e 	*unknown*
      f8:	2e 2f 2e 2e 	*unknown*
      fc:	2f 6e 65 77 	*unknown*
     100:	6c 69 62 2d 	l.lwa r3,25133(r9)
     104:	32 2e 32 2e 	*unknown*
     108:	30 2e 32 30 	*unknown*
     10c:	31 35 30 38 	*unknown*
     110:	32 34 2f 6e 	*unknown*
     114:	65 77 6c 69 	*unknown*
     118:	62 2f 6c 69 	*unknown*
     11c:	62 63 2f 73 	*unknown*
     120:	74 64 6c 69 	*unknown*
     124:	62 2f 61 74 	*unknown*
     128:	65 78 69 74 	*unknown*
     12c:	2e 63 00 73 	*unknown*
     130:	68 6f 72 74 	*unknown*
     134:	20 69 6e 74 	*unknown*
     138:	00 5f 5f 65 	l.j 17d7ecc <_end+0x17d3460>
     13c:	74 5f 61 74 	*unknown*
     140:	65 78 69 74 	*unknown*
     144:	00 73 69 7a 	l.j 1cda72c <_end+0x1cd5cc0>
     148:	65 74 79 70 	*unknown*
     14c:	65 00 5f 64 	*unknown*
     150:	73 6f 5f 68 	*unknown*
     154:	61 6e 64 6c 	*unknown*
     158:	65 00 5f 72 	*unknown*
     15c:	61 6e 64 34 	*unknown*
     160:	38 00 5f 65 	*unknown*
     164:	6d 65 72 67 	l.lwa r11,29287(r5)
     168:	65 6e 63 79 	*unknown*
     16c:	00 5f 77 63 	l.j 17ddef8 <_end+0x17d948c>
     170:	72 74 6f 6d 	*unknown*
     174:	62 5f 73 74 	*unknown*
     178:	61 74 65 00 	*unknown*
     17c:	5f 77 63 73 	*unknown*
     180:	72 74 6f 6d 	*unknown*
     184:	62 73 5f 73 	*unknown*
     188:	74 61 74 65 	*unknown*
     18c:	00 5f 6c 62 	l.j 17db314 <_end+0x17d68a8>
     190:	66 73 69 7a 	*unknown*
     194:	65 00 5f 6d 	*unknown*
     198:	62 72 74 6f 	*unknown*
     19c:	77 63 5f 73 	*unknown*
     1a0:	74 61 74 65 	*unknown*
     1a4:	00 5f 77 63 	l.j 17ddf30 <_end+0x17d94c4>
     1a8:	74 6f 6d 62 	*unknown*
     1ac:	5f 73 74 61 	*unknown*
     1b0:	74 65 00 5f 	*unknown*
     1b4:	5f 74 6d 5f 	*unknown*
     1b8:	73 65 63 00 	*unknown*
     1bc:	5f 75 62 75 	*unknown*
     1c0:	66 00 5f 5f 	*unknown*
     1c4:	74 6d 5f 68 	*unknown*
     1c8:	6f 75 72 00 	l.lwa r27,29184(r21)
     1cc:	5f 5f 73 66 	*unknown*
     1d0:	00 5f 6f 6e 	l.j 17dbf88 <_end+0x17d751c>
     1d4:	5f 65 78 69 	*unknown*
     1d8:	74 5f 61 72 	*unknown*
     1dc:	67 73 00 5f 	*unknown*
     1e0:	63 6f 6f 6b 	*unknown*
     1e4:	69 65 00 5f 	*unknown*
     1e8:	5f 73 67 6c 	*unknown*
     1ec:	75 65 00 5f 	*unknown*
     1f0:	66 6c 61 67 	*unknown*
     1f4:	73 00 5f 69 	*unknown*
     1f8:	73 5f 63 78 	*unknown*
     1fc:	61 00 5f 73 	*unknown*
     200:	74 64 69 6e 	*unknown*
     204:	00 5f 63 76 	l.j 17d8fdc <_end+0x17d4570>
     208:	74 62 75 66 	*unknown*
     20c:	00 5f 6f 66 	l.j 17dbfa4 <_end+0x17d7538>
     210:	66 73 65 74 	*unknown*
     214:	00 5f 6d 62 	l.j 17db79c <_end+0x17d6d30>
     218:	73 72 74 6f 	*unknown*
     21c:	77 63 73 5f 	*unknown*
     220:	73 74 61 74 	*unknown*
     224:	65 00 5f 6d 	*unknown*
     228:	62 72 6c 65 	*unknown*
     22c:	6e 5f 73 74 	l.lwa r18,29556(r31)
     230:	61 74 65 00 	*unknown*
     234:	5f 66 6e 61 	*unknown*
     238:	72 67 73 00 	*unknown*
     23c:	5f 66 6e 73 	*unknown*
     240:	00 5f 73 69 	l.j 17dcfe4 <_end+0x17d8578>
     244:	67 6e 00 5f 	*unknown*
     248:	66 6c 6f 63 	*unknown*
     24c:	6b 5f 74 00 	*unknown*
     250:	5f 73 74 64 	*unknown*
     254:	65 72 72 00 	*unknown*
     258:	5f 42 69 67 	*unknown*
     25c:	69 6e 74 00 	*unknown*
     260:	5f 67 61 6d 	*unknown*
     264:	6d 61 5f 73 	l.lwa r11,24435(r1)
     268:	69 67 6e 67 	*unknown*
     26c:	61 6d 00 5f 	*unknown*
     270:	72 65 61 64 	*unknown*
     274:	00 5f 72 65 	l.j 17dcc08 <_end+0x17d819c>
     278:	73 75 6c 74 	*unknown*
     27c:	5f 6b 00 5f 	*unknown*
     280:	5f 74 6d 00 	*unknown*
     284:	5f 5f 77 63 	*unknown*
     288:	68 62 00 5f 	*unknown*
     28c:	73 74 64 6f 	*unknown*
     290:	75 74 00 5f 	*unknown*
     294:	63 76 74 6c 	*unknown*
     298:	65 6e 00 5f 	*unknown*
     29c:	66 69 6c 65 	*unknown*
     2a0:	00 5f 6e 69 	l.j 17dbc44 <_end+0x17d71d8>
     2a4:	6f 62 73 00 	l.lwa r27,29440(r2)
     2a8:	2e 2e 2f 2e 	*unknown*
     2ac:	2e 2f 2e 2e 	*unknown*
     2b0:	2f 2e 2e 2f 	*unknown*
     2b4:	2e 2e 2f 6e 	*unknown*
     2b8:	65 77 6c 69 	*unknown*
     2bc:	62 2d 32 2e 	*unknown*
     2c0:	32 2e 30 2e 	*unknown*
     2c4:	32 30 31 35 	*unknown*
     2c8:	30 38 32 34 	*unknown*
     2cc:	2f 6e 65 77 	*unknown*
     2d0:	6c 69 62 2f 	l.lwa r3,25135(r9)
     2d4:	6c 69 62 63 	l.lwa r3,25187(r9)
     2d8:	2f 73 74 64 	*unknown*
     2dc:	6c 69 62 2f 	l.lwa r3,25135(r9)
     2e0:	65 78 69 74 	*unknown*
     2e4:	2e 63 00 5f 	*unknown*
     2e8:	61 74 65 78 	*unknown*
     2ec:	69 74 30 00 	*unknown*
     2f0:	5f 73 69 67 	*unknown*
     2f4:	6e 61 6c 5f 	l.lwa r19,27743(r1)
     2f8:	62 75 66 00 	*unknown*
     2fc:	5f 61 73 63 	*unknown*
     300:	74 69 6d 65 	*unknown*
     304:	5f 62 75 66 	*unknown*
     308:	00 5f 72 65 	l.j 17dcc9c <_end+0x17d8230>
     30c:	73 75 6c 74 	*unknown*
     310:	00 5f 5f 77 	l.j 17d80ec <_end+0x17d3680>
     314:	63 68 00 77 	*unknown*
     318:	69 6e 74 5f 	*unknown*
     31c:	74 00 5f 66 	*unknown*
     320:	6c 61 67 73 	l.lwa r3,26483(r1)
     324:	32 00 5f 5f 	*unknown*
     328:	74 6d 5f 79 	*unknown*
     32c:	65 61 72 00 	*unknown*
     330:	5f 6e 65 78 	*unknown*
     334:	74 66 00 5f 	*unknown*
     338:	5f 74 6d 5f 	*unknown*
     33c:	6d 6f 6e 00 	l.lwa r11,28160(r15)
     340:	5f 5f 73 64 	*unknown*
     344:	69 64 69 6e 	*unknown*
     348:	69 74 00 5f 	*unknown*
     34c:	6f 66 66 5f 	l.lwa r27,26207(r6)
     350:	74 00 5f 5f 	*unknown*
     354:	63 61 6c 6c 	*unknown*
     358:	5f 65 78 69 	*unknown*
     35c:	74 70 72 6f 	*unknown*
     360:	63 73 00 5f 	*unknown*
     364:	66 72 65 65 	*unknown*
     368:	6c 69 73 74 	l.lwa r3,29556(r9)
     36c:	00 5f 4c 4f 	l.j 17d34a8 <_end+0x17cea3c>
     370:	43 4b 5f 52 	*unknown*
     374:	45 43 55 52 	*unknown*
     378:	53 49 56 45 	*unknown*
     37c:	5f 54 00 5f 	*unknown*
     380:	6e 65 77 00 	l.lwa r19,30464(r5)
     384:	5f 68 5f 65 	*unknown*
     388:	72 72 6e 6f 	*unknown*
     38c:	00 5f 5f 74 	l.j 17d815c <_end+0x17d36f0>
     390:	6d 5f 79 64 	l.lwa r10,31076(r31)
     394:	61 79 00 5f 	*unknown*
     398:	5f 73 62 75 	*unknown*
     39c:	66 00 5f 69 	*unknown*
     3a0:	6f 62 73 00 	l.lwa r27,29440(r2)
     3a4:	5f 5f 46 49 	*unknown*
     3a8:	4c 45 00 5f 	*unknown*
     3ac:	6d 62 73 74 	l.lwa r11,29556(r2)
     3b0:	61 74 65 5f 	*unknown*
     3b4:	74 00 5f 5f 	*unknown*
     3b8:	73 46 49 4c 	*unknown*
     3bc:	45 00 5f 6d 	*unknown*
     3c0:	62 73 74 61 	*unknown*
     3c4:	74 65 00 5f 	*unknown*
     3c8:	72 61 6e 64 	*unknown*
     3cc:	5f 6e 65 78 	*unknown*
     3d0:	74 00 5f 6d 	*unknown*
     3d4:	62 6c 65 6e 	*unknown*
     3d8:	5f 73 74 61 	*unknown*
     3dc:	74 65 00 5f 	*unknown*
     3e0:	69 6e 63 00 	*unknown*
     3e4:	5f 69 6e 64 	*unknown*
     3e8:	00 5f 63 75 	l.j 17d91bc <_end+0x17d4750>
     3ec:	72 72 65 6e 	*unknown*
     3f0:	74 5f 6c 6f 	*unknown*
     3f4:	63 61 6c 65 	*unknown*
     3f8:	00 5f 5f 63 	l.j 17d8184 <_end+0x17d3718>
     3fc:	6c 65 61 6e 	l.lwa r3,24942(r5)
     400:	75 70 00 5f 	*unknown*
     404:	6d 61 78 77 	l.lwa r11,30839(r1)
     408:	64 73 00 5f 	*unknown*
     40c:	73 65 65 64 	*unknown*
     410:	00 5f 5f 63 	l.j 17d819c <_end+0x17d3730>
     414:	6f 75 6e 74 	l.lwa r27,28276(r21)
     418:	00 5f 5f 76 	l.j 17d81f0 <_end+0x17d3784>
     41c:	61 6c 75 65 	*unknown*
     420:	00 5f 73 65 	l.j 17dd1b4 <_end+0x17d8748>
     424:	65 6b 00 5f 	*unknown*
     428:	66 70 6f 73 	*unknown*
     42c:	5f 74 00 5f 	*unknown*
     430:	5f 74 6d 5f 	*unknown*
     434:	6d 69 6e 00 	l.lwa r11,28160(r9)
     438:	5f 6d 75 6c 	*unknown*
     43c:	74 00 5f 73 	*unknown*
     440:	74 72 74 6f 	*unknown*
     444:	6b 5f 6c 61 	*unknown*
     448:	73 74 00 5f 	*unknown*
     44c:	66 6e 74 79 	*unknown*
     450:	70 65 73 00 	*unknown*
     454:	5f 5f 55 4c 	*unknown*
     458:	6f 6e 67 00 	l.lwa r27,26368(r14)
     45c:	5f 67 65 74 	*unknown*
     460:	64 61 74 65 	*unknown*
     464:	5f 65 72 72 	*unknown*
     468:	00 5f 67 6c 	l.j 17da218 <_end+0x17d57ac>
     46c:	6f 62 61 6c 	l.lwa r27,24940(r2)
     470:	5f 69 6d 70 	*unknown*
     474:	75 72 65 5f 	*unknown*
     478:	70 74 72 00 	*unknown*
     47c:	5f 63 75 72 	*unknown*
     480:	72 65 6e 74 	*unknown*
     484:	5f 63 61 74 	*unknown*
     488:	65 67 6f 72 	*unknown*
     48c:	79 00 63 6f 	*unknown*
     490:	64 65 00 5f 	*unknown*
     494:	75 6e 75 73 	*unknown*
     498:	65 64 5f 72 	*unknown*
     49c:	61 6e 64 00 	*unknown*
     4a0:	5f 77 64 73 	*unknown*
     4a4:	00 5f 5f 74 	l.j 17d8274 <_end+0x17d3808>
     4a8:	6d 5f 77 64 	l.lwa r10,30564(r31)
     4ac:	61 79 00 5f 	*unknown*
     4b0:	67 6c 75 65 	*unknown*
     4b4:	00 5f 6e 6d 	l.j 17dbe68 <_end+0x17d73fc>
     4b8:	61 6c 6c 6f 	*unknown*
     4bc:	63 00 5f 6c 	*unknown*
     4c0:	36 34 61 5f 	*unknown*
     4c4:	62 75 66 00 	*unknown*
     4c8:	5f 73 69 67 	*unknown*
     4cc:	5f 66 75 6e 	*unknown*
     4d0:	63 00 5f 6e 	*unknown*
     4d4:	62 75 66 00 	*unknown*
     4d8:	5f 75 6e 75 	*unknown*
     4dc:	73 65 64 00 	*unknown*
     4e0:	5f 5f 74 6d 	*unknown*
     4e4:	5f 69 73 64 	*unknown*
     4e8:	73 74 00 5f 	*unknown*
     4ec:	6c 6f 63 61 	l.lwa r3,25441(r15)
     4f0:	6c 74 69 6d 	l.lwa r3,26989(r20)
     4f4:	65 5f 62 75 	*unknown*
     4f8:	66 00 5f 63 	*unknown*
     4fc:	6c 6f 73 65 	l.lwa r3,29541(r15)
     500:	00 5f 72 34 	l.j 17dcdd0 <_end+0x17d8364>
     504:	38 00 5f 6d 	*unknown*
     508:	62 74 6f 77 	*unknown*
     50c:	63 5f 73 74 	*unknown*
     510:	61 74 65 00 	*unknown*
     514:	5f 70 35 73 	*unknown*
     518:	00 5f 5f 74 	l.j 17d82e8 <_end+0x17d387c>
     51c:	6d 5f 6d 64 	l.lwa r10,28004(r31)
     520:	61 79 00 2e 	*unknown*
     524:	2e 2f 2e 2e 	*unknown*
     528:	2f 2e 2e 2f 	*unknown*
     52c:	2e 2e 2f 2e 	*unknown*
     530:	2e 2f 6e 65 	*unknown*
     534:	77 6c 69 62 	*unknown*
     538:	2d 32 2e 32 	*unknown*
     53c:	2e 30 2e 32 	*unknown*
     540:	30 31 35 30 	*unknown*
     544:	38 32 34 2f 	*unknown*
     548:	6e 65 77 6c 	l.lwa r19,30572(r5)
     54c:	69 62 2f 6c 	*unknown*
     550:	69 62 63 2f 	*unknown*
     554:	72 65 65 6e 	*unknown*
     558:	74 2f 69 6d 	*unknown*
     55c:	70 75 72 65 	*unknown*
     560:	2e 63 00 2f 	*unknown*
     564:	68 6f 6d 65 	*unknown*
     568:	2f 61 6e 64 	*unknown*
     56c:	72 7a 65 6a 	*unknown*
     570:	72 2f 44 69 	*unknown*
     574:	67 69 74 61 	*unknown*
     578:	6c 2f 4f 52 	l.lwa r1,20306(r15)
     57c:	50 53 6f 43 	*unknown*
     580:	2f 62 75 69 	*unknown*
     584:	6c 64 2d 6e 	l.lwa r3,11630(r4)
     588:	65 77 6c 69 	*unknown*
     58c:	62 2f 6f 72 	*unknown*
     590:	31 6b 2d 65 	*unknown*
     594:	6c 66 2f 6e 	l.lwa r3,12142(r6)
     598:	65 77 6c 69 	*unknown*
     59c:	62 2f 6c 69 	*unknown*
     5a0:	62 63 2f 72 	*unknown*
     5a4:	65 65 6e 74 	*unknown*
     5a8:	00 2e 2e 2f 	l.j b8be64 <_end+0xb873f8>
     5ac:	2e 2e 2f 2e 	*unknown*
     5b0:	2e 2f 2e 2e 	*unknown*
     5b4:	2f 2e 2e 2f 	*unknown*
     5b8:	6e 65 77 6c 	l.lwa r19,30572(r5)
     5bc:	69 62 2d 32 	*unknown*
     5c0:	2e 32 2e 30 	*unknown*
     5c4:	2e 32 30 31 	*unknown*
     5c8:	35 30 38 32 	*unknown*
     5cc:	34 2f 6e 65 	*unknown*
     5d0:	77 6c 69 62 	*unknown*
     5d4:	2f 6c 69 62 	*unknown*
     5d8:	63 2f 73 74 	*unknown*
     5dc:	64 6c 69 62 	*unknown*
     5e0:	2f 5f 5f 61 	*unknown*
     5e4:	74 65 78 69 	*unknown*
     5e8:	74 2e 63 00 	*unknown*
     5ec:	66 72 65 65 	*unknown*
     5f0:	00 6c 61 73 	l.j 1b18bbc <_end+0x1b14150>
     5f4:	74 70 00 72 	*unknown*
     5f8:	65 73 74 61 	*unknown*
     5fc:	72 74 00 2e 	*unknown*
     600:	2e 2f 2e 2e 	*unknown*
     604:	2f 2e 2e 2f 	*unknown*
     608:	2e 2e 2f 2e 	*unknown*
     60c:	2e 2f 6e 65 	*unknown*
     610:	77 6c 69 62 	*unknown*
     614:	2d 32 2e 32 	*unknown*
     618:	2e 30 2e 32 	*unknown*
     61c:	30 31 35 30 	*unknown*
     620:	38 32 34 2f 	*unknown*
     624:	6e 65 77 6c 	l.lwa r19,30572(r5)
     628:	69 62 2f 6c 	*unknown*
     62c:	69 62 63 2f 	*unknown*
     630:	73 74 64 6c 	*unknown*
     634:	69 62 2f 5f 	*unknown*
     638:	5f 63 61 6c 	*unknown*
     63c:	6c 5f 61 74 	l.lwa r2,24948(r31)
     640:	65 78 69 74 	*unknown*
     644:	2e 63 00 5f 	*unknown*
     648:	5f 61 74 65 	*unknown*
     64c:	78 69 74 5f 	*unknown*
     650:	6c 6f 63 6b 	l.lwa r3,25451(r15)
     654:	00 5f 5f 67 	l.j 17d83f0 <_end+0x17d3984>
     658:	69 64 5f 74 	*unknown*
     65c:	00 65 78 69 	l.j 195e800 <_end+0x1959d94>
     660:	73 74 69 6e 	*unknown*
     664:	67 00 5f 65 	*unknown*
     668:	78 65 63 76 	*unknown*
     66c:	65 5f 72 00 	*unknown*
     670:	6e 6c 69 6e 	l.lwa r19,26990(r12)
     674:	6b 5f 74 00 	*unknown*
     678:	6d 6f 64 65 	l.lwa r11,25701(r15)
     67c:	5f 74 00 73 	*unknown*
     680:	74 5f 64 65 	*unknown*
     684:	76 00 5f 6f 	*unknown*
     688:	72 31 6b 5f 	*unknown*
     68c:	62 6f 61 72 	*unknown*
     690:	64 5f 65 78 	*unknown*
     694:	69 74 00 73 	*unknown*
     698:	74 5f 67 69 	*unknown*
     69c:	64 00 70 61 	*unknown*
     6a0:	74 68 00 73 	*unknown*
     6a4:	74 5f 62 6c 	*unknown*
     6a8:	6f 63 6b 73 	l.lwa r27,27507(r3)
     6ac:	00 73 74 61 	l.j 1cdd830 <_end+0x1cd8dc4>
     6b0:	74 00 70 74 	*unknown*
     6b4:	69 6d 65 7a 	*unknown*
     6b8:	6f 6e 65 00 	l.lwa r27,25856(r14)
     6bc:	5f 5f 65 72 	*unknown*
     6c0:	72 6e 6f 00 	*unknown*
     6c4:	5f 6f 70 65 	*unknown*
     6c8:	6e 00 73 74 	l.lwa r16,29556(r0)
     6cc:	5f 73 70 61 	*unknown*
     6d0:	72 65 33 00 	*unknown*
     6d4:	5f 66 6f 72 	*unknown*
     6d8:	6b 5f 72 00 	*unknown*
     6dc:	5f 6b 69 6c 	*unknown*
     6e0:	6c 5f 72 00 	l.lwa r2,29184(r31)
     6e4:	2e 2e 2f 2e 	*unknown*
     6e8:	2e 2f 2e 2e 	*unknown*
     6ec:	2f 2e 2e 2f 	*unknown*
     6f0:	6e 65 77 6c 	l.lwa r19,30572(r5)
     6f4:	69 62 2d 32 	*unknown*
     6f8:	2e 32 2e 30 	*unknown*
     6fc:	2e 32 30 31 	*unknown*
     700:	35 30 38 32 	*unknown*
     704:	34 2f 6c 69 	*unknown*
     708:	62 67 6c 6f 	*unknown*
     70c:	73 73 2f 6f 	*unknown*
     710:	72 31 6b 2f 	*unknown*
     714:	73 79 73 63 	*unknown*
     718:	61 6c 6c 73 	*unknown*
     71c:	2e 63 00 62 	*unknown*
     720:	75 66 73 69 	*unknown*
     724:	7a 65 00 5f 	*unknown*
     728:	6c 73 65 65 	l.lwa r3,25957(r19)
     72c:	6b 5f 72 00 	*unknown*
     730:	5f 5f 64 65 	*unknown*
     734:	76 5f 74 00 	*unknown*
     738:	73 74 5f 73 	*unknown*
     73c:	70 61 72 65 	*unknown*
     740:	31 00 73 74 	*unknown*
     744:	5f 73 70 61 	*unknown*
     748:	72 65 32 00 	*unknown*
     74c:	5f 5f 73 75 	*unknown*
     750:	73 65 63 6f 	*unknown*
     754:	6e 64 73 5f 	l.lwa r19,29535(r4)
     758:	74 00 6e 61 	*unknown*
     75c:	6d 65 00 73 	l.lwa r11,115(r5)
     760:	74 5f 75 69 	*unknown*
     764:	64 00 5f 73 	*unknown*
     768:	74 61 74 5f 	*unknown*
     76c:	72 00 65 6e 	*unknown*
     770:	76 69 72 6f 	*unknown*
     774:	6e 00 5f 63 	l.lwa r16,24419(r0)
     778:	6c 6f 73 65 	l.lwa r3,29541(r15)
     77c:	5f 72 00 5f 	*unknown*
     780:	67 65 74 70 	*unknown*
     784:	69 64 5f 72 	*unknown*
     788:	00 73 74 5f 	l.j 1cdd904 <_end+0x1cd8e98>
     78c:	73 69 7a 65 	*unknown*
     790:	00 5f 77 72 	l.j 17de558 <_end+0x17d9aec>
     794:	69 74 65 5f 	*unknown*
     798:	72 00 70 74 	*unknown*
     79c:	69 6d 65 76 	*unknown*
     7a0:	61 6c 00 74 	*unknown*
     7a4:	76 5f 73 65 	*unknown*
     7a8:	63 00 2f 68 	*unknown*
     7ac:	6f 6d 65 2f 	l.lwa r27,25903(r13)
     7b0:	61 6e 64 72 	*unknown*
     7b4:	7a 65 6a 72 	*unknown*
     7b8:	2f 44 69 67 	*unknown*
     7bc:	69 74 61 6c 	*unknown*
     7c0:	2f 4f 52 50 	*unknown*
     7c4:	53 6f 43 2f 	*unknown*
     7c8:	62 75 69 6c 	*unknown*
     7cc:	64 2d 6e 65 	*unknown*
     7d0:	77 6c 69 62 	*unknown*
     7d4:	2f 6f 72 31 	*unknown*
     7d8:	6b 2d 65 6c 	*unknown*
     7dc:	66 2f 6c 69 	*unknown*
     7e0:	62 67 6c 6f 	*unknown*
     7e4:	73 73 2f 6f 	*unknown*
     7e8:	72 31 6b 00 	*unknown*
     7ec:	5f 72 65 61 	*unknown*
     7f0:	64 6c 69 6e 	*unknown*
     7f4:	6b 5f 72 00 	*unknown*
     7f8:	74 69 6d 65 	*unknown*
     7fc:	5f 74 00 6e 	*unknown*
     800:	62 79 74 65 	*unknown*
     804:	73 00 73 74 	*unknown*
     808:	5f 6e 6c 69 	*unknown*
     80c:	6e 6b 00 73 	l.lwa r19,115(r11)
     810:	74 5f 69 6e 	*unknown*
     814:	6f 00 73 74 	l.lwa r24,29556(r0)
     818:	5f 62 6c 6b 	*unknown*
     81c:	73 69 7a 65 	*unknown*
     820:	00 73 74 5f 	l.j 1cdd99c <_end+0x1cd8f30>
     824:	63 74 69 6d 	*unknown*
     828:	65 00 5f 5f 	*unknown*
     82c:	65 6e 76 00 	*unknown*
     830:	5f 69 73 61 	*unknown*
     834:	74 74 79 5f 	*unknown*
     838:	72 00 66 69 	*unknown*
     83c:	6c 64 65 73 	l.lwa r3,25971(r4)
     840:	00 73 74 5f 	l.j 1cdd9bc <_end+0x1cd8f50>
     844:	61 74 69 6d 	*unknown*
     848:	65 00 5f 72 	*unknown*
     84c:	65 61 64 5f 	*unknown*
     850:	72 00 73 74 	*unknown*
     854:	5f 73 70 61 	*unknown*
     858:	72 65 34 00 	*unknown*
     85c:	5f 5f 75 69 	*unknown*
     860:	64 5f 74 00 	*unknown*
     864:	73 74 5f 6d 	*unknown*
     868:	6f 64 65 00 	l.lwa r27,25856(r4)
     86c:	5f 67 65 74 	*unknown*
     870:	74 69 6d 65 	*unknown*
     874:	6f 66 64 61 	l.lwa r27,25697(r6)
     878:	79 00 74 76 	*unknown*
     87c:	5f 75 73 65 	*unknown*
     880:	63 00 5f 66 	*unknown*
     884:	73 74 61 74 	*unknown*
     888:	5f 72 00 5f 	*unknown*
     88c:	6f 72 31 6b 	l.lwa r27,12651(r18)
     890:	5f 6f 75 74 	*unknown*
     894:	62 79 74 65 	*unknown*
     898:	00 47 4e 55 	l.j 11d41ec <_end+0x11cf780>
     89c:	20 43 20 34 	*unknown*
     8a0:	2e 39 2e 32 	*unknown*
     8a4:	20 2d 6d 6e 	*unknown*
     8a8:	65 77 6c 69 	*unknown*
     8ac:	62 20 2d 67 	*unknown*
     8b0:	20 2d 67 20 	*unknown*
     8b4:	2d 4f 32 20 	*unknown*
     8b8:	2d 4f 32 20 	*unknown*
     8bc:	2d 4f 32 00 	*unknown*
     8c0:	5f 73 73 69 	*unknown*
     8c4:	7a 65 5f 74 	*unknown*
     8c8:	00 73 74 5f 	l.j 1cdda44 <_end+0x1cd8fd8>
     8cc:	72 64 65 76 	*unknown*
     8d0:	00 5f 6c 69 	l.j 17dba74 <_end+0x17d7008>
     8d4:	6e 6b 5f 72 	l.lwa r19,24434(r11)
     8d8:	00 61 72 67 	l.j 185d274 <_end+0x1858808>
     8dc:	76 00 73 74 	*unknown*
     8e0:	5f 6d 74 69 	*unknown*
     8e4:	6d 65 00 5f 	l.lwa r11,95(r5)
     8e8:	75 6e 6c 69 	*unknown*
     8ec:	6e 6b 5f 72 	l.lwa r19,24434(r11)
     8f0:	00 69 6e 6f 	l.j 1a5c2ac <_end+0x1a57840>
     8f4:	5f 74 00 5f 	*unknown*
     8f8:	5f 75 69 6e 	*unknown*
     8fc:	74 38 5f 74 	*unknown*
     900:	00 5f 6f 72 	l.j 17dc6c8 <_end+0x17d7c5c>
     904:	31 6b 5f 62 	*unknown*
     908:	6f 61 72 64 	l.lwa r27,29284(r1)
     90c:	5f 75 61 72 	*unknown*
     910:	74 5f 62 61 	*unknown*
     914:	75 64 00 5f 	*unknown*
     918:	6f 72 31 6b 	l.lwa r27,12651(r18)
     91c:	5f 75 61 72 	*unknown*
     920:	74 5f 69 6e 	*unknown*
     924:	74 65 72 72 	*unknown*
     928:	75 70 74 5f 	*unknown*
     92c:	68 61 6e 64 	*unknown*
     930:	6c 65 72 00 	l.lwa r3,29184(r5)
     934:	2e 2e 2f 2e 	*unknown*
     938:	2e 2f 2e 2e 	*unknown*
     93c:	2f 2e 2e 2f 	*unknown*
     940:	6e 65 77 6c 	l.lwa r19,30572(r5)
     944:	69 62 2d 32 	*unknown*
     948:	2e 32 2e 30 	*unknown*
     94c:	2e 32 30 31 	*unknown*
     950:	35 30 38 32 	*unknown*
     954:	34 2f 6c 69 	*unknown*
     958:	62 67 6c 6f 	*unknown*
     95c:	73 73 2f 6f 	*unknown*
     960:	72 31 6b 2f 	*unknown*
     964:	6f 72 31 6b 	l.lwa r27,12651(r18)
     968:	5f 75 61 72 	*unknown*
     96c:	74 2e 63 00 	*unknown*
     970:	6f 72 31 6b 	l.lwa r27,12651(r18)
     974:	5f 69 6e 74 	*unknown*
     978:	65 72 72 75 	*unknown*
     97c:	70 74 5f 65 	*unknown*
     980:	6e 61 62 6c 	l.lwa r19,25196(r1)
     984:	65 00 5f 5f 	*unknown*
     988:	75 69 6e 74 	*unknown*
     98c:	31 36 5f 74 	*unknown*
     990:	00 5f 5f 75 	l.j 17d8764 <_end+0x17d3cf8>
     994:	69 6e 74 33 	*unknown*
     998:	32 5f 74 00 	*unknown*
     99c:	5f 6f 72 31 	*unknown*
     9a0:	6b 5f 75 61 	*unknown*
     9a4:	72 74 5f 77 	*unknown*
     9a8:	72 69 74 65 	*unknown*
     9ac:	00 6f 72 31 	l.j 1bdd270 <_end+0x1bd8804>
     9b0:	6b 5f 75 61 	*unknown*
     9b4:	72 74 5f 73 	*unknown*
     9b8:	65 74 5f 72 	*unknown*
     9bc:	65 61 64 5f 	*unknown*
     9c0:	63 62 00 5f 	*unknown*
     9c4:	6f 72 31 6b 	l.lwa r27,12651(r18)
     9c8:	5f 75 61 72 	*unknown*
     9cc:	74 5f 69 6e 	*unknown*
     9d0:	69 74 00 5f 	*unknown*
     9d4:	6f 72 31 6b 	l.lwa r27,12651(r18)
     9d8:	5f 62 6f 61 	*unknown*
     9dc:	72 64 5f 63 	*unknown*
     9e0:	6c 6b 5f 66 	l.lwa r3,24422(r11)
     9e4:	72 65 71 00 	*unknown*
     9e8:	5f 6f 72 31 	*unknown*
     9ec:	6b 5f 62 6f 	*unknown*
     9f0:	61 72 64 5f 	*unknown*
     9f4:	75 61 72 74 	*unknown*
     9f8:	5f 62 61 73 	*unknown*
     9fc:	65 00 64 69 	*unknown*
     a00:	76 69 73 6f 	*unknown*
     a04:	72 00 6f 72 	*unknown*
     a08:	31 6b 5f 69 	*unknown*
     a0c:	6e 74 65 72 	l.lwa r19,25970(r20)
     a10:	72 75 70 74 	*unknown*
     a14:	5f 68 61 6e 	*unknown*
     a18:	64 6c 65 72 	*unknown*
     a1c:	5f 66 70 74 	*unknown*
     a20:	72 00 5f 6f 	*unknown*
     a24:	72 31 6b 5f 	*unknown*
     a28:	62 6f 61 72 	*unknown*
     a2c:	64 5f 75 61 	*unknown*
     a30:	72 74 5f 49 	*unknown*
     a34:	52 51 00 5f 	*unknown*
     a38:	6f 72 31 6b 	l.lwa r27,12651(r18)
     a3c:	5f 75 61 72 	*unknown*
     a40:	74 5f 72 65 	*unknown*
     a44:	61 64 5f 63 	*unknown*
     a48:	62 00 6f 72 	*unknown*
     a4c:	31 6b 5f 69 	*unknown*
     a50:	6e 74 65 72 	l.lwa r19,25970(r20)
     a54:	72 75 70 74 	*unknown*
     a58:	5f 68 61 6e 	*unknown*
     a5c:	64 6c 65 72 	*unknown*
     a60:	5f 61 64 64 	*unknown*
     a64:	00 2e 2e 2f 	l.j b8c320 <_end+0xb878b4>
     a68:	2e 2e 2f 2e 	*unknown*
     a6c:	2e 2f 2e 2e 	*unknown*
     a70:	2f 6e 65 77 	*unknown*
     a74:	6c 69 62 2d 	l.lwa r3,25133(r9)
     a78:	32 2e 32 2e 	*unknown*
     a7c:	30 2e 32 30 	*unknown*
     a80:	31 35 30 38 	*unknown*
     a84:	32 34 2f 6c 	*unknown*
     a88:	69 62 67 6c 	*unknown*
     a8c:	6f 73 73 2f 	l.lwa r27,29487(r19)
     a90:	6f 72 31 6b 	l.lwa r27,12651(r18)
     a94:	2f 69 6e 74 	*unknown*
     a98:	65 72 72 75 	*unknown*
     a9c:	70 74 73 2e 	*unknown*
     aa0:	63 00 6f 6c 	*unknown*
     aa4:	64 73 72 00 	*unknown*
     aa8:	64 61 74 61 	*unknown*
     aac:	5f 70 74 72 	*unknown*
     ab0:	00 5f 6f 72 	l.j 17dc878 <_end+0x17d7e0c>
     ab4:	31 6b 5f 69 	*unknown*
     ab8:	6e 74 65 72 	l.lwa r19,25970(r20)
     abc:	72 75 70 74 	*unknown*
     ac0:	5f 68 61 6e 	*unknown*
     ac4:	64 6c 65 72 	*unknown*
     ac8:	5f 64 61 74 	*unknown*
     acc:	61 5f 70 74 	*unknown*
     ad0:	72 5f 74 61 	*unknown*
     ad4:	62 6c 65 00 	*unknown*
     ad8:	6f 72 31 6b 	l.lwa r27,12651(r18)
     adc:	5f 69 6e 74 	*unknown*
     ae0:	65 72 72 75 	*unknown*
     ae4:	70 74 73 5f 	*unknown*
     ae8:	64 69 73 61 	*unknown*
     aec:	62 6c 65 00 	*unknown*
     af0:	5f 6f 72 31 	*unknown*
     af4:	6b 5f 69 6e 	*unknown*
     af8:	74 65 72 72 	*unknown*
     afc:	75 70 74 5f 	*unknown*
     b00:	68 61 6e 64 	*unknown*
     b04:	6c 65 72 5f 	l.lwa r3,29279(r5)
     b08:	74 61 62 6c 	*unknown*
     b0c:	65 00 6f 72 	*unknown*
     b10:	31 6b 5f 69 	*unknown*
     b14:	6e 74 65 72 	l.lwa r19,25970(r20)
     b18:	72 75 70 74 	*unknown*
     b1c:	73 5f 65 6e 	*unknown*
     b20:	61 62 6c 65 	*unknown*
     b24:	00 6f 72 31 	l.j 1bdd3e8 <_end+0x1bd897c>
     b28:	6b 5f 6d 74 	*unknown*
     b2c:	73 70 72 00 	*unknown*
     b30:	6f 72 31 6b 	l.lwa r27,12651(r18)
     b34:	5f 6d 66 73 	*unknown*
     b38:	70 72 00 6f 	*unknown*
     b3c:	72 31 6b 5f 	*unknown*
     b40:	69 6e 74 65 	*unknown*
     b44:	72 72 75 70 	*unknown*
     b48:	74 5f 68 61 	*unknown*
     b4c:	6e 64 6c 65 	l.lwa r19,27749(r4)
     b50:	72 5f 64 61 	*unknown*
     b54:	74 61 5f 70 	*unknown*
     b58:	74 72 5f 74 	*unknown*
     b5c:	61 62 6c 65 	*unknown*
     b60:	5f 74 00 6f 	*unknown*
     b64:	72 31 6b 5f 	*unknown*
     b68:	69 6e 74 65 	*unknown*
     b6c:	72 72 75 70 	*unknown*
     b70:	74 5f 68 61 	*unknown*
     b74:	6e 64 6c 65 	l.lwa r19,27749(r4)
     b78:	72 5f 74 61 	*unknown*
     b7c:	62 6c 65 5f 	*unknown*
     b80:	74 00 6f 72 	*unknown*
     b84:	31 6b 5f 69 	*unknown*
     b88:	6e 74 65 72 	l.lwa r19,25970(r20)
     b8c:	72 75 70 74 	*unknown*
     b90:	73 5f 72 65 	*unknown*
     b94:	73 74 6f 72 	*unknown*
     b98:	65 00 73 72 	*unknown*
     b9c:	5f 69 65 65 	*unknown*
     ba0:	00 6e 65 77 	l.j 1b9a17c <_end+0x1b95710>
     ba4:	73 72 00 2e 	*unknown*
     ba8:	2e 2f 2e 2e 	*unknown*
     bac:	2f 2e 2e 2f 	*unknown*
     bb0:	2e 2e 2f 6e 	*unknown*
     bb4:	65 77 6c 69 	*unknown*
     bb8:	62 2d 32 2e 	*unknown*
     bbc:	32 2e 30 2e 	*unknown*
     bc0:	32 30 31 35 	*unknown*
     bc4:	30 38 32 34 	*unknown*
     bc8:	2f 6c 69 62 	*unknown*
     bcc:	67 6c 6f 73 	*unknown*
     bd0:	73 2f 6f 72 	*unknown*
     bd4:	31 6b 2f 69 	*unknown*
     bd8:	6d 70 75 72 	l.lwa r11,30066(r16)
     bdc:	65 2e 63 00 	*unknown*
     be0:	5f 6f 72 31 	*unknown*
     be4:	6b 5f 6c 69 	*unknown*
     be8:	62 63 5f 67 	*unknown*
     bec:	65 74 72 65 	*unknown*
     bf0:	65 6e 74 00 	*unknown*
     bf4:	6f 72 31 6b 	l.lwa r27,12651(r18)
     bf8:	5f 74 69 6d 	*unknown*
     bfc:	65 72 5f 70 	*unknown*
     c00:	65 72 69 6f 	*unknown*
     c04:	64 00 5f 6f 	*unknown*
     c08:	72 31 6b 5f 	*unknown*
     c0c:	72 65 65 6e 	*unknown*
     c10:	74 00 5f 6f 	*unknown*
     c14:	72 31 6b 5f 	*unknown*
     c18:	65 78 63 65 	*unknown*
     c1c:	70 74 69 6f 	*unknown*
     c20:	6e 5f 69 6d 	l.lwa r18,26989(r31)
     c24:	70 75 72 65 	*unknown*
     c28:	5f 70 74 72 	*unknown*
     c2c:	00 5f 6f 72 	l.j 17dc9f4 <_end+0x17d7f88>
     c30:	31 6b 5f 6c 	*unknown*
     c34:	69 62 63 5f 	*unknown*
     c38:	69 6d 70 75 	*unknown*
     c3c:	72 65 5f 69 	*unknown*
     c40:	6e 69 74 00 	l.lwa r19,29696(r9)
     c44:	6f 72 31 6b 	l.lwa r27,12651(r18)
     c48:	5f 74 69 6d 	*unknown*
     c4c:	65 72 5f 6d 	*unknown*
     c50:	6f 64 65 00 	l.lwa r27,25856(r4)
     c54:	5f 6f 72 31 	*unknown*
     c58:	6b 5f 65 78 	*unknown*
     c5c:	63 65 70 74 	*unknown*
     c60:	69 6f 6e 5f 	*unknown*
     c64:	69 6d 70 75 	*unknown*
     c68:	72 65 5f 64 	*unknown*
     c6c:	61 74 61 00 	*unknown*
     c70:	5f 6f 72 31 	*unknown*
     c74:	6b 5f 72 65 	*unknown*
     c78:	65 6e 74 5f 	*unknown*
     c7c:	69 6e 69 74 	*unknown*
     c80:	00 5f 6f 72 	l.j 17dca48 <_end+0x17d7fdc>
     c84:	31 6b 5f 69 	*unknown*
     c88:	6d 70 75 72 	l.lwa r11,30066(r16)
     c8c:	65 5f 70 74 	*unknown*
     c90:	72 00 6d 65 	*unknown*
     c94:	6d 73 65 74 	l.lwa r11,25972(r19)
     c98:	00 6f 72 31 	l.j 1bdd55c <_end+0x1bd8af0>
     c9c:	6b 5f 74 69 	*unknown*
     ca0:	6d 65 72 5f 	l.lwa r11,29279(r5)
     ca4:	74 69 63 6b 	*unknown*
     ca8:	73 00 5f 6f 	*unknown*
     cac:	72 31 6b 5f 	*unknown*
     cb0:	63 75 72 72 	*unknown*
     cb4:	65 6e 74 5f 	*unknown*
     cb8:	69 6d 70 75 	*unknown*
     cbc:	72 65 5f 70 	*unknown*
     cc0:	74 72 00 6f 	*unknown*
     cc4:	72 31 6b 5f 	*unknown*
     cc8:	74 69 6d 65 	*unknown*
     ccc:	72 5f 64 69 	*unknown*
     cd0:	73 61 62 6c 	*unknown*
     cd4:	65 00 5f 6f 	*unknown*
     cd8:	72 31 6b 5f 	*unknown*
     cdc:	65 78 63 65 	*unknown*
     ce0:	70 74 69 6f 	*unknown*
     ce4:	6e 5f 73 74 	l.lwa r18,29556(r31)
     ce8:	61 63 6b 5f 	*unknown*
     cec:	74 6f 70 00 	*unknown*
     cf0:	5f 6f 72 31 	*unknown*
     cf4:	6b 5f 65 78 	*unknown*
     cf8:	63 65 70 74 	*unknown*
     cfc:	69 6f 6e 5f 	*unknown*
     d00:	6c 65 76 65 	l.lwa r3,30309(r5)
     d04:	6c 00 6f 72 	l.lwa r0,28530(r0)
     d08:	31 6b 5f 63 	*unknown*
     d0c:	72 69 74 69 	*unknown*
     d10:	63 61 6c 5f 	*unknown*
     d14:	65 6e 64 00 	*unknown*
     d18:	5f 6f 72 31 	*unknown*
     d1c:	6b 5f 65 78 	*unknown*
     d20:	63 65 70 74 	*unknown*
     d24:	69 6f 6e 5f 	*unknown*
     d28:	73 74 61 63 	*unknown*
     d2c:	6b 5f 62 6f 	*unknown*
     d30:	74 74 6f 6d 	*unknown*
     d34:	00 6f 72 31 	l.j 1bdd5f8 <_end+0x1bd8b8c>
     d38:	6b 5f 65 78 	*unknown*
     d3c:	63 65 70 74 	*unknown*
     d40:	69 6f 6e 5f 	*unknown*
     d44:	68 61 6e 64 	*unknown*
     d48:	6c 65 72 5f 	l.lwa r3,29279(r5)
     d4c:	66 70 74 72 	*unknown*
     d50:	00 5f 6f 72 	l.j 17dcb18 <_end+0x17d80ac>
     d54:	31 6b 5f 69 	*unknown*
     d58:	6e 69 74 00 	l.lwa r19,29696(r9)
     d5c:	5f 6f 72 31 	*unknown*
     d60:	6b 5f 73 74 	*unknown*
     d64:	61 63 6b 5f 	*unknown*
     d68:	62 6f 74 74 	*unknown*
     d6c:	6f 6d 00 2e 	l.lwa r27,46(r13)
     d70:	2e 2f 2e 2e 	*unknown*
     d74:	2f 2e 2e 2f 	*unknown*
     d78:	2e 2e 2f 6e 	*unknown*
     d7c:	65 77 6c 69 	*unknown*
     d80:	62 2d 32 2e 	*unknown*
     d84:	32 2e 30 2e 	*unknown*
     d88:	32 30 31 35 	*unknown*
     d8c:	30 38 32 34 	*unknown*
     d90:	2f 6c 69 62 	*unknown*
     d94:	67 6c 6f 73 	*unknown*
     d98:	73 2f 6f 72 	*unknown*
     d9c:	31 6b 2f 75 	*unknown*
     da0:	74 69 6c 2e 	*unknown*
     da4:	63 00 6f 72 	*unknown*
     da8:	31 6b 5f 65 	*unknown*
     dac:	78 63 65 70 	*unknown*
     db0:	74 69 6f 6e 	*unknown*
     db4:	5f 68 61 6e 	*unknown*
     db8:	64 6c 65 72 	*unknown*
     dbc:	5f 74 61 62 	*unknown*
     dc0:	6c 65 5f 74 	l.lwa r3,24436(r5)
     dc4:	00 5f 6f 72 	l.j 17dcb8c <_end+0x17d8120>
     dc8:	31 6b 5f 73 	*unknown*
     dcc:	74 61 63 6b 	*unknown*
     dd0:	5f 74 6f 70 	*unknown*
     dd4:	00 6f 72 31 	l.j 1bdd698 <_end+0x1bd8c2c>
     dd8:	6b 5f 74 69 	*unknown*
     ddc:	6d 65 72 5f 	l.lwa r11,29279(r5)
     de0:	72 65 73 74 	*unknown*
     de4:	6f 72 65 00 	l.lwa r27,25856(r18)
     de8:	6f 72 31 6b 	l.lwa r27,12651(r18)
     dec:	5f 63 72 69 	*unknown*
     df0:	74 69 63 61 	*unknown*
     df4:	6c 5f 62 65 	l.lwa r2,25189(r31)
     df8:	67 69 6e 00 	*unknown*
     dfc:	5f 6f 72 31 	*unknown*
     e00:	6b 5f 65 78 	*unknown*
     e04:	63 65 70 74 	*unknown*
     e08:	69 6f 6e 5f 	*unknown*
     e0c:	68 61 6e 64 	*unknown*
     e10:	6c 65 72 5f 	l.lwa r3,29279(r5)
     e14:	74 61 62 6c 	*unknown*
     e18:	65 00 2e 2e 	*unknown*
     e1c:	2f 2e 2e 2f 	*unknown*
     e20:	2e 2e 2f 2e 	*unknown*
     e24:	2e 2f 6e 65 	*unknown*
     e28:	77 6c 69 62 	*unknown*
     e2c:	2d 32 2e 32 	*unknown*
     e30:	2e 30 2e 32 	*unknown*
     e34:	30 31 35 30 	*unknown*
     e38:	38 32 34 2f 	*unknown*
     e3c:	6c 69 62 67 	l.lwa r3,25191(r9)
     e40:	6c 6f 73 73 	l.lwa r3,29555(r15)
     e44:	2f 6f 72 31 	*unknown*
     e48:	6b 2f 65 78 	*unknown*
     e4c:	63 65 70 74 	*unknown*
     e50:	69 6f 6e 73 	*unknown*
     e54:	2e 63 00 6f 	*unknown*
     e58:	72 31 6b 5f 	*unknown*
     e5c:	65 78 63 65 	*unknown*
     e60:	70 74 69 6f 	*unknown*
     e64:	6e 5f 68 61 	l.lwa r18,26721(r31)
     e68:	6e 64 6c 65 	l.lwa r19,27749(r4)
     e6c:	72 5f 61 64 	*unknown*
     e70:	64 00 6f 72 	*unknown*
     e74:	31 6b 5f 74 	*unknown*
     e78:	69 6d 65 72 	*unknown*
     e7c:	5f 73 65 74 	*unknown*
     e80:	5f 68 61 6e 	*unknown*
     e84:	64 6c 65 72 	*unknown*
     e88:	00 6f 72 31 	l.j 1bdd74c <_end+0x1bd8ce0>
     e8c:	6b 5f 74 69 	*unknown*
     e90:	6d 65 72 5f 	l.lwa r11,29279(r5)
     e94:	67 65 74 5f 	*unknown*
     e98:	74 69 63 6b 	*unknown*
     e9c:	73 00 6f 72 	*unknown*
     ea0:	31 6b 5f 74 	*unknown*
     ea4:	69 6d 65 72 	*unknown*
     ea8:	5f 73 65 74 	*unknown*
     eac:	5f 6d 6f 64 	*unknown*
     eb0:	65 00 6f 72 	*unknown*
     eb4:	31 6b 5f 74 	*unknown*
     eb8:	69 6d 65 72 	*unknown*
     ebc:	5f 69 6e 69 	*unknown*
     ec0:	74 00 6f 72 	*unknown*
     ec4:	31 6b 5f 74 	*unknown*
     ec8:	69 6d 65 72 	*unknown*
     ecc:	5f 70 61 75 	*unknown*
     ed0:	73 65 00 5f 	*unknown*
     ed4:	6f 72 31 6b 	l.lwa r27,12651(r18)
     ed8:	5f 74 69 6d 	*unknown*
     edc:	65 72 5f 69 	*unknown*
     ee0:	6e 74 65 72 	l.lwa r19,25970(r20)
     ee4:	72 75 70 74 	*unknown*
     ee8:	5f 68 61 6e 	*unknown*
     eec:	64 6c 65 72 	*unknown*
     ef0:	00 6f 72 31 	l.j 1bdd7b4 <_end+0x1bd8d48>
     ef4:	6b 5f 74 69 	*unknown*
     ef8:	6d 65 72 5f 	l.lwa r11,29279(r5)
     efc:	65 6e 61 62 	*unknown*
     f00:	6c 65 00 74 	l.lwa r3,116(r5)
     f04:	74 6d 72 00 	*unknown*
     f08:	73 72 5f 74 	*unknown*
     f0c:	65 65 00 2e 	*unknown*
     f10:	2e 2f 2e 2e 	*unknown*
     f14:	2f 2e 2e 2f 	*unknown*
     f18:	2e 2e 2f 6e 	*unknown*
     f1c:	65 77 6c 69 	*unknown*
     f20:	62 2d 32 2e 	*unknown*
     f24:	32 2e 30 2e 	*unknown*
     f28:	32 30 31 35 	*unknown*
     f2c:	30 38 32 34 	*unknown*
     f30:	2f 6c 69 62 	*unknown*
     f34:	67 6c 6f 73 	*unknown*
     f38:	73 2f 6f 72 	*unknown*
     f3c:	31 6b 2f 74 	*unknown*
     f40:	69 6d 65 72 	*unknown*
     f44:	2e 63 00 6f 	*unknown*
     f48:	72 31 6b 5f 	*unknown*
     f4c:	74 69 6d 65 	*unknown*
     f50:	72 5f 72 65 	*unknown*
     f54:	73 65 74 00 	*unknown*
     f58:	6f 72 31 6b 	l.lwa r27,12651(r18)
     f5c:	5f 74 69 6d 	*unknown*
     f60:	65 72 5f 72 	*unknown*
     f64:	65 73 65 74 	*unknown*
     f68:	5f 74 69 63 	*unknown*
     f6c:	6b 73 00 6f 	*unknown*
     f70:	72 31 6b 5f 	*unknown*
     f74:	74 69 6d 65 	*unknown*
     f78:	72 5f 73 65 	*unknown*
     f7c:	74 5f 70 65 	*unknown*
     f80:	72 69 6f 64 	*unknown*
     f84:	00 5f 5f 67 	l.j 17d8d20 <_end+0x17d42b4>
     f88:	65 74 72 65 	*unknown*
     f8c:	65 6e 74 00 	*unknown*
     f90:	2f 68 6f 6d 	*unknown*
     f94:	65 2f 61 6e 	*unknown*
     f98:	64 72 7a 65 	*unknown*
     f9c:	6a 72 2f 44 	*unknown*
     fa0:	69 67 69 74 	*unknown*
     fa4:	61 6c 2f 4f 	*unknown*
     fa8:	52 50 53 6f 	*unknown*
     fac:	43 2f 62 75 	*unknown*
     fb0:	69 6c 64 2d 	*unknown*
     fb4:	6e 65 77 6c 	l.lwa r19,30572(r5)
     fb8:	69 62 2f 6f 	*unknown*
     fbc:	72 31 6b 2d 	*unknown*
     fc0:	65 6c 66 2f 	*unknown*
     fc4:	6e 65 77 6c 	l.lwa r19,30572(r5)
     fc8:	69 62 2f 6c 	*unknown*
     fcc:	69 62 63 2f 	*unknown*
     fd0:	65 72 72 6e 	*unknown*
     fd4:	6f 00 2e 2e 	l.lwa r24,11822(r0)
     fd8:	2f 2e 2e 2f 	*unknown*
     fdc:	2e 2e 2f 2e 	*unknown*
     fe0:	2e 2f 2e 2e 	*unknown*
     fe4:	2f 6e 65 77 	*unknown*
     fe8:	6c 69 62 2d 	l.lwa r3,25133(r9)
     fec:	32 2e 32 2e 	*unknown*
     ff0:	30 2e 32 30 	*unknown*
     ff4:	31 35 30 38 	*unknown*
     ff8:	32 34 2f 6e 	*unknown*
     ffc:	65 77 6c 69 	*unknown*
    1000:	62 2f 6c 69 	*unknown*
    1004:	62 63 2f 65 	*unknown*
    1008:	72 72 6e 6f 	*unknown*
    100c:	2f 65 72 72 	*unknown*
    1010:	6e 6f 2e 63 	l.lwa r19,11875(r15)
    1014:	00 2e 2e 2f 	l.j b8c8d0 <_end+0xb87e64>
    1018:	2e 2e 2f 2e 	*unknown*
    101c:	2e 2f 2e 2e 	*unknown*
    1020:	2f 2e 2e 2f 	*unknown*
    1024:	6e 65 77 6c 	l.lwa r19,30572(r5)
    1028:	69 62 2d 32 	*unknown*
    102c:	2e 32 2e 30 	*unknown*
    1030:	2e 32 30 31 	*unknown*
    1034:	35 30 38 32 	*unknown*
    1038:	34 2f 6e 65 	*unknown*
    103c:	77 6c 69 62 	*unknown*
    1040:	2f 6c 69 62 	*unknown*
    1044:	63 2f 73 74 	*unknown*
    1048:	72 69 6e 67 	*unknown*
    104c:	2f 6d 65 6d 	*unknown*
    1050:	73 65 74 2e 	*unknown*
    1054:	63 00 61 6c 	*unknown*
    1058:	69 67 6e 65 	*unknown*
    105c:	64 5f 61 64 	*unknown*
    1060:	64 72 00 62 	*unknown*
    1064:	75 66 66 65 	*unknown*
    1068:	72 00 2f 68 	*unknown*
    106c:	6f 6d 65 2f 	l.lwa r27,25903(r13)
    1070:	61 6e 64 72 	*unknown*
    1074:	7a 65 6a 72 	*unknown*
    1078:	2f 44 69 67 	*unknown*
    107c:	69 74 61 6c 	*unknown*
    1080:	2f 4f 52 50 	*unknown*
    1084:	53 6f 43 2f 	*unknown*
    1088:	62 75 69 6c 	*unknown*
    108c:	64 2d 6e 65 	*unknown*
    1090:	77 6c 69 62 	*unknown*
    1094:	2f 6f 72 31 	*unknown*
    1098:	6b 2d 65 6c 	*unknown*
    109c:	66 2f 6e 65 	*unknown*
    10a0:	77 6c 69 62 	*unknown*
    10a4:	2f 6c 69 62 	*unknown*
    10a8:	63 2f 73 74 	*unknown*
    10ac:	72 69 6e 67 	*unknown*
	...

Disassembly of section .debug_loc:

00000000 <.debug_loc>:
   0:	00 00 00 00 	l.j 0 <_or1k_reset-0x100>
   4:	00 00 00 08 	l.j 24 <_or1k_reset-0xdc>
   8:	00 01 53 00 	l.j 54c08 <_end+0x5019c>
   c:	00 00 08 00 	l.j 200c <_init+0x8>
  10:	00 00 1f 00 	l.j 7c10 <_end+0x31a4>
  14:	01 54 00 00 	l.j 5500014 <_end+0x54fb5a8>
  18:	00 1f 00 00 	l.j 7c0018 <_end+0x7bb5ac>
  1c:	00 30 00 04 	l.j c0002c <_end+0xbfb5c0>
  20:	f3 01 53 9f 	*unknown*
	...
  30:	00 00 00 1b 	l.j 9c <_or1k_reset-0x64>
  34:	00 01 53 00 	l.j 54c34 <_end+0x501c8>
  38:	00 00 1b 00 	l.j 6c38 <_end+0x21cc>
  3c:	00 00 48 00 	l.j 1203c <_end+0xd5d0>
  40:	01 52 00 00 	l.j 5480040 <_end+0x547b5d4>
	...
  50:	00 64 00 01 	l.j 1900054 <_end+0x18fb5e8>
  54:	53 00 00 00 	*unknown*
  58:	64 00 00 00 	*unknown*
  5c:	98 00 01 66 	l.lhs r0,358(r0)
  60:	00 00 00 98 	l.j 2c0 <_or1k_reset+0x1c0>
  64:	00 00 00 a0 	l.j 2e4 <_or1k_reset+0x1e4>
  68:	00 04 f3 01 	l.j 13cc6c <_end+0x138200>
  6c:	53 9f 00 00 	*unknown*
  70:	00 a0 00 00 	l.j 2800070 <_end+0x27fb604>
  74:	01 2c 00 01 	l.j 4b00078 <_end+0x4afb60c>
  78:	66 00 00 01 	*unknown*
  7c:	2c 00 00 01 	*unknown*
  80:	38 00 01 53 	*unknown*
  84:	00 00 01 38 	l.j 564 <_or1k_reset+0x464>
  88:	00 00 01 40 	l.j 588 <_or1k_reset+0x488>
  8c:	00 01 66 00 	l.j 5988c <_end+0x54e20>
	...
  9c:	00 00 64 00 	l.j 1909c <_end+0x14630>
  a0:	01 54 00 00 	l.j 55000a0 <_end+0x54fb634>
  a4:	00 64 00 00 	l.j 19000a4 <_end+0x18fb638>
  a8:	00 8c 00 01 	l.j 23000ac <_end+0x22fb640>
  ac:	52 00 00 00 	*unknown*
  b0:	8c 00 00 00 	l.lbz r0,0(r0)
  b4:	a0 00 04 f3 	l.addic r0,r0,1267
  b8:	01 54 9f 00 	l.j 5527cb8 <_end+0x552324c>
  bc:	00 00 a0 00 	l.j 280bc <_end+0x23650>
  c0:	00 01 40 00 	l.j 500c0 <_end+0x4b654>
  c4:	01 52 00 00 	l.j 54800c4 <_end+0x547b658>
	...
  d4:	00 5c 00 01 	l.j 17000d8 <_end+0x16fb66c>
  d8:	55 00 00 00 	*unknown*
  dc:	5c 00 00 00 	*unknown*
  e0:	98 00 01 64 	l.lhs r0,356(r0)
  e4:	00 00 00 98 	l.j 344 <_or1k_reset+0x244>
  e8:	00 00 00 a0 	l.j 368 <_or1k_reset+0x268>
  ec:	00 04 f3 01 	l.j 13ccf0 <_end+0x138284>
  f0:	55 9f 00 00 	*unknown*
  f4:	00 a0 00 00 	l.j 28000f4 <_end+0x27fb688>
  f8:	00 b7 00 01 	l.j 2dc00fc <_end+0x2dbb690>
  fc:	55 00 00 00 	*unknown*
 100:	b7 00 00 01 	l.mfspr r24,r0,0x1
 104:	2c 00 01 64 	*unknown*
 108:	00 00 01 2c 	l.j 5b8 <_or1k_reset+0x4b8>
 10c:	00 00 01 38 	l.j 5ec <_or1k_reset+0x4ec>
 110:	00 01 55 00 	l.j 55510 <_end+0x50aa4>
 114:	00 01 38 00 	l.j 4e114 <_end+0x496a8>
 118:	00 01 40 00 	l.j 50118 <_end+0x4b6ac>
 11c:	01 64 00 00 	l.j 590011c <_end+0x58fb6b0>
	...
 12c:	00 64 00 01 	l.j 1900130 <_end+0x18fb6c4>
 130:	56 00 00 00 	*unknown*
 134:	64 00 00 00 	*unknown*
 138:	94 00 01 62 	l.lhz r0,354(r0)
 13c:	00 00 00 94 	l.j 38c <_or1k_reset+0x28c>
 140:	00 00 00 a0 	l.j 3c0 <_or1k_reset+0x2c0>
 144:	00 04 f3 01 	l.j 13cd48 <_end+0x1382dc>
 148:	56 9f 00 00 	*unknown*
 14c:	00 a0 00 00 	l.j 280014c <_end+0x27fb6e0>
 150:	00 b7 00 01 	l.j 2dc0154 <_end+0x2dbb6e8>
 154:	56 00 00 00 	*unknown*
 158:	b7 00 00 01 	l.mfspr r24,r0,0x1
 15c:	2c 00 01 62 	*unknown*
 160:	00 00 01 2c 	l.j 610 <_or1k_reset+0x510>
 164:	00 00 01 38 	l.j 644 <_or1k_reset+0x544>
 168:	00 01 56 00 	l.j 55968 <_end+0x50efc>
 16c:	00 01 38 00 	l.j 4e16c <_end+0x49700>
 170:	00 01 40 00 	l.j 50170 <_end+0x4b704>
 174:	01 62 00 00 	l.j 5880174 <_end+0x587b708>
	...
 180:	00 f0 00 00 	l.j 3c00180 <_end+0x3bfb714>
 184:	01 2c 00 04 	l.j 4b00194 <_end+0x4afb728>
 188:	78 88 01 9f 	*unknown*
	...
 194:	00 00 00 20 	l.j 214 <_or1k_reset+0x114>
 198:	00 00 00 b7 	l.j 474 <_or1k_reset+0x374>
 19c:	00 01 58 00 	l.j 5619c <_end+0x51730>
 1a0:	00 00 bc 00 	l.j 2f1a0 <_end+0x2a734>
 1a4:	00 01 40 00 	l.j 501a4 <_end+0x4b738>
 1a8:	01 58 00 00 	l.j 56001a8 <_end+0x55fb73c>
	...
 1b8:	00 54 00 01 	l.j 15001bc <_end+0x14fb750>
 1bc:	53 00 00 00 	*unknown*
 1c0:	54 00 00 01 	*unknown*
 1c4:	90 00 01 6a 	l.lbs r0,362(r0)
 1c8:	00 00 01 90 	l.j 808 <_or1k_reset+0x708>
 1cc:	00 00 01 9c 	l.j 83c <_or1k_reset+0x73c>
 1d0:	00 04 f3 01 	l.j 13cdd4 <_end+0x138368>
 1d4:	53 9f 00 00 	*unknown*
 1d8:	01 9c 00 00 	l.j 67001d8 <_end+0x66fb76c>
 1dc:	01 e8 00 01 	l.j 7a001e0 <_end+0x79fb774>
 1e0:	6a 00 00 00 	*unknown*
	...
 1f0:	54 00 01 54 	*unknown*
 1f4:	00 00 00 54 	l.j 344 <_or1k_reset+0x244>
 1f8:	00 00 01 80 	l.j 7f8 <_or1k_reset+0x6f8>
 1fc:	00 01 62 00 	l.j 589fc <_end+0x53f90>
 200:	00 01 80 00 	l.j 60200 <_end+0x5b794>
 204:	00 01 9c 00 	l.j 67204 <_end+0x62798>
 208:	04 f3 01 54 	l.jal 3cc0758 <_end+0x3cbbcec>
 20c:	9f 00 00 01 	l.addi r24,r0,1
 210:	9c 00 00 01 	l.addi r0,r0,1
 214:	e8 00 01 62 	*unknown*
	...
 220:	00 00 00 58 	l.j 380 <_or1k_reset+0x280>
 224:	00 00 01 84 	l.j 834 <_or1k_reset+0x734>
 228:	00 01 64 00 	l.j 59228 <_end+0x547bc>
 22c:	00 01 9c 00 	l.j 6722c <_end+0x627c0>
 230:	00 01 e0 00 	l.j 78230 <_end+0x737c4>
 234:	01 64 00 00 	l.j 5900234 <_end+0x58fb7c8>
 238:	01 e0 00 00 	l.j 7800238 <_end+0x77fb7cc>
 23c:	01 e8 00 01 	l.j 7a00240 <_end+0x79fb7d4>
 240:	52 00 00 00 	*unknown*
	...
 24c:	58 00 00 00 	*unknown*
 250:	64 00 02 71 	*unknown*
 254:	00 00 00 00 	l.j 254 <_or1k_reset+0x154>
 258:	64 00 00 01 	*unknown*
 25c:	68 00 01 66 	*unknown*
 260:	00 00 01 9c 	l.j 8d0 <_or1k_reset+0x7d0>
 264:	00 00 01 e8 	l.j a04 <_or1k_reset+0x904>
 268:	00 01 66 00 	l.j 59a68 <_end+0x54ffc>
	...
 274:	00 00 64 00 	l.j 19274 <_end+0x14808>
 278:	00 01 5c 00 	l.j 57278 <_end+0x5280c>
 27c:	04 84 88 01 	l.jal 2122280 <_end+0x211d814>
 280:	9f 00 00 01 	l.addi r24,r0,1
 284:	9c 00 00 01 	l.addi r0,r0,1
 288:	e0 00 04 84 	*unknown*
 28c:	88 01 9f 00 	l.lws r0,-24832(r1)
 290:	00 01 e0 00 	l.j 78290 <_end+0x73824>
 294:	00 01 e8 00 	l.j 7a294 <_end+0x75828>
 298:	04 86 88 01 	l.jal 21a229c <_end+0x219d830>
 29c:	9f 00 00 00 	l.addi r24,r0,0
	...
 2a8:	6c 00 00 01 	l.lwa r0,1(r0)
 2ac:	68 00 01 5e 	*unknown*
 2b0:	00 00 01 9c 	l.j 920 <_or1k_reset+0x820>
 2b4:	00 00 01 e8 	l.j a54 <_or1k_reset+0x954>
 2b8:	00 01 5e 00 	l.j 57ab8 <_end+0x5304c>
	...
 2c4:	00 00 88 00 	l.j 222c4 <_end+0x1d858>
 2c8:	00 00 9c 00 	l.j 272c8 <_end+0x2285c>
 2cc:	05 31 7e 00 	l.jal 4c5facc <_end+0x4c5b060>
 2d0:	24 9f 00 00 	*unknown*
 2d4:	00 9c 00 00 	l.j 27002d4 <_end+0x26fb868>
 2d8:	00 a8 00 05 	l.j 2a002ec <_end+0x29fb880>
 2dc:	31 7e 01 24 	*unknown*
 2e0:	9f 00 00 00 	l.addi r24,r0,0
 2e4:	a8 00 00 01 	l.ori r0,r0,0x1
 2e8:	1c 00 05 31 	*unknown*
 2ec:	7e 00 24 9f 	*unknown*
 2f0:	00 00 01 1c 	l.j 760 <_or1k_reset+0x660>
 2f4:	00 00 01 28 	l.j 794 <_or1k_reset+0x694>
 2f8:	00 05 31 7e 	l.j 14c8f0 <_end+0x147e84>
 2fc:	01 24 9f 00 	l.j 4927efc <_end+0x4923490>
 300:	00 01 9c 00 	l.j 67300 <_end+0x62894>
 304:	00 01 dc 00 	l.j 77304 <_end+0x72898>
 308:	05 31 7e 00 	l.jal 4c5fb08 <_end+0x4c5b09c>
 30c:	24 9f 00 00 	*unknown*
	...
 318:	00 b8 00 00 	l.j 2e00318 <_end+0x2dfb8ac>
 31c:	00 fb 00 01 	l.j 3ec0320 <_end+0x3ebb8b4>
 320:	55 00 00 01 	*unknown*
 324:	9c 00 00 01 	l.addi r0,r0,1
 328:	bb 00 01 55 	*unknown*
 32c:	00 00 01 c4 	l.j a3c <_or1k_reset+0x93c>
 330:	00 00 01 d3 	l.j a7c <_or1k_reset+0x97c>
 334:	00 01 55 00 	l.j 55734 <_end+0x50cc8>
	...
 340:	00 00 dc 00 	l.j 37340 <_end+0x328d4>
 344:	00 01 28 00 	l.j 4a344 <_end+0x458d8>
 348:	01 68 00 00 	l.j 5a00348 <_end+0x59fb8dc>
 34c:	01 9c 00 00 	l.j 670034c <_end+0x66fb8e0>
 350:	01 c4 00 01 	l.j 7100354 <_end+0x70fb8e8>
 354:	68 00 00 01 	*unknown*
 358:	cc 00 00 01 	l.swa 1(r0),r0
 35c:	dc 00 01 68 	l.sh 360(r0),r0
	...
 36c:	00 00 00 34 	l.j 43c <_or1k_reset+0x33c>
 370:	00 01 53 00 	l.j 54f70 <_end+0x50504>
 374:	00 00 34 00 	l.j d374 <_end+0x8908>
 378:	00 00 98 00 	l.j 26378 <_end+0x2190c>
 37c:	04 f3 01 53 	l.jal 3cc08c8 <_end+0x3cbbe5c>
 380:	9f 00 00 00 	l.addi r24,r0,0
	...
 390:	34 00 01 54 	*unknown*
 394:	00 00 00 34 	l.j 464 <_or1k_reset+0x364>
 398:	00 00 00 98 	l.j 5f8 <_or1k_reset+0x4f8>
 39c:	00 04 f3 01 	l.j 13cfa0 <_end+0x138534>
 3a0:	54 9f 00 00 	*unknown*
	...
 3b0:	00 34 00 01 	l.j d003b4 <_end+0xcfb948>
 3b4:	55 00 00 00 	*unknown*
 3b8:	34 00 00 00 	*unknown*
 3bc:	98 00 04 f3 	l.lhs r0,1267(r0)
 3c0:	01 55 9f 00 	l.j 5567fc0 <_end+0x5563554>
	...
 3d0:	00 00 34 00 	l.j d3d0 <_end+0x8964>
 3d4:	01 56 00 00 	l.j 55803d4 <_end+0x557b968>
 3d8:	00 34 00 00 	l.j d003d8 <_end+0xcfb96c>
 3dc:	00 90 00 01 	l.j 24003e0 <_end+0x23fb974>
 3e0:	62 00 00 00 	*unknown*
 3e4:	90 00 00 00 	l.lbs r0,0(r0)
 3e8:	98 00 01 5b 	l.lhs r0,347(r0)
	...
 3f8:	00 00 00 34 	l.j 4c8 <_or1k_reset+0x3c8>
 3fc:	00 02 30 9f 	l.j 8c678 <_end+0x87c0c>
 400:	00 00 00 34 	l.j 4d0 <_or1k_reset+0x3d0>
 404:	00 00 00 3c 	l.j 4f4 <_or1k_reset+0x3f4>
 408:	00 08 f3 01 	l.j 23d00c <_end+0x2385a0>
 40c:	55 20 72 00 	*unknown*
 410:	22 9f 00 00 	*unknown*
 414:	00 48 00 00 	l.j 1200414 <_end+0x11fb9a8>
 418:	00 5c 00 07 	l.j 1700434 <_end+0x16fb9c8>
 41c:	72 00 f3 01 	*unknown*
 420:	55 1c 9f 00 	*unknown*
 424:	00 00 5c 00 	l.j 17424 <_end+0x129b8>
 428:	00 00 6c 00 	l.j 1b428 <_end+0x169bc>
 42c:	08 f3 01 55 	*unknown*
 430:	20 72 00 22 	*unknown*
 434:	9f 00 00 00 	l.addi r24,r0,0
	...
 440:	98 00 00 00 	l.lhs r0,0(r0)
 444:	a7 00 01 53 	l.andi r24,r0,0x153
 448:	00 00 00 a7 	l.j 6e4 <_or1k_reset+0x5e4>
 44c:	00 00 00 b0 	l.j 70c <_or1k_reset+0x60c>
 450:	00 04 f3 01 	l.j 13d054 <_end+0x1385e8>
 454:	53 9f 00 00 	*unknown*
	...
 460:	00 b0 00 00 	l.j 2c00460 <_end+0x2bfb9f4>
 464:	00 bc 00 01 	l.j 2f00468 <_end+0x2efb9fc>
 468:	54 00 00 00 	*unknown*
 46c:	bc 00 00 00 	l.sfeqi r0,0
 470:	d0 00 04 f3 	*unknown*
 474:	01 54 9f 00 	l.j 5528074 <_end+0x5523608>
	...
 480:	00 00 d0 00 	l.j 34480 <_end+0x2fa14>
 484:	00 00 dc 00 	l.j 37484 <_end+0x32a18>
 488:	01 54 00 00 	l.j 5500488 <_end+0x54fba1c>
 48c:	00 dc 00 00 	l.j 370048c <_end+0x36fba20>
 490:	00 f0 00 04 	l.j 3c004a0 <_end+0x3bfba34>
 494:	f3 01 54 9f 	*unknown*
	...
 4a0:	00 00 00 f0 	l.j 860 <_or1k_reset+0x760>
 4a4:	00 00 00 ff 	l.j 8a0 <_or1k_reset+0x7a0>
 4a8:	00 01 53 00 	l.j 550a8 <_end+0x5063c>
 4ac:	00 00 ff 00 	l.j 400ac <_end+0x3b640>
 4b0:	00 01 1c 00 	l.j 474b0 <_end+0x42a44>
 4b4:	04 f3 01 53 	l.jal 3cc0a00 <_end+0x3cbbf94>
 4b8:	9f 00 00 00 	l.addi r24,r0,0
 4bc:	00 00 00 00 	l.j 4bc <_or1k_reset+0x3bc>
 4c0:	00 00 00 01 	l.j 4c4 <_or1k_reset+0x3c4>
 4c4:	1c 00 00 01 	*unknown*
 4c8:	28 00 01 54 	*unknown*
 4cc:	00 00 01 28 	l.j 96c <_or1k_reset+0x86c>
 4d0:	00 00 01 3c 	l.j 9c0 <_or1k_reset+0x8c0>
 4d4:	00 04 f3 01 	l.j 13d0d8 <_end+0x13866c>
 4d8:	54 9f 00 00 	*unknown*
	...
 4e4:	01 5c 00 00 	l.j 57004e4 <_end+0x56fba78>
 4e8:	01 68 00 01 	l.j 5a004ec <_end+0x59fba80>
 4ec:	54 00 00 01 	*unknown*
 4f0:	68 00 00 01 	*unknown*
 4f4:	7c 00 04 f3 	*unknown*
 4f8:	01 54 9f 00 	l.j 55280f8 <_end+0x552368c>
	...
 504:	00 01 7c 00 	l.j 5f504 <_end+0x5aa98>
 508:	00 01 88 00 	l.j 62508 <_end+0x5da9c>
 50c:	01 54 00 00 	l.j 550050c <_end+0x54fbaa0>
 510:	01 88 00 00 	l.j 6200510 <_end+0x61fbaa4>
 514:	01 9c 00 04 	l.j 6700524 <_end+0x66fbab8>
 518:	f3 01 54 9f 	*unknown*
	...
 524:	00 00 01 9c 	l.j b94 <_or1k_reset+0xa94>
 528:	00 00 01 a8 	l.j bc8 <_or1k_reset+0xac8>
 52c:	00 01 54 00 	l.j 5552c <_end+0x50ac0>
 530:	00 01 a8 00 	l.j 6a530 <_end+0x65ac4>
 534:	00 01 bc 00 	l.j 6f534 <_end+0x6aac8>
 538:	04 f3 01 54 	l.jal 3cc0a88 <_end+0x3cbc01c>
 53c:	9f 00 00 00 	l.addi r24,r0,0
 540:	00 00 00 00 	l.j 540 <_or1k_reset+0x440>
 544:	00 00 00 01 	l.j 548 <_or1k_reset+0x448>
 548:	bc 00 00 01 	l.sfeqi r0,1
 54c:	c8 00 01 54 	*unknown*
 550:	00 00 01 c8 	l.j c70 <_or1k_reset+0xb70>
 554:	00 00 01 dc 	l.j cc4 <_or1k_reset+0xbc4>
 558:	00 04 f3 01 	l.j 13d15c <_end+0x1386f0>
 55c:	54 9f 00 00 	*unknown*
	...
 568:	01 dc 00 00 	l.j 7700568 <_end+0x76fbafc>
 56c:	01 eb 00 01 	l.j 7ac0570 <_end+0x7abbb04>
 570:	53 00 00 01 	*unknown*
 574:	eb 00 00 02 	*unknown*
 578:	08 00 04 f3 	*unknown*
 57c:	01 53 9f 00 	l.j 54e817c <_end+0x54e3710>
	...
 588:	00 01 dc 00 	l.j 77588 <_end+0x72b1c>
 58c:	00 01 eb 00 	l.j 7b18c <_end+0x76720>
 590:	01 54 00 00 	l.j 5500590 <_end+0x54fbb24>
 594:	01 eb 00 00 	l.j 7ac0594 <_end+0x7abbb28>
 598:	02 08 00 04 	l.j f82005a8 <_end+0xf81fbb3c>
 59c:	f3 01 54 9f 	*unknown*
	...
 5a8:	00 00 01 dc 	l.j d18 <_or1k_reset+0xc18>
 5ac:	00 00 01 eb 	l.j d58 <_or1k_reset+0xc58>
 5b0:	00 01 55 00 	l.j 559b0 <_end+0x50f44>
 5b4:	00 01 eb 00 	l.j 7b1b4 <_end+0x76748>
 5b8:	00 02 08 00 	l.j 825b8 <_end+0x7db4c>
 5bc:	04 f3 01 55 	l.jal 3cc0b10 <_end+0x3cbc0a4>
 5c0:	9f 00 00 00 	l.addi r24,r0,0
 5c4:	00 00 00 00 	l.j 5c4 <_or1k_reset+0x4c4>
 5c8:	00 00 00 01 	l.j 5cc <_or1k_reset+0x4cc>
 5cc:	dc 00 00 01 	l.sh 1(r0),r0
 5d0:	eb 00 01 56 	*unknown*
 5d4:	00 00 01 eb 	l.j d80 <_or1k_reset+0xc80>
 5d8:	00 00 02 08 	l.j df8 <_or1k_reset+0xcf8>
 5dc:	00 04 f3 01 	l.j 13d1e0 <_end+0x138774>
 5e0:	56 9f 00 00 	*unknown*
	...
 5ec:	02 08 00 00 	l.j f82005ec <_end+0xf81fbb80>
 5f0:	02 14 00 01 	l.j f85005f4 <_end+0xf84fbb88>
 5f4:	54 00 00 02 	*unknown*
 5f8:	14 00 00 02 	*unknown*
 5fc:	28 00 04 f3 	*unknown*
 600:	01 54 9f 00 	l.j 5528200 <_end+0x5523794>
	...
 60c:	00 02 28 00 	l.j 8a60c <_end+0x85ba0>
 610:	00 02 34 00 	l.j 8d610 <_end+0x88ba4>
 614:	01 54 00 00 	l.j 5500614 <_end+0x54fbba8>
 618:	02 34 00 00 	l.j f8d00618 <_end+0xf8cfbbac>
 61c:	02 48 00 04 	l.j f920062c <_end+0xf91fbbc0>
 620:	f3 01 54 9f 	*unknown*
	...
 62c:	00 00 02 48 	l.j f4c <_or1k_reset+0xe4c>
 630:	00 00 02 54 	l.j f80 <_or1k_reset+0xe80>
 634:	00 01 54 00 	l.j 55634 <_end+0x50bc8>
 638:	00 02 54 00 	l.j 95638 <_end+0x90bcc>
 63c:	00 02 68 00 	l.j 9a63c <_end+0x95bd0>
 640:	04 f3 01 54 	l.jal 3cc0b90 <_end+0x3cbc124>
 644:	9f 00 00 00 	l.addi r24,r0,0
 648:	00 00 00 00 	l.j 648 <_or1k_reset+0x548>
 64c:	00 00 00 02 	l.j 654 <_or1k_reset+0x554>
 650:	68 00 00 02 	*unknown*
 654:	74 00 01 54 	*unknown*
 658:	00 00 02 74 	l.j 1028 <_or1k_reset+0xf28>
 65c:	00 00 02 88 	l.j 107c <_or1k_reset+0xf7c>
 660:	00 04 f3 01 	l.j 13d264 <_end+0x1387f8>
 664:	54 9f 00 00 	*unknown*
	...
 670:	02 88 00 00 	l.j fa200670 <_end+0xfa1fbc04>
 674:	02 94 00 01 	l.j fa500678 <_end+0xfa4fbc0c>
 678:	54 00 00 02 	*unknown*
 67c:	94 00 00 02 	l.lhz r0,2(r0)
 680:	a8 00 04 f3 	l.ori r0,r0,0x4f3
 684:	01 54 9f 00 	l.j 5528284 <_end+0x5523818>
	...
 694:	00 00 04 00 	l.j 1694 <_or1k_reset+0x1594>
 698:	01 53 00 00 	l.j 54c0698 <_end+0x54bbc2c>
 69c:	00 04 00 00 	l.j 10069c <_end+0xfbc30>
 6a0:	00 28 00 04 	l.j a006b0 <_end+0x9fbc44>
 6a4:	f3 01 53 9f 	*unknown*
	...
 6b0:	00 00 00 84 	l.j 8c0 <_or1k_reset+0x7c0>
 6b4:	00 00 00 94 	l.j 904 <_or1k_reset+0x804>
 6b8:	00 01 5b 00 	l.j 572b8 <_end+0x5284c>
	...
 6c4:	00 01 00 00 	l.j 406c4 <_end+0x3bc58>
 6c8:	00 01 08 00 	l.j 426c8 <_end+0x3dc5c>
 6cc:	01 53 00 00 	l.j 54c06cc <_end+0x54bbc60>
 6d0:	01 08 00 00 	l.j 42006d0 <_end+0x41fbc64>
 6d4:	01 48 00 04 	l.j 52006e4 <_end+0x51fbc78>
 6d8:	f3 01 53 9f 	*unknown*
	...
 6e4:	00 00 01 48 	l.j c04 <_or1k_reset+0xb04>
 6e8:	00 00 01 90 	l.j d28 <_or1k_reset+0xc28>
 6ec:	00 01 53 00 	l.j 552ec <_end+0x50880>
 6f0:	00 01 90 00 	l.j 646f0 <_end+0x5fc84>
 6f4:	00 01 b4 00 	l.j 6d6f4 <_end+0x68c88>
 6f8:	04 f3 01 53 	l.jal 3cc0c44 <_end+0x3cbc1d8>
 6fc:	9f 00 00 00 	l.addi r24,r0,0
	...
 70c:	0c 00 01 53 	l.bnf c58 <_or1k_reset+0xb58>
 710:	00 00 00 0c 	l.j 740 <_or1k_reset+0x640>
 714:	00 00 00 40 	l.j 814 <_or1k_reset+0x714>
 718:	00 04 f3 01 	l.j 13d31c <_end+0x1388b0>
 71c:	53 9f 00 00 	*unknown*
	...
 728:	00 50 00 00 	l.j 1400728 <_end+0x13fbcbc>
 72c:	00 54 00 05 	l.j 1500740 <_end+0x14fbcd4>
 730:	73 00 34 21 	*unknown*
 734:	9f 00 00 00 	l.addi r24,r0,0
 738:	54 00 00 00 	*unknown*
 73c:	64 00 01 53 	*unknown*
	...
 748:	00 00 00 50 	l.j 888 <_or1k_reset+0x788>
 74c:	00 00 00 54 	l.j 89c <_or1k_reset+0x79c>
 750:	00 01 53 00 	l.j 55350 <_end+0x508e4>
	...
 75c:	00 00 78 00 	l.j 1e75c <_end+0x19cf0>
 760:	00 00 80 00 	l.j 20760 <_end+0x1bcf4>
 764:	06 7b 00 09 	l.jal f9ec0788 <_end+0xf9ebbd1c>
 768:	fb 1a 9f 00 	*unknown*
 76c:	00 00 80 00 	l.j 2076c <_end+0x1bd00>
 770:	00 00 9c 00 	l.j 27770 <_end+0x22d04>
 774:	01 54 00 00 	l.j 5500774 <_end+0x54fbd08>
	...
 780:	00 78 00 00 	l.j 1e00780 <_end+0x1dfbd14>
 784:	00 8c 00 01 	l.j 2300788 <_end+0x22fbd1c>
 788:	5b 00 00 00 	*unknown*
	...
 794:	9c 00 00 00 	l.addi r0,r0,0
 798:	cc 00 01 53 	l.swa 339(r0),r0
 79c:	00 00 00 cc 	l.j acc <_or1k_reset+0x9cc>
 7a0:	00 00 00 e4 	l.j b30 <_or1k_reset+0xa30>
 7a4:	00 04 f3 01 	l.j 13d3a8 <_end+0x13893c>
 7a8:	53 9f 00 00 	*unknown*
	...
 7b4:	00 c8 00 00 	l.j 32007b4 <_end+0x31fbd48>
 7b8:	00 cc 00 06 	l.j 33007d0 <_end+0x32fbd64>
 7bc:	75 00 74 00 	*unknown*
 7c0:	21 9f 00 00 	*unknown*
 7c4:	00 cc 00 00 	l.j 33007c4 <_end+0x32fbd58>
 7c8:	00 e4 00 01 	l.j 39007cc <_end+0x38fbd60>
 7cc:	53 00 00 00 	*unknown*
	...
 7d8:	b0 00 00 00 	l.muli r0,r0,0
 7dc:	bc 00 01 54 	l.sfeqi r0,340
	...
 7e8:	00 00 00 60 	l.j 968 <_or1k_reset+0x868>
 7ec:	00 00 00 70 	l.j 9ac <_or1k_reset+0x8ac>
 7f0:	00 01 52 00 	l.j 54ff0 <_end+0x50584>
	...
 7fc:	00 00 68 00 	l.j 1a7fc <_end+0x15d90>
 800:	00 00 7c 00 	l.j 1f800 <_end+0x1ad94>
 804:	01 5b 00 00 	l.j 56c0804 <_end+0x56bbd98>
	...
 810:	00 84 00 00 	l.j 2100810 <_end+0x20fbda4>
 814:	00 98 00 01 	l.j 2600818 <_end+0x25fbdac>
 818:	53 00 00 00 	*unknown*
 81c:	98 00 00 00 	l.lhs r0,0(r0)
 820:	b8 00 01 52 	*unknown*
 824:	00 00 00 b8 	l.j b04 <_or1k_reset+0xa04>
 828:	00 00 00 c0 	l.j b28 <_or1k_reset+0xa28>
 82c:	00 04 f3 01 	l.j 13d430 <_end+0x1389c4>
 830:	53 9f 00 00 	*unknown*
	...
 83c:	33 50 00 00 	*unknown*
 840:	33 50 00 01 	*unknown*
 844:	53 00 00 33 	*unknown*
 848:	50 00 00 33 	*unknown*
 84c:	54 00 03 73 	*unknown*
 850:	7e 9f 00 00 	*unknown*
 854:	33 54 00 00 	*unknown*
 858:	33 60 00 01 	*unknown*
 85c:	53 00 00 33 	*unknown*
 860:	60 00 00 33 	*unknown*
 864:	84 00 06 f3 	l.lwz r0,1779(r0)
 868:	01 53 32 1c 	l.j 54cd0d8 <_end+0x54c866c>
 86c:	9f 00 00 00 	l.addi r24,r0,0
	...
 878:	28 00 00 00 	*unknown*
 87c:	34 00 0d 73 	*unknown*
 880:	00 0c 1f ff 	l.j 30887c <_end+0x303e10>
 884:	ff ff 1a 48 	*unknown*
 888:	4a 24 21 9f 	*unknown*
 88c:	00 00 00 34 	l.j 95c <_or1k_reset+0x85c>
 890:	00 00 00 3c 	l.j 980 <_or1k_reset+0x880>
 894:	00 07 73 00 	l.j 1dd494 <_end+0x1d8a28>
 898:	48 4a 24 21 	*unknown*
 89c:	9f 00 00 00 	l.addi r24,r0,0
 8a0:	3c 00 00 00 	*unknown*
 8a4:	50 00 01 53 	*unknown*
	...
 8b0:	00 00 00 28 	l.j 950 <_or1k_reset+0x850>
 8b4:	00 00 00 34 	l.j 984 <_or1k_reset+0x884>
 8b8:	00 01 53 00 	l.j 554b8 <_end+0x50a4c>
	...
 8c4:	00 00 34 00 	l.j d8c4 <_end+0x8e58>
 8c8:	00 00 3c 00 	l.j f8c8 <_end+0xae5c>
 8cc:	07 73 00 48 	l.jal fdcc09ec <_end+0xfdcbbf80>
 8d0:	4a 24 21 9f 	*unknown*
 8d4:	00 00 00 3c 	l.j 9c4 <_or1k_reset+0x8c4>
 8d8:	00 00 00 50 	l.j a18 <_or1k_reset+0x918>
 8dc:	00 01 53 00 	l.j 554dc <_end+0x50a70>
	...
 8e8:	00 00 50 00 	l.j 148e8 <_end+0xfe7c>
 8ec:	00 00 94 00 	l.j 258ec <_end+0x20e80>
 8f0:	01 53 00 00 	l.j 54c08f0 <_end+0x54bbe84>
 8f4:	00 94 00 00 	l.j 25008f4 <_end+0x24fbe88>
 8f8:	00 9b 00 01 	l.j 26c08fc <_end+0x26bbe90>
 8fc:	54 00 00 00 	*unknown*
 900:	9b 00 00 00 	l.lhs r24,0(r0)
 904:	fc 00 04 f3 	*unknown*
 908:	01 53 9f 00 	l.j 54e8508 <_end+0x54e3a9c>
 90c:	00 00 fc 00 	l.j 3f90c <_end+0x3aea0>
 910:	00 01 04 00 	l.j 41910 <_end+0x3cea4>
 914:	01 53 00 00 	l.j 54c0914 <_end+0x54bbea8>
	...
 920:	00 a8 00 00 	l.j 2a00920 <_end+0x29fbeb4>
 924:	00 cf 00 01 	l.j 33c0928 <_end+0x33bbebc>
 928:	5b 00 00 00 	*unknown*
	...
 934:	70 00 00 00 	l.cust1
 938:	74 00 01 55 	*unknown*
	...
 944:	00 00 00 b0 	l.j c04 <_or1k_reset+0xb04>
 948:	00 00 00 cf 	l.j c84 <_or1k_reset+0xb84>
 94c:	00 01 5b 00 	l.j 5754c <_end+0x52ae0>
	...
 958:	00 00 b0 00 	l.j 2c958 <_end+0x27eec>
 95c:	00 00 e0 00 	l.j 3895c <_end+0x33ef0>
 960:	04 0a 50 00 	l.jal 294960 <_end+0x28fef4>
 964:	9f 00 00 00 	l.addi r24,r0,0
	...
 970:	d8 00 00 00 	l.sb 0(r0),r0
 974:	e0 00 02 30 	*unknown*
 978:	9f 00 00 00 	l.addi r24,r0,0
	...
 984:	d8 00 00 00 	l.sb 0(r0),r0
 988:	e0 00 04 0a 	*unknown*
 98c:	50 01 9f 00 	*unknown*
	...
 998:	00 01 04 00 	l.j 41998 <_end+0x3cf2c>
 99c:	00 01 0c 00 	l.j 4399c <_end+0x3ef30>
 9a0:	01 53 00 00 	l.j 54c09a0 <_end+0x54bbf34>
 9a4:	01 0c 00 00 	l.j 43009a4 <_end+0x42fbf38>
 9a8:	01 2f 00 01 	l.j 4bc09ac <_end+0x4bbbf40>
 9ac:	54 00 00 01 	*unknown*
 9b0:	2f 00 00 01 	*unknown*
 9b4:	6c 00 04 f3 	l.lwa r0,1267(r0)
 9b8:	01 53 9f 00 	l.j 54e85b8 <_end+0x54e3b4c>
	...
 9c4:	00 01 44 00 	l.j 519c4 <_end+0x4cf58>
 9c8:	00 01 48 00 	l.j 529c8 <_end+0x4df5c>
 9cc:	06 73 00 7b 	l.jal f9cc0bb8 <_end+0xf9cbc14c>
 9d0:	00 21 9f 00 	l.j 8685d0 <_end+0x863b64>
 9d4:	00 01 48 00 	l.j 529d4 <_end+0x4df68>
 9d8:	00 01 50 00 	l.j 549d8 <_end+0x4ff6c>
 9dc:	01 53 00 00 	l.j 54c09dc <_end+0x54bbf70>
	...
 9e8:	01 3c 00 00 	l.j 4f009e8 <_end+0x4efbf7c>
 9ec:	01 44 00 01 	l.j 51009f0 <_end+0x50fbf84>
 9f0:	53 00 00 00 	*unknown*
 9f4:	00 00 00 00 	l.j 9f4 <_or1k_reset+0x8f4>
 9f8:	00 00 00 01 	l.j 9fc <_or1k_reset+0x8fc>
 9fc:	6c 00 00 01 	l.lwa r0,1(r0)
 a00:	7c 00 01 53 	*unknown*
 a04:	00 00 01 7c 	l.j ff4 <_or1k_reset+0xef4>
 a08:	00 00 01 83 	l.j 1014 <_or1k_reset+0xf14>
 a0c:	00 01 54 00 	l.j 55a0c <_end+0x50fa0>
 a10:	00 01 83 00 	l.j 61610 <_end+0x5cba4>
 a14:	00 01 94 00 	l.j 65a14 <_end+0x60fa8>
 a18:	04 f3 01 53 	l.jal 3cc0f64 <_end+0x3cbc4f8>
 a1c:	9f 00 00 00 	l.addi r24,r0,0
 a20:	00 00 00 00 	l.j a20 <_or1k_reset+0x920>
 a24:	00 00 00 01 	l.j a28 <_or1k_reset+0x928>
 a28:	94 00 00 01 	l.lhz r0,1(r0)
 a2c:	c8 00 01 53 	*unknown*
	...
 a38:	00 00 01 b4 	l.j 1108 <_or1k_reset+0x1008>
 a3c:	00 00 01 d0 	l.j 117c <_or1k_reset+0x107c>
 a40:	00 01 54 00 	l.j 55a40 <_end+0x50fd4>
	...
 a4c:	00 01 d0 00 	l.j 74a4c <_end+0x6ffe0>
 a50:	00 01 d8 00 	l.j 76a50 <_end+0x71fe4>
 a54:	04 0a 50 00 	l.jal 294a54 <_end+0x28ffe8>
 a58:	9f 00 00 00 	l.addi r24,r0,0
 a5c:	00 00 00 00 	l.j a5c <_or1k_reset+0x95c>
 a60:	00 00 00 01 	l.j a64 <_or1k_reset+0x964>
 a64:	fc 00 00 02 	*unknown*
 a68:	08 00 07 73 	*unknown*
 a6c:	00 40 49 24 	l.j 1012efc <_end+0x100e490>
 a70:	21 9f 00 00 	*unknown*
	...
 a7c:	02 30 00 00 	l.j f8c00a7c <_end+0xf8bfc010>
 a80:	02 34 00 05 	l.j f8d00a94 <_end+0xf8cfc028>
 a84:	73 00 32 21 	*unknown*
 a88:	9f 00 00 02 	l.addi r24,r0,2
 a8c:	34 00 00 02 	*unknown*
 a90:	48 00 01 53 	*unknown*
	...
 a9c:	00 00 01 fc 	l.j 128c <_or1k_reset+0x118c>
 aa0:	00 00 02 0c 	l.j 12d0 <_or1k_reset+0x11d0>
 aa4:	00 01 53 00 	l.j 556a4 <_end+0x50c38>
	...
 ab0:	00 02 30 00 	l.j 8cab0 <_end+0x88044>
 ab4:	00 02 34 00 	l.j 8dab4 <_end+0x89048>
 ab8:	01 53 00 00 	l.j 54c0ab8 <_end+0x54bc04c>
	...
 ac4:	02 5c 00 00 	l.j f9700ac4 <_end+0xf96fc058>
 ac8:	02 64 00 06 	l.j f9900ae0 <_end+0xf98fc074>
 acc:	7b 00 09 fd 	*unknown*
 ad0:	1a 9f 00 00 	*unknown*
 ad4:	02 64 00 00 	l.j f9900ad4 <_end+0xf98fc068>
 ad8:	02 80 00 01 	l.j fa000adc <_end+0xf9ffc070>
 adc:	54 00 00 00 	*unknown*
 ae0:	00 00 00 00 	l.j ae0 <_or1k_reset+0x9e0>
 ae4:	00 00 00 02 	l.j aec <_or1k_reset+0x9ec>
 ae8:	5c 00 00 02 	*unknown*
 aec:	70 00 01 5b 	*unknown*
	...
 af8:	00 00 02 80 	l.j 14f8 <_or1k_reset+0x13f8>
 afc:	00 00 02 90 	l.j 153c <_or1k_reset+0x143c>
 b00:	00 01 53 00 	l.j 55700 <_end+0x50c94>
 b04:	00 02 90 00 	l.j a4b04 <_end+0xa0098>
 b08:	00 02 a4 00 	l.j a9b08 <_end+0xa509c>
 b0c:	04 f3 01 53 	l.jal 3cc1058 <_end+0x3cbc5ec>
 b10:	9f 00 00 00 	l.addi r24,r0,0
 b14:	00 00 00 00 	l.j b14 <_or1k_reset+0xa14>
 b18:	00 00 00 02 	l.j b20 <_or1k_reset+0xa20>
 b1c:	90 00 00 02 	l.lbs r0,2(r0)
 b20:	94 00 05 73 	l.lhz r0,1395(r0)
 b24:	00 32 21 9f 	l.j c891a0 <_end+0xc84734>
 b28:	00 00 02 94 	l.j 1578 <_or1k_reset+0x1478>
 b2c:	00 00 02 a4 	l.j 15bc <_or1k_reset+0x14bc>
 b30:	00 01 53 00 	l.j 55730 <_end+0x50cc4>
	...
 b3c:	00 02 90 00 	l.j a4b3c <_end+0xa00d0>
 b40:	00 02 94 00 	l.j a5b40 <_end+0xa10d4>
 b44:	01 53 00 00 	l.j 54c0b44 <_end+0x54bc0d8>
	...
 b50:	02 b8 00 00 	l.j fae00b50 <_end+0xfadfc0e4>
 b54:	02 c4 00 09 	l.j fb100b78 <_end+0xfb0fc10c>
 b58:	73 00 0c 3f 	*unknown*
 b5c:	ff ff ff 1a 	*unknown*
 b60:	9f 00 00 00 	l.addi r24,r0,0
 b64:	00 00 00 00 	l.j b64 <_or1k_reset+0xa64>
 b68:	00 00 00 02 	l.j b70 <_or1k_reset+0xa70>
 b6c:	b8 00 00 02 	l.slli r0,r0,0x2
 b70:	c4 00 01 53 	*unknown*
	...
 b7c:	00 00 02 ec 	l.j 172c <_or1k_reset+0x162c>
 b80:	00 00 02 f8 	l.j 1760 <_or1k_reset+0x1660>
 b84:	00 0a 73 00 	l.j 29d784 <_end+0x298d18>
 b88:	11 ff ff ff 	l.bf 8000b84 <_end+0x7ffc118>
 b8c:	ff 7e 1a 9f 	*unknown*
	...
 b98:	00 00 02 ec 	l.j 1748 <_or1k_reset+0x1648>
 b9c:	00 00 02 f8 	l.j 177c <_or1k_reset+0x167c>
 ba0:	00 01 53 00 	l.j 557a0 <_end+0x50d34>
	...
 bb0:	00 01 30 00 	l.j 4cbb0 <_end+0x48144>
 bb4:	01 54 00 00 	l.j 5500bb4 <_end+0x54fc148>
 bb8:	01 30 00 00 	l.j 4c00bb8 <_end+0x4bfc14c>
 bbc:	01 60 00 04 	l.j 5800bcc <_end+0x57fc160>
 bc0:	f3 01 54 9f 	*unknown*
 bc4:	00 00 01 60 	l.j 1144 <_or1k_reset+0x1044>
 bc8:	00 00 01 68 	l.j 1168 <_or1k_reset+0x1068>
 bcc:	00 01 54 00 	l.j 55bcc <_end+0x51160>
	...
 bdc:	00 00 1c 00 	l.j 7bdc <_end+0x3170>
 be0:	01 55 00 00 	l.j 5540be0 <_end+0x553c174>
 be4:	00 1c 00 00 	l.j 700be4 <_end+0x6fc178>
 be8:	00 38 00 01 	l.j e00bec <_end+0xdfc180>
 bec:	55 00 00 00 	*unknown*
 bf0:	38 00 00 00 	*unknown*
 bf4:	44 00 01 5c 	*unknown*
 bf8:	00 00 00 44 	l.j d08 <_or1k_reset+0xc08>
 bfc:	00 00 00 54 	l.j d4c <_or1k_reset+0xc4c>
 c00:	00 01 55 00 	l.j 56000 <_end+0x51594>
 c04:	00 00 54 00 	l.j 15c04 <_end+0x11198>
 c08:	00 00 60 00 	l.j 18c08 <_end+0x1419c>
 c0c:	01 5c 00 00 	l.j 5700c0c <_end+0x56fc1a0>
 c10:	00 f4 00 00 	l.j 3d00c10 <_end+0x3cfc1a4>
 c14:	01 20 00 01 	l.j 4800c18 <_end+0x47fc1ac>
 c18:	58 00 00 01 	*unknown*
 c1c:	60 00 00 01 	*unknown*
 c20:	68 00 01 55 	*unknown*
	...
 c30:	00 00 00 38 	l.j d10 <_or1k_reset+0xc10>
 c34:	00 01 53 00 	l.j 55834 <_end+0x50dc8>
 c38:	00 00 38 00 	l.j ec38 <_end+0xa1cc>
 c3c:	00 00 44 00 	l.j 11c3c <_end+0xd1d0>
 c40:	01 56 00 00 	l.j 5580c40 <_end+0x557c1d4>
 c44:	00 44 00 00 	l.j 1100c44 <_end+0x10fc1d8>
 c48:	00 48 00 03 	l.j 1200c54 <_end+0x11fc1e8>
 c4c:	76 01 9f 00 	*unknown*
 c50:	00 00 48 00 	l.j 12c50 <_end+0xe1e4>
 c54:	00 00 60 00 	l.j 18c54 <_end+0x141e8>
 c58:	01 56 00 00 	l.j 5580c58 <_end+0x557c1ec>
 c5c:	01 20 00 00 	l.j 4800c5c <_end+0x47fc1f0>
 c60:	01 38 00 01 	l.j 4e00c64 <_end+0x4dfc1f8>
 c64:	56 00 00 01 	*unknown*
 c68:	38 00 00 01 	*unknown*
 c6c:	40 00 03 76 	*unknown*
 c70:	01 9f 00 00 	l.j 67c0c70 <_end+0x67bc204>
 c74:	01 40 00 00 	l.j 5000c74 <_end+0x4ffc208>
 c78:	01 4c 00 01 	l.j 5300c7c <_end+0x52fc210>
 c7c:	56 00 00 01 	*unknown*
 c80:	60 00 00 01 	*unknown*
 c84:	68 00 01 53 	*unknown*
	...
 c90:	00 00 00 80 	l.j e90 <_or1k_reset+0xd90>
 c94:	00 00 01 20 	l.j 1114 <_or1k_reset+0x1014>
 c98:	00 03 08 20 	l.j c2d18 <_end+0xbe2ac>
 c9c:	9f 00 00 00 	l.addi r24,r0,0
	...
 ca8:	7c 00 00 01 	*unknown*
 cac:	0c 00 01 57 	l.bnf 1208 <_or1k_reset+0x1108>
 cb0:	00 00 01 0c 	l.j 10e0 <_or1k_reset+0xfe0>
 cb4:	00 00 01 20 	l.j 1134 <_or1k_reset+0x1034>
 cb8:	00 02 7c 7c 	l.j 9fea8 <_end+0x9b43c>
	...
 cc4:	00 00 00 70 	l.j e84 <_or1k_reset+0xd84>
 cc8:	00 00 00 ac 	l.j f78 <_or1k_reset+0xe78>
 ccc:	00 01 56 00 	l.j 564cc <_end+0x51a60>
 cd0:	00 00 ac 00 	l.j 2bcd0 <_end+0x27264>
 cd4:	00 00 b0 00 	l.j 2ccd4 <_end+0x28268>
 cd8:	03 7c 04 9f 	l.j fdf01f54 <_end+0xfdefd4e8>
 cdc:	00 00 00 b0 	l.j f9c <_or1k_reset+0xe9c>
 ce0:	00 00 00 b4 	l.j fb0 <_or1k_reset+0xeb0>
 ce4:	00 03 7c 08 	l.j dfd04 <_end+0xdb298>
 ce8:	9f 00 00 00 	l.addi r24,r0,0
 cec:	b4 00 00 00 	l.mfspr r0,r0,0x0
 cf0:	c8 00 03 7c 	*unknown*
 cf4:	0c 9f 00 00 	l.bnf 27c0cf4 <_end+0x27bc288>
 cf8:	00 c8 00 00 	l.j 3200cf8 <_end+0x31fc28c>
 cfc:	00 e4 00 01 	l.j 3900d00 <_end+0x38fc294>
 d00:	5c 00 00 00 	*unknown*
 d04:	e4 00 00 00 	l.sfeq r0,r0
 d08:	f4 00 01 56 	*unknown*
 d0c:	00 00 00 f4 	l.j 10dc <_or1k_reset+0xfdc>
 d10:	00 00 01 00 	l.j 1110 <_or1k_reset+0x1010>
 d14:	00 03 7c 04 	l.j dfd24 <_end+0xdb2b8>
 d18:	9f 00 00 01 	l.addi r24,r0,1
 d1c:	00 00 00 01 	l.j d20 <_or1k_reset+0xc20>
 d20:	20 00 01 5c 	l.sys 0x15c
	...
 d30:	00 00 01 30 	l.j 11f0 <_or1k_reset+0x10f0>
 d34:	00 06 74 00 	l.j 19dd34 <_end+0x1992c8>
 d38:	08 ff 1a 9f 	*unknown*
 d3c:	00 00 01 30 	l.j 11fc <_or1k_reset+0x10fc>
 d40:	00 00 01 60 	l.j 12c0 <_or1k_reset+0x11c0>
 d44:	00 07 f3 01 	l.j 1fd948 <_end+0x1f8edc>
 d48:	54 08 ff 1a 	*unknown*
 d4c:	9f 00 00 01 	l.addi r24,r0,1
 d50:	60 00 00 01 	*unknown*
 d54:	68 00 06 74 	*unknown*
 d58:	00 08 ff 1a 	l.j 2409c0 <_end+0x23bf54>
 d5c:	9f 00 00 00 	l.addi r24,r0,0
 d60:	00 00 00 00 	l.j d60 <_or1k_reset+0xc60>
	...

Disassembly of section .debug_ranges:

00000000 <.debug_ranges>:
   0:	00 00 00 88 	l.j 220 <_or1k_reset+0x120>
   4:	00 00 00 98 	l.j 264 <_or1k_reset+0x164>
   8:	00 00 00 a8 	l.j 2a8 <_or1k_reset+0x1a8>
   c:	00 00 01 1c 	l.j 47c <_or1k_reset+0x37c>
  10:	00 00 01 9c 	l.j 680 <_or1k_reset+0x580>
  14:	00 00 01 d4 	l.j 764 <_or1k_reset+0x664>
	...
  20:	00 00 00 44 	l.j 130 <_or1k_reset+0x30>
  24:	00 00 00 48 	l.j 144 <_or1k_reset+0x44>
  28:	00 00 00 4c 	l.j 158 <_or1k_reset+0x58>
  2c:	00 00 00 50 	l.j 16c <_or1k_reset+0x6c>
	...
  38:	00 00 00 6c 	l.j 1e8 <_or1k_reset+0xe8>
  3c:	00 00 00 70 	l.j 1fc <_or1k_reset+0xfc>
  40:	00 00 00 74 	l.j 210 <_or1k_reset+0x110>
  44:	00 00 00 78 	l.j 224 <_or1k_reset+0x124>
	...
  50:	00 00 00 a4 	l.j 2e0 <_or1k_reset+0x1e0>
  54:	00 00 00 a8 	l.j 2f4 <_or1k_reset+0x1f4>
  58:	00 00 00 ac 	l.j 308 <_or1k_reset+0x208>
  5c:	00 00 00 b0 	l.j 31c <_or1k_reset+0x21c>
	...
  68:	00 00 33 50 	l.j cda8 <_end+0x833c>
  6c:	00 00 33 84 	l.j ce7c <_end+0x8410>
	...
  78:	00 00 00 1c 	l.j e8 <_or1k_reset-0x18>
  7c:	00 00 00 20 	l.j fc <_or1k_reset-0x4>
  80:	00 00 00 24 	l.j 110 <_or1k_reset+0x10>
  84:	00 00 00 28 	l.j 124 <_or1k_reset+0x24>
	...
  90:	00 00 00 64 	l.j 220 <_or1k_reset+0x120>
  94:	00 00 00 68 	l.j 234 <_or1k_reset+0x134>
  98:	00 00 00 6c 	l.j 248 <_or1k_reset+0x148>
  9c:	00 00 00 70 	l.j 25c <_or1k_reset+0x15c>
	...
  a8:	00 00 00 a8 	l.j 348 <_or1k_reset+0x248>
  ac:	00 00 00 ac 	l.j 35c <_or1k_reset+0x25c>
  b0:	00 00 00 b0 	l.j 370 <_or1k_reset+0x270>
  b4:	00 00 00 b4 	l.j 384 <_or1k_reset+0x284>
	...
  c0:	00 00 00 d0 	l.j 400 <_or1k_reset+0x300>
  c4:	00 00 00 d4 	l.j 414 <_or1k_reset+0x314>
  c8:	00 00 00 d8 	l.j 428 <_or1k_reset+0x328>
  cc:	00 00 00 dc 	l.j 43c <_or1k_reset+0x33c>
	...
  d8:	00 00 01 30 	l.j 598 <_or1k_reset+0x498>
  dc:	00 00 01 34 	l.j 5ac <_or1k_reset+0x4ac>
  e0:	00 00 01 38 	l.j 5c0 <_or1k_reset+0x4c0>
  e4:	00 00 01 3c 	l.j 5d4 <_or1k_reset+0x4d4>
	...
  f0:	00 00 01 a4 	l.j 780 <_or1k_reset+0x680>
  f4:	00 00 01 a8 	l.j 794 <_or1k_reset+0x694>
  f8:	00 00 01 b0 	l.j 7b8 <_or1k_reset+0x6b8>
  fc:	00 00 01 b4 	l.j 7cc <_or1k_reset+0x6cc>
	...
 108:	00 00 01 f0 	l.j 8c8 <_or1k_reset+0x7c8>
 10c:	00 00 01 f4 	l.j 8dc <_or1k_reset+0x7dc>
 110:	00 00 01 f8 	l.j 8f0 <_or1k_reset+0x7f0>
 114:	00 00 01 fc 	l.j 904 <_or1k_reset+0x804>
	...
 120:	00 00 02 50 	l.j a60 <_or1k_reset+0x960>
 124:	00 00 02 54 	l.j a74 <_or1k_reset+0x974>
 128:	00 00 02 58 	l.j a88 <_or1k_reset+0x988>
 12c:	00 00 02 5c 	l.j a9c <_or1k_reset+0x99c>
	...
 138:	00 00 02 84 	l.j b48 <_or1k_reset+0xa48>
 13c:	00 00 02 88 	l.j b5c <_or1k_reset+0xa5c>
 140:	00 00 02 8c 	l.j b70 <_or1k_reset+0xa70>
 144:	00 00 02 90 	l.j b84 <_or1k_reset+0xa84>
	...
 150:	00 00 02 ac 	l.j c00 <_or1k_reset+0xb00>
 154:	00 00 02 b0 	l.j c14 <_or1k_reset+0xb14>
 158:	00 00 02 b4 	l.j c28 <_or1k_reset+0xb28>
 15c:	00 00 02 b8 	l.j c3c <_or1k_reset+0xb3c>
	...
 168:	00 00 02 e0 	l.j ce8 <_or1k_reset+0xbe8>
 16c:	00 00 02 e4 	l.j cfc <_or1k_reset+0xbfc>
 170:	00 00 02 e8 	l.j d10 <_or1k_reset+0xc10>
 174:	00 00 02 ec 	l.j d24 <_or1k_reset+0xc24>
	...
