;=============================================================================
;������� - �������� �������������� ����������
;=============================================================================

;***** ���� ���������� ���������� *****

#define		ToW		0			;��������� ��������� � W
#define		ToF		1			;��������� ��������� � F

;***** ������� ��� W *****

#define		incw		addlw	0x01
#define		decw		addlw	0xFF
#define		comw		xorlw	0xFF
#define		negw		sublw	0x00

;***** ������� ��� ������ � ����� *****

#define		clrz		bcf	STATUS,Z
#define		setz		bsf	STATUS,Z
#define		clrc		bcf	STATUS,C
#define		setc		bsf	STATUS,C
#define		clrb		bsf	STATUS,C
#define		setb		bcf	STATUS,C

;���������� ��������� ������� �� �������� ����
#define		If_1		btfsc			;���������� ��������� �������, ���� ��� = 1
#define		If_FB		btfsc
#define		If_0		btfss			;���������� ��������� �������, ���� ��� = 0
#define		If_NFB		btfss
#define		If_Z		btfsc	STATUS,Z	;���������� ��������� �������, ���� ��� Z = 1
#define		If_NZ		btfss	STATUS,Z	;���������� ��������� �������, ���� ��� Z = 0
#define		If_C		btfsc	STATUS,C	;���������� ��������� �������, ���� ��� C = 1
#define		If_NC		btfss	STATUS,C	;���������� ��������� �������, ���� ��� C = 0
#define		If_B		btfss	STATUS,C	;���������� ��������� �������, ���� ��� B = 1
#define		If_NB		btfsc	STATUS,C	;���������� ��������� �������, ���� ��� B = 0

;���������� ��������� ������� �� ���������� ������� cmp (�����: 2 ����� ��� ��������� � ����)
#define		If_2Lt1		If_B			;���������, ���� �������� 2 < ��������� 1
#define		If_2GE1		If_NB			;���������, ���� �������� 2 >= ��������� 1

;����������� ���� Z � W (����� �� ����������)
adzw		MACRO
		If_Z
		  incw
		ENDM

;����������� ���� C � W (��� C �������������� �� ��������, ��������� ����� �� ����������)
adcw		MACRO
		If_C
		  incw
		ENDM

;����������� ���� C � ������������� ����� (��� C �������������� �� ��������, ��������� ����� �� ����������)
adcf		MACRO	Port
		If_C
		  incf	Port,ToF
		ENDM

;����������� ���� C � ��������� ��� ������������� �����
movcfb		MACRO	Port,Bit
		If_C
		  bsf	Port,Bit
		If_NC
		  bcf	Port,Bit
		ENDM

;����������� ���������� ���� ������������� ����� � ��� C
movfbc		MACRO	Port,Bit
		setc
		If_0	Port,Bit
		  clrc
		ENDM

;����������� ���������� ���� ������������� ����� 1 � ��������� ��� ������������� ����� 2
movfbfb		MACRO	Port1,Bit1,Port2,Bit2
		If_1	Port1,Bit1
		  bsf	Port2,Bit2
		If_0	Port1,Bit1
		  bcf	Port2,Bit2
		ENDM

;����������� ���������� ���� ������������� ����� 1 � ��������� ��� ������������� ����� 2 � ���������
mvnfbfb		MACRO	Port1,Bit1,Port2,Bit2
		If_1	Port1,Bit1
		  bcf	Port2,Bit2
		If_0	Port1,Bit1
		  bsf	Port2,Bit2
		ENDM

;������������ �������������� �������� �� ����
strobe1		MACRO	Port,Bit
		bsf	Port,Bit
		bcf	Port,Bit
		ENDM

;������������ �������������� �������� �� ����
strobe0		MACRO	Port,Bit
		bcf	Port,Bit
		bsf	Port,Bit
		ENDM

;***** ������� ��� ������������ ���������� *****

;�������� ������ �� ����. Z=1 - ����� ����
test_f		MACRO	Port
		movf	Port,ToF
		ENDM

;����� ���������� ����� � ������ ����� W (4 �����, ��� C �� ������)
chg_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToW
		xorwf	Port1,ToF
		xorwf	Port2,ToF
		ENDM

;������� ����� 1 � ���� 2 ����� W �� ������� ���������� ����� ����� C
rl_fwf		MACRO	Port1,Port2
		rlf	Port1,ToW
		movwf	Port2
		ENDM

;������� ����� � ���� ����� W �� ������� ���������� ������ ����� C
rr_fwf		MACRO	Port1,Port2
		rrf	Port1,ToW
		movwf	Port2
		ENDM

;MOV

;������� ����� � W
mov_fw		MACRO	Port
		movf	Port,ToW
		ENDM

;������ ��������� � ���� ����� W (2 �����)
mov_lwf		MACRO	LL,Port
		movlw	LL
		movwf	Port
		ENDM

;������� �� ����� � ���� ����� W (2 �����)
mov_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		movwf	Port2
		ENDM

;AND

;���������� � ����� � ������ ����� W
and_lwf		MACRO	LL,Port
		movlw	LL
		andwf	Port,ToF
		ENDM

;���������� � ����� � ������ ����� W
and_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		andwf	Port2,ToF
		ENDM

;���������� � ����� � ������ � W
and_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		andwf	Port2,ToW
		ENDM

;IOR

;���������� ��� ����� � ������ ����� W
ior_lwf		MACRO	LL,Port
		movlw	LL
		iorwf	Port,ToF
		ENDM

;���������� ��� ����� � ������ ����� W
ior_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		iorwf	Port2,ToF
		ENDM

;���������� ��� ����� � ������ � W
ior_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		iorwf	Port2,ToW
		ENDM

;XOR

;����������� ��� ����� � ������ ����� W
xor_lwf		MACRO	LL,Port
		movlw	LL
		xorwf	Port,ToF
		ENDM

;����������� ��� ����� � ������ � W
xor_lfw		MACRO	LL,Port
		movlw	LL
		xorwf	Port,ToW
		ENDM

;����������� ��� ����� � ������ ����� W
xor_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToF
		ENDM

;����������� ��� ����� � ������ � W
xor_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToW
		ENDM

;ADD

;����������� ����� � ����� ����� W
add_lwf		MACRO	LL,Port
		movlw	LL
		addwf	Port,ToF
		ENDM

;����������� ����� � ����� ����� W
add_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		addwf	Port2,ToF
		ENDM

;SUB

;��������� ����� �� ����� ����� W
sub_lwf		MACRO	LL,Port
		movlw	LL
		subwf	Port,ToF
		ENDM

;��������� ����� 1 �� ����� 2 ����� W
sub_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		subwf	Port2,ToF
		ENDM

;CMP

;��������� ��������� � ������ ����� W (2 �����)
;���������: Z=1 - �����; B=1 - Port<LL; B=0 - Port>=LL
cmp_lwf		MACRO	LL,Port
		movlw	LL
		subwf	Port,ToW
		ENDM

;��������� ����� � ������ ����� W (2 �����)
;���������: Z=1 - �����; B=1 - Port2<Port1; B=0 - Port2>=Port1
cmp_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		subwf	Port2,ToW
		ENDM

;***** ������� ��� ������������ ���������� *****

;������� ������������� ����� (2 �����)
clr2f		MACRO	PortH,PortL
		clrf	PortL
		clrf	PortH
		ENDM

;�������� ������������� ����� (2 �����)
com2f		MACRO	PortH,PortL
		comf	PortL,ToF
		comf	PortH,ToF
		ENDM

;��������� ������������� ����� (3 �����)
inc2f		MACRO	PortH,PortL
		incf	PortL,ToF
		If_Z
		  incf	PortH,ToF
		ENDM

;��������� ������������� ����� (4 �����)
dec2f		MACRO	PortH,PortL
		test_f	PortL
		If_Z
		  decf	PortH,ToF
		decf	PortL,ToF
		ENDM

;����� ����� ������������� ����� (5 ������)
neg2f		MACRO	PortH,PortL
		comf	PortL,ToF
		comf	PortH,ToF
		incf	PortL,ToF
		If_Z
		  incf	PortH,ToF
		ENDM

;����� ���������� ���� ������������ ������ ����� W (8 ������, ��� C �� ������)
chg2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		chg_fwf	Port1H,Port2H
		chg_fwf	Port1L,Port2L
		ENDM

;RLF

;����� ������������� ����� ����� ����� ������� (2 �����)
rl2f		MACRO	PortH,PortL
		rlf	PortL,ToF
		rlf	PortH,ToF
		ENDM

;������� ������������� ����� 1 � ���� 2 ����� W �� ������� ���������� ����� ����� C
rl2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		rl_fwf	Port1L,Port2L
		rl_fwf	Port1H,Port2H
		ENDM

;RRF

;����� ������������� ����� ������ ����� ������� (2 �����)
rr2f		MACRO	PortH,PortL
		rrf	PortH,ToF
		rrf	PortL,ToF
		ENDM

;������� ������������� ����� 1 � ���� 2 ����� W �� ������� ���������� ������ ����� C
rr2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		rr_fwf	Port1H,Port2H
		rr_fwf	Port1L,Port2L
		ENDM

;MOV

;������ ��������� � ������������ ���� ����� W (4 �����)
movlw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		movwf	PortL
		movlw	HIGH LL
		movwf	PortH
		ENDM

;������� �� ����� � ���� ����� W (4 �����)
mov2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		movwf	Port2L
		movf	Port1H,ToW
		movwf	Port2H
		ENDM

;ADD

;����������� W � ������������� ����� (3 �����, W �� ������)
addwf2		MACRO	PortH,PortL
		addwf	PortL,ToF
		If_C
		  incf	PortH,ToF
		ENDM

;����������� ��������� � ������������� ����� ����� W (6 ������)
addlw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		addwf	PortL,ToF
		movlw	HIGH LL
		If_C
		  incw
		addwf	PortH,ToF
		ENDM

;����������� ��������� � ������������� ����� 1 ����� W � ���� 2 (8 ������)
ad2lwff		MACRO	LL,Port1H,Port1L,Port2H,Port2L
		movlw	LOW LL
		addwf	Port1L,ToW
		movwf	Port2L
		movlw	HIGH LL
		If_C
		  incw
		addwf	Port1H,ToW
		movwf	Port2H
		ENDM

;����������� ������������� ����� 1 � ����� 2 ����� W (6 ������)
add2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		addwf	Port2L,ToF
		movf	Port1H,ToW
		If_C
		  incw
		addwf	Port2H,ToF
		ENDM

;SUB

;��������� W �� ������������� ����� (3 �����, W �� ������)
subwf2		MACRO	PortH,PortL
		subwf	PortL,ToF
		If_B
		  decf	PortH,ToF
		ENDM

;��������� ��������� �� ������������� ����� ����� W (6 ������)
;���������: B=1 - Port<LL; B=0 - Port>=LL
sublw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		subwf	PortL,ToF
		movlw	HIGH LL
		If_B
		  incw
		subwf	PortH,ToF
		ENDM

;��������� ������������� ����� 1 �� ����� 2 ����� W (6 ������)
;���������: B=1 - Port2<Port1; B=0 - Port2>=Port1
sub2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		subwf	Port2L,ToF
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToF
		ENDM

;��������� ������������� ����� 1 �� ����� 2 ����� W � ���� 3 (6 ������)
;���������: B=1 - Port2<Port1; B=0 - Port2>=Port1
sb2fwff		MACRO	Port1H,Port1L,Port2H,Port2L,Port3H,Port3L
		movf	Port1L,ToW
		subwf	Port2L,ToW
		movwf	Port3L
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToW
		movwf	Port3H
		ENDM

;CMP

;��������� ��������� � ������������ ������ ����� W (6 ������)
;���������: B=1 - Port<LL; B=0 - Port>=LL
cmplw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		subwf	PortL,ToW
		movlw	HIGH LL
		If_B
		  incw
		subwf	PortH,ToW
		ENDM

;��������� ������������ ����� � ������ ����� W (6 ������)
;���������: B=1 - Port2<Port1; B=0 - Port2>=Port1
cmp2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		subwf	Port2L,ToW
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToW
		ENDM

;***** ����������� ������ � �������� *****

;����������� �����
loop_f		MACRO	Port,Label
		decfsz	Port,ToF
		  goto	Label
		ENDM

;�������� ���� (��������), ���� ��� �� ��������� � 0
WaitUntil0	MACRO	Port,Bit
		If_1	Port,Bit
		  goto	$-1
		ENDM

;�������� ���� (��������), ���� ��� �� ����������� � 1
WaitUntil1	MACRO	Port,Bit
		If_0	Port,Bit
		  goto	$-1
		ENDM

;�������� ���� (��������), ���� ���� �� ��������� � 0
WaitFUntil0 	MACRO	Port
		test_f	Port
		If_NZ
		  goto	$-2
		ENDM

;���������� nop ������ 1 ����� � ������������� 2 �����
#define		nop2	goto	$+1

;***** ������� ��������� (��� ������ W) *****

;��������� W � ������ ��� ��������
If_wEl		MACRO	LL
		xorlw	LL
		If_Z
		ENDM
	;�����:	...
	;���:	...

If_wNEl		MACRO	LL
		xorlw	LL
		If_NZ
		ENDM
	;���:	...
	;�����:	...

;��������� W � ������
If_wEf		MACRO	Port
		xorwf	Port,W
		If_Z
		ENDM
	;�����:	...
	;���:	...

If_wNEf		MACRO	Port
		xorwf	Port,W
		If_NZ
		ENDM
	;���:	...
	;�����:	...

;��������� ����� � ������
If_fEl		MACRO	Port,LL
		mov_fw	Port
		If_wEl	LL
		ENDM
	;�����:	...
	;���:	...

If_fNEl		MACRO	Port,LL
		mov_fw	Port
		If_wNEl	LL
		ENDM
	;���:	...
	;�����:	...

;��������� ����� � ������
If_fEf		MACRO	Port1,Port2
		mov_fw	Port1
		If_wEf	Port2
		ENDM
	;�����:	...
	;���:	...

If_fNEf		MACRO	Port1,Port2
		mov_fw	Port1
		If_wNEf	Port2
		ENDM
	;���:	...
	;�����:	...

;***** ����������� ��������� ��������� *****

;��������� ������� �� ������ ������� � ������ ������ � W (����� �� ���� - 0, 1, 2, 3, ..., �������� 255).
;������� ��������� � ����� TableLabel - ������������������ goto, call, return ��� retlw - ���������� �������� �� �����.
;������������ 8 ������. W ������.
TableJumpW	MACRO	TableLabel
		xorlw	HIGH (TableLabel)	;��������� HIGH (TableLabel) � PCLATH � ����������� W
		movwf	PCLATH
		xorlw	HIGH (TableLabel)
		xorwf	PCLATH,ToF
		addlw	LOW (TableLabel)	;��������� �����
		If_C
		  incf	PCLATH,ToF
		movwf	PCL			;�������
		ENDM

;=============================================================================
