;=============================================================================
;��������� ��� ������� ����� COM-���� (USART)
;������� ����������� ������� "AddInstr.inc" � "BankSel.inc"
;��� ��������� ������� ������������ Bank_0 (SerialSetup ��������� Bank_1)
;��� ��������� ������������ � Bank_0
;=============================================================================

;���������� �����������:
;����������� ��������� �� ������� TX � RX.

;����������� �����������:
;1. �� ����������� ������ � ��������� ���������� ��������� ��������:
;	#define		BAUD_CONSTANT	0x...
;   ��������� �������� ��� Fosc = 4 ���:
;		0x67			�������� 2400 ��� (������� 2404)
;		0x33			�������� 4800 ��� (������� 4808)
;		0x19			�������� 9600 ��� (������� 9615)
;		0x0C			�������� 19200 ��� (������� 19231)
;   ��������� �������� ��� Fosc = 20 ���:
;		0x81			�������� 9600 ��� (������� 9615)
;		0x40			�������� 19200 ��� (������� 19231)
;		0x14			�������� 57600 ��� (������� 59524)
;		0x0A			�������� 115200 ��� (������� 113636)
;2. ��������� RX � TX ��� �������� ������ (�� ������ ��� � ������ ���������),
;   RX ��� ����, TX ��� �����.
;3. � ������ ������ ������� ��������� SerialSetup.

;===== ��������� � ���������� ================================================

;���������� � Bank_0
		CBLOCK
		  RcChar:	1	;������, �������� SerialReceive
		ENDC

;===== ��������� � ������� ===================================================

;***** ������������� *****

;������������� USART. ���������� ���� ��� �� ������������� USART. ������ W
SerialSetup			MACRO
				call	S_Setup_
				ENDM

;***** �������� *****

;�������� ��������������� ������ �������� (�.�. ����������� �������� ������)
If_SerialCanTransmit		MACRO
				If_1	PIR1,TXIF
				ENDM
			;�����:	...
			;�����:	...

;�������� ������������� ������ �������� (�.�. ������������� �������� ������)
If_SerialCannotTransmit		MACRO
				If_0	PIR1,TXIF
				ENDM
			;�����:	...
			;�����:	...

;�������� ����� �� W
SerialTransmit			MACRO
				call	S_Transmit_
				ENDM

;�������� ����� Char. ���� ���������� � W
SerialTransmitChar		MACRO	Char
				movlw	Char
				SerialTransmit
				ENDM

;�������� ����������� ����� Port. ���� ���������� � W
SerialTransmitPort		MACRO	Port
				mov_fw	Port
				SerialTransmit
				ENDM

;�������� ����� �� W ��� �������� ����������. 
;������������, ������ ���� ���� ����������� � ���������� ������ ��������
SerialSmartTransmit		MACRO
				movwf	TXREG
				ENDM

;***** ��Ȩ� *****

;�������� ������������� ������ ����� (�.�. ������������� ���������� ������)
If_SerialReceiveNeed		MACRO
				If_1	PIR1,RCIF
				ENDM
			;�����:	...
			;����:	...

;�������� ��������������� ������ ����� (�.�. ������������� �������� ������)
If_SerialReceiveNotNeed		MACRO
				If_0	PIR1,RCIF
				ENDM
			;����:	...
			;�����:	...

;�������� ���������� ������ � ���� ����� � W � RcChar
SerialReceive			MACRO
				call	S_Receive_
				ENDM

;�������� ���������� ������ � ���� ����� � W, RcChar � Port
SerialReceivePort		MACRO	Port
				SerialReceive
				movwf	Port
				ENDM

;���� ����� ��� �������� ���������� � W � RcChar. ���� �� ����, �� 0
;������ ������������ ���, ��� ������������� �������� ������� ������
SerialSmartReceive		MACRO
				call	S_SmartRec_
				ENDM

;���� ����� ��� �������� ���������� � W. �������� ��� ������� � ������
SerialHardReceiveToW		MACRO
				mov_fw	RCREG
				ENDM

;�������� RcChar �� ���������� � ��������
If_RcCharIs			MACRO	Char
				If_fEl	RcChar,Char
				ENDM
			;Char:	...
			;���:	...

;�������� RcChar �� ������������ � ��������
If_RcCharIsNot			MACRO	Char
				If_fNEl	RcChar,Char
				ENDM
			;���:	...
			;Char:	...

;===== ���������� ============================================================

S_Setup_:	Bank_0To1
		mov_lwf	BAUD_CONSTANT,SPBRG	;������� �������� ������
		mov_lwf	B'00100100',TXSTA	;TX9=0; TXEN=1; SYNC=0[������.]; BRGH=1[�������]
		Bank_1To0
		mov_lwf	B'10010000',RCSTA	;SPEN=1; RX9=0; CREN=1
		bcf	PIR1,RCIF
		return

S_Transmit_:	If_SerialCannotTransmit		;����� �����?
		  goto	$-1			;�� => ��������
		SerialSmartTransmit		;�������� �����
		return

S_Receive_:	If_SerialReceiveNotNeed
		  goto	$-1
		mov_fwf	RCREG,RcChar		;���� ����� � W, ������ � RcChar
		return

S_SmartRec_:	clrw
		If_SerialReceiveNeed
		  mov_fwf RCREG,RcChar		;���� ����� � W, ������ � RcChar
		return

;=============================================================================
