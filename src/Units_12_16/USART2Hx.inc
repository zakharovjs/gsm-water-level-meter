;=============================================================================
;�������������� ��������� ��� ������� ����� USART2 � ���������� (Hex) ����
;��� �������� ������������ ������ ��������� ����� 'A'..'F'!
;������� ����������� ������� "AddInstr.inc" � "USART2.inc"
;��� ��������� ������� ������������ Bank_0 � ������������ � Bank_0
;=============================================================================

;===== ��������� � ���������� ================================================

;���������� � Bank_0
		CBLOCK
		  HexByte2:	1	;���������� ��� ���� ������
		ENDC

;===== ��������� � ������� ===================================================

;���� ����� ASCII HEX-����� � � ������. � ���� � HexByte2 � W. ������ RcChar
Serial2HexReceiveDigit		MACRO
				clrf	HexByte2
				call	S2_HexRecDigt_
				ENDM

;���� ���� ASCII HEX-���� � �� ������. � ���� � HexByte2 � W. ������ RcChar
#define	Serial2HexReceive	call	S2_HexRec_

;���������� - ���� � ���������� ���������� � Port
Serial2HexReceivePort		MACRO	Port
				Serial2HexReceive
				movwf	Port
				ENDM

;������� ����� ASCII HEX-����� �� ������� ������� W. HexByte2 �� ������, W ������
#define	Serial2HexTransmitDigit	call	S2_HexTrDigit_

;������� ����������� W � ���� ���� ASCII Hex-����. ��������� W � HexByte2; W ������
#define	Serial2HexTransmit	call	S2_HexTrans_

;���������� - ������� ����� Byte
Serial2HexTransmitChar		MACRO	Char
				movlw	Char
				Serial2HexTransmit
				ENDM

;���������� - ������� ����������� Port
Serial2HexTransmitPort		MACRO	Port
				mov_fw	Port
				Serial2HexTransmit
				ENDM

;===== ���������� ============================================================

S2_HexRec_:	clrf	HexByte2		;���������� ����������
		call	S2_HexRecDigt_		;���� � ���������� ������� �����
		swapf	HexByte2,ToF		;������� ��� � ������� �������
S2_HexRecDigt_:	Serial2Receive			;���� ���������� (1/2-��) �����
		If_1	RcChar,6		;�������������� ����� � �����: 'A'-'F'?
		  addlw	0xF9				;�� => ������� ������ (-7)
		addlw	0xD0			;���������� � ����
		iorwf	HexByte2,ToF		;����������� � ����������
		mov_fw	HexByte2		;��������� ���������� � W
		return

S2_HexTrans_:	movwf	HexByte2		;���������� ����� � HexByte2
		call	S2H_a1_			;���������� � ������� ������� ������ ��������
S2H_a1_:	swapf	HexByte2,ToF		;��������� ������ ������� � �������
		mov_fw	HexByte2		;��������� ��������� ������ ��������
S2_HexTrDigit_:	andlw	0x0F			;������� ������� �������
		addlw	0xF6			;�������������� � �����: ��������� 10 (0x0a)
		If_C				;��������� �����������?
		  addlw	0x07				;��� => ��������� 65 (��� 'A' to 'F')
		addlw	0x3A				;�� => ��������� 58 (��� '0' to '9') 
		Serial2Transmit			;������� ��������� �����
		return

;=============================================================================
