;=============================================================================
;�������������� ��������� ��� ������� ����� USART � ���������� (Hex) ����
;��� �������� ������������ ������ ��������� ����� 'A'..'F'!
;������� ����������� ������� "AddInstr.inc" � "USART.inc"
;��� ��������� ������� ������������ Bank_0 � ������������ � Bank_0
;=============================================================================

;===== ��������� � ���������� ================================================

;���������� � Bank0
		CBLOCK
		  HexByte:	1	;���������� ��� ���� ������
		ENDC

;===== ��������� � ������� ===================================================

;���� ����� ASCII HEX-����� � � ������. � ���� � HexByte � W. ������ RcChar
SerialHexReceiveDigit		MACRO
				clrf	HexByte
				call	S_HexRecDigit_
				ENDM

;���� ���� ASCII HEX-���� � �� ������. � ���� � HexByte � W. ������ RcChar
#define	SerialHexReceive	call	S_HexRec_

;���������� - ���� � ���������� ���������� � Port
SerialHexReceivePort		MACRO	Port
				SerialHexReceive
				movwf	Port
				ENDM

;������� ����� ASCII HEX-����� �� ������� ������� W. HexByte �� ������, W ������
#define	SerialHexTransmitDigit	call	S_HexTrDigit_

;������� ����������� W � ���� ���� ASCII Hex-����. ��������� W � HexByte; W ������
#define	SerialHexTransmit	call	S_HexTrans_

;���������� - ������� ����� Byte
SerialHexTransmitChar		MACRO	Char
				movlw	Char
				SerialHexTransmit
				ENDM

;���������� - ������� ����������� Port
SerialHexTransmitPort		MACRO	Port
				mov_fw	Port
				SerialHexTransmit
				ENDM

;===== ���������� ============================================================

S_HexRec_:	clrf	HexByte			;���������� ����������
		call	S_HexRecDigit_		;���� � ���������� ������� �����
		swapf	HexByte,F		;������� ��� � ������� �������
S_HexRecDigit_:	SerialReceive			;���� ���������� (1/2-��) �����
		If_1	RcChar,6		;�������������� ����� � �����: 'A'-'F'?
		  addlw	0xF9				;�� => ������� ������ (-7)
		addlw	0xD0			;���������� � ����
		iorwf	HexByte,F		;����������� � ����������
		mov_fw	HexByte			;��������� ���������� � W
		return

S_HexTrans_:	movwf	HexByte			;���������� ����� � HexByte
		call	SH_a1_			;���������� � ������� ������� ������ ��������
SH_a1_:		swapf	HexByte,1		;��������� ������ ������� � �������
		mov_fw	HexByte			;��������� ��������� ������ ��������
S_HexTrDigit_:	andlw	0x0F			;������� ������� �������
		addlw	0xF6			;�������������� � �����: ��������� 10 (0x0a)
		If_C				;��������� �����������?
		  addlw	0x07				;��� => ��������� 65 (��� 'A' to 'F')
		addlw	0x3A				;�� => ��������� 58 (��� '0' to '9') 
		SerialTransmit			;������� ��������� �����
		return

;=============================================================================
