;=============================================================================
;��������� ��� ������� � �������� ����������� DS18�20 (����� ���������� ��������� � DS1822)
;��� ��������� ������� � ���������� Bank_0!
;=============================================================================

;������ �������� ��� �������� � � �������, � � ���������� ��������.
;������ ��������� �� ����������� ������ ������ ������� �� ����� (������������ "Skip ROM").

;������������ �������� ������� ��� 1 MIPS (��� 5 MIPS ����-���� ������) � �� ���������
;���������� ������� �� ��������� ���������� �� ����� �� ���������� (���� ����).

;������ �������� ��� �������� �� �� 1 � 5 MIPS. ������ ������ ��� 5 MIPS �� �����������!

;����������� ����������� ������:
;1. ���������� ������ ������ ���������� � Bank_0.
;2. �� ����������� ������ ������� � ��������� �����, � �������� ��������� ������:
;	#define		DS18B20_DQ_PORT	PORTA,0		;����� ������ �������
;	#define		DS18B20_DQ_TRIS	TRISA,0		;����������� ����� ������ �������
;3. �� ���������� ������ ���������� ������ MCUPauses.inc � ������ ��������� MCU_Speed (��. ������).
;4. ��� ��������� ����� ��������� ���������� �� ����� �� 12 �� 67-74 ���. ��������� ������ ����
;   ������������ ��������������. ����������� ������ ������ ��� ����������� �����������.
;5. � ������ ������ ������� ������ DS18B20Init (���� ������� ������� �����������, �� ��������
;   ����� ������� ��������� �������).
;6. ��� ������������� ������ ���������� ����� �� �������� DS18B20Set**bit

;������ ���������� DS18B20_T - [�C]*16 �� ������:
;S S S S S T6 T5 T4  |  T3 T2 T1 T0 T-1 T-2 T-3 T-4

;===== ��������� � ���������� ================================================

;����������
		CBLOCK
		  DS18B20_Flags:		1	;�����
		  DS18B20_TH,DS18B20_TL:	1	;����������� � ������� �������
		  DS18B20_Tmp1,DS18B20_Tmp2:	1	;��������� ����������
		ENDC

;����� ���������� DS18B20_Flags
#define		DS18B20_IsIntrs	DS18B20_Flags,0		;���������� � ��������� ���� ���������
#define		DS18B20_Present	DS18B20_Flags,1		;������ ������������ �� ����� (0=���)

;������������ ����������
#define		DS18B20_T	DS18B20_TH,DS18B20_TL

;===== ��������� � ������� ===================================================

;�������������. ���������� ���� ��� � ������ ������. ��������� DS18B20_Present.
;������������ ����� 980 ���, ��������� ���������� �������� �� 67-74 ���
#define		DS18B20Init		call	DS18B20_Setup_

;������� ����������� ��������������. ��������� DS18B20_Present.
;�������� �� ������� ��������������.
;������������ �� 4530 ���, ��������� ���������� �� ������� �� 67-74 ���
#define		DS18B20Set9bit		call	DS18B20_9bit_		;9 ���  (0.5 �C, �������������� 93.75 ��)
#define		DS18B20Set10bit		call	DS18B20_10bit_		;10 ��� (0.25 �C, �������������� 187.5 ��)
#define		DS18B20Set11bit		call	DS18B20_11bit_		;11 ��� (0.125 �C, �������������� 375 ��)
#define		DS18B20Set12bit		call	DS18B20_12bit_		;12 ��� (0.0625 �C, �������������� 750 ��)
;�� ��������� ����� ��������� ������� ������ ���������� 12 ���

;������ ��������� �����������. ��������� DS18B20_Present.
;����� ������ ��������� �������������� ��� ���������� ����� (��. ����� ������).
;������������ �� 2400 ���, ��������� ���������� �� ������� �� 67-74 ���
#define		DS18B20ConvertT		call	DS18B20_ConvT_

;������ ���������� �������������� ��� �������� � DS18B20_T. ��������� DS18B20_Present.
;����� ��������� ������� (�� ������� ��������������) ����������� �������� 0x0550.
;����� ������� �������������� � ������� ������ ��������� �������������� ������ ���� ��������� ���������.
;��� ������ ������� �������� DS18B20_T �� ��������.
;������������ �� 3620 ���, ��������� ���������� �� ������� �� 67-74 ���
#define		DS18B20ReadT		call	DS18B20_ReadT_

;===== ���������� ============================================================

DS18B20_Setup_:	clrf	DS18B20_Flags		;������� ������
		goto	_DS18B20_Init		;������������� ������ (����� 980 ���) � ����� (return ���)

DS18B20_9bit_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x4E			;Function Command 0x4E - "Write Scratchpad" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x7F			;������� TH = 0x7F (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x80			;������� TL = 0x80 (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x1F			;������� configuration register = 0x1F - 9 ��� (�� 706 ���)
		goto	_DS18B20_SendW			;(return ���)

DS18B20_10bit_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x4E			;Function Command 0x4E - "Write Scratchpad" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x7F			;������� TH = 0x7F (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x80			;������� TL = 0x80 (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x3F			;������� configuration register = 0x3F - 10 ��� (�� 706 ���)
		goto	_DS18B20_SendW			;(return ���)

DS18B20_11bit_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x4E			;Function Command 0x4E - "Write Scratchpad" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x7F			;������� TH = 0x7F (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x80			;������� TL = 0x80 (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x5F			;������� configuration register = 0x5F - 11 ��� (�� 706 ���)
		goto	_DS18B20_SendW			;(return ���)

DS18B20_12bit_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x4E			;Function Command 0x4E - "Write Scratchpad" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x7F			;������� TH = 0x7F (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x80			;������� TL = 0x80 (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x7F			;������� configuration register = 0x7F - 12 ��� (�� 706 ���)
		goto	_DS18B20_SendW			;(return ���)

DS18B20_ConvT_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0x44			;Function Command 0x44 - "Convert T" (�� 706 ���)
		goto	_DS18B20_SendW			;(return ���)

DS18B20_ReadT_:	call	_DS18B20_Init		;������������� ������ (����� 980 ���)
		If_0	DS18B20_Present		;������� ��� ��� ������ => �����
		  return
		movlw	0xCC			;ROM Command 0xCC - "Skip ROM" (�� 706 ���)
		call	_DS18B20_SendW
		movlw	0xBE			;Function Command 0xBE - "Read Scratchpad" (�� 706 ���)
		call	_DS18B20_SendW
		call	_DS18B20_RecW		;������ ����� Temperature LSB � W � DS18B20_TL (610 ���)
		movwf	DS18B20_TL
		call	_DS18B20_RecW		;������ ����� Temperature MSB � W � DS18B20_TH (610 ���)
		movwf	DS18B20_TH
		return

;===== ��������� ��������� ===================================================

;��� ��������� ��������� ��� ������� �������������� ������ ������ ��������� ���� � ������ MCUPauses.inc!
;��� �������� ���� ���� 1 MIPS � 5 MIPS ��������� ����������� ������� MCUPause_mks!

;***** ��������� ������ *****

;������������� ������ �� ����. ��������� DS18B20_Present
;������������ ����� 980 ���, �� ��� �������� � �������� ��������� ���������� �������� �� 67-74 ���
_DS18B20_Init:	bcf	DS18B20_IsIntrs		;���������� GIE � DS18B20_IsIntrs
		If_1	INTCON,GIE
		  bsf	DS18B20_IsIntrs
		bcf	DS18B20_DQ_PORT		;������ ��� ��������
		Bank_0To1
		bcf	DS18B20_DQ_TRIS		;����� � ��������� "�����"
		MCUPause_mks .200		;����� 480 ��� (�������)
		MCUPause_mks .200
		MCUPause_mks .80
		bcf	INTCON,GIE		;������ ���������� (�� ������ DS18B20_DQ!)
		bsf	DS18B20_DQ_TRIS		;����� � ��������� "����"
		Bank_1To0
		MCUPause_mks .66		;����� 66 ��� �����
		If_1	DS18B20_DQ_PORT		;������ �� ������� => ������
		  goto	_DS18B20_I_Err
		If_1	DS18B20_IsIntrs		;���������� ���������� (������� DS18B20_IsIntrs � GIE)
		  bsf	INTCON,GIE
		MCUPause_mks .200		;����� 480-66=414 ��� (�������)
		MCUPause_mks .200
		MCUPause_mks .14
		If_0	DS18B20_DQ_PORT		;����� ������� � ���� => ������
		  goto	_DS18B20_I_Err
		bsf	DS18B20_Present		;���� - ������ ���� � �����
		return
_DS18B20_I_Err:	;������ ����������
		If_1	DS18B20_IsIntrs		;���������� ���������� (������� DS18B20_IsIntrs � GIE)
		  bsf	INTCON,GIE
		bcf	DS18B20_Present		;���� - ������� ���
		return

;����� ����� �� W
;������������ �� 706 ���, ��������� ��� ��������� ���������� �������� �� 12 ��� 66 ���
_DS18B20_SendW:	movwf	DS18B20_Tmp1		;���������� ��������
		bcf	DS18B20_IsIntrs		;���������� GIE � DS18B20_IsIntrs
		If_1	INTCON,GIE
		  bsf	DS18B20_IsIntrs
		bcf	DS18B20_DQ_PORT		;������ ��� ���������
		mov_lwf	.8,DS18B20_Tmp2		;����� ���
_DS18B20_SW_1:	rrf	DS18B20_Tmp1,ToF	;����� ������, ������� ��� � C
		If_C				;������� � �������� ���� �� C
		  goto	_DS18B20_SW_S1
_DS18B20_SW_S0:	;�������� ���� (Write 0 time slot + ����� ������� 1 ���)
		bcf	INTCON,GIE		;������ ���������� (�����������, ����� ������� �� �������� 480 ���!)
		Bank_0To1
		bcf	DS18B20_DQ_TRIS		;����� � ��������� "�����"
		MCUPause_mks .60		;����� 60 ��� (�������, �������� 120 ���)
		bsf	DS18B20_DQ_TRIS		;����� � ��������� "����"
		Bank_1To0
		If_1	DS18B20_IsIntrs		;���������� ���������� (������� DS18B20_IsIntrs � GIE)
		  bsf	INTCON,GIE
		MCUPause_mks .12		;����� ����� ������� ������� 10 ��� (���� 12, �.�. ������ 4)
		goto	_DS18B20_SW_SE
_DS18B20_SW_S1:	;�������� ������� (Write 1 time slot + ����� ������� 1 ���)
		bcf	INTCON,GIE		;������ ����������
		Bank_0To1
		bcf	DS18B20_DQ_TRIS		;����� � ��������� "�����"
		MCUPause_6mks			;����� 6 ��� (����. ������� 1 � �������� 15 ���)
		bsf	DS18B20_DQ_TRIS		;����� � ��������� "����"
		Bank_1To0
		If_1	DS18B20_IsIntrs		;���������� ���������� (������� DS18B20_IsIntrs � GIE)
		  bsf	INTCON,GIE
		MCUPause_mks .64		;����� ������� 70-6=64 ��� (���� ������� 60 ��� + ����� ������� ������� 10 ���)
_DS18B20_SW_SE:	;��� ���������, ������������
		loop_f	DS18B20_Tmp2,_DS18B20_SW_1
		return
;����� ����� ������� ������� 1 ���, �� ���� ������� 10 ���, �.�. �� 1 ��� ����� ����� �� ������ ��������������

;���� ����� � W
;������������ 610 ���, ��������� ��� ��������� ���������� �������� �� 17 ���
_DS18B20_RecW:	bcf	DS18B20_IsIntrs		;���������� GIE � DS18B20_IsIntrs
		If_1	INTCON,GIE
		  bsf	DS18B20_IsIntrs
		bcf	DS18B20_DQ_PORT		;������ ��� ���������
		mov_lwf	.8,DS18B20_Tmp2		;����� ���
_DS18B20_RW_1:	;������ ���������� ����
		bcf	INTCON,GIE		;������ ����������
		Bank_0To1
		bcf	DS18B20_DQ_TRIS		;����� � ��������� "�����"
		MCUPause_1mks			;����� 1 ��� (�������, ������������� �� ������)
		bsf	DS18B20_DQ_TRIS		;����� � ��������� "����"
		Bank_1To0
		MCUPause_6mks			;����� 6 ��� (�������� 15-1=14, ������� - ����� ����� ��������������)
		movfbc	DS18B20_DQ_PORT		;������ ���� � C (3 �����)
		rrf	DS18B20_Tmp1,ToF	;����� ������, ������� ��� �� C � DS18B20_Tmp1
		If_1	DS18B20_IsIntrs		;���������� ���������� (������� DS18B20_IsIntrs � GIE)
		  bsf	INTCON,GIE
		MCUPause_mks .52		;����� ������� 61-1-6=54 ��� (���� 60 ��� + ����� ������� ������� 1 ���)
		MCUPause_2mks
		;��� �������, ������������
		loop_f	DS18B20_Tmp2,_DS18B20_RW_1
		mov_fw	DS18B20_Tmp1		;�������� �������� � W �� DS18B20_Tmp1
		return

;=============================================================================
