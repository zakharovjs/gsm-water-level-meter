;========================================================================================
;���������� ����������� � ���������� � GSM-�������
;========================================================================================

;����������� ������ �������� ��� "����� ���". �������� � �������� �� 5120 ��.
#define		SvetaNet_SMSPeriod	.58	;�������� 5 �����

;������ �������� ����� � ������� SIM900D. �������� � �������� �� 5120 ��.
#define		ATOK_Period		.12	;�������� 1 ������

;������� �������� ��� ���������� ����:
;	.12	- �������� 1 ������ (5120*12 = 61.44 �)
;	.23	- �������� 2 ������ (5120*23 = 117.76 �)
;	.58	- �������� 5 ����� (5120*58 = 296.96 �)
;	.117	- �������� 10 ����� (5120*117 = 599.04 �)
;	.234	- �������� 20 ����� (5120*234 = 1198.08 �)

;��������� ���������� ������� (������ Debug.inc), 1=��������, 0=���������
#define		Debug_SIM		.0	;������� ���������� SIM900D
#define		Debug_Log		.0	;������� (��������) ������ � ���
;� ������� ������ ��������� ������ ���� ��� ���������!

;========================================================================================

	processor	16f628a
	#include	"p16f628a.inc"
	list		p = 16f628a

	__config	0x1F14		;13	CP	0	������ ������ �������� ����
					;12..9	----	1111	�� ������������
					;8	CPD	1	������ ������ ������ ���
					;7	LVP	0	�������������� ���������������� ���
					;6	BODEN	0	����� �� ������� (BOR) ��������
					;5	MCLRE	0	RA5/-MCLR �������� ��� RA3
					;4	FOSC2	1	�������� ��������� INTRC
					;3	-PWRTE	0	PWRT �������
					;2	WDTE	1	WDT �������
					;1..0	FOSC1:0	00	�������� ��������� INTRC ��� CLKOUT

;===== ����������� ��������� ============================================================

;������ PwrKey
#define		IO_PwrKey	PORTA,2		;��� ������ ������� PWRKEY ������������� � ������ �������� (0=������)
#define		PwrKey_TRIS	TRISA,2		;��� ������� PWRKEY ������������ (RA2 ��� �����, �� ������ 0 = ������)
;�� �������� �� ����� �������� �������, �.�. ������ �������� ����� �� �����!
;������� ������ ������������ ����������� ������� �� ����� 0 � ����� ��������� ������ � ����� (PwrKey_TRIS=0).

;������ HC-SR04
#define		HCSR04_Vcc	PORTA,3		;����� ��� ������ ������� �� ������
#define		HCSR04_Trig	PORTA,4		;����� ��� ������ ��������� �������
#define		HCSR04_Echo	PORTB,3		;���� ��� ����� ������� ������� (CCP1)

;������ "�������"
#define		In_Trevoga	PORTB,0		;INT - ���� ��� ����� ������� (0=�������)

;������ DS18B20
#define		DS18B20_Vdd	PORTA,7		;����� ��� ������ ������� �� ������
#define		DS18B20_DQ_PORT	PORTA,6		;����� ������ �������
#define		DS18B20_DQ_TRIS	TRISA,6		;����������� ����� ������ �������

;������ USART2
#define		U2_RX		PORTB,5		;���� ��� ����� �� USART2
#define		U2_TX		PORTB,4		;����� ��� �������� �� USART2
;				PORTB,1		RX - ���� ��� ����� �� USART
;				PORTB,2		TX - ����� ��� �������� �� USART

;�������������� ������
#define		Free_RA0	PORTA,0
#define		Free_RA1	PORTA,1

;===== ������������� �������� ���������������� ==========================================

;CCP1 - ��������� ����������
;TMR1 - ��������� ����������
;TMR2 - ��������� ������������� ����������
;USART (����������) - ����� � SIM900
;USART2 (�����������) - ����� � ��
;WDT (������ �������� 2.3 �) - ��� ������ �� ��� ��������� ���������

;===== ���������� =======================================================================

#define		U1BufSize	.80		;������ ������ ����� USART

		CBLOCK	0x70			;���������� � ����� ������ ������� (0x70..0x7F - 16 ����)
		  U1BufWrAddr:		1	  ;�������� ��� ������ � ����� U1Buf (0..U1BufSize-1)
		  U1BufRdAddr:		1	  ;�������� ��� ������ �� ������ U1Buf (0..U1BufSize-1)
		  ;���������� ����������
		  W_Temp,Status_Temp:	1	  ;���������� ��� ���������� ���������
		  FSR_Temp:		1
		ENDC

		CBLOCK	0xA0			;���������� � ����� 1 (0xA0..0xEF - 80 ����)
		  U1Buf:		U1BufSize  ;����� ����� USART
		ENDC

		CBLOCK	0x20			;���������� � ����� 0 (0x20..0x6F - 80 ����)
		  Count100ms:		1	  ;������� ��� ������� 100 ��
		  GlubinaH,GlubinaL:	1	  ;������� [��]
		  Tmp1,Tmp2,Tmp3:	1	  ;��������� ����������
		  ;���������� ����������
		  Count20ms:		1	  ;������ 20 ��
		  Count5120ms:		1	  ;������ 5120 ��
		  DecEach20ms:		1	  ;���������������� ������ 20 �� �� ����
		  DecEach5120ms_NoSvet:	1	  ;���������������� ������ 5120 �� �� ���� (������ ��� "����� ���")
		  IncEach5120ms_ATOK:	1	  ;���������������� ������ 5120 �� �� 255 (������ �������� �����)
		ENDC

;������������ ����������
#define		Glubina		GlubinaH,GlubinaL

;===== ������� � �������������� ���������� ==============================================

#include "D:\Working\Common\PicUnits\12_16\AddInstr.inc"
#include "D:\Working\Common\PicUnits\16\BankSel302.inc"

;����������� ������ USART2.inc (�� ����������, �.�. ������� ������������ � ��)
#include "D:\Working\Common\PicUnits\12_16\USART2.inc"

;===== ������ ������� ===================================================================

		ORG	0x0000

		Bank_0
		clrf	INTCON			;��������� ��� ����������
		goto	Main

;===== ������ ���������� ================================================================

		ORG	0x0004

		Was_In_Bank_0

;���������� ����� ���������� ������ �� TMR2 ������ 138 ���.
;���������� �������� ��:
;	1. ��������� USART2.
;	2. ���� � USART � ����� �����.
;	3. ������ ��������� ����������.

;������ � PORTA (������� bcf, bsf) � ���������� �� �����������, �.�. �� ���� ����� ��� ��������������� �����!
;������������ call � ���������� �� �����������, ��� ��� ��������� ������������ �����!

Intr:		;***** ���������� ��������� � ����� TMR2 *****
		movwf	W_Temp			;���������� ���������
		swapf	STATUS,ToW
		movwf	Status_Temp
		clrf	STATUS			;����� ����� 0 ����� ����������� (Was_In_Bank_0 ���� ����)
		bcf	PIR1,TMR2IF		;����� ����� ����������

		;***** ������ � USART2 (���������� ��������!) *****
		U2_Intr_Fast			;������� ��������� USART2 (�� 22 ������ [36 ��� ������. ����� � ��������])
		;����� ������ � USART2

		;***** ���� �� USART (3..17 ������) *****
		If_0	PIR1,RCIF		;���� �� ��������� (������ If_SerialReceiveNotNeed) => ����������
		  goto	_IU1_Exit
		mov_fwf	FSR,FSR_Temp		;���������� FSR, �.�. ��������� ���� ���������� (IRP ����������� �� STATUS)
		bcf	STATUS,IRP		;9-� ��� ������ (IRP:FSR) = 0, �.�. ����� � ����� 1
		movlw	U1Buf			;������ ������ ������ � FSR
		addwf	U1BufWrAddr,ToW
		movwf	FSR
		mov_fw	RCREG			;���� ����� � W (������ SerialHardReceiveToW)
		movwf	INDF			;���������� W � �����
		incf	U1BufWrAddr,ToF		;��������� U1BufWrAddr � ������������� (0..U1BufWrAddr-1=>0)
		cmp_lwf	U1BufSize,U1BufWrAddr
		If_2GE1
		  clrf	U1BufWrAddr
		mov_fwf	FSR_Temp,FSR		;�������������� FSR (IRP ����������������� ����� �� STATUS)
_IU1_Exit:	;����� ������ � USART

		;***** ������ ��������� ���������� (4..14 ������) *****
		decfsz	Count20ms,ToF		;������ 20 ��
		  goto	_IT_Exit
		mov_lwf	.145,Count20ms
		mov_fw	DecEach20ms		;�������� ������ 20 ��: ������ DecEach20ms �� ����
		If_NZ
		  decf	DecEach20ms,ToF
		decfsz	Count5120ms,ToF		;������ 5120 ��
		  goto	_IT_Exit
		mov_fw	DecEach5120ms_NoSvet	;�������� ������ 5120 ��: ������ DecEach5120ms_NoSvet �� ����
		If_NZ
		  decf	DecEach5120ms_NoSvet,ToF
		incfsz	IncEach5120ms_ATOK,ToW		;��������� IncEach5120ms_ATOK �� 255
		  incf	IncEach5120ms_ATOK,ToF
_IT_Exit:	;����� ������� ��������� ����������

		;***** �������������� ��������� � ����� �� ���������� *****
		swapf	Status_Temp,ToW
		movwf	STATUS
		swapf	W_Temp,ToF
		swapf	W_Temp,ToW
		retfie

;������������ ���������� � ����������:
;15 - ������� (4), ���������� ��������� � ���������� TMR1 (5), �������������� ��������� � ����� (6)
;22 - ������ � USART2 (�� 36 ��� ������������� ����� � ��������)
;17 - ���� �� USART
;4..14 - ������ ��������� ����������
;�����: 15+22+17+14=68 ������
;�����: 15+36+17+14=82 ����� ��� ������������� ����� � ��������

;===== ������ ������ ���������� =========================================================

;***** ���������� *****

#define		Arifm_Mul		0	;��������� �� ������������
#define		Arifm_Div		1	;������� ������������

#include "D:\Working\Common\PicUnits\12_16\Arifm.inc"

;***** USART *****

#define		BAUD_CONSTANT	0x67		;�������� ������ �� USART 2400 ���

#include "D:\Working\Common\PicUnits\16\USART.inc"
#include "D:\Working\Common\PicUnits\12_16\USARTHex.inc"

;***** USART2 *****

;������ USART2.inc ��������� ����

		U2_Procedures			;����������� ������� �������� ��� USART2

#include "D:\Working\Common\PicUnits\12_16\USART2Hx.inc"

;***** ������� *****

#define		MCU_Speed	.1000		;�������� ������ �� 1 MIPS (Fosc = 4 ���)

#include "D:\Working\Common\PicUnits\12_16\MCUPauses.inc"
#include "D:\Working\Common\PicUnits\12_16\DS18B20.inc"

;===== ��������� ========================================================================

;����� 80-100 ��. ������ DecEach20ms
Pause100ms:	movlw	.5
		goto	Pause_Wx20ms

;����� 480-500 ��. ������ DecEach20ms
Pause500ms:	movlw	.25
		goto	Pause_Wx20ms

;����� 980-1000 ��. ������ DecEach20ms
Pause1s:	movlw	.50
		;����������� ����

;����� W*20 ��. ������ DecEach20ms
Pause_Wx20ms:	movwf	DecEach20ms		;������� �������� �����
		clrwdt				;����� WDT (���������� ������)
		WaitFUntil0 DecEach20ms		;�������� ���������� �����
		return

;===== ������� ������� ==================================================================

#include "Units\Debug.inc"

;===== ������� ������� ���� =============================================================

#include "Units\Log.inc"

;===== ��������� ������ � ��������� =====================================================

;***** ����������� *****

;��������� ����������� ������� DS18B20 � DS18B20_TL �� ������ (��������� DS18B20_Present)
;������������ �� 220 ��
GetTemperature:	bsf	DS18B20_Vdd		;������ ������� �������
		call	Pause100ms		;����� 80-100 �� ��� ������������ ������� � ������� �������
		DS18B20Init			;������������� DS18B20
		DS18B20Set9bit			;��������� DS18B20 - 9 ��� (0.5 �C, �������������� 93.75 ��)
		DS18B20ConvertT			;������ �������������� (�������������� ��� 9 ��� ��� 93.75 ��)
		movlw	.6			;����� 100-120 �� ��� ���������
		call	Pause_Wx20ms
		DS18B20ReadT			;������ ���������� � DS18B20_T (����� ��������� DS18B20_Present)
		bcf	DS18B20_Vdd		;���������� ������� �������
		call	_GT_4rrfT		;������������ ���������� � �C
		movlw	0x32			;���������� � ���: ��� �������
		If_0	DS18B20_Present			;������ ��������� => ���������� ������ � ���
		  iorlw	0x01
		goto	Log_AddW			;���������� � ����� (return ���)
_GT_4rrfT:	;���������: ������������ ���������� � �C: DS18B20_TL:=DS18B20_T/16
		call	_GT_2rrfT
_GT_2rrfT:	call	_GT_rrfT
_GT_rrfT:	rrf	DS18B20_TH,ToF
		rrf	DS18B20_TL,ToF
		return

;***** ���������� *****

#include "Units\HC-SR04.inc"

;===== ��������� ������ � GSM-������� ===================================================

;������� ������ ����� USART
U1BufClear:	clrf	U1BufWrAddr		;������������� ������ ������
		clrf	U1BufRdAddr		;������������� ������ ������
		return

;�������� � FSR ������ ������ �� ������ USART
U1RdAddrToFSR:	bcf	STATUS,IRP		;9-� ��� ������ (IRP:FSR) = 0, �.�. ����� � ����� 1
		movlw	U1Buf			;������ ������ ������ � FSR
		addwf	U1BufRdAddr,ToW
		movwf	FSR
		return

;��������� FSR � ������������� �� ������ ����� USART
;���������: C=1 - ��������� ����� ������ (�������� ���); Z=1 (��� C=0) - ��������� ������ ����� ������ (EOL)
U1IncFSR_CZ:	incf	FSR,ToF			;��������� FSR � ������������� �� ������
		cmp_lwf	U1Buf+U1BufSize,FSR
		movlw	U1Buf
		If_2GE1
		  movwf	FSR
		movlw	U1Buf			;������ ������ ������ � W
		addwf	U1BufWrAddr,ToW
		xorwf	FSR,ToW			;��������� � ������� ������ => ��������� ���� C
		clrc
		If_Z
		  setc
		;����������� ����

;�����������, ��� � INDF ������ ����� ������ 0x0A ��� 0xOD (���� ��, �� Z=1)
;��� C �� ������
U1FSRCheckEOL:	xor_lfw	0x0A,INDF		;������ ������ � ������ 0x0A => ����� � Z=1
		If_Z
		  return
		xor_lfw	0x0D,INDF		;��������� � 0x0D, ��������� � Z
		return

;��������� U1BufRdAddr � ������������� �� ������ ����� USART (�������� ������ ������� �� ������)
U1IncRdAddr:	incf	U1BufRdAddr,ToF		;��������� U1BufRdAddr � ������������� �� ������
		cmp_lwf	U1BufSize,U1BufRdAddr
		If_2GE1
		  clrf	U1BufRdAddr
		return

;��������� U1BufRdAddr �� �������� �������� FSR (�������� ��������� ������ �� ������)
U1FSRToRdAddr:	mov_fw	FSR			;U1BufRdAddr:=FSR-U1Buf
		addlw	-U1Buf
		movwf	U1BufRdAddr
		return

#include "Units\SIM900D.inc"

;===== ������� ������ � EEPROM ==========================================================

;������ W � EEADR
WriteWToEEADR:	Bank_0To1
		movwf	EEADR			;���������� W � EEADR
		Bank_1To0
		return

;������ ����� � ������� �� W � W � ����������� EEADR
EE_RdW_PI:	call	WriteWToEEADR		;���������� W � EEADR
		;����������� ����

;������ ����� � ������� �� W � W � ����������� EEADR
EE_RdEEADR_PI:	Bank_0To1
		bsf	EECON1,RD		;������
		mov_fw	EEDATA			;�������� � W
		incf	EEADR,ToF		;��������� ������ EEADR
		Bank_1To0
		return

;������ ����� �� W �� ������ EEADR � ���������� EEADR
;��������� ���������� �� 7 ���
EE_WrEEADR_PI:	Bank_0To1
		movwf	EEDATA			;��������� ������
		bcf	INTCON,GIE		;������ ����������
		bsf	EECON1,WREN		;���������� ������ � EEPROM
		mov_lwf	0x55,EECON2		;������������ ������
		mov_lwf	0xAA,EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE		;���������� ����������
		If_1	EECON1,WR		;�������� ���������� ������
		  goto	$-1
		bcf	EECON1,WREN		;������ ������ � EEPROM
		incf	EEADR,ToF		;��������� ������ EEADR
		Bank_1To0
		return

;������ ������� � Glubina
EE_ReadGlubina:	movlw	LOW (_EE_Glubina)	;����� �������� � EEPROM
		call	EE_RdW_PI		;������ �������� �������� �����
		movwf	GlubinaH		;����������
		call	EE_RdEEADR_PI		;������ �������� �������� �����
		movwf	GlubinaL		;����������
		return

;������ ������� �� Glubina
EE_SaveGlubina:	movlw	LOW (_EE_Glubina)	;����� �������� � EEPROM � W
		call	WriteWToEEADR		;������ W � EEADR
		mov_fw	GlubinaH		;�������� �������� �����
		call	EE_WrEEADR_PI		;������ �������� �������� �����
		mov_fw	GlubinaL		;�������� �������� �����
		goto	EE_WrEEADR_PI		;������ �������� �������� ����� � ����� (return ���)

;������ ������ �������� � PhoneNr
;������ Tmp1
EE_ReadPhoneNr:	mov_lwf	PhoneNr,FSR		;����� ������ ������
		mov_lwf	.15,Tmp1		;������������ ����� ���� ������, �� ������ 0
		movlw	LOW (_EE_PhoneNr)	;����� �������� � EEPROM
		call	EE_RdW_PI		;������ �������� ������� ����� � W
_EERPN_Next:	movwf	INDF			;���������� � ������
		test_f	INDF
		If_Z
		  return
		incf	FSR,ToF			;������� � ���������� �����
		call	EE_RdEEADR_PI		;������ �������� ���������� ����� � W
		loop_f	Tmp1,_EERPN_Next	;������������ (W �� ������)
		clrf	INDF			;������: ���� ���� EEPROM, ����� ����� �� ��� �����������
		return

;������ ������ �������� �� PhoneNr
EE_SavePhoneNr:	mov_lwf	PhoneNr,FSR		;����� ������ ������
		mov_lwf	.15,Tmp1		;������������ ����� ���� ������, �� ������ 0
		movlw	LOW (_EE_PhoneNr)	;����� �������� � EEPROM � W
		call	WriteWToEEADR		;������ W � EEADR
_EEWPN_Next:	mov_fw	INDF			;��������� ������ ������ � W
		If_Z					;������� => ������ ���� � �����
		  goto	_EEWPN_Wr0
		call	EE_WrEEADR_PI		;������ ����� �� W
		incf	FSR,ToF			;������� � ���������� �����
		loop_f	Tmp1,_EEWPN_Next	;������������
_EEWPN_Wr0:	clrw				;������ ���� �� ����� ������ � �����
		goto	EE_WrEEADR_PI			;(return ���)

;===== ��������� ������ ��� =============================================================

#include "Units\PC_Commands.inc"

;===== �������� ����� ��������� =========================================================

Main:		;������� ����� ������� (������� Bank_0)
		mov_lwf	0x07,CMCON		;���������� �������� (����������� ���), RA0, RA1, RA2 - ��������
		clrf	PORTA			;��������� ��������� ������
		mov_lwf	0x14,PORTB			;TX=1, U2_TX=1
		Bank_0To1
		mov_lwf	0xBF,OPTION_REG		;���������� ������. ���. PORTB; ������ ����� INT; ������������ WDT 1:128
		mov_lwf	0x27,TRISA		;��������� ������� (����� �������������!)
		mov_lwf	0xEB,TRISB			;TX, U2_TX - ������
		Bank_1To0
		SerialSetup			;������������ ������ �� USART
		Serial2Setup			;������������ ������ �� USART2

		;��������� TMR2 � ���������� �� ���� (������� Bank_0)
		clrf	TMR2			;��������� �������� TMR2
		Bank_0To1
		mov_lwf	.137,PR2		;������ 138 ������
		bsf	PIE1,TMR2IE		;���������� ���������� �� TMR2
		Bank_1To0
		mov_lwf	0x04,T2CON		;��������� TMR2: �������, 1:1, 1:1
		bcf	PIR1,TMR2IF		;����� ����� ����������
		bsf	INTCON,PEIE		;���������� ���������� ����������
		bsf	INTCON,GIE
		mov_lwf	.1,Count20ms		;����� ������� 20 ��
		clrf	Count5120ms		;����� ������� 5120 ��
		call	U1BufClear		;������� ������ ����� USART
		clrf	DecEach5120ms_NoSvet	;�������� ��� "����� ���" ���������

		;���������� ������������� (������� Bank_0)
		call	Log_Init		;������������� ������ � �����
		call	SIM_Init		;������������� �������� SIM900D
		call	Pause1s			;����� 1 ������� ��� ������������ �������
		call	GetTemperature		;�������� ��������� ����������� (��������� DS18B20_Present)
		call	GetDistance		;�������� ��������� ����������
SIM900Restart:	movlw	0x1E			;���������� ���� ������� � ���
		call	Log_AddW
_SIM900R_Again:	Debug_Restart			;���������� ����� => ��������� � ����������� ������ SIM900D
		call	SIM_OnOffByKey		;������������� SIM900D: �������� ��������� �������
		call	Pause1s				;����� 3 c������ �� ������ ������ (min 2.2, ���� 3 � �������)
		call	Pause1s
		call	Pause1s
		call	SIM_InitObmen			;������������� ������ �� USART � SIM900D
		If_NZ					;������ => �� � ������
		  goto	_SIM900R_Again
		call	SIM_SetTextSMS			;������� SMS � ��������� �����
		If_NZ					;������ => �� � ������
		  goto	_SIM900R_Again
		call	SIM_SetCharGSM			;����� ��������� GSM
		If_NZ					;������ => �� � ������
		  goto	_SIM900R_Again
		call	SIM_DelAllSMS			;�������� ���� SMS �� SIM-�����
		If_NZ					;������ => �� � ������
		  goto	_SIM900R_Again
		call	SIM_SleepToOn			;���������� ������� ������ SIM900D
		If_NZ					;������ => �� � ������
		  goto	_SIM900R_Again
		movlw	0x10			;���������� ���� ������� � ���
		call	Log_AddW

;===== �������� ���� ������ =============================================================

;�������� � Bank_0

Wait_ResetATOK:	clrf	IncEach5120ms_ATOK	;����� ������� �������� ����� � SIM900D

Wait:		clrwdt				;����� WDT (���������� ������)
		call	SIM_RecMessage		;��������� ��������� ������ SIM900D
		If_1	SIMNeedDelSMS		;��������� ������� ��� ����� "��" => ��������� (����� ����� SIM_RecMessage!)
		  call	DeleteAllSMS
		If_1	NeedSendT		;��������� ��������� ����������� => ���������
		  call	SendSMS_T
		If_1	NeedSendR		;��������� ��������� ���������� => ���������
		  call	SendSMS_R
		If_1	NeedSendG		;��������� ��������� ������� => ���������
		  call	SendSMS_G
		If_1	NeedSendOR		;��������� ��������� �������� ���������� => ���������
		  call	SendSMS_OR
		If_1	NeedSendAll		;��������� ��������� ��������� ��� ������� ������ => ���������
		  call	SendSMS_All
		If_1	INTCON,INTF		;��������� ��������� ��������� "����� ���" => ���������
		  call	SendSMS_NoSvet
		test_f	SIMNeedRecSMSNr		;��������� ��������� ��� => �������� (����� ���� ��������!)
		If_NZ
		  call	SIM_ReceiveSMS
		If_1	U2_IsNewRcChar		;���� ����� ������� �� => ���������
		  call	U2_RecPCCmd
		;�������� ����� � ������� SIM900D
		cmp_lwf	ATOK_Period,IncEach5120ms_ATOK ;����� �������� �� ��������� => ��������
		If_2Lt1
		  goto	Wait
		Debug_TestATOK			;���������� ����� => ��������� � ���������� �����
		call	SIM_TestObmen		;�������� �����
		If_Z				;�������� �������� => ���������� ������� �� ���������
		  goto	Wait_ResetATOK
		movlw	0xF0			;�������� �� ��������: ���������� ���� ������� � ���
		call	Log_AddW
		goto	SIM900Restart		;���������� ������

;===== ��������� ������ =================================================================

;***** �������� �������� *****

;������ ����������� (� = *** grad) ���� ��������� �� ������ (ERROR T)
SendTValue:	If_0	DS18B20_Present		;������� ��� => ��������� "ERROR "
		  call	_SendError_
		call	U1_SendT		;������ "T"
		If_0	DS18B20_Present		;������� ��� => �����
		  return
		call	_Send_Equal_		;" = "
		clrf	SIMNrH			;���������� ��������: ������ ������������
		movlw	'-'				;�������� ������������ => ���� "-"
		If_1	DS18B20_TL,7
		  SerialTransmit
		mov_fw	DS18B20_TL			;�������� ������������ => �����������, ����� ��� ���������
		If_1	DS18B20_TL,7
		  negw
		movwf	SIMNrL
		call	U1TransmitNr10		;�������� ��������
		call	_SendSpace		;" grad"
		SerialTransmitChar 'g'
		SerialTransmitChar 'r'
		SerialTransmitChar 'a'
		SerialTransmitChar 'd'
		return

;������ ���������� (R = *** sm) ���� ��������� �� ������ (ERROR R)
SendRValue:	ior_ffw	Dist_sm			;Dist_sm = 0x0000 => ������
		If_NZ
		  If_1	Dist_smH,7		;Dist_sm = 0xFFFF => ������
		  goto	_SRV_Error
		call	U1_SendR		;������ "R"
		call	_Send_Equal_		;" = "
		mov2fwf	Dist_sm,SIMNr		;���������� ��������
_SendSIMNrSm:	call	U1TransmitNr10		;�������� ��������
_SendSm:	;���������: ������ " sm"
		call	_SendSpace		;" sm"
		SerialTransmitChar 's'
		SerialTransmitChar 'm'
		return
_SRV_Error:	call	_SendError_		;��������� "ERROR R"
		goto	U1_SendR		;�������� "R" � ����� (return ���)
_SendError_:	;���������: ������ ������ "ERROR "
		SerialTransmitChar 'E'
		call	U1_SendR
		call	U1_SendR
		SerialTransmitChar 'O'
		call	U1_SendR
		goto	_SendSpace
_Send_Equal_:	;���������: ������ " = "
		call	_SendSpace
		call	U1_SendEqual
_SendSpace:	;���������: ������ �������
		SerialTransmitChar ' '
		return

;������ ��������� ���������� (ObratnoeR = *** sm) ���� ��������� �� ������ (ERROR R)
SendORValue:	ior_ffw	Dist_sm			;Dist_sm = 0x0000 => ������
		If_NZ
		  If_1	Dist_smH,7		;Dist_sm = 0xFFFF => ������
		  goto	_SRV_Error
		SerialTransmitChar 'O'		;������ "ObratnoeR"
		SerialTransmitChar 'b'
		SerialTransmitChar 'r'
		SerialTransmitChar 'a'
		SerialTransmitChar 't'
		SerialTransmitChar 'n'
		SerialTransmitChar 'o'
		SerialTransmitChar 'e'
		call	U1_SendR
		call	_Send_Equal_		;" = "
		sb2fwff	Dist_sm,Glubina,SIMNr	;���������� ��������: SIMNr:=Glubina-Dist_sm
		If_0	SIMNrH,7		;�������� ������������� => �����
		  goto	_SendSIMNrSm
		SerialTransmitChar '-'		;���� "�����"
		neg2f	SIMNr			;����������� ��������
		goto	_SendSIMNrSm		;������ � ����� (return ���)

;������ ������� (Glubina = *** sm)
SendGValue:	call	U1_SendG		;����� ���������: ������ "Glubina"
		SerialTransmitChar 'l'
		SerialTransmitChar 'u'
		SerialTransmitChar 'b'
		SerialTransmitChar 'i'
		SerialTransmitChar 'n'
		SerialTransmitChar 'a'
		call	_Send_Equal_			;������ " = "
		mov2fwf	Glubina,SIMNr			;���������� ��������
		call	U1TransmitNr10			;�������� ��������
		goto	_SendSm				;������ " sm" � ����� (return ���)

;������ ��������� "����� ���"
SendSvetaNet:	SerialTransmitChar 'S'
		SerialTransmitChar 'v'
		SerialTransmitChar 'e'
		SerialTransmitChar 't'
		SerialTransmitChar 'a'
		call	_SendSpace
		SerialTransmitChar 'n'
		SerialTransmitChar 'e'
		SerialTransmitChar 't'
		return

;***** �������� ��� *****

;�������� ���������� �� ����� NeedSendAll
SendSMS_All:	bcf	NeedSendAll		;���� ��������� (�������, �.�. ����� ��������� �����)
		movlw	0x22			;���������� � ��� ���� ������� � ����� ��� "SetThisNumber"
		call	Log_AddW
		call	GetTemperature		;��������� ����������� (��������� DS18B20_Present)
		call	GetDistance		;��������� ����������
		call	EE_ReadGlubina		;������ �������� �������
		call	SIM_StartSMS		;������ �������� ���
		call	SendTValue		;������ ����������� ���� ��������� �� ������
		call	_SendCommaSp		;�����������
		call	SendRValue		;������ ���������� ���� ��������� �� ������
		call	_SendCommaSp		;�����������
		call	SendGValue		;������ �������
		call	_SendCommaSp		;�����������
		call	SendORValue		;������ ��������� ���������� ���� ��������� �� ������
		If_1	In_Trevoga		;������� ������� ��� => ����������
		  goto	_SendAll_End
		call	_SendCommaSp		;�����������
		call	SendSvetaNet		;������ ��������� "����� ���"
_SendAll_End:	movlw	0x42			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)
_SendCommaSp:	;���������: ������ ", "
		SerialTransmitChar ','
		goto	_SendSpace

;�������� ����������� �� ����� NeedSendT
SendSMS_T:	bcf	NeedSendT		;���� ��������� (�������, �.�. ����� ��������� �����)
		movlw	0x24			;���������� � ��� ���� ������� � ����� ��� "T"
		call	Log_AddW
		call	GetTemperature		;��������� ����������� (��������� DS18B20_Present)
		call	SIM_StartSMS		;������ �������� ���
		call	SendTValue		;������ ����������� ���� ��������� �� ������
		movlw	0x44			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)

;�������� ���������� �� ����� NeedSendR
SendSMS_R:	bcf	NeedSendR		;���� ��������� (�������, �.�. ����� ��������� �����)
		movlw	0x26			;���������� � ��� ���� ������� � ����� ��� "R"
		call	Log_AddW
		call	GetDistance		;��������� ����������
		call	SIM_StartSMS		;������ �������� ���
		call	SendRValue		;������ ���������� ���� ��������� �� ������
		movlw	0x46			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)

;�������� ��������� ���������� �� ����� NeedSendOR
SendSMS_OR:	bcf	NeedSendOR		;���� ��������� (�������, �.�. ����� ��������� �����)
		movlw	0x28			;���������� � ��� ���� ������� � ����� ��� "ObratnoeR"
		call	Log_AddW
		call	GetDistance		;��������� ����������
		call	EE_ReadGlubina		;������ �������� �������
		call	SIM_StartSMS		;������ �������� ���
		call	SendORValue		;������ ��������� ���������� ���� ��������� �� ������
		movlw	0x48			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)

;�������� ������� �� ����� NeedSendG
SendSMS_G:	bcf	NeedSendG		;���� ��������� (�������, �.�. ����� ��������� �����)
		movlw	0x2A			;���������� � ��� ���� ������� � ����� ��� "Glubina ..."
		call	Log_AddW
		call	EE_ReadGlubina		;������ �������� �������
		call	SIM_StartSMS		;������ �������� ���
		call	SendGValue		;������ �������
		movlw	0x4A			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)

;�������� ��������� "����� ���"
SendSMS_NoSvet:	bcf	INTCON,INTF		;������� ����������
		test_f	DecEach5120ms_NoSvet	;�������� ��� ��� ��������� => ������� ������������
		If_NZ
		  return
		mov_lwf	SvetaNet_SMSPeriod,DecEach5120ms_NoSvet ;������ �������� ��� �� �������� �����
		call	SIM_StartSMS		;������ �������� ���
		call	SendSvetaNet		;������ ��������� "����� ���"
		movlw	0x4C			;����� �������� ���: ��� ������� ��� ���������� � ���
		goto	SIM_EndSMS			;������ � ���, ����� �������� � ����� (return ���)

;***** �������� ��� �� SIM-����� *****

;�������� ���� ��� �� ����� SIMNeedDelSMS
DeleteAllSMS:	bcf	SIMNeedDelSMS		;���� ��������� (�������, �.�. ����� ��������� �����)
		call	SIMWaitOK		;�������� "��" ����� �������� ������ ��� (������ DecEach20ms)
		goto	SIM_DelAllSMS		;�������� ���� SMS �� SIM-����� (Z=1 - �������) � ����� (return ���)

;===== �������� =========================================================================

		CBLOCK				;���������� � ����� 0
		  VariablesEndAddr1: 0		;����� ����� ����� 0. �� ������ ��������� 0x070
		ENDC

		IF VariablesEndAddr1>0x70
		  error "VariablesEndAddr1 ��������� 0x70"
		ENDIF

;===== EEPROM ===========================================================================

		ORG	0x2100		;EEPROM. ������������ DW, � �� DB!

_EE_PhoneNr:	DW	'+'		;����� 0 (16 ����): ����� �������� ��� �������� (�� ����� 0x00)
		DW	'7'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	0x00
		DW	0x00
		DW	0x00
		DW	0x00

_EE_Glubina:	DW	0x01		;����� 16 (2 �����): ������� (�� ��������� 400=0x0190)
		DW	0x90

_EE_LogRdAddr:	DW	LOW (_EE_Log)	;����� 18: ��������� ������ �� ����
_EE_LogWrAddr:	DW	LOW (_EE_Log)	;����� 19: ��������� ������ � ��� (������ ����� _EE_LogRdAddr!)

_EE_Log:	DW	0x00		;����� 20: ��� �� ����� EEPROM (128-20=108 ����)

;========================================================================================

		END
