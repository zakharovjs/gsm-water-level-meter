;========================================================================================
;���������� ����������� � ���������� � GSM-�������
;��������� ������ � ������� SIM900D
;========================================================================================

;��������� ��������� � ������ ����� ������� � �������� ���������.
;��������� ���������� ���������� DecEach20ms � Tmp1,Tmp2,Tmp3 ���������, � ����� �������������� ����������.

;� ������ ������ ������� SIM_Init, ���������� ��������� � ��������� ������.
;�� ����� ������ ������������ �������� ��������� U1_RecSIMMsg.
;��� SIMNeedRecSMSNr<>0 ���������� ��������� ��� � ��������� �������.
;�� ������ �� ������ SIMFlags2 ���������� ��������� ��� � �����������.

;===== ���������� =======================================================================

		CBLOCK				;���������� � ����� 0
		  SIMFlags1,SIMFlags2:	1	  ;�����
		  SIMNeedRecSMSNr:	1	  ;����� ���, ������� ���������� ��������� �� ������ (0=���)
		  SIMNrH,SIMNrL:	1	  ;�����������/������������ �����
		  PhoneNr:		.16	  ;����� �������� � ���������� ����, �� ����� 0x00
		ENDC

;����� ���������� SIMFlags1
#define		SIMMsgDetected	SIMFlags1,0	;��������� ��� U1_RecSIMMsg: ��� ��������� �����
#define		SIMSMSDetected	SIMFlags1,1	;��������� ��� SIMRecSMSText: ���� ���������� ���-�������
#define		SIMWasOK	SIMFlags1,2	;���� ������� ��������� "��"
#define		SIMNeedDelSMS	SIMFlags1,3	;��������� ����� ����� "��" ������� ��� ��� �� SIM-�����
#define		SIMPromptWait	SIMFlags1,4	;�������� ����������� "> " ��� �������� ������ ���
#define		SIMWasCMGS	SIMFlags1,5	;���� ������� ��������� ������ "+CMGS" (��� ����������)
#define		SIMSendSMSIsOK	SIMFlags1,6	;��������� �������� ��� (1=��)

;����� ���������� SIMFlags2 (��������������� ������� �� ����� ����� ��� � ���������)
#define		NeedSendAll	SIMFlags2,0	;���������� ��������� ��� � ����������� ��� ������� ������
#define		NeedSendT	SIMFlags2,1	;���������� ��������� ��� � ������������
#define		NeedSendR	SIMFlags2,2	;���������� ��������� ��� � �����������
#define		NeedSendOR	SIMFlags2,3	;���������� ��������� ��� � �������� �����������
#define		NeedSendG	SIMFlags2,4	;���������� ��������� ��� � ��������

;������������ ����������
#define		SIMNr		SIMNrH,SIMNrL

;===== ��������� ��������� ==============================================================

;�������� ��������� ��������� ��������
U1_SendCR:	SerialTransmitChar .13		;�������� CR
		return
U1_SendQuote:	SerialTransmitChar '"'		;�������� ������� '"'
		return
U1_SendAT:	SerialTransmitChar 'A'		;�������� "��"
U1_SendT:	SerialTransmitChar 'T'		;�������� "�"
		return
U1_SendCMG:	call	U1_SendC		;�������� "CMG"
		SerialTransmitChar 'M'
U1_SendG:	SerialTransmitChar 'G'
		return
U1_SendCSC:	call	U1_SendC		;�������� "CSC"
		SerialTransmitChar 'S'
U1_SendC:	SerialTransmitChar 'C'		;�������� "�"
		return
U1_SendSEqual:	SerialTransmitChar 'S'		;�������� "S="
U1_SendEqual:	SerialTransmitChar '='		;�������� "="
		return
U1_SendR:	SerialTransmitChar 'R'		;�������� "R"
		return

;������ �������� � ������� �� ���: �������� CR � "��+"
U1_SendCRATPls:	call	U1_SendCR		;����� �� ������� ������ (���� ���)
		call	U1_SendAT		;������ �������: ������ "AT+"
		SerialTransmitChar '+'
		return

;===== ��������� ���������/���������� � ������������� ===================================

;������������� �������� ������
SIM_Init:	clrf	SIMFlags1		;������� ������
		clrf	SIMFlags2
		clrf	SIMNeedRecSMSNr		;�������� ���-��������� �� ���������
		return

;��������� ��� ���������� SIM900D ��������� ������� ������
;������������ ����� 1 �������. ������ DecEach20ms
;����� ��������� ������ ������ ��� min 2.2 �������
SIM_OnOffByKey:	bcf	IO_PwrKey		;�������� ������� ������
		Bank_0To1
		bcf	PwrKey_TRIS
		Bank_1To0
		movlw	.51			;����� 1000-1020 ��
		call	Pause_Wx20ms
		Bank_0To1
		bsf	PwrKey_TRIS		;�������� ������� ������
		Bank_1To0
		return

;������������� ������ �� USART � SIM900D. �������� min ����� 2.2 ������� ����� SIM_OnOffByKey
;���������: Z=1 - �������, Z=0 - ���. ������ DecEach20ms, Tmp1
SIM_InitObmen:	call	U1_SendCR		;����� �� ������� ������ (���� ���)
		mov_lwf	.10,Tmp1		;10 ������� ������������ �����
_SIMInit_AT:	call	U1_SendAT		;������� AT
		call	U1_SendCR
		call	U1BufClear		;�������� �� ������ ����� ����� ��� ������ (���� ����)
		call	SIMWaitOK		;�������� "��" (������ DecEach20ms)
		If_Z				;"��" ������ => �����
		  goto	_SIMInit_ATOK
		loop_f	Tmp1,_SIMInit_AT
		goto	_Exit_With_Z0		;����� � Z=0
_SIMInit_ATOK:	call	Pause1s			;����� 1 �������
		call	U1_SendAT		;������� ATE0 - ���������� ��� SIM900D
		SerialTransmitChar 'E'
		SerialTransmitChar '0'
		call	U1_SendCR
		call	U1BufClear		;�������� �� ������ ����� ����� ��� ������ (���� ����)
		call	SIMWaitOK			;�������� "��" (������ DecEach20ms)
		If_NZ					;�� ������ => �� � ������
		  goto	SIM_InitObmen
		return				;����� � Z=1

;�������� �����
;�����: Z=1 - ���������, Z=0 - ����. ������ DecEach20ms
SIM_TestObmen:	call	U1_SendCR		;����� �� ������� ������ (���� ���)
		call	U1_SendAT		;������� AT
		call	U1_SendCR
		goto	SIMWaitOK		;�������� "��" � ����� (return ���)

;���������� ������� ������
;���������: Z=1 - OK ������, Z=0 - ���
SIM_SleepToOn:	call	U1_SendCRATPls		;������� AT+CSCLK=0
		call	U1_SendCSC
		SerialTransmitChar 'L'
		SerialTransmitChar 'K'
		call	U1_SendEqual
		SerialTransmitChar '0'
_SIM_CmdEndOK:	call	U1_SendCR		;�������� CR (���������� �������)
		goto	SIMWaitOK		;�������� "��" � ����� (return ���)

;������� SMS � ��������� �����
;���������: Z=1 - �������, Z=0 - ���
SIM_SetTextSMS:	call	U1_SendCRATPls		;������� AT+CMGF=1
		call	U1_SendCMG
		SerialTransmitChar 'F'
		call	U1_SendEqual
		SerialTransmitChar '1'
		goto	_SIM_CmdEndOK		;�������� CR, �������� "��" � ����� (return ���)

;����� ��������� GSM
;���������: Z=1 - �������, Z=0 - ���
SIM_SetCharGSM:	call	U1_SendCRATPls		;������� AT+CSCS="GSM"
		call	U1_SendCSC
		call	U1_SendSEqual
		call	U1_SendQuote
		call	U1_SendG
		SerialTransmitChar 'S'
		SerialTransmitChar 'M'
		call	U1_SendQuote
		goto	_SIM_CmdEndOK		;�������� CR, �������� "��" � ����� (return ���)

;�������� ���� SMS �� SIM-�����
;���������: Z=1 - �������, Z=0 - ���
SIM_DelAllSMS:	call	U1_SendCRATPls		;������� AT+CMGD=1,4
		call	U1_SendCMG
		SerialTransmitChar 'D'
		call	U1_SendEqual
		SerialTransmitChar '1'
		SerialTransmitChar ','
		SerialTransmitChar '4'
		goto	_SIM_CmdEndOK		;�������� CR, �������� "��" � ����� (return ���)

;===== �������� ��� =====================================================================

;��� �������� ��� ������� SIM_StartSMS, ��������� � USART1 ����� ��� � ������� SIM_EndSMS.

;������ �������� ���
SIM_StartSMS:	bsf	SIMSendSMSIsOK		;����������� ��������� ��������
		call	U1_SendCRATPls		;������� AT+CMGS="<����� ��������>"
		call	U1_SendCMG
		call	U1_SendSEqual
		call	U1_SendQuote			;������� ����� �������
		call	EE_ReadPhoneNr			;������ ������ �������� � PhoneNr
		mov_lwf	PhoneNr,FSR			;������ PhoneNr �� ����
_SIMSSMS_Next:	mov_fw	INDF
		If_Z
		  goto	_SIMSSMS_End
		SerialTransmit
		incf	FSR,ToF
		goto	_SIMSSMS_Next
_SIMSSMS_End:	call	U1_SendQuote			;������� ����� ������
		call	U1_SendCR		;������� � ������ ���
		call	SIMWaitPrompt		;�������� ����� ����������� "> "
		If_NZ				;����������� �� ������ => ������ ��������
		  bcf	SIMSendSMSIsOK
		return

;����� �������� ���
;�� ����� W = ��� ������� ��� ���������� � ��� (��������� �������� ��� ����������� ����������)
;��������� � SIMSendSMSIsOK: 1=����������, 0=������
SIM_EndSMS:	movwf	LogData			;���������� ���� ������� � LogData
		movlw	.26			;������ "Ctrl+Z" (����� �������� ���)
		If_0	SIMSendSMSIsOK		;������ �������� => ������ "Ctrl+Z" ���������� Esc (������ �������� ���)
		  movlw	.27
		SerialTransmit
		call	U1_SendCR		;�������� CR (�� ������ ������)
		If_0	SIMSendSMSIsOK		;������ �������� => �����
		  goto	SIMESMS_Error
		bcf	SIMWasCMGS		;��������� �� �������� ��� �� �������
		call	SIMWaitCMGS		;�������� ��������� �� �������� ��� (�� 10 ������)
		call	SIMWaitCMGS
		If_NZ				;�� ������� => ������
		  bcf	SIMSendSMSIsOK
		call	SIMWaitOK		;�������� ����� "��"
		If_NZ				;�� ������� => ������
		  bcf	SIMSendSMSIsOK
		If_0	SIMSendSMSIsOK
		  goto	SIMESMS_Error
		goto	Log_AddLogData		;���������� ���� ������� � ��� � ����� (return ���)
SIMESMS_Error:	;������ ��������
		bsf	LogData,0		;���������� ���������� "������"
		goto	Log_AddLogData		;���������� ���� ������� � ��� � ����� (return ���)

;===== �������� ��� =====================================================================

;�������� ��� � ������� SIMNeedRecSMSNr �� ������ SIM900D � ����� �����.
;SIMNeedRecSMSNr �������.
;���������: Z=1 - �������, Z=0 - ���
SIM_ReceiveSMS:	call	U1_SendCRATPls		;������� AT+CMGR=<SIMNeedRecSMSNr> (������ ���)
		call	U1_SendCMG
		call	U1_SendR
		call	U1_SendEqual
		clrf	SIMNrH				;����� ��� � SIMNr
		mov_fwf	SIMNeedRecSMSNr,SIMNrL
		call	U1TransmitNr10			;�������� ������ SMS �� SIMNr
		clrf	SIMNeedRecSMSNr		;������ �� ���� ��� ���������
		goto	_SIM_CmdEndOK		;�������� CR, �������� "��" � ����� (return ���)

;===== ��������� ��Ȩ�� � ��������� ��������� ������ ====================================

;�������� ����������� "> " (�� 1 �������)
;�����: Z=1 - �������, Z=0 - �� �������. ������ DecEach20ms
SIMWaitPrompt:	bsf	SIMPromptWait		;����������� �� �������, ���������
		mov_lwf	.50,DecEach20ms		;������������ ����� �������� 1 �������
_SIMWP_Wait:	clrwdt				;����� WDT (���������� ������)
		call	SIM_RecMessage		;��������� �������� ��������� ������
		If_0	SIMPromptWait		;��������� ������� => ����� � Z=1
		  goto	_SIMWait_P_OK			;����� 100 �� �� �������� � "��" (��. ����) � �����
		test_f	DecEach20ms		;����� �������� �� ��������� => ���
		If_NZ
		  goto	_SIMWP_Wait
		bcf	SIMPromptWait		;�������� ����������� ���������
_SIMWait_NotOK:	goto	_Exit_With_Z0		;����� � Z=0

;�������� ��������� "��" (�� 1 �������)
;�����: Z=1 - �������, Z=0 - �� �������. ������ DecEach20ms
SIMWaitOK:	bcf	SIMWasOK		;��������� �� �������
		mov_lwf	.50,DecEach20ms		;������������ ����� �������� 1 �������
_SIMWOK_Wait:	clrwdt				;����� WDT (���������� ������)
		call	SIM_RecMessage		;��������� �������� ��������� ������
		If_1	SIMWasOK		;��������� ������� => �����
		  goto	_SIMWOK_OK
		test_f	DecEach20ms		;����� �������� �� ��������� => ���
		If_NZ
		  goto	_SIMWOK_Wait
		goto	_SIMWait_NotOK		;����� � Z=0
_SIMWOK_OK:	bcf	SIMWasOK		;��������� ����������
_SIMWait_P_OK:	call	Pause100ms		;��������: SIM900D �� ����� ������ ����� ����� "��" (���� �� 25 �� �����)
		goto	_Exit_With_Z1		;����� � Z=1

;�������� ��������� "+CMGS" (�� 5 ������). �������������� �������� SIMWasCMGS.
;����� �������� ��������� ��� ������ ��� ���������� �������.
;�����: Z=1 - �������, Z=0 - �� �������. ������ DecEach20ms
SIMWaitCMGS:	mov_lwf	.250,DecEach20ms	;������������ ����� �������� 5 ������
_SIMWCMGS_Wait:	clrwdt				;����� WDT (���������� ������)
		call	SIM_RecMessage		;��������� �������� ��������� ������
		If_1	SIMWasCMGS		;��������� ������� => ����� � Z=1
		  goto	_Exit_With_Z1
		test_f	DecEach20ms		;����� �������� �� ��������� => ���
		If_NZ
		  goto	_SIMWCMGS_Wait
		goto	_SIMWait_NotOK		;����� � Z=0

;***** ������ ������ � ������ ����� *****

;���������: ��������� ������� � ������� � ���������� (������ ������ ����!).
;������������ ������ Char ������ ���� � ������� �������� (��� ���� 'a'..'z').
;��� ������������ � ����� ������ - return (� �.�. ���� ����� ������ �������� ������).
;�� ������ Z - ��������� ����� ������ (EOL).
U1CompareBuf:	MACRO	Char
		call	_U1GetUpperToW
		xorlw	Char
		If_NZ
		  return
		call	U1IncFSR_CZ
		If_C
		  return
		ENDM

;���������: ������ ������� �� INDF � W � ����������� � �������� ��������
_U1GetUpperToW:	cmp_lwf	'a',INDF		;����� 'a' => ��� ����
		If_2Lt1
		  goto	_U1GUW_AsIs
		cmp_lwf	'{',INDF		;'z' � ����� => � ������� �������, ����� ��� ����
		If_2Lt1
		  goto	_U1GUW_ToUpper
_U1GUW_AsIs:	mov_fw	INDF			;������� ������� ��� ����
		return
_U1GUW_ToUpper:	mov_fw	INDF			;������� ������� � ������� ��������
		xorlw	0x20				;0x61..0x7A => 0x41..0x5A
		return

;���������: �������� ������� ������� � ������� � ������� ������
_U1Msg_DelChar:	call	U1IncRdAddr		;�������� �������
		;����������� ����

;������ ������, ����������� � ������ �����. �������� ������������
SIM_RecMessage:	cmp_fwf	U1BufWrAddr,U1BufRdAddr	;���������� � ������ ����� USART ��� => �����
		If_Z
		  return
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		call	U1FSRCheckEOL		;������ ������ - ����� ������ => �������
		If_Z
		  goto	_U1Msg_DelChar
		test_f	INDF			;������ ������ 0x00 => �������
		If_Z
		  goto	_U1Msg_DelChar
		If_0	SIMPromptWait		;����������� �� ��������� => �������
		  goto	_U1Msg_FindEOL
		call	_U1Msg_IfPrmt		;����� �����������
		If_0	SIMPromptWait		;����������� ������� => �� �������
		  goto	SIM_RecMessage
		call	U1RdAddrToFSR		;��������� �������� ������ ������ � FSR
_U1Msg_FindEOL:	call	U1IncFSR_CZ		;���� � ������ ��� ������� ����� ������ (������ CR+LF)
		If_C					;������ ���������, EOL �� ������ => ���
		  return
		If_NZ					;EOL �� ������ => ���� ������
		  goto	_U1Msg_FindEOL
		call	U1IncFSR_CZ			;��������� ������
		If_C					;������ ���������, EOL �� ������ => ���
		  return
		If_NZ					;EOL �� ������ => ���� ������
		  goto	_U1Msg_FindEOL
		;� ������ ���� ����������� ������ � ����� EOL (������ CR+LF): ������������� ���������
		bcf	SIMMsgDetected		;��������� �� ����������
		call	_U1Msg_IfOK		;����� � ��������� ��������� "OK"
		If_1	SIMMsgDetected		;��������� ���������� => �� �������
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSI		;����� � ��������� ��������� � �������� ��� � ������
		If_1	SIMMsgDetected		;��������� ���������� => �� �������
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSR		;����� � ��������� ��������� � ����� ���
		If_1	SIMMsgDetected		;��������� ���������� => �� �������
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSS		;����� � ��������� ��������� �� �������� ���
		If_1	SIMMsgDetected		;��������� ���������� => �� �������
		  goto	SIM_RecMessage
		;�� ���� ��������� �� ����������
		Debug_OutLine			;���������� ����� ������� => ������ ������ �� EOL
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
_U1Msg_ToEOL:	call	U1IncFSR_CZ		;�������� ������ �� ���� EOL (������ CR+LF)
		If_NZ
		  goto	_U1Msg_ToEOL
		call	U1IncFSR_CZ
		If_NZ
		  goto	_U1Msg_ToEOL
		call	_U1Msg_MsgEOL
		goto	SIM_RecMessage		;��������� ���������� ����������� ������ (���� ����)

_U1Msg_IfPrmt:	;����� ���� �������� "> " � �������� �����
		U1CompareBuf '>'		;������������ ���������
		cmp_lwf	' ',INDF			;������ ����� ������, ������� U1CompareBuf ������
		If_NZ
		  return
		call	U1IncFSR_CZ			;������������ �����, ��� ����� ���� � ����� ������ (C=1)
		bcf	SIMPromptWait		;��������� �������
		Debug_Prompt			;���������� ����� => ��������� � ����� "> "
		goto	U1FSRToRdAddr		;�������� ����������� �� ������ � ����� (return ���)

_U1Msg_IfOK:	;����� ��������� "OK"
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'O'		;������������ ���������
		U1CompareBuf 'K'
		If_NZ				;�� ����� ������ => �����
		  return
		bsf	SIMMsgDetected		;��������� ����������
		bsf	SIMWasOK		;������� "��"
		Debug_OK			;���������� ����� => ��������� � ����� "��"
		goto	_U1Msg_MsgEOL		;���������� ��������� ���������

_U1Msg_IfSMSI:	;����� ��������� "+CMTI: <"mem">,<index>" (�������� ���)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf '+'		;������������ ���������
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'T'
		U1CompareBuf 'I'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;��������� ����������
_U1MsgIfSMSI_1:	If_Z				;������ ��������� => ������, ������� ��
		  goto	_U1Msg_MsgEOL
		cmp_lwf	',',INDF		;����� �������: ������� ������ ',' => �����
		If_Z
		  goto	_U1MsgIfSMSI_2
		call	U1IncFSR_CZ			;��������� ������
		goto	_U1MsgIfSMSI_1
_U1MsgIfSMSI_2:	call	U1IncFSR_CZ		;������� �������: ������� �������
		If_Z					;������ ��������� => ������, ������� ��
		  goto	_U1Msg_MsgEOL
		call	U1ReceiveNr10		;���� ������ SMS � SIMNr (������ Tmp1)
		Debug_SMSI			;���������� ����� => ��������� � ����� ��� � ��� ������ � ������
		mov_fw	SIMNrL			;����� ���������, ������� ���������� �������
		test_f	SIMNrH				;����� ��������� > 255 => �� ��������������
		If_Z
		  movwf	SIMNeedRecSMSNr			;���������� ������
		goto	_U1Msg_MsgEOL		;���������� ��������� ���������

_U1Msg_IfSMSR:	;����� ��������� "+CMGR: <"������">,<"�����">,<"">,<"����">//<�����>" (�������� ���)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf '+'		;������������ ���������
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'G'
		U1CompareBuf 'R'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;��������� ����������
_U1MsgIfSMSR_1:	If_Z				;������ ��������� => ������, ������� ��
		  goto	_U1Msg_MsgEOL
		cmp_lwf	',',INDF		;����� �������: ������� ������ ',' => �����
		If_Z
		  goto	_U1MsgIfSMSR_2
		call	U1IncFSR_CZ			;��������� ������
		goto	_U1MsgIfSMSR_1
_U1MsgIfSMSR_2:	call	U1IncFSR_CZ		;������� �������: ������� �������
		If_Z					;������ ��������� => ������, ������� ��
		  goto	_U1Msg_MsgEOL
		cmp_lwf	'"',INDF		;��������� ������ �� ������� => ������, ������� ��
		If_NZ
		  goto	_U1Msg_MsgEOL
		call	U1IncFSR_CZ		;������� �������: ������� �������
		If_Z					;������ ��������� => ������, ������� ��
		  goto	_U1Msg_MsgEOL
		call	ReceivePhoneNr		;����������� ������ ����������� �� ������ � PhoneNr
		cmp_lwf	'"',INDF		;��������� ������ �� ������� => ������, ������� ��
		If_NZ
		  goto	_U1Msg_MsgEOL
_U1MsgIfSMSR_3:	call	U1IncFSR_CZ		;�������� ������ �� EOL
		If_NZ
		  goto	_U1MsgIfSMSR_3
		call	_U1Msg_MsgEOL			;������� �� �����, ����� ���������� ����� ��� ������ ���
		Debug_SMSR			;���������� ����� => ��������� � ����� ��� � ������ ��������
		call	SIMRecSMSText		;��������, ����, ��������� � �������� ������ ���
		bsf	SIMNeedDelSMS		;��������� ������� ��� ��� ����� ����� "��"
		return				;����� ����� ������ �� ���������, �.�. ����� �� �� ��������

_U1Msg_IfSMSS:	;����� ��������� "+CMGS: <index>" (��� ����������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf '+'		;������������ ���������
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'G'
		U1CompareBuf 'S'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;��������� ����������
		bsf	SIMWasCMGS		;��� ����������
_U1MsgIfSMSS_1:	call	U1IncFSR_CZ		;�������� ������ �� EOL
		If_NZ
		  goto	_U1MsgIfSMSS_1
		;����������� ����		;���������� ��������� ���������

_U1Msg_MsgEOL:	;���������� ��������� ��������� ��� ���������� EOL
		call	U1IncFSR_CZ		;������� ������� ����� ������
		If_C				;������ ��������� => �����
		  goto	_U1Msg_DelLine
		If_Z				;��������� ������ ���� ����� ������ => ���� ����������
		  goto	_U1Msg_MsgEOL
_U1Msg_DelLine:	goto	U1FSRToRdAddr		;�������� ������ �� �������� ����� � ����� (return ���)

;===== ��������� ��Ȩ�� ������ ���-��������� ============================================

;���� sms �������� ����� ����� ������ (������ ������ ��� ����������� �� 0x0D+0x0A, � ������ 0x0A),
;��������� ������ ������� ����� ��� ��������� ������ � ����� �������, �.�. �� ����� ����������.
;������������ (��� ��������, �.�. ����� ��� ����� ���� ����� ������� ������) ��� ���� �� �������,
;��� ��� ������ 80 �������� ����� "������������", ��������� �� ����� ��� EOL � OK.
;���� ���-������� �� ���������� ��� ��� 20 � ����� ��������, ��� �� ���������, � ����� ����������
;����� ��� ��������� ������ � ����� �������, �.�. �� ����� ����������.
;���� ��� ������, ��������� "��" ���������� �� �����, �� ��������� � ����� ���������� ����� ��� "��".

;��������, ����, ��������� � �������� ������ ���
;������ �� INDF, ����� ������ ��������������� �� EOL
;������ Tmp1
SIMRecSMSText:	cmp_fwf	U1BufRdAddr,U1BufWrAddr	;�������� � ������ ��� => ���
		If_Z
		  goto	SIMRecSMSText
		call	U1FSRCheckEOL		;������ ������: �� ������ ����� ������ => ���������
		If_NZ
		  goto	_SIMRSTST_Text
		call	U1IncFSR_CZ		;������ ����� ������: ������� �������
		call	U1FSRToRdAddr			;�������� ������ �� �������� ����� ��� �������� ������ ��������� �������
		goto	SIMRecSMSText		;�������� ���������� �������
_SIMRSTST_Text:	;�������� ����� ������ ��� (������� 1 ������ ����)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		clrf	Tmp1			;������� ����� ��������
_SIMRSTST_T_1:	incf	Tmp1,ToF		;������� ����� �������� � ������
		cmp_lwf	.20,Tmp1		;������ 20 �������� � ����� => ����� ��� ������� � �������� ������
		If_2GE1
		  goto	_SIMRSTST_Err
		call	U1IncFSR_CZ		;���� � ������ ������� ����� ������
		If_C					;������ ���������, EOL �� ������ => ���
		  goto	_SIMRSTST_Text
		If_NZ					;������ EOL => ���������, ����� ���� ������
		  goto	_SIMRSTST_T_1
		;��� ����� 20 �������� � EOL �� ����� �������
		bcf	SIMSMSDetected		;���-������� �� ����������
		call	_SIMRSTST_STN		;����� ��������� "SetThisNumber"
		call	_SIMRSTST_T		;����� � ��������� ������� "T"
		call	_SIMRSTST_R		;����� � ��������� ������� "R"
		call	_SIMRSTST_OR		;����� � ��������� ������� "ObratnoeR"
		call	_SIMRSTST_G		;����� ��������� "Glubina NNN"
		If_0	SIMSMSDetected		;�� ���� ���-������� �� ���������� => ����� ��� �������� ������
		  goto	_SIMRSTST_Err
		;���-������� ����������
		Debug_OutCmd			;���������� ����� => ������ ������ ���-�������
		call	U1RdAddrToFSR		;�������� ������������ ���-�������: �������� ������ ������ � FSR
_SIMRSTST_T_2:	call	U1IncFSR_CZ			;����� ����� ������
		If_NZ
		  goto	_SIMRSTST_T_2
		call	U1FSRToRdAddr			;�������� ������ �� �������� �����
		return
_SIMRSTST_Err:	;���-������� �� ����������
		Debug_NoCmd			;���������� ����� => ��������� � ��������������� �������
		goto	U1RdAddrToFSR		;�������� ������ ������ � FSR � ����� (return ���)

_SIMRSTST_STN:	;����� ��������� "SetThisNumber" (��������� ������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'S'		;������������ ���������
		U1CompareBuf 'E'
		U1CompareBuf 'T'
		U1CompareBuf 'T'
		U1CompareBuf 'H'
		U1CompareBuf 'I'
		U1CompareBuf 'S'
		U1CompareBuf 'N'
		U1CompareBuf 'U'
		U1CompareBuf 'M'
		U1CompareBuf 'B'
		U1CompareBuf 'E'
		U1CompareBuf 'R'
		If_NZ				;����� ������ �� ��������� => �����
		  return
		bsf	SIMSMSDetected		;���-������� ����������
		bsf	NeedSendAll		;���������� ��������� ��� � �����������
		goto	EE_SavePhoneNr		;���������� ������ � EEPROM � ����� (return ���)

_SIMRSTST_T:	;����� ��������� "T" (������ �����������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'T'		;������������ ���������
		If_NZ				;����� ������ �� ��������� => �����
		  return
		bsf	SIMSMSDetected		;���-������� ����������
		bsf	NeedSendT		;���������� ��������� ��� � ������������
		return

_SIMRSTST_R:	;����� ��������� "R" (������ ����������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'R'		;������������ ���������
		If_NZ				;����� ������ �� ��������� => �����
		  return
		bsf	SIMSMSDetected		;���-������� ����������
		bsf	NeedSendR		;���������� ��������� ��� � �����������
		return

_SIMRSTST_OR:	;����� ��������� "ObratoeR" (������ ��������� ����������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'O'		;������������ ���������
		U1CompareBuf 'B'
		U1CompareBuf 'R'
		U1CompareBuf 'A'
		U1CompareBuf 'T'
		U1CompareBuf 'N'
		U1CompareBuf 'O'
		U1CompareBuf 'E'
		U1CompareBuf 'R'
		If_NZ				;����� ������ �� ��������� => �����
		  return
		bsf	SIMSMSDetected		;���-������� ����������
		bsf	NeedSendOR		;���������� ��������� ��� � �������� �����������
		return

_SIMRSTST_G:	;����� ��������� "Glubina NNN" (��������� �������)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		U1CompareBuf 'G'		;������������ ���������
		U1CompareBuf 'L'
		U1CompareBuf 'U'
		U1CompareBuf 'B'
		U1CompareBuf 'I'
		U1CompareBuf 'N'
		U1CompareBuf 'A'
		U1CompareBuf ' '
		If_Z				;��������� ����� ������ => �����
		  return
		call	U1ReceiveNr10		;���� ������� � SIMNr (������ Tmp1)
		ior_ffw	SIMNr			;������� = 0 => �����
		If_Z
		  return
		call	U1FSRCheckEOL		;����� ������ �� ��������� => �����
		If_NZ
		  return
		bsf	SIMSMSDetected		;���-������� ����������
		bsf	NeedSendG		;���������� ��������� ��� � ��������
		mov2fwf	SIMNr,Glubina		;����� �������� �������
		goto	EE_SaveGlubina		;������ ������� �� Glubina � ����� (return ���)

;===== ��������� ��Ȩ�� � �������� �������� =============================================

;�����������, ��� ��� ���������� INDF �����
;���������: Z=1 - �����, Z=0 - ���
U1Buf_IfDigit:	cmp_lwf	'0',INDF
		If_2Lt1
		  goto	_Exit_With_Z0
		cmp_lwf	':',INDF
		If_2GE1
		  goto	_Exit_With_Z0
_Exit_With_Z1:	setz
		return
_Exit_With_Z0:	clrz
		return

;���� ����� � ���������� ���� �� ������ ��� ���������� INDF � SIMNr
;��������� ��������������� �� �������, ��������� �� ������. ������ Tmp1
U1ReceiveNr10:	clr2f	SIMNr			;���������� ����������
_U1RecNr_Next:	call	U1Buf_IfDigit		;��� ���������� �� ����� => �����
		If_NZ
		  return
		clrc				;��������� SIMNr �� 10: ��������� �� 2
		rl2f	SIMNr
		mov_fwf	SIMNrH,Tmp1			;��������� �� 5: ���������� �������� � Tmp1:W
		mov_fw	SIMNrL
		clrc						;��������� �� 4
		rl2f	SIMNr
		clrc
		rl2f	SIMNr
		addwf	SIMNrL,ToF				;���������� Tmp1:W � ��������
		If_C
		  incf	SIMNrH,ToF
		add_fwf	Tmp1,SIMNrH
		mov_fw	INDF			;����� � W � ��������������� � 0..9
		addlw	-'0'
		addwf	SIMNrL,ToF		;���������� W � ����������
		If_C
		  incf	SIMNrH,ToF
		call	U1IncFSR_CZ		;������� � ���������� ������� (����� ������ �� �����������, �.�. ����� EOL)
		goto	_U1RecNr_Next		;������������

;�������� ����� � ���������� ���� (0...999) �� SIMNr � ����������� ������� �����
;������ �������������� ����������
U1TransmitNr10:	mov2fwf	SIMNr,PROD2		;������������ �����
		cmplw2f	.100,PROD2		;������ �������� � ������� �������
		If_2GE1
		  goto	_U1TNr_100
		cmp_lwf	.10,PRODL
		If_2GE1
		  goto	_U1TNr_10
		goto	_U1TNr_1
_U1TNr_100:	mov_lwf	.100,MulArg		;�������� 3-�� �������
		call	_U1TNr_DivSend		;��������� � �������� ����� 3-�� �������
_U1TNr_10:	mov_lwf	.10,MulArg		;�������� 2-�� �������
		call	_U1TNr_DivSend		;��������� � �������� ����� 2-�� �������
_U1TNr_1:	mov_fw	PRODL			;�������� 1-�� �������
		goto	_U1TNr_SendW		;�������� � ����� (return ���)
_U1TNr_DivSend:	;��������� �������
		call	Divide_2To1		;������� DivRes2:=PROD2/MulArg (0..9), ������� � DivOst
		clrf	PRODH			;���������� ��� ���������� �������
		mov_fwf	DivOst,PRODL
		mov_fw	DivResL			;������������ �������� �������
_U1TNr_SendW:	addlw	'0'			;�������������� � �����
		SerialTransmit			;��������
		return

;����������� ������ ����������� �� INDF � PhoneNr
;������ Tmp1, Tmp2, Tmp3
ReceivePhoneNr:	mov_lwf	PhoneNr,Tmp1		;����� ������ ������
		mov_lwf	.16,Tmp2		;������������ ����� ������ (15 ������ + 1 ��� �������� ����������)
_RPN_Next:	cmp_lwf	'"',INDF		;������� ������� => ����������
		If_Z
		  goto	_RPN_End
		mov_fwf	INDF,Tmp3		;������ ���������� �������
		chg_fwf	Tmp1,FSR		;����� ������ ������ � FSR
		mov_fwf	Tmp3,INDF		;���������� ���������� �������
		chg_fwf	Tmp1,FSR		;����� ������ ����� � FSR
		incf	Tmp1,ToF		;��������� ������ ������
		call	U1IncFSR_CZ		;������� � ���������� �������
		If_Z				;������ ��������� => ������, ������� ���� �����
		  goto	_RPN_Err
		loop_f	Tmp2,_RPN_Next		;������������
_RPN_Err:	mov_lwf	PhoneNr,Tmp1		;���� �������� 16 ���� => ������, ������� ���� �����
_RPN_End:	chg_fwf	Tmp1,FSR		;����� ������ ������ � FSR
		clrf	INDF			;������ ���� � ����� ������
		mov_fwf	Tmp1,FSR		;����� ������ ����� � FSR
		return

;========================================================================================
