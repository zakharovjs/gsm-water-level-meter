;========================================================================================
;���������� ����������� � ���������� � GSM-�������
;������� �������
;========================================================================================

;������� ������� ������ ���������� ���������� �� USART2 (�� �� � ���� �������� ���������).
;��� ������� �������� ������������ � ������� Debug_SIM � Debug_Log.
;����� ��������/��������� �� ����������� � ������� ���������������.
;� ������� ������ ��������� ������ ���� ��� ���������!

;������������ ����������:
;() - ����������� ��������� �� ������ SIM900D
;<> - ������������� ��������� �� ������ SIM900D
;[] - ��������� �� ��������� ������ �������� ���
;�� - ���������� ������ � ���

;��������� ������� �������� �� ������� ������������ �����, ������� ������������ � ���
;call ������! �������� ������� ��������� ����:
;SendSMS_(�����) > SIM_StartSMS > SIMWait...(�����) > SIM_RecMessage > _U1Msg_IfSMSR >
;> SIMRecSMSText > Serial2Transmit... (� ������� �������) > ����� ����������.

;========================================================================================

;��������� � ����������� ������ SIM900D
;� ��������� ������� � ������ ������ ��������� ������ SIM900D (������ �������)
Debug_Restart	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '@'
	ENDIF
		ENDM

;������ ��������� � ���������� ����� ATOK
;� ��������� ������� ����� ��������� ����� � ������� SIM900D
Debug_TestATOK	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '?'
	ENDIF
		ENDM

;������ ��������� � ����� "��"
;� ��������� ������� ��� ���������� � ������ ������ "��" (��������� ����� SIMWasOK)
Debug_OK	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar 'o'
		Serial2TransmitChar 'k'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;������ ��������� � ����� "> "
;� ��������� ������� ��� ���������� � ������ ������ "> " (������ ����� SIMPromptWait)
Debug_Prompt	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar '>'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;������ ��������� � ����� ��� � ��� ������ � ������ �� SIMNr
;� ��������� ������� ��� ���������� � ������ ���������� � ����� ���
Debug_SMSI	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar 'i'
		Serial2TransmitChar '='
		Serial2HexTransmitPort SIMNrH
		Serial2HexTransmitPort SIMNrL
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;������ ��������� � ����� ��� � ������ ����������� �� PhoneNr
;� ��������� ������� ��� ���������� � ������ ������ � ����� ���-����������
Debug_SMSR	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar 'r'
		Serial2TransmitChar '='
		Serial2TransmitChar '"'
		mov_lwf	PhoneNr,Tmp1		;����� �������� ������
		chg_fwf	Tmp1,FSR		;����� ������ ������ � FSR
_DSMSR_1:	test_f	INDF			;��������� ����� ������ => ����������
		If_Z
		  goto	_DSMSR_2
		Serial2TransmitPort INDF	;������ �������
		incf	FSR,ToF			;��������� ������
		goto	_DSMSR_1
_DSMSR_2:	chg_fwf	Tmp1,FSR		;����� ������ ����� � FSR
		Serial2TransmitChar '"'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;������ ������ �� ���� EOL (������ CR+LF)
;� ��������� ������� ��� ����� �� ������ ���������, ������� �� ���� ����������
Debug_OutLine	MACRO
	IF (Debug_SIM==1)
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
		Serial2TransmitChar '<'		;������ ������ �� EOL
_DOL_Next:	Serial2TransmitPort INDF
		call	U1IncFSR_CZ
		If_NZ
		  goto	_DOL_Next
		call	U1IncFSR_CZ
		If_NZ
		  goto	_DOL_Next
		Serial2TransmitChar '>'
	ENDIF
		ENDM

;������ ������ ��� ��� ������������� �������
;� ��������� ������� ��� ������������� �������� ���, ����� ������� ���� ����������
Debug_OutCmd	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '['
		call	U1RdAddrToFSR		;�������� ������ ������ � FSR
_DCO_1:		Serial2TransmitPort INDF	;������ ������ �� EOL
		call	U1IncFSR_CZ
		If_NZ
		  goto	_DCO_1
		Serial2TransmitChar ']'
	ENDIF
		ENDM

;��������� � ������������� ���������� �������
;� ��������� ������� ��� ������������� �������� ���, ����� ������� �� ���� ����������
Debug_NoCmd	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '['
		Serial2TransmitChar '?'
		Serial2TransmitChar ']'
	ENDIF
		ENDM

;��������� � ���������� ������ � ���
;� ��������� ������� ��� ���������� ������ ���������� �� ������������ ����
Debug_AddToLog	MACRO
	IF (Debug_Log==1)
		Serial2TransmitChar '�'
		Serial2HexTransmitPort LogData
		Serial2TransmitChar '�'
	ENDIF
		ENDM

;========================================================================================
