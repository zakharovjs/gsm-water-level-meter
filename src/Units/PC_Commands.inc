;========================================================================================
;���������� ����������� � ���������� � GSM-�������
;��������� ������ ���, ���������� �� USART2
;========================================================================================

;������� �� ������ ���������� ������ ��� ���������� ���������� �� USART1 (������ � SIM900D)!

;===== ��������� ��������� ==============================================================

;��������� ��������� ������ ���. �������� ��� U2_IsNewRcChar=1
U2_RecPCCmd:	Serial2Receive			;���� ������� � W
		;�������� ������������ �������
		addlw	-'{'			;�� ��� ������ 'z' - �������
		If_C
		  return
		addlw	'{'-'a'			;�� ��� ������ 'a' - �������
		If_NC
		  return
		;����������� ������� �� ������� �� W ('a'=0)
		TableJumpW _RC_JmpTab		;�������
_RC_JmpTab:	;������� ��������� (������� � 'a')
		goto	_Cmd_a			;'a' - ������ ��������� ����� "�������"
		return				;'b'
		goto	_Cmd_c			;'c' - ������� ���� ��� ������ (����� ������������� ����������)
		return				;'d'
		return				;'e'
		return				;'f'
		goto	_Cmd_g			;'g' - ������ ������� �������
		return				;'h'
		return				;'i'
		return				;'j'
		return				;'k'
		goto	_Cmd_l			;'l' - ������ ���� � ��� ��������
		return				;'m'
		goto	_Cmd_n			;'n' - ������ ������������ ������ ��������
		return				;'o'
		return				;'p'
		return				;'q'
		goto	_Cmd_r			;'r' - ��������� � ������ ���������� � ��
		return				;'s'
		goto	_Cmd_t			;'t' - ��������� � ������ ����������� � �C
		return				;'u'
		return				;'v'
		return				;'w'
		return				;'x'
		return				;'y'
		return				;'z'

;***** ������� ��������� *****

;'t' - ��������� � ������ ����������� � �C
;�������� ������� � Hex-���� � �������������� ���� (00 = 0�C, 7F = 127�C, FF = -1�C)
_Cmd_t:		call	GetTemperature		;���������
		call	_Cmd_OutStart		;������ ������
		mov_fw	DS18B20_TL		;������
		If_0	DS18B20_Present			;������ => ��� ������
		  movlw	0x80
		Serial2HexTransmit			;������ ��������
_Cmd_End:	Serial2TransmitChar '>'		;����� ������
		return

;'r' - ��������� � ������ ���������� � ��
;�������� ������� � Hex-���� � ����������� (0000 - ������ �����������; FFFF - ������ ���������)
_Cmd_r:		call	GetDistance		;���������
		call	_Cmd_OutStart		;������ ������
		Serial2HexTransmitPort Dist_smH	;������
		Serial2HexTransmitPort Dist_smL
		goto	_Cmd_End		;����� ������ � ����� (return ���)

;'a' - ������ ��������� ����� "�������"
;���������: '0' - ������ �������; '1' - ������� �������
_Cmd_a:		call	_Cmd_OutStart		;������ ������
		movlw	'0'			;������: ������������ ���������� � W
		If_1	In_Trevoga
		  movlw	'1'
		Serial2Transmit				;������ ���������� �� W
		goto	_Cmd_End		;����� ������ � ����� (return ���)

;***** ������� ���������� ����� � �������� ��������� *****

;'n' - ������ ������������ ������ ��������
_Cmd_n:		call	EE_ReadPhoneNr		;������ ������ �������� � PhoneNr
		call	_Cmd_OutStart		;������ ������
		mov_lwf	PhoneNr,FSR		;������
_Cmdn_Next:	mov_fw	INDF				;��������� ������ ������ � W
		If_Z					;����� ���������� => ����� ������ � ����� (return ���)
		  goto	_Cmd_End
		Serial2Transmit				;������ ������� �� W
		incf	FSR,ToF				;������� � ���������� ������� ������
		goto	_Cmdn_Next

;'g' - ������ ������� �������
_Cmd_g:		call	EE_ReadGlubina		;������ ������� � Glubina
		Serial2TransmitChar '<'		;������ ������
		Serial2HexTransmitPort GlubinaH	;������
		Serial2HexTransmitPort GlubinaL
		goto	_Cmd_End		;����� ������ � ����� (return ���)

;***** ������� ������ � ����� *****

;'l' - ������ ���� � ��� ��������
_Cmd_l:		call	_Cmd_OutStart		;������ ������
		call	Log_ReadFirst		;������: ������ ������� ��������
		If_Z					;�������� ��� => ����� ������ � ����� (return ���)
		  goto	_Cmd_End
_Cmdl_Out:	Serial2HexTransmitPort LogData		;������ ��������
		call	Log_ReadFirst			;������ ���������� ��������
		If_Z					;�������� ��������� => ����� ������ � ����� (return ���)
		  goto	_Cmd_End
		Serial2TransmitChar ','			;�����������
		goto	_Cmdl_Out			;������ �������� � ������� � ����������

;'c' - ������� ���� ��� ������ (����� ������������� ����������)
_Cmd_c:		call	Log_Clear		;����������
		call	Log_Init		;��������� �������� ���������� (��� ��������)
		goto	_Cmd_l			;������ ������� ���� (��� ��������)

;===== ��������� ��������� =============================================================

;������ ������� '<'
_Cmd_OutStart:	Serial2TransmitChar '<'
		return

;========================================================================================
