;========================================================================================
;���������� ����������� � ���������� � GSM-�������
;������� ������� �������
;========================================================================================

;����������� ������� ����: 1 ���� = 1 �������. 
;����: KKKKNNNR. KKKK - ��� �������; NNN - ���������; R - ���������.
;�������� KKKK:
;* 0001 (1) - ���������� ������. NNN=111 - ������ �������; 000 - �������� ������.
;* 0010	(2) - ��������� ���.
;		NNN=001 - SetThisNumber
;		NNN=010 - T
;		NNN=011 - R
;		NNN=100 - ObratnoeR
;		NNN=101 - Glubina � ���������� ���������
;* 0011 (3) - ���������.
;		NNN=001 - �����������. R=0 - �������, R=1 - ������
;		NNN=010 - ����������. R=0 - �������, R=1 - ������
;* 0100 (4) - �������� ���. ��� ����: R=0 - ����������, R=1 - ������.
;		NNN=001 - ����� �� SetThisNumber
;		NNN=010 - �����������
;		NNN=011 - ����������
;		NNN=100 - �������� ����������
;		NNN=101 - �������� �������
;		NNN=110 - "����� ���"
;* 1111 (F) - ���� ����� � �������. N=000.
;���� �������� R �� �������, ��� ����� ����.

;===== ���������� =======================================================================

		CBLOCK				;���������� � ����� 0
		  LogRdAddr:		1	  ;��������� ������ �� ����
		  LogWrAddr:		1	  ;��������� ������ � ���
		  LogData:		1	  ;��������, ����������� � ���
		ENDC

;===== ��������� ========================================================================

;������������� ������ � �����
Log_Init:	movlw	LOW(_EE_LogRdAddr)	;����� ��������� ������ � W
		call	EE_RdW_PI		;������ ��������� ������
		movwf	LogRdAddr
		call	EE_RdEEADR_PI		;������ ��������� ������
		movwf	LogWrAddr
		return

;���������� �������� �� W � ���
Log_AddW:	movwf	LogData			;���������� �������� � LogData
		;����������� ����

;���������� LogData � ���
Log_AddLogData:	Debug_AddToLog			;���������� ����� => ��������� � ���������� � ���
		mov_fw	LogWrAddr		;��������� ������ � EEADR
		call	WriteWToEEADR
		mov_fw	LogData			;����������� �������� � W
		call	EE_WrEEADR_PI		;������ � EEPROM � ����������� EEADR
		incf	LogWrAddr,ToF		;��������� LogWrAddr
		movlw	LOW(_EE_Log)
		If_1	LogWrAddr,7
		  movwf	LogWrAddr
		cmp_fwf	LogWrAddr,LogRdAddr	;��������� �� ���������� => �������
		If_NZ
		  goto	_LALD_End
_Log_IncRdAddr:	incf	LogRdAddr,ToF		;��������� LogRdAddr (�������� ����� ������ ������)
		movlw	LOW(_EE_Log)
		If_1	LogRdAddr,7
		  movwf	LogRdAddr
_LALD_End:	goto	Log_SaveRWAddr		;������ �������� ���������� � ����� (return ���)

;������ ���������� �������� �� ���� (�������� ��������� �� ����)
;�� ������: Z=1 - ��� ��� ����, �������� ���; Z=0 - ��� �� ��� ����, �������� � LogData
Log_ReadFirst:	cmp_fwf	LogWrAddr,LogRdAddr	;��� ���� => ����� � Z=1
		If_Z
		  return
		mov_fw	LogRdAddr		;��������� ������ � W
		call	EE_RdW_PI		;������ �������� � LogData
		movwf	LogData
		call	_Log_IncRdAddr		;��������� LogRdAddr � ����������� � EEPROM
		goto	_Exit_With_Z0		;����� � Z=0

;������� ����
Log_Clear:	movlw	LOW(_EE_Log)		;��������� �������� ����������
		movwf	LogRdAddr
		movwf	LogWrAddr
		;����������� ����

;���������: ���������� ���������� ������ � ������
Log_SaveRWAddr:	movlw	LOW(_EE_LogRdAddr)	;����� ��������� ������ � W
		call	WriteWToEEADR
		mov_fw	LogRdAddr		;������ ��������� ������
		call	EE_WrEEADR_PI
		mov_fw	LogWrAddr		;������ ��������� ������ � �����
		goto	EE_WrEEADR_PI			;(return ���)

;========================================================================================
