;=============================================================================
;������� ������ ������ 0-3 � ��������������� ������ Message 302
;=============================================================================

;***** ���������� ����� ������ *****

Bank_0		MACRO			;������������� ����� ����� 0
		bcf	STATUS,RP1
		bcf	STATUS,RP0
		errorlevel +302
		ENDM

Bank_1		MACRO			;������������� ����� ����� 1
		bcf	STATUS,RP1
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_2		MACRO			;������������� ����� ����� 2
		bsf	STATUS,RP1
		bcf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_3		MACRO			;������������� ����� ����� 3
		bsf	STATUS,RP1
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

;***** ������������ ������ ����� ����������� *****

Bank_0To1	MACRO			;����� ����� 1 ����� ���������� ����� ����� 0
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_1To0	MACRO			;����� ����� 0 ����� ���������� ����� ����� 1
		bcf	STATUS,RP0
		errorlevel +302
		ENDM

Bank_2To3	MACRO			;����� ����� 3 ����� ���������� ����� ����� 2
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_3To2	MACRO			;����� ����� 2 ����� ���������� ����� ����� 3
		bcf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_0To2	MACRO			;����� ����� 2 ����� ���������� ����� ����� 0
		bsf	STATUS,RP1
		errorlevel -302
		ENDM

Bank_2To0	MACRO			;����� ����� 0 ����� ���������� ����� ����� 2
		bcf	STATUS,RP1
		errorlevel +302
		ENDM

Bank_1To3	MACRO			;����� ����� 3 ����� ���������� ����� ����� 1
		bsf	STATUS,RP1
		errorlevel -302
		ENDM

Bank_3To1	MACRO			;����� ����� 1 ����� ���������� ����� ����� 3
		bcf	STATUS,RP1
		errorlevel -302
		ENDM

;***** ��������� � ���������� � ������ *****

Was_In_Bank_0	MACRO			;��������� � ���������� � ����� 0
		errorlevel +302
		ENDM

Was_In_Bank_1	MACRO			;��������� � ���������� � ����� 1
		errorlevel -302
		ENDM

Was_In_Bank_2	MACRO			;��������� � ���������� � ����� 2
		errorlevel -302
		ENDM

Was_In_Bank_3	MACRO			;��������� � ���������� � ����� 3
		errorlevel -302
		ENDM

;=============================================================================
