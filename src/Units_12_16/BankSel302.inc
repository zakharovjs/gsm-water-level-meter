;=============================================================================
;Макросы выбора банков 0-3 с предотвращением вывода Message 302
;=============================================================================

;***** Абсолютный выбор банков *****

Bank_0		MACRO			;Универсальный выбор банка 0
		bcf	STATUS,RP1
		bcf	STATUS,RP0
		errorlevel +302
		ENDM

Bank_1		MACRO			;Универсальный выбор банка 1
		bcf	STATUS,RP1
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_2		MACRO			;Универсальный выбор банка 2
		bsf	STATUS,RP1
		bcf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_3		MACRO			;Универсальный выбор банка 3
		bsf	STATUS,RP1
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

;***** Переключение банков одной инструкцией *****

Bank_0To1	MACRO			;Выбор банка 1 после выбранного ранее банка 0
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_1To0	MACRO			;Выбор банка 0 после выбранного ранее банка 1
		bcf	STATUS,RP0
		errorlevel +302
		ENDM

Bank_2To3	MACRO			;Выбор банка 3 после выбранного ранее банка 2
		bsf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_3To2	MACRO			;Выбор банка 2 после выбранного ранее банка 3
		bcf	STATUS,RP0
		errorlevel -302
		ENDM

Bank_0To2	MACRO			;Выбор банка 2 после выбранного ранее банка 0
		bsf	STATUS,RP1
		errorlevel -302
		ENDM

Bank_2To0	MACRO			;Выбор банка 0 после выбранного ранее банка 2
		bcf	STATUS,RP1
		errorlevel +302
		ENDM

Bank_1To3	MACRO			;Выбор банка 3 после выбранного ранее банка 1
		bsf	STATUS,RP1
		errorlevel -302
		ENDM

Bank_3To1	MACRO			;Выбор банка 1 после выбранного ранее банка 3
		bcf	STATUS,RP1
		errorlevel -302
		ENDM

;***** Сообщения о нахождении в банках *****

Was_In_Bank_0	MACRO			;Сообщение о нахождении в банке 0
		errorlevel +302
		ENDM

Was_In_Bank_1	MACRO			;Сообщение о нахождении в банке 1
		errorlevel -302
		ENDM

Was_In_Bank_2	MACRO			;Сообщение о нахождении в банке 2
		errorlevel -302
		ENDM

Was_In_Bank_3	MACRO			;Сообщение о нахождении в банке 3
		errorlevel -302
		ENDM

;=============================================================================
