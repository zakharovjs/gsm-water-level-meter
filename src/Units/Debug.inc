;========================================================================================
;Измеритель температуры и расстояния с GSM-каналом
;Функции отладки
;========================================================================================

;Функции отладки выдают отладочную информацию по USART2 (на ПК в виде коротких сообщений).
;Все функции являются отключаемыми с помощью Debug_SIM и Debug_Log.
;Можно включать/отключать по отдельности с помощью комментирования.
;В рабочей версии программы должны быть все отключены!

;Используются обрамления:
;() - распозанное сообщение от модуля SIM900D
;<> - нераспозанное сообщение от модуля SIM900D
;[] - сообщения об обработке текста входящих смс
;«» - добавление записи в лог

;Некоторые макросы работают на пределе переполнения стека, поэтому использовать в них
;call нельзя! Наиболее длинный найденный путь:
;SendSMS_(любая) > SIM_StartSMS > SIMWait...(любая) > SIM_RecMessage > _U1Msg_IfSMSR >
;> SIMRecSMSText > Serial2Transmit... (в макросе отладки) > Вызов прерывания.

;========================================================================================

;Сообщение о перезапуске модуля SIM900D
;В программе выдаётся в момент начала включения модуля SIM900D (каждой попытки)
Debug_Restart	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '@'
	ENDIF
		ENDM

;Выдача сообщения о проведении теста ATOK
;В программе выдаётся перед проверкой связи с модулем SIM900D
Debug_TestATOK	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '?'
	ENDIF
		ENDM

;Выдача сообщения о приёме "ОК"
;В программе выдаётся при нахождении в буфере ответа "ОК" (установка флага SIMWasOK)
Debug_OK	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar 'o'
		Serial2TransmitChar 'k'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;Выдача сообщения о приёме "> "
;В программе выдаётся при нахождении в буфере ответа "> " (снятия флага SIMPromptWait)
Debug_Prompt	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar '>'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;Выдача сообщения о приёме смс и его номере в памяти из SIMNr
;В программе выдаётся при нахождении в буфере оповещения о новой смс
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

;Выдача сообщения о приёме смс и номере отправителя из PhoneNr
;В программе выдаётся при нахождении в буфере ответа с новым смс-сообщением
Debug_SMSR	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '('
		Serial2TransmitChar 'r'
		Serial2TransmitChar '='
		Serial2TransmitChar '"'
		mov_lwf	PhoneNr,Tmp1		;Адрес хранения номера
		chg_fwf	Tmp1,FSR		;Адрес записи номера в FSR
_DSMSR_1:	test_f	INDF			;Достигнут конец номера => Завершение
		If_Z
		  goto	_DSMSR_2
		Serial2TransmitPort INDF	;Выдача символа
		incf	FSR,ToF			;Следующий символ
		goto	_DSMSR_1
_DSMSR_2:	chg_fwf	Tmp1,FSR		;Адрес чтения назад в FSR
		Serial2TransmitChar '"'
		Serial2TransmitChar ')'
	ENDIF
		ENDM

;Выдача строки до двух EOL (обычно CR+LF)
;В программе выдаётся при приёме от модуля сообщения, которое не было распознано
Debug_OutLine	MACRO
	IF (Debug_SIM==1)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		Serial2TransmitChar '<'		;Выдача строки до EOL
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

;Выдача текста смс при распознавании команды
;В программе выдаётся при распознавании входящей смс, когда команда была распознана
Debug_OutCmd	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '['
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
_DCO_1:		Serial2TransmitPort INDF	;Выдача строки до EOL
		call	U1IncFSR_CZ
		If_NZ
		  goto	_DCO_1
		Serial2TransmitChar ']'
	ENDIF
		ENDM

;Сообщение о невозможности распознать команду
;В программе выдаётся при распознавании входящей смс, когда команда не была распознана
Debug_NoCmd	MACRO
	IF (Debug_SIM==1)
		Serial2TransmitChar '['
		Serial2TransmitChar '?'
		Serial2TransmitChar ']'
	ENDIF
		ENDM

;Сообщение о добавлении записи в лог
;В программе выдаётся при добавлении записи независимо от переполнения лога
Debug_AddToLog	MACRO
	IF (Debug_Log==1)
		Serial2TransmitChar '«'
		Serial2HexTransmitPort LogData
		Serial2TransmitChar '»'
	ENDIF
		ENDM

;========================================================================================
