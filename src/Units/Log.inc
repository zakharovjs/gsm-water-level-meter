;========================================================================================
;Измеритель температуры и расстояния с GSM-каналом
;Ведение журнала событий
;========================================================================================

;Кодирование событий лога: 1 байт = 1 событие. 
;Биты: KKKKNNNR. KKKK - код события; NNN - уточнение; R - результат.
;Значения KKKK:
;* 0001 (1) - перезапуск модуля. NNN=111 - начало запуска; 000 - успешный запуск.
;* 0010	(2) - получение смс.
;		NNN=001 - SetThisNumber
;		NNN=010 - T
;		NNN=011 - R
;		NNN=100 - ObratnoeR
;		NNN=101 - Glubina с корректным значением
;* 0011 (3) - измерение.
;		NNN=001 - температуры. R=0 - успешно, R=1 - ошибка
;		NNN=010 - расстояния. R=0 - успешно, R=1 - ошибка
;* 0100 (4) - отправка смс. Для всех: R=0 - отправлено, R=1 - ошибка.
;		NNN=001 - ответ на SetThisNumber
;		NNN=010 - температура
;		NNN=011 - расстояние
;		NNN=100 - обратное расстояние
;		NNN=101 - значение глубины
;		NNN=110 - "Света нет"
;* 1111 (F) - сбой связи с модулем. N=000.
;Если значение R не указано, оно равно нулю.

;===== ПЕРЕМЕННЫЕ =======================================================================

		CBLOCK				;Переменные в банке 0
		  LogRdAddr:		1	  ;Указатель чтения из лога
		  LogWrAddr:		1	  ;Указатель записи в лог
		  LogData:		1	  ;Значение, добавляемое в лог
		ENDC

;===== ПРОЦЕДУРЫ ========================================================================

;Инициализация работы с логом
Log_Init:	movlw	LOW(_EE_LogRdAddr)	;Адрес указателя чтения в W
		call	EE_RdW_PI		;Чтение указателя чтения
		movwf	LogRdAddr
		call	EE_RdEEADR_PI		;Чтение указателя записи
		movwf	LogWrAddr
		return

;Добавление значения из W в лог
Log_AddW:	movwf	LogData			;Сохранение значения в LogData
		;Продолжение ниже

;Добавление LogData в лог
Log_AddLogData:	Debug_AddToLog			;Отладочный режим => Сообщение о добавлении в лог
		mov_fw	LogWrAddr		;Указатель записи в EEADR
		call	WriteWToEEADR
		mov_fw	LogData			;Сохраняемое значение в W
		call	EE_WrEEADR_PI		;Запись в EEPROM с инкрементом EEADR
		incf	LogWrAddr,ToF		;Инкремент LogWrAddr
		movlw	LOW(_EE_Log)
		If_1	LogWrAddr,7
		  movwf	LogWrAddr
		cmp_fwf	LogWrAddr,LogRdAddr	;Указатели не сравнялись => Пропуск
		If_NZ
		  goto	_LALD_End
_Log_IncRdAddr:	incf	LogRdAddr,ToF		;Инкремент LogRdAddr (удаление самой старой записи)
		movlw	LOW(_EE_Log)
		If_1	LogRdAddr,7
		  movwf	LogRdAddr
_LALD_End:	goto	Log_SaveRWAddr		;Запись значений указателей и выход (return там)

;Чтение очередного значения из лога (значение удаляется из лога)
;На выходе: Z=1 - лог был пуст, значения нет; Z=0 - лог не был пуст, значение в LogData
Log_ReadFirst:	cmp_fwf	LogWrAddr,LogRdAddr	;Лог пуст => Выход с Z=1
		If_Z
		  return
		mov_fw	LogRdAddr		;Указатель чтения в W
		call	EE_RdW_PI		;Чтение значения в LogData
		movwf	LogData
		call	_Log_IncRdAddr		;Инкремент LogRdAddr с сохранением в EEPROM
		goto	_Exit_With_Z0		;Выход с Z=0

;Очистка лога
Log_Clear:	movlw	LOW(_EE_Log)		;Начальное значение указателей
		movwf	LogRdAddr
		movwf	LogWrAddr
		;Продолжение ниже

;Служебная: сохранение указателей чтения и записи
Log_SaveRWAddr:	movlw	LOW(_EE_LogRdAddr)	;Адрес указателя чтения в W
		call	WriteWToEEADR
		mov_fw	LogRdAddr		;Запись указателя чтения
		call	EE_WrEEADR_PI
		mov_fw	LogWrAddr		;Запись указателя записи и выход
		goto	EE_WrEEADR_PI			;(return там)

;========================================================================================
