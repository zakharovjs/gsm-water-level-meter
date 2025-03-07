;========================================================================================
;Измеритель температуры и расстояния с GSM-каналом
;Процедуры работы с модулем SIM900D
;========================================================================================

;Процедуры обращения к буферу приёма описаны в основной программе.
;Процедуры используют переменные DecEach20ms и Tmp1,Tmp2,Tmp3 программы, а также арифметические переменные.

;В начале работы вызвать SIM_Init, произвести включение и настройку модуля.
;Во время работы периодически вызывать процедуру U1_RecSIMMsg.
;При SIMNeedRecSMSNr<>0 необходимо загрузить смс с указанным номером.
;По любому из флагов SIMFlags2 необходимо отправить смс с результатом.

;===== ПЕРЕМЕННЫЕ =======================================================================

		CBLOCK				;Переменные в банке 0
		  SIMFlags1,SIMFlags2:	1	  ;Флаги
		  SIMNeedRecSMSNr:	1	  ;Номер смс, которую необходимо загрузить из буфера (0=нет)
		  SIMNrH,SIMNrL:	1	  ;Принимаемое/отправляемое число
		  PhoneNr:		.16	  ;Номер телефона в символьном виде, на конце 0x00
		ENDC

;Флаги переменной SIMFlags1
#define		SIMMsgDetected	SIMFlags1,0	;Служебный для U1_RecSIMMsg: Был распознан ответ
#define		SIMSMSDetected	SIMFlags1,1	;Служебный для SIMRecSMSText: Была распознана смс-команда
#define		SIMWasOK	SIMFlags1,2	;Было принято сообщение "ОК"
#define		SIMNeedDelSMS	SIMFlags1,3	;Требуется после приёма "ОК" удалить все смс на SIM-карте
#define		SIMPromptWait	SIMFlags1,4	;Ожидание приглашения "> " для отправки текста смс
#define		SIMWasCMGS	SIMFlags1,5	;Было принято сообщение модуля "+CMGS" (смс отправлено)
#define		SIMSendSMSIsOK	SIMFlags1,6	;Результат отправки смс (1=ОК)

;Флаги переменной SIMFlags2 (устанавливаются модулем по факту приёма смс с командами)
#define		NeedSendAll	SIMFlags2,0	;Необходимо отправить смс с параметрами при задании номера
#define		NeedSendT	SIMFlags2,1	;Необходимо отправить смс с температутой
#define		NeedSendR	SIMFlags2,2	;Необходимо отправить смс с расстоянием
#define		NeedSendOR	SIMFlags2,3	;Необходимо отправить смс с обратным расстоянием
#define		NeedSendG	SIMFlags2,4	;Необходимо отправить смс с глубиной

;Двухбайтовые переменные
#define		SIMNr		SIMNrH,SIMNrL

;===== СЛУЖЕБНЫЕ ПРОЦЕДУРЫ ==============================================================

;Отправка различных сочетаний символов
U1_SendCR:	SerialTransmitChar .13		;Отправка CR
		return
U1_SendQuote:	SerialTransmitChar '"'		;Отправка кавычки '"'
		return
U1_SendAT:	SerialTransmitChar 'A'		;Отправка "АТ"
U1_SendT:	SerialTransmitChar 'T'		;Отправка "Т"
		return
U1_SendCMG:	call	U1_SendC		;Отправка "CMG"
		SerialTransmitChar 'M'
U1_SendG:	SerialTransmitChar 'G'
		return
U1_SendCSC:	call	U1_SendC		;Отправка "CSC"
		SerialTransmitChar 'S'
U1_SendC:	SerialTransmitChar 'C'		;Отправка "С"
		return
U1_SendSEqual:	SerialTransmitChar 'S'		;Отправка "S="
U1_SendEqual:	SerialTransmitChar '='		;Отправка "="
		return
U1_SendR:	SerialTransmitChar 'R'		;Отправка "R"
		return

;Начало конманды с выходом из сна: отправка CR и "АТ+"
U1_SendCRATPls:	call	U1_SendCR		;Выход из спящего режима (если был)
		call	U1_SendAT		;Начало команды: выдача "AT+"
		SerialTransmitChar '+'
		return

;===== ПРОЦЕДУРЫ ВКЛЮЧЕНИЯ/ВЫКЛЮЧЕНИЯ И ИНИЦИАЛИЗАЦИИ ===================================

;Инициализация процедур модуля
SIM_Init:	clrf	SIMFlags1		;Очистка флагов
		clrf	SIMFlags2
		clrf	SIMNeedRecSMSNr		;Загрузка смс-сообщения не требуется
		return

;Включение или выключение SIM900D эмуляцией нажатия кнопки
;Длительность около 1 секунды. Портит DecEach20ms
;После процедуры запуск модуля идёт min 2.2 секунды
SIM_OnOffByKey:	bcf	IO_PwrKey		;Эмуляция нажатия кнопки
		Bank_0To1
		bcf	PwrKey_TRIS
		Bank_1To0
		movlw	.51			;Пауза 1000-1020 мс
		call	Pause_Wx20ms
		Bank_0To1
		bsf	PwrKey_TRIS		;Эмуляция отжатия кнопки
		Bank_1To0
		return

;Инициализация обмена по USART с SIM900D. Вызывать min через 2.2 секунды после SIM_OnOffByKey
;Результат: Z=1 - успешно, Z=0 - нет. Портит DecEach20ms, Tmp1
SIM_InitObmen:	call	U1_SendCR		;Выход из спящего режима (если был)
		mov_lwf	.10,Tmp1		;10 попыток установления связи
_SIMInit_AT:	call	U1_SendAT		;Команда AT
		call	U1_SendCR
		call	U1BufClear		;Удаление из буфера приёма всего что пришло (если было)
		call	SIMWaitOK		;Ожидание "ОК" (портит DecEach20ms)
		If_Z				;"ОК" пришло => Далее
		  goto	_SIMInit_ATOK
		loop_f	Tmp1,_SIMInit_AT
		goto	_Exit_With_Z0		;Выход с Z=0
_SIMInit_ATOK:	call	Pause1s			;Пауза 1 секунда
		call	U1_SendAT		;Команда ATE0 - Отключение эха SIM900D
		SerialTransmitChar 'E'
		SerialTransmitChar '0'
		call	U1_SendCR
		call	U1BufClear		;Удаление из буфера приёма всего что пришло (если было)
		call	SIMWaitOK			;Ожидание "ОК" (портит DecEach20ms)
		If_NZ					;Не пришло => Всё с начала
		  goto	SIM_InitObmen
		return				;Выход с Z=1

;Проверка связи
;Ответ: Z=1 - нормально, Z=0 - сбой. Портит DecEach20ms
SIM_TestObmen:	call	U1_SendCR		;Выход из спящего режима (если был)
		call	U1_SendAT		;Команда AT
		call	U1_SendCR
		goto	SIMWaitOK		;Ожидание "ОК" и выход (return там)

;Разрешение спящего режима
;Результат: Z=1 - OK принят, Z=0 - нет
SIM_SleepToOn:	call	U1_SendCRATPls		;Команда AT+CSCLK=0
		call	U1_SendCSC
		SerialTransmitChar 'L'
		SerialTransmitChar 'K'
		call	U1_SendEqual
		SerialTransmitChar '0'
_SIM_CmdEndOK:	call	U1_SendCR		;Отправка CR (завершения команды)
		goto	SIMWaitOK		;Ожидание "ОК" и выход (return там)

;Перевод SMS в текстовый режим
;Результат: Z=1 - успешно, Z=0 - нет
SIM_SetTextSMS:	call	U1_SendCRATPls		;Команда AT+CMGF=1
		call	U1_SendCMG
		SerialTransmitChar 'F'
		call	U1_SendEqual
		SerialTransmitChar '1'
		goto	_SIM_CmdEndOK		;Отправка CR, ожидание "ОК" и выход (return там)

;Выбор кодировки GSM
;Результат: Z=1 - успешно, Z=0 - нет
SIM_SetCharGSM:	call	U1_SendCRATPls		;Команда AT+CSCS="GSM"
		call	U1_SendCSC
		call	U1_SendSEqual
		call	U1_SendQuote
		call	U1_SendG
		SerialTransmitChar 'S'
		SerialTransmitChar 'M'
		call	U1_SendQuote
		goto	_SIM_CmdEndOK		;Отправка CR, ожидание "ОК" и выход (return там)

;Удаление всех SMS на SIM-карте
;Результат: Z=1 - успешно, Z=0 - нет
SIM_DelAllSMS:	call	U1_SendCRATPls		;Команда AT+CMGD=1,4
		call	U1_SendCMG
		SerialTransmitChar 'D'
		call	U1_SendEqual
		SerialTransmitChar '1'
		SerialTransmitChar ','
		SerialTransmitChar '4'
		goto	_SIM_CmdEndOK		;Отправка CR, ожидание "ОК" и выход (return там)

;===== ОТПРАВКА СМС =====================================================================

;Для отправки смс вызвать SIM_StartSMS, отправить в USART1 текст смс и вызвать SIM_EndSMS.

;Начало отправки смс
SIM_StartSMS:	bsf	SIMSendSMSIsOK		;Изначальный результат отправки
		call	U1_SendCRATPls		;Команда AT+CMGS="<номер телефона>"
		call	U1_SendCMG
		call	U1_SendSEqual
		call	U1_SendQuote			;Кавычка перед номером
		call	EE_ReadPhoneNr			;Чтение номера телефона в PhoneNr
		mov_lwf	PhoneNr,FSR			;Выдача PhoneNr до нуля
_SIMSSMS_Next:	mov_fw	INDF
		If_Z
		  goto	_SIMSSMS_End
		SerialTransmit
		incf	FSR,ToF
		goto	_SIMSSMS_Next
_SIMSSMS_End:	call	U1_SendQuote			;Кавычка после номера
		call	U1_SendCR		;Переход к тексту смс
		call	SIMWaitPrompt		;Ожидание приёма приглашения "> "
		If_NZ				;Приглашение не пришло => Ошибка отправки
		  bcf	SIMSendSMSIsOK
		return

;Конец отправки смс
;На входе W = код события для добавления в лог (результат отправки смс добавляется процедурой)
;Результат в SIMSendSMSIsOK: 1=отправлено, 0=ошибка
SIM_EndSMS:	movwf	LogData			;Сохранение кода события в LogData
		movlw	.26			;Символ "Ctrl+Z" (конец загрузки смс)
		If_0	SIMSendSMSIsOK		;Ошибка отправки => Вместо "Ctrl+Z" отправляем Esc (отмена загрузки смс)
		  movlw	.27
		SerialTransmit
		call	U1_SendCR		;Отправка CR (на всякий случай)
		If_0	SIMSendSMSIsOK		;Ошибка отправки => Выход
		  goto	SIMESMS_Error
		bcf	SIMWasCMGS		;Сообщение об отправке смс не принято
		call	SIMWaitCMGS		;Ожидание сообщения от отправке смс (до 10 секунд)
		call	SIMWaitCMGS
		If_NZ				;Не принято => Ошибка
		  bcf	SIMSendSMSIsOK
		call	SIMWaitOK		;Ожидание приёма "ОК"
		If_NZ				;Не принято => Ошибка
		  bcf	SIMSendSMSIsOK
		If_0	SIMSendSMSIsOK
		  goto	SIMESMS_Error
		goto	Log_AddLogData		;Сохранение кода события в лог и выход (return там)
SIMESMS_Error:	;Ошибка отправки
		bsf	LogData,0		;Добавление результата "ошибка"
		goto	Log_AddLogData		;Сохранение кода события в лог и выход (return там)

;===== ЗАГРУЗКА СМС =====================================================================

;Загрузка смс с номером SIMNeedRecSMSNr из памяти SIM900D в буфер приёма.
;SIMNeedRecSMSNr очищает.
;Результат: Z=1 - успешно, Z=0 - нет
SIM_ReceiveSMS:	call	U1_SendCRATPls		;Команда AT+CMGR=<SIMNeedRecSMSNr> (чтение смс)
		call	U1_SendCMG
		call	U1_SendR
		call	U1_SendEqual
		clrf	SIMNrH				;Номер смс в SIMNr
		mov_fwf	SIMNeedRecSMSNr,SIMNrL
		call	U1TransmitNr10			;Отправка номера SMS из SIMNr
		clrf	SIMNeedRecSMSNr		;Запрос на приём смс отправлен
		goto	_SIM_CmdEndOK		;Отправка CR, ожидание "ОК" и выход (return там)

;===== ПРОЦЕДУРЫ ПРИЁМА И ОБРАБОТКИ СООБЩЕНИЙ МОДУЛЯ ====================================

;Ожидание приглашения "> " (до 1 секунды)
;Ответ: Z=1 - принято, Z=0 - не принято. Портит DecEach20ms
SIMWaitPrompt:	bsf	SIMPromptWait		;Приглашение не принято, ожидается
		mov_lwf	.50,DecEach20ms		;Максимальное время ожидания 1 секунда
_SIMWP_Wait:	clrwdt				;Сброс WDT (нормальная работа)
		call	SIM_RecMessage		;Обработка входящих сообщений модуля
		If_0	SIMPromptWait		;Сообщение принято => Выход с Z=1
		  goto	_SIMWait_P_OK			;Пауза 100 мс по аналогии с "ОК" (см. ниже) и выход
		test_f	DecEach20ms		;Время ожидания не кончилось => Ждём
		If_NZ
		  goto	_SIMWP_Wait
		bcf	SIMPromptWait		;Ожидание приглашения завершено
_SIMWait_NotOK:	goto	_Exit_With_Z0		;Выход с Z=0

;Ожидание сообщения "ОК" (до 1 секунды)
;Ответ: Z=1 - принято, Z=0 - не принято. Портит DecEach20ms
SIMWaitOK:	bcf	SIMWasOK		;Сообщение не принято
		mov_lwf	.50,DecEach20ms		;Максимальное время ожидания 1 секунда
_SIMWOK_Wait:	clrwdt				;Сброс WDT (нормальная работа)
		call	SIM_RecMessage		;Обработка входящих сообщений модуля
		If_1	SIMWasOK		;Сообщение принято => Выход
		  goto	_SIMWOK_OK
		test_f	DecEach20ms		;Время ожидания не кончилось => Ждём
		If_NZ
		  goto	_SIMWOK_Wait
		goto	_SIMWait_NotOK		;Выход с Z=0
_SIMWOK_OK:	bcf	SIMWasOK		;Сообщение обработано
_SIMWait_P_OK:	call	Pause100ms		;Замечено: SIM900D не любит команд сразу после "ОК" (хотя бы 25 мс паузы)
		goto	_Exit_With_Z1		;Выход с Z=1

;Ожидание сообщения "+CMGS" (до 5 секунд). Предварительно очистить SIMWasCMGS.
;Можно вызывать несколько раз подряд для увеличения времени.
;Ответ: Z=1 - принято, Z=0 - не принято. Портит DecEach20ms
SIMWaitCMGS:	mov_lwf	.250,DecEach20ms	;Максимальное время ожидания 5 секунд
_SIMWCMGS_Wait:	clrwdt				;Сброс WDT (нормальная работа)
		call	SIM_RecMessage		;Обработка входящих сообщений модуля
		If_1	SIMWasCMGS		;Сообщение принято => Выход с Z=1
		  goto	_Exit_With_Z1
		test_f	DecEach20ms		;Время ожидания не кончилось => Ждём
		If_NZ
		  goto	_SIMWCMGS_Wait
		goto	_SIMWait_NotOK		;Выход с Z=0

;***** Анализ строки в буфере приёма *****

;Служебный: сравнение символа и переход к следующему (символ должен быть!).
;Сравниваемый символ Char должен быть в верхнем регистре (для букв 'a'..'z').
;При несовпадении и конце буфера - return (в т.ч. если конец строки наступит раньше).
;На выходе Z - достигнут конец строки (EOL).
U1CompareBuf:	MACRO	Char
		call	_U1GetUpperToW
		xorlw	Char
		If_NZ
		  return
		call	U1IncFSR_CZ
		If_C
		  return
		ENDM

;Служебная: чтение символа из INDF в W с приведением к верхнему регистру
_U1GetUpperToW:	cmp_lwf	'a',INDF		;Менее 'a' => Как есть
		If_2Lt1
		  goto	_U1GUW_AsIs
		cmp_lwf	'{',INDF		;'z' и менее => В верхний регистр, иначе как есть
		If_2Lt1
		  goto	_U1GUW_ToUpper
_U1GUW_AsIs:	mov_fw	INDF			;Возврат символа как есть
		return
_U1GUW_ToUpper:	mov_fw	INDF			;Возврат символа в верхнем регистре
		xorlw	0x20				;0x61..0x7A => 0x41..0x5A
		return

;Служебная: удаление первого символа и возврат к анализу строки
_U1Msg_DelChar:	call	U1IncRdAddr		;Удаление символа
		;Продолжение ниже

;Анализ строки, находящейся в буфере приёма. Вызывать периодически
SIM_RecMessage:	cmp_fwf	U1BufWrAddr,U1BufRdAddr	;Информации в буфере приёма USART нет => Выход
		If_Z
		  return
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		call	U1FSRCheckEOL		;Первый символ - конец строки => Удаляем
		If_Z
		  goto	_U1Msg_DelChar
		test_f	INDF			;Первый символ 0x00 => Удаляем
		If_Z
		  goto	_U1Msg_DelChar
		If_0	SIMPromptWait		;Приглашение не ожидается => Пропуск
		  goto	_U1Msg_FindEOL
		call	_U1Msg_IfPrmt		;Поиск приглашения
		If_0	SIMPromptWait		;Приглашение найдено => Всё сначала
		  goto	SIM_RecMessage
		call	U1RdAddrToFSR		;Повторная загрузка адреса чтения в FSR
_U1Msg_FindEOL:	call	U1IncFSR_CZ		;Ищем в строке два символа конца строки (обычно CR+LF)
		If_C					;Строка кончилась, EOL не найден => Ждём
		  return
		If_NZ					;EOL не найден => Ищем дальше
		  goto	_U1Msg_FindEOL
		call	U1IncFSR_CZ			;Следующий символ
		If_C					;Строка кончилась, EOL не найден => Ждём
		  return
		If_NZ					;EOL не найден => Ищем дальше
		  goto	_U1Msg_FindEOL
		;В буфере есть законченная строка с двумя EOL (обычно CR+LF): распознавание сообщений
		bcf	SIMMsgDetected		;Сообщение не распознано
		call	_U1Msg_IfOK		;Поиск и обработка сообщения "OK"
		If_1	SIMMsgDetected		;Сообщение распознано => Всё сначала
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSI		;Поиск и обработка сообщения о входящем смс в буфере
		If_1	SIMMsgDetected		;Сообщение распознано => Всё сначала
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSR		;Поиск и обработка сообщения о приёме смс
		If_1	SIMMsgDetected		;Сообщение распознано => Всё сначала
		  goto	SIM_RecMessage
		call	_U1Msg_IfSMSS		;Поиск и обработка сообщения об отправке смс
		If_1	SIMMsgDetected		;Сообщение распознано => Всё сначала
		  goto	SIM_RecMessage
		;Ни одно сообщение не распознано
		Debug_OutLine			;Отладочный режим включен => Выдача строки до EOL
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
_U1Msg_ToEOL:	call	U1IncFSR_CZ		;Удаление строки до двух EOL (обычно CR+LF)
		If_NZ
		  goto	_U1Msg_ToEOL
		call	U1IncFSR_CZ
		If_NZ
		  goto	_U1Msg_ToEOL
		call	_U1Msg_MsgEOL
		goto	SIM_RecMessage		;Обработка остального содержимого буфера (если есть)

_U1Msg_IfPrmt:	;Поиск пары символов "> " с текущего места
		U1CompareBuf '>'		;Посимвольное сравнение
		cmp_lwf	' ',INDF			;Следом конец буфера, поэтому U1CompareBuf нельзя
		If_NZ
		  return
		call	U1IncFSR_CZ			;Перемещаемся далее, там может быть и конец буфера (C=1)
		bcf	SIMPromptWait		;Сочетание найдено
		Debug_Prompt			;Отладочный режим => Сообщение о приёме "> "
		goto	U1FSRToRdAddr		;Удаление приглашения из буфера и выход (return там)

_U1Msg_IfOK:	;Поиск сообщения "OK"
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'O'		;Посимвольное сравнение
		U1CompareBuf 'K'
		If_NZ				;Не конец строки => Выход
		  return
		bsf	SIMMsgDetected		;Сообщение распознано
		bsf	SIMWasOK		;Принято "ОК"
		Debug_OK			;Отладочный режим => Сообщение о приёме "ОК"
		goto	_U1Msg_MsgEOL		;Завершение обработки сообщения

_U1Msg_IfSMSI:	;Поиск сообщения "+CMTI: <"mem">,<index>" (входящая смс)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf '+'		;Посимвольное сравнение
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'T'
		U1CompareBuf 'I'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;Сообщение распознано
_U1MsgIfSMSI_1:	If_Z				;Строка кончилась => Ошибка, удаляем всё
		  goto	_U1Msg_MsgEOL
		cmp_lwf	',',INDF		;Поиск запятой: Текущий символ ',' => Нашли
		If_Z
		  goto	_U1MsgIfSMSI_2
		call	U1IncFSR_CZ			;Следующий символ
		goto	_U1MsgIfSMSI_1
_U1MsgIfSMSI_2:	call	U1IncFSR_CZ		;Запятая найдена: Пропуск запятой
		If_Z					;Строка кончилась => Ошибка, удаляем всё
		  goto	_U1Msg_MsgEOL
		call	U1ReceiveNr10		;Приём номера SMS в SIMNr (портит Tmp1)
		Debug_SMSI			;Отладочный режим => Сообщение о приёме смс и его номере в памяти
		mov_fw	SIMNrL			;Номер сообщения, которое необходимо принять
		test_f	SIMNrH				;Номер сообщения > 255 => Не поддерживается
		If_Z
		  movwf	SIMNeedRecSMSNr			;Сохранение номера
		goto	_U1Msg_MsgEOL		;Завершение обработки сообщения

_U1Msg_IfSMSR:	;Поиск сообщения "+CMGR: <"статус">,<"номер">,<"">,<"дата">//<текст>" (входящая смс)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf '+'		;Посимвольное сравнение
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'G'
		U1CompareBuf 'R'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;Сообщение распознано
_U1MsgIfSMSR_1:	If_Z				;Строка кончилась => Ошибка, удаляем всё
		  goto	_U1Msg_MsgEOL
		cmp_lwf	',',INDF		;Поиск запятой: Текущий символ ',' => Нашли
		If_Z
		  goto	_U1MsgIfSMSR_2
		call	U1IncFSR_CZ			;Следующий символ
		goto	_U1MsgIfSMSR_1
_U1MsgIfSMSR_2:	call	U1IncFSR_CZ		;Запятая найдена: Пропуск запятой
		If_Z					;Строка кончилась => Ошибка, удаляем всё
		  goto	_U1Msg_MsgEOL
		cmp_lwf	'"',INDF		;Следующий символ не кавычка => Ошибка, удаляем всё
		If_NZ
		  goto	_U1Msg_MsgEOL
		call	U1IncFSR_CZ		;Кавычка найдена: Пропуск кавычки
		If_Z					;Строка кончилась => Ошибка, удаляем всё
		  goto	_U1Msg_MsgEOL
		call	ReceivePhoneNr		;Копирование номера отправителя из буфера в PhoneNr
		cmp_lwf	'"',INDF		;Следующий символ не кавычка => Ошибка, удаляем всё
		If_NZ
		  goto	_U1Msg_MsgEOL
_U1MsgIfSMSR_3:	call	U1IncFSR_CZ		;Удаление строки до EOL
		If_NZ
		  goto	_U1MsgIfSMSR_3
		call	_U1Msg_MsgEOL			;Удаляем всё здесь, чтобы освободить буфер для текста смс
		Debug_SMSR			;Отладочный режим => Сообщение о приёме смс и номере телефона
		call	SIMRecSMSText		;Ожидание, приём, обработка и удаление текста смс
		bsf	SIMNeedDelSMS		;Требуется удалить все смс после приёма "ОК"
		return				;Здесь конец строки не удаляется, т.к. может им не являться

_U1Msg_IfSMSS:	;Поиск сообщения "+CMGS: <index>" (смс отправлено)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf '+'		;Посимвольное сравнение
		U1CompareBuf 'C'
		U1CompareBuf 'M'
		U1CompareBuf 'G'
		U1CompareBuf 'S'
		U1CompareBuf ':'
		bsf	SIMMsgDetected		;Сообщение распознано
		bsf	SIMWasCMGS		;Смс отправлена
_U1MsgIfSMSS_1:	call	U1IncFSR_CZ		;Удаление строки до EOL
		If_NZ
		  goto	_U1MsgIfSMSS_1
		;Продолжение ниже		;Завершение обработки сообщения

_U1Msg_MsgEOL:	;Завершение обработки сообщения при нахождении EOL
		call	U1IncFSR_CZ		;Пропуск символа конца строки
		If_C				;Строка кончилась => Выход
		  goto	_U1Msg_DelLine
		If_Z				;Следующий символ тоже конец строки => Тоже пропускаем
		  goto	_U1Msg_MsgEOL
_U1Msg_DelLine:	goto	U1FSRToRdAddr		;Удаление строки до текущего места и выход (return там)

;===== ПРОЦЕДУРЫ ПРИЁМА ТЕКСТА СМС-СООБЩЕНИЙ ============================================

;Если sms содержит более одной строки (строки текста смс разделяются не 0x0D+0x0A, а просто 0x0A),
;следующие строки пройдут позже как сообщения модуля и будут удалены, т.к. не будут распознаны.
;Переполнение (оно возможно, т.к. текст смс может быть более размера буфера) при этом не страшно,
;так как каждые 80 символов буфер "опустошается", последним всё равно идёт EOL и OK.
;Если смс-команда не распознана или смс 20 и более символов, она не удаляется, а будет обработана
;позже как сообщение модуля и будет удалена, т.к. не будет распознана.
;Если смс пустая, сообщение "ОК" распознано не будет, не удаляется и будет распознано позже как "ОК".

;Ожидание, приём, обработка и удаление текста смс
;Чтение из INDF, после выхода останавливается на EOL
;Портит Tmp1
SIMRecSMSText:	cmp_fwf	U1BufRdAddr,U1BufWrAddr	;Символов в буфере нет => Ждём
		If_Z
		  goto	SIMRecSMSText
		call	U1FSRCheckEOL		;Символ принят: Не символ конца строки => Обработка
		If_NZ
		  goto	_SIMRSTST_Text
		call	U1IncFSR_CZ		;Символ конца строки: Пропуск символа
		call	U1FSRToRdAddr			;Удаление строки до текущего места для фиксации начала значащего символа
		goto	SIMRecSMSText		;Ожидание следующего символа
_SIMRSTST_Text:	;Ожидание приёма текста смс (минимум 1 символ есть)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		clrf	Tmp1			;Счётчик числа символов
_SIMRSTST_T_1:	incf	Tmp1,ToF		;Подсчёт числа символов в строке
		cmp_lwf	.20,Tmp1		;Строка 20 символов и более => Выход без анализа и удаления строки
		If_2GE1
		  goto	_SIMRSTST_Err
		call	U1IncFSR_CZ		;Ищем в строке символы конца строки
		If_C					;Строка кончилась, EOL не найден => Ждём
		  goto	_SIMRSTST_Text
		If_NZ					;Найден EOL => Обработка, иначе ищем дальше
		  goto	_SIMRSTST_T_1
		;Смс менее 20 символов с EOL на конце принята
		bcf	SIMSMSDetected		;Смс-команда не распознана
		call	_SIMRSTST_STN		;Поиск сообщения "SetThisNumber"
		call	_SIMRSTST_T		;Поиск и обработка команды "T"
		call	_SIMRSTST_R		;Поиск и обработка команды "R"
		call	_SIMRSTST_OR		;Поиск и обработка команды "ObratnoeR"
		call	_SIMRSTST_G		;Поиск сообщения "Glubina NNN"
		If_0	SIMSMSDetected		;Ни одна смс-команда не распознана => Выход без удаления строки
		  goto	_SIMRSTST_Err
		;Смс-команда распознана
		Debug_OutCmd			;Отладочный режим => Выдача текста смс-команды
		call	U1RdAddrToFSR		;Удаление обработанной смс-команды: Загрузка адреса чтения в FSR
_SIMRSTST_T_2:	call	U1IncFSR_CZ			;Поиск конца строки
		If_NZ
		  goto	_SIMRSTST_T_2
		call	U1FSRToRdAddr			;Удаление строки до текущего места
		return
_SIMRSTST_Err:	;Смс-команда не распознана
		Debug_NoCmd			;Отладочный режим => Сообщение о нераспознавании команды
		goto	U1RdAddrToFSR		;Загрузка адреса чтения в FSR и выход (return там)

_SIMRSTST_STN:	;Поиск сообщения "SetThisNumber" (установка номера)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'S'		;Посимвольное сравнение
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
		If_NZ				;Конец строки не достигнут => Выход
		  return
		bsf	SIMSMSDetected		;Смс-команда распознана
		bsf	NeedSendAll		;Необходимо отправить смс с параметрами
		goto	EE_SavePhoneNr		;Сохранение номера в EEPROM и выход (return там)

_SIMRSTST_T:	;Поиск сообщения "T" (запрос температуры)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'T'		;Посимвольное сравнение
		If_NZ				;Конец строки не достигнут => Выход
		  return
		bsf	SIMSMSDetected		;Смс-команда распознана
		bsf	NeedSendT		;Необходимо отправить смс с температутой
		return

_SIMRSTST_R:	;Поиск сообщения "R" (запрос расстояния)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'R'		;Посимвольное сравнение
		If_NZ				;Конец строки не достигнут => Выход
		  return
		bsf	SIMSMSDetected		;Смс-команда распознана
		bsf	NeedSendR		;Необходимо отправить смс с расстоянием
		return

_SIMRSTST_OR:	;Поиск сообщения "ObratoeR" (запрос обратного расстояния)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'O'		;Посимвольное сравнение
		U1CompareBuf 'B'
		U1CompareBuf 'R'
		U1CompareBuf 'A'
		U1CompareBuf 'T'
		U1CompareBuf 'N'
		U1CompareBuf 'O'
		U1CompareBuf 'E'
		U1CompareBuf 'R'
		If_NZ				;Конец строки не достигнут => Выход
		  return
		bsf	SIMSMSDetected		;Смс-команда распознана
		bsf	NeedSendOR		;Необходимо отправить смс с обратным расстоянием
		return

_SIMRSTST_G:	;Поиск сообщения "Glubina NNN" (установка глубины)
		call	U1RdAddrToFSR		;Загрузка адреса чтения в FSR
		U1CompareBuf 'G'		;Посимвольное сравнение
		U1CompareBuf 'L'
		U1CompareBuf 'U'
		U1CompareBuf 'B'
		U1CompareBuf 'I'
		U1CompareBuf 'N'
		U1CompareBuf 'A'
		U1CompareBuf ' '
		If_Z				;Достигнут конец строки => Выход
		  return
		call	U1ReceiveNr10		;Приём глубины в SIMNr (портит Tmp1)
		ior_ffw	SIMNr			;Глубина = 0 => Выход
		If_Z
		  return
		call	U1FSRCheckEOL		;Конец строки не достигнут => Выход
		If_NZ
		  return
		bsf	SIMSMSDetected		;Смс-команда распознана
		bsf	NeedSendG		;Необходимо отправить смс с глубиной
		mov2fwf	SIMNr,Glubina		;Новое значение глубины
		goto	EE_SaveGlubina		;Запись глубины из Glubina и выход (return там)

;===== ПРОЦЕДУРЫ ПРИЁМА И ОТПРАВКИ ЗНАЧЕНИЙ =============================================

;Определение, что под указателем INDF цифра
;Результат: Z=1 - цифра, Z=0 - нет
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

;Приём числа в десятичном виде из буфера под указателем INDF в SIMNr
;Указатель останавливается на символе, следующем на числом. Портит Tmp1
U1ReceiveNr10:	clr2f	SIMNr			;Подготовка результата
_U1RecNr_Next:	call	U1Buf_IfDigit		;Под указателем не цифра => Выход
		If_NZ
		  return
		clrc				;Умножение SIMNr на 10: Умножение на 2
		rl2f	SIMNr
		mov_fwf	SIMNrH,Tmp1			;Умножение на 5: Сохранение значения в Tmp1:W
		mov_fw	SIMNrL
		clrc						;Умножение на 4
		rl2f	SIMNr
		clrc
		rl2f	SIMNr
		addwf	SIMNrL,ToF				;Добавление Tmp1:W к значению
		If_C
		  incf	SIMNrH,ToF
		add_fwf	Tmp1,SIMNrH
		mov_fw	INDF			;Цифра в W с преобразованием в 0..9
		addlw	-'0'
		addwf	SIMNrL,ToF		;Добавление W к результату
		If_C
		  incf	SIMNrH,ToF
		call	U1IncFSR_CZ		;Переход к следующему символу (конец строки не проверяется, т.к. ранее EOL)
		goto	_U1RecNr_Next		;Зацикливание

;Отправка числа в десятичном виде (0...999) из SIMNr с подавлением ведущих нулей
;Портит арифметические переменные
U1TransmitNr10:	mov2fwf	SIMNr,PROD2		;Отправляемое число
		cmplw2f	.100,PROD2		;Начало отправки с нужного разряда
		If_2GE1
		  goto	_U1TNr_100
		cmp_lwf	.10,PRODL
		If_2GE1
		  goto	_U1TNr_10
		goto	_U1TNr_1
_U1TNr_100:	mov_lwf	.100,MulArg		;Делитель 3-го разряда
		call	_U1TNr_DivSend		;Получение и отправка цифры 3-го разряда
_U1TNr_10:	mov_lwf	.10,MulArg		;Делитель 2-го разряда
		call	_U1TNr_DivSend		;Получение и отправка цифры 2-го разряда
_U1TNr_1:	mov_fw	PRODL			;Значение 1-го разряда
		goto	_U1TNr_SendW		;Отправка и выход (return там)
_U1TNr_DivSend:	;Обработка разряда
		call	Divide_2To1		;Деление DivRes2:=PROD2/MulArg (0..9), остаток в DivOst
		clrf	PRODH			;Подготовка для следующего разряда
		mov_fwf	DivOst,PRODL
		mov_fw	DivResL			;Отправляемое значение разряда
_U1TNr_SendW:	addlw	'0'			;Преобразование в цифру
		SerialTransmit			;Отправка
		return

;Копирование номера отправителя из INDF в PhoneNr
;Портит Tmp1, Tmp2, Tmp3
ReceivePhoneNr:	mov_lwf	PhoneNr,Tmp1		;Адрес записи номера
		mov_lwf	.16,Tmp2		;Максимальная длина номера (15 знаков + 1 для фиксации превышения)
_RPN_Next:	cmp_lwf	'"',INDF		;Найдена кавычка => Завершение
		If_Z
		  goto	_RPN_End
		mov_fwf	INDF,Tmp3		;Чтение очередного символа
		chg_fwf	Tmp1,FSR		;Адрес записи номера в FSR
		mov_fwf	Tmp3,INDF		;Сохранение очередного символа
		chg_fwf	Tmp1,FSR		;Адрес чтения назад в FSR
		incf	Tmp1,ToF		;Инкремент адреса чтения
		call	U1IncFSR_CZ		;Переход к следующему символу
		If_Z				;Строка кончилась => Ошибка, стираем весь номер
		  goto	_RPN_Err
		loop_f	Tmp2,_RPN_Next		;Зацикливание
_RPN_Err:	mov_lwf	PhoneNr,Tmp1		;Было записано 16 цифр => Ошибка, стираем весь номер
_RPN_End:	chg_fwf	Tmp1,FSR		;Адрес записи номера в FSR
		clrf	INDF			;Запись нуля в конец номера
		mov_fwf	Tmp1,FSR		;Адрес чтения назад в FSR
		return

;========================================================================================
