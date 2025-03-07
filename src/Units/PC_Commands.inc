;========================================================================================
;Измеритель температуры и расстояния с GSM-каналом
;Обработка команд ЭВМ, полученных по USART2
;========================================================================================

;Команды ПК должны подаваться только при отсутствии активности по USART1 (обмена с SIM900D)!

;===== ПРОЦЕДУРА ОБРАБОТКИ ==============================================================

;Процедура обработки команд ЭВМ. Вызывать при U2_IsNewRcChar=1
U2_RecPCCmd:	Serial2Receive			;Приём символа в W
		;Проверка корректности команды
		addlw	-'{'			;Всё что больше 'z' - неверно
		If_C
		  return
		addlw	'{'-'a'			;Всё что меньше 'a' - неверно
		If_NC
		  return
		;Вычисляемый переход на команду по W ('a'=0)
		TableJumpW _RC_JmpTab		;Переход
_RC_JmpTab:	;Таблица переходов (начиная с 'a')
		goto	_Cmd_a			;'a' - чтение состояния входа "Тревога"
		return				;'b'
		goto	_Cmd_c			;'c' - очистка лога без чтения (также инициализация указателей)
		return				;'d'
		return				;'e'
		return				;'f'
		goto	_Cmd_g			;'g' - выдача текущей глубины
		return				;'h'
		return				;'i'
		return				;'j'
		return				;'k'
		goto	_Cmd_l			;'l' - чтение лога с его очисткой
		return				;'m'
		goto	_Cmd_n			;'n' - выдача сохранённого номера телефона
		return				;'o'
		return				;'p'
		return				;'q'
		goto	_Cmd_r			;'r' - измерение и выдача расстояния в см
		return				;'s'
		goto	_Cmd_t			;'t' - измерение и выдача температуры в °C
		return				;'u'
		return				;'v'
		return				;'w'
		return				;'x'
		return				;'y'
		return				;'z'

;***** Команды измерений *****

;'t' - измерение и выдача температуры в °C
;Значение выдаётся в Hex-виде в дополнительном коде (00 = 0°C, 7F = 127°C, FF = -1°C)
_Cmd_t:		call	GetTemperature		;Измерение
		call	_Cmd_OutStart		;Начало выдачи
		mov_fw	DS18B20_TL		;Выдача
		If_0	DS18B20_Present			;Ошибка => Код ошибки
		  movlw	0x80
		Serial2HexTransmit			;Выдача значения
_Cmd_End:	Serial2TransmitChar '>'		;Конец выдачи
		return

;'r' - измерение и выдача расстояния в см
;Значение выдаётся в Hex-виде в сантиметрах (0000 - датчик отсутствует; FFFF - ошибка измерения)
_Cmd_r:		call	GetDistance		;Измерение
		call	_Cmd_OutStart		;Начало выдачи
		Serial2HexTransmitPort Dist_smH	;Выдача
		Serial2HexTransmitPort Dist_smL
		goto	_Cmd_End		;Конец выдачи и выход (return там)

;'a' - чтение состояния входа "Тревога"
;Результат: '0' - низкий уровень; '1' - высокий уровень
_Cmd_a:		call	_Cmd_OutStart		;Начало выдачи
		movlw	'0'			;Выдача: Формирование результата в W
		If_1	In_Trevoga
		  movlw	'1'
		Serial2Transmit				;Выдача результата из W
		goto	_Cmd_End		;Конец выдачи и выход (return там)

;***** Команды параметров связи и отправки сообщений *****

;'n' - выдача сохранённого номера телефона
_Cmd_n:		call	EE_ReadPhoneNr		;Чтение номера телефона в PhoneNr
		call	_Cmd_OutStart		;Начало выдачи
		mov_lwf	PhoneNr,FSR		;Выдача
_Cmdn_Next:	mov_fw	INDF				;Очередной символ номера в W
		If_Z					;Номер закончился => Конец выдачи и выход (return там)
		  goto	_Cmd_End
		Serial2Transmit				;Выдача символа из W
		incf	FSR,ToF				;Переход к следующему символу номера
		goto	_Cmdn_Next

;'g' - выдача текущей глубины
_Cmd_g:		call	EE_ReadGlubina		;Чтение глубины в Glubina
		Serial2TransmitChar '<'		;Начало выдачи
		Serial2HexTransmitPort GlubinaH	;Выдача
		Serial2HexTransmitPort GlubinaL
		goto	_Cmd_End		;Конец выдачи и выход (return там)

;***** Команды работы с логом *****

;'l' - чтение лога с его очисткой
_Cmd_l:		call	_Cmd_OutStart		;Начало выдачи
		call	Log_ReadFirst		;Выдача: Чтение первого значения
		If_Z					;Значений нет => Конец выдачи и выход (return там)
		  goto	_Cmd_End
_Cmdl_Out:	Serial2HexTransmitPort LogData		;Выдача значения
		call	Log_ReadFirst			;Чтение следующего значения
		If_Z					;Значения кончились => Конец выдачи и выход (return там)
		  goto	_Cmd_End
		Serial2TransmitChar ','			;Разделитель
		goto	_Cmdl_Out			;Выдача значения и переход к следующему

;'c' - очистка лога без чтения (также инициализация указателей)
_Cmd_c:		call	Log_Clear		;Выполнение
		call	Log_Init		;Повторная загрузка указателей (для контроля)
		goto	_Cmd_l			;Выдача пустого лога (для контроля)

;===== СЛУЖЕБНЫЕ ПРОЦЕДУРЫ =============================================================

;Выдача символа '<'
_Cmd_OutStart:	Serial2TransmitChar '<'
		return

;========================================================================================
