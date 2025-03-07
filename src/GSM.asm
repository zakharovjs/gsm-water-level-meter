;========================================================================================
;Измеритель температуры и расстояния с GSM-каналом
;========================================================================================

;Минимальный период отправки смс "Света нет". Значение в единицах по 5120 мс.
#define		SvetaNet_SMSPeriod	.58	;Примерно 5 минут

;Период проверки связи с модулем SIM900D. Значение в единицах по 5120 мс.
#define		ATOK_Period		.12	;Примерно 1 минута

;Примеры значений для параметров выше:
;	.12	- Примерно 1 минута (5120*12 = 61.44 с)
;	.23	- Примерно 2 минуты (5120*23 = 117.76 с)
;	.58	- Примерно 5 минут (5120*58 = 296.96 с)
;	.117	- Примерно 10 минут (5120*117 = 599.04 с)
;	.234	- Примерно 20 минут (5120*234 = 1198.08 с)

;Параметры отладочных режимов (модуль Debug.inc), 1=включена, 0=выключена
#define		Debug_SIM		.0	;Отладка управления SIM900D
#define		Debug_Log		.0	;Отладка (контроль) записи в лог
;В рабочей версии программы должны быть все отключены!

;========================================================================================

	processor	16f628a
	#include	"p16f628a.inc"
	list		p = 16f628a

	__config	0x1F14		;13	CP	0	Защита памяти программ есть
					;12..9	----	1111	Не используются
					;8	CPD	1	Защиты памяти данных нет
					;7	LVP	0	Низковольтного программирования нет
					;6	BODEN	0	Сброс по питанию (BOR) запрещён
					;5	MCLRE	0	RA5/-MCLR работает как RA3
					;4	FOSC2	1	Тактовый генератор INTRC
					;3	-PWRTE	0	PWRT включен
					;2	WDTE	1	WDT включен
					;1..0	FOSC1:0	00	Тактовый генератор INTRC без CLKOUT

;===== ПОДКЛЮЧЕНИЕ ПЕРИФЕРИИ ============================================================

;Кнопка PwrKey
#define		IO_PwrKey	PORTA,2		;Для чтения нажатия PWRKEY пользователем и выдачи значения (0=нажата)
#define		PwrKey_TRIS	TRISA,2		;Для нажатия PWRKEY контроллером (RA2 как выход, на выходе 0 = нажата)
;Не выдавать на выход активную единицу, т.к. кнопка замыкает вывод на землю!
;Нажатие кнопки контроллером производить выдачей на выход 0 и затем переводом вывода в выход (PwrKey_TRIS=0).

;Датчик HC-SR04
#define		HCSR04_Vcc	PORTA,3		;Выход для подачи питания на датчик
#define		HCSR04_Trig	PORTA,4		;Выход для начала измерений датчика
#define		HCSR04_Echo	PORTB,3		;Вход для приёма сигнала датчика (CCP1)

;Сигнал "Тревога"
#define		In_Trevoga	PORTB,0		;INT - Вход для приёма сигнала (0=тревога)

;Датчик DS18B20
#define		DS18B20_Vdd	PORTA,7		;Выход для подачи питания на датчик
#define		DS18B20_DQ_PORT	PORTA,6		;Линия данных датчика
#define		DS18B20_DQ_TRIS	TRISA,6		;Направление линии данных датчика

;Выводы USART2
#define		U2_RX		PORTB,5		;Вход для приёма по USART2
#define		U2_TX		PORTB,4		;Выход для передачи по USART2
;				PORTB,1		RX - вход для приёма по USART
;				PORTB,2		TX - выход для передачи по USART

;Неиспользуемые выводы
#define		Free_RA0	PORTA,0
#define		Free_RA1	PORTA,1

;===== ИСПОЛЬЗОВАНИЕ РЕСУРСОВ МИКРОКОНТРОЛЛЕРА ==========================================

;CCP1 - измерение расстояния
;TMR1 - измерение расстояния
;TMR2 - генерация периодических прерываний
;USART (аппаратный) - связь с SIM900
;USART2 (программный) - связь с ПК
;WDT (период примерно 2.3 с) - для сброса МК при зависании программы

;===== ПЕРЕМЕННЫЕ =======================================================================

#define		U1BufSize	.80		;Размер буфера приёма USART

		CBLOCK	0x70			;Переменные в банке общего доступа (0x70..0x7F - 16 байт)
		  U1BufWrAddr:		1	  ;Смещение для записи а буфер U1Buf (0..U1BufSize-1)
		  U1BufRdAddr:		1	  ;Смещение для чтения из буфера U1Buf (0..U1BufSize-1)
		  ;Переменные прерывания
		  W_Temp,Status_Temp:	1	  ;Переменные для сохранения контекста
		  FSR_Temp:		1
		ENDC

		CBLOCK	0xA0			;Переменные в банке 1 (0xA0..0xEF - 80 байт)
		  U1Buf:		U1BufSize  ;Буфер приёма USART
		ENDC

		CBLOCK	0x20			;Переменные в банке 0 (0x20..0x6F - 80 байт)
		  Count100ms:		1	  ;Счётчик для отсчёта 100 мс
		  GlubinaH,GlubinaL:	1	  ;Глубина [см]
		  Tmp1,Tmp2,Tmp3:	1	  ;Временные переменные
		  ;Переменные прерывания
		  Count20ms:		1	  ;Отсчёт 20 мс
		  Count5120ms:		1	  ;Отсчёт 5120 мс
		  DecEach20ms:		1	  ;Декрементируется каждые 20 мс до нуля
		  DecEach5120ms_NoSvet:	1	  ;Декрементируется каждые 5120 мс до нуля (запрет смс "Света нет")
		  IncEach5120ms_ATOK:	1	  ;Декрементируется каждые 5120 мс до 255 (таймер проверки связи)
		ENDC

;Двухбайтовые переменные
#define		Glubina		GlubinaH,GlubinaL

;===== МАКРОСЫ И АЛЬТЕРНАТИВНЫЕ ИНСТРУКЦИИ ==============================================

#include "D:\Working\Common\PicUnits\12_16\AddInstr.inc"
#include "D:\Working\Common\PicUnits\16\BankSel302.inc"

;Подключение модуля USART2.inc (до прерывания, т.к. макросы используются в нём)
#include "D:\Working\Common\PicUnits\12_16\USART2.inc"

;===== ВЕКТОР ЗАПУСКА ===================================================================

		ORG	0x0000

		Bank_0
		clrf	INTCON			;Запретить все прерывания
		goto	Main

;===== ВЕКТОР ПРЕРЫВАНИЯ ================================================================

		ORG	0x0004

		Was_In_Bank_0

;Прерывание здесь происходит только от TMR2 каждые 138 мкс.
;Прерывание отвечает за:
;	1. Обработку USART2.
;	2. Приём с USART в буфер приёма.
;	3. Отсчёт временных интервалов.

;Запись в PORTA (включая bcf, bsf) в прерывании не допускается, т.к. на этом порту две двунаправленные линии!
;Использовать call в прерывании не допускается, так как возникает переполнение стека!

Intr:		;***** Сохранение контекста и сброс TMR2 *****
		movwf	W_Temp			;Сохранение контекста
		swapf	STATUS,ToW
		movwf	Status_Temp
		clrf	STATUS			;Выбор банка 0 одной инструкцией (Was_In_Bank_0 есть выше)
		bcf	PIR1,TMR2IF		;Сброс флага прерывания

		;***** Работа с USART2 (постоянное смещение!) *****
		U2_Intr_Fast			;Макросы обработки USART2 (до 22 тактов [36 при одновр. приёме и передаче])
		;Конец работы с USART2

		;***** Приём по USART (3..17 тактов) *****
		If_0	PIR1,RCIF		;Приём не требуется (аналог If_SerialReceiveNotNeed) => Пропускаем
		  goto	_IU1_Exit
		mov_fwf	FSR,FSR_Temp		;Сохранение FSR, т.к. программа тоже использует (IRP сохраняется со STATUS)
		bcf	STATUS,IRP		;9-й бит адреса (IRP:FSR) = 0, т.к. буфер в банке 1
		movlw	U1Buf			;Расчёт адреса записи в FSR
		addwf	U1BufWrAddr,ToW
		movwf	FSR
		mov_fw	RCREG			;Приём байта в W (аналог SerialHardReceiveToW)
		movwf	INDF			;Сохранение W в буфер
		incf	U1BufWrAddr,ToF		;Инкремент U1BufWrAddr с зацикливанием (0..U1BufWrAddr-1=>0)
		cmp_lwf	U1BufSize,U1BufWrAddr
		If_2GE1
		  clrf	U1BufWrAddr
		mov_fwf	FSR_Temp,FSR		;Восстановление FSR (IRP восстанавливается далее со STATUS)
_IU1_Exit:	;Конец работы с USART

		;***** Отсчёт временных интервалов (4..14 тактов) *****
		decfsz	Count20ms,ToF		;Отсчёт 20 мс
		  goto	_IT_Exit
		mov_lwf	.145,Count20ms
		mov_fw	DecEach20ms		;Действия каждые 20 мс: Отсчёт DecEach20ms до нуля
		If_NZ
		  decf	DecEach20ms,ToF
		decfsz	Count5120ms,ToF		;Отсчёт 5120 мс
		  goto	_IT_Exit
		mov_fw	DecEach5120ms_NoSvet	;Действия каждые 5120 мс: Отсчёт DecEach5120ms_NoSvet до нуля
		If_NZ
		  decf	DecEach5120ms_NoSvet,ToF
		incfsz	IncEach5120ms_ATOK,ToW		;Инкремент IncEach5120ms_ATOK до 255
		  incf	IncEach5120ms_ATOK,ToF
_IT_Exit:	;Конец отсчёта временных интервалов

		;***** Восстановление контекста и выход из прерывания *****
		swapf	Status_Temp,ToW
		movwf	STATUS
		swapf	W_Temp,ToF
		swapf	W_Temp,ToW
		retfie

;Длительность нахождения в прерывании:
;15 - переход (4), сохранение контекста и перезапуск TMR1 (5), восстановление контекста и выход (6)
;22 - работа с USART2 (до 36 при одновременных приёме и передаче)
;17 - приём по USART
;4..14 - отсчёт временных интервалов
;Итого: 15+22+17+14=68 тактов
;Итого: 15+36+17+14=82 такта при одновременных приёме и передаче

;===== МОДУЛИ ОБЩЕГО НАЗНАЧЕНИЯ =========================================================

;***** АРИФМЕТИКА *****

#define		Arifm_Mul		0	;Умножение не используется
#define		Arifm_Div		1	;Деление используется

#include "D:\Working\Common\PicUnits\12_16\Arifm.inc"

;***** USART *****

#define		BAUD_CONSTANT	0x67		;Скорость обмена по USART 2400 бод

#include "D:\Working\Common\PicUnits\16\USART.inc"
#include "D:\Working\Common\PicUnits\12_16\USARTHex.inc"

;***** USART2 *****

;Модуль USART2.inc подключен выше

		U2_Procedures			;Подключение макроса процедур для USART2

#include "D:\Working\Common\PicUnits\12_16\USART2Hx.inc"

;***** Датчики *****

#define		MCU_Speed	.1000		;Скорость работы МК 1 MIPS (Fosc = 4 МГц)

#include "D:\Working\Common\PicUnits\12_16\MCUPauses.inc"
#include "D:\Working\Common\PicUnits\12_16\DS18B20.inc"

;===== ПРОЦЕДУРЫ ========================================================================

;Пауза 80-100 мс. Портит DecEach20ms
Pause100ms:	movlw	.5
		goto	Pause_Wx20ms

;Пауза 480-500 мс. Портит DecEach20ms
Pause500ms:	movlw	.25
		goto	Pause_Wx20ms

;Пауза 980-1000 мс. Портит DecEach20ms
Pause1s:	movlw	.50
		;Продолжение ниже

;Пауза W*20 мс. Портит DecEach20ms
Pause_Wx20ms:	movwf	DecEach20ms		;Задание значения паузы
		clrwdt				;Сброс WDT (нормальная работа)
		WaitFUntil0 DecEach20ms		;Ожидание завершения паузы
		return

;===== ФУНКЦИИ ОТЛАДКИ ==================================================================

#include "Units\Debug.inc"

;===== ФУНКЦИИ ВЕДЕНИЯ ЛОГА =============================================================

#include "Units\Log.inc"

;===== ПРОЦЕДУРЫ РАБОТЫ С ДАТЧИКАМИ =====================================================

;***** Температура *****

;Измерение температуры датчика DS18B20 в DS18B20_TL со знаком (заполняет DS18B20_Present)
;Длительность до 220 мс
GetTemperature:	bsf	DS18B20_Vdd		;Подача питания датчика
		call	Pause100ms		;Пауза 80-100 мс для стабилизации питания и запуска датчика
		DS18B20Init			;Инициализация DS18B20
		DS18B20Set9bit			;Результат DS18B20 - 9 бит (0.5 °C, преобразование 93.75 мс)
		DS18B20ConvertT			;Запуск преобразования (преобразование для 9 бит идёт 93.75 мс)
		movlw	.6			;Пауза 100-120 мс для измерений
		call	Pause_Wx20ms
		DS18B20ReadT			;Чтение результата в DS18B20_T (также заполняет DS18B20_Present)
		bcf	DS18B20_Vdd		;Отключение питания датчика
		call	_GT_4rrfT		;Формирование результата в °C
		movlw	0x32			;Добавление в лог: Код события
		If_0	DS18B20_Present			;Ошибка измерения => Добавление ошибки в код
		  iorlw	0x01
		goto	Log_AddW			;Добавление и выход (return там)
_GT_4rrfT:	;Служебная: Формирование результата в °C: DS18B20_TL:=DS18B20_T/16
		call	_GT_2rrfT
_GT_2rrfT:	call	_GT_rrfT
_GT_rrfT:	rrf	DS18B20_TH,ToF
		rrf	DS18B20_TL,ToF
		return

;***** Расстояние *****

#include "Units\HC-SR04.inc"

;===== ПРОЦЕДУРЫ РАБОТЫ С GSM-МОДУЛЕМ ===================================================

;Очистка буфера приёма USART
U1BufClear:	clrf	U1BufWrAddr		;Инициализация адреса записи
		clrf	U1BufRdAddr		;Инициализация адреса чтения
		return

;Загрузка в FSR адреса чтения из буфера USART
U1RdAddrToFSR:	bcf	STATUS,IRP		;9-й бит адреса (IRP:FSR) = 0, т.к. буфер в банке 1
		movlw	U1Buf			;Расчёт адреса чтения в FSR
		addwf	U1BufRdAddr,ToW
		movwf	FSR
		return

;Инкремент FSR с зацикливанием по буферу приёма USART
;Результат: C=1 - достигнут конец буфера (символов нет); Z=1 (при C=0) - достигнут символ конца строки (EOL)
U1IncFSR_CZ:	incf	FSR,ToF			;Инкремент FSR с зацикливанием по буферу
		cmp_lwf	U1Buf+U1BufSize,FSR
		movlw	U1Buf
		If_2GE1
		  movwf	FSR
		movlw	U1Buf			;Расчёт адреса записи в W
		addwf	U1BufWrAddr,ToW
		xorwf	FSR,ToW			;Совпадает с адресом чтения => Установка бита C
		clrc
		If_Z
		  setc
		;Продолжение ниже

;Определение, что в INDF символ конца строки 0x0A или 0xOD (если да, то Z=1)
;Бит C не портит
U1FSRCheckEOL:	xor_lfw	0x0A,INDF		;Первый символ в буфере 0x0A => Выход с Z=1
		If_Z
		  return
		xor_lfw	0x0D,INDF		;Сравнение с 0x0D, результат в Z
		return

;Инкремент U1BufRdAddr с зацикливанием по буферу приёма USART (удаление одного символа из буфера)
U1IncRdAddr:	incf	U1BufRdAddr,ToF		;Инкремент U1BufRdAddr с зацикливанием по буферу
		cmp_lwf	U1BufSize,U1BufRdAddr
		If_2GE1
		  clrf	U1BufRdAddr
		return

;Заполение U1BufRdAddr по текущему значению FSR (удаление считанной строки из буфера)
U1FSRToRdAddr:	mov_fw	FSR			;U1BufRdAddr:=FSR-U1Buf
		addlw	-U1Buf
		movwf	U1BufRdAddr
		return

#include "Units\SIM900D.inc"

;===== КОМАНДЫ РАБОТЫ С EEPROM ==========================================================

;Запись W в EEADR
WriteWToEEADR:	Bank_0To1
		movwf	EEADR			;Сохранение W в EEADR
		Bank_1To0
		return

;Чтение байта с адресом из W в W с инкрементом EEADR
EE_RdW_PI:	call	WriteWToEEADR		;Сохранение W в EEADR
		;Продолжение ниже

;Чтение байта с адресом из W в W с инкрементом EEADR
EE_RdEEADR_PI:	Bank_0To1
		bsf	EECON1,RD		;Чтение
		mov_fw	EEDATA			;Значение в W
		incf	EEADR,ToF		;Инкремент адреса EEADR
		Bank_1To0
		return

;Запись байта из W по адресу EEADR с инкременом EEADR
;Запрещает прерывания на 7 мкс
EE_WrEEADR_PI:	Bank_0To1
		movwf	EEDATA			;Сохраняем данные
		bcf	INTCON,GIE		;Запрет прерываний
		bsf	EECON1,WREN		;Разрешение записи в EEPROM
		mov_lwf	0x55,EECON2		;Инициируется запись
		mov_lwf	0xAA,EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE		;Разрешение прерываний
		If_1	EECON1,WR		;Ожидание завершения записи
		  goto	$-1
		bcf	EECON1,WREN		;Запрет записи в EEPROM
		incf	EEADR,ToF		;Инкремент адреса EEADR
		Bank_1To0
		return

;Чтение глубины в Glubina
EE_ReadGlubina:	movlw	LOW (_EE_Glubina)	;Адрес значения в EEPROM
		call	EE_RdW_PI		;Чтение значения старшего байта
		movwf	GlubinaH		;Сохранение
		call	EE_RdEEADR_PI		;Чтение значения младшего байта
		movwf	GlubinaL		;Сохранение
		return

;Запись глубины из Glubina
EE_SaveGlubina:	movlw	LOW (_EE_Glubina)	;Адрес значения в EEPROM в W
		call	WriteWToEEADR		;Запись W в EEADR
		mov_fw	GlubinaH		;Значение старшего байта
		call	EE_WrEEADR_PI		;Запись значения старшего байта
		mov_fw	GlubinaL		;Значение младшего байта
		goto	EE_WrEEADR_PI		;Запись значения младшего байта и выход (return там)

;Чтение номера телефона в PhoneNr
;Портит Tmp1
EE_ReadPhoneNr:	mov_lwf	PhoneNr,FSR		;Адрес записи номера
		mov_lwf	.15,Tmp1		;Максимальное число байт номера, не считая 0
		movlw	LOW (_EE_PhoneNr)	;Адрес значения в EEPROM
		call	EE_RdW_PI		;Чтение значения первого байта в W
_EERPN_Next:	movwf	INDF			;Добавление к номеру
		test_f	INDF
		If_Z
		  return
		incf	FSR,ToF			;Переход к следующему байту
		call	EE_RdEEADR_PI		;Чтение значения очередного байта в W
		loop_f	Tmp1,_EERPN_Next	;Зацикливание (W не портит)
		clrf	INDF			;Защита: если сбой EEPROM, чтобы номер не был бесконечным
		return

;Запись номера телефона из PhoneNr
EE_SavePhoneNr:	mov_lwf	PhoneNr,FSR		;Адрес чтения номера
		mov_lwf	.15,Tmp1		;Максимальное число байт номера, не считая 0
		movlw	LOW (_EE_PhoneNr)	;Адрес значения в EEPROM в W
		call	WriteWToEEADR		;Запись W в EEADR
_EEWPN_Next:	mov_fw	INDF			;Очередной символ номера в W
		If_Z					;Нулевой => Запись нуля и выход
		  goto	_EEWPN_Wr0
		call	EE_WrEEADR_PI		;Запись байта из W
		incf	FSR,ToF			;Переход к следующему байту
		loop_f	Tmp1,_EEWPN_Next	;Зацикливание
_EEWPN_Wr0:	clrw				;Запись нуля на конце номера и выход
		goto	EE_WrEEADR_PI			;(return там)

;===== ОБРАБОТКА КОМАНД ЭВМ =============================================================

#include "Units\PC_Commands.inc"

;===== ОСНОВНАЯ ЧАСТЬ ПРОГРАММЫ =========================================================

Main:		;Переход после запуска (включён Bank_0)
		mov_lwf	0x07,CMCON		;Компаратор выключен (минимальный ток), RA0, RA1, RA2 - цифровые
		clrf	PORTA			;Начальные состояния портов
		mov_lwf	0x14,PORTB			;TX=1, U2_TX=1
		Bank_0To1
		mov_lwf	0xBF,OPTION_REG		;Выключение подтяг. рез. PORTB; задний фронт INT; предделитель WDT 1:128
		mov_lwf	0x27,TRISA		;Настройка выходов (после инициализации!)
		mov_lwf	0xEB,TRISB			;TX, U2_TX - выходы
		Bank_1To0
		SerialSetup			;Инициалиация обмена по USART
		Serial2Setup			;Инициалиация обмена по USART2

		;Настройка TMR2 и прерываний от него (включен Bank_0)
		clrf	TMR2			;Начальное значение TMR2
		Bank_0To1
		mov_lwf	.137,PR2		;Период 138 тактов
		bsf	PIE1,TMR2IE		;Разрешение прерываний от TMR2
		Bank_1To0
		mov_lwf	0x04,T2CON		;Настройка TMR2: включён, 1:1, 1:1
		bcf	PIR1,TMR2IF		;Сброс флага прерывания
		bsf	INTCON,PEIE		;Глобальное разрешение прерываний
		bsf	INTCON,GIE
		mov_lwf	.1,Count20ms		;Сброс отсчёта 20 мс
		clrf	Count5120ms		;Сброс отсчёта 5120 мс
		call	U1BufClear		;Очистка буфера приёма USART
		clrf	DecEach5120ms_NoSvet	;Отправка смс "Света нет" разрешена

		;Завершение инициализации (включен Bank_0)
		call	Log_Init		;Инициализация работы с логом
		call	SIM_Init		;Инициализация процедур SIM900D
		call	Pause1s			;Пауза 1 секунда для стабилизации питания
		call	GetTemperature		;Холостое измерение температуры (заполняет DS18B20_Present)
		call	GetDistance		;Холостое измерение расстояния
SIM900Restart:	movlw	0x1E			;Добавление кода события в лог
		call	Log_AddW
_SIM900R_Again:	Debug_Restart			;Отладочный режим => Сообщение о перезапуске модуля SIM900D
		call	SIM_OnOffByKey		;Инициализация SIM900D: эмуляция включения кнопкой
		call	Pause1s				;Пауза 3 cекунды на запуск подуля (min 2.2, беру 3 с запасом)
		call	Pause1s
		call	Pause1s
		call	SIM_InitObmen			;Инициализация обмена по USART с SIM900D
		If_NZ					;Ошибка => Всё с начала
		  goto	_SIM900R_Again
		call	SIM_SetTextSMS			;Перевод SMS в текстовый режим
		If_NZ					;Ошибка => Всё с начала
		  goto	_SIM900R_Again
		call	SIM_SetCharGSM			;Выбор кодировки GSM
		If_NZ					;Ошибка => Всё с начала
		  goto	_SIM900R_Again
		call	SIM_DelAllSMS			;Удаление всех SMS на SIM-карте
		If_NZ					;Ошибка => Всё с начала
		  goto	_SIM900R_Again
		call	SIM_SleepToOn			;Разрешение спящего режима SIM900D
		If_NZ					;Ошибка => Всё с начала
		  goto	_SIM900R_Again
		movlw	0x10			;Добавление кода события в лог
		call	Log_AddW

;===== ОСНОВНОЙ ЦИКЛ РАБОТЫ =============================================================

;Работает в Bank_0

Wait_ResetATOK:	clrf	IncEach5120ms_ATOK	;Сброс таймера проверки связи с SIM900D

Wait:		clrwdt				;Сброс WDT (нормальная работа)
		call	SIM_RecMessage		;Обработка сообщений модуля SIM900D
		If_1	SIMNeedDelSMS		;Требуется удалить смс после "ОК" => Обработка (сразу после SIM_RecMessage!)
		  call	DeleteAllSMS
		If_1	NeedSendT		;Требуется отправить температуру => Обработка
		  call	SendSMS_T
		If_1	NeedSendR		;Требуется отправить расстояние => Обработка
		  call	SendSMS_R
		If_1	NeedSendG		;Требуется отправить глубину => Обработка
		  call	SendSMS_G
		If_1	NeedSendOR		;Требуется отправить обратное расстояние => Обработка
		  call	SendSMS_OR
		If_1	NeedSendAll		;Требуется отправить параметры при задании номера => Обработка
		  call	SendSMS_All
		If_1	INTCON,INTF		;Требуется отправить сообщение "Света нет" => Обработка
		  call	SendSMS_NoSvet
		test_f	SIMNeedRecSMSNr		;Требуется загрузить смс => Загрузка (после всех отправок!)
		If_NZ
		  call	SIM_ReceiveSMS
		If_1	U2_IsNewRcChar		;Есть новая команда ПК => Обработка
		  call	U2_RecPCCmd
		;Проверка связи с модулем SIM900D
		cmp_lwf	ATOK_Period,IncEach5120ms_ATOK ;Время проверки не наступило => Ожидание
		If_2Lt1
		  goto	Wait
		Debug_TestATOK			;Отладочный режим => Сообщение о проведении теста
		call	SIM_TestObmen		;Проверка связи
		If_Z				;Проверка пройдена => Перезапуск отсчёта до следующей
		  goto	Wait_ResetATOK
		movlw	0xF0			;Проверка не пройдена: Добавление кода события в лог
		call	Log_AddW
		goto	SIM900Restart		;Перезапуск модуля

;===== ОБРАБОТКА КОМАНД =================================================================

;***** Отправка значений *****

;Выдача температуры (Т = *** grad) либо сообщения об ошибке (ERROR T)
SendTValue:	If_0	DS18B20_Present		;Датчика нет => Сообщение "ERROR "
		  call	_SendError_
		call	U1_SendT		;Символ "T"
		If_0	DS18B20_Present		;Датчика нет => выход
		  return
		call	_Send_Equal_		;" = "
		clrf	SIMNrH			;Подготовка значения: Всегда однобайтовое
		movlw	'-'				;Значение отрицательно => Знак "-"
		If_1	DS18B20_TL,7
		  SerialTransmit
		mov_fw	DS18B20_TL			;Значение отрицательно => Инвертируем, иначе без изменений
		If_1	DS18B20_TL,7
		  negw
		movwf	SIMNrL
		call	U1TransmitNr10		;Отправка значения
		call	_SendSpace		;" grad"
		SerialTransmitChar 'g'
		SerialTransmitChar 'r'
		SerialTransmitChar 'a'
		SerialTransmitChar 'd'
		return

;Выдача расстояния (R = *** sm) либо сообщения об ошибке (ERROR R)
SendRValue:	ior_ffw	Dist_sm			;Dist_sm = 0x0000 => Ошибка
		If_NZ
		  If_1	Dist_smH,7		;Dist_sm = 0xFFFF => Ошибка
		  goto	_SRV_Error
		call	U1_SendR		;Символ "R"
		call	_Send_Equal_		;" = "
		mov2fwf	Dist_sm,SIMNr		;Подготовка значения
_SendSIMNrSm:	call	U1TransmitNr10		;Отправка значения
_SendSm:	;Служебная: выдача " sm"
		call	_SendSpace		;" sm"
		SerialTransmitChar 's'
		SerialTransmitChar 'm'
		return
_SRV_Error:	call	_SendError_		;Сообщение "ERROR R"
		goto	U1_SendR		;Отправка "R" и выход (return там)
_SendError_:	;Служебная: выдача текста "ERROR "
		SerialTransmitChar 'E'
		call	U1_SendR
		call	U1_SendR
		SerialTransmitChar 'O'
		call	U1_SendR
		goto	_SendSpace
_Send_Equal_:	;Служебная: выдача " = "
		call	_SendSpace
		call	U1_SendEqual
_SendSpace:	;Служебная: выдача пробела
		SerialTransmitChar ' '
		return

;Выдача обратного расстояния (ObratnoeR = *** sm) либо сообщения об ошибке (ERROR R)
SendORValue:	ior_ffw	Dist_sm			;Dist_sm = 0x0000 => Ошибка
		If_NZ
		  If_1	Dist_smH,7		;Dist_sm = 0xFFFF => Ошибка
		  goto	_SRV_Error
		SerialTransmitChar 'O'		;Выдача "ObratnoeR"
		SerialTransmitChar 'b'
		SerialTransmitChar 'r'
		SerialTransmitChar 'a'
		SerialTransmitChar 't'
		SerialTransmitChar 'n'
		SerialTransmitChar 'o'
		SerialTransmitChar 'e'
		call	U1_SendR
		call	_Send_Equal_		;" = "
		sb2fwff	Dist_sm,Glubina,SIMNr	;Подготовка значения: SIMNr:=Glubina-Dist_sm
		If_0	SIMNrH,7		;Значение положительное => Выдаём
		  goto	_SendSIMNrSm
		SerialTransmitChar '-'		;Знак "минус"
		neg2f	SIMNr			;Инвертируем значение
		goto	_SendSIMNrSm		;Выдача и выход (return там)

;Выдача глубины (Glubina = *** sm)
SendGValue:	call	U1_SendG		;Текст сообщения: Выдача "Glubina"
		SerialTransmitChar 'l'
		SerialTransmitChar 'u'
		SerialTransmitChar 'b'
		SerialTransmitChar 'i'
		SerialTransmitChar 'n'
		SerialTransmitChar 'a'
		call	_Send_Equal_			;Выдача " = "
		mov2fwf	Glubina,SIMNr			;Подготовка значения
		call	U1TransmitNr10			;Отправка значения
		goto	_SendSm				;Выдача " sm" и выход (return там)

;Выдача сообщения "Света нет"
SendSvetaNet:	SerialTransmitChar 'S'
		SerialTransmitChar 'v'
		SerialTransmitChar 'e'
		SerialTransmitChar 't'
		SerialTransmitChar 'a'
		call	_SendSpace
		SerialTransmitChar 'n'
		SerialTransmitChar 'e'
		SerialTransmitChar 't'
		return

;***** Отправка смс *****

;Отправка параметров по флагу NeedSendAll
SendSMS_All:	bcf	NeedSendAll		;Флаг обработан (сначала, т.к. может появиться новый)
		movlw	0x22			;Добавление в лог кода события о приёме смс "SetThisNumber"
		call	Log_AddW
		call	GetTemperature		;Измерение температуры (заполняет DS18B20_Present)
		call	GetDistance		;Измерение расстояния
		call	EE_ReadGlubina		;Чтение значения глубины
		call	SIM_StartSMS		;Начало отправки смс
		call	SendTValue		;Выдача температуры либо сообщения об ошибке
		call	_SendCommaSp		;Разделитель
		call	SendRValue		;Выдача расстояния либо сообщения об ошибке
		call	_SendCommaSp		;Разделитель
		call	SendGValue		;Выдача глубины
		call	_SendCommaSp		;Разделитель
		call	SendORValue		;Выдача обратного расстояния либо сообщения об ошибке
		If_1	In_Trevoga		;Сигнала тревоги нет => Завершение
		  goto	_SendAll_End
		call	_SendCommaSp		;Разделитель
		call	SendSvetaNet		;Выдача сообщения "Света нет"
_SendAll_End:	movlw	0x42			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)
_SendCommaSp:	;Служебная: выдача ", "
		SerialTransmitChar ','
		goto	_SendSpace

;Отправка температуры по флагу NeedSendT
SendSMS_T:	bcf	NeedSendT		;Флаг обработан (сначала, т.к. может появиться новый)
		movlw	0x24			;Добавление в лог кода события о приёме смс "T"
		call	Log_AddW
		call	GetTemperature		;Измерение температуры (заполняет DS18B20_Present)
		call	SIM_StartSMS		;Начало отправки смс
		call	SendTValue		;Выдача температуры либо сообщения об ошибке
		movlw	0x44			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)

;Отправка расстояния по флагу NeedSendR
SendSMS_R:	bcf	NeedSendR		;Флаг обработан (сначала, т.к. может появиться новый)
		movlw	0x26			;Добавление в лог кода события о приёме смс "R"
		call	Log_AddW
		call	GetDistance		;Измерение расстояния
		call	SIM_StartSMS		;Начало отправки смс
		call	SendRValue		;Выдача расстояния либо сообщения об ошибке
		movlw	0x46			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)

;Отправка обратного расстояния по флагу NeedSendOR
SendSMS_OR:	bcf	NeedSendOR		;Флаг обработан (сначала, т.к. может появиться новый)
		movlw	0x28			;Добавление в лог кода события о приёме смс "ObratnoeR"
		call	Log_AddW
		call	GetDistance		;Измерение расстояния
		call	EE_ReadGlubina		;Чтение значения глубины
		call	SIM_StartSMS		;Начало отправки смс
		call	SendORValue		;Выдача обратного расстояния либо сообщения об ошибке
		movlw	0x48			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)

;Отправка глубины по флагу NeedSendG
SendSMS_G:	bcf	NeedSendG		;Флаг обработан (сначала, т.к. может появиться новый)
		movlw	0x2A			;Добавление в лог кода события о приёме смс "Glubina ..."
		call	Log_AddW
		call	EE_ReadGlubina		;Чтение значения глубины
		call	SIM_StartSMS		;Начало отправки смс
		call	SendGValue		;Выдача глубины
		movlw	0x4A			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)

;Отправка сообщения "Света нет"
SendSMS_NoSvet:	bcf	INTCON,INTF		;Событие обработано
		test_f	DecEach5120ms_NoSvet	;Отправка смс ещё запрещена => Событие игнорируется
		If_NZ
		  return
		mov_lwf	SvetaNet_SMSPeriod,DecEach5120ms_NoSvet ;Запрет отправки смс на заданное время
		call	SIM_StartSMS		;Начало отправки смс
		call	SendSvetaNet		;Выдача сообщения "Света нет"
		movlw	0x4C			;Конец отправки смс: Код события для добавления в лог
		goto	SIM_EndSMS			;Запись в лог, конец отправки и выход (return там)

;***** Удаление смс на SIM-карте *****

;Удаление всех смс по флагу SIMNeedDelSMS
DeleteAllSMS:	bcf	SIMNeedDelSMS		;Флаг обработан (сначала, т.к. может появиться новый)
		call	SIMWaitOK		;Ожидание "ОК" после передачи текста смс (портит DecEach20ms)
		goto	SIM_DelAllSMS		;Удаление всех SMS на SIM-карте (Z=1 - успешно) и выход (return там)

;===== ПРОВЕРКА =========================================================================

		CBLOCK				;Переменные в банке 0
		  VariablesEndAddr1: 0		;Адрес конца банка 0. Не должен превышать 0x070
		ENDC

		IF VariablesEndAddr1>0x70
		  error "VariablesEndAddr1 превышает 0x70"
		ENDIF

;===== EEPROM ===========================================================================

		ORG	0x2100		;EEPROM. Использовать DW, а не DB!

_EE_PhoneNr:	DW	'+'		;Адрес 0 (16 байт): Номер телефона для отправки (на конце 0x00)
		DW	'7'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	'x'
		DW	0x00
		DW	0x00
		DW	0x00
		DW	0x00

_EE_Glubina:	DW	0x01		;Адрес 16 (2 байта): Глубина (по умолчанию 400=0x0190)
		DW	0x90

_EE_LogRdAddr:	DW	LOW (_EE_Log)	;Адрес 18: Указатель чтения из лога
_EE_LogWrAddr:	DW	LOW (_EE_Log)	;Адрес 19: Указатель записи в лог (строго после _EE_LogRdAddr!)

_EE_Log:	DW	0x00		;Адрес 20: Лог до конца EEPROM (128-20=108 байт)

;========================================================================================

		END
