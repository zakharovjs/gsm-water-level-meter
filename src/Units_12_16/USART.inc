;=============================================================================
;Процедуры для общения через COM-порт (USART)
;Требует подключения модулей "AddInstr.inc" и "BankSel.inc"
;Все процедуры требуют включенности Bank_0 (SerialSetup допускает Bank_1)
;Все процедуры возвращаются в Bank_0
;=============================================================================

;АППАРАТНОЕ ПОДКЛЮЧЕНИЕ:
;Стандартный интерфейс на выводах TX и RX.

;ПРОГРАММНОЕ ПОДКЛЮЧЕНИЕ:
;1. До подключения модуля в программе определить константу скорости:
;	#define		BAUD_CONSTANT	0x...
;   Возможные значения для Fosc = 4 МГц:
;		0x67			Скорость 2400 бод (реально 2404)
;		0x33			Скорость 4800 бод (реально 4808)
;		0x19			Скорость 9600 бод (реально 9615)
;		0x0C			Скорость 19200 бод (реально 19231)
;   Возможные значения для Fosc = 20 МГц:
;		0x81			Скорость 9600 бод (реально 9615)
;		0x40			Скорость 19200 бод (реально 19231)
;		0x14			Скорость 57600 бод (реально 59524)
;		0x0A			Скорость 115200 бод (реально 113636)
;2. Настроить RX и TX как цифровые выводы (не выводы АЦП и прочей периферии),
;   RX как вход, TX как выход.
;3. В начале работы вызвать процедуру SerialSetup.

;===== КОНСТАНТЫ И ПЕРЕМЕННЫЕ ================================================

;Переменные в Bank_0
		CBLOCK
		  RcChar:	1	;Символ, принятый SerialReceive
		ENDC

;===== ПРОЦЕДУРЫ И ФУНКЦИИ ===================================================

;***** ИНИЦИАЛИЗАЦИЯ *****

;Инициализация USART. Вызывается один раз до использования USART. Портит W
SerialSetup			MACRO
				call	S_Setup_
				ENDM

;***** ПЕРЕДАЧА *****

;Проверка незаполненности буфера передачи (т.е. возможности передачи данных)
If_SerialCanTransmit		MACRO
				If_1	PIR1,TXIF
				ENDM
			;Можно:	...
			;Занят:	...

;Проверка заполненности буфера передачи (т.е. невозможности передачи данных)
If_SerialCannotTransmit		MACRO
				If_0	PIR1,TXIF
				ENDM
			;Занят:	...
			;Можно:	...

;Передача байта из W
SerialTransmit			MACRO
				call	S_Transmit_
				ENDM

;Передача байта Char. Байт помещается в W
SerialTransmitChar		MACRO	Char
				movlw	Char
				SerialTransmit
				ENDM

;Передача содержимого порта Port. Байт помещается в W
SerialTransmitPort		MACRO	Port
				mov_fw	Port
				SerialTransmit
				ENDM

;Передача байта из W без проверки готовности. 
;Использовать, только если есть уверенность в готовности буфера передачи
SerialSmartTransmit		MACRO
				movwf	TXREG
				ENDM

;***** ПРИЁМ *****

;Проверка заполненности буфера приёма (т.е. необходимости считывания данных)
If_SerialReceiveNeed		MACRO
				If_1	PIR1,RCIF
				ENDM
			;Полон:	...
			;Пуст:	...

;Проверка незаполненности буфера приёма (т.е. необходимости ожидания данных)
If_SerialReceiveNotNeed		MACRO
				If_0	PIR1,RCIF
				ENDM
			;Пуст:	...
			;Полон:	...

;Ожидание готовности данных и приём байта в W и RcChar
SerialReceive			MACRO
				call	S_Receive_
				ENDM

;Ожидание готовности данных и приём байта в W, RcChar и Port
SerialReceivePort		MACRO	Port
				SerialReceive
				movwf	Port
				ENDM

;Приём байта без ожидания готовности в W и RcChar. Если не было, то 0
;Нельзя использовать там, где предусмотрена передача нулевых байтов
SerialSmartReceive		MACRO
				call	S_SmartRec_
				ENDM

;Приём байта без ожидания готовности в W. Вызывать при наличии в буфере
SerialHardReceiveToW		MACRO
				mov_fw	RCREG
				ENDM

;Проверка RcChar на совпадение с символом
If_RcCharIs			MACRO	Char
				If_fEl	RcChar,Char
				ENDM
			;Char:	...
			;Нет:	...

;Проверка RcChar на несовпадение с символом
If_RcCharIsNot			MACRO	Char
				If_fNEl	RcChar,Char
				ENDM
			;Нет:	...
			;Char:	...

;===== РЕАЛИЗАЦИЯ ============================================================

S_Setup_:	Bank_0To1
		mov_lwf	BAUD_CONSTANT,SPBRG	;Задание скорости обмена
		mov_lwf	B'00100100',TXSTA	;TX9=0; TXEN=1; SYNC=0[асинхр.]; BRGH=1[быстрая]
		Bank_1To0
		mov_lwf	B'10010000',RCSTA	;SPEN=1; RX9=0; CREN=1
		bcf	PIR1,RCIF
		return

S_Transmit_:	If_SerialCannotTransmit		;Буфер полон?
		  goto	$-1			;Да => Ожидание
		SerialSmartTransmit		;Передача байта
		return

S_Receive_:	If_SerialReceiveNotNeed
		  goto	$-1
		mov_fwf	RCREG,RcChar		;Приём байта в W, запись в RcChar
		return

S_SmartRec_:	clrw
		If_SerialReceiveNeed
		  mov_fwf RCREG,RcChar		;Приём байта в W, запись в RcChar
		return

;=============================================================================
