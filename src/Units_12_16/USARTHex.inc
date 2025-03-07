;=============================================================================
;Дополнительные процедуры для общения через USART в символьном (Hex) виде
;При передаче используются только заглавные буквы 'A'..'F'!
;Требует подключения модулей "AddInstr.inc" и "USART.inc"
;Все процедуры требуют включенности Bank_0 и возвращаются в Bank_0
;=============================================================================

;===== КОНСТАНТЫ И ПЕРЕМЕННЫЕ ================================================

;Переменные в Bank0
		CBLOCK
		  HexByte:	1	;Накопитель для двух байтов
		ENDC

;===== ПРОЦЕДУРЫ И ФУНКЦИИ ===================================================

;Приём одной ASCII HEX-цифры и её преобр. в байт в HexByte и W. Портит RcChar
SerialHexReceiveDigit		MACRO
				clrf	HexByte
				call	S_HexRecDigit_
				ENDM

;Приём двух ASCII HEX-цифр и их преобр. в байт в HexByte и W. Портит RcChar
#define	SerialHexReceive	call	S_HexRec_

;Аналогично - приём с помещением результата в Port
SerialHexReceivePort		MACRO	Port
				SerialHexReceive
				movwf	Port
				ENDM

;Посылка одной ASCII HEX-цифры из младшей тетрады W. HexByte не меняет, W портит
#define	SerialHexTransmitDigit	call	S_HexTrDigit_

;Посылка содержимого W в виде двух ASCII Hex-цифр. Сохраняет W в HexByte; W портит
#define	SerialHexTransmit	call	S_HexTrans_

;Аналогично - посылка числа Byte
SerialHexTransmitChar		MACRO	Char
				movlw	Char
				SerialHexTransmit
				ENDM

;Аналогично - посылка содержимого Port
SerialHexTransmitPort		MACRO	Port
				mov_fw	Port
				SerialHexTransmit
				ENDM

;===== РЕАЛИЗАЦИЯ ============================================================

S_HexRec_:	clrf	HexByte			;Подготовка результата
		call	S_HexRecDigit_		;Приём и сохранение первого байта
		swapf	HexByte,F		;Перевод его в старшие разряды
S_HexRecDigit_:	SerialReceive			;Приём очередного (1/2-го) байта
		If_1	RcChar,6		;Преобразование цифры в число: 'A'-'F'?
		  addlw	0xF9				;Да => Убираем разрыв (-7)
		addlw	0xD0			;Приведение к нулю
		iorwf	HexByte,F		;Прибавление к результату
		mov_fw	HexByte			;Помещение результата в W
		return

S_HexTrans_:	movwf	HexByte			;Сохранение байта в HexByte
		call	SH_a1_			;Подготовка и посылка старших четырёх разрядов
SH_a1_:		swapf	HexByte,1		;Очередные четыре разряда в младшие
		mov_fw	HexByte			;Выделение очередных четырёх разрядов
S_HexTrDigit_:	andlw	0x0F			;Очистка старшей тетрады
		addlw	0xF6			;Преобразование в цифру: вычитание 10 (0x0a)
		If_C				;Результат положителен?
		  addlw	0x07				;Нет => Прибавить 65 (для 'A' to 'F')
		addlw	0x3A				;Да => Прибавить 58 (для '0' to '9') 
		SerialTransmit			;Посылка очередной цифры
		return

;=============================================================================
