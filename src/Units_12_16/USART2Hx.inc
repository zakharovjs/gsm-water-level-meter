;=============================================================================
;Дополнительные процедуры для общения через USART2 в символьном (Hex) виде
;При передаче используются только заглавные буквы 'A'..'F'!
;Требует подключения модулей "AddInstr.inc" и "USART2.inc"
;Все процедуры требуют включенности Bank_0 и возвращаются в Bank_0
;=============================================================================

;===== КОНСТАНТЫ И ПЕРЕМЕННЫЕ ================================================

;Переменные в Bank_0
		CBLOCK
		  HexByte2:	1	;Накопитель для двух байтов
		ENDC

;===== ПРОЦЕДУРЫ И ФУНКЦИИ ===================================================

;Приём одной ASCII HEX-цифры и её преобр. в байт в HexByte2 и W. Портит RcChar
Serial2HexReceiveDigit		MACRO
				clrf	HexByte2
				call	S2_HexRecDigt_
				ENDM

;Приём двух ASCII HEX-цифр и их преобр. в байт в HexByte2 и W. Портит RcChar
#define	Serial2HexReceive	call	S2_HexRec_

;Аналогично - приём с помещением результата в Port
Serial2HexReceivePort		MACRO	Port
				Serial2HexReceive
				movwf	Port
				ENDM

;Посылка одной ASCII HEX-цифры из младшей тетрады W. HexByte2 не меняет, W портит
#define	Serial2HexTransmitDigit	call	S2_HexTrDigit_

;Посылка содержимого W в виде двух ASCII Hex-цифр. Сохраняет W в HexByte2; W портит
#define	Serial2HexTransmit	call	S2_HexTrans_

;Аналогично - посылка числа Byte
Serial2HexTransmitChar		MACRO	Char
				movlw	Char
				Serial2HexTransmit
				ENDM

;Аналогично - посылка содержимого Port
Serial2HexTransmitPort		MACRO	Port
				mov_fw	Port
				Serial2HexTransmit
				ENDM

;===== РЕАЛИЗАЦИЯ ============================================================

S2_HexRec_:	clrf	HexByte2		;Подготовка результата
		call	S2_HexRecDigt_		;Приём и сохранение первого байта
		swapf	HexByte2,ToF		;Перевод его в старшие разряды
S2_HexRecDigt_:	Serial2Receive			;Приём очередного (1/2-го) байта
		If_1	RcChar,6		;Преобразование цифры в число: 'A'-'F'?
		  addlw	0xF9				;Да => Убираем разрыв (-7)
		addlw	0xD0			;Приведение к нулю
		iorwf	HexByte2,ToF		;Прибавление к результату
		mov_fw	HexByte2		;Помещение результата в W
		return

S2_HexTrans_:	movwf	HexByte2		;Сохранение байта в HexByte2
		call	S2H_a1_			;Подготовка и посылка старших четырёх разрядов
S2H_a1_:	swapf	HexByte2,ToF		;Очередные четыре разряда в младшие
		mov_fw	HexByte2		;Выделение очередных четырёх разрядов
S2_HexTrDigit_:	andlw	0x0F			;Очистка старшей тетрады
		addlw	0xF6			;Преобразование в цифру: вычитание 10 (0x0a)
		If_C				;Результат положителен?
		  addlw	0x07				;Нет => Прибавить 65 (для 'A' to 'F')
		addlw	0x3A				;Да => Прибавить 58 (для '0' to '9') 
		Serial2Transmit			;Посылка очередной цифры
		return

;=============================================================================
