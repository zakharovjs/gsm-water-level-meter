;=============================================================================
;Макросы - эмуляция дополнительных инструкций
;=============================================================================

;***** Биты сохранения результата *****

#define		ToW		0			;Результат сохранить в W
#define		ToF		1			;Результат сохранить в F

;***** Команды для W *****

#define		incw		addlw	0x01
#define		decw		addlw	0xFF
#define		comw		xorlw	0xFF
#define		negw		sublw	0x00

;***** Команды для флагов и битов *****

#define		clrz		bcf	STATUS,Z
#define		setz		bsf	STATUS,Z
#define		clrc		bcf	STATUS,C
#define		setc		bsf	STATUS,C
#define		clrb		bsf	STATUS,C
#define		setb		bcf	STATUS,C

;Выполнение следующей команды по значению бита
#define		If_1		btfsc			;Выполнение следующей команды, если бит = 1
#define		If_FB		btfsc
#define		If_0		btfss			;Выполнение следующей команды, если бит = 0
#define		If_NFB		btfss
#define		If_Z		btfsc	STATUS,Z	;Выполнение следующей команды, если бит Z = 1
#define		If_NZ		btfss	STATUS,Z	;Выполнение следующей команды, если бит Z = 0
#define		If_C		btfsc	STATUS,C	;Выполнение следующей команды, если бит C = 1
#define		If_NC		btfss	STATUS,C	;Выполнение следующей команды, если бит C = 0
#define		If_B		btfss	STATUS,C	;Выполнение следующей команды, если бит B = 1
#define		If_NB		btfsc	STATUS,C	;Выполнение следующей команды, если бит B = 0

;Выполнение следующей команды по результату команды cmp (любой: 2 порта или константа и порт)
#define		If_2Lt1		If_B			;Выполнить, если аргумент 2 < аргумента 1
#define		If_2GE1		If_NB			;Выполнить, если аргумент 2 >= аргумента 1

;Прибавление бита Z к W (флаги не определены)
adzw		MACRO
		If_Z
		  incw
		ENDM

;Прибавление бита C к W (бит C корректируется по сложению, остальные флаги не определены)
adcw		MACRO
		If_C
		  incw
		ENDM

;Прибавление бита C к произвольному порту (бит C корректируется по сложению, остальные флаги не определены)
adcf		MACRO	Port
		If_C
		  incf	Port,ToF
		ENDM

;Копирование бита C в требуемый бит произвольного порта
movcfb		MACRO	Port,Bit
		If_C
		  bsf	Port,Bit
		If_NC
		  bcf	Port,Bit
		ENDM

;Копирование требуемого бита произвольного порта в бит C
movfbc		MACRO	Port,Bit
		setc
		If_0	Port,Bit
		  clrc
		ENDM

;Копирование требуемого бита произвольного порта 1 в требуемый бит произвольного порта 2
movfbfb		MACRO	Port1,Bit1,Port2,Bit2
		If_1	Port1,Bit1
		  bsf	Port2,Bit2
		If_0	Port1,Bit1
		  bcf	Port2,Bit2
		ENDM

;Копирование требуемого бита произвольного порта 1 в требуемый бит произвольного порта 2 с инверсией
mvnfbfb		MACRO	Port1,Bit1,Port2,Bit2
		If_1	Port1,Bit1
		  bcf	Port2,Bit2
		If_0	Port1,Bit1
		  bsf	Port2,Bit2
		ENDM

;Формирование положительного импульса на бите
strobe1		MACRO	Port,Bit
		bsf	Port,Bit
		bcf	Port,Bit
		ENDM

;Формирование отрицательного импульса на бите
strobe0		MACRO	Port,Bit
		bcf	Port,Bit
		bsf	Port,Bit
		ENDM

;***** Команды для однобайтовых переменных *****

;Проверка ячейки на ноль. Z=1 - равна нулю
test_f		MACRO	Port
		movf	Port,ToF
		ENDM

;Обмен значениями порта с портом через W (4 такта, бит C не портит)
chg_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToW
		xorwf	Port1,ToF
		xorwf	Port2,ToF
		ENDM

;Перенос порта 1 в порт 2 через W со сдвигом результата влево через C
rl_fwf		MACRO	Port1,Port2
		rlf	Port1,ToW
		movwf	Port2
		ENDM

;Перенос порта в порт через W со сдвигом результата вправо через C
rr_fwf		MACRO	Port1,Port2
		rrf	Port1,ToW
		movwf	Port2
		ENDM

;MOV

;Перенос порта в W
mov_fw		MACRO	Port
		movf	Port,ToW
		ENDM

;Запись константы в порт через W (2 такта)
mov_lwf		MACRO	LL,Port
		movlw	LL
		movwf	Port
		ENDM

;Перенос из порта в порт через W (2 такта)
mov_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		movwf	Port2
		ENDM

;AND

;Логическое И числа с портом через W
and_lwf		MACRO	LL,Port
		movlw	LL
		andwf	Port,ToF
		ENDM

;Логическое И порта с портом через W
and_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		andwf	Port2,ToF
		ENDM

;Логическое И порта с портом в W
and_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		andwf	Port2,ToW
		ENDM

;IOR

;Логическое ИЛИ числа с портом через W
ior_lwf		MACRO	LL,Port
		movlw	LL
		iorwf	Port,ToF
		ENDM

;Логическое ИЛИ порта с портом через W
ior_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		iorwf	Port2,ToF
		ENDM

;Логическое ИЛИ порта с портом в W
ior_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		iorwf	Port2,ToW
		ENDM

;XOR

;Исключающее ИЛИ числа с портом через W
xor_lwf		MACRO	LL,Port
		movlw	LL
		xorwf	Port,ToF
		ENDM

;Исключающее ИЛИ числа с портом в W
xor_lfw		MACRO	LL,Port
		movlw	LL
		xorwf	Port,ToW
		ENDM

;Исключающее ИЛИ порта с портом через W
xor_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToF
		ENDM

;Исключающее ИЛИ порта с портом в W
xor_ffw		MACRO	Port1,Port2
		movf	Port1,ToW
		xorwf	Port2,ToW
		ENDM

;ADD

;Прибавление числа к порту через W
add_lwf		MACRO	LL,Port
		movlw	LL
		addwf	Port,ToF
		ENDM

;Прибавление порта к порту через W
add_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		addwf	Port2,ToF
		ENDM

;SUB

;Вычитание числа из порта через W
sub_lwf		MACRO	LL,Port
		movlw	LL
		subwf	Port,ToF
		ENDM

;Вычитание порта 1 из порта 2 через W
sub_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		subwf	Port2,ToF
		ENDM

;CMP

;Сравнение константы с портом через W (2 такта)
;Результат: Z=1 - равны; B=1 - Port<LL; B=0 - Port>=LL
cmp_lwf		MACRO	LL,Port
		movlw	LL
		subwf	Port,ToW
		ENDM

;Сравнение порта с портом через W (2 такта)
;Результат: Z=1 - равны; B=1 - Port2<Port1; B=0 - Port2>=Port1
cmp_fwf		MACRO	Port1,Port2
		movf	Port1,ToW
		subwf	Port2,ToW
		ENDM

;***** Команды для двухбайтовых переменных *****

;Очистка двухбайтового порта (2 такта)
clr2f		MACRO	PortH,PortL
		clrf	PortL
		clrf	PortH
		ENDM

;Инверсия двухбайтового порта (2 такта)
com2f		MACRO	PortH,PortL
		comf	PortL,ToF
		comf	PortH,ToF
		ENDM

;Инкремент двухбайтового порта (3 такта)
inc2f		MACRO	PortH,PortL
		incf	PortL,ToF
		If_Z
		  incf	PortH,ToF
		ENDM

;Декремент двухбайтового порта (4 такта)
dec2f		MACRO	PortH,PortL
		test_f	PortL
		If_Z
		  decf	PortH,ToF
		decf	PortL,ToF
		ENDM

;Смена знака двухбайтового порта (5 тактов)
neg2f		MACRO	PortH,PortL
		comf	PortL,ToF
		comf	PortH,ToF
		incf	PortL,ToF
		If_Z
		  incf	PortH,ToF
		ENDM

;Обмен значениями двух двухбайтовых портов через W (8 тактов, бит C не портит)
chg2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		chg_fwf	Port1H,Port2H
		chg_fwf	Port1L,Port2L
		ENDM

;RLF

;Сдвиг двухбайтового порта влево через перенос (2 такта)
rl2f		MACRO	PortH,PortL
		rlf	PortL,ToF
		rlf	PortH,ToF
		ENDM

;Перенос двухбайтового порта 1 в порт 2 через W со сдвигом результата влево через C
rl2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		rl_fwf	Port1L,Port2L
		rl_fwf	Port1H,Port2H
		ENDM

;RRF

;Сдвиг двухбайтового порта вправо через перенос (2 такта)
rr2f		MACRO	PortH,PortL
		rrf	PortH,ToF
		rrf	PortL,ToF
		ENDM

;Перенос двухбайтового порта 1 в порт 2 через W со сдвигом результата вправо через C
rr2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		rr_fwf	Port1H,Port2H
		rr_fwf	Port1L,Port2L
		ENDM

;MOV

;Запись константы в двухбайтовый порт через W (4 такта)
movlw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		movwf	PortL
		movlw	HIGH LL
		movwf	PortH
		ENDM

;Перенос из порта в порт через W (4 такта)
mov2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		movwf	Port2L
		movf	Port1H,ToW
		movwf	Port2H
		ENDM

;ADD

;Прибавление W к двухбайтовому порту (3 такта, W не портит)
addwf2		MACRO	PortH,PortL
		addwf	PortL,ToF
		If_C
		  incf	PortH,ToF
		ENDM

;Прибавление константы к двухбайтовому порту через W (6 тактов)
addlw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		addwf	PortL,ToF
		movlw	HIGH LL
		If_C
		  incw
		addwf	PortH,ToF
		ENDM

;Прибавление константы к двухбайтовому порту 1 через W в порт 2 (8 тактов)
ad2lwff		MACRO	LL,Port1H,Port1L,Port2H,Port2L
		movlw	LOW LL
		addwf	Port1L,ToW
		movwf	Port2L
		movlw	HIGH LL
		If_C
		  incw
		addwf	Port1H,ToW
		movwf	Port2H
		ENDM

;Прибавление двухбайтового порта 1 к порту 2 через W (6 тактов)
add2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		addwf	Port2L,ToF
		movf	Port1H,ToW
		If_C
		  incw
		addwf	Port2H,ToF
		ENDM

;SUB

;Вычитание W из двухбайтового порта (3 такта, W не портит)
subwf2		MACRO	PortH,PortL
		subwf	PortL,ToF
		If_B
		  decf	PortH,ToF
		ENDM

;Вычитание константы из двухбайтового порта через W (6 тактов)
;Результат: B=1 - Port<LL; B=0 - Port>=LL
sublw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		subwf	PortL,ToF
		movlw	HIGH LL
		If_B
		  incw
		subwf	PortH,ToF
		ENDM

;Вычитание двухбайтового порта 1 из порта 2 через W (6 тактов)
;Результат: B=1 - Port2<Port1; B=0 - Port2>=Port1
sub2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		subwf	Port2L,ToF
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToF
		ENDM

;Вычитание двухбайтового порта 1 из порта 2 через W в порт 3 (6 тактов)
;Результат: B=1 - Port2<Port1; B=0 - Port2>=Port1
sb2fwff		MACRO	Port1H,Port1L,Port2H,Port2L,Port3H,Port3L
		movf	Port1L,ToW
		subwf	Port2L,ToW
		movwf	Port3L
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToW
		movwf	Port3H
		ENDM

;CMP

;Сравнение константы с двухбайтовым портом через W (6 тактов)
;Результат: B=1 - Port<LL; B=0 - Port>=LL
cmplw2f		MACRO	LL,PortH,PortL
		movlw	LOW LL
		subwf	PortL,ToW
		movlw	HIGH LL
		If_B
		  incw
		subwf	PortH,ToW
		ENDM

;Сравнение двухбайтовых порта с портом через W (6 тактов)
;Результат: B=1 - Port2<Port1; B=0 - Port2>=Port1
cmp2fwf		MACRO	Port1H,Port1L,Port2H,Port2L
		movf	Port1L,ToW
		subwf	Port2L,ToW
		movf	Port1H,ToW
		If_B
		  incw
		subwf	Port2H,ToW
		ENDM

;***** Организация циклов и задержек *****

;Организация цикла
loop_f		MACRO	Port,Label
		decfsz	Port,ToF
		  goto	Label
		ENDM

;Холостой цикл (ожидание), пока бит не сбросится в 0
WaitUntil0	MACRO	Port,Bit
		If_1	Port,Bit
		  goto	$-1
		ENDM

;Холостой цикл (ожидание), пока бит не установится в 1
WaitUntil1	MACRO	Port,Bit
		If_0	Port,Bit
		  goto	$-1
		ENDM

;Холостой цикл (ожидание), пока порт не сбросится в 0
WaitFUntil0 	MACRO	Port
		test_f	Port
		If_NZ
		  goto	$-2
		ENDM

;Инструкция nop длиной 1 слово и длительностью 2 цикла
#define		nop2	goto	$+1

;***** Команды сравнения (все портят W) *****

;Сравнение W с числом или символом
If_wEl		MACRO	LL
		xorlw	LL
		If_Z
		ENDM
	;Равно:	...
	;Нет:	...

If_wNEl		MACRO	LL
		xorlw	LL
		If_NZ
		ENDM
	;Нет:	...
	;Равно:	...

;Сравнение W с портом
If_wEf		MACRO	Port
		xorwf	Port,W
		If_Z
		ENDM
	;Равно:	...
	;Нет:	...

If_wNEf		MACRO	Port
		xorwf	Port,W
		If_NZ
		ENDM
	;Нет:	...
	;Равно:	...

;Сравнение порта с числом
If_fEl		MACRO	Port,LL
		mov_fw	Port
		If_wEl	LL
		ENDM
	;Равно:	...
	;Нет:	...

If_fNEl		MACRO	Port,LL
		mov_fw	Port
		If_wNEl	LL
		ENDM
	;Нет:	...
	;Равно:	...

;Сравнение порта с портом
If_fEf		MACRO	Port1,Port2
		mov_fw	Port1
		If_wEf	Port2
		ENDM
	;Равно:	...
	;Нет:	...

If_fNEf		MACRO	Port1,Port2
		mov_fw	Port1
		If_wNEf	Port2
		ENDM
	;Нет:	...
	;Равно:	...

;***** Организация табличных переходов *****

;Табличный переход по адресу таблицы и номеру пункта в W (номер от нуля - 0, 1, 2, 3, ..., максимум 255).
;Таблица переходов с метки TableLabel - последовательность goto, call, return или retlw - инструкций перехода на пункт.
;Длительность 8 тактов. W портит.
TableJumpW	MACRO	TableLabel
		xorlw	HIGH (TableLabel)	;Занесение HIGH (TableLabel) в PCLATH с сохранением W
		movwf	PCLATH
		xorlw	HIGH (TableLabel)
		xorwf	PCLATH,ToF
		addlw	LOW (TableLabel)	;Вычисляем адрес
		If_C
		  incf	PCLATH,ToF
		movwf	PCL			;Переход
		ENDM

;=============================================================================
