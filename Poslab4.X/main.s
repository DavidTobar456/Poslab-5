; Archivo: main.s
; Autor: David Antonio Tobar López

; Programa: Segundero con display, y contador con Push buttons
; Hardware: PIC16F887, 2x Display de 7 seg., 2 p-buttons, 4x R470 Ohms
; Compilador: PIC-AS (v2.32)
    
; Fecha de creación:
; Última fecha de modificación

; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

PROCESSOR   16F887
#include <xc.inc>

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

res_TMR0    macro
    BANKSEL PORTA
    movlw   100
    movwf   TMR0
    bcf	    T0IF
endm
 
PSECT udata_shr
    W_TEMP:	    DS 1
    STATUS_TEMP:    DS 1
    
PSECT udata_bank0
    cont_TMR0:	    DS 1
    cont_Unidades:  DS 1
    cont_Decenas:   DS 1
    comp_TMR0:	    DS 1
    comp_Un:	    DS 1
    comp_Dec:	    DS 1

PSECT resVet, class=CODE, delta=2, abs
    ORG 00h
    resetVec:
	PAGESEL main
	goto	main

PSECT intVec, class=code, delta=2, abs
    ORG 04h
    PUSH:
	movwf	W_TEMP
	swapf	STATUS,w
	movwf	STATUS_TEMP
    ISR:
	btfsc	RBIF
	call	int_iocb
	btfsc	T0IF
	call	int_TMR0
    POP:
	swapf	STATUS_TEMP,w
	movwf	STATUS
	swapf	W_TEMP,f
	swapf	W_TEMP,w
	retfie

PSECT CODE, delta=2, abs
    ORG 100h
    tabla:
	clrf	PCLATH
	bsf	PCLATH, 0
	andlw	0x0F;
	addwf	PCL;
	retlw	00111111B; cero
	retlw	00000110B; uno
	retlw	01011011B; dos
	retlw	01001111B; tres
	retlw	01100110B; cuatro
	retlw	01101101B; cinco
	retlw	01111101B; seis
	retlw	00000111B; siete
	retlw	01111111B; ocho
	retlw	01100111B; nueve
	
    ;---------------Configuración----------------
    main:
	call	config_io
	call	config_osc
	call	config_banderas
	call	config_iocrb
	call	config_TMR0
    
    ;-------------------LOOP---------------------
    loop:
	call	ver_seg
	call	ver_decena
	call	mostrar_Display
	goto loop
	
    ;----------Sub-rutinas de Config-------------
    config_io:
	BANKSEL ANSEL
	clrf	ANSEL
	clrf	ANSELH;	Se configuran los I/0 como digitales
	
	BANKSEL TRISA
	clrf	TRISA
	clrf	TRISC
	clrf	TRISD;	Se configuran los pines RA, RC, RE como salidas
	bsf	TRISB,0
	bsf	TRISB,1;Se configuran RB0 y RB1 como entradas
	bcf	OPTION_REG,7;	Se habilitan las resistencias pull-ups
	bsf	WPUB,0
	bsf	WPUB,1;	Se configuran las pull-up de RB 0 y 1.
	
	BANKSEL	PORTA
	clrf	PORTA
	clrf	PORTC
	clrf	PORTD;	Se limpian los valores de PORTA, PORTC, y PORTD
    return
    
    config_osc:
	BANKSEL OSCCON
	bsf	IRCF2
	bcf	IRCF1
	bcf	IRCF0;	Se configura el reloj a 1MHz
	bsf	SCS;	Se elige el reloj interno del PIC
    return
    
    config_banderas:
	bsf	GIE;	Se habilita el Global Interruption Enable
	bsf	RBIE;	Se habilita la interrupción por cambio en PORTB
	bsf	T0IE;	Se habilita la interrupción por overflow de TMR0
	bcf	RBIF;	Se borra el valor en RBIF
	bcf	T0IF;	Se borra el valor en T0IF
    return
    
    config_iocrb:
	BANKSEL TRISA
	bsf	IOCB,0
	bsf	IOCB,1;	Si estos pines cambian su valor se interrumpe
	
	BANKSEL PORTA
	movf	PORTB,w
	bcf	RBIF
    return
    
    config_TMR0:
	BANKSEL	TRISA
	bcf	T0CS;	Se utiliza el reloj interno como reloj
	bcf	PSA;	El pre escalador se asigna a TIMER0
	bsf	PS2
	bcf	PS1
	bcf	PS0;	Se asignó  el prescaler de 32 al timer
	res_TMR0
    return
    
    ;---------Sub-rutinas generales--------------
    ver_seg:
	movf	cont_TMR0,w
	addlw	14
	movwf	comp_TMR0
	btfsc	comp_TMR0, 6
	call	inc_seg
    return
    
    inc_seg:
	clrf	cont_TMR0
	incf	cont_Unidades
    return
    
    ver_decena:
	movf	cont_Unidades,w
	addlw	6
	movwf	comp_Un
	btfsc	comp_Un,4
	call	inc_decena
    return
    
    inc_decena:
	clrf	cont_Unidades
	incf	cont_Decenas
	movf	cont_Decenas,w
	addlw	2
	movwf	comp_Dec
	btfsc	comp_Dec,3
	clrf	cont_Decenas
    return
    
    mostrar_Display:
	movf	cont_Unidades,w
	call	tabla
	movwf	PORTC
	movf	cont_Decenas,w
	call	tabla
	movwf	PORTA
    return
    
    ;--------Sub-rutinas interrupción------------
    int_iocb:
	btfss	PORTB,0
	incf	PORTD
	btfsc	PORTD,4
	clrf	PORTD
	btfss	PORTB,1
	decf	PORTD
	btfsc	PORTD,7
	call	res_bits_PORTD
	bcf RBIF;   Se apaga la bandera de RBIF
    return
    
    res_bits_PORTD:
	clrf	PORTD
	bsf	PORTD,0
	bsf	PORTD,1
	bsf	PORTD,2
	bsf	PORTD,3; Se setean solamente los primeros 4 bits
    return
    
    int_TMR0:
	incf	cont_TMR0
	res_TMR0
    return
END


