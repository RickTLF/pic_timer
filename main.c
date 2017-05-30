/*
 * File:   main.c
 * Author: Rick
 *
 * Created on May 19, 2017, 10:46 AM
 */


#pragma config RETEN = ON       // VREG Sleep Enable bit (Enabled)
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = SWDTDIS  // Watchdog Timer (WDT enabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3L
#pragma config RTCOSC = SOSCREF // RTCC Clock Select (RTCC uses SOSC)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 Mux (RC1)
#pragma config ECCPMX = PORTE   // ECCP Mux (Enhanced CCP1/3 [P1B/P1C/P3B/P3C] muxed with RE6/RE5/RE4/RE3)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RG5 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)
#pragma config CP4 = OFF        // Code Protect 10000-13FFF (Disabled)
#pragma config CP5 = OFF        // Code Protect 14000-17FFF (Disabled)
#pragma config CP6 = OFF        // Code Protect 18000-1BFFF (Disabled)
#pragma config CP7 = OFF        // Code Protect 1C000-1FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)
#pragma config WRT4 = OFF       // Table Write Protect 10000-13FFF (Disabled)
#pragma config WRT5 = OFF       // Table Write Protect 14000-17FFF (Disabled)
#pragma config WRT6 = OFF       // Table Write Protect 18000-1BFFF (Disabled)
#pragma config WRT7 = OFF       // Table Write Protect 1C000-1FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBRT0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBRT1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBRT2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBRT3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)
#pragma config EBRT4 = OFF      // Table Read Protect 10000-13FFF (Disabled)
#pragma config EBRT5 = OFF      // Table Read Protect 14000-17FFF (Disabled)
#pragma config EBRT6 = OFF      // Table Read Protect 18000-1BFFF (Disabled)
#pragma config EBRT7 = OFF      // Table Read Protect 1C000-1FFFF (Disabled)

// CONFIG7H
#pragma config EBRTB = OFF      // Table Read Protect Boot (Disabled)

#include <xc.h>
#include <pic18f87k90.h>

#define _XTAL_FREQ 8000000
//#define TMR2PRESCALE 4
//
//=================================================================
void led_init(void);
void timer0_init(void);
void enable_timer0(void);
void stop_timer0(void);
void configure_timer0_prescale_value(unsigned char prescaleValue);
void enable_timer0_interrupt(void);
void clear_timer0_interrupt(void);
//=================================================================

void main(void) {
    led_init();
    configure_timer0_prescale_value(0b000);
    timer0_init();
    enable_timer0_interrupt();
    enable_timer0();
    
    while (1) {
        // everything handled by ISR
    }
    return;
}

void interrupt ISR(void) {
    
    // interrupt fired by timer 0?
    if (INTCONbits.TMR0IF == 1) {
        // toggle led
        PORTAbits.RA0 ^= 1;
    }
    // reset overflow interrupt flag
    INTCONbits.TMR0IF = 0;
}

//=================================================================

void led_init(void) {
    // set a0 pin to output
    TRISAbits.TRISA0 = 0;
}

//=================================================================

void timer0_init(void) {
    
    // clear timer 0 high and low holding registers
    TMR0H = TMR0L = 0;
    
    // configure timer as 8-bit timer/counter
    T0CONbits.T08BIT = 1;   
    
    // use internal clock (Fosc/4)
    T0CONbits.T0CS = 0;
    
    // timer 0 prescaler is assigned; 
    // timer value comes from prescaler output
    T0CONbits.PSA = 0;
}
//
void enable_timer0(void) {
    // enable timer 0
    T0CONbits.TMR0ON = 1;
}

void stop_timer0(void) {
    // enable timer 0
    T0CONbits.TMR0ON = 0;
}

void configure_timer0_prescale_value(unsigned char prescaleValue) {
    T0CONbits.T0PS = prescaleValue;
}

void enable_timer0_interrupt(void) {
    // enables global interrupts
    INTCONbits.GIE = 1;
    // enables interrupt
    INTCONbits.TMR0IE = 1;
}

void clear_timer0_interrupt(void) {
    // clears interrupt overflow flag
    INTCONbits.TMR0IF = 0;
}

//=================================================================