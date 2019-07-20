/*
 * Serial LED Controller
 * Copyright (c) 2019 David Rice
 * 
 * Controls a string of WS281x LEDs with a UART interface to control various effects.
 * 
 * Supports up to 300 LEDs due to memory limitation of PIC16F18325.
 * 
 * Supports WS2811 in fast mode only.
 * 
 * Processor: PIC16F18325
 * 
 * Peripheral usage:
 * EUSART1 - Receives serial data
 * MSSP1 - Drives WS281x LED strip (SPI master mode) via CLC1
 * Timer1 - Generates system tick for processing LED strip updates
 * Timer2 - PWM clock source for WS281x protocol
 * PWM5 - Generates PWM signal for WS281x protocol
 * CLC1 - Combines MSSP1 and PWM5 signals to produce WS2812x protocol
 * 
 * Pin assignments:
 * RA5 - WS281x data output from CLC1
 * RC5 - EUSART1 data input
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/**** FREQUENTLY CHANGED CONFIGURATION VALUES START HERE ****/

/* Type of LEDs connected to driver - supported options are WS2811 and WS2812B */
#define WS2811

/* Maximum LED index connected to this driver (i.e., number of LEDs - 1) */
#define MAX_LED_INDEX 49

/* Initial modes */
#define INIT_MODE MODE_RND_ONE
#define INIT_LEVEL LEVEL_MID

/* Overall maximum values for various brightness levels
 * 
 * For strings or bright strips, usual values are 0xFF, 0x7F, 0x3F 
 * For small rings, usual values are 0x7F, 0x1F, 0x0F 
 */
#define MAX_BRIGHT 0xFF
#define MID_BRIGHT 0x7F
#define DIM_BRIGHT 0x3F

/* Maximums for secondary colors used in fire and ice effects 
 *
 * For strings or bright strips, usual values are 0x7F, 0x3F, 0x1F 
 * For small rings, usual values are 0x3F, 0x0F, 0x08 
 */
#define MAX_BRIGHT_SEC 0x7F
#define MID_BRIGHT_SEC 0x3F
#define DIM_BRIGHT_SEC 0x1F

/**** FREQUENTLY CHANGED CONFIGURATION VALUES END HERE ****/

/* Frequency must be specified for delay loops */
#define _XTAL_FREQ 32000000

/* 
 * Baud Rate Generator value for 9600 baud at Fosc = 32MHz
 * 9600 baud: ((32000000 / 9600) / 64) - 1
 */
#define BAUD_RATE 51 

/* Define output pin for WS281x data stream */
#define LED_DATA_OUT LATAbits.LATA5

/* Define TRIS register for UART data */
#define UART_RX_TRIS TRISCbits.TRISC5

/* Define EEPROM addresses for random seed */
#define SEED_LOW_ADDR 0x7000
#define SEED_HIGH_ADDR 0x7001

/* 
 * Macros for putting color values at the appropriate index
 * Note that the WS2812B uses GRB format instead of RGB
 */
#ifdef WS2811
#define RED(x)   (x * 3)
#define GREEN(x) (x * 3) + 1
#define BLUE(x)  (x * 3) + 2
#else
#ifdef WS2812B
#define RED(x)   (x * 3) + 1
#define GREEN(x) (x * 3)
#define BLUE(x)  (x * 3) + 2
#else
#error LED controller type must be specified
#endif
#endif

/* Define available display modes */
typedef enum {
    MODE_STATIC = 0,
    MODE_INC,
    MODE_DEC,
    MODE_RND_ALL,
    MODE_RND_ONE,
    MODE_FIRE,
    MODE_ICE,
    MODE_SPARKS,
    MODE_XMAS,
    MODE_TNET
} DISP_MODE;

/* Define brightness levels */
typedef enum {
    LEVEL_DIM = 0,
    LEVEL_MID,
    LEVEL_BRIGHT
} DISP_LEVEL;

uint8_t color_data[(MAX_LED_INDEX + 1) * 3];

uint8_t brightness_val;
uint8_t secondary_val;

/* Data received from via UART */
volatile uint8_t serial_data;

/* Flag to indicate if UART data was received in ISR */
volatile bool serial_pending = false;

/* Index of LED currently being updated by MSSP1 and CLC1 */
volatile uint16_t update_index = 0;

/* Flag to indicate if LED strip needs to be updated */
volatile bool update_needed = false;

/* ISR functions:
 * 
 * 1. Transmit LED data via MSSP1 and CLC1.
 * 2. Start update cycle if needed based on timer overflow.
 * 3. Process received data from EUSART1. 
 */
void __interrupt() isr(void) {
    /* Peripheral interrupts */
    if (INTCONbits.PEIE) {
        /* MSSP1 interrupt - fires when MSSP1 has finished sending SPI data */
        if (PIE1bits.SSP1IE && PIR1bits.SSP1IF) {
            PIR1bits.SSP1IF = 0;
            
            if (update_index <= ((MAX_LED_INDEX + 1) * 3)) {       
                SSP1BUF = color_data[update_index]; /* Transmit one byte - interrupt will fire again as soon as this byte is transmitted */
                update_index++;
            } else {               
                update_index = 0; /* If all data has been transmitted, set update_index to 0 to indicate that update is not running */
            }  
        }
        
        /* Timer1 interrupt - fires approximately 60 times per second */
        if (PIE1bits.TMR1IE && PIR1bits.TMR1IF) {
            PIR1bits.TMR1IF = 0;
            
            /* Only start update cycle if update is needed and update is not already in progress */
            if (update_needed && update_index == 0) {
                SSP1BUF = color_data[update_index]; /* Transmit first byte - subsequent bytes will be handled by MSSP1 interrupt */
                update_index++;
                update_needed = false;
            }
        }
        
        /* EUSART1 interrupt - fires when serial data has been received */
        if (PIE1bits.RCIE && PIR1bits.RCIF) {
            serial_data = RC1REG;
            serial_pending = true; /* Set flag so data can be processed in main loop */
        }
    }
}

/* 
 * Standard port initialization
 * Later functions will change some of these settings 
 */
void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Set all ports to output */
    TRISA = 0x00;
    TRISC = 0x00;
    
    /* Pull all outputs low */
    LATA = 0x00;
    LATC = 0x00;
    
    /* Set TTL on RC5 due to 3.3V output from Bluteooth module */
    INLVLCbits.INLVLC5 = 0;
}

/* Initialize EUSART1 hardware registers */
void init_uart(void) {
    UART_RX_TRIS = 1; /* Set RX pin as input */
    
    RC1STAbits.CREN = 1; /* Continuous receive */
    TX1STAbits.SYNC = 0; /* Asynchronous */
    SP1BRG = BAUD_RATE; /* Baud rate as defined above */
    RC1STAbits.SPEN = 1; /* Enable serial port */
}

/* Initialize SPI module that will drive the CLC */
void init_spi(void) {
    /* SPI used only to drive CLC so no outputs defined */
    
    SSP1CON1bits.SSPM = 0b0011; /* Set SPI mode with CLK = T2_match/2 */
    SSP1CON1bits.SSPEN = 1; /* Enable MSSP */
}

/* Initialize Timer1 to drive display updates and Timer2 to be PWM clock source */
void init_timers(void) {
    T1CONbits.TMR1CS = 0b00; /* Timer1 source is Fosc/4 (instruction clock) */
    T1CONbits.T1CKPS = 0b10; /* Timer1 prescaler 1/4 = 2 MHz at 32 Mhz Fosc */
    T1CONbits.TMR1ON = 1; /* Enable Timer1 */
    
    PR2 = 4; /* 0.625 uSec at Fosc = 32 MHz */
    T2CONbits.TMR2ON = 1; /* Enable Timer2 */
}

/* Initialize PWM generator for WS281x zero-bit pulses */
void init_pwm(void) {
    PWM5DCH = 1;
    PWM5DCL = 0;
    
    PWM5CONbits.PWM5EN = 1; /* Enable PWM generator */
}

/* Set up CLC to output (SCK && SDO) || (nSDO && SCK && PWM) */
void init_clc(void) {
    RA5PPS = 0b00100; /* CLC1OUT on RA5 */
    
    CLC1SEL0bits.LC1D1S = 0b10011; /* CLC1 input 1 is SDO1 */
    CLC1SEL1bits.LC1D2S = 0b10010; /* CLC1 input 2 is SCK1 */
    CLC1SEL2bits.LC1D3S = 0b10000; /* CLC1 input 3 is PWM5OUT */
    
    CLC1GLS0 = 0x00;
    CLC1GLS1 = 0x00;
    CLC1GLS2 = 0x00;
    CLC1GLS3 = 0x00; /* Gate behavior is undefined at power-on so must be set to zero */
    
    CLC1GLS0bits.LC1G1D1T = 1; /* SDO input to AND gate 1 */
    
    CLC1GLS1bits.LC1G2D2T = 1; /* SCK input to AND gate 1 */
    
    /* nSDO && SCK = n(SDO || nSCK) */
    CLC1GLS2bits.LC1G3D1T = 1;
    CLC1GLS2bits.LC1G3D2N = 1; /* SDO || nSCK input to AND gate 2 */
    
    CLC1GLS3bits.LC1G4D3T = 1; /* PWM5OUT input to AND gate 2 */
    
    CLC1POL = 0x00; /* Clear all inversion bits */
    CLC1POLbits.LC1G3POL = 1; /* Gate 3 n(SDO || nSCK) is inverted to obtain (nSDO && SDK) */
    
    CLC1CONbits.LC1EN = 1; /* Enable CLC1 */
}

/* Sets brightness variables based on selected brightness value */
void set_brightness(DISP_LEVEL brt) {
    switch (brt) {
        case LEVEL_DIM:
            brightness_val = DIM_BRIGHT;
            secondary_val = DIM_BRIGHT_SEC;
            break;
        case LEVEL_MID:
            brightness_val = MID_BRIGHT;
            secondary_val = MID_BRIGHT_SEC;
            break;
        case LEVEL_BRIGHT:
            brightness_val = MAX_BRIGHT;
            secondary_val = MAX_BRIGHT_SEC;
            break;
        default:
            break;
    }
}

/* Sets all LEDs to a given RGB value */
void set_all(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t current_led;
    
    for (current_led = 0; current_led <= MAX_LED_INDEX; current_led++) {
        color_data[RED(current_led)] = red;
        color_data[GREEN(current_led)] = green;
        color_data[BLUE(current_led)] = blue;
    }
}

/* Sets all LEDs to random RGB values with maximums specified in parameters */
void random_all(uint8_t max_red, uint8_t max_green, uint8_t max_blue) {
    uint16_t current_led;
    
    for (current_led = 0; current_led <= MAX_LED_INDEX; current_led++) {
        color_data[RED(current_led)] = (max_red == 0) ? 0 : (rand() % max_red);
        color_data[GREEN(current_led)] = (max_green == 0) ? 0 : (rand() % max_green);
        color_data[BLUE(current_led)] = (max_blue == 0) ? 0 : (rand() % max_blue);
    }
}

/* Sets one random LED to a random RGB value with maximums specified in parameters */
void random_one(uint8_t max_red, uint8_t max_green, uint8_t max_blue) {
    uint16_t current_led;
    
    current_led = rand() % (MAX_LED_INDEX + 1);
    
    color_data[RED(current_led)] = (max_red == 0) ? 0 : (rand() % max_red);
    color_data[GREEN(current_led)] = (max_green == 0) ? 0 : (rand() % max_green);
    color_data[BLUE(current_led)] = (max_blue == 0) ? 0 : (rand() % max_blue);
}

/* Sets one random LED to either red or green */
void random_one_xmas(uint8_t brightness) {
    uint16_t current_led;
    
    current_led = rand() % (MAX_LED_INDEX + 1);
    
    if ((rand() % 100) > 50) {
        color_data[RED(current_led)] = brightness;
        color_data[GREEN(current_led)] = 0;
        color_data[BLUE(current_led)] = 0;
    } else {
        color_data[RED(current_led)] = 0;
        color_data[GREEN(current_led)] = brightness;
        color_data[BLUE(current_led)] = 0;
    }
}

/* Sets one random LED to either white, green, or blue */
void random_one_tnet(uint8_t brightness) {
    uint16_t current_led;
    uint8_t rand_val;
    
    current_led = rand() % (MAX_LED_INDEX + 1);
    rand_val = rand() % 100;
    
    if (rand_val > 66) {
        color_data[RED(current_led)] = brightness;
        color_data[GREEN(current_led)] = brightness;
        color_data[BLUE(current_led)] = brightness;
    } else if (rand_val > 33) {
        color_data[RED(current_led)] = 0;
        color_data[GREEN(current_led)] = brightness;
        color_data[BLUE(current_led)] = 0;
    } else {
        color_data[RED(current_led)] = 0;
        color_data[GREEN(current_led)] = 0;
        color_data[BLUE(current_led)] = brightness;
    }
}

/* Fire effect */
void random_fire(uint8_t red, uint8_t max_green) {
    uint16_t current_led;
    
    current_led = rand() % (MAX_LED_INDEX + 1);
    
    color_data[RED(current_led)] = red;
    color_data[GREEN(current_led)] = (max_green == 0) ? 0 : (rand() % max_green);
    color_data[BLUE(current_led)] = 0;
}

/* Ice effect */
void random_ice(uint8_t max_red_green, uint8_t blue) {
    uint16_t current_led;
    uint8_t red_green;
    
    current_led = rand() % (MAX_LED_INDEX + 1);
    red_green = (max_red_green == 0) ? 0 : (rand() % max_red_green);
    
    color_data[RED(current_led)] = red_green;
    color_data[GREEN(current_led)] = red_green;
    color_data[BLUE(current_led)] = blue;
}

/* Retrieves a random number generator seed from EEPROM and stores an updated seed for next power-on */
void seed_random(void) {
    uint16_t seed;
    
    NVMCON1bits.NVMREGS = 1; /* Set NV module to access EEPROM */ 
    
    NVMADR = SEED_LOW_ADDR;
    NVMCON1bits.RD = 1;
    seed = NVMDATL; /* Read low byte of seed from EEPROM */
    
    NVMADR = SEED_HIGH_ADDR;
    NVMCON1bits.RD = 1;
    seed = seed | (NVMDATL << 8); /* Read high byte of seed from EEPROM */
    
    srand(seed); /* Seed random number generator with stored seed */
    
    seed = rand();
    
    NVMCON1bits.NVMREGS = 1;
    NVMCON1bits.WREN = 1; /* Enable EEPROM writes */
    
    NVMADR = SEED_LOW_ADDR;
    NVMDATL = (uint8_t)seed;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1; /* Write low byte of seed to EEPROM */
    
    NVMADR = SEED_HIGH_ADDR;
    NVMDATL = (uint8_t)(seed >> 8);
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1; /* Write high byte of seed to EEPROM */
    
    NVMCON1bits.WREN = 0; /* Disable EEPROM writes */
}

void main(void) {
    DISP_MODE mode = INIT_MODE;

    uint8_t red;
    uint8_t green;
    uint8_t blue;
    
    uint16_t active_led = 0;
    
    seed_random(); /* Seed the random number generator using stored value */
    
    init_ports(); /* Initialize I/O ports */
    init_spi(); /* Initialize MSSP1 for SPI */
    init_timers(); /* Initialize Timer1 for display updates and Timer2 to drive PWM generator */
    init_pwm(); /* Initlialze PWM5 generator */
    init_clc(); /* Initialize CLC1 to drive WS281x LEDs */
    
    set_brightness(INIT_LEVEL);
    
    mode = MODE_RND_ONE;
    random_all(brightness_val, brightness_val, brightness_val); /* Set all LEDs to random value */
    update_needed = true; /* Update display */
    
    init_uart(); /* Initialize EUSART1 hardware */
    
    PIE1bits.RCIE = 1; /* Enable EUSART1 receive interrupt */
    PIE1bits.SSP1IE = 1; /* Enable MSSP1 interrupt */
    
    TMR1 = 0; /* Reset Timer1 before enabling interrupt */
    PIR1bits.TMR1IF = 0; /* Clear Timer1 interrupt flag before enabling interrupt */
    PIE1bits.TMR1IE = 1; /* Enable Timer1 overflow interrupt */
    
    INTCONbits.PEIE = 1; /* Enable peripheral interrupts */
    INTCONbits.GIE = 1; /* Enable global interrupts */
    
    while(1) {
        if (serial_pending) {
            switch (serial_data) {
                case 'R':
                    set_all(brightness_val, 0x00, 0x00);
                    mode = MODE_STATIC;
                    update_needed = true;
                    break;
                case 'G':
                    set_all(0x00, brightness_val, 0x00);
                    mode = MODE_STATIC;
                    update_needed = true;
                    break;
                case 'B':
                    set_all(0x00, 0x00, brightness_val);
                    update_needed = true;
                    mode = MODE_STATIC;
                    break;
                case '0':
                    set_all(0x00, 0x00, 0x00);
                    update_needed = true;
                    mode = MODE_STATIC;
                    break;
                case 'X':
                    mode = MODE_INC;
                    red = rand() % brightness_val;
                    green = rand() % brightness_val;
                    blue = rand() % brightness_val;
                    break;
                case 'Y':
                    mode = MODE_DEC;
                    red = rand() % brightness_val;
                    green = rand() % brightness_val;
                    blue = rand() % brightness_val;
                    break;
                case '1':
                    mode = MODE_RND_ONE;
                    random_all(brightness_val, brightness_val, brightness_val);
                    update_needed = true;
                    break;
                case 'A':
                    mode = MODE_RND_ALL;
                    break;
                case 'S':
                    mode = MODE_SPARKS;
                    break;
                case 'F':
                    mode = MODE_FIRE;
                    set_all(brightness_val, secondary_val, 0x00);
                    update_needed = true;
                    break;
                case 'I':
                    mode = MODE_ICE;
                    set_all(secondary_val, secondary_val, brightness_val);
                    update_needed = true;
                    break;
                case 'C':
                    mode = MODE_XMAS;
                    set_all(0, brightness_val, 0);
                    update_needed = true;
                    break;
                case 'T':
                    mode = MODE_TNET;
                    set_all(brightness_val, brightness_val, brightness_val);
                    update_needed = true;
                    break;
                case 'L':
                    mode = MODE_STATIC;
                    red = rand() % brightness_val;
                    green = rand() % brightness_val;
                    blue = rand() % brightness_val;
                    set_all(red, green, blue);
                    update_needed = true;
                    break;
                case '7':
                    set_brightness(LEVEL_DIM);
                    break;
                case '8':
                    set_brightness(LEVEL_MID);
                    break;
                case '9':
                    set_brightness(LEVEL_BRIGHT);
                    break;
            }
            serial_pending = false;
        }

       switch (mode) {
            case MODE_INC:
                set_all(0x00, 0x00, 0x00);
                color_data[RED(active_led)] = red;
                color_data[GREEN(active_led)] = green;
                color_data[BLUE(active_led)] = blue;

                active_led++;

                if (active_led > MAX_LED_INDEX) {
                    active_led = 0;
                }

                update_needed = true;
                __delay_ms(50);
                break;
            case MODE_DEC:
                set_all(0x00, 0x00, 0x00);
                color_data[RED(active_led)] = red;
                color_data[GREEN(active_led)] = green;
                color_data[BLUE(active_led)] = blue;

                active_led--;
                
                if (active_led > MAX_LED_INDEX) {
                    active_led = MAX_LED_INDEX;
                }

                update_needed = true;
                __delay_ms(50);
                break;
           case MODE_RND_ALL:
               random_all(brightness_val, brightness_val, brightness_val);
               update_needed = true;
               
               __delay_ms(250);
               break;
           case MODE_RND_ONE:
               random_one(brightness_val, brightness_val, brightness_val);
               update_needed = true;
               
               __delay_ms(50);
               break;         
           case MODE_FIRE:
               random_fire(brightness_val, secondary_val);
               update_needed = true;
               
               __delay_ms(50);
               break;
            case MODE_ICE:
               random_ice(brightness_val, secondary_val);
               update_needed = true;
               
               __delay_ms(50);
               break;
           case MODE_XMAS:
               random_one_xmas(brightness_val);
               update_needed = true;
               
               __delay_ms(50);
               break;
           case MODE_TNET:
               random_one_tnet(brightness_val);
               update_needed = true;
               
               __delay_ms(50);
               break;
           case MODE_SPARKS:
               set_all(0x00, 0x00, 0x00);
               random_one(brightness_val, brightness_val, brightness_val);
               update_needed = true;
               
               __delay_ms(20);
               break;
           case MODE_STATIC:
               break;
        }
    }
}
