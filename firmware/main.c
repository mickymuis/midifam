/** main.c
 *
 * MiDiFAM -- A MIDI interface for Moog DFAM
 * Firmware for attiny2313/2313A/4213
 *
 * https://github.com/mickymuis/midifam/
 *
 *
 * (C) 2021 by Micky Faas <micky@edukitty.org>
 *
 * Connection overview (also see schematic)
 *
 * MIDI Rx              -> 2    PD0/RX
 * LED0                 -> 11   PD6
 * LED1                 -> 3    PD1
 * DIPSWITCH0 (MSB)     -> 6    PD2
 * DIPSWITCH1           -> 7    PD3
 * DIPSWITCH2           -> 8    PD4
 * DIPSWITCH3 (LSB)     -> 9    PD5
 * BTTN0                -> 12   PB0
 * TRIG0 (adv/clock)    -> 13   PB1
 * TRIG1 (run/stop)     -> 14   PB2
 * CV0 (MIDI velocity)  -> 15   PB3/OC1A
 * CV1 (MIDI mod)       -> 16   PB4/OC1B
 *
 * fuses: E:FF, H:DF, L:FF (external 16Mhz crystal connected)
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define TIM0_CLK F_CPU >> 3
#define TIM1_CLK F_CPU

#include <util/delay.h>

struct {
    uint8_t position;           // Position of DFAM's sequencer 0=first step
    uint8_t running;
} dfam;

void
led( uint8_t on ) {
    if( on )
        PORTD |= (1 << PD6);
    else
        PORTD &= ~(1 << PD6);
}


ISR(TIMER0_OVF_vect)    // tim0 interrupt service routine
{
}

void
adv_clock() {
    // Send a short 2uS pulse to the adv/clock port
    PORTB |= (1 << PB1);
    _delay_us(1);
    PORTB &= ~(1 << PB1);
    _delay_us(1);
    if( ++dfam.position > 7 ) dfam.position =0;
}

void
adv_step( uint8_t step ) {
    if( step > 7 ) return;
    while( dfam.position != step )
        adv_clock();
}

void
bttn_held() {
    led( 1 );
    adv_step( 3 );
}

void
bttn_released() {
    led( 0 );
}

void
setup_io() {
    // Enable trig0, trig1, cv0, cv1 ports (PB1, PB2, PB3, PB4 resp.)
    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);
    
    // LED0, LED1
    DDRD |= (1 << PD6) | (1 << PD1);

    // Tactile switch , enable pullup on PB0
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);
    
    // Timer1 setup procedure
    // PWM output will be on pin PB4/OC1B and PB3/OC1A, enable it
    DDRB |= (1 << DDB4) | (1 << DDB3);

    // Set the TTC1x registers to do the following
    // - Fast PWM (WGM12)
    // - TOP=ICR1 (WGM13, WGM11, ~WGM10)
    // - Non-inverted output compare on OC1A and OC1B (COM1B1, COM1A1)
    // - Prescaler 1 (~CS12, ~CS11, CS10)
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    TCCR1C = 0;

    // We use 7-bit fast PWM, since MIDI velocity is only 7-bit anyway
    // This yields a PWM frequency of F_CPU/(1*(1+127)) = 125 Khz when using a 16Mhz crystal
    ICR1 = 0x007F; 

    OCR1A =63; OCR1B =63;

}

void
setup_serial() {

}

void
reset() {
    dfam.position =7; 
    dfam.running =0;
}

int 
main() {
    setup_io();
    setup_serial();
    reset();

    int bttn =1, rbttn;

    sei();
    while(1) { 
    
        if( (rbttn = PINB & (1<<PB0)) != bttn ) {
            _delay_ms( 20 );
            if( rbttn == (PINB & (1<<PB0)) ) {
                bttn = rbttn;
                if( !bttn ) {
                    bttn_held();
                }
                else {
                    led( 0 );
                    bttn_released();
                }
            }
        }
    
    }

}
