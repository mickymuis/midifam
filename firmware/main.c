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
 * SWITCH0              -> 12   PB0
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

#define F_CPU 16000000UL        // 16Mhz crystal is attached
#define BAUD_UBRR 31            // Sets the BAUD rate to 31,250 bit/s on a 16Mhz (31) crystal (use 15 on 8Mhz)

//#define TIM0_CLK F_CPU >> 3

//#define TIM1_CLK F_CPU

#include <util/delay.h>

enum MIDI_STATUS {
    MIDI_ST_NOTEOFF      = 0x8,
    MIDI_ST_NOTEON       = 0x9,
    MIDI_ST_PPRESSURE    = 0xA,
    MIDI_ST_CC           = 0xB,
    MIDI_ST_PC           = 0xC,
    MIDI_ST_CPRESSURE    = 0xD,
    MIDI_ST_PBEND        = 0xE,
    MIDI_ST_SYS          = 0xF
};

enum MIDI_SYS_COMMON {
    MIDI_SC_SYSEX_START  = 0xF0,
    MIDI_SC_TCQF         = 0xF1,
    MIDI_SC_SONGPOS      = 0xF2,
    MIDI_SC_SONGSEL      = 0xF3,
    MIDI_SC_TUNEREQ      = 0xF6
};

enum MIDI_SYS_REALTIME {
    MIDI_SR_CLOCK        = 0xF8,
    MIDI_SR_START        = 0xFA,
    MIDI_SR_CONTINUE     = 0xFB,
    MIDI_SR_STOP         = 0xFC,
    MIDI_SR_ACTIVE       = 0xFE,
    MIDI_SR_RESET        = 0xFF
};

#define MIDI_STATUS_BIT 0x80
#define MIDI_STATUS_MASK 0xF0
#define MIDI_SYSRT_BITS  0xF8

struct {
    int8_t msgStatus;         // Status-code of the current message (0 if none)
    int8_t msgChannel;        // Destination channel of the current message
    int8_t msgSize;           // Expected number of data bytes of the current message
    int8_t msgBufOffs;        // Current offset into the buffer
    uint8_t msgBuf[16];       // Buffer holding the data bytes for the current message
    int8_t listenChannel;     // Channel to respond to (0-15, -1 for all)
    int8_t clock;             // Current index of the 24PPQN clock beat (if any) 

} midirx;


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
    do { adv_clock(); }
    while( dfam.position != step );
}

void
sw_held() {
    led( 1 );
    adv_step( 3 );
}

void
sw_released() {
    led( 0 );
}

void
reset() {
    dfam.position =7; 
    dfam.running =0;
}

uint8_t
midi_dataSize( uint8_t status ) {
    uint8_t s =0;

    switch( status ) {
        case MIDI_ST_NOTEOFF:
        case MIDI_ST_NOTEON:
        case MIDI_ST_PPRESSURE:
        case MIDI_ST_CPRESSURE:
        case MIDI_ST_PBEND:
        case MIDI_ST_CC:
            s =2;
            break;
        case MIDI_ST_PC:
            s =1;
            break;
        case MIDI_ST_SYS:
            break;
    };
    return s;
}

void
midirx_noteOn( uint8_t note, uint8_t velocity ) {
    // Map the notes C..G in each octave to steps 0..7
    // Notes G#..B trigger no steps
    adv_step( note % 12 );
    led( 1 );
}

void
midirx_noteOff( uint8_t note, uint8_t velocity ) {
    led( 0 );
}

void
midirx_clock( uint8_t npulse ) {
    // Advance the clock on 1/8 notes
    if( npulse == 0 || npulse == 12 ) adv_clock();
}

void
midirx_msgComplete() {
    // See if this message is actually for us...
    if( midirx.msgChannel != midirx.listenChannel 
     && midirx.listenChannel != -1 ) return;

    switch( midirx.msgStatus ) {
        case MIDI_ST_NOTEOFF:
            midirx_noteOff( midirx.msgBuf[0] & 0x7F, midirx.msgBuf[1] & 0x7F );
            break;
        case MIDI_ST_NOTEON:
            if( (midirx.msgBuf[1] & 0x7F) == 0 )
                midirx_noteOff( midirx.msgBuf[0] & 0x7F, 0 );
            else    
                midirx_noteOn( midirx.msgBuf[0] & 0x7F, midirx.msgBuf[1] & 0x7F );
            break;
        case MIDI_ST_PPRESSURE:
            break;
        case MIDI_ST_CC:
            break;
        case MIDI_ST_PC:
            break;
        case MIDI_ST_CPRESSURE:
            break;
        case MIDI_ST_PBEND:
            break;
        case MIDI_ST_SYS:
            break;
    };

}

void
midirx_sysRt( uint8_t data ) {
    // System-wide realtime messages
    switch( data ) {
        case MIDI_SR_CLOCK:
            if( ++midirx.clock == 24 ) midirx.clock =0;
            midirx_clock( midirx.clock );
            break;
        case MIDI_SR_START:
            break;
        case MIDI_SR_CONTINUE:
            break;
        case MIDI_SR_STOP:
            break;
        // We don't implement these
        case MIDI_SR_ACTIVE:
        case MIDI_SR_RESET:
            break;
    }
}

void
midirx_msg( uint8_t msg ) {
    if( (msg & MIDI_SYSRT_BITS) == MIDI_SYSRT_BITS ) {
        // System realtime message
        midirx_sysRt( msg );
    } else if( msg & MIDI_STATUS_BIT ) {
        // Status message / message begin
        midirx.msgChannel = msg & 0xF;
        midirx.msgStatus  = (msg & MIDI_STATUS_MASK) >> 4;
        midirx.msgSize    = midi_dataSize( midirx.msgStatus );
        midirx.msgBufOffs = 0;
    } else if( midirx.msgStatus ) {
        // Data byte
        midirx.msgBuf[midirx.msgBufOffs++] =msg;
        if( midirx.msgBufOffs == midirx.msgSize ) {
            midirx_msgComplete();
            // Reset for a possible next message with the same status
            midirx.msgBufOffs =0;
        }
    }
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
    // Set the BAUD rate (specified at the top)
    UBRRH =(uint8_t)(BAUD_UBRR >> 8); // High bits
    UBRRL =(uint8_t)(BAUD_UBRR);      // Low bits

    // Enable receiving only
    UCSRB =(1 << RXEN);

    // Set the MIDI frame type: 
    // Asynchronous, one stop bit, eight data bits, one stop bit, no parity bits
    // UCSRC = 0b00000110
    UCSRC =(3 << UCSZ0);
}

void
setup_midi() {
    // Read the selected MIDI channel from DIPSWITCH3..DIPSWITCH0

    midirx.listenChannel =-1;

    midirx.msgStatus     =0;
    midirx.msgBufOffs    =0;
}

void
poll_sw() {
    static int sw =1;
    int rsw;
    
    // Poll for changes on SWITCH0 / debounce
    if( (rsw = PINB & (1<<PB0)) != sw ) {
        _delay_ms( 20 );
        if( rsw == (PINB & (1<<PB0)) ) {
            sw = rsw;
            if( !sw ) {
                sw_held();
            }
            else {
                sw_released();
            }
        }
    }
}

void
poll_serial() {
    // Poll for incoming MIDI frames
    if( UCSRA & (1 << RXC) ) {
        uint8_t msg = UDR;
        midirx_msg( msg );
        //adv_clock();
    }
}

int 
main() {
    setup_io();
    setup_serial();
    setup_midi();
    reset();

    sei();
    while(1) { 
        poll_sw();
        poll_serial();
    }
}
