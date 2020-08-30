/**
 * YM2612 Test Code (AVR / Arduino Uno)
 * 
 * This program tests the YM2612 FM sound chip by configuring it to sound like a grand piano
 * and cycling a note on and off in a loop. 
 * 
 * For more information about the YM2612 registers please see: http://www.smspower.org/maxim/Documents/YM2612
 * A copy of the YM2612 pinout can be found here: https://console5.com/wiki/YM2612
 * 
 * @remarks This code is adapted from the YM2612 test code of Furrtek and Fabien Batteix <skywodd@gmail.com>
 * @warning This test code is made to run on an ATmega328/ATmega168 mcu with a 16MHz external crystal.
 * @remarks The Arduino uno comes equipped with the requisite 16MHz crystal, see: https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf
 */

#include <Arduino.h>

void setup() {
    /* === Dependencies === */
    #include <avr/io.h>     // For I/O and other AVR registers
    #include <util/delay.h> // For timing

    /* === Pinmap (Arduino UNO compatible) === */
    // YM2612 Clock Pin (ATmega Port B)
    #define YM_MCLOCK_DDR DDRB
    #define YM_MCLOCK (1) // PB1 = OC1A (= pin D9 for Arduino UNO)

    // YM2612 Control Pins (ATmega Port C) 
    #define YM_CTRL_DDR DDRC
    #define YM_CTRL_PORT PORTC
    #define YM_IC (5) // PC5 (= pin A5 for Arduino UNO)
    #define YM_CS (4) // PC4 (= pin A4 for Arduino UNO)
    #define YM_WR (3) // PC3 (= pin A3 for Arduino UNO)
    #define YM_RD (2) // PC2 (= pin A2 for Arduino UNO)
    #define YM_A0 (1) // PC1 (= pin A1 for Arduino UNO)
    #define YM_A1 (0) // PC0 (= pin A0 for Arduino UNO)

    // YM2612 Data Pins (ATmega Port D)
    #define YM_DATA_DDR DDRD
    #define YM_DATA_PORT PORTD // Whole PORT D for data bus (= pins D0 to D7 for Arduino UNO)

    
    /* === Pins setup === */
    YM_MCLOCK_DDR |= _BV(YM_MCLOCK);

    YM_CTRL_DDR |= _BV(YM_IC) | _BV(YM_CS) | _BV(YM_WR) | _BV(YM_RD) | _BV(YM_A0) | _BV(YM_A1); // sets as write pins
    YM_CTRL_PORT |= _BV(YM_IC) | _BV(YM_CS) | _BV(YM_WR) | _BV(YM_RD); // IC, CS, WR and RD HIGH by default
    YM_CTRL_PORT &= ~(_BV(YM_A0) | _BV(YM_A1)); // A0 and A1 LOW by default
    
    YM_DATA_DDR = 0xFF; // set all 8 pins on this port to write

    
    /* === F_CPU / 2 clock generation === */
    // See ATmega datasheet for register definitions
    TCCR1A = _BV(COM1A0);            // Toggle OC1A on compare match
    TCCR1B = _BV(WGM12) | _BV(CS10); // CTC mode with prescaler /1 
    TCCR1C = 0;                      // Flag reset 
    TCNT1 = 0;                       // Counter reset 
    OCR1A = 0;                       // Divide base clock by two 

}

static void write_ym(uint8_t data) {
    // open the data bus and pass the data
    YM_CTRL_PORT &= ~_BV(YM_CS); // CS LOW (sets data bus impedances low)
    YM_DATA_PORT = data;
    _delay_us(1);
    // set write-to mode
    YM_CTRL_PORT &= ~_BV(YM_WR); // write data LOW (write mode on)
    YM_CTRL_PORT |= _BV(YM_RD);  // read data HIGH (write mode on)
    _delay_us(5);
    // set read statuses mode 
    YM_CTRL_PORT |= _BV(YM_WR);  // write data HIGH (read mode on)
    YM_CTRL_PORT &= ~_BV(YM_RD); // read data LOW (read mode on)
    _delay_us(5);
    // close the data bus
    YM_CTRL_PORT |= _BV(YM_CS); // CS HIGH

}


static void setreg(uint8_t reg, uint8_t data) {
    // specify registers (timers + channels 1-3) to write
    YM_CTRL_PORT &= ~_BV(YM_A0); // A0 LOW (select register)
    write_ym(reg);
    // write the data to those registers
    YM_CTRL_PORT |= _BV(YM_A0);  // A0 HIGH (write register)
    write_ym(data);

}

void init() {
    /* === Reset YM2612 === */
    YM_CTRL_PORT &= ~_BV(YM_IC);
    _delay_ms(10);
    YM_CTRL_PORT |= _BV(YM_IC);
    _delay_ms(10);

    /* === YM2612 Test code === */ 
    setreg(0x22, 0x00); // LFO off
    setreg(0x27, 0x00); // Note off (channel 0)
    setreg(0x28, 0x01); // Note off (channel 1)
    setreg(0x28, 0x02); // Note off (channel 2)
    setreg(0x28, 0x04); // Note off (channel 3)
    setreg(0x28, 0x05); // Note off (channel 4)
    setreg(0x28, 0x06); // Note off (channel 5)
    setreg(0x2B, 0x00); // DAC off
    setreg(0x30, 0x71); //
    setreg(0x34, 0x0D); //
    setreg(0x38, 0x33); //
    setreg(0x3C, 0x01); // DT1/MUL
    setreg(0x40, 0x23); //
    setreg(0x44, 0x2D); //
    setreg(0x48, 0x26); //
    setreg(0x4C, 0x00); // Total level
    setreg(0x50, 0x5F); //
    setreg(0x54, 0x99); //
    setreg(0x58, 0x5F); //
    setreg(0x5C, 0x94); // RS/AR 
    setreg(0x60, 0x05); //
    setreg(0x64, 0x05); //
    setreg(0x68, 0x05); //
    setreg(0x6C, 0x07); // AM/D1R
    setreg(0x70, 0x02); //
    setreg(0x74, 0x02); //
    setreg(0x78, 0x02); //
    setreg(0x7C, 0x02); // D2R
    setreg(0x80, 0x11); //
    setreg(0x84, 0x11); //
    setreg(0x88, 0x11); //
    setreg(0x8C, 0xA6); // D1L/RR
    setreg(0x90, 0x00); //
    setreg(0x94, 0x00); //
    setreg(0x98, 0x00); //
    setreg(0x9C, 0x00); // Proprietary
    setreg(0xB0, 0x32); // Feedback/algorithm
    setreg(0xB4, 0xC0); // Both speakers on
    setreg(0x28, 0x00); // Key off
    setreg(0xA4, 0x22); // 
    setreg(0xA0, 0x69); // Set frequency

}

void loop() {
    // cycle tone on/off
    _delay_ms(1000);
    setreg(0x28, 0xF0); // Key on
    _delay_ms(1000);
    setreg(0x28, 0x00); // Key off

}