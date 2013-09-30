/*
 * Mavrx_DX4e.c
 *
 * Created: 01/09/2013 23:43:32
 *  Author: Yuan
 */ 
/*
1  PD3 - Buzzer (should idle low to prevent unnecessary current drain)
2  PD4 - Trainer port detect (detects whether trainer port is plugged in, plugged in is low) active low
3  GND
4  VCC
5  GND
6  VCC
7  XTAL1
8  XTAL2
9  PD5 - BIND button (depressed is low, released is high) active low
10 PD6 - RATE switch (LO is low, HI is high)
11 PD7 - AUX switch (OFF is low, ON is high)
12 PB0 - THR REV switch (NOR is low, REV is high)
13 PB1 - AIL REV switch (NOR is low, REV is high)
14 PB2 - ELE REV switch (NOR is low, REV is high)
15 PB3 - RUD REV switch (NOR is low, REV is high), also ICSP MOSI (ensure switch is in REV position for ICSP)
16 PB4 - MIX switch (OFF is low, ON is high), also ICSP MISO (ensure switch is in ON position for ICSP)
17 PB5 - MD switch (MD-2 is low, MD-4 is high), also ICSP SCK (ensure switch is in MD-4 position for ICSP)
18 AVCC
19 ADC6 - battery voltage
20 AREF
21 GND
22 ADC7 - Throttle axis
23 PC0 - Green LED 1
24 PC1 - Green LED 2
25 PC2 - Red LED 0
26 PC3/ADC3 - Rudder axis
27 PC4/ADC4 - Elevator Axis
28 PC5/ADC5 - Aileron Axis
29 RESET
30 PD0 - Green LED 3
31 PD1/TXD - radio module
    32 PD2 - trainer port (capacitor decoupled, and can be either input or output)

// battery voltage thresholds: 5.75V, 5.25V, 4.75V (brownout at 3.6V)
*/

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED0On()	PORTC &= ~0x04;
#define LED1On()	PORTC &= ~0x01;
#define LED2On()	PORTC &= ~0x02;
#define LED3On()	PORTD &= ~0x01;

#define TRAINERPIN  ((PIND >> 4) & 0x1)
#define BINDPIN     ((PIND >> 5) & 0x1)
#define RATEPIN     ((PIND >> 6) & 0x1)
#define AUXPIN      ((PIND >> 7) & 0x1)
#define THROPIN     ((PINB >> 0) & 0x1)
#define AILEPIN     ((PINB >> 1) & 0x1)
#define ELEVPIN     ((PINB >> 2) & 0x1)
#define RUDDPIN     ((PINB >> 3) & 0x1)
#define MIXPIN      ((PINB >> 4) & 0x1)
#define MDPIN       ((PINB >> 5) & 0x1)

#define NOTE5C	239
#define NOTE5CS	225
#define NOTE5D	213
#define NOTE5DS	201
#define NOTE5E	190
#define NOTE5F	179
#define NOTE5FS	169
#define NOTE5G	159
#define NOTE5GS	150
#define NOTE5A	142
#define NOTE5AS	134
#define NOTE5B	127

#define NOTE6C	119
#define NOTE6CS	113
#define NOTE6D	106
#define NOTE6DS	100
#define NOTE6E	95
#define NOTE6F	89
#define NOTE6FS	84
#define NOTE6G	80
#define NOTE6GS	75
#define NOTE6A	71
#define NOTE6AS	67
#define NOTE6B	63

#define NOTE7C	60
#define NOTE7CS	56
#define NOTE7D	53
#define NOTE7DS	50
#define NOTE7E	47
#define NOTE7F	45
#define NOTE7FS	42
#define NOTE7G	40
#define NOTE7GS	38
#define NOTE7A	36
#define NOTE7AS	34
#define NOTE7B	32

#define NOTEPAUSE 1
#define NOTESTOP 0

#define OVERSAMPLE 10

void fastLoop(void);
void slowLoop(void);

void getADC(void);

uint8_t noteBuffer[20];
uint8_t noteCounter;
uint8_t noteInterruptable;

uint8_t LED0Duty = 0;
uint8_t LED1Duty = 0;
uint8_t LED2Duty = 0;
uint8_t LED3Duty = 0;

uint8_t battPulse;
uint16_t throVoltage;
uint16_t aileVoltage;
uint16_t elevVoltage;
uint16_t ruddVoltage;
uint8_t toggleCounter;

uint8_t rateSwitch; // 0 for LOW, 1 for HIGH, treat as active-low
uint8_t auxSwitch;  // 0 for OFF, 1 for ON, treat as active-high
uint8_t bindSwitch; // 0 for pressed, treat as active-low
uint8_t throToggle; // 0 for NOR, 1 for REV, treat as active-low
uint8_t aileToggle; // 0 for NOR, 1 for REV, treat as active-low
uint8_t elevToggle; // 0 for NOR, 1 for REV, treat as active-low
uint8_t ruddToggle; // 0 for NOR, 1 for REV, treat as active-low
uint8_t mixToggle;  // 0 for NOR, 1 for REV, treat as active-low
uint8_t mdToggle;   // 0 for NOR, 1 for REV, treat as active-low

uint8_t mute;

uint8_t trainerPlugged;
uint8_t transmitMode; // 0 for normal, 1 for bind, 2 for trainer slave, 3 for trainer master

uint8_t uartBuffer[18];
uint8_t uartCounter;
uint8_t uartLength;

void LEDOff(void) {
    PORTC |= 0x07;
    PORTD |= 0x01;
}

void playNote(uint8_t note) {
    if(mute == 0) {
        TCCR2A = 0x12; // CTC mode toggle OC2B
        if(OCR2A != note) {
            OCR2A = note;
            OCR2B = note;
            TCNT2 = 0;
        }
    }
}

void stopNote(void) {
	TCCR2A = 0x00; // disconnect OC2B
	PORTD &= ~0x08; // set low
}

void twoTone(uint8_t note) {
    if(noteInterruptable) {
        noteBuffer[0] = note;
        noteBuffer[1] = NOTEPAUSE;
        noteBuffer[2] = note;
        noteBuffer[3] = NOTESTOP;
        noteCounter = 0;
        noteInterruptable = 1;
    }
}

void oneTone(uint8_t note) {
    if(noteInterruptable) {
        noteBuffer[0] = note;
        noteBuffer[1] = NOTESTOP;
        noteCounter = 0;
        noteInterruptable = 1;
    }
}

void paschaOvo(void);

int main(void) {
	// Setup pins
	DDRB = 0x00;	// All pins inputs
	DDRC = 0x07;	// PC0,1,2 as outputs for the LEDs
	DDRD = 0x0b;	// PD0 as output for LED, PD1 for TXD, PD3 for buzzer
	
	PORTD = 0xe0;	// Pullups for switches
	PORTB = 0x3f;	// Pullups for switches
	
    // LEDs
    LEDOff();
    
	// Buzzer
    TCCR2B = 0x03; // prescaler at 64
    stopNote();
    noteBuffer[0] = NOTE6G;
    noteBuffer[1] = NOTE6FS;
    noteBuffer[2] = NOTE6DS;
    noteBuffer[3] = NOTE5A;
    noteBuffer[4] = NOTE5GS;
    noteBuffer[5] = NOTE6E;
    noteBuffer[6] = NOTE6GS;
    noteBuffer[7] = NOTE7C;
    noteBuffer[8] = NOTE7C;
    noteBuffer[9] = NOTESTOP;
    noteCounter = 0;
    noteInterruptable = 0;
    
    // ADC
    ADMUX = 0x4f; // start off with mux on internal input
    ADCSRA = 0x80; // ADC EN
    DIDR0 = 0x38; // disable digital buffers on ADC pins
    
    // Timer
    TCCR0A = 0x00;  // Normal mode
    TCCR0B = 0x03;  // prescaler at 64 results in 125kHz ticks
	
    // UART
    cli(); // disable global interrupts
    UBRR0L = 3;  UBRR0H = 0;  // Baud at 125kbaud
    UCSR0C = 0x06; // Async mode, 8N1
    UCSR0B = 0x08; // TX enable
    uartCounter = 0;
    uartLength = 0;
    
    // Get startup mode
    trainerPlugged = TRAINERPIN;
    bindSwitch = BINDPIN;
    transmitMode = 0;
    if(trainerPlugged == 0) transmitMode = 2;
    if(bindSwitch == 0) transmitMode = 1;
    
    // Timer loop!
    static uint8_t fastScaler = 255;
    static uint8_t slowScaler = 255;
    
    while(1) {
        TCNT0 = 0;
        
        while(TCNT0 < 200) {
            if(TCNT0 < 200 && TCNT0 >= 200-LED0Duty/4) LED0On(); // red LED too bright, reduce duty
            if(TCNT0 < 200 && TCNT0 >= 200-LED1Duty) LED1On();
            if(TCNT0 < 200 && TCNT0 >= 200-LED2Duty) LED2On();
            if(TCNT0 < 200 && TCNT0 >= 200-LED3Duty) LED3On();
        }
        LEDOff();
        
        // should be going at about 625Hz
        getADC();
        
        if(++fastScaler >= OVERSAMPLE) { // going at about 60Hz
            fastScaler = 0;
            
            if(++slowScaler >= 6) { // should be going at about 10Hz
                slowScaler = 0;
                slowLoop();
            }
            
            // this loop runs slower than 50Hz (actual time depends on how long it takes to run the code
            fastLoop();
            throVoltage = 0;
            ruddVoltage = 0;
            elevVoltage = 0;
            aileVoltage = 0;
        }
    }
}

void getADC(void) {
    // Get inputs and switches - the ADC is interleaved with switch stuff becase some delay is needed for the ADC cap to charge
    ADMUX = 0x47;   // select chan 7 Throttle
        static uint8_t rateSwitchPrev = 3;
        rateSwitch = RATEPIN;
        if(rateSwitch != rateSwitchPrev) {
            rateSwitchPrev = rateSwitch;
            if(rateSwitch) oneTone(NOTE7C);
            else twoTone(NOTE7C);
            
            if(bindSwitch == 0) { // if bind button is held down and the rate switch toggled four times, enter range check mode
                toggleCounter++;
                if(toggleCounter == 4) {
                    noteBuffer[0] = NOTE5G;
                    noteBuffer[1] = NOTE5FS;
                    noteBuffer[2] = NOTE5F;
                    noteBuffer[3] = NOTE5E;
                    noteBuffer[4] = NOTE5DS;
                    noteBuffer[5] = NOTE5D;
                    noteBuffer[6] = NOTE5CS;
                    noteBuffer[7] = NOTE5C;
                    noteBuffer[8] = NOTESTOP;
                    noteCounter = 0;
                    noteInterruptable = 0;
                }
                else if(toggleCounter > 10) paschaOvo();
            }
        }
        
        static uint8_t auxSwitchPrev = 3;
        auxSwitch  = AUXPIN;
        if(auxSwitch != auxSwitchPrev) {
            auxSwitchPrev = auxSwitch;
            if(auxSwitch) twoTone(NOTE6E);
            else oneTone(NOTE6E);
        }
    ADCSRA |= 0x40; // start
    while(ADCSRA & 0x40); // wait for completion
    throVoltage += ADC;
    
    ADMUX = 0x43;   // select chan 3 Rudder
        static uint8_t bindSwitchPrev = 3;
        bindSwitch = BINDPIN;
        if(bindSwitch != bindSwitchPrev) {
            bindSwitchPrev = bindSwitch;
            if(bindSwitch) {
                oneTone(NOTE6C);
                toggleCounter = 0; // when releasing Bind switch, zero the toggle counter (for the range check mode)
                if(transmitMode == 1) transmitMode = 0; // when releasing Bind switch, escape from bind mode
            }
            else oneTone(NOTE6G);
        }
        static uint8_t throTogglePrev = 3;
        throToggle  = THROPIN;
        if(throToggle != throTogglePrev) {
            throTogglePrev = throToggle;
            if(throToggle) oneTone(NOTE5C);
            else twoTone(NOTE5C);
        }
    ADCSRA |= 0x40; // start
    while(ADCSRA & 0x40); // wait for completion
    ruddVoltage += ADC;
    
    ADMUX = 0x44;   // select chan 4 Elevator
        static uint8_t aileTogglePrev = 3;
        aileToggle  = AILEPIN;
        if(aileToggle != aileTogglePrev) {
            aileTogglePrev = aileToggle;
            if(aileToggle) oneTone(NOTE5D);
            else twoTone(NOTE5D);
        }
        
        static uint8_t elevTogglePrev = 3;
        elevToggle  = ELEVPIN;
        if(elevToggle != elevTogglePrev) {
            elevTogglePrev = elevToggle;
            if(elevToggle) oneTone(NOTE5E);
            else twoTone(NOTE5E);
        }
        
        static uint8_t ruddTogglePrev = 3;
        ruddToggle  = RUDDPIN;
        if(ruddToggle != ruddTogglePrev) {
            ruddTogglePrev = ruddToggle;
            if(ruddToggle) oneTone(NOTE5F);
            else twoTone(NOTE5F);
        }
    ADCSRA |= 0x40; // start
    while(ADCSRA & 0x40); // wait for completion
    elevVoltage += ADC;
    
    ADMUX = 0x45;   // select chan 5 Aileron
        static uint8_t mixTogglePrev = 3;
        mixToggle  = MIXPIN;
        if(mixToggle != mixTogglePrev) {
            mixTogglePrev = mixToggle;
            if(mixToggle) oneTone(NOTE5G);
            else twoTone(NOTE5G);
        }
        
        static uint8_t mdTogglePrev = 3;
        mdToggle  = MDPIN; // mute
        if(mdToggle != mdTogglePrev) {
            mdTogglePrev = mdToggle;
            if(mdToggle) mute = 1;
            else {
                mute = 0; // plays tone on unmute
                if(noteInterruptable) {
                    noteBuffer[0] = NOTE6C;
                    noteBuffer[1] = NOTE7C;
                    noteBuffer[2] = NOTE7D;
                    noteBuffer[3] = NOTE7E;
                    noteBuffer[4] = NOTESTOP;
                    noteCounter = 0;
                }
            }
        }
    ADCSRA |= 0x40; // start
    while(ADCSRA & 0x40); // wait for completion
    aileVoltage += ADC;
}

void fastLoop(void) {
    // Output to radio
    uint16_t throV = throVoltage / OVERSAMPLE;
    uint16_t aileV = 0x3ff - (aileVoltage / OVERSAMPLE);
    uint16_t elevV = elevVoltage / OVERSAMPLE;
    uint16_t ruddV = 0x3ff - (ruddVoltage / OVERSAMPLE);
    
    switch(transmitMode) {
        case 3: // in trainer master mode
            if(rateSwitch == 0) { // rate swith in slave has control position
                break;
            }
            // FALLTHROUGH! only falls through if rateSwitch is 1, i.e. master in control
        case 0: // in normal mode
            uartBuffer[0] = 0x18;
            if(toggleCounter >= 4) uartBuffer[0] = 0x38; // in range check mode
            
            uartBuffer[1] = 0x00;
            
            uartBuffer[2] = 0x00 | ((throV >> 8) & 0x03);
            uartBuffer[3] = ((throV) & 0xff);
            
            uartBuffer[4] = 0x04 | ((aileV >> 8) & 0x03);
            uartBuffer[5] = ((aileV) & 0xff);
            
            uartBuffer[6] = 0x08 | ((elevV >> 8) & 0x03);
            uartBuffer[7] = ((elevV) & 0xff);
            
            uartBuffer[8] = 0x0c | ((ruddV >> 8) & 0x03);
            uartBuffer[9] = ((ruddV) & 0xff);
            
            if(mixToggle) { // in mix mode, send the four front switches mixed into the bind channel, and the rate switch mixed into the aux channel
                uartBuffer[10] = 0x10;
                if(auxSwitch == 0) uartBuffer[10] |= 0x03;
                uartBuffer[11] = (rateSwitch << 4);
                
                uartBuffer[12] = 0x14;
                if(bindSwitch == 0) uartBuffer[12] |= 0x03;
                uartBuffer[13] = (throToggle << 7) | (aileToggle << 6) | (elevToggle << 5) | (ruddToggle << 4);
                
            }
            else {
                if(auxSwitch) {
                    uartBuffer[10] = 0x10;
                    uartBuffer[11] = 0xaa;
                }
                else {
                    uartBuffer[10] = 0x13;
                    uartBuffer[11] = 0x54;
                }
                
                if(bindSwitch) {
                    uartBuffer[12] = 0x14;
                    uartBuffer[13] = 0xaa;
                }
                else {
                    uartBuffer[12] = 0x17;
                    uartBuffer[13] = 0x54;
                }
            }
            
            uartCounter = 0;
            uartLength = 14;
            break;
        case 1: // in bind mode
            uartBuffer[0] = 0x98;
            uartBuffer[1] = 0x00;
            uartBuffer[2] = 0x00;
            uartBuffer[3] = 0x00;
            uartBuffer[4] = 0x05;
            uartBuffer[5] = 0xff;
            uartBuffer[6] = 0x09;
            uartBuffer[7] = 0xff;
            uartBuffer[8] = 0x0d;
            uartBuffer[9] = 0xff;
            uartBuffer[10] = 0x10;
            uartBuffer[11] = 0xaa;
            uartBuffer[12] = 0x14;
            uartBuffer[13] = 0xaa;
            uartCounter = 0;
            uartLength = 14;
            break;
        case 2: // in trainer slave mode
            uartCounter = 0;
            uartLength = 0;
            break;
    }
    
    while(uartCounter < uartLength) {
        while((UCSR0A & 0x20) == 0);
        UDR0 = uartBuffer[uartCounter++];
    }
    
    // Power LED - pulses depending on battery level
    static uint8_t LED0Dir = 1;
    if(LED0Dir) {
        if(LED0Duty > 100) LED0Dir = 0;
        else LED0Duty += battPulse;
    }
    else {
        if(LED0Duty < battPulse) LED0Dir = 1;
        else LED0Duty -= battPulse;
    }
    
    // LED1 - is on for range check or bind mode (solid for range check, pulse for bind)
    static uint8_t LED1Dir = 0;
    if(transmitMode == 1) {// bind mode
        if(LED1Dir) {
            if(LED1Duty > 100) LED1Dir = 0;
            else LED1Duty += 10;
        }
        else {
            if(LED1Duty < 10) LED1Dir = 1;
            else LED1Duty -= 10;
        }
    }
    else if(toggleCounter >= 4) { // range check
        LED1Dir = 1;
        if(LED1Duty <= 190) LED1Duty+=10;
    }
    else {
        LED1Dir = 0;
        if(LED1Duty >= 10) LED1Duty-=10;
    }
    
    // LED2 - is on in trainer mode, pulses when slave is in control
    static uint8_t LED2Dir = 0;
    if(trainerPlugged) { // trainer unplugged, LED fades off
        LED2Dir = 0;
        if(LED2Duty >= 10) LED2Duty-=10;
    }
    else { // trainer plugged
        if(transmitMode == 3 && rateSwitch == 0) { // LED pulses in traner master mode and slave in control
            if(LED2Dir) {
                if(LED2Duty > 100) LED2Dir = 0;
                else LED2Duty += 10;
            }
            else {
                if(LED2Duty < 10) LED2Dir = 1;
                else LED2Duty -= 10;
            }
        }
        else {
            // LED is on (trainer slave mode, or trainer master mode but master in control)
            LED2Dir = 1;
            if(LED2Duty <= 190) LED2Duty+=10;
        }
    }
    
    // LED3 - mode indicator for the MIX switch
    static uint8_t LED3Dir = 0;
    LED3Dir = mixToggle;
    if(LED3Dir) {
        if(LED3Duty <= 190) LED3Duty+=10;
    }
    else {
        if(LED3Duty >= 10) LED3Duty-=10;
    }
    
}

void slowLoop(void) {
    // Read battery voltage
    ADMUX = 0x46;   // select chan 6
    ADCSRA |= 0x40; // start
    while(ADCSRA & 0x40); // wait for completion
    uint8_t level = ADC >> 2; // note: level is now approx 26.1x the actual voltage
    
    // Battery level pulse speeds
    if(level > 150) battPulse = 1; // slow pulse
    else if(level > 137) battPulse = 2;
    else if(level > 120) battPulse = 4;
    else {
        battPulse = 24; // very fast pulses
        
        if(noteBuffer[noteCounter] == 0) {
            noteBuffer[0] = NOTEPAUSE;
            noteBuffer[1] = NOTEPAUSE;
            noteBuffer[2] = NOTEPAUSE;
            noteBuffer[3] = NOTEPAUSE;
            noteBuffer[4] = NOTEPAUSE;
            noteBuffer[5] = NOTEPAUSE;
            noteBuffer[6] = NOTEPAUSE;
            noteBuffer[7] = NOTEPAUSE;
            noteBuffer[8] = NOTEPAUSE;
            noteBuffer[9] = NOTE5C;
            noteBuffer[10] = NOTE5C;
            noteBuffer[11] = NOTE5CS;
            noteBuffer[12] = NOTE5CS;
            noteBuffer[13] = NOTESTOP;
            noteCounter = 0;
            noteInterruptable = 1;
        }
    }
    
    // Check trainer plug plug
    static uint8_t trainerPluggedPrev;
    trainerPlugged = TRAINERPIN;
    if(trainerPlugged != trainerPluggedPrev) {
        trainerPluggedPrev = trainerPlugged;
        if(trainerPlugged) {
            // Not Plugged IN
            if(noteInterruptable) {
                noteBuffer[0] = NOTE6C;
                noteBuffer[1] = NOTE7C;
                noteBuffer[2] = NOTE6C;
                noteBuffer[3] = NOTESTOP;
                noteCounter = 0;
            }
            if(transmitMode == 2 || transmitMode == 3) transmitMode = 0; // escape out of trainer mode
        }
        else {
            // Plugged IN
            noteBuffer[0] = NOTE6C; // these notes interrupt the startup tune, this is by design
            noteBuffer[1] = NOTE6C;
            noteBuffer[2] = NOTE7C;
            noteBuffer[3] = NOTESTOP;
            noteCounter = 0;
            noteInterruptable = 0;
            if(transmitMode == 0) transmitMode = 3; // enter trainer mode
        }
    }
    
    // Beeps while toggling or binding beeps
    if(toggleCounter >= 4 || transmitMode == 1) {
        if(noteBuffer[noteCounter] == 0) {
            noteBuffer[0] = NOTEPAUSE;
            noteBuffer[1] = NOTEPAUSE;
            noteBuffer[2] = NOTEPAUSE;
            noteBuffer[3] = NOTEPAUSE;
            noteBuffer[4] = NOTEPAUSE;
            noteBuffer[5] = NOTEPAUSE;
            noteBuffer[6] = NOTEPAUSE;
            noteBuffer[7] = NOTEPAUSE;
            noteBuffer[8] = NOTEPAUSE;
            noteBuffer[9] = NOTE5C;
            noteBuffer[10] = NOTESTOP;
            noteCounter = 0;
            noteInterruptable = 1;
        }
    }
    
    // Play sounds
    if(noteBuffer[noteCounter] > 0) {
        if(noteBuffer[noteCounter] == NOTEPAUSE) {
            stopNote();
        }
        else {
            playNote(noteBuffer[noteCounter]);
        }
        noteCounter++;
    }
    else {
        stopNote();
        noteInterruptable = 1;
    }
}

#define GAP         20
#define SEMIQUAVER  110
#define QUAVER      2*SEMIQUAVER
#define CROTCHET    2*QUAVER
#define MINIM       2*CROTCHET
#define BREVE       2*MINIM
#define TRIPLET     CROTCHET/3
void paschaOvo(void) {
    stopNote();         _delay_ms(CROTCHET);
    playNote(NOTE5AS);  _delay_ms(MINIM - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    
    playNote(NOTE5AS);  _delay_ms(QUAVER + SEMIQUAVER);
    playNote(NOTE5GS);  _delay_ms(SEMIQUAVER);
    playNote(NOTE5AS);  _delay_ms(CROTCHET);
    stopNote();         _delay_ms(QUAVER);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    
    playNote(NOTE5AS);  _delay_ms(QUAVER + SEMIQUAVER);
    playNote(NOTE5GS);  _delay_ms(SEMIQUAVER);
    playNote(NOTE5AS);  _delay_ms(CROTCHET);
    stopNote();         _delay_ms(QUAVER);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5AS);  _delay_ms(TRIPLET - GAP);
    stopNote();         _delay_ms(GAP);
    
    playNote(NOTE5AS);  _delay_ms(QUAVER);
    playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
    stopNote();         _delay_ms(GAP);
    
    while(1) {
        playNote(NOTE5AS);  _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE5F);   _delay_ms(CROTCHET + QUAVER);
        playNote(NOTE5AS);  _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5AS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6D);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6DS);  _delay_ms(SEMIQUAVER);
        
        playNote(NOTE6F);   _delay_ms(MINIM - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(TRIPLET);
        playNote(NOTE6FS);  _delay_ms(TRIPLET);
        playNote(NOTE6GS);  _delay_ms(TRIPLET);
        
        playNote(NOTE6AS);  _delay_ms(MINIM + QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6AS);  _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6AS);  _delay_ms(TRIPLET);
        playNote(NOTE6GS);  _delay_ms(TRIPLET);
        playNote(NOTE6FS);  _delay_ms(TRIPLET);
        
        playNote(NOTE6GS);  _delay_ms(QUAVER + SEMIQUAVER);
        playNote(NOTE6FS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6F);   _delay_ms(CROTCHET + QUAVER);
        stopNote();         _delay_ms(QUAVER);
        playNote(NOTE6F);   _delay_ms(CROTCHET);
        
        playNote(NOTE6DS);  _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER);
        playNote(NOTE6DS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6F);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6FS);  _delay_ms(MINIM);
        playNote(NOTE6F);   _delay_ms(QUAVER);
        playNote(NOTE6DS);  _delay_ms(QUAVER);
        
        playNote(NOTE6CS);  _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER);
        playNote(NOTE6CS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6DS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6F);   _delay_ms(MINIM);
        playNote(NOTE6DS);  _delay_ms(QUAVER);
        playNote(NOTE6CS);  _delay_ms(QUAVER);
        
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER);
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6D);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6E);   _delay_ms(MINIM);
        playNote(NOTE6G);   _delay_ms(CROTCHET);
        
        playNote(NOTE6F);   _delay_ms(QUAVER);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        
        playNote(NOTE5AS);  _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE5F);   _delay_ms(CROTCHET + QUAVER);
        playNote(NOTE5AS);  _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5AS);  _delay_ms(SEMIQUAVER);
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6D);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6DS);  _delay_ms(SEMIQUAVER);
        
        playNote(NOTE6F);   _delay_ms(MINIM - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(TRIPLET);
        playNote(NOTE6FS);  _delay_ms(TRIPLET);
        playNote(NOTE6GS);  _delay_ms(TRIPLET);
        
        playNote(NOTE6AS);  _delay_ms(MINIM + CROTCHET);
        playNote(NOTE7CS);  _delay_ms(CROTCHET);
        
        playNote(NOTE7C);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE6A);   _delay_ms(MINIM);
        playNote(NOTE6F);   _delay_ms(CROTCHET);
        
        playNote(NOTE6FS);  _delay_ms(MINIM + CROTCHET);
        playNote(NOTE6AS);  _delay_ms(CROTCHET);
        
        playNote(NOTE6A);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE6F);   _delay_ms(MINIM - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE6F);   _delay_ms(CROTCHET);
        
        playNote(NOTE6FS);  _delay_ms(MINIM + CROTCHET);
        playNote(NOTE6AS);  _delay_ms(CROTCHET);
        
        playNote(NOTE6A);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE6F);   _delay_ms(MINIM);
        playNote(NOTE6D);   _delay_ms(CROTCHET);
        
        playNote(NOTE6DS);  _delay_ms(MINIM + CROTCHET);
        playNote(NOTE6FS);  _delay_ms(CROTCHET);
        
        playNote(NOTE6F);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER + QUAVER);
        playNote(NOTE6CS);  _delay_ms(MINIM);
        playNote(NOTE5AS);  _delay_ms(CROTCHET);
        
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        stopNote();         _delay_ms(SEMIQUAVER);
        playNote(NOTE6C);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6D);   _delay_ms(SEMIQUAVER);
        playNote(NOTE6E);   _delay_ms(MINIM);
        playNote(NOTE6G);   _delay_ms(CROTCHET);
        
        playNote(NOTE6F);   _delay_ms(QUAVER);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(SEMIQUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
        playNote(NOTE5F);   _delay_ms(QUAVER - GAP);
        stopNote();         _delay_ms(GAP);
    }
}