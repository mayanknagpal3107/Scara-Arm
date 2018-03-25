
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <compat/deprecated.h>
#include <string.h>
#include <stdbool.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////			 Servo Drive 			/////////////////////////////////////////////
#define SERVO_PORT  PORTC
#define SERVO_DDR   DDRC

// Upto 8 servos (since pulses are generated in
// sequence + only one port is used).
#define N_SERVOS   3

// Servo times (this is Futaba timing).
#define SERVO_MIN    500 // 탎
#define SERVO_MAX   2500 // 탎
#define SERVO_MID   (SERVO_MIN + SERVO_MAX) / 2

// Time between servo pulses. 
// for 8 servos servo frame=20ms(20000us) frame, for 2servos=(8000us)
/*servo frame depends on the warnings below and hence
  depends on the no_of_servos.*/
#define SERVO_FRAME 22000 // 탎 // 

// Time slot available for each servo.
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS)

#if (SERVO_TIME_DIV < SERVO_MAX + 50)
#warning "Output fewer servo signals or increase SERVO_FRAME"
#endif
#if ((SERVO_TIME_DIV * (F_CPU / 1000000UL)) >= 0xFF00)
#warning "Output more servo signals or decrease SERVO_FRAME (or use the prescaler)"
#endif

// Computing timer ticks given 탎.
// Note, this version works out ok with even MHz F_CPU (e.g., 1, 2, 4, 8, 16 MHz).
// (Not a good idea to have this end up as a floating point operation)
#define US2TIMER1(us) ((us) * (uint16_t)(F_CPU / 1E6))

#define SERVO_MASK 0xff

// Servo times - to be entered as timer1 ticks (using US2TIMER1).
// This must be updated with interrupts disabled.
volatile uint16_t servoTime[N_SERVOS];

// Servo output allocation (on a single port currently).
// array of masks for initiating servos  

const static uint8_t servoOutMask[N_SERVOS] = {
    
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,
	0b00010000,
	0b00100000,
	0b01000000, // PX0  		
	0b10000000, // PX7
};
// Servo mask is just the above masks ored.


void servoStart(void)
{
    // Outputs
    /*sbi(DDRB,6);
    sbi(DDRB,7);
	*/
	DDRC=0xff;
    
	// Setupt a first compare match
    OCR1A = TCNT1 + US2TIMER1(100);
    // start timer 1 with no prescaler
    TCCR1B = (1 << CS10);      
    // Enable interrupt
    TIMSK |= (1 << OCIE1A);
}

void servoSet(uint8_t servo, uint16_t time /* 탎 */)
{
    uint16_t ticks = US2TIMER1(time);
    cli();
    servoTime[servo] = ticks;
    sei();
}

/*void old_setServoPosition(int portNumber,int Angle){
	uint16_t time = 0;
	time =  400+(2100/180)*Angle;
	
	servoSet(portNumber,time);     
}*/

void setServoPosition(int portNumber,int Angle){
	uint16_t time = 0;
	time =  500+(int)(2500/180)*Angle;
	
	servoSet(portNumber,time);     
}

ISR(TIMER1_COMPA_vect)
{
    
	static uint16_t nextStart;
    static uint8_t servo;
    static bool outputHigh = true;
    uint16_t currentTime = OCR1A;
    uint8_t mask = servoOutMask[servo];
  
    if (outputHigh) {
        SERVO_PORT |= mask;
        // Set the end time for the servo pulse
        OCR1A = currentTime + servoTime[servo];
        nextStart = currentTime + US2TIMER1(SERVO_TIME_DIV);
    } else {
        SERVO_PORT &= ~mask;
        if (++servo == N_SERVOS) {
            servo = 0;
        }
        OCR1A = nextStart;
    }
  outputHigh = !outputHigh;
	
}
