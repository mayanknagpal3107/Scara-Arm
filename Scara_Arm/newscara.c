#include <avr/io.h>
#include <compat/deprecated.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/wdt.h>
//#include <avr/eeprom.h>
#include <stdlib.h>

int MyDelay0=50;
int MyDelay1=50;
int MyDelay2=50;
//------------------------------------------------------------------------//
//////////////////////////	HEADER FILES///////////////////////////////////////
//------------------------------------------------------------------------//

#include "uart.h"
#include "uart.c"
#include "motor.c"

void decode_angle_data(void);
void decode_speed_data(void);
void move_scara_angle(unsigned int servo_1_angle,unsigned int servo_2_angle,unsigned int servo_3_angle);
void set_scara_speed(unsigned int servo_1_speed,unsigned int servo_2_speed,unsigned int servo_3_speed);
void setServoSpeed(int portNumber, unsigned int speed);
void setServoPosition(int portNumber,int Angle); 
void get_serial_data(void);
void servoinit1(void);


int set_servo=0; //This variable takes care of the servo part init and acts as a flag

unsigned int packet[10], serial_data[10]={0};
int packetindex=0;
int Start=0, a=0 , s=0, angle=0;
unsigned char c=0;
int index=0;
unsigned int str[20], i=0;

int Start1=0;
int rxlength1=0;
//int storeflag=1;

int LastDegree0=0;
int LastDegree1=0;
int LastDegree2=0;
int CurrentDegree0=90;
int CurrentDegree1=90;
int CurrentDegree2=20;



unsigned char	command;
unsigned char 	counter;
unsigned int last_angle[5]={0};
unsigned int get=0;

void waits(unsigned int ms){				    //		WAITS		//
	unsigned int val;
	val=ms/60;
	while(val--){
		_delay_ms(10);
	}
}


//------------------------------------------------------------------------//
//////////////////////	 Servo Drive 	////////////////////////////////////
//------------------------------------------------------------------------//

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
// for 8 servos servo frame=60ms(60000us) frame, for 2servos=(8000us)
/*servo frame depends on the warnings below and hence
  depends on the no_of_servos.*/
//#define SERVO_FRAME 16000 // 탎 // 
#define SERVO_FRAME 8100 // 탎 // 

// Time slot available for each servo.
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS)

#if(SERVO_TIME_DIV < SERVO_MAX + 60)
#warning "Output fewer servo signals or increase SERVO_FRAME"
#endif
#if ((SERVO_TIME_DIV * (F_CPU / 1000000UL)) >= 0xFF00)
#warning "Output more servo signals or decrease SERVO_FRAME (or use the prescaler)"
#endif

// Computing timer ticks given 탎.
// Note, this version works out ok with even MHz F_CPU (e.g., 1, 2, 4, 8, 16 MHz).
// (Not a good idea to have this end up as a floating point operation)
#define US2TIMER1(us) ((us) * (uint16_t)(F_CPU / 1E6))		
// Servo mask is just the above masks ored.
#define SERVO_MASK 0b1111111
//unsigned int	debugflag1=0;		//////FOR DEBUGGING THE SUBROUTINE
//unsigned int	debugflag2=0;		//////FOR DEBUGGING THE SUBROUTINE

//unsigned char	buff[10];
//unsigned int	serial_data[10];
// Servo times - to be entered as timer1 ticks (using US2TIMER1).
// This must be updated with interrupts disabled.
volatile uint16_t servoTime[N_SERVOS];

// Servo output allocation (on a single port currently).
// array of masks for initiating servos  

const static uint8_t servoOutMask[N_SERVOS] = 
{
	0b0000010, // PX0  		
	0b0000100, // PX2  		
	0b0010000, // PX4
};

void servoSet(uint8_t servo, uint16_t time /* 탎 */)
{
    uint16_t ticks = US2TIMER1(time);				//(16*600=8000)//90 degree=(16*668=10688)
    cli();
    servoTime[servo] = ticks;
    sei();
}

void setServoSpeed(int portNumber,unsigned int speed)
{
	_delay_ms(speed*10);
}
/*
void setServoPosition(int portNumber,int angle)
{
	/*for(int i=0;i<=100;i+5){
		if(speed>=i && speed<=i+4)
			delay=60-i/5;
	}
	
	uint16_t time = 0;
	
	signed int check = last_angle[portNumber]-angle;
	
	if(check>0)
	{
		for(int zx = last_angle[portNumber];zx!=angle;zx--)
		{
			time =  500+(int)(2500/180)*angle;
			servoSet(portNumber,time);
		}
	}
	else
	{
		for(int zx = last_angle[portNumber];zx!=angle;zx++)
		{
			time =  600+(int)(1600/180)*angle;
			servoSet(portNumber,time);
		}
	}
	
	last_angle[portNumber]=angle;   

}

*/

void setServoPosition(int portNumber,int Angle){
	uint16_t time = 0;
	time =  500+(int)(2500/180)*Angle;
	
	servoSet(portNumber,time);     
}


//------------------------------------------------------------------------//
//////////////////	ISR CONTROLLING THE SERVO /////////////////////////////////
//------------------------------------------------------------------------//

ISR(TIMER1_COMPA_vect)
{
	
   static uint16_t nextStart;
    static uint8_t servo;
    static bool outputHigh = true;
    uint16_t currentTime = OCR1A;				//initially 1600
    uint8_t mask = servoOutMask[servo];
  
    if (outputHigh) 
	{
        SERVO_PORT |= mask;
		
		// Set the end time for the servo pulse
        OCR1A = currentTime + servoTime[servo];				//(1600 + 8000)//(9600+10688=60288)
        
		nextStart = currentTime + US2TIMER1(SERVO_TIME_DIV);		//9600-60000//60288-60000
		
    } 
	else 
	{
       SERVO_PORT &= ~mask;
		if (servo++ == N_SERVOS) 
		{
            servo = 0;
        }
        OCR1A = nextStart;
    }
   outputHigh = !outputHigh;
}


//------------------------------------------------------------------------//
//////////////////////////	SERVO STARTUP	/////////////////////////////////
//------------------------------------------------------------------------//
void servoStart(void)
{	
	//SERVO_DDR |= SERVO_MASK; 
	//SERVO_DDR |= 15;
	SERVO_DDR |= 0b1111111;
    // Setupt a first compare match
    OCR1A = TCNT1 + US2TIMER1(100);
    // start timer 1 with no prescaler
    TCCR1B = (1 << CS10);			//0.0625us 1 clock cycle
    // Enable interrupt
    TIMSK |= (1 << OCIE1A);
}

//------------------------------------------------------------------------//
//////////////////	INITIALISATION OF SERVO		///////////////////////////
//------------------------------------------------------------------------//

void servoinit(void)
{
	setServoPosition(0,90);	
	setServoPosition(1,90);	
	setServoPosition(2,90);	
}


void servoinit1(void)
{
	//set_scara_speed(7,7,7);
	while(1){
		if(LastDegree0<CurrentDegree0){
		
			uart_putc('a');
			setServoPosition(0,LastDegree0++);
		}
		if(LastDegree0>CurrentDegree0){
		
			uart_putc('b');
			setServoPosition(0,LastDegree0--);
		}
		_delay_ms(MyDelay0);
		if(LastDegree0>=CurrentDegree0) { uart_puts("break");break;}

	}
	
	
	ne:
	while(1){
		if(LastDegree1<CurrentDegree1){
		
			setServoPosition(1,LastDegree1++);
		}
		if(LastDegree1>CurrentDegree1){
		
			setServoPosition(1,LastDegree1--);
		}
		_delay_ms(MyDelay1);
		if(LastDegree1>=CurrentDegree1) { uart_puts("break1");break;}

	}
	while(1){
		if(LastDegree2<CurrentDegree2){
		
			setServoPosition(2,LastDegree2++);
		}
		if(LastDegree2>CurrentDegree2){
		
			setServoPosition(2,LastDegree2--);
		}
		_delay_ms(MyDelay2);
		if(LastDegree2>=CurrentDegree2) { uart_puts("break1");break;}

	}
	
	LastDegree2=CurrentDegree2;
	LastDegree1=CurrentDegree1;
	LastDegree0=CurrentDegree0;
	
	//setServoPosition(0,30);	
	//setServoPosition(1,30);	
	//setServoPosition(2,30);	
	_delay_ms(1000);
	
	
}

void InitMotor(){
	
	sbi(DDRB,2);// PWMA
	sbi(DDRB,1);// PWMB
	cbi(PORTB,2);
	cbi(PORTB,1);
	
	sbi(DDRD,6);// Strobe  pin
	sbi(PORTD,6);// pull up pin
	
	sbi(DDRD,4);// input 1A
	sbi(DDRD,5);// input 1B
	sbi(DDRD,7);// input 2A
	sbi(DDRB,0);// input 2B
	
	// stop condition for motors
	cbi(PORTD,4);// input 1A
	cbi(PORTD,5);// input 1B
	cbi(PORTD,7);// input 2A
	cbi(PORTB,0);// input 2B
	
/*	sbi(DDRD,4);// enable pin for DC motor
	sbi(DDRD,5);// enable pin for DC motor
	
	sbi(PORTD,4);
	sbi(PORTD,5);
	
	sbi(DDRD,6);
	sbi(DDRD,7);
	sbi(DDRD,2);
	sbi(DDRD,3);
	
	cbi(PORTD,2);
	cbi(PORTD,3);
	cbi(PORTD,6);
	cbi(PORTD,7);
*/	
	
	Set_pwm();
	uart_puts("PWM  initialized");
	uart_puts("\n");
	uart_puts("\r");
		
}

int readADC(int channel)
{

	ADMUX=channel;
	
	ADMUX|=(1<<ADLAR)|(1<<REFS0);	//supply vcc
	
	ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADEN)|(1<<ADSC);	//scaling factor=8 ,adc enable,start conversion

	while(!(ADCSRA & (1<<ADIF)));
	
	ADCSRA|=(1<<ADIF);
	
	ADCSRA &= ~(1<<ADEN);
	
	return ADCH;

}


//------------------------------------------------------------------------//
//////////////////////		MAIN FUNCTION		////////////////////////////////
//------------------------------------------------------------------------//

int main(void)
{
	
	wdt_disable();
	uart_init( UART_BAUD_SELECT(9600,F_CPU) ); 
	sei();
	
	uart_puts("start");
	uart_puts("\n");
	uart_puts("\r");
	
	// waits(100);
	
	DDRC |= (1<<PC2)|(1<<PC4)|(1<<PC1);	// initialize servo
	
	sbi(DDRD,4);// enable pin for DC motor
	sbi(DDRD,5);// enable pin for DC motor
	
	sbi(PORTD,4);
	sbi(PORTD,5);
	
	servoStart();
	servoinit();
	//while(1);
	//servoinit1();
	InitMotor();                        
	
	uart_puts("Enter command...");
	uart_puts("\n");
	uart_puts("\r");

		//waits(3000);
	
	while(1)
	{
		get_serial_data();
	}
	return 0;

}

void get_serial_data(void)
{

	unsigned int c=0;
	
	c = uart_getc();
	
	if(c & UART_NO_DATA){
		c=0;
	}
	else{
		//  if sync is recieved
		if(c==255){
			
		}		
		//  start storing packet if sync received
		else{
			
			if(rxlength1==3){
				serial_data[rxlength1++] = c;
				
				decode_data(serial_data);
				serial_data[rxlength1] = 0;
				c=rxlength1;
				//softuart_puts(itoa(rxlength1,buff,10));
				rxlength1=0;
				Start1=0;
				
				return ;
			}
			
			else{
				//softuart_puts(itoa(rxlength1,buff,10));
				serial_data[rxlength1++] = c;
				
			}
		}
	}
	
	return ;
	
	
}

/*
void decode_data(){
	
	if(serial_data[0]==100){// command for speed setting of servo
		
		
		MyDelay0=serial_data[1];
		MyDelay1=serial_data[2];
		MyDelay2=serial_data[3];
		
		//set_scara_speed(serial_data[1],serial_data[2],serial_data[3]);
		
	}
	
	if(serial_data[0]==101){// command for angle setting of servo
		
		
		uart_puts("am here");
		CurrentDegree0=serial_data[1];
		CurrentDegree1=serial_data[2];
		CurrentDegree2=serial_data[3];
		
		while(1){
			if(LastDegree0<CurrentDegree0){
			
				setServoPosition(0,LastDegree0++);
			}
			if(LastDegree0>CurrentDegree0){
			
				setServoPosition(0,LastDegree0--);
			}
			_delay_ms(MyDelay0);
			if(LastDegree0==CurrentDegree0) { uart_puts("break1");break;}

		}
		
		while(1){
			if(LastDegree1<CurrentDegree1){
			
				setServoPosition(1,LastDegree1++);
			}
			if(LastDegree1>CurrentDegree1){
			
				setServoPosition(1,LastDegree1--);
			}
			_delay_ms(MyDelay1);
			if(LastDegree1==CurrentDegree1) { uart_puts("break1");break;}

		}
		
		while(1){
			if(LastDegree2<CurrentDegree2){
			
				setServoPosition(2,LastDegree2++);
			}
			if(LastDegree2>CurrentDegree2){
			
				setServoPosition(2,LastDegree2--);
			}
			_delay_ms(MyDelay2);
			if(LastDegree2==CurrentDegree2) { uart_puts("break1");break;}

		}
		//setServoPosition(0,serial_data[1]);	
		//setServoPosition(1,serial_data[2]);	
		//setServoPosition(2,serial_data[3]);	
		//_delay_ms(1000);
		//move_scara_angle(serial_data[1],serial_data[2],serial_data[3]);
	}
	
	if(serial_data[0]==200){// to set direction of DC motor
	
		
		int drive_motor=serial_data[1];// motor to drive
		int DirectionOfMotor=serial_data[2];// direction of motor
		
		if(drive_motor==1){
			uart_puts("motor1");
			motor1=1;
			if(DirectionOfMotor==1){
				sbi(PORTD,2);// input 1A
				cbi(PORTD,3);// input 1B
			}
			if(DirectionOfMotor==2){
				cbi(PORTD,2);// input 1A
				sbi(PORTD,3);// input 1B
			}
			if(DirectionOfMotor==0){
				cbi(PORTD,2);// input 1A
				cbi(PORTD,3);// input 1B
			}
		}
		if(drive_motor==2){
		uart_puts("motor2");
			motor2=1;
			if(DirectionOfMotor==1){
				sbi(PORTD,7);// input 2A
				cbi(PORTD,6);// input 2B
			}
			if(DirectionOfMotor==2){
				cbi(PORTD,7);// input 2A
				sbi(PORTD,6);// input 2B
			}
			if(DirectionOfMotor==0){
				cbi(PORTD,7);// input 2A
				cbi(PORTD,6);// input 2B
			}
		}
	}
	
	if(serial_data[0]==201){// to set speed of DC motor	
		dutycycle=serial_data[1];
	}
	
}

*/

void decode_data(){
	
	if(serial_data[0]==100){// command for speed setting of servo
		
		
		MyDelay0=serial_data[1];
		MyDelay1=serial_data[2];
		MyDelay2=serial_data[3];
		
		//set_scara_speed(serial_data[1],serial_data[2],serial_data[3]);
		
	}
	
	if(serial_data[0]==101){// command for angle setting of servo
		
		servoStart();
		servoinit();
		
		uart_puts("Servo initialized...");
		uart_puts("\n");
		uart_puts("\r");
		
		uart_puts("setting servo...");
		uart_puts("\n");
		uart_puts("\r");
		
		CurrentDegree0=serial_data[1];
		CurrentDegree1=serial_data[2];
		CurrentDegree2=serial_data[3];
		
		setServoPosition(0,CurrentDegree0);
		//_delay_ms(1000);
		
		setServoPosition(1,CurrentDegree1);
		//_delay_ms(1000);
		
		setServoPosition(2,CurrentDegree2);
		//_delay_ms(1000);
		
		
		/*
		while(1){
			if(LastDegree0<CurrentDegree0){
			
				setServoPosition(0,LastDegree0++);
			}
			if(LastDegree0>CurrentDegree0){
			
				setServoPosition(0,LastDegree0--);
			}
			_delay_ms(MyDelay0);
			if(LastDegree0==CurrentDegree0) { uart_puts("break1");break;}

		}
		
		while(1){
			if(LastDegree1<CurrentDegree1){
			
				setServoPosition(1,LastDegree1++);
			}
			if(LastDegree1>CurrentDegree1){
			
				setServoPosition(1,LastDegree1--);
			}
			_delay_ms(MyDelay1);
			if(LastDegree1==CurrentDegree1) { uart_puts("break1");break;}

		}
		
		while(1){
			if(LastDegree2<CurrentDegree2){
			
				setServoPosition(2,LastDegree2++);
			}
			if(LastDegree2>CurrentDegree2){
			
				setServoPosition(2,LastDegree2--);
			}
			_delay_ms(MyDelay2);
			if(LastDegree2==CurrentDegree2) { uart_puts("break1");break;}
		
		}
		//setServoPosition(0,serial_data[1]);	
		//setServoPosition(1,serial_data[2]);	
		//setServoPosition(2,serial_data[3]);	
		//_delay_ms(1000);
		//move_scara_angle(serial_data[1],serial_data[2],serial_data[3]);
	
	*/
	
	}
	
	if(serial_data[0]==200){// to set direction of DC motor
	
		uart_puts("Entered in Motor control wizard");
		uart_puts("\n");
		uart_puts("\r");
		
		int drive_motor=serial_data[1];// motor to drive
		int DirectionOfMotor=serial_data[2];// direction of motor
		
		if(drive_motor==1){
			uart_puts("motor1");
			uart_puts("\n");
			uart_puts("\r");
		
			motor1=1;
			if(DirectionOfMotor==1){
				sbi(PORTD,4);// input 1A
				cbi(PORTD,5);// input 1B
			}
			if(DirectionOfMotor==2){
				cbi(PORTD,4);// input 1A
				sbi(PORTD,5);// input 1B
			}
			if(DirectionOfMotor==0){
				cbi(PORTD,4);// input 1A
				cbi(PORTD,5);// input 1B
			}
			uart_puts("motor1 set");
			uart_puts("\n");
			uart_puts("\r");
		}
		if(drive_motor==2){
		uart_puts("motor2");
		uart_puts("\n");
		uart_puts("\r");
		
			motor2=1;
			if(DirectionOfMotor==1){
				sbi(PORTD,7);// input 2A
				cbi(PORTB,0);// input 2B
			}
			if(DirectionOfMotor==2){
				cbi(PORTD,7);// input 2A
				sbi(PORTB,0);// input 2B
			}
			if(DirectionOfMotor==0){
				cbi(PORTD,7);// input 2A
				cbi(PORTB,0);// input 2B
			}
			uart_puts("motor2 set");
			uart_puts("\n");
			uart_puts("\r");
		}
	}
	
	if(serial_data[0]==201){// to set speed of DC motor	
		
		uart_puts("motor speed setting wizard...");
		uart_puts("\n");
		uart_puts("\r");
		
		dutycycle=serial_data[1];
		
		uart_puts("done...");
		uart_puts("\n");
		uart_puts("\r");
		
	}
	
	if(serial_data[0]==202){// to set speed of DC motor	
		
		uart_puts("ADC initialized...");
		uart_puts("\n");
		uart_puts("\r");
		
		i=readADC(1);
		itoa(i,str,10);
		
		uart_puts("Sensor value is...");
		uart_puts("\n");
		uart_puts("\r");
		
		uart_puts(str);
		
		uart_puts("\n");
		uart_puts("\r");
		
	}
	
	
}





/*
void decode_angle_data(void)
{
	
	unsigned int servo_1_angle=serial_data[0];
	unsigned int servo_2_angle=serial_data[1];
	unsigned int servo_3_angle=serial_data[2];
	
	move_scara_angle(servo_1_angle,servo_2_angle,servo_3_angle);
}*/
/*
void move_scara_angle(unsigned int servo_1_angle,unsigned int servo_2_angle,unsigned int servo_3_angle)

{
	char buff[10]={0};
	
	uart_puts(itoa(servo_1_angle,buff,10));
	uart_putc(',');
	uart_puts(itoa(servo_2_angle,buff,10));
	uart_putc(',');
	uart_puts(itoa(servo_3_angle,buff,10));
	
	setServoPosition(0,100);
	setServoPosition(1,100);
	setServoPosition(2,100);
	
	_delay_ms(1000);
	_delay_ms(1000);
}*/
/*
void decode_speed_data(void)
{
	unsigned int servo_1_speed=serial_data[0];
	unsigned int servo_2_speed=serial_data[1];
	unsigned int servo_3_speed=serial_data[2];
	
	set_scara_speed(servo_1_speed,servo_2_speed,servo_3_speed);
}*/

void set_scara_speed(unsigned int servo_1_speed,unsigned int servo_2_speed,unsigned int servo_3_speed)
{
	setServoSpeed(0,servo_1_speed);
	setServoSpeed(1,servo_2_speed);
	setServoSpeed(2,servo_3_speed);
}