
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <compat/deprecated.h>
#include <string.h>
#include <stdbool.h>
#include <servo_test.c>


void main()
{

//	DDRA=0xff;
	DDRC=0xff;
//	PORTA=0x00;
//	PORTC=0x00;
	
	
	while(1)
	{
	
		servoStart();
		
		setServoPosition(1,45);

		_delay_ms(4000);
	
	}

}