
#define BOTH_MOTORS 0
#define LEFT_MOTOR 1
#define RIGHT_MOTOR 2

#define CLKWISE 1
#define ANTICLKWISE 2
#define STOP 0


int dutycycle=50;

long int actual_speed_L=2400;
long int actual_speed_R=2300;
long int actual_speed=1;

volatile int motor1_for=0;
volatile int motor1_rev=0;

volatile int motor2_for=0;
volatile int motor2_rev=0;
volatile int motor1=0;
volatile int motor2=0;
volatile unsigned int count=0;


void start_motor(int motor,int direction){
	
	if(motor==LEFT_MOTOR){
		
		
		Init_Pwm(LEFT_MOTOR);
		
		if(direction==CLKWISE){
			
			sbi(PORTC,4);
		}
		if(direction==ANTICLKWISE){
			
			cbi(PORTC,4);
		}		
		if(direction==STOP){
			
			OCR1A=9999;
			
		}
	}
	
	if(motor==RIGHT_MOTOR){
		
		Init_Pwm(RIGHT_MOTOR);
		if(direction==CLKWISE){
			
			sbi(PORTC,5);
		}
		if(direction==ANTICLKWISE){
			
			cbi(PORTC,5);
		}		
		if(direction==STOP){
			OCR1B=9999;
			//TIMSK &= ~(1<<OCIE1B);
		}
	}
	

	
	if(motor==BOTH_MOTORS){
		
		start_motor(LEFT_MOTOR,direction);
		start_motor(RIGHT_MOTOR,direction);
	
	}
}


void Set_Speed(int motor, int speed){

	
	actual_speed=speed*80;
	
	actual_speed_L=actual_speed;
	actual_speed_R=actual_speed;;
	
	
	
	if(actual_speed>7000){
		actual_speed_L=7000;
		actual_speed_R=7000;
		
	}
	
	if(actual_speed<10){
		actual_speed_L=10;
		actual_speed_R=10;
	}

	
	
	if(motor==LEFT_MOTOR){
			
		OCR1A=actual_speed_L;	
	}
	
	if(motor==RIGHT_MOTOR){	
		OCR1B=actual_speed_R;
	}
	

	if(motor==BOTH_MOTORS){
		
		Set_Speed(LEFT_MOTOR,speed);
		Set_Speed(RIGHT_MOTOR,speed);
		
		
	}
}





void Set_pwm(){
  
	TCCR2|= (1<<WGM21);  //CTC mode
	//TCCR0|=(1<<COM00); //toggle
    
	TCCR2|= (1<<CS21);//|(1<<CS00);//prescale by 8
        
	TIMSK|=(1<<OCIE2);
	TCNT2=0;

	OCR2=184; // for 50hz it is 288 and then divide by 100 so 2..
	sbi(DDRB,2);
	
}

ISR(TIMER2_COMP_vect)
{

	if(count<dutycycle){
		
		
		if(motor1==1){
			sbi(PORTB,2);// PWm1 pin
			
		}
		if(motor2==1){
			sbi(PORTB,1);// PWm1 pin
			
		}
		
	}
	if(count>=dutycycle&&count<=100){
	
		if(motor2==1){
			cbi(PORTB,1);// PWm1 pin
			
		}
		if(motor1==1){
			cbi(PORTB,2);// PWm1 pin
			
		}
		
		
	}
	if(count>100){
		count=0;
		//cbi(PORTC,2);
		//cbi(PORTC,3);
	}
	count++;  
}

void Init_Pwm(int motor){
	
	if(motor==BOTH_MOTORS){
		
		//uart_putc('%');
		TCCR1A  |= ((1 << COM1A1));
		TCCR1A  |= ((1 << COM1B1));
	
		TIMSK|=(1<<TOIE1);          //TIMER INTERRUPT ENABLE
		
		OCR1A=actual_speed_L;
		OCR1B=actual_speed_R;
	}
	if(motor==LEFT_MOTOR){
		TCCR1A  |= ((1 << COM1A1));
		TIMSK|=(1<<TOIE1);          //TIMER INTERRUPT ENABLE
	
		OCR1A=actual_speed_L;
	}	
	if(motor==RIGHT_MOTOR){
	
		//uart_putc('^');
		TCCR1A  |= ((1 << COM1B1));
		TIMSK|=(1<<TOIE1);          //TIMER INTERRUPT ENABLE
		
		OCR1B=actual_speed_R;
	}
	
	
	TCCR1B  |=(1 << CS10);      // prescale by 8
	//TCCR0|=(1<<CS01)|(1<<CS00);// prescale by 64
	//TCCR2|= (1<<CS22);//prescale by 64
	
}