#include <avr/interrupt.h>

#define Left_PWM PIND6	//change these
#define Right_PWM PIND5

#define Left_Mode_1 PIND1
#define Right_Mode_1 PIND2

#define Left_Mode_2 PIND3
#define Right_Mode_2 PIND4

volatile uint16_t Left_time_period = 2;
volatile uint16_t Left_duty_cycle = 0;

volatile uint16_t Right_time_period = 2;
volatile uint16_t Right_duty_cycle = 0;



void inits(void)
{
		//Set up Data Direction Registers
		DDRD |= (1<<Left_PWM)|(1<<Right_PWM)|(1<<Left_Mode_1)|(1<<Right_Mode_1)|(1<<Left_Mode_2)|(1<<Right_Mode_2);
	
		//Set up Timer 0 for PWM at about 50kHz
		TCCR0A |= (1<<WGM01)|(1<<WGM00)|(1<<WGM02)|(1<<COM0B1);	//Fast PWM Mode
		OCR0A = Left_time_period;		//set frequency to 50kHz
		OCR0B = Left_duty_cycle;		//D=0 at start for calibration
		TIMSK0 |= (1<<OCIE0A)|(1<<OCIE0B);
		TCCR0B = 5; //1024 prescalar
		
		//Set up Timer 2 for PWM at about 50kHz
		TCCR2A |= (1<<WGM21)|(1<<WGM20)|(1<<WGM22)|(1<<COM2B1);	//Fast PWM Mode
		OCR2A = Right_time_period;		//set frequency to 50kHz
		OCR2B = Right_duty_cycle;		//D = 0 at start for calibration
		TIMSK2 |= (1<<OCIE2A)|(1<<OCIE2B);
		TCCR2B = 5; //1024 prescalar
		
		sei();
		

}

ISR(TIMER0_COMPA_vect)
{
	OCR0B = Left_duty_cycle;
	PORTD |= (1<<Left_PWM);
}

ISR(TIMER1_COMPB_vect)
{
	PORTD &= ~(1<<Left_PWM);
}

ISR (TIMER2_COMPA_vect)
{
	OCR2B = Right_duty_cycle;
	PORTD |= (1<<Right_PWM);
}

ISR (TIMER2_COMPB_vect)
{
	PORTD &= ~(1<<Right_PWM);
}

void setSpeeds(int error)
{
	//assuming error -90 to 90 degrees
	if(error>0)
	{
		Left_duty_cycle =  (error/90)*Left_time_period;
		Right_duty_cycle = (((90-error))/90) *Right_time_period;
	}
	else if (error<0)
	{
		Left_duty_cycle = ((90+error)/90) *Left_time_period;
		Right_duty_cycle = (-1*error/90)*Right_time_period;
	}
	
}

void stopMotors()
{
	PORTD &= ~((1<<Left_Mode_1)|(1<<Left_Mode_2)|(Right_Mode_1)|(Right_Mode_2));	//put both motors in stop mode
	Left_duty_cycle = 0;
	Right_duty_cycle = 0;
}

void initMotors()
{
	PORTD |= (1<<Left_Mode_1)|(Right_Mode_1);	//put both motors in forward mode
	PORTD &= ~((1<<Left_Mode_2)|(Right_Mode_2));	//put both motors in forward mode
	Left_duty_cycle = Left_time_period;
	Right_duty_cycle = Right_time_period;		//MAX SPEED
}

void motorForward()
{
	PORTD |= (1<<Left_Mode_1)|(Right_Mode_1);	//put both motors in forward mode
	PORTD &= ~((1<<Left_Mode_2)|(Right_Mode_2));	//put both motors in forward mode
}

void leftBrake()
{
	PORTD|=(1<<Left_Mode_1)|(1<<Left_Mode_2);
	Left_duty_cycle = 0;
}

void rightBrake()
{
	PORTD|=(1<<Right_Mode_1)|(1<<Right_Mode_2);
	Right_duty_cycle = 0;
}

void leftReverse()
{
	PORTD &= ~((1<<Left_Mode_1));
	PORTD |= (1<<Left_Mode_2);		//put left motor in reverse mode
	//keep left PWM the same
}

void rightReverse()
{
	PORTD &= ~((1<<Right_Mode_1));
	PORTD |= (1<<Right_Mode_2);		//put right motor in reverse mode
	//keep right PWM the same
}

void Reverse()
{
	//put both motors in reverse mode but keep speed the same for now
	PORTD &= ~((1<<Left_Mode_1)|(1<<Right_Mode_1));
	PORTD |= (1<<Left_Mode_2)|(1<<Right_Mode_2);
}

int main(void)
{
	inits();
	while(1)
	{
		
	}
	
	return 0;
}