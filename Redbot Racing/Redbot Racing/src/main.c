#include <avr/interrupt.h>
#define F_CPU 16000000UL /* Tells the Clock Freq to the Compiler. */
#include <avr/io.h> /* Defines pins, ports etc. */
#include <stdio.h>
#include "PIDController.h"

PIDController motorRatioController;

volatile int controllerTimer = 0.0;
volatile float motorControllerSetpoint = 230.0;
volatile float CONTROLLER_GAIN = 1.0;
volatile float CONTROLLER_INTEGRAL_TIME = 1.0; //seconds
volatile float CONTROLLER_DERIVATIVE_TIME = 1.0; //seconds
volatile float CONTROLLER_MIN_OUTPUT = -90.0;
volatile float CONTROLLER_MAX_OUTPUT = 90.0;
volatile float CONTROLLER_SAMPLING_PERIOD = 0.001;
volatile float INITIAL_CONTROLLER_OFFSET = 0.0;

#define Left_PWM 5
#define Right_PWM 6

#define Left_Mode_1 2
#define Right_Mode_1 7

#define Left_Mode_2 4
#define Right_Mode_2 0

#define Motor_DDR DDRD
#define Motor_Bank PORTD
#define Motor_DDR2 DDRB
#define Motor_Bank2 PORTB

volatile uint16_t Left_time_period = 2;
volatile uint16_t Left_duty_cycle = 0;

volatile uint16_t Right_time_period = 319;
volatile uint16_t Right_duty_cycle = 317;	//0-317 (Highest Duty Ratio)


void InitTimer1();
void initADC();

void inits(void)
{
		//Set up Data Direction Registers
		Motor_DDR |= (1<<Left_PWM)|(1<<Right_PWM)|(1<<Left_Mode_1)|(1<<Right_Mode_1)|(1<<Left_Mode_2);
		Motor_DDR2 |=(1<<Right_Mode_2);

		//Set up Timer 0 for PWM at about 50kHz
		TCCR0A |= (1<<WGM01)|(1<<WGM00)|(1<<COM0B1);//Fast PWM Mode
		OCR0B = 254; //Duty ratio currently at max value 0-255
		TIMSK0 |= (1<<OCIE0B);
		TCCR0B |= (1<<CS00);//prescalar of 1

		//Set up Timer 1 for Right Motor PWM
		TCCR1A |= (1<<WGM10)|(1<<WGM11);		//Fast PWM
		TCCR1B |= ((1<<WGM12)|(1<<WGM13)|(1<<CS10));		// Prescalar = 1
		TIMSK1 |= ((1<<OCIE1B)|(1<<OCIE1A));
		OCR1A = Right_time_period;
		OCR1B = Right_duty_cycle;


		//Set up Timer 2 as a 1ms clock
		TCCR2A |= (1<<WGM21);	//CTC Mode
		OCR2A = 249;
		TIMSK2 |= (1<<OCIE2A);
		TCCR2B |= (1<<CS22);  //64 prescalar */
/*
		InitTimer1();
		initADC();
		motorRatioController = PIDControllerCreate(motorControllerSetpoint,
		CONTROLLER_GAIN, CONTROLLER_INTEGRAL_TIME, CONTROLLER_DERIVATIVE_TIME,
		CONTROLLER_MIN_OUTPUT,CONTROLLER_MAX_OUTPUT, CONTROLLER_SAMPLING_PERIOD,
		INITIAL_CONTROLLER_OFFSET);
*/
		sei();


}

float readAnalogVoltage(){
	int adcIn = (ADCL);
	adcIn |= ( ADCH << 8 );
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1<<ADSC)));
	float voltage = ((1.0*(float)adcIn) / 1024.0) * 5.0;
}

ISR(TIMER2_COMPA_vect) {
	controllerTimer++;
	if(controllerTimer > motorRatioController.samplingPeriod * 1000){
		PIDControllerComputeOutput(&motorRatioController, readAnalogVoltage());
		controllerTimer = 0;
	}
}

void initADC(){
	//init the A to D converter
	ADMUX = 0b00000110;
	ADMUX |= (1<<MUX2)|(1<<MUX1);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (ADPS0);
}

// 1ms ISR for Timer 1 assuming F_CPU = 16MHz
void InitTimer1(void) {
	TCCR1A |= (1<<COM1A1);
	OCR1A = 249;
	TIMSK1 |= (1<<OCIE1A);
	TCCR1B |= (1 << WGM12)|(1<<CS11)|(1<<CS10);
}


ISR(TIMER0_COMPB_vect)
{
	//OCR0B = Left_duty_cycle;
}
ISR (TIMER1_COMPA_vect)
{
	//OCR2B = Right_duty_cycle;
	Motor_Bank |= (1<<Right_PWM);
}

ISR (TIMER1_COMPB_vect)
{
	Motor_Bank &= ~(1<<Right_PWM);
}

void setSpeeds(float error)
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
	Motor_Bank &= ~((1<<Left_Mode_1)|(1<<Left_Mode_2)|(1<<Right_Mode_1));	//put both motors in stop mode
	Motor_Bank2 &= ~(1<<Right_Mode_2);
	Left_duty_cycle = 0;
	Right_duty_cycle = 0;
}

void motorForward()
{
	Motor_Bank |= (1<<Left_Mode_2)|(1<<Right_Mode_1);	//put both motors in forward mode
	Motor_Bank &= ~(1<<Left_Mode_1);	//put both motors in forward mode
	Motor_Bank2 &= ~(1<<Right_Mode_2);
}

void rightForward()
{
	Motor_Bank |= (1<<Right_Mode_1);
	Motor_Bank2 &= ~(1<<Right_Mode_2);
}

void leftForward()
{
	Motor_Bank |= (1<<Left_Mode_1);
	Motor_Bank &= ~(1<<Left_Mode_2);
}

void initMotors()
{
	motorForward();
	Left_duty_cycle = 0;//Left_time_period/5;
	Right_duty_cycle = 0;//Right_time_period/5;		//MAX SPEED
}

void leftBrake()
{
	Motor_Bank|=(1<<Left_Mode_1)|(1<<Left_Mode_2);
	Left_duty_cycle = 0;
}

void rightBrake()
{
	Motor_Bank|=(1<<Right_Mode_1)|(1<<Right_Mode_2);
	Right_duty_cycle = 0;
}

void leftReverse()
{
	Motor_Bank &= ~((1<<Left_Mode_1));
	Motor_Bank |= (1<<Left_Mode_2);		//put left motor in reverse mode
	//keep left PWM the same
}

void rightReverse()
{
	Motor_Bank &= ~((1<<Right_Mode_1));
	Motor_Bank |= (1<<Right_Mode_2);		//put right motor in reverse mode
	//keep right PWM the same
}

void Reverse()
{
	//put both motors in reverse mode but keep speed the same for now
	Motor_Bank &= ~((1<<Left_Mode_1)|(1<<Right_Mode_1));
	Motor_Bank |= (1<<Left_Mode_2)|(1<<Right_Mode_2);
}

int main(void)
{
	inits();
	initMotors();
	while(1);

	return 0;
}
