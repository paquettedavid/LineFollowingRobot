#include <avr/interrupt.h>
#define F_CPU 16000000UL /* Tells the Clock Freq to the Compiler. */
#include <avr/io.h> /* Defines pins, ports etc. */
#include <stdio.h>
#include "PIDController.h"
#include <stdlib.h>
PIDController motorRatioController;

volatile int controllerTimer = 0.0;
volatile float motorControllerSetpoint = 450.0;
volatile float CONTROLLER_GAIN = 0.07; // 0.07 for good line tracking at 0.55 turn ratio power
volatile float CONTROLLER_INTEGRAL_TIME = 0;//0.15; //seconds
volatile float CONTROLLER_DERIVATIVE_TIME = 0; //seconds
volatile float CONTROLLER_MIN_OUTPUT = -90.0;
volatile float CONTROLLER_MAX_OUTPUT = 90.0;
volatile float CONTROLLER_SAMPLING_PERIOD = 0.001;
volatile float INITIAL_CONTROLLER_OFFSET = 0.0;
volatile float direction = -1;
#define Left_PWM 5
#define Right_PWM 6

#define MIDDLE_SENSOR 0
#define LEFT_SENSOR 1
#define RIGHT_SENSOR 2


#define Left_Mode_1 2
#define Right_Mode_1 7

#define Left_Mode_2 4
#define Right_Mode_2 0

#define Motor_DDR DDRD
#define Motor_Bank PORTD
#define Motor_DDR2 DDRB
#define Motor_Bank2 PORTB
// straigt values : L = 150 R = 159
volatile uint16_t Left_time_period = 255;
volatile uint16_t Left_duty_cycle = 80; // 255 max

volatile uint16_t Right_time_period = 319;
volatile uint16_t Right_duty_cycle = 120;	//0-317 (Highest Duty Ratio)

#define TURN_POWER 55.0 //40 for line, 55 for arbitrary path

volatile float turnRatio = TURN_POWER;

void InitTimer1();
void initADC();
void setSpeeds(float error);

#define IR_m (1<<MUX1)|(1<<MUX2)|(1<<REFS0)
#define IR_l (1<<MUX1)|(1<<REFS0)
#define IR_r (1<<MUX0)|(1<<MUX1)|(1<<REFS0)

volatile uint16_t LeftSensor;
volatile uint16_t RightSensor;

volatile int adcChannel = 0;

void inits(void)
{
		//Set up Data Direction Registers
		Motor_DDR |= (1<<Left_PWM)|(1<<Right_PWM)|(1<<Left_Mode_1)|(1<<Right_Mode_1)|(1<<Left_Mode_2);
		Motor_DDR2 |=(1<<Right_Mode_2);

		//Set up Timer 0 for PWM at about 50kHz
		TCCR0A |= (1<<WGM01)|(1<<WGM00)|(1<<COM0B1);//Fast PWM Mode
		OCR0B = Left_duty_cycle; //Duty ratio currently at max value 0-255
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
		initADC();
		motorRatioController = PIDControllerCreate(motorControllerSetpoint,
		CONTROLLER_GAIN, CONTROLLER_INTEGRAL_TIME, CONTROLLER_DERIVATIVE_TIME,
		CONTROLLER_MIN_OUTPUT,CONTROLLER_MAX_OUTPUT, CONTROLLER_SAMPLING_PERIOD,
		INITIAL_CONTROLLER_OFFSET);
		sei();
}

ISR( ADC_vect ) {
	switch(adcChannel){
		case MIDDLE_SENSOR:
			ADMUX = 0;
			ADMUX |= IR_l;
			adcChannel = LEFT_SENSOR;
			break;
		case LEFT_SENSOR:
			ADMUX = 0;
			ADMUX|= IR_r;
			adcChannel = RIGHT_SENSOR;
			break;
		case RIGHT_SENSOR:
			ADMUX = 0;
			ADMUX|= IR_m;
			adcChannel = MIDDLE_SENSOR;
			break;
	}
}

float readAnalogVoltage(){

	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1<<ADSC)));
	int adcIn = (ADCL);
	adcIn |= ( ADCH << 8 );

	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1<<ADSC)));
	RightSensor = (ADCL);
	RightSensor |= ( ADCH << 8 );


	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1<<ADSC)));
	LeftSensor = (ADCL);
	LeftSensor |= ( ADCH << 8 );

	return adcIn;
}

ISR(TIMER2_COMPA_vect) {
	controllerTimer++;
}

void initADC(){
	//init the A to D converter
	ADMUX |= (1<<MUX1)|(1<<MUX2) |(1<< REFS0);
	ADCSRA = (1<<ADEN) | (1<<ADPS1)|(1<<ADIE);
}

ISR(TIMER0_COMPB_vect)
{
	OCR0B = Left_duty_cycle;
}
ISR (TIMER1_COMPA_vect)
{
	OCR1B = Right_duty_cycle;
	Motor_Bank |= (1<<Right_PWM);
}

ISR (TIMER1_COMPB_vect)
{
	Motor_Bank &= ~(1<<Right_PWM);
}

void setLeftMotorDutyCycle(float dutyCycle){
	Left_duty_cycle = (int)((float)(Left_time_period)*(dutyCycle/100.0));
}

void setRightMotorDutyCycle(float dutyCycle){
	Right_duty_cycle = (int)((304.0)*(dutyCycle/100.0));
}

void setSpeeds(float error)
{
	if(abs(error) < 11.2){
		if(turnRatio>0){
			turnRatio-=0.05;
		}
	} else if(abs(error) < 20){
		if(turnRatio<TURN_POWER) {
			turnRatio+=0.10;
		}
	} else {
		turnRatio = TURN_POWER;
	}
	setLeftMotorDutyCycle(((direction)*-1*error/90.0)*turnRatio+(100.0-turnRatio));
	setRightMotorDutyCycle(direction*(error/90)*turnRatio+(100.0-turnRatio));
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
	Motor_Bank2 |= (1<<Right_Mode_2);		//put right motor in reverse mode
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
	leftForward();
	rightReverse();
	while(1){
		if(controllerTimer > motorRatioController.samplingPeriod * 1000){
			float middleSensorValue = readAnalogVoltage();
			if(direction > 0 && LeftSensor > 700) direction *= -1;
			if(direction < 0 && RightSensor > 700) direction *= -1;
			PIDControllerComputeOutput(&motorRatioController, middleSensorValue);
			setSpeeds(motorRatioController.controllerOutput);
			controllerTimer = 0;
		}
	}
	return 0;
}
