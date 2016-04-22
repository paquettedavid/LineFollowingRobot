#define F_CPU 16000000UL /* Tells the Clock Freq to the Compiler. */
#include <avr/io.h> /* Defines pins, ports etc. */
#include <interrupt.h>
#include <stdio.h>

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

float readAnalogVoltage(){
	int adcIn = (ADCL);
	adcIn |= ( ADCH << 8 );
	readADCandWait();
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1<<ADSC)));
	float voltage = ((1.0*(float)adcIn) / 1024.0) * 5.0;
}

ISR(TIMER1_COMPA_vect) {
	controllerTimer++;
	if(controllerTimer > motorRatioController.samplingPeriod * 1000){
		PIDControllerComputeOutput(&motorRatioController, readAndConvertAnalogVoltage());
		controllerTimer = CLEAR;
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

void initializeAll(){
	initPWMTimer1(1000, 64);
	initADC();
	DDRD = 0b00100111;
	smokeTemperatureController = PIDControllerCreate(motorControllerSetpoint,
            CONTROLLER_GAIN, CONTROLLER_INTEGRAL_TIME, CONTROLLER_DERIVATIVE_TIME,
			CONTROLLER_MIN_OUTPUT,CONTROLLER_MAX_OUTPUT, CONTROLLER_SAMPLING_PERIOD,
			INITIAL_CONTROLLER_OFFSET
	);
	sei();
}

void main() {
	initializeAll();
	while(1);
}
