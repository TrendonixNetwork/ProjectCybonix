#include<avr/io.h>
#include<util/delay.h>


//#include<adc.h>
//#include<motor.h>
//#include<led.h>

void InitADC()
{
	ADMUX=(1<<REFS0);                                  	// For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //Prescalar div factor =128

}

/*

Function To Read ADC Channel

Argument: Channel Number between 0-7
Return Value : Between 0-1023

*/
uint16_t ReadADC(uint8_t ch)
{
	//Select ADC Channel ch must be 0-7
	ch=ch&0b00000111;
	ADMUX&=0b11100000;
	ADMUX|=ch;

	//Start Single conversion
	ADCSRA|=(1<<ADSC);

	//Wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));

	//Clear ADIF by writing one to it

	//Note you may be wondering why we have write one to clear it
	//This is standard way of clearing bits in io as said in datasheets.
	//The code writes '1' but it result in setting bit to '0' !!!

	ADCSRA|=(1<<ADIF);

	return(ADC);
}

#ifndef MOTOR_H
#define MOTOR_H

#define OC1A_PORT B
#define OC1A_PIN PB1

#define OC1B_PORT B
#define OC1B_PIN PB2

#define MOTOR_STOP 	0
#define MOTOR_CW	1
#define MOTOR_CCW	2

#define ROBO_SPEED 200 //0-255

void MotorInit();
void MotorA(uint8_t dir,uint8_t speed);
void MotorB(uint8_t dir,uint8_t speed);

#endif

/*

                                 DC Motor Core
						        
This module is used for controlling DC motors via the on board L298 driver IC
The library has functions to control both the direction and speed of DC motors.
The PWM speed controlled is done by the library internally.*/



#include <avr/io.h>
#include <avr/interrupt.h>

//#include "myutils.h"
//#include "motor.h"
#ifndef MOTOR_H
#define MOTOR_H

#define OC1A_PORT B
#define OC1A_PIN PB1

#define OC1B_PORT B
#define OC1B_PIN PB2

#define MOTOR_STOP 	0
#define MOTOR_CW	1
#define MOTOR_CCW	2

#define ROBO_SPEED 200 //0-255

void MotorInit();
void MotorA(uint8_t dir,uint8_t speed);
void MotorB(uint8_t dir,uint8_t speed);

#endif


#define XBOARD2
#ifndef MYUTILS_H
#define MYUTILS_H

#define _CONCAT(a,b) a##b
#define PORT(x) _CONCAT(PORT,x)
#define PIN(x) _CONCAT(PIN,x)
#define DDR(x) _CONCAT(DDR,x)

#endif
void MotorInit()
{
	#ifdef XBOARD2

	//set up pwm for speed control

	TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);

	//clk=fcpu/64

	TCCR1B=(1<<CS11)|(1<<CS10);

	//Set the corresponding port pin to output

	DDR(OC1A_PORT)|=(1<<OC1A_PIN);
	DDR(OC1B_PORT)|=(1<<OC1B_PIN);

	//Set the direction control PINs to OUT
	DDRD|=0X0F;	//PC0 to PC3 as output
	
	#endif
	
	#ifdef FIREBIRDV
	
	//Set up timer 5 for PWM
	
	TCCR5B = 0x00;	//Stop
	
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionalit to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
	
	//Set PWM Pins to out
	DDRL|=(1<<PL3)|(1<<PL4);
	
	//Set Direction Pins to output
	DDRA|=(0x0F);
	
	#endif
	

}

/**********************************************************
Descriptipn:
	Set the rotation of motor A(LEFT) in given direction and speed

dir = 0 for stop
dir = 1 for CW rotation
dir = 2 for CCW rotation
dir = 3 for brake

speed is any value between 0 and 255

Example:
	MotorA(MOTOR_CW,120);

Note:
	Other Constants for direction are
	MOTOR_CCW and MOTOR_STOP

***********************************************************/
void MotorA(uint8_t dir,uint8_t speed)//LEFT
{
	#ifdef XBOARD2
	//Direction
	if(dir == MOTOR_STOP)
	{
		PORTD&=(~(1<<PD0));
		PORTD&=(~(1<<PD1));
	}

	else if(dir == MOTOR_CCW)
	{
		PORTD&=(~(1<<PD0));
		PORTD|=(1<<PD1);
	}
	else if(dir == MOTOR_CW)
	{
		PORTD&=(~(1<<PD1));
		PORTD|=(1<<PD0);
	}

	//Speed
	uint8_t sreg=SREG;

	cli();

	OCR1A=(uint8_t)(((float)speed/255.0)*ROBO_SPEED);

	SREG=sreg;
	#endif
	
	#ifdef FIREBIRDV
	//Direction
	if(dir == MOTOR_STOP)
	{
		PORTA&=(~(1<<PA0));
		PORTA&=(~(1<<PA1));
	}

	else if(dir == MOTOR_CCW)//forword
	{
		PORTA&=(~(1<<PA0));
		PORTA|=(1<<PA1);
	}
	else if(dir == MOTOR_CW)//back
	{
		PORTA&=(~(1<<PA1));
		PORTA|=(1<<PA0);
	}

	//Speed
	OCR5AL=(uint8_t)(((float)speed/255.0)*ROBO_SPEED);
	#endif
	
}

/**********************************************************
Descriptipn:
	Set the rotation of motor B in given direction and speed

dir = 0 for stop
dir = 1 for CW rotation
dir = 2 for CCW rotation
dir = 3 for brake

speed is any value between 0 and 255

Example:
	MotorB(MOTOR_CW,120);

Note:
	Other Constants for direction are
	MOTOR_CCW and MOTOR_STOP

***********************************************************/
void MotorB(uint8_t dir,uint8_t speed)//RIGHT
{
	#ifdef XBOARD2
	//Direction
	if(dir == MOTOR_STOP)
	{
		PORTD&=(~(1<<PD2));
		PORTD&=(~(1<<PD3));
	}

	else if(dir == MOTOR_CCW)
	{
		PORTD&=(~(1<<PD2));
		PORTD|=(1<<PD3);
	}
	else if(dir == MOTOR_CW)
	{
		PORTD&=(~(1<<PD3));
		PORTD|=(1<<PD2);
	}

	//Speed
	uint8_t sreg=SREG;

	cli();

	OCR1B=(uint8_t)(((float)speed/255.0)*ROBO_SPEED);

	SREG=sreg;
	#endif
	
	#ifdef FIREBIRDV
	//Direction
	if(dir == MOTOR_STOP)
	{
		PORTA&=(~(1<<PA2));
		PORTA&=(~(1<<PA3));
	}

	else if(dir == MOTOR_CCW)//back
	{
		PORTA&=(~(1<<PA2));
		PORTA|=(1<<PA3);
	}
	else if(dir == MOTOR_CW)//forword
	{
		PORTA&=(~(1<<PA3));
		PORTA|=(1<<PA2);
	}

	//Speed
	OCR5BL=(uint8_t)(((float)speed/255.0)*ROBO_SPEED);
	#endif
}
	


/*#ifndef LED_H_
#define LED_H_

void LEDInit();

void LEDOff(uint8_t id);
void LEDOn(uint8_t id);




#endif /* LED_H_ */


#define SENSOR_THRES 800

//Map Sensor Number to ADC Channel
#define SENSOR1 0
#define SENSOR2 1
#define SENSOR3 2
#define SENSOR4 3
#define SENSOR5 4

//Gloabal varriables
float pGain = 200;	//Proportional Gain
float iGain =  0.2;	//Integral Gain
float dGain =  120;	//Differential Gain
int delay = 10;

int32_t eInteg = 0;	//Integral accumulator
int32_t ePrev  =0;		//Previous Error


void DelayMs(uint8_t ms);
float ReadSensors();
float PID(float cur_value,float req_value);


float control;
float s;


void LEDInit()
{
	//Make i/o pins as output type
	DDRD|=(0Xf0);
	DDRB|=(1<<PB0);
	
	//Make them high to turn off led.
	PORTD|=(0Xf0);
	PORTB|=(1<<PB0);
}

void LEDOff(uint8_t id)
{
	if(id>5) return;
	
	id=6-id;
	
	if(id==1)
	{
		PORTB|=(1<<PB0);
	}
	else if(id<=5)
	{
		PORTD|=(1<<(2+id));
	}
}

void LEDOn(uint8_t id)
{
	if(id>5) return;
	
	id=6-id;
	
	if(id==1)
	{
		PORTB&=~(1<<PB0);
	}
	else if(id<=5)
	{
		PORTD&=~(1<<(2+id));
	}
}
int main(void)
{
	//Initialize Motors subsystem.
	MotorInit();
	
	//Initialize LED subsystem
	LEDInit();
	
	//Initialize Analog to Digital Converter (ADC)
	InitADC();
	
	
	while(1)
	{
		//Previous Sensor Reading
		float sprev;
		
		//Take current sensor reading
		//return value is between 0 to 5
		//When the line is towards right of center then value tends to 5
		//When the line is towards left of center then value tends to 1
		//When line is in the exact center the the valeue is 3
		s=ReadSensors();
		
		//If line is not found beneath any sensor, use last sensor value.
		if(s==0xFF)
		{
			s=sprev;
		}
		
		//PID Algorithm generates a control variable from the current value
		//and the required value. Since the aim is to keep the line always
		//beneath the center sensor so the required value is 3 (second parameter)
		//The first argument is the current sensor reading.
		//The more the difference between the two greater is the control variable.
		//This control variable is used to produce turning in the robot.
		//When current value is close to required value is close to 0.
		control = PID(s,3.0);
		
		//Limit the control
		if(control > 510)
		control = 510;
		if(control < -510)
		control = -510;

		if(control > 0.0)//the left sensor sees the line so we must turn right
		{
			if(control>255)
			MotorA(MOTOR_CW,control-255);
			else
			MotorA(MOTOR_CCW,255-control);
			
			MotorB(MOTOR_CW,255);
		}
		if(control <= 0.0)//the right sensor sees the line so we must turn left
		{
			if(control<-255)
			MotorB(MOTOR_CCW,-(control+255));
			else
			MotorB(MOTOR_CW,255+control);
			
			MotorA(MOTOR_CCW,255);
		}
		
		//Delay
		DelayMs(delay);
		
		sprev=s;
	}
}

void DelayMs(uint8_t ms)
{
	uint8_t i;
	for(i=0;i<ms;i++)
	{
		_delay_ms(1);
	}
}

//Implements PID control
float PID(float cur_value,float req_value)
{
	float pid;
	float error;

	error = req_value - cur_value;
	pid = (pGain * error)  + (iGain * eInteg) + (dGain * (error - ePrev));

	eInteg += error;                  // integral is simply a summation over time
	ePrev = error;                    // save previous for derivative

	return pid;
}

float ReadSensors()
{
	uint16_t	eright,right,middle,left,eleft;
	uint8_t		sensor1,sensor2, sensor3, sensor4,sensor5;
	
	float avgSensor = 0.0;
	
	eright=ReadADC(SENSOR5);
	if(eright>SENSOR_THRES)//Right black line sensor
	{
		sensor5 = 1;
		LEDOn(5);
	}
	else
	{
		sensor5 = 0;
		LEDOff(5);
	}

	// Read analog inputs
	right=ReadADC(SENSOR4);
	if(right>SENSOR_THRES)//Right black line sensor
	{
		sensor4 = 1;
		LEDOn(4);
	}
	else
	{
		sensor4 = 0;
		LEDOff(4);
	}

	middle=ReadADC(SENSOR3);
	if(middle>SENSOR_THRES)// Middle black line sensor
	{
		sensor3 = 1;
		LEDOn(3);
	}
	else
	{
		sensor3 = 0;
		LEDOff(3);
	}

	left=ReadADC(SENSOR2);
	if(left>SENSOR_THRES)// Left black line sensor
	{
		sensor2 = 1;
		LEDOn(2);
	}
	else
	{
		sensor2 = 0;
		LEDOff(2);
	}
	
	eleft=ReadADC(SENSOR1);
	if(eleft>SENSOR_THRES)// Left black line sensor
	{
		sensor1 = 1;
		LEDOn(1);
	}
	else
	{
		sensor1 = 0;
		LEDOff(1);
	}
	
	
	if(sensor1==0 && sensor2==0 && sensor3==0 && sensor4==0 && sensor5==0)
	{
		return 0xFF;
	}
	
	// Calculate weighted mean
	avgSensor = (float) sensor1*1 + sensor2*2 + sensor3*3 + sensor4*4 + sensor5*5 ;
	avgSensor = (float) avgSensor / (sensor1 + sensor2 + sensor3 + sensor4 + sensor5);

	return avgSensor;
}


