#include<avr/io.h>
#include<util/delay.h>

void InitADC()
{
	ADMUX=(1<<REFS0);                                  	// For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //Prescalar div factor =128

}


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


void main()
{
	//Initialize Analog to Digital Converter (ADC)
	InitADC();
	DDRA=0x00;
	DDRD=0xff;
	DDRB=0xff;
	
	while(1)
	{
		//Previous Sensor Reading

		float sprev;
		
		//Take current sensor reading
		//return value is between 0 to 5
		//When the line is towards right of center then value tends to 5
		//When the line is towards left of center then value tends to 1
		//When line is in the exact center the the value is 3

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

//Implementing PID control

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
	int s1=0, s2=0, s3=0;
	
	DDRA=0x00;
    DDRB=0xff;
	DDRD=0xff;
	while(1)
    {
	   s1=(PINA&0x01);
	   s2=(PINA&0x02);
	   s3=(PINA&0x04);
       

	if ((s1==0x00)&&(s2==0x00)&&(s3==0x00))
	{ 
	 PORTB=0x01;//stop
	 PORTD=0x00;
	 s1=0;
	 s2=0;
	 s3=0;
	}
	 if ((s1==0x01)&&(s2==0x00)&&(s3==0x00))
	{
		PORTB=0x02;//turn right
		PORTD=0x0c;
		s1=0;
		s2=0;
		s3=0;
	}
	
	 if((s1==0x00)&&(s2==0x00)&&(s3==0x04))
	{
		PORTB=0x04;//turn left
		PORTD=0x03;
		s1=0;
		s2=0;
		s3=0;
	}
	if((s1==0x01)&(s2==0x00)&&(s3==0x04))
	{
		PORTB=0x08;//forward
		PORTD=0x0f;
	s1=0;
	s2=0;
	s3=0;
	}
	if((s1==0x01)&(s2==0x02)&&(s3==0x04))
	{
	PORTB=0x10;//stop
	PORTD=0x00;
	s1=0;
	s2=0;
	s3=0;
	}
	else
	PORTB=0x00;
		}
}
