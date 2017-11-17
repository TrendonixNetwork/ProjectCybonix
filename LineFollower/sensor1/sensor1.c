#include<avr/io.h>
#include<util/delay.h>

void main()

{

int s0=0,s1=0, s2=0, s3=0;

DDRB=0xff;
DDRA=0x00;


while(1)
    {
PORTA= (1<<PA7);
	   s0=(PINA&0x01);
	   s1=(PINA&0x02);
	   s2=(PINA&0x04);
       s3=(PINA&0x08);

	/*if ((s1==0x00)&&(s2==0x00)&&(s3==0x00)&&(s4==0x00))
	{ 
	 PORTB=0x00;//stop
	 s1=0;
	 s2=0;
	 s3=0;
	 s4=0;
	}*/

	 if ((s0==0x01)&&(s1==0x02)&&(s2==0x04)&&(s3==0x08))
	{
		PORTB=0xff;//turn right
		s0=0;
		s1=0;
		s2=0;
		s3=0;
	}
	

	 /*if((s1==0x01)&&(s2==0x02)&&(s3==0x04)&&(s4==0x00))
	{
		PORTD=0x08;//turn left
		s1=0;
		s2=0;
		s3=0;
		s4=0;
	}


	if((s1==0x00)&&(s2==0x02)&&(s3==0x04)&&(s4==0x00))
	{
		PORTD=0x09;//forward
	s1=0;
	s2=0;
	s3=0;
	s4=0;
	}

	/*if((s1==0x01)&(s2==0x02)&&(s3==0x04))
	{
	PORTB=0x08;
	s1=0;
	s2=0;
	s3=0;
	}*/

	else

	PORTB=0x00;

		}
}
