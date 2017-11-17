#include<avr/io.h>
#include<util/delay.h>

/* connections
 Vcc=+5v
 Trig=PA7
 Echo=PA0
 Ground=0v
*/

volatile uint32_t r=0;

void InitADC()   // initiating adc
{
	ADMUX=(1<<REFS0);                                  	// For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //Prescalar div factor =128

}

uint16_t adc_read(uint8_t ch)
{
  
  ch =ch& 0b00000111;  
  ADMUX|=ch; 
 
  // start single convertion
  // write ’1' to ADSC
  ADCSRA|=(1<<ADSC);
 
  while(!(ADCSRA & (1<<ADIF)));
 
	ADCSRA|=(1<<ADIF);

  return (ADC);
}




void send_trig() // function to send out signal through trig pin
{

	PORTA=0x00;
	_delay_us(5);

    PORTA=0xf0;
	r=1;
	_delay_us(10);
    PORTA=0x00;
	

}
void main()

{
	int s1=0;	

    DDRA = 0xf0;
	
	DDRB = 0xff;
	
	uint16_t adc_result0,adc_result1;
    
  
    InitADC();
	

	while(1)
	{

	s1=(PINA&0x01);

	adc_result0 = adc_read(0);      // read adc value at PA0
    adc_result1 = adc_read(7);      // read adc value at PA7
	

	 if(r==0)
	 {	
	
	_delay_ms(60);
	send_trig();
	
	if(s1==0x01)
	{
	PORTB=0b00100000;
	s1=0;
	}
	else
	PORTB=0b10000000;
	}
	}
}
