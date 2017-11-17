#include<avr/io.h>
#define F_CPU 1000000UL
#include<util/delay.h>

#define lcd PORTB
#define rs  0
#define rw  1
#define en  2
/*#define TRIG_PORT   PORTD
#define TRIG_BIT    PD0
#define TRIG_LOW() TRIG_PORT &= ~(1<<TRIG_BIT)
#define TRIG_HIGH() TRIG_PORT |=(1<<TRIG_BIT)*/

#define out_of_range 50//select any value except(0-20)

void lcd_init();
void lcd_command(unsigned char);
void lcd_data(unsigned char);
void lcd_string(unsigned char *str);
void lcd_number(unsigned int);
void adc_init();
unsigned int adc_read(unsigned char);
int main()
{	
	DDRA = 0x00;
	DDRB = 0XFF;
	DDRD = 0xFF;
	
	unsigned int dis=0;

	lcd_init();
	adc_init();
	//trigger();
	while(1)
	{
	lcd_command(0x80);
	lcd_string("Distance:       ");
	dis=adc_read(0);

	/*
	20cm adc value is 60
	15cm adc value is 85
	10cm adc value is 120
	5cm adc value is 280
	0cm adc value is 970
	*/

	if((dis>=60) && (dis<65))
	dis=20;
	else if((dis>=65) && (dis<70))
	dis=19;
	else if((dis>=70) && (dis<75))
	dis=18;
	else if((dis>=75) && (dis<80))
	dis=17;
	else if((dis>=80) && (dis<85))
	dis=16;
	else if((dis>=85) && (dis<92))
	dis=15;
	else if((dis>=92) && (dis<99))
	dis=14;
	else if((dis>=99) && (dis<96))
	dis=13;
	else if((dis>=96) && (dis<113))
	dis=12;
	else if((dis>=113) && (dis<120))
	dis=11;
	else if((dis>=120) && (dis<152))
	dis=10;
	else if((dis>=152) && (dis<184))
	dis=9;
	else if((dis>=184) && (dis<216))
	dis=8;
	else if((dis>=216) && (dis<248))
	dis=7;
	else if((dis>=248) && (dis<280))
	dis=6;
	else if((dis>=280) && (dis<418))
	dis=5;
	else if((dis>=418) && (dis<556))
	dis=4;
	else if((dis>=556) && (dis<694))
	dis=3;
	else if((dis>=694) && (dis<832))
	dis=2;
	else if((dis>=832) && (dis<=950))
	dis=1;
	else if((dis>950))
	dis=0;
	else
	dis=out_of_range;


	if(dis==out_of_range)
	{
	lcd_command(0x8b);
	lcd_string("error  ");
	}
	else
	{
	lcd_command(0x8d);
	lcd_number(dis);
	lcd_command(0x8e);
	lcd_string("cm");
	}

	_delay_ms(100);
	}
		  
	return 0;
}

void lcd_init()
{
	lcd_command(0x02);
 	lcd_command(0x28);
	lcd_command(0x06);
	lcd_command(0x0c);
}
void lcd_command(unsigned char com)
	{
	lcd = com & 0xF0;		//send higher bit

	lcd &= ~(1<<rs); 		//rs =0
	lcd &= ~(1<<rw);		//rw =0
	lcd |=(1<<en);			//en =1
	_delay_ms(1);
	lcd &= ~(1<<en);		//en =0
	_delay_ms(1);

	lcd = (com<<4) & 0xF0;	//send lower bit

	lcd &= ~(1<<rs); 		//rs =0
	lcd &= ~(1<<rw);		//rw =0
	lcd |=(1<<en);			//en =1
	_delay_ms(1);
	lcd &= ~(1<<en);		//en =0
	_delay_ms(1);
	}
void lcd_data(unsigned char value)
	{

	lcd =value & 0xF0;		//send higher bit

	lcd |= (1<<rs); 		//rs =1
	lcd &= ~(1<<rw);		//rw =0
	lcd |=(1<<en);			//en =1
	_delay_ms(1);
	lcd &= ~(1<<en);		//en =0
	_delay_ms(1);


	lcd =(value<<4) & 0xF0;	//send lower bit

	lcd |= (1<<rs); 		//rs =1
	lcd &= ~(1<<rw);		//rw =0
	lcd |=(1<<en);			//en =1
	_delay_ms(1);
	lcd &= ~(1<<en);		//en =0
	_delay_ms(1);
	}

void lcd_string(unsigned char *str)
{
	char i=0;
	while(str[i]!='\0')
	{
	lcd_data(str[i]);
	i++;
	}
}

void lcd_number(unsigned int value)
{
	unsigned int d=0;
	lcd_command(0x04);	//auto decrement mode

	if(value==0)
	lcd_data(value+48);

	while(value!=0)
	{
	d=value%10;
	lcd_data(d+48);
	value=value/10;
	}
	lcd_command(0x06);	//auto increment mode
}

void adc_init()
{
	ADMUX = 0B01000000;//(1<<REFS0);// | (1<<REFS1);
	ADCSRA = ((1<<ADEN) | (1<<ADPS2)| (1<<ADPS1)| (1<<ADPS0));
	}
unsigned int adc_read(unsigned char channel)
{	
	
	ADMUX = 0x40|channel; 
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));

	ADCSRA |= (1<<ADIF);
	return ADC;
}
/*void trigger(){
    TRIG_LOW();             // Clear pin before setting it high
    _delay_us(1);           // Clear to zero and give time for electronics to set
    TRIG_HIGH();            // Set pin high
    _delay_us(12);          // Send high pulse for minimum 10us
    TRIG_LOW();             // Clear pin
    _delay_us(1);           // Delay not required, but just in case...
}*/
