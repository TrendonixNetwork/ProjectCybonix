#define F_CPU	4000000UL
#include <avr/delay.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
 
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
char time[] = "00:00:00";
 

void LCD_command(unsigned char cmnd);
void LCD_data(unsigned char data);
void LCD_init();
void LCD_goto(unsigned char y, unsigned char x);
void LCD_string(char *string);
void LCD_update_time(void);
 

ISR(TIMER1_COMPA_vect);
 
#define LCD_DATA_PORT	PORTB
#define LCD_DATA_DDR	DDRB
#define LCD_DATA_PIN	PINB
 
#define LCD_CNTRL_PORT	PORTC
#define LCD_CNTRL_DDR	DDRC
#define LCD_CNTRL_PIN	PINC
 #define LCD_RS_PIN		5
#define LCD_RW_PIN		6
#define LCD_ENABLE_PIN	7
#define SET_HOUR		3
#define SET_MINUTE		4
 
void main()
{
	unsigned char i; 
 
    LCD_init();
	LCD_goto(1,3);
	LCD_string("AEROFARMING");
	LCD_goto(2,4);
	LCD_string(time);
 
	LCD_CNTRL_PORT = (1<<SET_HOUR | 1<<SET_MINUTE);
 
	TCCR1B = (1<<CS12|1<<WGM12);
	OCR1A = (15625-1);
	TIMSK = 1<<OCIE1A;
	sei();
 
	while(1)
    {
		if(!(LCD_CNTRL_PIN & (1<<SET_HOUR)))
		{
			hours++;
			if(hours > 23)
				hours = 0;
		}
		if(!(LCD_CNTRL_PIN & (1<<SET_MINUTE)))
		{
			minutes++;
			if(minutes > 59)
				minutes = 0;
		}
		_delay_ms(250);
	}
}
 

void LCD_command(unsigned char cmd)
{
	LCD_DATA_PORT = cmd;
	LCD_CNTRL_PORT &= ~(1<<LCD_RW_PIN);
	LCD_CNTRL_PORT &= ~(1<<LCD_RS_PIN);
 
	LCD_CNTRL_PORT |= (1<<LCD_ENABLE_PIN);
	_delay_us(2);
	LCD_CNTRL_PORT &= ~(1<<LCD_ENABLE_PIN);
	_delay_us(100);
}
 

void LCD_data(unsigned char data)
{
	LCD_DATA_PORT = data;
	LCD_CNTRL_PORT &= ~(1<<LCD_RW_PIN);
	LCD_CNTRL_PORT |= (1<<LCD_RS_PIN);
 
	LCD_CNTRL_PORT |= (1<<LCD_ENABLE_PIN);
	_delay_us(2);
	LCD_CNTRL_PORT &= ~(1<<LCD_ENABLE_PIN);
	_delay_us(100);
}
 
void LCD_init()
{
	LCD_CNTRL_DDR = 0xFF;
	LCD_CNTRL_PORT = 0x00;
	LCD_DATA_DDR = 0xFF;
	LCD_DATA_PORT = 0x00;
 
	_delay_ms(10);
	LCD_command(0x38);
	LCD_command(0x0C);
	LCD_command(0x01);
	_delay_ms(10);
	LCD_command(0x06);
}
 

void LCD_goto(unsigned char y, unsigned char x)
{
	unsigned char firstAddress[] = {0x80,0xC0,0x94,0xD4};
 
	LCD_command(firstAddress[y-1] + x-1);
	_delay_ms(10);	
}
 
void LCD_string(char *string)
{
	unsigned char i;
 
	while(string[i]!=0)
	{
		LCD_data(string[i]);
		i++;
	}
}
void LCD_update_time()
{
	unsigned char temp;
 
	LCD_goto(2,4);
 
	itoa(hours/10,temp,10);
	LCD_string(temp);
	itoa(hours%10,temp,10);
	LCD_string(temp);
	LCD_string(":");
 
	itoa(minutes/10,temp,10);
	LCD_string(temp);
	itoa((minutes%10),temp,10);
	LCD_string(temp);
	LCD_string(":");
 
	itoa(seconds/10,temp,10);
	LCD_string(temp);
	itoa(seconds%10,temp,10);
	LCD_string(temp);
}

ISR(TIMER1_COMPA_vect)
{		
	seconds++;
 
	if(seconds == 60)
	{
		seconds = 0;
		minutes++;
	}
	if(minutes == 60)
	{
		minutes = 0;
		hours++;		
	}
	if(hours > 23)
		hours = 0;
 
	LCD_update_time();
}
