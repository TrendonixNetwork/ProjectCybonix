#include<avr/io.h>

//#define sbi
//#define cbi

void main()
{
DDRC = 0xFF; // PORTC as output
//PORTC = 0xFF; //Initially LED is Off
TCCR1B = (1<<CS12)|(1<<CS10)|(0<<CS11); // Clock prescaler is set to divided by 64
while(1)
{
delay_1sec();
PORTC=0xff;
delay_1sec(); 
PORTC=0x00;
delay_1sec(); 
}
}

void delay_1sec()
{
TCNT1=78125;
do
{
TCNT1=(TCNT1-100);
}while(TCNT1>0);
}
