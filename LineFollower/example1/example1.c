#include<avr/io.h>
#include<util/delay.h>

void main()
{
DDRB=0xff;

while(1)
{
PORTB=0xff;
}}
