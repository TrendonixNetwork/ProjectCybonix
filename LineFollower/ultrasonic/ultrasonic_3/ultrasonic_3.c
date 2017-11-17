#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

/* connections
 Vcc=+5v
 Trig=PD6
 Echo=PD3
 Ground=0v
*/

volatile unsigned char up = 0;
volatile uint32_t running = 0;
volatile uint32_t timercounter =0;
volatile long avg = 0;

// interrupt for INT1 pin, to detect high/low voltage changes

void InitADC()
{
	ADMUX=(1<<REFS0);                                  	// For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //Prescalar div factor =128

}


ISR(TIMER0_OVF_vect)
{
    if(up) 
	
	{
        timercounter++;
    }
}


// We assume, that high voltage rise comes before low drop and not vice versa
// Check change in the level at the PD3 for falling/rising edge
SIGNAL(INT1_vect){
    if(running){ //accept interrupts only when sonar was started
        if (up == 0) { // voltage rise, start time measurement
            up = 1;
            timercounter = 0;            
            TCCR0 |= (0 << CS02)|(0 << CS01)|(1 << CS00); // Start/initialize timer with prescalar 0
            TCNT0 = 0; // Initialize Counter
        } else { // voltage drop, stop time measurement
            up = 0;
            avg = (timercounter*256+TCNT0)/58;// divide by 58 to get distance in cm
           // light_flashing();
            running = 0;
        }
    }
}


void send_trig() // function to send out signal through trig pin
{

	PORTD = 0x00;
	_delay_us(5);

    PORTD = 0xf0;
     running = 1;
	 
    _delay_us(10);
    PORTD = 0x00;


}

void echo_rec()
{

if(PIND&0x0f)
{
PORTC=0xff;
}
}


void main()

{

	DDRD = 0xf0;
	PORTD=0x00;
	DDRB = 0xff;
	PORTB=0x00;
	DDRC=0xff;

	MCUCR |= (0 << ISC11) | (1 << ISC10); // enable interrupt on any(rising/droping) edge
    GICR |= (1 << INT1);  //Turns on INT1

    TIMSK |= (1 << TOIE0);  // enable timer interrupt
    sei();  // enable all(global) interrupts


	

	while(1)
	{
	if(running==0)
	{
		_delay_ms(60);

	send_trig();
	PORTB=0b00000111;
}
	echo_rec();
	
	
}

}


