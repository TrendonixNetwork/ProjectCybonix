#define F_CPU 8000000UL
#include <avr/io.h>
#include<avr/wdt.h>
#include<avr/interrupt.h>
#include<util/delay.h>

volatile long avg = 0;
volatile unsigned char up = 0;
volatile uint32_t running = 0;
volatile uint32_t timercounter =0;

// interrupt for INT1 pin, to detect high/low voltage changes
ISR(TIMER0_OVF_vect)
{
    if (up) {
        timercounter++;
    }
}

void light_flashing()
{
    
    if((avg>0)&&(avg<=15))
      {
          PORTB=0b00000001; // first led on
       
      }

      else if((avg>15)&&(avg<=30))
      {
        PORTB=0b00000010; // second led on
      }

      else if((avg>30)&&(avg<=45))
      {
        PORTB=0b00000100; // third led on
      }

      else if((avg>45)&&(avg<=60))
      {
          PORTB=0b00001000; // fourth led on
      }

      else if((avg>60)&&(avg<=75))
      {
           PORTB=0b00010000; // fifth led on
      }

      else if(avg>75)
      {
           PORTB=0b00100000; // sixth led on
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
            light_flashing();
            running = 0;
        }
    }
}

//send signal for trigger the ultrasonic for 10uS
void send_trigger()
{
    PORTD = 0x00;
    _delay_us(5);
    PORTD = 0xf0;
    running = 1;
    _delay_us(10);
    PORTD = 0x00;
	
	
}

int main()
{
    DDRD = 0xf0;//pin d3 is used as input and pin d6 as output for trigger
    PORTD = 0x00;
    DDRB = 0xff;
    PORTB = 0x00;
    DDRA = 0xff;
    PORTA = 0x00;

    MCUCR |= (0 << ISC11) | (1 << ISC10); // enable interrupt on any(rising/droping) edge
    GICR |= (1 << INT1);  //Turns on INT1

    TIMSK |= (1 << TOIE0);  // enable timer interrupt
    sei();  // enable all(global) interrupts
    while(1)
    {

        if(running == 1) {
            _delay_ms(60);
            send_trigger();
			if(0<<PD6)
	{
	PORTB=0xff;
	}
			
        }
    }
}
