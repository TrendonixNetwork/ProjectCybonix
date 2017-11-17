#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include <avr/interrupt.h>

/*
//#ifndef SONAR_H_
//#define SONAR_H_
 
//#ifndef F_CPU
  //  #define F_CPU 1000000UL     // CPU Frequency
//#endif
 
//#include <avr/io.h>

//#include <util/delay.h>
 
/*...- . . .-. --- -... --- -
 * Define Ports and Pins as required
 * Modify Maximum response time and delay as required
 * MAX_RESP_TIME : default: 300
 * DELAY_BETWEEN_TESTS : default: 50
 */
#define TRIG_DDR    DDRD            // Trigger Port
#define TRIG_PORT   PORTD
#define TRIG_PIN    PIND
#define TRIG_BIT    PD0             // Trigger Pin
 
#define ECHO_DDR    DDRA            // Echo Port
#define ECHO_PORT   PORTA
#define ECHO_PIN    PINA
#define ECHO_BIT    PA0             // Echo Pin
 
// Speed of sound
// Default: 343 meters per second in dry air at room temperature (~20C)
#define SPEED_OF_SOUND  343
#define MAX_SONAR_RANGE 10          // This is trigger + echo range (in meters) for SR04
#define DELAY_BETWEEN_TESTS 500     // Echo canceling time between sampling. Default: 500us
#define TIMER_MAX 65535             // 65535 for 16 bit timer and 255 for 8 bit timer
 
/* ...- . . .-. --- -... --- -
 * Do not change anything further unless you know what you're doing
 * */
#define TRIG_ERROR -1
#define ECHO_ERROR -2
 
#define CYCLES_PER_US (F_CPU/1000000)// instructions per microsecond
#define CYCLES_PER_MS (F_CPU/1000)      // instructions per millisecond
// Timeout. Decreasing this decreases measuring distance
// but gives faster sampling
#define SONAR_TIMEOUT ((F_CPU*MAX_SONAR_RANGE)/SPEED_OF_SOUND)
 
#define TRIG_INPUT_MODE() TRIG_DDR &= ~(1<<TRIG_BIT)
#define TRIG_OUTPUT_MODE() TRIG_DDR |= (1<<TRIG_BIT)
#define TRIG_LOW() TRIG_PORT &= ~(1<<TRIG_BIT)
#define TRIG_HIGH() TRIG_PORT |=(1<<TRIG_BIT)
 
#define ECHO_INPUT_MODE() ECHO_DDR &= ~(1<<ECHO_BIT)
#define ECHO_OUTPUT_MODE() ECHO_DDR |= (1<<ECHO_BIT)
#define ECHO_LOW() ECHO_PORT &= ~(1<<ECHO_BIT)
#define ECHO_HIGH() ECHO_PORT |=(1<<ECHO_BIT)
 
#define CONVERT_TO_CM ((10000*2)/SPEED_OF_SOUND)    // or simply 58
 
/** ...- . . .-. --- -... --- -
 * @brief   Initiate Ports for Trigger and Echo pins
 * @param   void
 * @return  none
*/
void init_sonar();
 
/**  ...- . . .-. --- -... --- -
 * @brief   Send 10us pulse on Ultrasonic Trigger pin
 * @param   void
 * @return  none
*/
void trigger_sonar();
 
/**  ...- . . .-. --- -... --- -
 * @brief   Calculate and store echo time and return distance
 * @param   void
 * @return  unsigned int
 * Usage    int foo = read_sonar();
*/
unsigned int read_sonar();
 
//#endif /* SONAR_H_ */


void main()
{
DDRA=0x00;
DDRD=0xff;
int s=0;
while(1)
{
s=(PINA&0x01);
if(s==0x01)
{
PORTD=0b00000010;
s=0;
}
else

PORTD=0x00;
}}


//#include "sonar.h"
 
volatile uint32_t overFlowCounter = 0;
volatile uint32_t trig_counter = 0;
volatile uint32_t no_of_ticks = 0;
 
/********** ...- . . .-. --- -... --- - *********************************
 * Initiate Ultrasonic Module Ports and Pins
 * Input:   none
 * Returns: none
*********** ...- . . .-. --- -... --- - *********************************/
void init_sonar(){
    TRIG_OUTPUT_MODE();     // Set Trigger pin as output
    ECHO_INPUT_MODE();      // Set Echo pin as input
}
 
/********** ...- . . .-. --- -... --- - *********************************
 * Send 10us pulse on Sonar Trigger pin
 * 1.   Clear trigger pin before sending a pulse
 * 2.   Send high pulse to trigger pin for 10us
 * 3.   Clear trigger pin to pull it trigger pin low
 *  Input:   none
 *  Returns: none
********** ...- . . .-. --- -... --- - *********************************/
void trigger_sonar(){
    TRIG_LOW();             // Clear pin before setting it high
    _delay_us(1);           // Clear to zero and give time for electronics to set
    TRIG_HIGH();            // Set pin high
    _delay_us(12);          // Send high pulse for minimum 10us
    TRIG_LOW();             // Clear pin
    _delay_us(1);           // Delay not required, but just in case...
}
 
/********** ...- . . .-. --- -... --- - *********************************
 * Increment timer on each overflow
 * Input:   none
 * Returns: none
********** ...- . . .-. --- -... --- - *********************************/
ISR(TIMER1_OVF_vect){   // Timer1 overflow interrupt
    overFlowCounter++;
    TCNT1=0;
}
 
/********** ...- . . .-. --- -... --- - *********************************
 * Calculate and store echo time and return distance
 * Input:   none
 * Returns: 1. -1       :   Indicates trigger error. Could not pull trigger high
 *          2. -2       :   Indicates echo error. No echo received within range
 *          3. Distance :   Sonar calculated distance in cm.
********** ...- . . .-. --- -... --- - *********************************/
unsigned int read_sonar(){
    int dist_in_cm = 0;
    init_sonar();                       // Setup pins and ports
    trigger_sonar();                    // send a 10us high pulse
 
    while(!(ECHO_PIN & (1<<ECHO_BIT))){   // while echo pin is still low
        trig_counter++;
         uint32_t max_response_time = SONAR_TIMEOUT;
        if (trig_counter > max_response_time){   // SONAR_TIMEOUT
            return TRIG_ERROR;
        }
    }
 
    TCNT1=0;                            // reset timer
    TCCR1B |= (1<<CS10);              // start 16 bit timer with no prescaler
    //TIMSK1 |= (1<<TOIE1);             // enable overflow interrupt on timer1
    overFlowCounter=0;                  // reset overflow counter
    sei();                              // enable global interrupts
 
    while((ECHO_PIN & (1<<ECHO_BIT))){    // while echo pin is still high
        if (((overFlowCounter*TIMER_MAX)+TCNT1) > SONAR_TIMEOUT){
            return ECHO_ERROR;          // No echo within sonar range
        }
    };
 
    TCCR1B = 0x00;                      // stop 16 bit timer with no prescaler
    cli();                              // disable global interrupts
    no_of_ticks = ((overFlowCounter*TIMER_MAX)+TCNT1);  // counter count
    dist_in_cm = (no_of_ticks/(CONVERT_TO_CM*CYCLES_PER_US));   // distance in cm
    return (dist_in_cm );
}


