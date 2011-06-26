#ifndef WC_CONTROLLER_H
#define WC_CONTROLLER_H

#define BAUDRATE 115200 
#define TIMER_TCCR2A 0b00000011 // Fast PWM 
#define TIMER_TCCR2B 0b00001110 // Prescaler 256
#define TIMER_TOP 249 
#define TIMER_FREQ 250 
#define RT_LOOP_TOP 5 
#define RT_LOOP_FREQ 50 

#endif
