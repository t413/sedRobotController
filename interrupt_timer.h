// Automatic Altitude control from ultrasonic sensor
// by Tim O'Brien (t413.com), Sept. 16, 2010.


#ifndef INTERRUPT_TIMER_H
#define INTERRUPT_TIMER_H
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif

    
void timer0_init(void);
unsigned long millis(void);
unsigned long tics(void);
    
void ppm_timing_read_init(void);
int8_t get_ppm_timings(unsigned long * timing_array);

    
#ifdef __cplusplus
}
#endif
#endif
