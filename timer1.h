/*
 * File:   timer1.h
 * Author: dcl
 *
 * Created on April 29, 2015, 3:29 PM
 */

#ifndef TIMER1_H
#define TIMER1_H

#ifdef  __cplusplus
extern "C" {
#endif

void InitTimer1( void );
uint32_t millis();
float micros();
int control_loop();


#ifdef  __cplusplus
}
#endif

#endif  /* TIMER1_H */
