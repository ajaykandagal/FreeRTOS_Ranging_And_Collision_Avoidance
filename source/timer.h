/*******************************************************************************
 * @file	timer.h
 * @brief 	Initializes SysTick timer to generate interrupt for every 1 second
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Nov 23, 2022
 ******************************************************************************/
#ifndef TIMER_H_
#define TIMER_H_


#include <MKL25Z4.H>


/*******************************************************************************
 * Initializes SysTick timer to generate interrupts at every 1 second.
 ******************************************************************************/
void init_systick();


/*******************************************************************************
 * Returns true when there is systick timer overflow.
 *
 * Return:
 *   Returns true if there is timer overflow else returns false
 ******************************************************************************/
uint32_t get_time_ms();

void delay_ms(uint32_t ms);


#endif /* TIMER_H_ */
