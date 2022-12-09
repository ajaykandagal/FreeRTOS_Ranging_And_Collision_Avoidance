/*******************************************************************************
 * @file	buzzer.h
 * @brief	This file contains functions to initialize TPM0 to generate a tone
 * 			frequency based on ranging data which will be used to drive the
 * 			buzzer.
 *
 * @author	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data	Dec 03, 2022
 ******************************************************************************/
#ifndef BUZZER_H_
#define BUZZER_H_


#include "common.h"


/*******************************************************************************
 * Initializes TPM0 and selects channel 0 to generate PWM signal on PORTD PIN0.
 ******************************************************************************/
void buzzer_init();


/*******************************************************************************
 * Enables TPM0 to generate PWM signal and on period of the buzzer based on the
 * proximity measured by the ToF sensor.
 *
 * @param
 *  e_proximity		Represents the proximity of the obstacle.
 *
 ******************************************************************************/
void buzzer_setup(e_proximity_t e_proximity);


/*******************************************************************************
 * Enables TPM0 to start PWM signal generation and buzzer starts playing the
 * tone.
 ******************************************************************************/
void buzzer_start();


/*******************************************************************************
 * Enables TPM0 to stop PWM signal generation and buzzer stops playing the tone.
 ******************************************************************************/
void buzzer_stop();


/*******************************************************************************
 * Returns the buzzer on period set by buzzer_setup() based on proximity.
 *
 * @return
 *  Returns the on period of the buzzer
 *
 ******************************************************************************/
uint16_t get_buzzer_on_period();


/*******************************************************************************
 * Returns the buzzer off period set by buzzer_setup() based on proximity.
 *
 * @return
 *  Returns the on period of the buzzer
 ******************************************************************************/
uint16_t get_buzzer_off_period();


#endif /* BUZZER_H_ */
