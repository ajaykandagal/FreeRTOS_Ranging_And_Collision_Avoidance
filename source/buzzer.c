/*******************************************************************************
 * @file	buzzer.h
 * @brief	This file contains functions to initialize TPM0 to generate a tone
 * 			frequency based on ranging data which will be used to drive the
 * 			buzzer.
 *
 * @author	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data	Dec 03, 2022
 *
 * @editor	Dec 09, 2022, Ajay Kandagal, ajka9053@colorado.edu
 * @change	Fixed the buzzer since the proximity enum values were rearranged.
 *
 ******************************************************************************/
#include "MKL25Z4.h"
#include "buzzer.h"


#define BUZZER_PIN 				(0)
#define BUZZER_PORT 			(PORTD)
#define BUZZER_SCGC5_MASK 		(SIM_SCGC5_PORTD_MASK)
#define TPM_CHANNEL 			(0)
#define TPM_SCGC6_MASK 			(SIM_SCGC6_TPM0_MASK)

#define CPU_CLOCK_FREQ			(48000000)
#define TPM_PRESCALER_BIN_VAL 	(7)
#define TPM_PRESCALER_DIV_VAL	(128)


const uint16_t g_tone_frequencies[] = {1200, 2400, 4000};		// ToF1, ToF1 & ToF2, ToF3
const uint16_t g_buzzer_period = 500;	// in milli-seconds
uint16_t g_buzzer_on_period = 0;


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Initializes TPM1 and selects channel 0 to generate PWM signal on PORTD PIN0.
 ******************************************************************************/
void buzzer_init()
{
	SIM->SCGC5 |= BUZZER_SCGC5_MASK;
	SIM->SCGC6 |= TPM_SCGC6_MASK;

	BUZZER_PORT->PCR[BUZZER_PIN] &= ~((uint32_t) PORT_PCR_MUX_MASK);
	BUZZER_PORT->PCR[BUZZER_PIN] |= ((uint32_t) PORT_PCR_MUX(4));

	// Set to use external clock source for TPM
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);

	TPM0->SC = TPM_SC_PS(TPM_PRESCALER_BIN_VAL);
	TPM0->CONF |= TPM_CONF_DBGMODE(3);
	TPM0->CONTROLS[TPM_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	// Disabled
	TPM0->SC &= ~TPM_SC_CMOD(1);
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FOR FULL DESCRIPTION
 *
 * Enables TPM0 to generate PWM signal and on period of the buzzer based on the
 * proximity measured by the ToF sensor.
 ******************************************************************************/
void buzzer_setup(e_proximity_t e_proximity)
{
	TPM0->SC &= ~TPM_SC_CMOD(1);

	g_buzzer_on_period = 0;

	if (e_proximity >= PROXIMITY_SAFE)
		return;

	g_buzzer_on_period = (PROXIMITY_SAFE - e_proximity) * 100;

	uint32_t tmp_mod_val =  ((CPU_CLOCK_FREQ / TPM_PRESCALER_DIV_VAL) / g_tone_frequencies[0]) - 1;

	if (tmp_mod_val > 65535)
		return;

	TPM0->MOD = tmp_mod_val;
	TPM0->CONTROLS[TPM_CHANNEL].CnV = tmp_mod_val / 4;			// 25% duty cycle

	// Enable PWM signal generation on TPM0
	TPM0->SC |= TPM_SC_CMOD(1);
}


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Enables TPM0 to start PWM signal generation and buzzer starts playing the
 * tone.
 ******************************************************************************/
void buzzer_start()
{
	TPM0->SC |= TPM_SC_CMOD(1);
}


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Enables TPM0 to stop PWM signal generation and buzzer stops playing the tone.
 ******************************************************************************/
void buzzer_stop()
{
	TPM0->SC &= ~TPM_SC_CMOD(1);
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FOR FULL DESCRIPTION
 *
 * Returns the buzzer on period set by buzzer_setup() based on proximity.
 ******************************************************************************/
uint16_t get_buzzer_on_period()
{
	return g_buzzer_on_period;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FOR FULL DESCRIPTION
 *
 * Returns the buzzer off period set by buzzer_setup() based on proximity.
 ******************************************************************************/
uint16_t get_buzzer_off_period()
{
	return (g_buzzer_period - g_buzzer_on_period);
}
