/*******************************************************************************
 * @file	main_test.h
 * @brief 	This file contains functions to test ToF sensor and the buzzer.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @date 	Dec 07, 2022
 *
 * @editor	Dec 09, 2022, Ajay Kandagal, ajka9053@colorado.edu
 * @change	Added test cases for buzzer.
 ******************************************************************************/
#ifndef MAIN_TEST_H_
#define MAIN_TEST_H_


/*******************************************************************************
 * A common call to test both ToF sensor and the buzzer.
 *
 * TOF SENSOR TEST
 * The test will check the ToF sensor working. To carry out this test ToF
 * sensor should be placed at a known distance from the obstacle. Set the macro
 * OBSTACLE_DISTANE_MM with known object distance.
 *
 * BUZZER TEST
 * The test will verify the buzzer on period by passing various proximity values.
 * Next will be hearing based test, the buzzer is played by passing the valid
 * proximity value. You should be able to able to hear the buzzer with different
 * on periods.
 ******************************************************************************/
void main_test();


#endif /* MAIN_TEST_H_ */
