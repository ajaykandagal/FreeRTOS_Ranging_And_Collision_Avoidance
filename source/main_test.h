/*
 * main_test.h
 *
 *  Created on: 09-Dec-2022
 *      Author: ajayk
 */

#ifndef MAIN_TEST_H_
#define MAIN_TEST_H_


#include "tof.h"


/*******************************************************************************
 * This test will check the ToF sensor working. To carry out this test ToF
 * sensor should be placed at a known distance from the obstacle. Set the macro
 * OBSTACLE_DISTANE_MM with known object distance.
 *
 * @param
 *  idx		Id of ToF sensor to test.
 *
 ******************************************************************************/
void test_tof_sensor(vl53l0x_idx_t idx);


/*******************************************************************************
 * This is a observation based test. The test will verify the buzzer on period
 * by passing various proximity values and then plays the tone by passing the
 * valid proximity value.
 *
 * Note: You should be able to able to hear the buzzer with different on
 * periods and lastly the buzzer will be played continuously.
 ******************************************************************************/
void test_buzzer();


#endif /* MAIN_TEST_H_ */
