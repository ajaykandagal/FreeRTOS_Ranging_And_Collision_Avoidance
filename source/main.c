/*******************************************************************************
 * @file	main.c
 * @brief 	This is an freeRTOS based application for ranging using vl53l0x ToF
 * 			sensor. The application measures the distance from an obstacle using
 * 			ToF sensor and based on the proximity turns on/off the buzzer.
 *
 * @project	Range Sensing and Collision Avoidance Using ToF Sensor on FreeRTOS
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 01, 2022
 ******************************************************************************/


/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"


/* Application specific includes */
#include "tof.h"
#include "vl53l0x.h"
#include "buzzer.h"
#include "common.h"


/* Task priorities */
#define HIGHEST_PRIORITY 			(configMAX_PRIORITIES - 1)
#define TOF1_SENSOR_TASK_PRIORITY 	(HIGHEST_PRIORITY)
#define TOF2_SENSOR_TASK_PRIORITY 	(HIGHEST_PRIORITY)
#define WARNING_TASK_PRIORITY 		(HIGHEST_PRIORITY - 1)


/* Application specific variables*/
typedef struct {
	uint8_t tof_id;
	e_proximity_t e_proximity;
	uint16_t proximity_arr[5];
} tof_sensor_data_t;


/* Holds the current data of ToF sensor */
tof_sensor_data_t tof_sensor_data[] = {
		{
				.tof_id = VL53L0X_IDX_FIRST,
				.e_proximity = PROXIMITY_SAFE,
				.proximity_arr = {250, 200, 160, 130, 100}
		}
};


/* Task prototypes */
static void tof_ranging_task(void *pvParameters);
static void warning_task(void *pvParameters);


/* Task handles */
TaskHandle_t warning_handle;
TaskHandle_t tof_ranging_handle;


/**
 * @brief Application entry point.
 */
int main(void)
{
	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	/* Application specific initialization */
	buzzer_init();
	tof_init();

	/* Creating RTOS tasks for ranging */
	xTaskCreate(warning_task, "Warning", configMINIMAL_STACK_SIZE + 10, NULL, TOF1_SENSOR_TASK_PRIORITY, &warning_handle);
	xTaskCreate(tof_ranging_task, "ToF Ranging", configMINIMAL_STACK_SIZE + 10, NULL, WARNING_TASK_PRIORITY, &tof_ranging_handle);
	vTaskStartScheduler();
	for (;;);
}


/*******************************************************************************
 * RTOS task for ranging using the ToF sensor. Based on the range data sets the
 * prximity which will be used by the other tasks to take respective action.
 *
 * @param
 *  *pvParameters	Pointer to the parameters used by the RTOS task.
 *
 ******************************************************************************/
static void tof_ranging_task(void *pvParameters)
{
	uint16_t range;
	for (;;)
	{
		range = tof_get_range(tof_sensor_data[0].tof_id);

		if (range != VL53L0X_OUT_OF_RANGE) {
			PRINTF("\n\rRange: %u",range);
			tof_sensor_data[0].e_proximity = PROXIMITY_SAFE;
			for (int i = 4; i >= 0; i--) {
				if (range < tof_sensor_data[0].proximity_arr[i]) {
					tof_sensor_data[0].e_proximity = (e_proximity_t) (i + 1);
					break;
				}
			}

			if (tof_sensor_data[0].e_proximity != PROXIMITY_SAFE) {
				if (eTaskGetState(warning_handle) == eSuspended)
					vTaskResume(warning_handle);
			}
		}
		else {
			tof_sensor_data[0].e_proximity = PROXIMITY_SAFE;
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


/*******************************************************************************
 * RTOS task for setting up the tone frequency using TPM0 and turning on/off the
 * buzzer based on the proximity set by the ToF ranging task.
 *
 * @param
 *  *pvParameters	Pointer to the parameters used by the RTOS task.
 *
 ******************************************************************************/
static void warning_task(void *pvParameters)
{
	e_proximity_t last_e_proximity = PROXIMITY_SAFE;

	while (1)
	{
		if (last_e_proximity != tof_sensor_data[0].e_proximity)
		{
			last_e_proximity = tof_sensor_data[0].e_proximity;

			if (last_e_proximity ==  PROXIMITY_SAFE) {
				buzzer_stop();
				vTaskSuspend(NULL);
			}
			else {
				buzzer_setup(last_e_proximity);
			}
		}

		buzzer_start();
		vTaskDelay(get_buzzer_on_period() / portTICK_PERIOD_MS);
		buzzer_stop();
		vTaskDelay(get_buzzer_off_period() / portTICK_PERIOD_MS);
	}
}
