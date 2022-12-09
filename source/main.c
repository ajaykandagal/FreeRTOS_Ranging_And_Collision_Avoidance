/*******************************************************************************
 * @file	main.c
 * @brief 	This is an FreeRTOS based application for ranging using vl53l0x ToF
 * 			sensor. The application measures the distance from an obstacle using
 * 			ToF sensor and based on the proximity turns on/off the buzzer.
 *
 * @project	Range Sensing and Collision Avoidance Using ToF Sensor on FreeRTOS
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 08, 2022
 ******************************************************************************/


#include "common.h"

#if TESTING  == 0
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#endif	// TESTING  == 0

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"


/* Application specific includes */
#include "tof.h"
#include "vl53l0x.h"
#include "buzzer.h"


#if TESTING
#include "main_test.h"
#endif

/* Task priorities */
#define HIGHEST_PRIORITY 			(configMAX_PRIORITIES - 1)
#define TOF1_SENSOR_TASK_PRIORITY 	(HIGHEST_PRIORITY)
#define TOF2_SENSOR_TASK_PRIORITY 	(HIGHEST_PRIORITY)
#define TOF1_PROCESS_TASK_PRIORITY 	(HIGHEST_PRIORITY - 1)
#define TOF2_PROCESS_TASK_PRIORITY 	(HIGHEST_PRIORITY - 1)
#define WARNING_TASK_PRIORITY 		(HIGHEST_PRIORITY - 2)

#define TOF_RANGE_Q_SIZE			(10)


#if TESTING == 0
/* Application specific variables*/
typedef struct {
	uint8_t tof_id;
	e_proximity_t e_proximity;
	QueueHandle_t range_val;
} tof_sensor_data_t;


/* Holds the current data of ToF sensor */
tof_sensor_data_t tof_sensor_data[] = {
		{
				.tof_id = VL53L0X_IDX_FIRST,
				.e_proximity = PROXIMITY_SAFE
		}
};


/* Task prototypes */
static void tof_ranging_task(void *pvParameters);
static void tof_process_task(void *pvParameters);
static void warning_task(void *pvParameters);


/* Task handles */
TaskHandle_t tof_ranging_handle;
TaskHandle_t tof_process_handle;
TaskHandle_t warning_handle;
#endif	// TESTING  == 0


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

#if TESTING == 0
	/* Create queue to accumulate range data and then give it for processing */
	tof_sensor_data[0].range_val = xQueueCreate(TOF_RANGE_Q_SIZE, sizeof(uint16_t));

	/* Creating RTOS tasks for ranging */
	xTaskCreate(tof_ranging_task, "ToF Ranging", configMINIMAL_STACK_SIZE + 10, NULL,
			WARNING_TASK_PRIORITY, &tof_ranging_handle);

	xTaskCreate(tof_process_task, "ToF Processing", configMINIMAL_STACK_SIZE + 10, NULL,
			WARNING_TASK_PRIORITY, &tof_process_handle);

	xTaskCreate(warning_task, "Warning", configMINIMAL_STACK_SIZE + 10, NULL,
			TOF1_SENSOR_TASK_PRIORITY, &warning_handle);

	vTaskStartScheduler();
#endif	// TESTING == 0

	while(1);
}


#if TESTING == 0
/*******************************************************************************
 * RTOS task for ranging using the ToF sensor.
 *
 * @param
 *  *pvParameters	Pointer to the parameters used by the RTOS task.
 *
 ******************************************************************************/
static void tof_ranging_task(void *pvParameters)
{
	uint16_t range = VL53L0X_OUT_OF_RANGE;
	uint16_t last_range = VL53L0X_OUT_OF_RANGE;

	while(1)
	{
		range = tof_get_range(tof_sensor_data[0].tof_id);

		if (range != VL53L0X_OUT_OF_RANGE)
		{
			PRINTF("\n\rR: %u",range);

			/* Add valid range to queue */
			if (xQueueSend(tof_sensor_data[0].range_val, (void*)&range, 0) != pdTRUE)
				PRINTF("Queue is full");

			last_range = range;
		}
		else
		{
			/* Add VL53L0X_OUT_OF_RANGE data to queue to indicate process task
			 * that ToF giving out-of-range values so that it can stop the buzzer */
			if (last_range != VL53L0X_OUT_OF_RANGE)
			{
				last_range = VL53L0X_OUT_OF_RANGE;
				if (xQueueSend(tof_sensor_data[0].range_val, (void*)&last_range, 0) != pdTRUE)
					PRINTF("Queue is full");
			}
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}


/*******************************************************************************
 * Processes the range data and sets the proximity which will be used by the
 * other tasks to take respective action.
 *
 * @param
 *  *pvParameters	Pointer to the parameters used by the RTOS task.
 *
 ******************************************************************************/
static void tof_process_task(void *pvParameters)
{
	uint16_t range_val;
	e_proximity_t e_proximity = PROXIMITY_SAFE;

	while (1)
	{
		if (xQueueReceive(tof_sensor_data[0].range_val, (void*)&range_val, portMAX_DELAY) == pdTRUE)
		{
			PRINTF("\n\rP: %u", range_val);

			if (range_val < proximity_slots[PROXIMITY_QUITE_CLOSE])
				e_proximity = PROXIMITY_QUITE_CLOSE;
			else if (range_val < proximity_slots[PROXIMITY_CLOSE])
				e_proximity = PROXIMITY_CLOSE;
			else if (range_val < proximity_slots[PROXIMITY_MID])
				e_proximity = PROXIMITY_MID;
			else if (range_val < proximity_slots[PROXIMITY_FAR])
				e_proximity = PROXIMITY_FAR;
			else if (range_val < proximity_slots[PROXIMITY_QUITE_FAR])
				e_proximity = PROXIMITY_QUITE_FAR;
			else
				e_proximity = PROXIMITY_SAFE;

			tof_sensor_data[0].e_proximity = e_proximity;

			if (e_proximity != PROXIMITY_SAFE)
			{
				if (eTaskGetState(warning_handle) == eSuspended)
					vTaskResume(warning_handle);
			}
		}
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
	e_proximity_t e_proximity = PROXIMITY_SAFE;

	while (1)
	{
		e_proximity = tof_sensor_data[0].e_proximity;

		if (e_proximity == PROXIMITY_SAFE)
		{
			PRINTF("\n\rStopped");

			last_e_proximity = PROXIMITY_SAFE;
			buzzer_stop();
			vTaskSuspend(NULL);
		}
		else
		{
			PRINTF("\n\rPlaying");

			if (e_proximity !=  last_e_proximity)
				buzzer_setup(e_proximity);

			buzzer_start();
			vTaskDelay(get_buzzer_on_period() / portTICK_PERIOD_MS);
			buzzer_stop();
			vTaskDelay(get_buzzer_off_period() / portTICK_PERIOD_MS);

			last_e_proximity = e_proximity;
		}
	}
}
#endif	// TESTING == 0
