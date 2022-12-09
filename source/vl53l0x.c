/*******************************************************************************
 * @file	vl53l0x.c
 * @brief	This code is heavily inspired from Niklas' code. STM doesn't provide
 * 			register mapping documentation for vl5310x ToF sensor and instead
 * 			they have given API library which has to be modified for platform
 * 			specific I2C bus implementation. The library is results in quite
 * 			large memory footprint and uses C++ approach. I have decided to use
 * 			Niklas' code which only implements basic configuration and single
 * 			ranging.
 *
 * @author	Niklas Nilsson
 * @link	https://github.com/artfulbytes/vl6180x_vl53l0x_msp430.git
 *
 * @editor	Dec 01, 2022, Ajay Kandagal, ajaykandagal94@gmail.com
 * @change	Functions have been modified to handle a single ToF sensor based on
 * 			address passed. A bug was observed while taking single measurement
 * 			and has been corrected as per STM's API documentation.
 *
 ******************************************************************************/
#include "vl53l0x.h"
#include "i2c.h"
#include "fsl_debug_console.h"


#define VL53L0X_DEFAULT_MAX_LOOP 200
#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK 0x02
#define VL53L0X_REG_OSC_CALIBRATE_VAL 0x00f8
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD 0x01
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x0013
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY 0x04

#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (0x29)
#define VL53L0X_REG_SYSRANGE_START (0x000)

/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (48)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)


typedef enum
{
	CALIBRATION_TYPE_VHV,
	CALIBRATION_TYPE_PHASE
} calibration_type_t;


static uint8_t g_stop_variable = 0;


/**
 * We can read the model id to confirm that the device is booted.
 * (There is no fresh_out_of_reset as on the vl6180x)
 */
static bool device_is_booted()
{
	uint8_t device_id = 0;
	if (!i2c_read_addr8_data8(REG_IDENTIFICATION_MODEL_ID, &device_id)) {
		return false;
	}
	PRINTF("\n\rDevice ID: %x", device_id);
	return device_id == VL53L0X_EXPECTED_DEVICE_ID;
}


/**
 * One time device initialization
 */
static bool data_init()
{
	bool success = false;

	/* Set 2v8 mode */
	uint8_t vhv_config_scl_sda = 0;
	if (!i2c_read_addr8_data8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda)) {
		return false;
	}
	vhv_config_scl_sda |= 0x01;
	if (!i2c_write_addr8_data8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda)) {
		return false;
	}

	/* Set I2C standard mode */
	success = i2c_write_addr8_data8(0x88, 0x00);

	success &= i2c_write_addr8_data8(0x80, 0x01);
	success &= i2c_write_addr8_data8(0xFF, 0x01);
	success &= i2c_write_addr8_data8(0x00, 0x00);
	/* It may be unnecessary to retrieve the stop variable for each sensor */
	success &= i2c_read_addr8_data8(0x91, &g_stop_variable);
	success &= i2c_write_addr8_data8(0x00, 0x01);
	success &= i2c_write_addr8_data8(0xFF, 0x00);
	success &= i2c_write_addr8_data8(0x80, 0x00);

	return success;
}


/**
 * Wait for strobe value to be set. This is used when we read values
 * from NVM (non volatile memory).
 */
static bool read_strobe()
{
	bool success = false;
	uint8_t strobe = 0;
	if (!i2c_write_addr8_data8(0x83, 0x00)) {
		return false;
	}
	do {
		success = i2c_read_addr8_data8(0x83, &strobe);
	} while (success && (strobe == 0));
	if (!success) {
		return false;
	}
	if (!i2c_write_addr8_data8(0x83, 0x01)) {
		return false;
	}
	return true;
}


/**
 * Gets the spad count, spad type och "good" spad map stored by ST in NVM at
 * their production line.
 * .
 * According to the datasheet, ST runs a calibration (without cover glass) and
 * saves a "good" SPAD map to NVM (non volatile memory). The SPAD array has two
 * types of SPADs: aperture and non-aperture. By default, all of the
 * good SPADs are enabled, but we should only enable a subset of them to get
 * an optimized signal rate. We should also only enable either only aperture
 * or only non-aperture SPADs. The number of SPADs to enable and which type
 * are also saved during the calibration step at ST factory and can be retrieved
 * from NVM.
 */
static bool get_spad_info_from_nvm(uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
	bool success = false;
	uint8_t tmp_data8 = 0;
	uint32_t tmp_data32 = 0;

	/* Setup to read from NVM */
	success  = i2c_write_addr8_data8(0x80, 0x01);
	success &= i2c_write_addr8_data8(0xFF, 0x01);
	success &= i2c_write_addr8_data8(0x00, 0x00);
	success &= i2c_write_addr8_data8(0xFF, 0x06);
	success &= i2c_read_addr8_data8(0x83, &tmp_data8);
	success &= i2c_write_addr8_data8(0x83, tmp_data8 | 0x04);
	success &= i2c_write_addr8_data8(0xFF, 0x07);
	success &= i2c_write_addr8_data8(0x81, 0x01);
	success &= i2c_write_addr8_data8(0x80, 0x01);
	if (!success) {
		return false;
	}

	/* Get the SPAD count and type */
	success &= i2c_write_addr8_data8(0x94, 0x6b);
	if (!success) {
		return false;
	}
	if (!read_strobe()) {
		return false;
	}
	success &= i2c_read_addr8_data32(0x90, &tmp_data32);
	if (!success) {
		return false;
	}
	*spad_count = (tmp_data32 >> 8) & 0x7f;
	*spad_type = (tmp_data32 >> 15) & 0x01;

	/* Restore after reading from NVM */
	success &=i2c_write_addr8_data8(0x81, 0x00);
	success &=i2c_write_addr8_data8(0xFF, 0x06);
	success &=i2c_read_addr8_data8(0x83, &tmp_data8);
	success &=i2c_write_addr8_data8(0x83, tmp_data8 & 0xfb);
	success &=i2c_write_addr8_data8(0xFF, 0x01);
	success &=i2c_write_addr8_data8(0x00, 0x01);
	success &=i2c_write_addr8_data8(0xFF, 0x00);
	success &=i2c_write_addr8_data8(0x80, 0x00);

	/* When we haven't configured the SPAD map yet, the SPAD map register actually
	 * contains the good SPAD map, so we can retrieve it straight from this register
	 * instead of reading it from the NVM. */
	if (!i2c_read_addr8_bytes(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, good_spad_map, 6)) {
		return false;
	}
	return success;
}


/**
 * Sets the SPADs according to the value saved to NVM by ST during production. Assuming
 * similar conditions (e.g. no cover glass), this should give reasonable readings and we
 * can avoid running ref spad management (tedious code).
 */
static bool set_spads_from_nvm()
{
	uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
	uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
	uint8_t spads_enabled_count = 0;
	uint8_t spads_to_enable_count = 0;
	uint8_t spad_type = 0;
	volatile uint32_t total_val = 0;

	if (!get_spad_info_from_nvm(&spads_to_enable_count, &spad_type, good_spad_map)) {
		PRINTF("\n\rget_spad_info_from_nvm failed!");
		return false;
	}
	PRINTF("\n\r%u, %u, %u\n\r", spads_to_enable_count, spad_type, good_spad_map);

	for (int i = 0; i < 6; i++) {
		total_val += good_spad_map[i];
		PRINTF("{%u , %u}\t", i, good_spad_map[i]);
	}
	PRINTF("\n\r");

	bool success = i2c_write_addr8_data8(0xFF, 0x01);
	success &= i2c_write_addr8_data8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	success &= i2c_write_addr8_data8(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	success &= i2c_write_addr8_data8(0xFF, 0x00);
	success &= i2c_write_addr8_data8(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT);
	if (!success) {
		return false;
	}

	uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

	/* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
	 * The subset should only have the number of type enabled as suggested by the reading from
	 * the NVM (spads_to_enable_count and spad_type). */
	for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
		for (int column = 0; column < SPAD_ROW_SIZE; column++) {
			int index = (row * SPAD_ROW_SIZE) + column;
			if (index > SPAD_MAX_COUNT) {
				PRINTF("\n\rset_spads_from_nvm failed 1!");
				return false;
			}
			if (spads_enabled_count == spads_to_enable_count) {
				/* We are done */
				break;
			}
			if (index < offset) {
				continue;
			}
			if ((good_spad_map[row] >> column) & 0x1) {
				spad_map[row] |= (1 << column);
				spads_enabled_count++;
			}
		}
		if (spads_enabled_count == spads_to_enable_count) {
			/* To avoid looping unnecessarily when we are already done. */
			break;
		}
	}

	if (spads_enabled_count != spads_to_enable_count) {
		PRINTF("\n\rset_spads_from_nvm failed 2!");
		return false;
	}

	/* Write the new SPAD configuration */
	if (!i2c_write_addr8_bytes(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, SPAD_MAP_ROW_COUNT)) {
		PRINTF("\n\rset_spads_from_nvm failed 3!");
		return false;
	}

	return true;
}


/**
 * Load tuning settings (same as default tuning settings provided by ST api code)
 */
static bool load_default_tuning_settings()
{
	bool success = i2c_write_addr8_data8(0xFF, 0x01);

	uint8_t tuning_data[79][2] = {{0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00},
			{0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C},
			{0x48, 0x00}, {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04},
			{0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06},
			{0x51, 0x00}, {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00},
			{0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14},
			{0x49, 0xFF}, {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},
			{0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05},
			{0x40, 0x40}, {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03},
			{0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04},
			{0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00},
			{0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01},
			{0x0D, 0x01}, {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01},
			{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
	};

	for(uint8_t i = 0; i < 79; i++)
	{
		success &= i2c_write_addr8_data8(tuning_data[i][0], tuning_data[i][1]);
	}

	return success;
}


static bool configure_interrupt()
{
	/* Interrupt on new sample ready */
	if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) {
		return false;
	}

	/* Configure active low since the pin is pulled-up on most breakout boards */
	uint8_t gpio_hv_mux_active_high = 0;
	if (!i2c_read_addr8_data8(REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high)) {
		return false;
	}
	gpio_hv_mux_active_high &= ~0x10;
	if (!i2c_write_addr8_data8(REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high)) {
		return false;
	}

	if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
		return false;
	}
	return true;
}


/**
 * Enable (or disable) specific steps in the sequence
 */
static bool set_sequence_steps_enabled(uint8_t sequence_step)
{
	return i2c_write_addr8_data8(REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}


/**
 * Basic device initialization
 */
static bool static_init()
{
	if (!set_spads_from_nvm()) {
		PRINTF("\n\rset_spads_from_nvm failed!");
		//return false;
	}
	else {
		PRINTF("\n\rset_spads_from_nvm passed!");
	}

	if (!load_default_tuning_settings()) {
		PRINTF("\n\rload_default_tuning_settings failed!");
		return false;
	}
	else {
		PRINTF("\n\rload_default_tuning_settings passed!");
	}

	if (!configure_interrupt()) {
		PRINTF("\n\rconfigure_interrupt failed!");
		return false;
	}
	else {
		PRINTF("\n\rconfigure_interrupt passed!");
	}

	if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
			RANGE_SEQUENCE_STEP_PRE_RANGE +
			RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		PRINTF("set_sequence_steps_enabled failed!\n");
		return false;
	}

	return true;
}


/**
 * Does a single range measurement.
 */
static bool perform_single_ref_calibration(calibration_type_t calib_type)
{
	uint8_t sysrange_start = 0;
	uint8_t sequence_config = 0;
	switch (calib_type)
	{
	case CALIBRATION_TYPE_VHV:
		sequence_config = 0x01;
		sysrange_start = 0x01 | 0x40;
		break;
	case CALIBRATION_TYPE_PHASE:
		sequence_config = 0x02;
		sysrange_start = 0x01 | 0x00;
		break;
	}
	if (!i2c_write_addr8_data8(REG_SYSTEM_SEQUENCE_CONFIG, sequence_config)) {
		return false;
	}
	if (!i2c_write_addr8_data8(REG_SYSRANGE_START, sysrange_start)) {
		return false;
	}
	/* Wait for interrupt */
	uint8_t interrupt_status = 0;
	bool success = false;
	do {
		success = i2c_read_addr8_data8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
	} while (success && ((interrupt_status & 0x07) == 0));
	if (!success) {
		return false;
	}
	if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
		return false;
	}

	if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x00)) {
		return false;
	}
	return true;
}


/**
 * Temperature calibration needs to be run again if the temperature changes by
 * more than 8 degrees according to the datasheet.
 */
static bool perform_ref_calibration()
{
	if (!perform_single_ref_calibration(CALIBRATION_TYPE_VHV)) {
		PRINTF("perform_single_ref_calibration failed!\n");
		return false;
	}
	if (!perform_single_ref_calibration(CALIBRATION_TYPE_PHASE)) {
		PRINTF("perform_single_ref_calibration failed!\n");
		return false;
	}
	/* Restore sequence steps enabled */
	if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
			RANGE_SEQUENCE_STEP_PRE_RANGE +
			RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		PRINTF("set_sequence_steps_enabled failed!\n");
		return false;
	}
	return true;
}


/**
 * vl5310x sensor allows to configure the address for a session.
 */
static bool configure_address(uint8_t addr)
{
	/* 7-bit address */
	return i2c_write_addr8_data8(REG_SLAVE_DEVICE_ADDRESS, addr & 0x7F);
}


/* Sets the address of a single VL53L0X sensor.
 * This functions assumes that all non-configured VL53L0X are still
 * in hardware standby.
 */
static bool init_address(const uint8_t addr)
{
	i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);

	/* The datasheet doesn't say how long we must wait to leave hw standby,
	 * but using the same delay as vl6180x seems to work fine. */
	for (int i = 0; i < 400; i++);

	if (!device_is_booted()) {
		PRINTF("\n\rdevice_is_booted failed!");
		//      return false;
	}

	if (!configure_address(addr)) {
		PRINTF("\n\rconfigure_address failed!");
		return false;
	}
	return true;
}


/**
 * Initializes the sensors by putting them in hw standby and then
 * waking them up one-by-one as described in AN4846.
 */
static bool init_addresses(const uint8_t addr)
{
	/* Wake each sensor up one by one and set a unique address for each one */
	if (!init_address(addr)) {
		PRINTF("\n\rinit_address failed!");
		return false;
	}

	return true;
}


static bool init_config(const uint8_t addr)
{
	i2c_set_slave_address(addr);
	if (!data_init()) {
		PRINTF("\n\rdata_init failed!");
		return false;
	}
	else {
		PRINTF("\n\rdata_init passed!");
	}

	if (!static_init()) {
		PRINTF("\n\rstatic_init failed!");
		return false;
	}
	else {
		PRINTF("\n\rstatic_init passed!");
	}

	if (!perform_ref_calibration()) {
		PRINTF("\n\rperform_ref_calibration failed!");
		return false;
	}
	else {
		PRINTF("\n\rperform_ref_calibration failed!");
	}

	return true;
}


/**
 * Initializes the sensors to the given address.
 */
bool vl53l0x_init(const uint8_t addr)
{
	if (!init_addresses(addr)) {
		PRINTF("\n\rinit_addresses failed!");
		return false;
	}
	else {
		PRINTF("\n\rinit_addresses passed!");
	}

	if (!init_config(addr)) {
		PRINTF("\n\rinit_config failed!");
		return false;
	}
	else {
		PRINTF("\n\rinit_config passed!");
	}

	return true;
}


/**
 * Does a single range measurement.
 */
bool vl53l0x_read_range_single(const uint8_t addr, uint16_t *range)
{
	i2c_set_slave_address(addr);
	bool success = i2c_write_addr8_data8(0x80, 0x01);
	success &= i2c_write_addr8_data8(0xFF, 0x01);
	success &= i2c_write_addr8_data8(0x00, 0x00);
	success &= i2c_write_addr8_data8(0x91, g_stop_variable);
	success &= i2c_write_addr8_data8(0x00, 0x01);
	success &= i2c_write_addr8_data8(0xFF, 0x00);
	success &= i2c_write_addr8_data8(0x80, 0x00);

	//	if (!success) {
	//		return false;
	//	}

	if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x01)) {
		PRINTF("\n\r REG_SYSRANGE_START Failed");
		return false;
	}

	uint8_t sysrange_start = 0;
	uint16_t loop_count = 0;

	do {
		success = i2c_read_addr8_data8(REG_SYSRANGE_START, &sysrange_start);
		loop_count++;
	} while (success && (sysrange_start & 0x01) && loop_count < VL53L0X_DEFAULT_MAX_LOOP);

	if (loop_count >= VL53L0X_DEFAULT_MAX_LOOP) {
		PRINTF("\n\r VL53L0X_DEFAULT_MAX_LOOP Failed");
		return false;
	}

	//	if (!success) {
	//		return false;
	//	}

	uint8_t interrupt_status = 0;

	do {
		success = i2c_read_addr8_data8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status);

		// Can add delay 5ms
	} while (success && ((interrupt_status & 0x07) == 0));

	if (interrupt_status & 0x18) {
		PRINTF("\n\rVL53L0X_ERROR_RANGE_ERROR");
		return false;
	}

	//	if (!success) {
	//		return false;
	//	}

	uint8_t localBuffer[12];
	if (!i2c_read_addr8_bytes(REG_RESULT_RANGE_STATUS, localBuffer, 12)) {
		PRINTF("\n\r REG_RESULT_RANGE_STATUS Failed");
		return false;
	}

	if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
		PRINTF("\n\r REG_SYSTEM_INTERRUPT_CLEAR Failed");
		return false;
	}

	*range = (uint16_t)((((uint16_t)(localBuffer[10])) << 8) + (uint16_t)localBuffer[11]);

	/* 8190 or 8191 may be returned when obstacle is out of range. */
	if (*range == 8190 || *range == 8191) {
		*range = VL53L0X_OUT_OF_RANGE;
	}

	return true;
}
