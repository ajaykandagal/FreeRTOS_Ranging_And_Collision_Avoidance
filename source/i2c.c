/*******************************************************************************
 * @file	i2c.c
 * @brief 	This file contains functions to initialize I2C1 at 400kHz, send and
 * 			receive data over I2C.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 01, 2022
 ******************************************************************************/
#include "i2c.h"


#define I2C1_SCL_PIN 1
#define I2C1_SDA_PIN 0
#define DATA_BUS_SPEED 24000000U


// Macro functions to handle I2C1 communications
#define I2C_DisableAck()       I2C1->C1 |= I2C_C1_TXAK_MASK
#define I2C_EnableAck()        I2C1->C1 &= ~I2C_C1_TXAK_MASK
#define I2C_RepeatedStart()    I2C1->C1 |= I2C_C1_RSTA_MASK
#define I2C_EnterRxMode()      I2C1->C1 &= ~I2C_C1_TX_MASK
#define I2C_write_byte(data)   I2C1_D = data

#define I2C_Start()            I2C1->C1 |= I2C_C1_TX_MASK;\
		I2C1->C1 |= I2C_C1_MST_MASK

#define I2C_Stop()             I2C1->C1 &= ~I2C_C1_MST_MASK;\
		I2C1->C1 &= ~I2C_C1_TX_MASK

#define I2C_Wait()             while((I2C1->S & I2C_S_IICIF_MASK)==0); \
		I2C1->S |= I2C_S_IICIF_MASK;


// I2C device address for a session
uint8_t g_nxp_device_addr = 0x00;


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Initializes I2C1 bus at 400kHz, PORTE PIN0 as SDA and PORTE PIN1 as SCL.
 ******************************************************************************/
void i2c_init()
{
	// Enable clock gating for I2C1 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// Set I2C pins
	PORTE->PCR[I2C1_SCL_PIN] |= PORT_PCR_MUX(6);
	PORTE->PCR[I2C1_SDA_PIN] |= PORT_PCR_MUX(6);

	// Setting baud-rate to 400kHz
	I2C1->F = I2C_F_ICR(0x11) | I2C_F_MULT(0);

	// Disable DMA transfer
	I2C1->C1 &= ~I2C_C1_DMAEN_MASK;

	// Enable I2C1 interrupts
	I2C1->C1 = I2C_C2_HDRS_MASK | I2C_C1_IICEN_MASK;
}


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Sets the I2C device address for a session. This function should be called
 * before initiating any I2C communication.
 ******************************************************************************/
void i2c_set_slave_address(const uint8_t addr)
{
	g_nxp_device_addr = addr;
}


/*******************************************************************************
 * Creates a small delay between I2C writes and reads
 *
 * @param
 * 	val		Amount a for loop iteration delay to be created.
 *
 ******************************************************************************/
void i2c_delay(uint16_t val)
{
	for (int i = 0; i < val; i++);
}


/*******************************************************************************
 * Writes give number of bytes to the given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device to which data will be
 * 				written.
 * 	*bytes		Address from which data will be written to the register.
 * 	byte_count	Number of bytes to be written.
 *
 ******************************************************************************/
void i2c_write_write(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count)
{
	I2C_Start();
	I2C1->D = g_nxp_device_addr << 1;					/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();

	I2C1->D = reg_addr;									/* Send register address */
	I2C_Wait();

	for (uint16_t i = 0; i < byte_count; i++)
	{
		I2C1->D = bytes[i];								/* Send the data */
		I2C_Wait();
	}

	I2C_Stop();

	i2c_delay(50);
}


/*******************************************************************************
 * Reads given number of bytes from given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device from which value will be
 * 				read.
 * 	*bytes		Address to which the register value read will be stored.
 *	byte_count	Number of bytes to be read.
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
void i2c_read_read(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count)
{
	I2C_Start();	          
	I2C1->D = g_nxp_device_addr << 1;					/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();										

	I2C1->D = reg_addr;									/* Send register address */
	I2C_Wait();

	I2C_RepeatedStart();

	I2C1->D = (g_nxp_device_addr << 1) | 0x01;			/* Send I2C device address this time with W/R bit = 1 */
	I2C_Wait();	

	I2C_EnterRxMode();
	I2C_EnableAck();

	*bytes = I2C1->D;
	I2C_Wait();

	for(uint16_t byte_index = 0; byte_index < (byte_count - 2); byte_index++)
	{
		*bytes = I2C1->D;
		bytes++;
		I2C_Wait();
	}

	I2C_DisableAck();
	*bytes = I2C1->D;
	bytes++;
	I2C_Wait();
	I2C_Stop();
	*bytes = I2C1->D;

	i2c_delay(50);
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Writes a byte to the given register address over I2C.
 ******************************************************************************/
uint8_t i2c_write_addr8_data8(uint8_t reg_addr, uint8_t byte)
{
	I2C_Start();
	I2C1->D = g_nxp_device_addr << 1;				/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();

	I2C1->D = reg_addr;								/* Send register address */
	I2C_Wait();

	I2C1->D = byte;									/* Send the data */
	I2C_Wait();

	I2C_Stop();

	i2c_delay(50);

	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Writes 4 bytes to the given register address over I2C.
 ******************************************************************************/
uint8_t i2c_write_addr8_data32(uint8_t reg_addr, uint32_t bytes)
{
	i2c_write_write(reg_addr, (uint8_t*)&bytes, 4);
	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Writes give number of bytes to the given register address over I2C.
 ******************************************************************************/
uint8_t i2c_write_addr8_bytes(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count)
{
	i2c_write_write(reg_addr, (uint8_t*)bytes, byte_count);
	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Reads a byte from given register address over I2C.
 ******************************************************************************/
uint8_t i2c_read_addr8_data8(uint8_t reg_addr, uint8_t *byte)
{
	I2C_Start();
	I2C1->D = g_nxp_device_addr << 1;					/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();

	I2C1->D = reg_addr;									/* Send register address */
	I2C_Wait();

	I2C_RepeatedStart();

	I2C1->D = (g_nxp_device_addr << 1) | 0x01;			/* Send I2C device address this time with W/R bit = 1 */
	I2C_Wait();

	I2C_EnterRxMode();
	I2C_DisableAck();

	*byte = I2C1->D;
	I2C_Wait();
	I2C_Stop();
	*byte = I2C1->D;
	i2c_delay(500);

	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Reads 2 bytes from given register address over I2C.
 ******************************************************************************/
uint8_t i2c_read_addr8_data16(uint8_t reg_addr, uint16_t *bytes)
{
	i2c_read_read(reg_addr, (uint8_t*)bytes, 2);
	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Reads 4 bytes from given register address over I2C.
 ******************************************************************************/
uint8_t i2c_read_addr8_data32(uint8_t reg_addr, uint32_t *bytes)
{
	i2c_read_read(reg_addr, (uint8_t*)bytes, 2);
	return 1;
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * Reads given number of bytes from given register address over I2C.
 ******************************************************************************/
uint8_t i2c_read_addr8_bytes(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count)
{
	i2c_read_read(reg_addr, (uint8_t*)bytes, byte_count);
	return 1;
}
