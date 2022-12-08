/*******************************************************************************
 * @file	i2c.h
 * @brief 	This file contains functions to initialize I2C1 at 400kHz, send and
 * 			receive data over I2C.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 01, 2022
 ******************************************************************************/
#include "MKL25Z4.h"


/*******************************************************************************
 * Initializes I2C1 bus at 400kHz, PORTE PIN0 as SDA and PORTE PIN1 as SCL.
 ******************************************************************************/
void i2c_init();


/*******************************************************************************
 * Sets the I2C device address for a session. This function should be called
 * before initiating any I2C communication.
 ******************************************************************************/
void i2c_set_slave_address(const uint8_t addr);


/*******************************************************************************
 * Writes a byte to the given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device to which data will be
 * 				written.
 * 	*byte		Address from which data will be written to the register.
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_write_addr8_data8(uint8_t reg_addr, uint8_t byte);


/*******************************************************************************
 * Writes 4 bytes to the given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device to which data will be
 * 				written.
 * 	*bytes		Address from which data will be written to the register.
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_write_addr8_data32(uint8_t reg_addr, uint32_t bytes);


/*******************************************************************************
 * Writes give number of bytes to the given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device to which data will be
 * 				written.
 * 	*bytes		Address from which data will be written to the register.
 * 	byte_count	Number of bytes to be written.
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_write_addr8_bytes(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count);


/*******************************************************************************
 * Reads a byte from given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device from which value will be
 * 				read.
 * 	*byte		Address to which the register value read will be stored
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_read_addr8_data8(uint8_t reg_addr, uint8_t *byte);


/*******************************************************************************
 * Reads 2 bytes from given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device from which value will be
 * 				read.
 * 	*bytes		Address to which the register value read will be stored
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_read_addr8_data16(uint8_t reg_addr, uint16_t *bytes);


/*******************************************************************************
 * Reads 4 bytes from given register address over I2C.
 *
 * @param
 * 	reg_addr	Address of register on target device from which value will be
 * 				read.
 * 	*bytes		Address to which the register value read will be stored
 *
 * @return
 * 	Returns true if the communication is successful else returns 0.
 *
 ******************************************************************************/
uint8_t i2c_read_addr8_data32(uint8_t reg_addr, uint32_t *bytes);


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
uint8_t i2c_read_addr8_bytes(uint8_t reg_addr, uint8_t *bytes, uint16_t byte_count);
