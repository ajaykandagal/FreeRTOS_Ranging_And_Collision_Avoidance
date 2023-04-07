# Project: FreeRTOS Ranging and Collision Avoidance
---
#### **Author:** Ajaykumar Kandagal
#### **Email-Id:** ajaykandagal94@gmail.com
#### **Description:** This is a FreeRTOS based application for ranging using vl53l0x ToF sensor. The application measures the distance from an obstacle using vl53l0x ToF sensor and based on the proximity turns on/off the buzzer.
#### **Date:** Dec 09, 2022
---
### Info
* Proof for test results are placed under Report/Testing
* Proof for working of the implementation is placed under Report/Demo
---
### Status
* I was planning to implement some extra features but due to time contraints I could not carry out them fully hence the code is written to handle two ToF sensors even though current setup uses only one sensor.
---
### Setup
* To trigger the test functionality set the macro TESTING in source/common.h
* Debug logs can be enabled by setting the macro DEBUG_LOGS in source/common.h
* The vl53l0x ToF sensor communicates over I2C1 and the SDA pin should be connected to PORTE PIN1 (J2 18) and SCL pin should be connected to PORTE PIN0 (J2 20).
* The buzzer uses TPM0 Channel 0 and should be connected on PORTD PIN0 (J2 06).
---
### Working
* There are three RTOS tasks: Reading the range, Processing range data and Driving the buzzer
* Reading the range: Reads the range data from ToF sensor and only sends over queue if the the range value is valid.
* Processing range data: The range data mapped to 5 sections based on the proximity of the obstacle. And if the proximity of the obsatcle is close enough then the buzzer task will be resumed.
* Driving the buzzer: Based on the proximity of the obstacle the on/off periods of the buzzer are calculated. Then the buzzer is played with the calculated on/off periods.
---
### Challeneges
* The vl53l0x has a complex register mapping and the datasheet from STM does not mention about register mapping but instead provide API library which is quite cumbersome. I was able to find more simplified code from which I could figure out how the sensor works and modify the code accordingly.
---
### Learnings
* Setting up I2C and performing read and write opreations.
* Using TPM to create different tones via buzzer.
* Creating FreeRTOS tasks and managing them. Learnt how setting different priorities affects the system behaviour. Used QueueHandle_t to send and receive data between tasks.
