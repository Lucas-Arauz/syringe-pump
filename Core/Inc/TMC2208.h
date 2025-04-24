/*
 * TMC2208.h
 *
 *  Created on: Jan 2, 2025
 *      Author: Lucas Arauz
 */


#include "stm32f1xx_hal.h"

// TMC2208 Library for STM32
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef INC_TMC2208_H_
#define INC_TMC2208_H_

/*
 * TMC2208_Init()
 *
 * Description: TMC2208 Initialization.
 * Parameters:  *huart -> pointer to uart handle
 * 				Rsense -> sense resistor value
 */

/*
 * TMC2208_WriteRegister()
 *
 * Description: write a register
 * Parameters: 	reg -> register to write
 * 				value -> value to write in the register
 */

/*
 * TMC2208_ReadRegister()
 *
 * Description: Read a register
 * Parameters:	reg -> Register to read
 * Return: uint32_t -> value of the had read register
 */

/*
 * TMC2208_SetMicrostepping()
 *
 * Description: This reads the CHOPCONF register, sets the microstepping resolution (MRES)
 * 				bits to the given value, and writes the register back
 * Parameters:	microsteps -> microsteps value (1, 2, 4, 8, 16, 32, 64, 128, 256)
 */

/*
 * TMC2208_SetMotorCurrent()
 *
 * Description: Sets the motor current by calculating the IRUN value based on the rsense resistor
 * 				and the desired current value (mA)
 */

uint8_t TMC2208_CalculateChecksum(uint8_t *data, uint8_t length);

void TMC2208_Init(UART_HandleTypeDef *huart, float Rsense);

void TMC2208_WriteRegister(uint8_t reg, uint32_t value);
uint32_t TMC2208_ReadRegister(uint8_t reg);

void TMC2208_SetMicrostepping(uint16_t microsteps);
void TMC2208_SetMotorCurrent(uint16_t milliamps);
void TMC2208_SetOperationMode(bool enableStealthChop);
void TMC2208_SetPWMFrequency(uint8_t pwm_autoscale, uint8_t pwm_freq);
void TMC2208_SetStallGuard(uint8_t sgthrs, uint32_t tcoolthrs);
uint16_t TMC2208_GetStallGuardResult(void);
uint8_t TMC2208_VerifyRegister(uint8_t reg, uint32_t value);

#endif /* INC_TMC2208_H_ */
