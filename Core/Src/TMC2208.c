/*
 * TMC2208.c
 *
 *  Created on: Jan 2, 2025
 *      Author: Lucas
 */

#include <TMC2208.h>



// UART handler to be initialized externally
UART_HandleTypeDef *tmc2208_huart;

// Rsense value to be initialized during setup
float tmc2208_Rsense;

// Helper Function: Calculate checksum
uint8_t TMC2208_CalculateChecksum(uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Function 1: TMC2208 Initialization
void TMC2208_Init(UART_HandleTypeDef *huart, float Rsense) {
    tmc2208_huart = huart; // Assign the UART handler
    tmc2208_Rsense = Rsense; // Assign Rsense value
}

// Function 2: Write to a TMC2208 register
void TMC2208_WriteRegister(uint8_t reg, uint32_t value) {
    uint8_t packet[7];

    // Packet structure: Sync (0x05), Slave Address (0x00), Register, Value (4 bytes), Checksum
    packet[0] = 0x05; // Sync byte
    packet[1] = 0x00; // Slave address (default)
    packet[2] = reg; // Register address
    packet[3] = (value >> 24) & 0xFF; // Value byte 1 (MSB)
    packet[4] = (value >> 16) & 0xFF; // Value byte 2
    packet[5] = (value >> 8) & 0xFF;  // Value byte 3
    packet[6] = value & 0xFF;         // Value byte 4 (LSB)

    // Calculate checksum (XOR of all bytes except Sync)
    uint8_t checksum = TMC2208_CalculateChecksum(&packet[1], 6);

    // Append checksum
    packet[6] = checksum;

    // Transmit the packet
    HAL_UART_Transmit(tmc2208_huart, packet, 7, HAL_MAX_DELAY);
}

// Function 3: Read from a TMC2208 register
uint32_t TMC2208_ReadRegister(uint8_t reg) {
    uint8_t request[4];
    uint8_t response[8];

    // Construcción del paquete de solicitud
    request[0] = 0x05; // Sync byte
    request[1] = 0x00; // Dirección del esclavo (default)
    request[2] = reg | 0x80; // Dirección del registro con bit de lectura
    request[3] = TMC2208_CalculateChecksum(&request[1], 2); // Checksum

    // Transmitir la solicitud
    if (HAL_UART_Transmit(tmc2208_huart, request, 4, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFFFFFF; // Error en la transmisión
    }

    // Recibir respuesta (esperar un breve tiempo antes)
    HAL_Delay(2); // Pequeña espera para que el TMC2208 responda

    if (HAL_UART_Receive(tmc2208_huart, response, 8, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFFFFFF; // Error en la recepción
    }

    // Validar checksum de la respuesta
    uint8_t checksum = TMC2208_CalculateChecksum(response, 7);
    if (checksum != response[7]) {
        return 0xFFFFFFFF; // Checksum incorrecto
    }

    // Extraer el valor del registro
    uint32_t value = (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6];
    return value;
}

// Function 4: Set microstepping resolution
void TMC2208_SetMicrostepping(uint16_t microsteps) {
    uint8_t reg = 0x00; // CHOPCONF register
    uint32_t value = TMC2208_ReadRegister(reg);

    if (value == 0xFFFFFFFF) {
        return; // Error reading the register
    }

    // Mask and set the microstepping bits (MRES)
    value &= ~(0x0F << 24); // Clear MRES bits

    switch (microsteps) {
        case 256: value |= (0x00 << 24); break;
        case 128: value |= (0x01 << 24); break;
        case 64:  value |= (0x02 << 24); break;
        case 32:  value |= (0x03 << 24); break;
        case 16:  value |= (0x04 << 24); break;
        case 8:   value |= (0x05 << 24); break;
        case 4:   value |= (0x06 << 24); break;
        case 2:   value |= (0x07 << 24); break;
        case 1:   value |= (0x08 << 24); break;
        default: return; // Invalid microstep value
    }

    TMC2208_WriteRegister(reg, value);
}

// Function 5: Set motor current
void TMC2208_SetMotorCurrent(uint16_t milliamps) {
    uint8_t reg = 0x10; // IHOLD_IRUN register
    uint32_t value = TMC2208_ReadRegister(reg);

    if (value == 0xFFFFFFFF) {
        return; // Error reading the register
    }

    // Calculate IRUN based on desired current and Rsense
    uint8_t IRUN = (milliamps * 32) / (1000 * tmc2208_Rsense * 1.41);

    if (IRUN > 31) {
        IRUN = 31; // Limit IRUN to max value
    }

    // Mask and set the IRUN bits
    value &= ~(0x1F << 8); // Clear IRUN bits
    value |= (IRUN << 8);  // Set new IRUN value

    TMC2208_WriteRegister(reg, value);
}

// Function 6: Configure operation mode
void TMC2208_SetOperationMode(bool enableStealthChop) {
    uint8_t reg = 0x00; // GCONF register
    uint32_t value = TMC2208_ReadRegister(reg);

    if (value == 0xFFFFFFFF) {
        return; // Error reading the register
    }

    if (enableStealthChop) {
        value |= (1 << 7); // Enable stealthChop (EN_PWM_MODE)
    } else {
        value &= ~(1 << 7); // Disable stealthChop
    }

    TMC2208_WriteRegister(reg, value);
}

// Function 7: Configure PWM frequency for speed control
void TMC2208_SetPWMFrequency(uint8_t pwm_autoscale, uint8_t pwm_freq) {
    uint8_t reg = 0x10; // PWMCONF register
    uint32_t value = TMC2208_ReadRegister(reg);

    if (value == 0xFFFFFFFF) {
        return; // Error reading the register
    }

    // Configure PWM_AUTOSCALE (bit 15) and PWM_FREQ (bits 0-1)
    if (pwm_autoscale) {
        value |= (1 << 15); // Enable automatic scaling
    } else {
        value &= ~(1 << 15); // Disable automatic scaling
    }

    value &= ~(0x03); // Clear PWM_FREQ bits
    value |= (pwm_freq & 0x03); // Set PWM frequency

    TMC2208_WriteRegister(reg, value);
}

// Function 8: Configure StallGuard parameters
void TMC2208_SetStallGuard(uint8_t sgthrs, uint32_t tcoolthrs) {
    // Set SGTHRS (sensitivity threshold for StallGuard)
    uint8_t sgthrs_reg = 0x40; // SGTHRS register
    TMC2208_WriteRegister(sgthrs_reg, sgthrs);

    // Set TCOOLTHRS (CoolStep threshold speed)
    uint8_t tcoolthrs_reg = 0x14; // TCOOLTHRS register
    TMC2208_WriteRegister(tcoolthrs_reg, tcoolthrs);
}

// Function 9: Read StallGuard result
uint16_t TMC2208_GetStallGuardResult(void) {
    uint8_t sg_result_reg = 0x41; // SG_RESULT register
    uint32_t value = TMC2208_ReadRegister(sg_result_reg);

    if (value == 0xFFFFFFFF) {
        return 0xFFFF; // Indicate error with an invalid value
    }

    return (uint16_t)(value & 0x3FF); // SG_RESULT is 10 bits
}

// Función para escribir y luego verificar un registro
uint8_t TMC2208_VerifyRegister(uint8_t reg, uint32_t value) {
    // Escribir el valor en el registro
    TMC2208_WriteRegister(reg, value);

    // Leer el registro después de la escritura
    uint32_t readValue = TMC2208_ReadRegister(reg);

    // Comparar los valores escritos y leídos
    if (readValue == value) {
        return 1; // Éxito, el valor fue escrito correctamente
    } else {
        return 0; // Error, los valores no coinciden
    }
}






