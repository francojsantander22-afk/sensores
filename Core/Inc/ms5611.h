/* ms5611.h */
#ifndef MS5611_H
#define MS5611_H

#include "stm32wlxx_hal.h"
#include <stdint.h>

/* Dirección I2C (CSB → VCC = 0x77, shifted para HAL) */
#define MS5611_I2C_ADDR     (0x76 << 1)

/* Comandos */
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48   /* Presión,     OSR=4096 */
#define MS5611_CMD_CONV_D2_4096 0x58   /* Temperatura, OSR=4096 */
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_BASE    0xA2   /* C1 en 0xA2, C2 en 0xA4 ... C6 en 0xAC */

/* Handle del sensor */
typedef struct {
    uint16_t C[7];      /* C[1]..C[6] = coeficientes PROM */
    int32_t  TEMP;      /* Temperatura compensada ×100 (°C) */
    int32_t  P;         /* Presión compensada ×100 (mbar)   */
} MS5611_t;

/* Funciones públicas */
HAL_StatusTypeDef MS5611_Init(MS5611_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MS5611_Read(MS5611_t *dev, I2C_HandleTypeDef *hi2c);

#endif /* MS5611_H */
