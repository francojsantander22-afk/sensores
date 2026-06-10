/* ms5611.c */
#include "ms5611.h"
#include "stm32wlxx_hal.h"

/* ── Enviar un comando de 1 byte ─────────────────────────────────────── */
static HAL_StatusTypeDef MS5611_SendCmd(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(hi2c, MS5611_I2C_ADDR, &cmd, 1, 10);
}

/* ── Leer N bytes después de haber enviado un comando ───────────────── */
static HAL_StatusTypeDef MS5611_ReadBytes(I2C_HandleTypeDef *hi2c,
                                          uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Master_Receive(hi2c, MS5611_I2C_ADDR, buf, len, 10);
}

/* ── Leer resultado ADC (24 bits) ───────────────────────────────────── */
static HAL_StatusTypeDef MS5611_ReadADC(I2C_HandleTypeDef *hi2c,
                                         uint32_t *result)
{
    uint8_t buf[3];
    HAL_StatusTypeDef st = MS5611_SendCmd(hi2c, MS5611_CMD_ADC_READ);
    if (st != HAL_OK) return st;
    st = MS5611_ReadBytes(hi2c, buf, 3);
    if (st != HAL_OK) return st;
    *result = ((uint32_t)buf[0] << 16) |
              ((uint32_t)buf[1] <<  8) |
               (uint32_t)buf[2];
    return HAL_OK;
}

/* ── Init: Reset + leer 6 coeficientes PROM ────────────────────────── */
HAL_StatusTypeDef MS5611_Init(MS5611_t *dev, I2C_HandleTypeDef *hi2c)
{
    /* Reset */
    HAL_StatusTypeDef st = MS5611_SendCmd(hi2c, MS5611_CMD_RESET);
    if (st != HAL_OK) return st;
    HAL_Delay(5);   /* datasheet: 2.8 ms mínimo */

    /* Leer C1..C6 (cada uno en 2 bytes, direcciones 0xA2, 0xA4 .. 0xAC) */
    for (uint8_t i = 1; i <= 6; i++)
    {
        uint8_t cmd = MS5611_CMD_PROM_BASE + (i - 1) * 2; /* 0xA2,A4,A6,A8,AA,AC */
        uint8_t buf[2];
        st = MS5611_SendCmd(hi2c, cmd);
        if (st != HAL_OK) return st;
        st = MS5611_ReadBytes(hi2c, buf, 2);
        if (st != HAL_OK) return st;
        dev->C[i] = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return HAL_OK;
}

/* ── Read: medir D1 + D2 y calcular TEMP y P compensados ───────────── */
HAL_StatusTypeDef MS5611_Read(MS5611_t *dev, I2C_HandleTypeDef *hi2c)
{
    uint32_t D1, D2;

    /* 1. Convertir presión (D1) */
    MS5611_SendCmd(hi2c, MS5611_CMD_CONV_D1_4096);
    HAL_Delay(10);   /* OSR=4096: máximo 9.04 ms */
    if (MS5611_ReadADC(hi2c, &D1) != HAL_OK) return HAL_ERROR;

    /* 2. Convertir temperatura (D2) */
    MS5611_SendCmd(hi2c, MS5611_CMD_CONV_D2_4096);
    HAL_Delay(10);
    if (MS5611_ReadADC(hi2c, &D2) != HAL_OK) return HAL_ERROR;

    /* 3. Cálculo temperatura — datasheet pág. 8 */
    int32_t dT   = (int32_t)D2 - (int32_t)dev->C[5] * 256;
    int32_t TEMP = 2000 + ((int64_t)dT * dev->C[6]) / 8388608;

    /* 4. Compensación de offset y sensibilidad */
    int64_t OFF  = (int64_t)dev->C[2] * 65536
                 + ((int64_t)dev->C[4] * dT) / 128;
    int64_t SENS = (int64_t)dev->C[1] * 32768
                 + ((int64_t)dev->C[3] * dT) / 256;

    /* 5. Segunda compensación (T < 20°C) — importante para altitud */
    int32_t T2    = 0;
    int64_t OFF2  = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000)
    {
        T2    = ((int64_t)dT * dT) / 2147483648U;   /* dT² / 2^31 */
        OFF2  = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) / 4;

        if (TEMP < -1500)   /* T < −15°C: corrección adicional */
        {
            OFF2  += 7  * (int64_t)(TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11 * (int64_t)(TEMP + 1500) * (TEMP + 1500) / 2;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    /* 6. Presión compensada */
    int32_t P = ((int64_t)D1 * SENS / 2097152 - OFF) / 32768;

    dev->TEMP = TEMP;   /* ej: 2007  → 20.07 °C  */
    dev->P    = P;      /* ej: 100009 → 1000.09 mbar */
    return HAL_OK;
}
