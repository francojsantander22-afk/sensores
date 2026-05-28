/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <bmi270_config.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* --- Registros BMI270 --- */
#define BMI270_I2C_ADDR     (0x68 << 1)
#define REG_CHIP_ID         0x00
#define REG_PWR_CONF        0x7C
#define REG_INIT_CTRL       0x59
#define REG_INIT_DATA       0x5E
#define REG_INTERNAL_STATUS 0x21
#define REG_PWR_CTRL        0x7D
#define REG_ACC_CONF        0x40
#define REG_ACC_RANGE       0x41
#define REG_DATA_8          0x0C
#define REG_TEMP_LSB        0x22

/* --- Tags del protocolo TLV --- */
#define TLV_TAG_TIME        0x01
#define TLV_TAG_GPS_COORD   0x02
#define TLV_TAG_ALTITUDE    0x03
#define TLV_TAG_ACCEL       0x04
#define TLV_TAG_TEMP        0x05
#define TLV_TAG_GYRO        0x06   /* Giroscopio: nuevo tag */

/* --- Estructura de datos GPS parseados --- */
typedef struct {
	uint8_t valid; /* 1 = fix confirmado, 0 = sin fix */
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int32_t lat; /* Grados decimales × 100000 */
	int32_t lon;
	int16_t altitude; /* Metros sobre el nivel del mar */
} GpsData_t;

/* --- Estructura de datos IMU --- */
typedef struct {
	int16_t acc_x, acc_y, acc_z;
	int16_t gyr_x, gyr_y, gyr_z;
	float temperature;
} ImuData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- Buffer circular UART para GPS --- */
#define RX_BUF_SIZE 512
extern volatile uint8_t rx_buffer[RX_BUF_SIZE];
extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;
volatile uint8_t tlv_ready = 0;
extern volatile uint8_t rx_byte; // byte temporal que llena la ISR

/* --- Buffer de línea NMEA --- */
char line_buffer[100];
uint8_t line_index = 0;

/* --- Buffer TLV para transmisión LoRa --- */
uint8_t tlv_buf[64];
uint8_t tlv_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Utilidades UART */
void UART_Print(const char *msg);

/* BMI270 */
void BMI270_WriteReg(uint8_t reg, uint8_t val);
uint8_t BMI270_ReadReg(uint8_t reg);
void BMI270_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef BMI270_LoadConfigFile(void);
float BMI270_ReadTemperature(void);

/* TLV */
static void tlv_reset(void);
static uint8_t tlv_space_left(uint8_t needed);
static void tlv_pack_time(uint8_t h, uint8_t m, uint8_t s);
static void tlv_pack_coords(int32_t lat, int32_t lon);
static void tlv_pack_int16(uint8_t tag, int16_t val);
static void tlv_pack_vec3(uint8_t tag, int16_t x, int16_t y, int16_t z);

/* GPS / NMEA */
static uint8_t nmea_get_field(const char *nmea, uint8_t field_num, char *out,
		uint8_t out_size);
static int32_t nmea_coord_to_int32(const char *coord, char dir);
static uint8_t gps_parse_gga(const char *nmea, GpsData_t *out);

/* IMU */
static void imu_read(ImuData_t *out);

/* Telemetría */
void telemetry_build_packet(const GpsData_t *gps, const ImuData_t *imu);
void telemetry_debug_print(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =========================================================
 *  UART
 * ========================================================= */
void UART_Print(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/* =========================================================
 *  BMI270 – acceso I2C
 * ========================================================= */
void BMI270_WriteReg(uint8_t reg, uint8_t val) {
	char dbg[70];
	snprintf(dbg, sizeof(dbg), "   [I2C] Escribiendo registro 0x%02X... ", reg);
	UART_Print(dbg);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &val, 1, 100);

	snprintf(dbg, sizeof(dbg), "Status HAL: %d\r\n", status);
	UART_Print(dbg);

	for (volatile int k = 0; k < 50000; k++)
		;
}

uint8_t BMI270_ReadReg(uint8_t reg) {
	uint8_t val = 0;
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
	return val;
}

void BMI270_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef BMI270_LoadConfigFile(void) {
	UART_Print("   -> Configurando punteros 0x5B y 0x5C...\r\n");
	BMI270_WriteReg(0x5B, 0x00);
	BMI270_WriteReg(0x5C, 0x00);

	UART_Print("   -> Enviando 8KB de un solo golpe (Burst Write)...\r\n");

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, BMI270_I2C_ADDR,
	REG_INIT_DATA, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bmi270_config_file,
			bmi270_config_file_size, 1000);

	if (status != HAL_OK) {
		char err[60];
		snprintf(err, sizeof(err),
				"   [!] Error I2C en burst write. Codigo HAL: %d\r\n", status);
		UART_Print(err);
		return status;
	}

	UART_Print("   -> Transmision completada.\r\n");
	return HAL_OK;
}

float BMI270_ReadTemperature(void) {
	uint8_t raw[2];
	BMI270_ReadRegs(REG_TEMP_LSB, raw, 2);
	int16_t raw_temp = (int16_t) ((raw[1] << 8) | raw[0]);
	return (raw_temp / 512.0f) + 23.0f;
}

/* =========================================================
 *  TLV – empaquetado con guard de overflow
 * ========================================================= */
void tlv_reset(void) {
	tlv_len = 0;
	memset(tlv_buf, 0, sizeof(tlv_buf));
}

uint8_t tlv_space_left(uint8_t needed) {
	return (tlv_len + needed) <= (uint8_t) sizeof(tlv_buf);
}

void tlv_pack_time(uint8_t h, uint8_t m, uint8_t s) {
	if (!tlv_space_left(5))
		return;
	tlv_buf[tlv_len++] = TLV_TAG_TIME;
	tlv_buf[tlv_len++] = 3;
	tlv_buf[tlv_len++] = h;
	tlv_buf[tlv_len++] = m;
	tlv_buf[tlv_len++] = s;
}

void tlv_pack_coords(int32_t lat, int32_t lon) {
	if (!tlv_space_left(10))
		return;
	tlv_buf[tlv_len++] = TLV_TAG_GPS_COORD;
	tlv_buf[tlv_len++] = 8;
	tlv_buf[tlv_len++] = (lat >> 24) & 0xFF;
	tlv_buf[tlv_len++] = (lat >> 16) & 0xFF;
	tlv_buf[tlv_len++] = (lat >> 8) & 0xFF;
	tlv_buf[tlv_len++] = lat & 0xFF;
	tlv_buf[tlv_len++] = (lon >> 24) & 0xFF;
	tlv_buf[tlv_len++] = (lon >> 16) & 0xFF;
	tlv_buf[tlv_len++] = (lon >> 8) & 0xFF;
	tlv_buf[tlv_len++] = lon & 0xFF;
}

void tlv_pack_int16(uint8_t tag, int16_t val) {
	if (!tlv_space_left(4))
		return;
	tlv_buf[tlv_len++] = tag;
	tlv_buf[tlv_len++] = 2;
	tlv_buf[tlv_len++] = (val >> 8) & 0xFF;
	tlv_buf[tlv_len++] = val & 0xFF;
}

void tlv_pack_vec3(uint8_t tag, int16_t x, int16_t y, int16_t z) {
	if (!tlv_space_left(8))
		return;
	tlv_buf[tlv_len++] = tag;
	tlv_buf[tlv_len++] = 6;
	tlv_buf[tlv_len++] = (x >> 8) & 0xFF;
	tlv_buf[tlv_len++] = x & 0xFF;
	tlv_buf[tlv_len++] = (y >> 8) & 0xFF;
	tlv_buf[tlv_len++] = y & 0xFF;
	tlv_buf[tlv_len++] = (z >> 8) & 0xFF;
	tlv_buf[tlv_len++] = z & 0xFF;
}

/* =========================================================
 *  GPS – parseo NMEA robusto
 * ========================================================= */

/*
 * Extrae el campo número field_num (base 0 = tipo sentencia) de una cadena NMEA.
 * Retorna 1 si el campo existe y no está vacío, 0 en caso contrario.
 */
static uint8_t nmea_get_field(const char *nmea, uint8_t field_num, char *out,
		uint8_t out_size) {
	uint8_t commas = 0, j = 0;
	for (uint8_t i = 0; nmea[i] != '\0' && nmea[i] != '*'; i++) {
		if (nmea[i] == ',') {
			commas++;
			continue;
		}
		if (commas == field_num) {
			if (j < out_size - 1)
				out[j++] = nmea[i];
		}
		if (commas > field_num)
			break;
	}
	out[j] = '\0';
	return (j > 0) ? 1 : 0;
}

/*
 * Convierte un campo de coordenada NMEA (DDDMM.MMMMM) a grados decimales × 100000.
 * dir: 'N','S','E','W'
 */
static int32_t nmea_coord_to_int32(const char *coord, char dir) {
	if (coord[0] == '\0')
		return 0;
	float raw = atof(coord);
	int deg = (int) (raw / 100.0f);
	float dec_deg = deg + (raw - (float) (deg * 100)) / 60.0f;
	if (dir == 'S' || dir == 'W')
		dec_deg = -dec_deg;
	return (int32_t) (dec_deg * 100000.0f);
}

/*
 * Parsea una sentencia GGA y llena gps_out.
 * Retorna 1 si hay fix válido, 0 si no.
 */
static uint8_t gps_parse_gga(const char *nmea, GpsData_t *gps_out) {
	memset(gps_out, 0, sizeof(GpsData_t));

	char fix[2] = { 0 };
	if (!nmea_get_field(nmea, 6, fix, sizeof(fix)))
		return 0;
	if (fix[0] < '1')
		return 0; /* 0 = sin fix */

	char time_s[12] = { 0 };
	char lat_s[14] = { 0 }, ns[2] = { 0 };
	char lon_s[14] = { 0 }, ew[2] = { 0 };
	char alt_s[10] = { 0 };

	nmea_get_field(nmea, 1, time_s, sizeof(time_s));
	nmea_get_field(nmea, 2, lat_s, sizeof(lat_s));
	nmea_get_field(nmea, 3, ns, sizeof(ns));
	nmea_get_field(nmea, 4, lon_s, sizeof(lon_s));
	nmea_get_field(nmea, 5, ew, sizeof(ew));
	nmea_get_field(nmea, 9, alt_s, sizeof(alt_s));

	/* Hora UTC → ARG (UTC-3), con wrap en medianoche */
	int utc_h = (time_s[0] - '0') * 10 + (time_s[1] - '0');
	gps_out->hour = (uint8_t) ((utc_h - 3 + 24) % 24);
	gps_out->minute = (uint8_t) ((time_s[2] - '0') * 10 + (time_s[3] - '0'));
	gps_out->second = (uint8_t) ((time_s[4] - '0') * 10 + (time_s[5] - '0'));

	gps_out->lat = nmea_coord_to_int32(lat_s, ns[0]);
	gps_out->lon = nmea_coord_to_int32(lon_s, ew[0]);
	gps_out->altitude = (int16_t) atof(alt_s);
	gps_out->valid = 1;
	return 1;
}

/* =========================================================
 *  IMU – lectura BMI270 (accel + gyro + temp)
 * ========================================================= */
static void imu_read(ImuData_t *imu_out) {
	uint8_t raw[14];
	BMI270_ReadRegs(REG_DATA_8, raw, 14);

	imu_out->acc_x = (int16_t) ((raw[1] << 8) | raw[0]);
	imu_out->acc_y = (int16_t) ((raw[3] << 8) | raw[2]);
	imu_out->acc_z = (int16_t) ((raw[5] << 8) | raw[4]);
	imu_out->gyr_x = (int16_t) ((raw[7] << 8) | raw[6]);
	imu_out->gyr_y = (int16_t) ((raw[9] << 8) | raw[8]);
	imu_out->gyr_z = (int16_t) ((raw[11] << 8) | raw[10]);
	imu_out->temperature = BMI270_ReadTemperature();
}

/* =========================================================
 *  TELEMETRÍA – ensamblar paquete TLV completo
 * ========================================================= */

/*
 * Arma el paquete TLV en tlv_buf[]/tlv_len.
 * Tamaño máximo esperado:
 *   GPS: TIME(5) + COORD(10) + ALT(4)  = 19 bytes
 *   IMU: ACCEL(8) + GYRO(8) + TEMP(4)  = 20 bytes
 *   TOTAL máximo: 39 bytes  < 64 bytes del buffer
 */
void telemetry_build_packet(const GpsData_t *gps, const ImuData_t *imu) {
	tlv_reset();

	/* Bloque GPS – solo si hay fix válido */
	if (gps->valid) {
		tlv_pack_time(gps->hour, gps->minute, gps->second);
		tlv_pack_coords(gps->lat, gps->lon);
		tlv_pack_int16(TLV_TAG_ALTITUDE, gps->altitude);
	}

	/* Bloque IMU – siempre presente */
	tlv_pack_vec3(TLV_TAG_ACCEL, imu->acc_x, imu->acc_y, imu->acc_z);
	tlv_pack_vec3(TLV_TAG_GYRO, imu->gyr_x, imu->gyr_y, imu->gyr_z);
	tlv_pack_int16(TLV_TAG_TEMP, (int16_t) (imu->temperature * 100.0f));
}

/*
 * Imprime por UART el contenido del último paquete TLV ensamblado.
 * Buffer estático calculado: "TLV [" + 64×"XX " + "] 64 bytes\r\n" < 300 chars.
 */
void telemetry_debug_print(void) {
	char buf[300];
	int pos = 0;

	pos += snprintf(buf + pos, sizeof(buf) - pos, "TLV [");
	for (uint8_t i = 0; i < tlv_len && pos < (int) (sizeof(buf) - 20); i++) {
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", tlv_buf[i]);
	}
	snprintf(buf + pos, sizeof(buf) - pos, "] %d bytes\r\n", tlv_len);
	UART_Print(buf);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration -------------------------------------------------------*/
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SubGHz_Phy_Init();
	MX_I2C3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	HAL_Delay(1000);
	UART_Print("Inicializando sistema...\r\n");

	/* USER CODE BEGIN 2 */

	/* --- Inicialización BMI270 --- */
	uint8_t chip_id = BMI270_ReadReg(REG_CHIP_ID);
	;
	/* WORKAROUND: En STM32WL la USART1 requiere doble llamada a Receive_IT
	 * para comenzar a generar interrupciones correctamente.
	 * La segunda llamada retorna HAL_BUSY, lo cual es esperado e inofensivo. Si se hace una sola llamada, el sistema nunca habilita las interrupciones de la UART1*/
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
	if (chip_id != 0x24) {
		char err[60];
		snprintf(err, sizeof(err),
				"ERROR: BMI270 CHIP_ID=0x%02X (esperado 0x24)\r\n", chip_id);
		UART_Print(err);
	} else {
		BMI270_WriteReg(REG_PWR_CONF, 0x00);
		HAL_Delay(1);
		BMI270_WriteReg(REG_INIT_CTRL, 0x00);
		HAL_Delay(1);
		BMI270_LoadConfigFile();
		HAL_Delay(20);
		BMI270_WriteReg(REG_INIT_CTRL, 0x01);
		HAL_Delay(20);
		BMI270_WriteReg(REG_ACC_CONF, 0xA8);
		HAL_Delay(1);
		BMI270_WriteReg(REG_ACC_RANGE, 0x02);
		HAL_Delay(1);
		BMI270_WriteReg(REG_PWR_CTRL, 0x0E);
		HAL_Delay(1);
		BMI270_WriteReg(REG_PWR_CONF, 0x02);
		HAL_Delay(2);
		BMI270_WriteReg(0x42, 0xA9);
		HAL_Delay(1);
		BMI270_WriteReg(0x43, 0x00);
		HAL_Delay(1);
		UART_Print("BMI270 OK.\r\n");
	}

	uint32_t last_gps_tick = HAL_GetTick();
	uint32_t last_imu_tick = HAL_GetTick();
	static GpsData_t last_valid_gps = { 0 };
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint8_t trigger = 0;
		/* Watchdog ISR */
		if (huart1.RxState == HAL_UART_STATE_READY) {
			HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
			HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
		}
		/* TAREA 1: GPS — solo si no hay paquete pendiente */
		if (!tlv_ready) {
			while (rx_head != rx_tail) {
				char c = (char) rx_buffer[rx_tail];
				rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

				if (c == '\n' || line_index >= sizeof(line_buffer) - 1) {
					line_buffer[line_index] = '\0';
					line_index = 0;

					if (strstr(line_buffer, "GGA") != NULL) {
						last_gps_tick = HAL_GetTick();
						GpsData_t gps_temp = { 0 };
						if (gps_parse_gga(line_buffer, &gps_temp)) {
							last_valid_gps = gps_temp; /* guardar fix válido */
						}
						trigger = 1;
						break;
					}
				} else if (c != '\r') {
					line_buffer[line_index++] = c;
				}
			}

			/* TAREA 2: Watchdog */
			if (!trigger) {
				uint32_t now = HAL_GetTick();
				if ((now - last_gps_tick > 2000)
						&& (now - last_imu_tick > 1000)) {
					last_imu_tick = now;
					trigger = 1;
				}
			} else {
				last_imu_tick = HAL_GetTick();
			}

			/* TAREA 3: Empaquetar */
			if (trigger) {
				ImuData_t imu = { 0 };
				imu_read(&imu);
				telemetry_build_packet(&last_valid_gps, &imu);
				telemetry_debug_print();
				tlv_ready = 1;
			}
		}

		MX_SubGHz_Phy_Process();
	}

	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
