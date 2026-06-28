/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * --- Mapa TLV ---
 * 0x01  TIME        3B  hh mm ss
 * 0x02  GPS_COORD   8B  lat(4) lon(4) ×100000
 * 0x03  GPS_ALT     2B  metros (int16)
 * 0x04  GPS_SPEED   2B  km/h ×100 (uint16)
 * 0x05  ACCEL       6B  x y z (int16 cada uno)
 * 0x06  GYRO        6B  x y z (int16 cada uno)
 * 0x07  TEMP_SHT    2B  °C ×100 (int16)
 * 0x08  HUM_SHT     2B  % ×100 (uint16)
 * 0x09  PRESSURE    4B  mbar ×100 (uint32)
 * 0x0A  BARO_ALT    2B  metros ISA (int16)
 * 0x0B  BAT_VOLT    2B  mV (uint16)
 * 0x0C  BAT_SOC     1B  % (uint8)
 * 0x0D  MAG_ANGLE   2B  grados ×100 (uint16)
 *
 *//* <--- AQUI ESTA LA SOLUCION: Cierre del bloque */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <bmi270_config.h>
#include <stdbool.h>
#include "cmps2.h"
#include "ms5611.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Direcciones BMI270
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

typedef struct {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	float temp_BMI270;
} BMI270_Data_t;

//Direcciones SHT20
#define SHT20_I2C_ADDR   (0x40 << 1) // Dirección I2C desplazada 1 bit
#define CMD_MEASURE_T    0xF3        // Trigger T measurement (no hold master)
#define CMD_MEASURE_RH   0xF5        // Trigger RH measurement (no hold master)

#define VREF              3.3f
#define ADC_BITS          4096
#define R1                100000.0f
#define R2                220000.0f
#define NUM_MUESTRAS      10
#define INTERVALO_MS      500
#define LM35_MV_POR_C     10.0f
#define TEMP_ON           25.0f
#define TEMP_OFF          22.0f
#define ALPHA  0.1f   // factor del filtro: 0.1 = muy suave, 0.3 = más rápido
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// FIX: Los defines de LoRa se eliminaron de aquí.
// Ahora viven únicamente en subghz_phy_app.c para evitar redefinición.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_data;
uint8_t rx_data_uart2; // Variable para capturar el input del menú
uint8_t spi_tx_buffer[128];
volatile uint32_t spi_tx_start_time = 0;

static float vbatFiltrado = 0.0f;

typedef struct {
	float voltaje;
	uint8_t porcentaje;
} PuntoSoC;

static const PuntoSoC tablaSoC[] =
		{ { 4.20f, 100 }, { 4.10f, 90 }, { 4.00f, 80 }, { 3.90f, 70 }, { 3.80f,
				60 }, { 3.70f, 45 }, { 3.60f, 30 }, { 3.50f, 20 },
				{ 3.40f, 10 }, { 3.20f, 5 }, { 3.00f, 0 }, };
static const uint8_t PUNTOS_SOC = sizeof(tablaSoC) / sizeof(tablaSoC[0]);

// --- ESTADOS DEL MENÚ ---
volatile uint8_t display_mode = 2; // 1 = ASCII, 2 = TLV (Default)
volatile uint8_t menu_active = 0;  // 0 = Mostrando datos, 1 = Mostrando menú

// --- BÚFER CIRCULAR GPS ---
#define RX_BUF_SIZE 512
volatile uint8_t rx_buffer[RX_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// --- PARSEO NMEA ---
char line_buffer[100];
uint8_t line_index = 0;

// --- BÚFER LORA TLV ---
uint8_t lora_tx_buffer[128];
uint8_t lora_tx_len = 0;

// FIX: radio_tx_done vive en subghz_phy_app.c, aquí solo la declaramos extern
extern volatile uint8_t radio_tx_done;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(const char *msg);
uint32_t leerCanalADC(uint32_t canal);
float leerVoltajeBateria(void);
uint8_t voltajeASoC(float voltaje);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val,
			1, 100);
	return val;
}

void BMI270_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data,
			len, 100);
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

void UART_Print(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

float BMI270_ReadTemperature(void) {
	uint8_t raw[2];
	BMI270_ReadRegs(REG_TEMP_LSB, raw, 2);
	int16_t raw_temp = (int16_t) ((raw[1] << 8) | raw[0]);
	return (raw_temp / 512.0f) + 23.0f;
}

void BMI270_Get_Data(BMI270_Data_t *imu_data) {
	uint8_t raw[14];

	BMI270_ReadRegs(REG_DATA_8, raw, 14);

	// Guardamos los datos dentro de la estructura usando la flecha (->)
	imu_data->acc_x = (int16_t) ((raw[1] << 8) | raw[0]);
	imu_data->acc_y = (int16_t) ((raw[3] << 8) | raw[2]);
	imu_data->acc_z = (int16_t) ((raw[5] << 8) | raw[4]);

	imu_data->gyr_x = (int16_t) ((raw[7] << 8) | raw[6]);
	imu_data->gyr_y = (int16_t) ((raw[9] << 8) | raw[8]);
	imu_data->gyr_z = (int16_t) ((raw[11] << 8) | raw[10]);

	imu_data->temp_BMI270 = BMI270_ReadTemperature();
}

void get_nmea_field(const char *nmea, uint8_t field_num, char *result) {
	uint8_t comma_count = 0;
	uint8_t i = 0, j = 0;
	while (nmea[i] != '\0' && nmea[i] != '*' && comma_count <= field_num) {
		if (nmea[i] == ',') {
			comma_count++;
		} else if (comma_count == field_num) {
			result[j++] = nmea[i];
		}
		i++;
	}
	result[j] = '\0';
}

void SHT20_Read(float *temperature, float *humidity) {
	uint8_t cmd;
	uint8_t data[3];
	uint16_t raw_val;
	// 1. Leer Temperatura
	cmd = CMD_MEASURE_T;
	HAL_I2C_Master_Transmit(&hi2c3, SHT20_I2C_ADDR, &cmd, 1, 150); // Enviamos el comando de medición de temperatura
	HAL_Delay(85); // Esperamos el tiempo máximo de conversión para 14 bits (85 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT20_I2C_ADDR, data, 3, 150); // Leemos 3 bytes: MSB, LSB y CRC
	// Unimos los bytes y ponemos a '0' los últimos 2 bits de estado (0xFC = 11111100 en binario)
	raw_val = (data[0] << 8) | (data[1] & 0xFC);
	*temperature = -46.85 + 175.72 * ((float) raw_val / 65536.0); // Aplicamos la fórmula del datasheet
	// 2. Leer Humedad Relativa
	cmd = CMD_MEASURE_RH;
	HAL_I2C_Master_Transmit(&hi2c3, SHT20_I2C_ADDR, &cmd, 1, 150); // Enviamos el comando de medición de humedad
	HAL_Delay(29); // Esperamos el tiempo máximo de conversión para 12 bits (29 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT20_I2C_ADDR, data, 3, 150); // Leemos 3 bytes: MSB, LSB y CRC
	raw_val = (data[0] << 8) | (data[1] & 0xFC); // Unimos los bytes y ponemos a '0' los últimos 2 bits de estado
	*humidity = -6.0 + 125.0 * ((float) raw_val / 65536.0);	// Aplicamos la fórmula del datasheet
}
uint32_t leerCanalADC(uint32_t canal) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = canal;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	uint32_t valor = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	return valor;
}
float leerVoltajeBateria(void) {
	uint32_t suma = 0;
	for (uint8_t i = 0; i < NUM_MUESTRAS; i++) {
		suma += leerCanalADC(ADC_CHANNEL_5); /* PB1 = ADC_IN5 */
		HAL_Delay(5);
	}
	float rawProm = (float) suma / NUM_MUESTRAS;
	float vPin = (rawProm / ADC_BITS) * VREF;
	return vPin * (R1 + R2) / R2;
}
uint8_t voltajeASoC(float voltaje) {
	if (voltaje >= tablaSoC[0].voltaje)
		return 100;
	if (voltaje <= tablaSoC[PUNTOS_SOC - 1].voltaje)
		return 0;

	for (uint8_t i = 0; i < PUNTOS_SOC - 1; i++) {
		if (voltaje <= tablaSoC[i].voltaje
				&& voltaje >= tablaSoC[i + 1].voltaje) {
			float rango_v = tablaSoC[i].voltaje - tablaSoC[i + 1].voltaje;
			float rango_p = tablaSoC[i].porcentaje - tablaSoC[i + 1].porcentaje;
			float fraccion = (voltaje - tablaSoC[i + 1].voltaje) / rango_v;
			return (uint8_t) (tablaSoC[i + 1].porcentaje + fraccion * rango_p);
		}
	}
	return 0;
}

// Empaquetar 1 Byte (8 bits)
void tlv_pack_8(uint8_t type, uint8_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 1; // Longitud: 1 byte
	lora_tx_buffer[lora_tx_len++] = val;
}

// Empaquetar 2 Bytes
void tlv_pack_16(uint8_t type, uint16_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 2; // Longitud: 2 bytes
	lora_tx_buffer[lora_tx_len++] = (val >> 8) & 0xFF; // MSB
	lora_tx_buffer[lora_tx_len++] = val & 0xFF;        // LSB
}

// Empaquetar 4 Bytes
void tlv_pack_32(uint8_t type, uint32_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 4; // Longitud: 4 bytes
	lora_tx_buffer[lora_tx_len++] = (val >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (val >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (val >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = val & 0xFF;
}

void tlv_pack_time(uint8_t type, uint8_t h, uint8_t m, uint8_t s) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 3; // Longitud: 3 bytes
	lora_tx_buffer[lora_tx_len++] = h;
	lora_tx_buffer[lora_tx_len++] = m;
	lora_tx_buffer[lora_tx_len++] = s;
}

void tlv_pack_xyz(uint8_t type, int16_t x, int16_t y, int16_t z) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 6;
	lora_tx_buffer[lora_tx_len++] = (x >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = x & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (y >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = y & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (z >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = z & 0xFF;
}

void tlv_pack_gps_coords(uint8_t type, int32_t lat, int32_t lon) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 8;
	lora_tx_buffer[lora_tx_len++] = (lat >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lat >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lat >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = lat & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = lon & 0xFF;
}

int32_t nmea_to_int32(char *coord_str, char dir) {
	if (strlen(coord_str) == 0)
		return 0;
	float raw = atof(coord_str);
	int degrees = (int) (raw / 100.0f);
	float minutes = raw - (degrees * 100.0f);
	float decimal_deg = degrees + (minutes / 60.0f);
	if (dir == 'S' || dir == 'W')
		decimal_deg *= -1.0f;
	return (int32_t) (decimal_deg * 100000.0f);
}
// Función maestra para armar el paquete LoRa
uint8_t build_telemetry_payload(uint8_t gps_has_fix, uint8_t h, uint8_t m,
		uint8_t s, int32_t lat, int32_t lon, int16_t alt, uint16_t speed,
		int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x,
		int16_t gyr_y, int16_t gyr_z, int16_t temp_sht, uint16_t hum_sht,
		uint32_t pressure, uint16_t vbat_mv, uint8_t soc, uint16_t mag_angle) {
	lora_tx_len = 0; // Reiniciar el puntero del búfer global

	// 1. Datos GPS (Solo se agregan si hay satélites válidos)
	if (gps_has_fix) {
		tlv_pack_time(0x01, h, m, s);
		tlv_pack_gps_coords(0x02, lat, lon);
		tlv_pack_16(0x03, alt);
		tlv_pack_16(0x04, speed);
	}

	// 2. Datos IMU (Siempre se empaquetan)
	tlv_pack_xyz(0x05, acc_x, acc_y, acc_z);
	tlv_pack_xyz(0x06, gyr_x, gyr_y, gyr_z);
	//tlv_pack_16(0x0A, temp_imu); // Type 0x0A para Temperatura IMU

	// 3. Datos Atmosféricos (Siempre se empaquetan)
	tlv_pack_16(0x07, temp_sht);
	tlv_pack_16(0x08, hum_sht);
	tlv_pack_32(0x09, pressure);

	// 4. Batería y Magnetómetro (Nuevos datos)
	tlv_pack_16(0x0B, vbat_mv);     // Voltaje en mV
	tlv_pack_8(0x0C, soc);          // Porcentaje 0-100%
	tlv_pack_16(0x0D, mag_angle);   // Ángulo escalado x100

	return lora_tx_len; // Devuelve el tamaño final del paquete
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(2000);
	UART_Print("Escaneando I2C3...\r\n");
	for (uint8_t addr = 1; addr < 128; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 1, 10) == HAL_OK) {
			char found[30];
			snprintf(found, sizeof(found), "  Dispositivo en 0x%02X\r\n", addr);
			UART_Print(found);
		}
	}
	UART_Print("Scan completo.\r\n");
	UART_Print("Inicializando sistema de telemetria...\r\n");
	CMPS2_Init(&hi2c3);
	MS5611_Init(&hi2c3, 0);
	float temperature_sht20, hum;
	float gps_speed_kmh = 0.0f;
	// Iniciar interrupciones de recepción para ambos UARTs
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1);

	uint8_t chip_id = BMI270_ReadReg(REG_CHIP_ID);

	if (chip_id != 0x24) {
		char err[60];
		snprintf(err, sizeof(err),
				"ERROR: BMI270 CHIP_ID=0x%02X (esperado 0x24)\r\n", chip_id);
		HAL_UART_Transmit(&huart2, (uint8_t*) err, strlen(err), HAL_MAX_DELAY);
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_gps_time = HAL_GetTick();
	uint32_t last_imu_time = HAL_GetTick();
	BMI270_Data_t imu;
	while (1) {

		uint8_t trigger_telemetry = 0;
		uint8_t gps_valid = 0;

		if (rx_head != rx_tail) {
			char c = (char) rx_buffer[rx_tail];
			rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

			if (c == '\n' || line_index >= sizeof(line_buffer) - 1) {
				line_buffer[line_index] = '\0';

				// 1. Interceptar VTG para guardar la 

				if (strstr(line_buffer, "VTG") != NULL) {
					char speed_str[10] = { 0 };
					// El campo 7 contiene la velocidad en km/h
					get_nmea_field(line_buffer, 7, speed_str);
					if (strlen(speed_str) > 0) {
						gps_speed_kmh = atof(speed_str);
					}
				}
				// 2. Usar GGA como gatillo de telemetría
				else if (strstr(line_buffer, "GGA") != NULL) {
					last_gps_time = HAL_GetTick();
					trigger_telemetry = 1;
					gps_valid = 1;
				}
				line_index = 0;
			} else if (c != '\r') {
				line_buffer[line_index++] = c;
			}
		}

		// ==========================================
		// TAREA 2: WATCHDOG (telemetría sin GPS)
		// ==========================================
		if (HAL_GetTick() - last_gps_time > 2000) {
			if (HAL_GetTick() - last_imu_time > 1000) {
				last_imu_time = HAL_GetTick();
				trigger_telemetry = 1;
				gps_valid = 0;
			}
		} else {
			last_imu_time = HAL_GetTick();
		}

		// ==========================================
		// TAREA 4: TIMEOUT DEL SPI (Evitar bloqueos)
		// ==========================================
		// Si el pin está en ALTO, significa que la Raspi todavía no terminó de leer
		if (HAL_GPIO_ReadPin(DATA_READY_GPIO_Port, DATA_READY_Pin)
				== GPIO_PIN_SET) {

			// Si pasaron más de 500 ms desde que levantamos el pin...
			if (HAL_GetTick() - spi_tx_start_time > 500) {

				// 1. Abortamos la transmisión trabada a nivel de hardware
				HAL_SPI_Abort_IT(&hspi1);

				// 2. Bajamos el pin manualmente para resetear el estado
				HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin,
						GPIO_PIN_RESET);

				// 3. (Opcional) Notificamos el error en el menú 3 de debug
				if (!menu_active && display_mode == 3) {
					UART_Print(
							"\r\n[!] ERROR TIMEOUT SPI: La Raspberry Pi no respondio. Transmision abortada.\r\n");
				}
			}
		}
		// ==========================================
		// TAREA 3: ENSAMBLAR Y ENVIAR PAQUETE TLV
		// ==========================================
		if (trigger_telemetry == 1) {
			lora_tx_len = 0;

			// Variables NMEA subidas de scope para usarlas en el ASCII print
			char time_str[15] = { 0 }, lat_str[15] = { 0 }, ns[2] = { 0 };
			char lon_str[15] = { 0 }, ew[2] = { 0 }, alt_str[10] = { 0 },
					fix_str[2] = { 0 };
			uint8_t gps_has_fix = 0; // Nuestra bandera lógica
			uint8_t h = 0, m = 0, s = 0;
			uint16_t speed_scaled = 0;
			int32_t lat_int = 0, lon_int = 0;
			int16_t alt_int = 0;

			// 1. GPS (si tiene fix)
			if (gps_valid) {
				get_nmea_field(line_buffer, 1, time_str);
				get_nmea_field(line_buffer, 2, lat_str);
				get_nmea_field(line_buffer, 3, ns);
				get_nmea_field(line_buffer, 4, lon_str);
				get_nmea_field(line_buffer, 5, ew);
				get_nmea_field(line_buffer, 6, fix_str);
				get_nmea_field(line_buffer, 9, alt_str);

				if (fix_str[0] >= '1') {
					gps_has_fix = 1;

					int utc_h = (time_str[0] - '0') * 10 + (time_str[1] - '0');
					h = (utc_h - 3 < 0) ? utc_h - 3 + 24 : utc_h - 3;
					m = (time_str[2] - '0') * 10 + (time_str[3] - '0');
					s = (time_str[4] - '0') * 10 + (time_str[5] - '0');
					//tlv_pack_time(0x01, (uint8_t) art_h, m, s); //TLV HORA

					lat_int = nmea_to_int32(lat_str, ns[0]);
					lon_int = nmea_to_int32(lon_str, ew[0]);
					//tlv_pack_gps_coords(0x02, lat_int, lon_int); //TLV LATITUD Y LONGITUD

					alt_int = (int16_t) atof(alt_str);
					//tlv_pack_16(0x03, alt_int); //TLV ALTITUD
					speed_scaled = (uint16_t) (gps_speed_kmh * 100.0f);
					//tlv_pack_16(0x04, (uint16_t) (gps_speed_kmh * 100.0f));
				}
			}

			// 2. IMU (siempre)
			BMI270_Get_Data(&imu);

			//float temp = BMI270_ReadTemperature();

			//tlv_pack_xyz(0x05, acc_x, acc_y, acc_z); //TLV acelerometro X,Y,Z
			//tlv_pack_xyz(0x06, gyr_x, gyr_y, gyr_z); //TLV giroscopio X,Y,Z

			// Conversión a valores reales para el Print ASCII
			// (Basado en rango de +-8g configurado en REG_ACC_RANGE = 0x02 y giroscopio por defecto de +-2000dps)
			float ax_g = imu.acc_x / 4096.0f;
			float ay_g = imu.acc_y / 4096.0f;
			float az_g = imu.acc_z / 4096.0f;

			float gx_dps = imu.gyr_x / 16.4f;
			float gy_dps = imu.gyr_y / 16.4f;
			float gz_dps = imu.gyr_z / 16.4f;

			//SHT20 (SIEMPRE)
			SHT20_Read(&temperature_sht20, &hum);
			int16_t temp_sht_bits = (int16_t) (temperature_sht20 * 100.0f);
			//tlv_pack_16(0x07, temp_sht_bits); //TLV temperatura sth_21
			uint16_t hum_bits = (uint16_t) (hum * 100.0f);
			//tlv_pack_16(0x08, hum_bits); //TLV humedad

			//MS5611
			float temperature_ms5611 = 0.0f;
			float pressure_pa = 0.0f;
			float pressure_ms5611;
			uint32_t press_bits;

			MS5611_Measure(&hi2c3, &temperature_ms5611, &pressure_pa);
			pressure_ms5611 = pressure_pa / 100.0f;  // convertir a mbar
			press_bits = (uint32_t) (pressure_ms5611 * 100.0f); // si tu TLV espera centésimas de mbar

			float measured_angle = CMPS2_GetHeading();
			const char *direccion_viento = CMPS2_DecodeHeading(measured_angle);
			uint16_t mag_angle_scaled = (uint16_t) (measured_angle * 100.0f);

			//BATERIAS ADC
			float voltajeCrudo = leerVoltajeBateria();

			// Filtro exponencial: suaviza la lectura entre ciclos
			if (vbatFiltrado == 0.0f)
				vbatFiltrado = voltajeCrudo; // inicialización en el primer ciclo
			else
				vbatFiltrado = ALPHA * voltajeCrudo
						+ (1.0f - ALPHA) * vbatFiltrado;

			float voltaje = vbatFiltrado;
			uint8_t soc = voltajeASoC(voltaje);
			uint16_t vbat_mv = (uint16_t) (voltaje * 1000.0f);

			build_telemetry_payload(gps_has_fix, h, m, s, lat_int, lon_int,
					alt_int, speed_scaled, imu.acc_x, imu.acc_y, imu.acc_z,
					imu.gyr_x, imu.gyr_y, imu.gyr_z, temp_sht_bits, hum_bits,
					press_bits, vbat_mv, soc, mag_angle_scaled);
			// ==========================================
			// NUEVO: TRANSMISIÓN SPI HACIA LA RASPBERRY
			// ==========================================

			// 1. Abortamos cualquier transmisión SPI anterior que haya quedado colgada
			// (por si la Raspi se reinició y no leyó el paquete anterior)
			HAL_SPI_Abort_IT(&hspi1);

			// 2. Limpiamos el búfer y preparamos el nuevo paquete
			memset(spi_tx_buffer, 0, sizeof(spi_tx_buffer));
			spi_tx_buffer[0] = lora_tx_len; // Byte 0 = Longitud
			memcpy(&spi_tx_buffer[1], lora_tx_buffer, lora_tx_len); // Copiamos el TLV

			// 3. Cargamos el SPI por Interrupción (El STM32 queda listo esperando el Clock)
			// Mandamos siempre los 128 bytes fijos para mantener la sincronía.
			HAL_SPI_Transmit_IT(&hspi1, spi_tx_buffer, sizeof(spi_tx_buffer));

			// 4. Levantamos el pin DATA_READY para avisarle a la Raspi
			HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin,
					GPIO_PIN_SET);
			spi_tx_start_time = HAL_GetTick();

			// 3. Debug por UART
			if (!menu_active) {
				if (display_mode == 2) {
					// MODO TLV
					char debug_msg[200] = "TLV [";
					char hex_byte[5];
					for (int i = 0; i < lora_tx_len; i++) {
						snprintf(hex_byte, sizeof(hex_byte), "%02X ",
								lora_tx_buffer[i]);
						strcat(debug_msg, hex_byte);
					}
					char tail_buf[20];
					snprintf(tail_buf, sizeof(tail_buf), "] %d bytes\r\n",
							lora_tx_len);
					strcat(debug_msg, tail_buf);
					UART_Print(debug_msg);
				} else if (display_mode == 1) {
					// MODO ASCII NORMAL (Formateado según imagen)
					// Aumentamos el tamaño del buffer para evitar desbordamientos
					char ascii_msg[512];

					// 1. Verificamos si pasaron más de 2 segundos sin datos (Desconectado)
					if (HAL_GetTick() - last_gps_time > 2000) {
						snprintf(ascii_msg, sizeof(ascii_msg),
								"\r\n[GPS] Tx desconectado\r\n"
										"[PMOD] Angulo: %.2f | Direccion: %s"
										"[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
										"[SHT20] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
										"[MS5611] Temperatura: %.2f C | Presion: %.2f\r\n"
										"VBAT: %.3f V | SOC: %u%%\r\n",
								measured_angle, direccion_viento, ax_g, ay_g,
								az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
								temperature_sht20, hum, temperature_ms5611,
								pressure_ms5611, voltaje, soc);
					}
					// 2. Verificamos si hay conexión y tenemos fix satelital
					else if (gps_valid && fix_str[0] >= '1') {
						snprintf(ascii_msg, sizeof(ascii_msg),
								"\r\n[GPS] Lat: %s %s, Lon: %s %s, Alt: %s, Vel: %.2f km/h, Hora: %.2d:%.2d:%.2d\r\n"
										"[PMOD] Angulo: %.2f | Direccion: %s"
										"[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
										"[SHT20] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
										"[MS5611] Temperatura: %.2f C | Presion: %.2f\r\n"
										"VBAT: %.3f V | SOC: %u%%\r\n", lat_str,
								ns, lon_str, ew, alt_str, gps_speed_kmh, h, m,
								s, measured_angle, direccion_viento, ax_g, ay_g,
								az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
								temperature_sht20, hum, temperature_ms5611,
								pressure_ms5611, voltaje, soc);
					}
					// 3. Hay conexión pero aún no hay fix
					else {
						snprintf(ascii_msg, sizeof(ascii_msg),
								"\r\n[GPS] Buscando satelites...\r\n"
										"[PMOD] Angulo: %.2f | Direccion: %s"
										"[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
										"[SHT20] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
										"[MS5611] Temperatura: %.2f C | Presion: %.2f\r\n"
										"VBAT: %.3f V | SOC: %u%%\r\n",
								measured_angle, direccion_viento, ax_g, ay_g,
								az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
								temperature_sht20, hum, temperature_ms5611,
								pressure_ms5611, voltaje, soc);
					}
					UART_Print(ascii_msg);
				} else if (display_mode == 3) {
					// MODO 3: DEBUG SPI Y ERRORES
					char debug_msg[256];

					// Obtener el estado y los errores reales del periférico SPI
					HAL_SPI_StateTypeDef spi_state = HAL_SPI_GetState(&hspi1);
					uint32_t spi_error = HAL_SPI_GetError(&hspi1);

					// Leer el estado del pin que le avisa a la Raspberry
					GPIO_PinState pin_state = HAL_GPIO_ReadPin(
					DATA_READY_GPIO_Port, DATA_READY_Pin);

					snprintf(debug_msg, sizeof(debug_msg),
							"\r\n--- DEBUG SPI ---\r\n"
									"[*] SPI State: 0x%02X | Error Code: 0x%08lX\r\n"
									"[*] DATA_READY Pin: %d (1=Esperando Raspi, 0=Leido/Libre)\r\n"
									"[*] Payload SPI a enviar: %d bytes\r\n",
							spi_state, spi_error, pin_state, lora_tx_len + 1); // +1 por el byte de longitud
					UART_Print(debug_msg);

					// Imprimir los primeros 8 bytes del búfer SPI para verificar el empaquetado
					char hex_buf[100] = "[*] SPI TX Buffer [0..7]: ";
					char hex[5];
					for (int i = 0; i < 8 && i < sizeof(spi_tx_buffer); i++) {
						snprintf(hex, sizeof(hex), "%02X ", spi_tx_buffer[i]);
						strcat(hex_buf, hex);
					}
					strcat(hex_buf, "\r\n");
					UART_Print(hex_buf);

					// Alerta de posible bloqueo
					if (pin_state == GPIO_PIN_SET
							&& spi_state == HAL_SPI_STATE_BUSY_TX) {
						UART_Print(
								"[!] ADVERTENCIA: La Raspberry no ha generado el Clock de lectura.\r\n");
					}
				}
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00B07CB4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DATA_READY_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DATA_READY_Pin CS_Pin */
  GPIO_InitStruct.Pin = DATA_READY_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// GPS UART
	if (huart->Instance == USART1) {
		rx_buffer[rx_head] = rx_data;
		rx_head = (rx_head + 1) % RX_BUF_SIZE;
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
// INTERFAZ DE USUARIO UART (DEBUG)
	else if (huart->Instance == USART2) {
		if (rx_data_uart2 == '0' && !menu_active) {
			// Abrir menú
			menu_active = 1;
			UART_Print("\r\n========================\r\n");
			UART_Print("     MENU DE AJUSTES    \r\n");
			UART_Print("========================\r\n");
			UART_Print("1. Modo ASCII (gps\\r\\nIMU)\r\n");
			UART_Print("2. Modo TLV (Hexadecimal)\r\n");
			UART_Print("3. Modo Debug SPI/Errores\r\n"); // <-- NUEVA OPCIÓN
			UART_Print("Seleccione una opcion: ");
		} else if (menu_active) {
			// Procesar selección del menú
			if (rx_data_uart2 == '1') {
				display_mode = 1;
				menu_active = 0;
				UART_Print("1\r\n[+] Modo ASCII activado.\r\n");
			} else if (rx_data_uart2 == '2') {
				display_mode = 2;
				menu_active = 0;
				UART_Print("2\r\n[+] Modo TLV activado.\r\n");
			} else if (rx_data_uart2 == '3') {
				display_mode = 3;
				menu_active = 0;
				UART_Print("3\r\n[+] Modo Debug SPI/Errores activado.\r\n"); // <-- NUEVA ACCIÓN
			} else {
				UART_Print("\r\n[!] Opcion no valida. Seleccione 1, 2 o 3: ");
			}
		}
		// Rehabilitar interrupción de recepción
		HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	} else if (huart->Instance == USART2) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1);
	}
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) { // Cambia SPI1 por el que uses
		// La Raspi terminó de leer. Bajamos la bandera.
		HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_RESET);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
