/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "driver_bmp280.h"
#include "driver_bmp280_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Direccion SHT21
#define SHT21_I2C_ADDR   (0x40 << 1) // Dirección I2C desplazada 1 bit
#define CMD_MEASURE_T    0xF3        // Trigger T measurement (no hold master)
#define CMD_MEASURE_RH   0xF5        // Trigger RH measurement (no hold master)

//Dirección BMP280BMP280_I2C_ADDR_GND
#define BMP280_ADDRESS_0 	0x76
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_DATA      0xF7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bmp280_handle_t gs_handle; // Estructura de control del sensor

void SHT21_Read(float *temperature, float *humidity) {
	uint8_t cmd;
	uint8_t data[3];
	uint16_t raw_val;
	// 1. Leer Temperatura
	cmd = CMD_MEASURE_T;
	HAL_I2C_Master_Transmit(&hi2c3, SHT21_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY); // Enviamos el comando de medición de temperatura
	HAL_Delay(85); // Esperamos el tiempo máximo de conversión para 14 bits (85 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT21_I2C_ADDR, data, 3, HAL_MAX_DELAY); // Leemos 3 bytes: MSB, LSB y CRC
	// Unimos los bytes y ponemos a '0' los últimos 2 bits de estado (0xFC = 11111100 en binario)
	raw_val = (data[0] << 8) | (data[1] & 0xFC);
	*temperature = -46.85 + 175.72 * ((float) raw_val / 65536.0); // Aplicamos la fórmula del datasheet
	// 2. Leer Humedad Relativa
	cmd = CMD_MEASURE_RH;
	HAL_I2C_Master_Transmit(&hi2c3, SHT21_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY); // Enviamos el comando de medición de humedad
	HAL_Delay(29); // Esperamos el tiempo máximo de conversión para 12 bits (29 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT21_I2C_ADDR, data, 3, HAL_MAX_DELAY); // Leemos 3 bytes: MSB, LSB y CRC
	raw_val = (data[0] << 8) | (data[1] & 0xFC); // Unimos los bytes y ponemos a '0' los últimos 2 bits de estado
	*humidity = -6.0 + 125.0 * ((float) raw_val / 65536.0);	// Aplicamos la fórmula del datasheet
}

void UART_Print(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/* USER CODE END 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_I2C3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	float tempe, hum;
	char uart_buf[100];
	HAL_Delay(2500);
	//BMP280
	// 1. Enlazar las funciones de la interfaz
	DRIVER_BMP280_LINK_INIT(&gs_handle, bmp280_handle_t);

	// Enlaces I2C (Los que ya arreglamos)
	DRIVER_BMP280_LINK_IIC_INIT(&gs_handle, bmp280_interface_iic_init);
	DRIVER_BMP280_LINK_IIC_DEINIT(&gs_handle, bmp280_interface_iic_deinit);
	DRIVER_BMP280_LINK_IIC_READ(&gs_handle, bmp280_interface_iic_read);
	DRIVER_BMP280_LINK_IIC_WRITE(&gs_handle, bmp280_interface_iic_write);

	// NUEVOS: Enlaces SPI (Ficticios para pasar el control de seguridad)
	DRIVER_BMP280_LINK_SPI_INIT(&gs_handle, bmp280_interface_spi_init);
	DRIVER_BMP280_LINK_SPI_DEINIT(&gs_handle, bmp280_interface_spi_deinit);
	DRIVER_BMP280_LINK_SPI_READ(&gs_handle, bmp280_interface_spi_read);
	DRIVER_BMP280_LINK_SPI_WRITE(&gs_handle, bmp280_interface_spi_write);

	// Enlaces misceláneos
	DRIVER_BMP280_LINK_DELAY_MS(&gs_handle, bmp280_interface_delay_ms);
	DRIVER_BMP280_LINK_DEBUG_PRINT(&gs_handle, bmp280_interface_debug_print);

	// 2. Configurar el protocolo e inicializar
	bmp280_set_interface(&gs_handle, BMP280_INTERFACE_IIC);
	bmp280_set_addr_pin(&gs_handle, BMP280_ADDRESS_0); // O BMP280_ADDR_GND dependiendo de tu pin SDO

	// Probamos si el sensor responde en la dirección 0x76 (desplazada)
	if (HAL_I2C_IsDeviceReady(&hi2c3, (0x76 << 1), 3, 100) == HAL_OK) {
		UART_Print("BMP280 detectado en el bus I2C!\r\n");
	} else {
		UART_Print("BMP280 NO responde (revisar cables/CSB/pull-ups)\r\n");
	}

	if (bmp280_init(&gs_handle) != 0) {
		UART_Print("Error al inicializar sensor BMP280\r\n");
	}

	// 3. Configuraciones básicas (recomendadas para clima/estándar)
	bmp280_set_temperature_oversampling(&gs_handle, BMP280_OVERSAMPLING_x1);
	bmp280_set_pressure_oversampling(&gs_handle, BMP280_OVERSAMPLING_x4);
	bmp280_set_standby_time(&gs_handle, BMP280_STANDBY_TIME_500_MS);
	bmp280_set_mode(&gs_handle, BMP280_MODE_NORMAL); // Arrancar mediciones continuas

	uint32_t raw_temperature;
	uint32_t raw_pressure;
	float temperature_c;
	float pressure_pa;
	char buffer_uart[100];
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (bmp280_read_temperature_pressure(&gs_handle, &raw_temperature,
				&temperature_c, &raw_pressure, &pressure_pa) == 0) {
			sprintf(buffer_uart, "Temperatura: %.2f C, Presion: %.2f Pa\r\n",
					temperature_c, pressure_pa);
			UART_Print(buffer_uart);
		} else {
			UART_Print("Error al leer datos\r\n");
		}
		HAL_Delay(100);
		SHT21_Read(&tempe, &hum);

		// Formateamos la cadena (Asegúrate de tener habilitado el soporte para float en printf en la config del IDE si usas STM32CubeIDE)
		sprintf(uart_buf, "Temperatura: %.2f C | Humedad: %.2f %%\r\n", tempe,
				hum);

		// Enviamos por el puerto serial (USART)
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf),
		HAL_MAX_DELAY);

		// Esperamos un segundo antes de la próxima lectura
		HAL_Delay(1000);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00100D14;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
