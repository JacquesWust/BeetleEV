/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

IWDG_HandleTypeDef hiwdg1;

/* USER CODE BEGIN PV */
volatile uint16_t adc1_values[3] = { 0 };
/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

bool malfunction = false;

volatile uint32_t last0x166MsgTime = 0;
volatile uint8_t control_byte_last = 0;

volatile bool sendFastMsg = false;
volatile bool sendSlowMsg = false;
volatile uint8_t fastMsgData[8];
volatile uint8_t slowMsgData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_IWDG1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_DAC1_Init();
	MX_DAC2_Init();
	MX_IWDG1_Init();
	MX_FDCAN2_Init();
	MX_FDCAN1_Init();
	/* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_values, 3); // Start ADC in DMA mode

	// Configure global filter to accept all frames in FIFO0
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
	FDCAN_ACCEPT_IN_RX_FIFO0,  // Non-matching standard frames
			FDCAN_ACCEPT_IN_RX_FIFO0,  // Non-matching extended frames
			FDCAN_FILTER_REMOTE,       // Remote standard frames
			FDCAN_FILTER_REMOTE);      // Remote extended frames

	// Start FDCAN controller
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}

	// Enable interrupt-based reception
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}

	// Configure global filter for FDCAN1
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	FDCAN_ACCEPT_IN_RX_FIFO0,  // Non-matching standard frames
			FDCAN_ACCEPT_IN_RX_FIFO0,  // Non-matching extended frames
			FDCAN_FILTER_REMOTE,       // Remote standard frames
			FDCAN_FILTER_REMOTE);      // Remote extended frames

	// Start FDCAN1 controller
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	// Enable interrupt-based reception for FDCAN1
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}

	// Prepare Tx Header (customize as needed)
	TxHeader.Identifier = 0x123;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // k1-4 traction enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // k1-5 forward
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // k1-6 reverse
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); // k1-7 torque limit
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // k1-18 profile 2
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); // k1-19 profile 3
	/* USER CODE END 2 */

	/* Initialize leds */
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_YELLOW);
	BSP_LED_Init(LED_RED);

	/* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	/* USER CODE BEGIN BSP */

	/* USER CODE END BSP */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		HAL_IWDG_Refresh(&hiwdg1);

		// throttle 1
		int expected_adc2 = (int)(-0.83803 * adc1_values[0] + 3337.7 + 0.5);

		uint32_t currentTime = HAL_GetTick();
		if (currentTime > 1000 && (adc1_values[1] < 400 || adc1_values[2] < 1200 || adc1_values[2] > 2600 || adc1_values[0] < 1200 || adc1_values[0] > 2600 || abs(expected_adc2-adc1_values[2]) > 600)) {
			malfunction = true;
		}

		uint16_t throttle1 = (adc1_values[2] - 1350) * 4095 / (2400 - 1350);
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, throttle1);
		// throttle 2
		uint16_t throttle2 = (adc1_values[0] - 1350) * 4095 / (2400 - 1350);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, throttle2);
		// brake
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
				adc1_values[1]);


		//printf("ADC Values: %05u %05u %05u %05u\r\n", adc1_values[0], adc1_values[1], adc1_values[2], expected_adc2);

		// If no control message received in last 0.5 seconds, reset all outputs so that traction is disabled.
		if ((currentTime + 50 - last0x166MsgTime > 550
				&& control_byte_last != 0x00) || malfunction) { // sometimes last0x166MsgTime gets updated through an interrupt after currentTime has been set. So then you don't want to subtract them and find them and get a negative number as it will be seen as higher than 500.
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // k1-4 traction enable
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // k1-5 forward
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // k1-6 reverse
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); // k1-7 torque limit
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // k1-18 profile 2
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); // k1-19 profile 3
			control_byte_last = 0x00;
			last0x166MsgTime = currentTime;
		}

		if (sendFastMsg) {
		    // Prepare data buffer for CAN message - DON'T initialize to zero!
		    uint8_t data[8];

		    // Copy all bytes from the received message
		    for (int i = 0; i < 8; i++) {
		        data[i] = fastMsgData[i];
		    }

		    // Check brake pedal status (NOT depressed if between 450-490)
		    bool brake_depressed = (adc1_values[1] < 450 || adc1_values[1] > 510);

		    // Check throttle pedal status (NO pressure if both ADCs in specific ranges)
		    bool throttle_pressure = !(adc1_values[2] >= 1500 && adc1_values[2] <= 1550 &&
		                             adc1_values[0] >= 2160 && adc1_values[0] <= 2210);

		    // Get the original value and mask it to 0-7
		    uint8_t original_value = data[7] & 0x07; // Use 3 bits for 0-7

		    // Construct the new byte: brake in bit 6, throttle in bit 7, original value in bits 0-2
		    data[7] = (brake_depressed ? 0x80 : 0) | (throttle_pressure ? 0x40 : 0) | original_value;

			// Set up transmission header
			TxHeader.Identifier = 0x154;
			TxHeader.IdType = FDCAN_STANDARD_ID;
			TxHeader.TxFrameType = FDCAN_DATA_FRAME;
			TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // Using all 8 bytes
			TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
			TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
			TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			TxHeader.MessageMarker = 0;

			// Send the message
			HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,
					&TxHeader, data);
			if (status != HAL_OK) {
				printf("CAN TX Error: %d\r\n", status);
			}

			// Reset flag
			sendFastMsg = false;
		}

		if (sendSlowMsg) {
			// Prepare data buffer for CAN message
			uint8_t data[8] = { 0 }; // Initialize all bytes to zero

		    // Copy data from the received message - forward exactly as received
		    for (int i = 0; i < 4; i++) {
		        data[i] = slowMsgData[i];
		    }

			// Set up transmission header
			TxHeader.Identifier = 0x155;
			TxHeader.IdType = FDCAN_STANDARD_ID;
			TxHeader.TxFrameType = FDCAN_DATA_FRAME;
			TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // Using all 8 bytes
			TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
			TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
			TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			TxHeader.MessageMarker = 0;

			// Send the message
			HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,
					&TxHeader, data);
			if (status != HAL_OK) {
				printf("CAN TX Error: %d\r\n", status);
			}

			// Reset flag
			sendSlowMsg = false;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/*AXI clock gating */
	RCC->CKGAENR = 0xFFFFFFFF;

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = 64;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.Oversampling.OversamplingStopReset =
	ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief DAC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC2_Init(void) {

	/* USER CODE BEGIN DAC2_Init 0 */

	/* USER CODE END DAC2_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC2_Init 1 */

	/* USER CODE END DAC2_Init 1 */

	/** DAC Initialization
	 */
	hdac2.Instance = DAC2;
	if (HAL_DAC_Init(&hdac2) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC2_Init 2 */

	/* USER CODE END DAC2_Init 2 */

}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

	/* USER CODE BEGIN FDCAN1_Init 0 */

	/* USER CODE END FDCAN1_Init 0 */

	/* USER CODE BEGIN FDCAN1_Init 1 */

	/* USER CODE END FDCAN1_Init 1 */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 1;
	hfdcan1.Init.NominalSyncJumpWidth = 4;
	hfdcan1.Init.NominalTimeSeg1 = 27;
	hfdcan1.Init.NominalTimeSeg2 = 4;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 14;
	hfdcan1.Init.DataTimeSeg1 = 17;
	hfdcan1.Init.DataTimeSeg2 = 14;
	hfdcan1.Init.MessageRAMOffset = 0;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.RxFifo0ElmtsNbr = 1;
	hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxFifo1ElmtsNbr = 0;
	hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxBuffersNbr = 0;
	hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.TxEventsNbr = 0;
	hfdcan1.Init.TxBuffersNbr = 0;
	hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN1_Init 2 */

	/* USER CODE END FDCAN1_Init 2 */

}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = DISABLE;
	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;
	hfdcan2.Init.NominalPrescaler = 1;
	hfdcan2.Init.NominalSyncJumpWidth = 4;
	hfdcan2.Init.NominalTimeSeg1 = 27;
	hfdcan2.Init.NominalTimeSeg2 = 4;
	hfdcan2.Init.DataPrescaler = 1;
	hfdcan2.Init.DataSyncJumpWidth = 14;
	hfdcan2.Init.DataTimeSeg1 = 17;
	hfdcan2.Init.DataTimeSeg2 = 14;
	hfdcan2.Init.MessageRAMOffset = 0;
	hfdcan2.Init.StdFiltersNbr = 1;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.RxFifo0ElmtsNbr = 1;
	hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxFifo1ElmtsNbr = 0;
	hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxBuffersNbr = 0;
	hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.TxEventsNbr = 0;
	hfdcan2.Init.TxBuffersNbr = 0;
	hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	/* USER CODE END FDCAN2_Init 2 */

}

/**
 * @brief IWDG1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG1_Init(void) {

	/* USER CODE BEGIN IWDG1_Init 0 */

	/* USER CODE END IWDG1_Init 0 */

	/* USER CODE BEGIN IWDG1_Init 1 */

	/* USER CODE END IWDG1_Init 1 */
	hiwdg1.Instance = IWDG1;
	hiwdg1.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg1.Init.Window = 4095;
	hiwdg1.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG1_Init 2 */

	/* USER CODE END IWDG1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10 | GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PA10 PA13 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		// Get message from FIFO 0
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
				== HAL_OK) {
			// Check message ID
			if (RxHeader.Identifier == 0x166 && hfdcan->Instance == FDCAN2) {
				// Message for GPIO control
				uint8_t control_byte = RxData[0];

				if (control_byte != control_byte_last && !malfunction) {
					// Update GPIO pins based on control byte (in k1-pin order)
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,
							(control_byte & 0x01) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-4 traction enable
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,
							(control_byte & 0x02) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-5 forward
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
							(control_byte & 0x04) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-6 reverse
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,
							(control_byte & 0x08) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-7 torque limit
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,
							(control_byte & 0x10) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-18 profile 2
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,
							(control_byte & 0x20) ?
									GPIO_PIN_SET : GPIO_PIN_RESET); // k1-19 profile 3
					control_byte_last = control_byte;
				}

				last0x166MsgTime = HAL_GetTick();

//				printf("Received GPIO control: 0x%02X\r\n", control_byte);
			} else if (RxHeader.Identifier
					== 0x154&& hfdcan->Instance == FDCAN1) { // Fast message received
			// Fast message received on FDCAN1, copy to buffer for forwarding
				for (int i = 0; i < 8; i++) {
					fastMsgData[i] = RxData[i];
				}
				sendFastMsg = true;
			} else if (RxHeader.Identifier
					== 0x155&& hfdcan->Instance == FDCAN1) { // Slow message received
			// Slow message received on FDCAN1, copy to buffer for forwarding
				for (int i = 0; i < 8; i++) {
					slowMsgData[i] = RxData[i];
				}
				sendSlowMsg = true;
			}
		}
	}
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief BSP Push Button callback
 * @param Button Specifies the pressed button
 * @retval None
 */
void BSP_PB_Callback(Button_TypeDef Button) {
	if (Button == BUTTON_USER) {
		BspButtonState = BUTTON_PRESSED;
	}
}

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

#ifdef  USE_FULL_ASSERT
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
