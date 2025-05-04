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
#include "display_utils.h"

#include <stdbool.h>
#include <stdio.h>
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
bool indicatorOn = true;
uint8_t hazardState[] =
		{ GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET };
uint8_t leftState[] = { GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET };
uint8_t rightState[] =
		{ GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET };
uint8_t indicatorCounter = 0;

volatile bool battery = false;
volatile bool uart_dma_busy = false;
volatile uint8_t indicator_state = 0;  // 0: off; 1: hazard; 2: left; 3: right

/* CAN message structures */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* UART buffer */
char uart_buf[100];
int uart_buf_len;

volatile bool contactor = false;    // Contactor state based on MCU summary
volatile bool plugged = false;      // Plugged in status based on MCU summary
volatile uint8_t SOC = 0;           // State of Charge percentage
volatile float current_bms = 0.0f;      // Current in Amps
volatile uint8_t thmax = 0;         // Maximum thermistor temperature
volatile float chargekw = 0.0f;     // Charge power in kW
volatile float voltage = 0.0f;      // Pack voltage in volts

volatile bool underTemp;
volatile bool overTemp;
volatile bool LVC;
volatile bool HVC;
volatile bool thermistorCensus;
volatile bool cellCensus;
volatile bool hardwareFault;

bool inverter_traction_enable = false;   // k1-4 traction enable
bool inverter_forward = false;           // k1-5 forward
bool inverter_reverse = false;           // k1-6 reverse
bool inverter_torque_limit = false;      // k1-7 torque limit
bool inverter_profile2 = false;          // k1-18 profile 2
bool inverter_profile3 = false;          // k1-19 profile 3

volatile uint32_t lastBMSMsg = 0;

// error display
#define numErrors 21
bool errors[numErrors] = { true, false, false, false, false, false, false,
false, false, false, false, false, false, false, false, false, false, false,
false, false, false };
uint8_t errorMsgs[numErrors][21] = { "  EV switched on  ", // 0
		"  Inv. > 75deg C  ", // 1
		" Motor > 135deg C ", // 2
		" Bat. below 100V  ", // 3
		"  Bat. below 20%  ", // 4
		"  Bat. < 5deg C   ", // 5
		"  Bat. > 40deg C  ", // 6
		"  Inv. > 90deg C  ", // 7
		" Motor > 155deg C ", // 8
		"  Bat. below 90V  ", // 9
		"  BMS CAN error   ", // 10
		" Fr-Rr CAN error  ", // 11
		"  Inv. CAN error  ", // 12
		"  BMS LVC < 2.8V  ", // 13
		" BMS HVC > 4.25V  ", // 14
		"  Bat. < 0deg C   ", // 15
		"  Bat. > 55deg C  ", // 16
		"  Therm. Census   ", // 17
		"   Cell Census    ", // 18
		"   BMS HW Fault   ", // 19
		" Inv. Error #     " }; // 20
uint32_t lastDispUpdate = 0;
uint8_t lastDispError = 0;

// Inverter telemetry data from 0x154 message
volatile double speed = 0;
volatile double current_inv = 0.0;
volatile int16_t motor_temp = 0;        // Motor temperature (signed)
volatile int16_t inverter_temp = 0;     // Inverter temperature (signed)
volatile uint8_t fault_code = 0;
volatile uint8_t fault_level = 0;
volatile bool brake_active_inv = false;     // Brake status (true/false)
volatile bool throttle_active_inv = false;  // Throttle status (true/false)
volatile double odometer = 0.0;
volatile uint32_t last154MsgTime = 0;         // Timestamp for timeout detection

uint32_t lastReversePress = 0;
uint32_t lastHeaterPress = 0;
bool heaterState = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_BMS_Request(void) {
	TxHeader.StdId = 0x313; /* PDO2 MOSI ID */
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;

	// Requests in 50ms ticks
	TxData[0] = 0;
	TxData[1] = 1; 	  // MCU summary interval
	TxData[2] = 1;    // Pack summary interval
	TxData[3] = 20;   // Cell voltage summary interval
	TxData[4] = 5; // Thermistor summary interval HAS BUG IF SET TO 20 LIKE OTHERS
	TxData[5] = 20;   // SOC summary interval
	TxData[6] = 0;    // Cellmap summary (don't need this)
	TxData[7] = 0;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void Send_Inverter_State(void) {  // Send custom CAN control msg to rear STM32
	TxHeader.StdId = 0x166;  // Inverter control message ID
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;  // Standard 8-byte message

	// Pack all control bits into a single byte (bits 6-7 reserved)
	uint8_t control_byte = 0;

	// Set bits according to function parameters (following k1-pin order)
	if (inverter_traction_enable)
		control_byte |= 0x01;  // Bit 0: k1-4 traction enable
	if (inverter_forward)
		control_byte |= 0x02;  // Bit 1: k1-5 forward
	if (inverter_reverse)
		control_byte |= 0x04;  // Bit 2: k1-6 reverse
	if (inverter_torque_limit)
		control_byte |= 0x08;  // Bit 3: k1-7 torque limit
	if (inverter_profile2)
		control_byte |= 0x10;  // Bit 4: k1-18 profile 2
	if (inverter_profile3)
		control_byte |= 0x20;  // Bit 5: k1-19 profile 3

	// Set the control byte as the first byte of the message
	TxData[0] = control_byte;

	// Clear the rest of the message (not strictly necessary but good practice)
	for (int i = 1; i < 8; i++) {
		TxData[i] = 0;
	}

	// Send the message
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint8_t main_state = 0;  // 0: off; 1: on driving; 2: charging
	bool starter_held = true;
	uint32_t starter_start = 0;
	uint32_t lastBMSRequest = 0;
	uint32_t lastInverterControl = 0;
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_CAN_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	CAN_FilterTypeDef filter;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0; // Important: use FIFO0, not FIFO1
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &filter);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan);
	// Switch on main relay for display, horn, wipers, etc.
	HAL_GPIO_WritePin(main_relay_GPIO_Port, main_relay_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t currentTime = HAL_GetTick();

		// if we haven't received recent BMS update via CAN, set error and turn off contactor
		if (currentTime + 50 - lastBMSMsg > 300) {
			contactor = false;
			SOC = 0;
			thmax = 98;
			voltage = 987.0f;
			errors[10] = true;
		} else {
			errors[10] = false;
		}

		// More than 1 second since last telemetry message from rear STM32, set default values
		if (currentTime - last154MsgTime > 100 && currentTime > 1000) {
			throttle_active_inv = false;
			speed = 198.0;
			motor_temp = 987;
			inverter_temp = 987;
			odometer = 987654.0;
			current_inv = 200.0;
			HAL_GPIO_WritePin(brake_relay_GPIO_Port, brake_relay_Pin,
					GPIO_PIN_SET);
			errors[11] = true;
			errors[12] = true;
		} else {
			errors[11] = false;
			errors[12] = false;
		}

		if (contactor) {
			HAL_GPIO_WritePin(contactor_relay_GPIO_Port, contactor_relay_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(contactor_relay_GPIO_Port, contactor_relay_Pin,
					GPIO_PIN_RESET);
		}

		if (brake_active_inv) {
			HAL_GPIO_WritePin(brake_relay_GPIO_Port, brake_relay_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(brake_relay_GPIO_Port, brake_relay_Pin,
					GPIO_PIN_RESET);
		}

		bool heater_button = !HAL_GPIO_ReadPin(heater_in_GPIO_Port,
		heater_in_Pin);
		if (heater_button && currentTime > lastHeaterPress + 50) {
			if (currentTime < lastHeaterPress + 500) {
				heaterState = !heaterState;
				if (heaterState) {
					HAL_GPIO_WritePin(heater_relay_GPIO_Port, heater_relay_Pin,
							GPIO_PIN_SET);
					HAL_GPIO_WritePin(heaterLED_GPIO_Port, heaterLED_Pin,
							GPIO_PIN_SET);
				} else {
					HAL_GPIO_WritePin(heater_relay_GPIO_Port, heater_relay_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(heaterLED_GPIO_Port, heaterLED_Pin,
							GPIO_PIN_RESET);
				}
				lastHeaterPress = currentTime + 500;
			} else {
				lastHeaterPress = currentTime;
			}
		}

		bool reverse_sensor = !HAL_GPIO_ReadPin(reverse_gearbox_in_GPIO_Port,
		reverse_gearbox_in_Pin);
		if ((inverter_reverse && !reverse_sensor)
				|| (!inverter_reverse && reverse_sensor)) {
			HAL_GPIO_WritePin(reverse_relay_GPIO_Port, reverse_relay_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(reverseLED_GPIO_Port, reverseLED_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(reverse_relay_GPIO_Port, reverse_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(reverseLED_GPIO_Port, reverseLED_Pin,
					GPIO_PIN_RESET);
		}

		if (currentTime - lastBMSRequest > 500) {
			Send_BMS_Request();
			lastBMSRequest = currentTime;
		}

		if (currentTime - lastInverterControl > 100) {
			Send_Inverter_State();
			lastInverterControl = currentTime;
		}

		if (plugged == true) {
			main_state = 2;
		}

		if (main_state == 0) {  // car off mode
			if (currentTime < 2000) {
				set_page(3);
			} else if (!HAL_GPIO_ReadPin(reverse_panel_in_GPIO_Port,
			reverse_panel_in_Pin) && !HAL_GPIO_ReadPin(heater_in_GPIO_Port,
			heater_in_Pin)) {
				set_page(1);
			}
			HAL_Delay(50);

			uint32_t current_time = HAL_GetTick();

			uint8_t starter_in = HAL_GPIO_ReadPin(starter_in_GPIO_Port,
			starter_in_Pin);
			if (starter_in == GPIO_PIN_RESET) {
				set_state(0x2110, 1);
				HAL_UART_Transmit(&huart2, (uint8_t*) "CODE Raan\n", 10,
				HAL_MAX_DELAY);

				if (starter_start == 0 && current_time > 200
						&& brake_active_inv) {
					starter_start = current_time;
					HAL_UART_Transmit(&huart2, (uint8_t*) "CODE RUUn\n", 10,
					HAL_MAX_DELAY);
				} else if (starter_start
						> 0&& current_time - starter_start > 700 && starter_held == true) {
					set_current_bar(0.0);
					main_state = 1;
					inverter_traction_enable = true;
					inverter_forward = true;
					HAL_UART_Transmit(&huart2, (uint8_t*) "CODE RYYN\n", 10,
					HAL_MAX_DELAY);
				}
			} else {
				set_state(0x2110, 0);
				if (starter_start > 0) {
					starter_held = false;
				}
			}
			if (brake_active_inv) {
				set_state(0x2100, 1);
			} else {
				set_state(0x2100, 0);
				if (starter_start > 0) {
					starter_held = false;
				}
			}
			if (throttle_active_inv && starter_start > 0) {
				starter_held = false;
			}

		} else if (main_state == 1) {  // driving mode
			set_page(1);
			set_battery(SOC);
			write_two(0x1550, thmax);
			write_three(0x1540, motor_temp);
			write_three(0x1560, inverter_temp);
			// Convert odometer to integer (multiply by 100 since odometer has 2 decimal places)
			uint32_t odo_int = (uint32_t) odometer;

			// Cap at 999999 if needed
			if (odo_int > 999999)
				odo_int = 999999;

			// Prepare 6 ASCII bytes for display
			uint8_t odo_bytes[6];
			odo_bytes[0] = (odo_int / 100000) + '0';       // 100000s place
			odo_bytes[1] = ((odo_int / 10000) % 10) + '0'; // 10000s place
			odo_bytes[2] = ((odo_int / 1000) % 10) + '0';  // 1000s place
			odo_bytes[3] = ((odo_int / 100) % 10) + '0';   // 100s place
			odo_bytes[4] = ((odo_int / 10) % 10) + '0';    // 10s place
			odo_bytes[5] = (odo_int % 10) + '0';           // 1s place

			// Send to display
			dwin_write(0x1570, odo_bytes, 6);

			uint32_t vol_int = (uint32_t) voltage;

			// Cap at 999999 if needed
			if (vol_int > 999)
				vol_int = 999;

			uint8_t vol_bytes[5];
			vol_bytes[0] = ((vol_int / 100) % 10) + '0';   // 100s place
			vol_bytes[1] = ((vol_int / 10) % 10) + '0';    // 10s place
			vol_bytes[2] = (vol_int % 10) + '0';           // 1s place
			vol_bytes[3] = ' '; // 10000s place
			vol_bytes[4] = 'v';  // 1000s place

			dwin_write(0x1580, vol_bytes, 5);

			set_state(0x1740, (int) speed / 100);         // hundreds digit
			set_state(0x1750, ((int) speed % 100) / 10);  // tens digit
			set_state(0x1760, (int) speed % 10);          // ones digit
			set_current_bar(current_inv);

			if (indicator_state == 0) {
				set_state(0x1710, 0);
				set_state(0x1730, 0);
			} else if (indicator_state == 1) {
				set_state(0x1710, 1);
				set_state(0x1730, 1);
			} else if (indicator_state == 2) {
				set_state(0x1710, 1);
				set_state(0x1730, 0);
			} else if (indicator_state == 3) {
				set_state(0x1710, 0);
				set_state(0x1730, 1);
			}

			bool reverse_button = !HAL_GPIO_ReadPin(reverse_panel_in_GPIO_Port,
			reverse_panel_in_Pin);
			if (reverse_button && currentTime > lastReversePress + 50) {
				if (currentTime < lastReversePress + 500) {
					inverter_reverse = !inverter_reverse;
					inverter_forward = !inverter_forward;
					lastReversePress = currentTime + 500;
				} else {
					lastReversePress = currentTime;
				}
			}
		} else {  // charging
			inverter_traction_enable = false;
			set_page(2);
			set_battery(SOC);
			set_voltage(voltage);
			set_current(current_bms);
			set_charge_kw((voltage * current_bms) / 1000.0f);
		}

		// always running code:
		// parking/highbeam/lowbeam and display
		uint8_t parking_in = HAL_GPIO_ReadPin(parking_in_GPIO_Port,
		parking_in_Pin);
		uint8_t lowbeam_in = HAL_GPIO_ReadPin(lowbeam_in_GPIO_Port,
		lowbeam_in_Pin);
		uint8_t highbeam_in = HAL_GPIO_ReadPin(highbeam_in_GPIO_Port,
		highbeam_in_Pin);
		if (highbeam_in == GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(highbeam_relay_GPIO_Port, highbeam_relay_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(lowbeam_relay_GPIO_Port, lowbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(parking_relay_GPIO_Port, parking_relay_Pin,
					GPIO_PIN_SET);
			set_state(0x1720, 3);
		} else if (lowbeam_in == GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(highbeam_relay_GPIO_Port, highbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(lowbeam_relay_GPIO_Port, lowbeam_relay_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(parking_relay_GPIO_Port, parking_relay_Pin,
					GPIO_PIN_SET);
			set_state(0x1720, 2);
		} else if (parking_in == GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(highbeam_relay_GPIO_Port, highbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(lowbeam_relay_GPIO_Port, lowbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(parking_relay_GPIO_Port, parking_relay_Pin,
					GPIO_PIN_SET);
			set_state(0x1720, 1);
		} else {
			HAL_GPIO_WritePin(highbeam_relay_GPIO_Port, highbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(lowbeam_relay_GPIO_Port, lowbeam_relay_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(parking_relay_GPIO_Port, parking_relay_Pin,
					GPIO_PIN_RESET);
			set_state(0x1720, 0);
		}

		if (currentTime - lastDispUpdate >= 1000) {
//					"Cell voltage<2.80V", // 11
//					"Cell voltage>4.25V", // 12
			// inverter temp
			if (inverter_temp > 90) {
				errors[7] = true;
				errors[1] = false;
			} else if (inverter_temp > 75) {
				errors[7] = false;
				errors[1] = true;
			} else {
				errors[7] = false;
				errors[1] = false;
			}
			// motor temp
			if (motor_temp > 155) {
				errors[8] = true;
				errors[2] = false;
			} else if (motor_temp > 135) {
				errors[8] = false;
				errors[2] = true;
			} else {
				errors[8] = false;
				errors[2] = false;
			}
			// voltage
			if (voltage < 90.0f) {
				errors[9] = true;
				errors[3] = false;
			} else if (voltage < 100.0f) {
				errors[9] = false;
				errors[3] = true;
			} else {
				errors[9] = false;
				errors[3] = false;
			}
			// soc
			if (SOC < 20) {
				errors[4] = true;
			} else {
				errors[4] = false;
			}
			// bat low temp
			if (underTemp) {
				errors[15] = true;
				errors[5] = false;
			} else if (thmax < 5) {
				errors[15] = false;
				errors[5] = true;
			} else {
				errors[15] = false;
				errors[5] = false;
			}
			// bat high temp
			if (overTemp) {
				errors[16] = true;
				errors[6] = false;
			} else if (thmax < 5) {
				errors[16] = false;
				errors[6] = true;
			} else {
				errors[16] = false;
				errors[6] = false;
			}
			if (LVC) {
				errors[13] = true;
			} else {
				errors[13] = false;
			}
			if (HVC) {
				errors[14] = true;
			} else {
				errors[14] = false;
			}
			if (thermistorCensus) {
				errors[17] = true;
			} else {
				errors[17] = false;
			}
			if (cellCensus) {
				errors[18] = true;
			} else {
				errors[18] = false;
			}
			if (hardwareFault) {
				errors[19] = true;
			} else {
				errors[19] = false;
			}
			// inverter error
			if (fault_code > 0) {
				errors[20] = true;
				errorMsgs[20][13] = ((fault_code / 100) % 10) + '0'; // 100s place
				errorMsgs[20][14] = ((fault_code / 10) % 10) + '0'; // 10s place
				errorMsgs[20][15] = (fault_code % 10) + '0';         // 1s place
			} else {
				errors[20] = false;
			}

			// Find the next active error (where errors[i] is true)
			uint8_t i = (lastDispError + 1) % numErrors; // Start from the next error index

			// Keep searching until we find an active error
			while (errors[i] == false) {
				i = (i + 1) % numErrors; // Wrap around if needed
			}

			// Update the last error index
			lastDispError = i;

			// Display the corresponding error message
			uint8_t colour[2];
			if (lastDispError <= 0) {
				colour[0] = 0x4D;
				colour[1] = 0x25;
			} else if (lastDispError <= 6) {
				colour[0] = 0xD4;
				colour[1] = 0xC0;
			} else {
				colour[0] = 0xD0;
				colour[1] = 0x00;
			}
			dwin_write(0x7903, colour, sizeof(colour));
			dwin_write(0x2900, errorMsgs[lastDispError], 18);

			// Update the timestamp for the next cycle
			lastDispUpdate = currentTime;
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 63999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 49;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	lowbeam_relay_Pin | heaterLED_Pin | LD2_Pin | highbeam_relay_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			l_indicator_relay_Pin | r_indicator_relay_Pin | parking_relay_Pin
					| main_relay_Pin | fan_relay_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			reverseLED_Pin | brake_relay_Pin | reverse_relay_Pin
					| pump_relay_Pin | contactor_relay_Pin | heater_relay_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : l_indicator_in_Pin r_indicator_in_Pin cruise_in_Pin */
	GPIO_InitStruct.Pin = l_indicator_in_Pin | r_indicator_in_Pin
			| cruise_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : lowbeam_relay_Pin heaterLED_Pin LD2_Pin highbeam_relay_Pin */
	GPIO_InitStruct.Pin = lowbeam_relay_Pin | heaterLED_Pin | LD2_Pin
			| highbeam_relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : parking_in_Pin highbeam_in_Pin reverse_gearbox_in_Pin */
	GPIO_InitStruct.Pin = parking_in_Pin | highbeam_in_Pin
			| reverse_gearbox_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : l_indicator_relay_Pin r_indicator_relay_Pin parking_relay_Pin main_relay_Pin
	 fan_relay_Pin */
	GPIO_InitStruct.Pin = l_indicator_relay_Pin | r_indicator_relay_Pin
			| parking_relay_Pin | main_relay_Pin | fan_relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : reverseLED_Pin brake_relay_Pin reverse_relay_Pin pump_relay_Pin
	 contactor_relay_Pin heater_relay_Pin */
	GPIO_InitStruct.Pin = reverseLED_Pin | brake_relay_Pin | reverse_relay_Pin
			| pump_relay_Pin | contactor_relay_Pin | heater_relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : starter_in_Pin */
	GPIO_InitStruct.Pin = starter_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(starter_in_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : heater_in_Pin reverse_panel_in_Pin lowbeam_in_Pin hazard_in_Pin */
	GPIO_InitStruct.Pin = heater_in_Pin | reverse_panel_in_Pin | lowbeam_in_Pin
			| hazard_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { // CAN msg received from BMS or rear STM32
	CAN_RxHeaderTypeDef RxHeader;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if (RxHeader.StdId == 0x155) {
		char debug_buf[50];
		/* Print message ID and length first - fixed warning */
		unsigned int stdid = (unsigned int) RxHeader.StdId; // Explicit conversion
		int len = sprintf(debug_buf, "RX: ID=0x%03X LEN=%d DATA=", stdid,
				(int) RxHeader.DLC);

		HAL_UART_Transmit(&huart2, (uint8_t*) debug_buf, len, HAL_MAX_DELAY);

		/* Now print each byte as a hex value */
		for (int i = 0; i < RxHeader.DLC; i++) {
			len = sprintf(debug_buf, "%02X ", RxData[i]);
			HAL_UART_Transmit(&huart2, (uint8_t*) debug_buf, len,
			HAL_MAX_DELAY);
		}

		/* Add newline at the end */
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
	}

	if (RxHeader.StdId == 0x293) {	// BMS statusmsg
		lastBMSMsg = HAL_GetTick();

		uint8_t msgType = RxData[0];

		switch (msgType) {
		case 0x01:  // MCU Summary
			/* Set contactor state based on last two bytes */
			contactor = (RxData[6] == 0x00 && RxData[7] == 0x00);

			underTemp = (RxData[7] & 0x01) != 0;        // bit 56
			overTemp = (RxData[7] & 0x02) != 0;         // bit 57
			LVC = (RxData[7] & 0x04) != 0;              // bit 58
			HVC = (RxData[7] & 0x08) != 0;              // bit 59
			thermistorCensus = (RxData[7] & 0x10) != 0; // bit 60
			cellCensus = (RxData[7] & 0x20) != 0;       // bit 61
			hardwareFault = (RxData[7] & 0x40) != 0;    // bit 62

			/* Set plugged state based on third last byte */
			plugged = (RxData[5] != 0x01);

			/* Extract charge power from bytes 2-3 of MCU summary */
			chargekw = ((RxData[3] << 8) | RxData[2]) * 0.1f; // Charge power in kW
			break;

		case 0x02:  // Pack Summary
			/* Extract pack voltage from bytes 2-3 */
			voltage = ((RxData[3] << 8) | RxData[2]) * 0.1f; // Voltage in volts
			/* Extract current */
			int16_t current_raw = (int16_t) ((RxData[5] << 8) | RxData[4]);
			current_bms = current_raw * 0.1f;  // Current in amps
			break;

		case 0x03:  // Cell Voltage summary
			// No action needed for this message
			break;

		case 0x04:  // Thermistor data
			/* Extract maximum temperature from the third byte */
			thmax = RxData[3];  // Max temperature in °C (third byte)
			break;

		case 0x05:  // SOC summary
			/* Extract SOC */
			SOC = RxData[1];  // SOC as percentage
			break;
		}
	} else if (RxHeader.StdId == 0x154) {  // Rear STM32 msg
		// Extract temperatures (bytes 4 and 5)
		// Temperature is offset by +40, so we subtract 40 to get actual temperature
		motor_temp = (int16_t) RxData[4] - 40;      // Motor temp in byte 5
		inverter_temp = (int16_t) RxData[5] - 40;   // Inverter temp in byte 4

		// Extract fault code and fault level
		fault_code = RxData[6];
		fault_level = RxData[7] & 0x07;  // Lower 3 bits are fault level

		// Extract brake and throttle status from the highest 2 bits of byte 7
		brake_active_inv = (RxData[7] & 0x80) ? true : false;     // Bit 7
		throttle_active_inv = (RxData[7] & 0x40) ? true : false;  // Bit 6

		// Extract speed (bytes 0-1) - little endian format
		speed = ((uint16_t) ((RxData[1] << 8) | RxData[0])) / 16.0; // Convert to km/h as double

		// Extract current (bytes 2-3) - little endian format, signed value
		int16_t current_raw = (int16_t) ((RxData[3] << 8) | RxData[2]);
		current_inv = current_raw / 10.0;  // Convert to Amps

		// Update timestamp for timeout detection
		last154MsgTime = HAL_GetTick();
	} else if (RxHeader.StdId == 0x155) {  // Slow message with odometer data
		// Extract odometer (bytes 0-3, little endian format with 2 decimal places)
		uint32_t odo_raw = ((uint32_t) RxData[0]) | ((uint32_t) RxData[1] << 8)
				| ((uint32_t) RxData[2] << 16) | ((uint32_t) RxData[3] << 24);

		// Convert to kilometers with 2 decimal places
		odometer = odo_raw / 100.0;
	}
}

//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {  // update indicators every 50ms
		hazardState[0] = hazardState[1];
		hazardState[1] = hazardState[2];
		hazardState[2] = HAL_GPIO_ReadPin(hazard_in_GPIO_Port, hazard_in_Pin);
		if (hazardState[1] == hazardState[2]
				&& hazardState[0] != hazardState[1]) {
			hazardState[3] = hazardState[2];
			if (leftState[3] == GPIO_PIN_SET || rightState[3] == GPIO_PIN_SET) {
				indicatorCounter = 99;
				indicatorOn = true;
			}
		}
		leftState[0] = leftState[1];
		leftState[1] = leftState[2];
		leftState[2] = HAL_GPIO_ReadPin(l_indicator_in_GPIO_Port,
		l_indicator_in_Pin);
		if (leftState[1] == leftState[2] && leftState[0] != leftState[1]) {
			leftState[3] = leftState[2];
			if (hazardState[3] == GPIO_PIN_SET) {
				indicatorOn = true;
				indicatorCounter = 99;
			}
		}
		rightState[0] = rightState[1];
		rightState[1] = rightState[2];
		rightState[2] = HAL_GPIO_ReadPin(r_indicator_in_GPIO_Port,
		r_indicator_in_Pin);
		if (rightState[1] == rightState[2] && rightState[0] != rightState[1]) {
			rightState[3] = rightState[2];
			if (hazardState[3] == GPIO_PIN_SET) {
				indicatorOn = true;
				indicatorCounter = 99;
			}
		}

		if (indicatorCounter > 11) {
			indicatorCounter = 0;
			if (indicatorOn == false) {
				/* Turn OFF all indicators */
				indicatorOn = true;
				HAL_GPIO_WritePin(l_indicator_relay_GPIO_Port,
				l_indicator_relay_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(r_indicator_relay_GPIO_Port,
				r_indicator_relay_Pin, GPIO_PIN_RESET);
				indicator_state = 0;
			} else {
				indicatorOn = false;
				/* Set outputs based on input states */
				if (hazardState[3] == GPIO_PIN_RESET) {
					/* Both indicators ON for hazard */
					HAL_GPIO_WritePin(l_indicator_relay_GPIO_Port,
					l_indicator_relay_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(r_indicator_relay_GPIO_Port,
					r_indicator_relay_Pin, GPIO_PIN_SET);
					indicator_state = 1;
				} else if (leftState[3] == GPIO_PIN_RESET) {
					/* Left indicator ON */
					HAL_GPIO_WritePin(l_indicator_relay_GPIO_Port,
					l_indicator_relay_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(r_indicator_relay_GPIO_Port,
					r_indicator_relay_Pin, GPIO_PIN_RESET);
					indicator_state = 2;
				} else if (rightState[3] == GPIO_PIN_RESET) {
					/* Right indicator ON */
					HAL_GPIO_WritePin(l_indicator_relay_GPIO_Port,
					l_indicator_relay_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(r_indicator_relay_GPIO_Port,
					r_indicator_relay_Pin, GPIO_PIN_SET);
					indicator_state = 3;
				} else {
					/* All inputs OFF, turn off both indicators */
					HAL_GPIO_WritePin(l_indicator_relay_GPIO_Port,
					l_indicator_relay_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(r_indicator_relay_GPIO_Port,
					r_indicator_relay_Pin, GPIO_PIN_RESET);
					indicator_state = 0;
				}

			}
		} else {
			indicatorCounter = indicatorCounter + 1;
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uart_dma_busy = false;  // Mark DMA as free
	}
}
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
