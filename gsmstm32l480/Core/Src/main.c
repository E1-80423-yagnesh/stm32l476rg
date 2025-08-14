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
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct {
    char serverIP[20];
    uint16_t serverPort;
    uint16_t refreshTime;
    //char password[20];
} SystemConfig_t;
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int strength = -1;

uint16_t delay = 300;
uint16_t Delay = 8000;
char uart_buf[64];
uint16_t adc_values[8];
char imei[64];
char sms_msg[256];
char sms_buffer[512];
char gsm_time[32] = {0};
char json_string[1024];
uint8_t i;
int gsm_battery_voltage = -1;
int gsm_input_voltage = -1; // From external device

uint32_t channels[8] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void GPRS_Send_JSON(void);

int Extract_IP_Port_From_SMS(char* sms_content);
void Parse_SMS_Messages(char* sms_data);
void Check_Incoming_SMS(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SystemConfig_t systemConfig;
void GSM_Debug_Print(const char *data)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}

void GSM_SendCommand(const char *cmd, char *response, uint16_t resp_size)
{
    memset(response, 0, resp_size);
    HAL_UART_Transmit(&huart4, (uint8_t*)cmd, strlen(cmd), 100);
    HAL_UART_Receive(&huart4, (uint8_t*)response, resp_size, 100);

//    GSM_Debug_Print("CMD: ");
//    GSM_Debug_Print(cmd);
//    GSM_Debug_Print("RESP: ");
//    GSM_Debug_Print(response);
//    GSM_Debug_Print("\r\n");
}

// Initialize default configuration
void Config_Init(void)
{
    strcpy(systemConfig.serverIP, "69.62.83.250");
    systemConfig.serverPort = 4200;
    //systemConfig.refreshTime = 5; // seconds
   // strcpy(systemConfig.password, "default123");
}

void Read_ADC(uint16_t *values, uint32_t *channels, uint8_t num_channels)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    for (uint8_t i = 0; i < num_channels; i++)
    {
        sConfig.Channel = channels[i];
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;

        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
        {
            Error_Handler();
        }

        values[i] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
}

void gsm_get_time_from_module(void)
{
    char response[64] = {0};
    // Enable network time sync and save config
    GSM_SendCommand("AT+CLTS=1\r\n", response, sizeof(response));
    GSM_SendCommand("AT&W\r\n", response, sizeof(response));

    GSM_SendCommand("AT+CCLK?\r\n", response, sizeof(response));

    // Find the quote-delimited time string
    char *start = strchr(response, '\"');
    char *end = strrchr(response, '\"');
    if (start && end && (end > start)) {
        strncpy(gsm_time, start + 1, end - start - 1);
        gsm_time[end - start - 1] = '\0';
    }
}

void gsm_get_imei(void)
{
    char resp[128] = {0};
    GSM_SendCommand("AT+CGSN\r\n", resp, sizeof(resp));

    // Find the first digit sequence in the response
    char *p = resp;
    while (*p && !(*p >= '0' && *p <= '9')) {
        p++;
    }

    if (*p) {
        char *end = p;
        while (*end >= '0' && *end <= '9') {
            end++;
        }

        int imei_len = end - p;
        if (imei_len > 0 && imei_len < sizeof(imei)) {
            strncpy(imei, p, imei_len);
            imei[imei_len] = '\0';
        }
    }

}



int gsm_get_battery_voltage(void)
{
    char response[64] = {0};
    int battery_mv = -1;

    GSM_SendCommand("AT+CBC\r\n", response, sizeof(response));

    char *ptr = strstr(response, "+CBC:");
    if (ptr)
    {
        int bcs, bcl;
        if (sscanf(ptr, "+CBC: %d,%d,%d", &bcs, &bcl, &battery_mv) == 3)
        {
            return battery_mv;
        }
    }
    return battery_mv;
}

int signal_strength(void)
{
    char response[64] = {0};
    int rssi = -1, ber = -1;

    GSM_SendCommand("AT+CSQ\r\n", response, sizeof(response));

    char *ptr = strstr(response, "+CSQ:");
    if (ptr)
    {
        // Parse two integers: rssi and ber
        if (sscanf(ptr, "+CSQ: %d,%d", &rssi, &ber) == 2)
        {
            strength = rssi;
            return strength;
        }
    }
    return -1; // failed
}



void Create_JSON_String(void)
{
    // Read values
	gsm_get_imei();
    gsm_get_time_from_module();
    gsm_battery_voltage = gsm_get_battery_voltage();
    Read_ADC(adc_values, channels, 8);
    int sig_strength = signal_strength(); // get latest signal strength

    // Create JSON
    snprintf(json_string, sizeof(json_string),
        "{"
        "\"imei\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"signal_strength\":%d,"
        "\"battery_voltage\":%d,"
        "\"input_voltage\":%d,"
        "\"adc_channels\":{"
            "\"A0\":%d,"
            "\"A1\":%d,"
            "\"A2\":%d,"
            "\"A3\":%d,"
            "\"A4\":%d,"
            "\"A5\":%d,"
            "\"A6\":%d,"
            "\"A7\":%d"
        "}"
        "}",
        imei, gsm_time,
        sig_strength,
        gsm_battery_voltage, gsm_input_voltage,
        adc_values[0], adc_values[1], adc_values[2], adc_values[3],
        adc_values[4], adc_values[5], adc_values[6], adc_values[7]
    );
}




void GPRS_Send_JSON(void)
{
    char resp[512] = {0};

    // Create fresh JSON data
    Create_JSON_String();

//    GSM_Debug_Print("Sending JSON: ");
//    GSM_Debug_Print(json_string);
//    GSM_Debug_Print("\r\n");

    // 1. Basic init


    // 2. Set bearer profile
    GSM_SendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", resp, sizeof(resp));
    HAL_Delay(5);

    GSM_SendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n", resp, sizeof(resp));
    HAL_Delay(5);



    GSM_SendCommand("AT+SAPBR=1,1\r\n", resp, sizeof(resp));
    HAL_Delay(5);

    GSM_SendCommand("AT+SAPBR=2,1\r\n", resp, sizeof(resp));
    HAL_Delay(5);

    // 3. HTTP Init
//    GSM_SendCommand("AT+HTTPTERM\r\n", resp, sizeof(resp)); // ensure clean state
//    HAL_Delay(delay0);

    GSM_SendCommand("AT+HTTPINIT\r\n", resp, sizeof(resp));
    HAL_Delay(5);

    GSM_SendCommand("AT+HTTPPARA=\"CID\",1\r\n", resp, sizeof(resp));
    HAL_Delay(5);



    char http_url[256] = {0};
    snprintf(http_url, sizeof(http_url), "AT+HTTPPARA=\"URL\",\"http://%s:%d/api/test\"\r\n", systemConfig.serverIP, systemConfig.serverPort);
    GSM_SendCommand(http_url, resp, sizeof(resp));
       HAL_Delay(delay);

//    // 4. Set URL
//    char http_url[256] = {0};
//    snprintf(http_url, sizeof(http_url),
//             "AT+HTTPPARA=\"URL\",\"http://69.62.83.251:4200/api/test\"\r\n");
//    GSM_SendCommand(http_url, resp, sizeof(resp));
//    HAL_Delay(delay);

    // 5. Set content type to JSON (or x-www-form-urlencoded)
    GSM_SendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", resp, sizeof(resp));
    HAL_Delay(delay);

//    // 6. Send data length
    char data_len_cmd[64];
    snprintf(data_len_cmd, sizeof(data_len_cmd), "AT+HTTPDATA=%d,1000\r\n", (int)strlen(json_string));
    GSM_SendCommand(data_len_cmd, resp, sizeof(resp));
    HAL_Delay(delay);

    // 7. Send actual JSON data
    HAL_UART_Transmit(&huart4, (uint8_t*)json_string, strlen(json_string), HAL_MAX_DELAY);
    HAL_Delay(delay);

    // 8. HTTP POST
    GSM_SendCommand("AT+HTTPACTION=1\r\n", resp, sizeof(resp));
    HAL_Delay(300);

//    // 9. Read server response
//    GSM_SendCommand("AT+HTTPREAD\r\n", resp, sizeof(resp));
//    HAL_Delay(delay);

    // 10. End HTTP
    GSM_SendCommand("AT+HTTPTERM\r\n", resp, sizeof(resp));
    HAL_Delay(10);

    // 11. Close bearer
//    GSM_SendCommand("AT+SAPBR=0,1\r\n", resp, sizeof(resp));
//    HAL_Delay(delay);
}


void Check_Incoming_SMS(void)
{
    char response[512] = {0};

    // Set SMS text mode
    GSM_SendCommand("AT+CMGF=1\r\n", response, sizeof(response));
    HAL_Delay(100);

    // List all SMS messages (unread + read)
    GSM_SendCommand("AT+CMGL=\"ALL\"\r\n", response, sizeof(response));
    HAL_Delay(500);

    // Parse SMS messages and look for configuration updates
    Parse_SMS_Messages(response);


}

// Function to parse SMS messages and extract configuration
void Parse_SMS_Messages(char* sms_data)
{
    char *line_start = sms_data;
    char *line_end;

    while ((line_end = strstr(line_start, "\r\n")) != NULL)
    {
        *line_end = '\0'; // End the current line temporarily

        // Skip SMS headers and empty lines
        if (strstr(line_start, "+CMGL:") == NULL && strlen(line_start) > 0)
        {
            // Only process lines that start with CONFIG:
            if (strncmp(line_start, "CONFIG:", 7) == 0)
            {
                if (Extract_IP_Port_From_SMS(line_start))
                {
                    GSM_Debug_Print("Configuration updated from SMS.\r\n");
                    break; // Stop after the first successful config
                }
            }
        }

        *line_end = '\r';              // Restore original
        line_start = line_end + 2;     // Move to next line
    }
}


int Extract_IP_Port_From_SMS(char* sms_content)
{
    char temp_ip[20] = {0};
    int temp_port = 0;

    // Try to match CONFIG:IP=xxx.xxx.xxx.xxx,PORT=xxxx
    if (sscanf(sms_content, "CONFIG:IP=%19[^,],PORT=%d", temp_ip, &temp_port) == 2)
    {
        // Basic validation
        int ip_parts[4];
        if (sscanf(temp_ip, "%d.%d.%d.%d",
                   &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts[3]) == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                if (ip_parts[i] < 0 || ip_parts[i] > 255)
                    return 0; // Invalid IP
            }

            if (temp_port > 0 && temp_port <= 65535)
            {
                strncpy(systemConfig.serverIP, temp_ip, sizeof(systemConfig.serverIP) - 1);
                systemConfig.serverPort = (uint16_t)temp_port;

                // Debug print
                char debug_msg[80];
                snprintf(debug_msg, sizeof(debug_msg),
                         "Config Updated - IP: %s, Port: %d\r\n",
                         systemConfig.serverIP, systemConfig.serverPort);
                GSM_Debug_Print(debug_msg);

                return 1; // Success
            }
        }
    }

    return 0; // No match
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
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  Config_Init();

  //Check_Incoming_SMS();
  /* USER CODE BEGIN 2 */


 // HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


		Check_Incoming_SMS();
		GPRS_Send_JSON();
		HAL_Delay(Delay);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 29999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
