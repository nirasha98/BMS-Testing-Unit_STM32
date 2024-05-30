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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#include <string.h>

#include "usbd_cdc_if.h"
#include "string.h"
#include "usbd_cdc.h"




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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// --------------------------------------- serial port communication with PC - VARIABLES AND COMMANDS ----------------------------------------//
// commands send from PC
#define CELL_VOLTAGE_CMD "Cell Voltages"
#define TEMPERATURE_CMD "Temperatures"
#define RS485_TX_CMD "RS485_tx"
#define RS485_RX_CMD "RS485_rx"
#define CAN_TX_CMD "CAN_tx"
#define CAN_RX_CMD "CAN_rx"
#define VBUS_CMD "VBUS"
#define CONTACTOR_MAIN_CMD "Contactor_MAIN"
#define CONTACTOR_PRECHARGER_CMD "Contactor_PRECHARGER"


uint8_t buffer[64]; // buffer to store recieved command from PC


uint8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);  /* CDC_Receive_FS() function declaration */



//uint8_t cell_voltage_array[]={123,45,67,89,34,45,56,78,67,89,11,12,13,14,15,16,17,181,19,20,21,22};
//uint8_t temeperture_array[]={12,13,52,78};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





//------------------CAN variables ,functions -----------------------------------//





CAN_FilterTypeDef CANfilterConfig;  //CAN Filter Configuration Initialization

/* Define the CAN message structure */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]; // buffer to send CAN command
uint8_t RxData[8]; // buffer to store received data
uint32_t TxMailbox;





/* Define the string command to be sent */
char* string_command = "send_CAN";


// Define the commands to be sent
uint8_t Cell_voltages_command[] = {0x01};
uint8_t Temperature_command[] = {0x02};
uint8_t RS485_Tx_command[]={0x03};
uint8_t RS485_Rx_command[]={0x04};
uint8_t CAN_Tx_command[]={0x05};
uint8_t CAN_Rx_command[]={0x06};
uint8_t Vbus_command[]={0x07};
uint8_t Contactor_Main_command[]={0x08};
uint8_t Contactor_Precharger_command[]={0x09};




// defining variables to store recieved data seperately
uint8_t received_data = 0;
uint8_t Vbus_value=0;
uint8_t Contactor_Main_value =0;
uint8_t Contactor_Precharger_value=0;
uint8_t temeperture_array[4];
uint8_t cell_voltage_array[22];
uint32_t cell_voltage_array_size = sizeof(cell_voltage_array);
uint8_t my_variable=0;
uint32_t temeperture_array_size = sizeof(temeperture_array);

uint8_t CAN_tx_value = 0;
uint8_t CAN_rx_value = 0;








// ---------------------  RS485 variables , functions ----------------------------------//








uint8_t txData[] = "send_RS485";
uint8_t rxData[16]; // array to store recieved string message


uint8_t RS485_tx_value = 0;
uint8_t RS485_rx_value = 0;


//uint8_t txData[16];
//uint8_t rxData[16];

int a = 0x10;
int indx = 0; // counter variable



//function to send string command through RS485
void sendData (uint8_t *data, uint16_t size)
{
    // Transmit data over UART
    HAL_UART_Transmit(&huart1, data, size, 1000);

    // Wait for the response
    HAL_UART_Receive(&huart1, rxData, 16, 1000);
}






// ---------------------------------end variables ,functions of RS485 --------------------------------------------//






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */




    //defining RX Header parameters for CAN

	  RxHeader.DLC = 8;
	  RxHeader.IDE = CAN_ID_STD;
	  RxHeader.RTR = CAN_RTR_DATA;
	  RxHeader.StdId = 0x101;





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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */





  // ------------------------RS 485 test-------------------------------//





	  	  sendData(txData, sizeof(txData));  // transmit the rs485 command through function

       // Process the received data here


       HAL_Delay(1000);


       char* predefined_string_rs485 = "RS485 test";   // pre defined string to check communication


       if (memcmp(rxData, predefined_string_rs485, strlen(predefined_string_rs485)) == 0)  // copy the received string from bms to rxData and checking for similarity
       {
         // Set the uint8_t variable to a value if the received message matches  // transmiting and receiving fail
         //my_variable = 2;
     	  RS485_tx_value = 20;
     	  RS485_rx_value = 20;
       }
       else
       {
     	//my_variable =1;
     	  RS485_tx_value = 50;     // transmiting and receiving success
     	  RS485_rx_value = 50;
       }







       // -----------------------------CAN test-------------------------------//



           // defining tx header for CAN

     	  TxHeader.DLC = 8;
     	  TxHeader.IDE = CAN_ID_STD;
     	  TxHeader.RTR = CAN_RTR_DATA;
     	  TxHeader.StdId = 0x100;
     	  TxHeader.TransmitGlobalTime = DISABLE;






     	  //------------------CAN Filter Configuration Setup------------------//






     	   CANfilterConfig.FilterBank = 0;
     	   CANfilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
     	   CANfilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
     	   CANfilterConfig.FilterIdHigh = 0;
     	   CANfilterConfig.FilterIdLow = 0;
     	   CANfilterConfig.FilterMaskIdHigh = 0;
     	   CANfilterConfig.FilterMaskIdLow = 0;
     	   CANfilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
     	   CANfilterConfig.FilterActivation = ENABLE;
     	   CANfilterConfig.SlaveStartFilterBank = 14;







     	   //------------------CAN Communication Starting----------------------//







     	  	HAL_CAN_ConfigFilter(&hcan, &CANfilterConfig);
     	  	HAL_CAN_Start(&hcan);
     	  	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
     	  	HAL_Delay(2000);




     	  	// ------------- Checking for CAN communication ---------------------------//





     	  	  // CAN test



     	  	  /* Copy the string command to the TxData buffer */
     	  	  strcpy((char*)TxData, string_command);


     	  	  // Transmit the message
     	  	  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	  	  /* Wait for the CAN message to be transmitted */
     	  	  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);


     	  	  /* Wait for the CAN message to be received */
     	  	  while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0);


     	  	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

     	  	  char* predefined_string = "CAN test";
     	  	      //send (43 41 4E 20 74 65 73 74) from bms side

     	  	  if (memcmp(RxData, predefined_string, strlen(predefined_string)) == 0)
     	  	  {
     	  	    // Set the uint8_t variable to a value if the received message matches
     	  	    //my_variable = 2;
     	  		  CAN_tx_value = 20;    // CAN Transmiting and receiving fail
     	  		  CAN_rx_value = 20;
     	  	  }
     	  	  else
     	  	  {
     	  		//my_variable =1;
     	  		  CAN_tx_value = 50;   // CAN Transmiting and receiving success
     	  		  CAN_rx_value = 50;
     	  	  }






     	  	// ------------------------VBUS value--------------------------------//






     	   	// defining tx header parameters for V bus
     	 	  //TxHeader.DLC = 8;
     	 	  TxHeader.DLC = 1;
     	 	  TxHeader.IDE = CAN_ID_STD;
     	 	  TxHeader.RTR = CAN_RTR_DATA;
     	 	  TxHeader.StdId = 0x100;
     	 	  TxHeader.TransmitGlobalTime = DISABLE;
     	 	  TxData[0] = Vbus_command[0];




     	     // Send the vbus command message
     	     HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	     /* Wait for the CAN message to be transmitted */
     	     while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}

     	     /* Wait for the CAN message to be received */
     	     while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) < 1) {}

     	     // receiving the vbus value from bms
     	     HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, &RxHeader, RxData);

     	     // Process the response to the first command
     	     Vbus_value = RxData[0];
     	     //uint8_t receivedValue = RxData[0];

     	     // Clear the buffer before next message reception
     	     memset((uint8_t*)RxData, 0, sizeof(RxData));

     	     HAL_Delay(2000);





     	     //----------------------Contactor Main-------------------------------//






     	    // Prepare the contactor main command message to be sent
     	    TxHeader.DLC = 1;
     	    TxHeader.IDE = CAN_ID_STD;
     	    TxHeader.RTR = CAN_RTR_DATA;
     	    TxHeader.StdId = 0x100;
     	    TxData[0] = Contactor_Main_command[0];

     	    // Send the  command message
     	    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	    /* Wait for the CAN message to be transmitted */
     	    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}

     	    /* Wait for the CAN message to be received */
     	    while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) < 1) {}

     	    // Wait for the response to the  command
     	    HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, &RxHeader, RxData);

     	    // Process the response to the  command
     	    Contactor_Main_value = RxData[0];
     	    //uint8_t receivedValue = RxData[0];

     	    // Clear the buffer before next message reception
     	    memset((uint8_t*)RxData, 0, sizeof(RxData));

     	    HAL_Delay(2000);





     	   //--------------------- Contactor Precharger ---------------------------//








     	    // Prepare the contactor precharger command message to be sent
     	    TxHeader.DLC = 1;
     	    TxHeader.IDE = CAN_ID_STD;
     	    TxHeader.RTR = CAN_RTR_DATA;
     	    TxHeader.StdId = 0x100;
     	    TxData[0] = Contactor_Precharger_command[0];



     	    // Send the  command message
     	    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	    /* Wait for the CAN message to be transmitted */
     	    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}

     	    /* Wait for the CAN message to be received */
     	    while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) < 1) {}

     	    // Wait for the response to the  command
     	    HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, &RxHeader, RxData);

     	    // Process the response to the  command
     	    Contactor_Precharger_value = RxData[0];
     	    //uint8_t receivedValue = RxData[0];

     	    // Clear the buffer before next message reception
     	    memset((uint8_t*)RxData, 0, sizeof(RxData));

     	    HAL_Delay(2000);






     	    //----------------------- temperature values---------------------------//






     	    // Prepare the temperature command message to be sent
     	    TxHeader.DLC = 1;
     	    TxHeader.IDE = CAN_ID_STD;
     	    TxHeader.RTR = CAN_RTR_DATA;
     	    TxHeader.StdId = 0x100;
     	    TxData[0] = Temperature_command[0];




     	    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	    // Wait for the CAN message to be transmitted
     	    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}

     	    // Receive the temperature values
     	    for (int i = 0; i < 4; i++) {
     	        // Wait for a message to be received from the BMS
     	        while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) < 1) {}

     	        // Receive the next temperature value
     	        if (HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, &RxHeader, &temeperture_array[i]) == HAL_OK) {
     	            HAL_Delay(2000);
     	        }

     	        // Clear the buffer before next message reception
     	        memset((uint8_t*)&RxHeader, 0, sizeof(RxHeader));
     	    }







     	   //-------------------------- cell voltage values -----------------------------//







     	    // Prepare the cell voltage command message to be sent
     	    TxHeader.DLC = 1;
     	    TxHeader.IDE = CAN_ID_STD;
     	    TxHeader.RTR = CAN_RTR_DATA;
     	    TxHeader.StdId = 0x100;
     	    TxData[0] = Cell_voltages_command[0];



     	    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

     	    // Wait for the CAN message to be transmitted
     	    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}

     	    // Receive the cell voltage values
     	    for (int i = 0; i < 22; i++) {
     	        // Wait for a message to be received from the BMS
     	        while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) < 1) {}

     	        // Receive the next cell voltage value
     	        if (HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, &RxHeader, &cell_voltage_array[i]) == HAL_OK) {
     	            HAL_Delay(2000);
     	        }

     	        // Clear the buffer before next message reception
     	        memset((uint8_t*)&RxHeader, 0, sizeof(RxHeader));
     	    }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //uint8_t buffer[64];



	  	  // RS485 RX


	     if (strncmp((char*)buffer,RS485_RX_CMD,strlen(RS485_RX_CMD))==0) // read the command for rs485 received data from the buffer and check it

	      {
	        char value_str[10];  // array for store it as a string
	        sprintf(value_str,"%d\n",RS485_rx_value);  // converting the value to a string
	        CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));  // transmit it
	        //HAL_Delay(1000);
	      }



	     // RS485 TX



	      else if (strncmp((char*)buffer,RS485_TX_CMD,strlen(RS485_TX_CMD))==0)  // read the command for rs485 transmit data from the buffer and check it
	      {
	        char value_str[10];  // array for store it as a string
	        sprintf(value_str,"%d\n",RS485_tx_value);   // converting the value to a string
	        CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));  // transmit as a string, bcz this funtion works only for strings
	        //HAL_Delay(1000);
	      }




	     // CAN TX



	      else if (strncmp((char*)buffer,CAN_TX_CMD,strlen(CAN_TX_CMD))==0)  // read the command for can transmit data from the buffer and check it
	      {
	        char value_str[10];  // array for store it as a string
	        sprintf(value_str, "%d\n", CAN_tx_value);  // converting the value to a string
	        CDC_Transmit_FS((uint8_t *)value_str,strlen(value_str));
	        //HAL_Delay(1000);
	      }


	     // CAN_RX


	      else if (strncmp((char*)buffer,CAN_RX_CMD,strlen(CAN_RX_CMD))==0)
	      {
	        char value_str[10];
	        sprintf(value_str,"%d\n",CAN_rx_value);
	        CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));
	        //HAL_Delay(1000);
	      }



	     // VBUS



	      else if (strncmp((char*)buffer, VBUS_CMD, strlen(VBUS_CMD)) == 0)

	  {

		  char value_str[10];  // array for store it as a string
	  	  sprintf(value_str,"%d\n",Vbus_value);   // converting the value to a string
	  	  CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));
	  	  //HAL_Delay(2000);
	  }




	     // CELL VOLTAGES



       // read the command for cell voltages from the buffer and check it

	      else if (strncmp((char*)buffer,CELL_VOLTAGE_CMD,strlen(CELL_VOLTAGE_CMD))==0)
      {

        for (int i=0;i< cell_voltage_array_size;i++)   // for loop for sending values one by one
        {
          char value_str[10];
          sprintf(value_str,"%d\n",cell_voltage_array[i]);
          CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str)); // sending 22 values one by one - edit for 22 values
          memset(value_str, 0, sizeof(value_str));
          HAL_Delay(100); // 1 second gap in each value
        }

      }



	     // TEMPERATURES



      else if  (strncmp((char*)buffer,TEMPERATURE_CMD,strlen(TEMPERATURE_CMD))==0)  // read the command for temperatures from the buffer and check it

      {
        for (int i=0 ; i< temeperture_array_size ;i++)  // for loop for sending values one by one
        {
          char value_str[10];  // array for store it as a string
          sprintf(value_str,"%d\n",temeperture_array[i]);   // converting the value to a string
          CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));  // sending 4 values one by one - edit for 4 values
          memset(value_str, 0, sizeof(value_str));
          HAL_Delay(100);  // 1 second gap in each value
        }

      }


	     // COTACTOR MAIN



     else if (strncmp((char*)buffer,CONTACTOR_MAIN_CMD,strlen(CONTACTOR_MAIN_CMD))==0)
     {
        char value_str[10];
        sprintf(value_str,"%d\n",Contactor_Main_value);
        CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));
        //HAL_Delay(2000);
      }



	     // CONTACTOR PRECHARGER



      else if (strncmp((char*)buffer,CONTACTOR_PRECHARGER_CMD,strlen(CONTACTOR_PRECHARGER_CMD))==0)
      {
        char value_str[10];
        sprintf(value_str,"%d\n",Contactor_Precharger_value);
        CDC_Transmit_FS((uint8_t*)value_str,strlen(value_str));
        //HAL_Delay(2000);
      }

      //memset(buffer, 0, 64);
      //HAL_Delay(500);




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_EN_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
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
