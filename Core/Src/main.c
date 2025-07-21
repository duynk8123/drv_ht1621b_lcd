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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Chân kết nối HT1621 trên STM32F103
#define CS1 HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET);
#define CS0 HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);
#define WR1 HAL_GPIO_WritePin(GPIOA, WR_Pin, GPIO_PIN_SET);
#define WR0 HAL_GPIO_WritePin(GPIOA, WR_Pin, GPIO_PIN_RESET);
#define DATA1 HAL_GPIO_WritePin(GPIOA, DATA_Pin, GPIO_PIN_SET);
#define DATA0 HAL_GPIO_WritePin(GPIOA, DATA_Pin, GPIO_PIN_RESET);

#define uchar   unsigned char
#define uint   unsigned int

#define DOT 0x10
#define DOT_LINE2 0x80
/*Commode:
 * 		  010a bXcX
 * 	0x40: 0100 0000 : 1/2 bias 2 common
 *	0x48: 0100 1000 : 1/2 bias 3 common
 *  0x50: 0101 0000 : 1/2 bias 4 common
 *	0x42: 0100 0010 : 1/3 bias 2 common
 *	0x4A: 0100 1010 : 1/3 bias 3 common
 *	0x52: 0101 0010 : 1/3 bias 4 common
 * */
#define  ComMode    0x50
#define  RCosc      0x30
#define  LCD_on     0x06
#define  LCD_off    0x04
#define  Sys_en     0x02
//#define  CTRl_cmd   0x80
#define  Data_cmd   0xa0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*number from 0->9 and blank*/
uint8_t number[11]={0xAF,0x06,0x6D,0x4F,0xC6,0xCB,0xEB,0x0E,0xEF,0xCF,0x00};
uint8_t number_line1[11][2]={
							{0b1010,0b1111}, //số 0
							{0b0000,0b0110}, //số 1
							{0b0110,0b1101}, //số 2
							{0b0100,0b1111}, //số 3
							{0b1100,0b0110}, //số 4
							{0b1100,0b1011}, //số 5
							{0b1110,0b1011}, //số 6
							{0b0000,0b1110}, //số 7
							{0b1110,0b1111}, //số 8
							{0b1100,0b1111}  //số 9
							};
//uint8_t number_line2[11]={0x5F,0x06,0x6B,0x2F,0x36,0x3D,0x7D,0x07,0x7F,0x3F,0x00};
uint8_t number_line2[11][2] = {
    {0b0101, 0b1111}, // 0x5F: 0101 1111
    {0b0000, 0b0110}, // 0x06: 0000 0110
    {0b0110, 0b1011}, // 0x6B: 0110 1011
    {0b0010, 0b1111}, // 0x2F: 0010 1111
    {0b0011, 0b0110}, // 0x36: 0011 0110
    {0b0011, 0b1101}, // 0x3D: 0011 1101
    {0b0111, 0b1101}, // 0x7D: 0111 1101
    {0b0000, 0b0111}, // 0x07: 0000 0111
    {0b0111, 0b1111}, // 0x7F: 0111 1111
    {0b0011, 0b1111}, // 0x3F: 0011 1111
    {0b0000, 0b0000}  // 0x00: blank
};
uint8_t num_pos_of_line_1[4] = {0x02,0x04,0x06,0x08};
uint8_t num_pos_of_line_2[4] = {0x14,0x16,0x18,0x1A};
uint8_t combined_value = 0x08 | 0x80;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void SendBit_1621(uchar sdata, uchar cnt);
void SendCmd_1621(uchar command);
void Write_1621(uchar addr, uchar sdata);
void HT1621_all_off(uint8_t totalNibbles);
void HT1621_all_on(uint8_t totalNibbles);
void Init_1621(void);
void displaydata(int p);
void display_num(uint16_t num);
void WriteMulti_1621(uint8_t startAddr, const uint8_t *pData, uint8_t length,uint8_t dot);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */
  CS1;
  DATA1;
  WR1;
  HAL_Delay(50);
  Init_1621();
  HT1621_all_on(32);
  HAL_Delay(500);
  HT1621_all_off(32);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    //Write_1621(0x00,0xFF);
	    //led hang 1
	    //display_num(1234);
	   //Write_1621(0x02,0xFF);
	    //do c,dò,ppb,WC,ppm,pappb,mm,inch
	    //Write_1621(0x0A,0xFF);
	    // neu co dong nay se hien Pa
	    //Write_1621(0x0B,0x01);
	    //Write_1621(0x0C,0xFF);
	    //Write_1621(0x0D,0xFF);
//	    for(uint8_t i =0;i<4;i++){
//	    		Write_1621(0x1D,0x01<<i);
//	    		HAL_Delay(2000);
//	    	}

	   /* Write_1621(num_pos_of_line_1[1],number[6]);
	    Write_1621(num_pos_of_line_1[2],number[4] | DOT);
	    Write_1621(num_pos_of_line_1[3],number[3]);
	    Write_1621(num_pos_of_line_2[1],number_line2[3]);
	    Write_1621(num_pos_of_line_2[2],number_line2[1]|DOT_LINE2);
	    Write_1621(num_pos_of_line_2[3],number_line2[4]);
	    */
	   // Write_1621(0x0A,0x01<<0);
	    //Write_1621(0x0E, combined_value);
	    //Write_1621(0x0F, combined_value);
	  	//Write_1621(0x10,0x01<<2);
	  // Ghi chữ số 5 (number_line1[5]) vào địa chỉ 0x02
	   //hang 1
	   WriteMulti_1621(0x04, number_line1[6] , 2, 0);
	   WriteMulti_1621(0x06, number_line1[4] , 2, DOT);
	   WriteMulti_1621(0x08, number_line1[3] , 2, 0);
	   //hang 2
	   WriteMulti_1621(0x16, number_line2[3] , 2, 0);
	   WriteMulti_1621(0x18, number_line2[1],  2 ,DOT_LINE2);
	   WriteMulti_1621(0x1A, number_line2[4] , 2, 0);
	   Write_1621(0x0B,0x01<<0);
	   Write_1621(0x0F,0x01<<3);
	   Write_1621(0x10,0x01<<3);
	   Write_1621(0x11,0x01<<2);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|WR_Pin|DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin WR_Pin DATA_Pin */
  GPIO_InitStruct.Pin = CS_Pin|WR_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SendBit_1621(uchar sdata, uchar cnt)
{
  uchar i;
  for(i = 0; i < cnt; i++)
  {
    WR0;
    if(sdata & 0x80)
    {
        DATA1;
    }
    else
    {
        DATA0;
    }
    WR1;
    sdata <<= 1;
  }
}


void SendCmd_1621(uchar command)
{
  CS0;
  SendBit_1621(0x80, 4);
  SendBit_1621(command, 8);
  CS1;
}

void Write_1621(uchar addr, uchar sdata)
{
    addr <<= 2;      // Chuyển đổi địa chỉ sang 6-bit theo yêu cầu của HT1621
    CS0;             // Kéo CS xuống để bắt đầu giao tiếp
    SendBit_1621(0xA0, 3);  // Gửi lệnh ghi dữ liệu vào RAM (3 bit)
    SendBit_1621(addr, 6);  // Gửi địa chỉ 6-bit
    // Gửi 4 bit dữ liệu:
    // Nếu sdata chỉ có ý nghĩa ở 4 bit thấp, ta cần đưa nibble đó lên vị trí bit cao (bit 7 đến bit 4)
    SendBit_1621((sdata & 0x0F) << 4, 4);
    CS1;             // Thả CS lên kết thúc giao tiếp
}


void HT1621_all_off(uint8_t totalNibbles)
{
    for(uint8_t addr = 0; addr < totalNibbles; addr++)
    {
        Write_1621(addr, 0x0); // Ghi 4 bit 0 vào từng ô nhớ
    }
}

void HT1621_all_on(uint8_t totalNibbles)
{
    for(uint8_t addr = 0; addr < totalNibbles; addr++)
    {
        Write_1621(addr, 0xF); // Ghi 4 bit 1 (0xF) vào từng ô nhớ
    }
}

void WriteMulti_1621(uint8_t startAddr, const uint8_t *pData, uint8_t length,uint8_t dot)
{
    // HT1621 sử dụng 6 bit địa chỉ,
    // nhưng ta có 32 địa chỉ (0..31) => dịch trái 2 bit để đưa vào (A5..A0).
    startAddr <<= 2;

    // Kéo CS xuống (CS0) để bắt đầu giao tiếp
    CS0;
    SendBit_1621(0xA0, 3);
    // Gửi 6 bit địa chỉ bắt đầu
    SendBit_1621(startAddr, 6);

    for (uint8_t i = 0; i < length; i++)
    {
    	//vì đưa giá trị đọc là 4 bit MSB mà hàm sendbit thì đọc từ 4 bit cao nên <<4
        uint8_t multidata = (pData[i] & 0x0F) << 4;
        if(i==0)
        	{multidata |= dot;}
        SendBit_1621(multidata, 4);
    }
    CS1;
}
void Init_1621(void)
{
  SendCmd_1621(Sys_en);
  SendCmd_1621(RCosc);
  SendCmd_1621(ComMode);
  SendCmd_1621(LCD_on);
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
