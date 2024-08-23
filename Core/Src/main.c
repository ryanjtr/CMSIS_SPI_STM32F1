/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void spi_gpio_config(void)
{
  // Bật xung PORTA
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  //Bật xung clock cho Alternate function I/O
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // SCK: PA5, MOSI:PA7, chế độ: output, tốc độ: 10MHz
  GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_MODE7);
  GPIOA->CRL |= (GPIO_CRL_MODE5_0 | GPIO_CRL_MODE7_0);
  // Alternate push-pull
  GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7);
  GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_CNF7_1);

  // MISO:PA6, chế độ: input floating, tốc độ
  GPIOA->CRL &= ~(GPIO_CRL_MODE6);
  // Iput floating
  GPIOA->CRL &= ~(GPIO_CRL_CNF6);
  GPIOA->CRL |= (GPIO_CRL_CNF6_0);

  // CS:PA3, chế độ: output, tốc độ 2MHz
  GPIOA->CRL &= ~(GPIO_CRL_MODE3);//input
  GPIOA->CRL |= (GPIO_CRL_MODE3_1);
  GPIOA->CRL &= ~(GPIO_CRL_CNF3);//general purpose output

  //Đặt giá trị ban đầu chân PA3 lên 1
  GPIOA->ODR |= (GPIO_ODR_ODR3);
}

void spi_config(void)
{
  // Bật xung clock ngoại vi SPI
  RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);
  // Cài đặt pha xung clock: cạnh đầu tiên của tín hiệu
  // clock là thời điểm dữ liệu bắt đầu được ghi nhận.
  SPI1->CR1 &= ~(SPI_CR1_CPHA);
  // Cài đặt cực xung clock, xung clock là 0 khi ở trạng thái rỗi
  SPI1->CR1 &= ~(SPI_CR1_CPOL);
  // Chế độ master
  SPI1->CR1 |= SPI_CR1_MSTR;
  // Baudrate fPCLK/2
  SPI1->CR1 &= ~(SPI_CR1_BR);
  // Truyền MSB trước
  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
  // Full duplex
  SPI1->CR1 &= ~(SPI_CR1_RXONLY);
  // Dữ liệu 8 bit
  SPI1->CR1 &= ~(SPI_CR1_DFF);
  // Chọn slave
  SPI1->CR1 |= (SPI_CR1_SSM);
  SPI1->CR1 |= (SPI_CR1_SSI);
  // Bật ngoại vi SPI
  SPI1->CR1 |= SPI_CR1_SPE;
  // Xóa tất cả cờ
  //Clear initial flags
//  __IO uint32_t tempRd = SPI1->SR;
  (void)SPI1->SR;


}

bool spi_transmit(uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t index = 0;
  //Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
      if (count > timeout)
      {
        return false;
      }
      else
        count++;
    }
    count=0;

  // Bật ngoại vi SPI
  SPI1->CR1 |= SPI_CR1_SPE;
  // Truyền dữ liệu
  while (index < len)
  {
    //Kiểm tra bộ đệm truyền có trống hay không
    if (SPI1->SR & SPI_SR_TXE)
    {
      SPI1->DR = pData[index];
      index++;
      count = 0;
    }
    else
    {
      if (count > timeout)
      {
        return false;
      }
      else
        count++;
    }
  }
  //Chờ cờ BUSY tắt
  while (SPI1->SR & SPI_SR_BSY)
  {
    if (count > timeout)
    {
      return false;
    }
    else
      count++;
  }
  count=0;
  //Xóa cờ OVERRUN
  (void)SPI1->DR;
  (void)SPI1->SR;
  return true;
}

uint8_t *global_txdata;
uint8_t global_txlen;
uint8_t global_txindex=0;

bool spi_transmit_it(uint8_t *pData, uint8_t len, uint32_t timeout)
{
    global_txdata = pData;
    global_txlen = len;
    global_txindex = 0; // Đặt lại chỉ số toàn cục
    uint32_t count = 0;
    // Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
        if (count > timeout)
        {
            return false;
        }
        else
            count++;
    }
    // Bật ngoại vi SPI
    SPI1->CR1 |= SPI_CR1_SPE;
    // Bật ngắt bộ đệm truyền
    SPI1->CR2 |= SPI_CR2_TXEIE;
    return true;
}


uint8_t *global_rxdata;
uint8_t global_rxindex = 0;
uint8_t global_rxlen;
bool spi_receive_it(uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  global_rxdata = pData;
  global_rxlen = len;
  global_rxindex = 0; // Đặt lại chỉ số toàn cục
  //Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
      if (count > timeout)
      {
        return false;
      }
      else
        count++;
    }
    // Bật ngoại vi SPI
    SPI1->CR1 |= SPI_CR1_SPE;
    // Bật bộ đệm ngắt
    SPI1->CR2 |= SPI_CR2_RXNEIE;

    return true;

}

bool spi_receive(uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t index = 0;

  //Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
      if (count > timeout)
      {
        return false;
      }
      else
        count++;
    }
    count=0;

  // Bật ngoại vi SPI
  SPI1->CR1 |= SPI_CR1_SPE;
  bool isTransmit = 1;
  //Truyền trước sau đó nhận dữ liệu về
  while(index<len)
  {
    //Truyền dữ liệu rác trước
    //Kiểm tra bộ đệm truyền có trống hay không và isTransmit = 1 hay không
    if ((SPI1->SR & SPI_SR_TXE) && (isTransmit))
    {
      SPI1->DR = 0xFF;
      isTransmit=0;
    }

    //Nhận dữ liệu
    if (SPI1->SR & SPI_SR_RXNE)
    {
      pData[index] = SPI1->DR ;
      index++;
      isTransmit=1;
      count=0;
    }
    else
        {
          if (count > timeout)
          {
            return false;
          }
          else
            count++;
        }
  }
  //Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
      if (count > timeout)
      {
        return false;
      }
      else
        count++;
    }
    count=0;
    //Xóa cờ OVERRUN
    (void)SPI1->DR;
    (void)SPI1->SR;
    return true;
}
bool exchange(uint8_t *data)
{
  uint8_t count=0;
  spi_transmit(data, 1,1000);
  //Chờ cờ BUSY tắt
    while (SPI1->SR & SPI_SR_BSY)
    {
      if (count > 1000)
      {
        return false;
      }
      else
        count++;
    }
   spi_receive(data, 1, 1000);
   return true;
}

void read_id_manufact()
{
  //Dùng salae để coi ID
  uint8_t data[4] = {0x9F,0x00,0x00,0x00};
  uint8_t enable_write_intruction = 0x06;

  //Kéo CS xuống mức 0
  GPIOA->ODR &=~(GPIO_ODR_ODR3);
  //Bật tính năng ghi
  spi_transmit(&enable_write_intruction, 1,1000);
  //Kéo CS lên mức 1
  GPIOA->ODR |= (GPIO_ODR_ODR3);

  GPIOA->ODR &=~(GPIO_ODR_ODR3);
  spi_transmit(data, 4, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);
  for(int i=0;i<1000;++i);//Delay
}

void write_data(uint32_t Address, uint8_t *pdata, uint16_t size)
{
    uint8_t enable_write_instruction = 0x06;
    uint8_t write_page_instruction = 0x02;

    // Kéo CS xuống mức 0 để bắt đầu giao tiếp
    GPIOA->ODR &= ~(GPIO_ODR_ODR3);
    // Bật tính năng ghi bằng lệnh Write Enable (0x06)
    spi_transmit(&enable_write_instruction, 1, 1000);
    // Kéo CS lên mức 1 để hoàn thành lệnh
    GPIOA->ODR |= (GPIO_ODR_ODR3);

    // Kéo CS xuống mức 0 để bắt đầu lệnh ghi trang
    GPIOA->ODR &= ~(GPIO_ODR_ODR3);
    // Gửi lệnh ghi trang (0x02)
    spi_transmit(&write_page_instruction, 1, 1000);
    // Gửi địa chỉ 24-bit (A23-A0)
    uint8_t addr_bytes[3];
    addr_bytes[0] = (Address >> 16) & 0xFF; // A23-A16
    addr_bytes[1] = (Address >> 8) & 0xFF;  // A15-A8
    addr_bytes[2] = Address & 0xFF;         // A7-A0
    spi_transmit(addr_bytes, 3, 1000);
    // Gửi dữ liệu cần ghi
    spi_transmit(pdata, size, 1000);
    // Kéo CS lên mức 1 để hoàn thành lệnh
    GPIOA->ODR |= (GPIO_ODR_ODR3);

    for(int i=0;i<1000;++i);//Delay
}

void read_data(uint32_t Address, uint8_t *pdata, uint16_t size)
{
    uint8_t read_page_instruction = 0x03;

    // Kéo CS xuống mức 0 để bắt đầu lệnh ghi trang
    GPIOA->ODR &= ~(GPIO_ODR_ODR3);
    // Gửi lệnh đọc (0x03)
    spi_transmit(&read_page_instruction, 1, 1000);
    // Gửi địa chỉ 24-bit (A23-A0)
    uint8_t addr_bytes[3];
    addr_bytes[0] = (Address >> 16) & 0xFF; // A23-A16
    addr_bytes[1] = (Address >> 8) & 0xFF;  // A15-A8
    addr_bytes[2] = Address & 0xFF;         // A7-A0

    spi_transmit(addr_bytes, 3, 1000);
    // Lưu dữ liệu
    spi_receive(pdata, size, 1000);
    // Kéo CS lên mức 1 để hoàn thành lệnh
    GPIOA->ODR |= (GPIO_ODR_ODR3);
    for(int i=0;i<1000;++i);//Delay
}




void read_data_it(uint32_t Address, uint8_t *pdata, uint16_t size)
{
    uint8_t read_page_instruction = 0x03;

    // Kéo CS xuống mức 0 để bắt đầu lệnh ghi trang
    GPIOA->ODR &= ~(GPIO_ODR_ODR3);
    // Gửi lệnh đọc (0x03)
    spi_transmit_it(&read_page_instruction, 1, 1000);
    // Gửi địa chỉ 24-bit (A23-A0)
    uint8_t addr_bytes[3];
    addr_bytes[0] = (Address >> 16) & 0xFF;
    addr_bytes[1] = (Address >> 8) & 0xFF;
    addr_bytes[2] = Address & 0xFF;
    spi_transmit(addr_bytes, 3, 1000);
    // Lưu dữ liệu
    spi_receive_it(pdata, size, 1000);
    // Kéo CS lên mức 1 để hoàn thành lệnh
    GPIOA->ODR |= (GPIO_ODR_ODR3);
    for(int i=0;i<1000;++i);//Delay
}


void erase_sector(uint32_t Address)
{

  uint8_t enable_write_instruction = 0x06;
  uint8_t erase_sector_instruction = 0x20;

  uint8_t addr_bytes[3];
  addr_bytes[0] = (Address >> 16) & 0xFF;
  addr_bytes[1] = (Address >> 8) & 0xFF;
  addr_bytes[2] = Address & 0xFF;

  // Bật tính năng ghi bằng lệnh Write Enable (0x06)
  GPIOA->ODR &= ~(GPIO_ODR_ODR3);
  spi_transmit(&enable_write_instruction, 1, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);

  // Gửi lệnh xóa sector
  GPIOA->ODR &= ~(GPIO_ODR_ODR3);
  spi_transmit(&erase_sector_instruction, 1, 1000);
  spi_transmit(addr_bytes, 3, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);

  for(uint32_t i=0;i<1000;++i);
}

void reset_device(void)
{
  uint8_t enable_reset_instruction = 0x66;
  uint8_t reset_instruction = 0x99;

  GPIOA->ODR &= ~(GPIO_ODR_ODR3);
  spi_transmit(&enable_reset_instruction, 1, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);

  GPIOA->ODR &= ~(GPIO_ODR_ODR3);
  spi_transmit(&reset_instruction, 1, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);
  for(int i=0;i<5000;i++);

}

void erase_chip(void)
{
  uint8_t enable_write_instruction = 0x06;
  uint8_t chip_erase_instruction = 0x60;

  // Kéo CS xuống mức 0 để bắt đầu giao tiếp
   GPIOA->ODR &= ~(GPIO_ODR_ODR3);
   // Bật tính năng ghi bằng lệnh Write Enable (0x06)
   spi_transmit_it(&enable_write_instruction, 1, 1000);
   // Kéo CS lên mức 1 để hoàn thành lệnh
   GPIOA->ODR |= (GPIO_ODR_ODR3);


  GPIOA->ODR &= ~(GPIO_ODR_ODR3);
  spi_transmit(&chip_erase_instruction, 1, 1000);
  GPIOA->ODR |= (GPIO_ODR_ODR3);

  for(int i=0;i<1000;i++);

  //Dữ liệu sau khi xóa là 0x00
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
  /* USER CODE BEGIN 2 */
  spi_gpio_config();
  spi_config();

  //Bật ngắt SPI1
  NVIC_SetPriority(SPI1_IRQn, 0);
  NVIC_EnableIRQ(SPI1_IRQn);

  uint8_t txbuffer[10]={0};
  uint8_t rxbuffer[10]={0};
  //Tạo data cho txbuffer
  for(int i=0;i<10;i++)
    txbuffer[i]=i;

//  reset_device();
  erase_chip();

//  read_id_manufact(id_rom);
  read_data(0x0000000, rxbuffer, 10);
//  erase_sector(0x000000);

  write_data(0x000000,txbuffer , 10);

  read_data(0x000000, rxbuffer, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SPI1_IRQHandler(void)
{
    // Kiểm tra ngắt TXE (Truyền dữ liệu)
    if ((SPI1->SR & SPI_SR_TXE) && (SPI1->CR2 & SPI_CR2_TXEIE))
    {

        if (global_txindex >= global_txlen) // Kiểm tra điều kiện chính xác
        {
            // Tắt ngắt bộ đệm truyền
            SPI1->CR2 &= ~(SPI_CR2_TXEIE);

            // Xóa cờ OVERRUN bằng cách đọc DR và SR
            (void)SPI1->DR;
            (void)SPI1->SR;
        }
        else
        {
          SPI1->DR = global_txdata[global_txindex];
          global_txindex++;
        }
    }
    // Kiểm tra ngắt RXNE (Nhận dữ liệu)
    //Ngắt nhận không chạy được
    if (!(SPI1->SR & SPI_SR_RXNE) && (SPI1->CR2 & SPI_CR2_RXNEIE))
    {
        if(global_rxindex >= global_rxlen)
        {
          // Tắt ngắt bộ đệm truyền
          SPI1->CR2 &= ~(SPI_CR2_RXNEIE);

          // Xóa cờ OVERRUN bằng cách đọc DR và SR
          (void)SPI1->DR;
          (void)SPI1->SR;
        }
        //Truyền dữ liệu rác trước
        //Kiểm tra bộ đệm truyền có trống hay không và isTransmit = 1 hay không
        else if (SPI1->SR & SPI_SR_TXE)
        {
          SPI1->DR = 0xFF;
          global_rxdata[global_rxindex] = SPI1->DR ;
          global_rxindex++;
        }
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
  while (1)
  {
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
