#include <main.h>

UART_HandleTypeDef huart2 = {0};
SPI_HandleTypeDef hspi1 = {0};
ICM20602_HandleTypeDef hicm20602 = {0};

volatile uint8_t data_state = 0;
volatile uint32_t exti_count = 0;

void Error_Handler(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	while(1);
}

void SystemClock_Config(void){
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_ON;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc_init.PLL.PLLM = 8;
    osc_init.PLL.PLLN = 336;
    osc_init.PLL.PLLP = RCC_PLLP_DIV2;
    osc_init.PLL.PLLQ =  7;
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
    {
        Error_Handler();
    }

   	clk_init.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
	                        | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
	clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
	clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void GPIO_Init(void){
    GPIO_InitTypeDef led = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    led.Pin = GPIO_PIN_8;
    led.Mode = GPIO_MODE_OUTPUT_PP;
    led.Pull = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &led);
}

void UART2_Init(void){
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK){
        Error_Handler();
    }
}

void SPI1_Init(void){
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if(HAL_SPI_Init(&hspi1) != HAL_OK){
        Error_Handler();
    }
}

void IMU_Init(void){
    hicm20602.pSPIHandle = &hspi1;
    hicm20602.CS_Port = GPIOA;
    hicm20602.CS_Pin = GPIO_PIN_4;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(50);
    hicm20602.Config.Gyro_FS = ICM20602_GYRO_FS_SEL_2000;
    hicm20602.Config.Accel_FS = ICM20602_ACCEL_FS_SEL_16G;
    hicm20602.Config.DLPF_Gyro = ICM20602_GYRO_DLPF_176Hz;
    hicm20602.Config.DLPF_Accel = ICM20602_ACCEL_DLPF_99Hz;
    hicm20602.Config.Smplrt_Div = SMPLRT_DIV_1000Hz;
    hicm20602.Config.Int_Pin_Cfg = ICM20602_INT_ACTIVE_HIGH 
                                | ICM20602_INT_PUSHPULL 
                                | ICM20602_INT_PULSE_50US   
                                | ICM20602_INT_CLEAR_ON_ANY_R;
    hicm20602.Config.Int_Enable  = INT_RAW_RDY_EN;
    if(ICM20602_Init(&hicm20602) != HAL_OK){
        Error_Handler();
    }
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART2_Init();
    SPI1_Init();
    IMU_Init();
    
    while(1){
        if(data_state == DATA_READY){
            data_state = DATA_EMPTY;
            if(hspi1.State == HAL_SPI_STATE_READY){
                ICM20602_ReadData(&hicm20602);
            }
        }
        if(hicm20602.data_done_read == 1){
            static uint32_t print_div = 0;
            hicm20602.data_done_read = 0;

            if (++print_div >= 50) {   // in khoảng 20 lần/giây
                print_div = 0;
                printf("Ax: %.2f, Ay: %.2f, Az: %.2f, Gx: %.2f, Gy: %.2f, Gz: %.2f\r\n",
                hicm20602.Ax, hicm20602.Ay, hicm20602.Az,
                hicm20602.Gx, hicm20602.Gy, hicm20602.Gz);
             }  
        }
    }
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    data_state = DATA_READY;
    exti_count++;
}
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}


