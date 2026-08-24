#include <main.h>

void HAL_UART_MspInit(UART_HandleTypeDef *huart){
    if(huart->Instance == USART2){
        GPIO_InitTypeDef uart2 = {0};
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        uart2.Pin = GPIO_PIN_2;
        uart2.Mode = GPIO_MODE_AF_PP;
        uart2.Pull = GPIO_NOPULL;
        uart2.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        uart2.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &uart2);
        uart2.Pin = GPIO_PIN_3;
        uart2.Mode = GPIO_MODE_AF_PP;
        uart2.Pull = GPIO_PULLUP;
        uart2.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &uart2);
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SPI1){
        GPIO_InitTypeDef spi1 = {0};
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        spi1.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        spi1.Mode = GPIO_MODE_AF_PP;
        spi1.Pull = GPIO_NOPULL;
        spi1.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        spi1.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &spi1);

        GPIO_InitTypeDef cs = {0};
        cs.Pin = GPIO_PIN_4;
        cs.Mode = GPIO_MODE_OUTPUT_PP;
        cs.Pull = GPIO_NOPULL;
        cs.Speed= GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &cs);

        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    }
}

void ICM20602_MspInit(ICM20602_HandleTypeDef *hicm20602){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef int_pin = {0};
    int_pin.Pin = GPIO_PIN_9;
    int_pin.Mode = GPIO_MODE_IT_RISING;
    int_pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    int_pin.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &int_pin);

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
}