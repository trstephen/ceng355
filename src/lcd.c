#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "cmsis/cmsis_device.h"
#include "diag/Trace.h"
#include "lcd.h"

void LCD_Init(void){
    // Configure GPIOB to control LCD via SPI
    myGPIOB_Init();
    mySPI_Init();

    // Configure the internal LCD controls
    // PBMCUSLK has a shift register connected to the LCD like:
    //    Q7  Q6  Q5  Q4  |  Q3  Q2  Q1  Q0
    //    =================================
    //    EN  R/S NC  NC  |  D7  D6  D5  D4
    // When the LCD is initialized it's working in 8bit word mode.
    // The PBMCUSLK will interpret the 8bit word sent by SPI according to
    // the above diagram and D3->D0 = 0. The first thing we have to do is
    // shift into 4b mode operation so we can ignore D3->D0.
    // We can't use LCD_SendWord now since it's written with the assumption
    // we're in 4b mode.
    uint8_t set4bMode = 0x2;
    HC595_Write(LCD_DISABLE | LCD_COMMAND | set4bMode);
    HC595_Write(LCD_ENABLE | LCD_COMMAND | set4bMode);
    HC595_Write(LCD_DISABLE | LCD_COMMAND | set4bMode);

    // Now do the rest of the LCD config
    // 4 bits, 2 lines, 5x7 font
    LCD_SendWord(LCD_COMMAND, 0x28);

    // Display ON, No cursors
    LCD_SendWord(LCD_COMMAND, 0x0E);

    // Entry mode: Increment, No Display shifting
    LCD_SendWord(LCD_COMMAND, 0x06);

    // Clear display
    LCD_SendWord(LCD_COMMAND, 0x01);

    // Write the initial units and resistance / freq placeholders manually instead
    // of trying to pass non-int values through the number printing functions
    //  ?.?    Ω
    //  ?.?    H
    // TODO: Ω and H are static, so draw them in place
}

void myGPIOB_Init(void){
    // Turn on the GPIOB clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    // PB3 --AF0-> SPI MOSI
    // PB5 --AF0-> SPI SCK
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB4 will be manually toggled to control the output update on the shift register.
    // This will expose the contents of the register to the LCD controller.
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void mySPI_Init(void){
    // Turn on the SPI clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    SPI_InitTypeDef SPI_InitStruct;
    SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, &SPI_InitStruct);
    SPI_Cmd(SPI1, ENABLE);
}

void HC595_Write(uint8_t word){
    // We only want to expose the register values to the LCD once the new word
    // has loaded completely. We do this by toggling LCK, which controls whether
    // or not the register output tracks its current contents

    // Don't update the register output; LCK = 0
    GPIOB->BRR = GPIO_Pin_4;

    // Poll SPI until its ready to receive more data
    while (!SPI_ReadyToSend()) {
    };

    SPI_SendData8(SPI1, word);

    // Poll SPI to determine when it's finished transmitting
    while (!SPI_DoneSending()) {
    };

    // Update the output; LCK = 1
    GPIOB->BSRR = GPIO_Pin_4;
}

uint8_t SPI_ReadyToSend(void){
    // SPI can accept more data into its TX queue if TXE = 1 OR BSY = 0
    return ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET)
            || (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET));
}

uint8_t SPI_DoneSending(void){
    // SPI is done sending when BSY = 0
    return (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET);
}

void LCD_SendWord(uint8_t type, uint8_t word){
    // The high half of the register output is always reserved for EN and R/S.
    // To send an 8b target word we have to send it as two sequential 4b half-words,
    // with the target half-words occupying the lower half of the word.

    uint8_t high = ((word & 0xF0) >> 4);
    uint8_t low = word & 0x0F;

    HC595_Write(LCD_DISABLE | type | high);
    HC595_Write(LCD_ENABLE | type | high);
    HC595_Write(LCD_DISABLE | type | high);

    HC595_Write(LCD_DISABLE | type | low);
    HC595_Write(LCD_ENABLE | type | low);
    HC595_Write(LCD_DISABLE | type | low);
}

