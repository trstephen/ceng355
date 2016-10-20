#ifndef __LCD_H
#define __LCD_H

// Maps to EN and R/S bits for LCD
#define LCD_ENABLE (0x80)
#define LCD_DISABLE (0x0)
#define LCD_COMMAND (0x0)
#define LCD_DATA (0x40)

// Initializes the LCD from a fresh power-on state on the PBMCUSLK.
// It's a 2x8 character LCD with its input buffered by a shift register.
// The LCD is set to auto-increment and use 4bit data mode.
// Configures:
//   - GPIOB
//   - SPI
void LCD_Init(void);

// Turns on GPIOB so it can be used by SPI.
//  PB3 -> MOSI (Master Out Slave In)
//  PB4 -> LCK  (Load clock)
//  PB5 -> SCK  (Shift clock)
void myGPIOB_Init(void);

// Use SPI in master mode, 8b data out.
void mySPI_Init(void);

// Send data to the shift register via SPI
void HC595_Write(uint8_t word);

// Write 8b word to an LCD configured for 4b input using shift register
// @param type: Can be LCD_COMMAND or LCD_DATA
void LCD_SendWord(uint8_t type, uint8_t word);

// True if SPI can accept data to send
// @return: False / True as 0 / 1
uint8_t SPI_ReadyToSend(void);

// True if the SPI is not in use
// @return: False / True as 0 / 1
uint8_t SPI_DoneSending(void);

#endif /* __LCD_H */
