#ifndef __LCD_H
#define __LCD_H

// ----------------------------------------------------------------------------
//                      USER CONFIGURATION
// ----------------------------------------------------------------------------

// Pin configuration, must be on GPIOB
#define LCD_LCK_PIN (GPIO_Pin_4)

// Maps info to LCD rows [1,2]
#define LCD_FREQ_ROW (1)
#define LCD_RESISTANCE_ROW (2)

// ----------------------------------------------------------------------------
//                      PROTOTYPES
// ----------------------------------------------------------------------------

// Initializes the LCD from a fresh power-on state on the PBMCUSLK.
// It's a 2x8 character LCD with its input buffered by a shift register.
// The LCD is set to auto-increment and use 4bit data mode.
// Configures:
//   - GPIOB
//   - SPI
//   - TIM3
void LCD_Init(void);

// Turns on GPIOB so it can be used by SPI.
//  PB3 -> MOSI (Master Out Slave In)
//  PB4 -> LCK  (Load clock)
//  PB5 -> SCK  (Shift clock)
void myGPIOB_Init(void);

// Use SPI in master mode, 8b data out.
void mySPI_Init(void);

// Configures TIM3 for use as a delay timer
void DELAY_Init(void);

// Send data to the shift register via SPI
void HC595_Write(uint8_t word);

// Write 8b word to an LCD configured for 4b input using shift register
// @param type: Can be LCD_COMMAND or LCD_DATA
void LCD_SendWord(uint8_t type, uint8_t word);

// Convenience variant of LCD_SendWord that accepts ASCII words
// @param character: A single character e.g. "H"
void LCD_SendASCIIChar(const char* character);

// Convenience variant of LCD_SendWord that prints single digits 0:9
// @param digit: An int, 0:9
void LCD_SendDigit(uint8_t digit);

// Prints all values in the text to the LCD
// @param text: a null terminated string
void LCD_SendText(char* text);

// Clears the LCD and injects a 2ms delay to allow the LCD to finish the operation
void LCD_Clear(void);

// Execute empty loop for specified time in ms
void DELAY_Set(uint32_t milliseconds);

// True if SPI can accept data to send
// @return: False / True as 0 / 1
uint8_t SPI_ReadyToSend(void);

// True if the SPI is not in use
// @return: False / True as 0 / 1
uint8_t SPI_DoneSending(void);

// Positions the LCD cursor at row x [1,2] col y [1:8]
void LCD_MoveCursor(uint8_t row, uint8_t col);

// Write a new frequency value to the LCD
void LCD_UpdateFreq(float freq);

// Write a new resistance to the LCD;
void LCD_UpdateResistance(float resistance);

// Write a new value to the specified row, leaving the unit symbol untouched
void LCD_UpdateRow(uint8_t row, float val);

#endif /* __LCD_H */
