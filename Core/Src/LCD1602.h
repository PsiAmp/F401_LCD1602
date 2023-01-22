/*
 * LCD1602.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Psi
 */

#ifndef SRC_LCD1602_H_
#define SRC_LCD1602_H_

//#include "stm32f1xx_hal.h"
#include "stm32f4xx_hal.h"

typedef enum {
	LCD_LINE_ONE = 0b10000000,
	LCD_LINE_TWO = 0b11000000
} LCD_Line;

void I2C_Scan(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LCD_SendInternal(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr, uint8_t data, uint8_t flags);
void LCD_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr, uint8_t data);
void LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr);
void LCD_SendString(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr, char *str);
void LCD_SendStringToLine(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr, char *str, LCD_Line line);
void LCD_Clear(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr);
void init(I2C_HandleTypeDef *hi2c, uint8_t lcd_addr);
void loop(I2C_HandleTypeDef *hi2c);

#endif /* SRC_LCD1602_H_ */
