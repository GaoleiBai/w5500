#ifndef  _MACRAND_H_
#define  _MACRAND_H_

#include "main.h"

#define I2C_WR	0		/* Ð´¿ØÖÆbit */
#define I2C_RD	1		/* ¶Á¿ØÖÆbit */

void InitI2C(void);
static void i2c_Delay(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
void EEPROM_WriteByte(uint8_t _ucRegAddr, uint8_t _ucRegData);
uint8_t EEPROM_ReadByte(uint8_t _ucRegAddr);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
void ADC_init(void);
int Get_ADC(void);
void ADC_disable(void);
void MAC_rand(void);
void EEPROM_MAC_check(void);


#endif   // _MACRAND_H_
