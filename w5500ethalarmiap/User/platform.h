#ifndef  _PLATFORM_H_
#define  _PLATFORM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "socket.h"	// Just include one header for WIZCHIP
#include "Internet/DHCP/dhcp.h"
#include "w5500.h"
#include "network.h"

#define UPDATE_FLAG_Address 0x0801FC00 //127th pageFLASH
#define GO_UPDATE_FLAG 0x87654321
#define DONE_UPDATE_FLAG 0x12345678

typedef  void (*pFunction)(void);
extern uint8_t sys_upgradeflag ;
extern void go_jump(void);
/* Exported Functions --------------------------------------------------------*/
typedef struct INITREG
{
   uint8_t IR_REG;
	 uint8_t SIR_REG;
	 uint8_t SnSIR_REG;
	 uint8_t SnSR_REG;
	 uint8_t PHY_LNK_REG;
}INITREG;
typedef struct PORT
{
	GPIO_TypeDef* Port;
	uint16_t Port_Pin;
	uint8_t Pin_Value;
}PORT;
typedef struct FLAGS
{
   bool SYS_Interrupt;
	 bool W5500_Dislink_Status;
	 bool Reset_Flag;
	 bool Got_Data_Flag;
	 bool W5500_Read_OK;
	 bool W5500_Send_OK;
	 bool KeepAlive_Flag;
	 bool WDG_Flag;
	 bool Scan_Pins_Flag;
	 bool Arming_Flag;
	 bool Count_Flag;
	 bool NET_Arming_Flag;
	 bool Alarm_Flag;
	 bool Alarm_Count_Flag;
}FLAGS;
typedef struct COUNT
{
   uint8_t Sec_Count;
	 uint16_t mSec_Count;
	 uint8_t Min_Count;
	 uint16_t Hou_Count;
	 uint8_t WDG_Count_Back;
	 uint8_t Arming_Count;
	 uint8_t Alarm_Count;
	 uint16_t Send_Count;
	 uint16_t W5500_Reset_Count;
	 uint16_t WDG_Count;
	 uint16_t Interrupt_Count;
}COUNT;

extern PORT IN[14];
extern PORT OUT[8];
extern PORT ARMING_PORT[2];

extern uint16_t Input_Status;
extern uint16_t Input_Status_Confirm;
extern uint8_t Output_Status;
extern uint8_t Output_Status_EEPROM;
extern uint16_t Input_Status_ARMING;
#define SCAN_ARMING SCAN_ALL
#define SCAN_LOOPS SCAN_IN

#define SCAN_IN 0x01
#define SCAN_OUT 0x02
#define SCAN_ALL 0xFF
#define MAC_SF_add 0x00
#define MAC_reg_add 0x01
#define OUT_reg_add 0x05
#define IN_ARMING_reg_add 0x0D
extern uint8_t Erro_Flag;
extern uint8_t Arming_Status;
#define INPORT_RCC_Periph RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC 
#define OUTPORT_RCC_Periph RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC 
extern void SYSRESET(void);
void PORTInit(void);
extern void W5500_init(SOCKET s);
extern void LOOPs(SOCKET s);
extern INITREG W5500_int_reg;
extern FLAGS SYS_Flags;
extern COUNT TIM_Count;
void USART_int(long BaudRate);
void W5500_GPIO_Configuration(void);
void delay_init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
extern unsigned int TIM2_COUNT;
extern unsigned int TIM2_COUNTx;
extern wiz_NetInfo gWIZNETINFO;
extern bool W5500_Interrupt;
#define PHYCFGR_LNK_STATUS 0x01
#define I2C_WR	0		/* Ð´¿ØÖÆbit */
#define I2C_RD	1		/* ¶Á¿ØÖÆbit */
#define S0_INT 0x01
extern void W5500_RESET(SOCKET s);
extern void W5500_interrupt_config(SOCKET s);
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
void platform_init(SOCKET s);
void SPI_CS_Deselect(void);
void SPI_CS_Select(void);
void SPI_CrisExit(void);
void SPI_CrisEnter(void);
uint8_t SPI_ReadByte(void);
void SPI_WriteByte(uint8_t TxData);
void SPI_Configuration(void);														
void TIMER_init(void);
void IWDG_Config(void);
void NVIC_init(void);
void WDOG_FeedBack_TASK(void);
uint32_t Flash_Read(uint32_t iAddress);
#endif   // _PLATFORM_H_


