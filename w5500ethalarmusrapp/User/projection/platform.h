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
#include "dataprocess.h"
#include "USR7S3.h"

#define DEBUG
//#define _DHCP_DEBUG_

typedef  void (*pFunction)(void);
extern uint8_t sys_upgradeflag ;
#define  ApplicationAddress0     0x08000000    //应用程序起始地址 

#define  ApplicationAddress1     0x08007000    //应用程序起始地址 

extern void jump(void);
extern bool PortData2EEProm(uint8_t _ucRegAddr, uint8_t _ucRegData);
/* Exported Functions --------------------------------------------------------*/
typedef struct INITREG
{
   uint8_t IR_REG;
	 uint8_t SIR_REG;
	 uint8_t SnSIR_REG;
	 uint8_t SnSR_REG;
	 uint8_t PHY_LNK_REG;
}INITREG;
typedef struct DATE
{
 	 uint8_t Second;
	 uint8_t Minute;
	 uint8_t Hour;
	 uint8_t Day;
	 uint8_t Month;
	 uint8_t Year;
}DATE;
typedef struct PORT
{
	GPIO_TypeDef* Port;
	uint16_t Port_Pin;
	uint8_t Pin_Value;
	uint16_t Delay_Time;
	DATE Prot_Date;
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
	 bool Wait_KeepAlive_Back_Flag;
	 bool WDG_Flag;
	 bool Scan_Pins_Flag;
	 bool Arming_Flag;
	 bool Count_Flag;
	 bool NET_Arming_Flag;
	 bool Alarm_Flag;
	 bool Alarm_Count_Flag;
	 bool DHCP_Count_Flag;
	 bool DHCP_ReCount_Flag;
	 bool GPRS_Read_OK;
	 bool GPRS_Send_Flag;
}FLAGS;
typedef struct COUNT
{
	 uint8_t ticks;
	 uint16_t mSec_Count;
	 uint8_t WDG_Count_Back;
	 uint16_t Arming_Count;
	 uint8_t Alarm_Count;
	 uint16_t Send_Count;
	 uint16_t W5500_Reset_Count;
	 uint16_t WDG_Count;
	 uint16_t Interrupt_Count;
	 uint8_t DHCP_TimeOut;
	 uint8_t DHCP_ReConnect_Count;
	 uint8_t Wait_KeepAlive_Back_Count;
}COUNT;
typedef struct ALARM
{
	uint8_t Alarm_Type;
	uint8_t Alarm_Count;
	bool Alarm_Count_Flag;
	bool Alarm_Confirm_Flag;
	DATE Alarm_Date;
}ALARM;

extern ALARM Alarm2Server[3];

extern DATE Current_Date;
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

#define MASK_HIGH 0xF0
#define MASK_LOW 0x0F


/*
*	STATUS_add:
*	0xAx:staticIP,0x5x:DHCP
*	0xxA:mac_rand,0xx5:mac_ok
*/
#define STATUS_add 0x00
#define MAC_reg_add 0x01
#define Local_IP_add 0x07
#define Local_SubNet_add 0x0B
#define Local_GateWay_add 0x10
#define Dest_IP_add 0x14
#define OUT_reg_add 0x18
#define IN_ARMING_reg_add 0x19
#define Local_Port_add 0x1B
#define Dst_Port_add 0x1D
#define HardWare_Version_add 0x20
#define SoftWare_Version_add 0x24
#define Arming_Time_add 0x28
#define DisArming_Time_add 0x2A


#define DHCP 0x01
#define Static_IP 0x02
#define IP_MODE 0x01
#define Port_Time_Set 0x02
#define Static_IP_set 0x03

extern uint8_t W5500_Status;

#define W5500_begin NULL
#define W5500_Reset_OK 0x01
#define W5500_PHY_OK 0x02
#define W5500_DHCP_OK 0x03
#define W5500_SOCK_OK 0x04

extern uint8_t ETH_STATUS;
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
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
extern unsigned int TIM2_COUNT;
extern unsigned int TIM2_COUNTx;
extern wiz_NetInfo gWIZNETINFO;
extern bool W5500_Interrupt;
#define PHYCFGR_LNK_STATUS 0x01
#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */
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
void EEPROM_check(void);
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
extern void EEPROM_GetData(uint8_t _Address, uint8_t  *buff, uint8_t len);

#endif   // _PLATFORM_H_


