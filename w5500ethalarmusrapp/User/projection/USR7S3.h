#ifndef  _USR7S3_H_
#define  _USR7S3_H_

#include "platform.h"
#define GPRS_RCV_Buf_Max 64
extern uint8_t First_Int;
extern uint8_t GPRS_SEND_STATUS;
extern char GPRS_RCV_Buf[GPRS_RCV_Buf_Max];
#define GPRS_SEND_CHECK 0x01
#define GPRS_SEND_PREP 0x02

#define GPRS_PORT 				GPIOB
#define GPRS_RST_PIN 			GPIO_Pin_9
#define GPRS_RST_RELEASE 		GPIO_SetBits(GPRS_PORT, GPRS_RST_PIN);
#define GPRS_RST 				GPIO_ResetBits(GPRS_PORT, GPRS_RST_PIN);
#define GPRS_LINK_PIN 			GPIO_Pin_4
#define GPRS_NET_PIN				GPIO_Pin_5
#define GPRS_NET_STATUS			GPIO_ReadInputDataBit(GPRS_PORT, GPRS_NET_PIN)
#define GPRS_LINK_STATUS 		GPIO_ReadInputDataBit(GPRS_PORT, GPRS_LINK_PIN)

#define SEND_TYPE_A 0x01 //无需\r\n的命令
#define SEND_TYPE_B 0x02 //需要\r\n的命令
#define SEND_TYPE_C 0x03 //不发送任何
#define COMMOND_EXCUTE_SEND_OK 0x01 

#define RETURN_OK 0xAA
#define RETURN_NOTHING 0x55

#define COMM_STATUS_OK 0x01
#define COMM_STATUS_NONE 0x02
#define COMM_STATUS_TIMEOUT 0x03


extern uint8_t GPRS_STATUS;
#define GPRS_HardWare_ERROR 0xFF
#define GPRS_CHECK 0x01
#define GPRS_RESET_OK 0x02
#define GPRS_SET_OK	0x03
#define GPRS_CONNECT_OK	0x04

extern uint8_t AT_SET_STATUS;
#define AT_SET_ATE 0x01
#define AT_SET_WKMOD 0x02
#define AT_SET_SOCKETAEN 0x03
#define AT_SET_SOCKETA 0x04
#define AT_SET_HEART 0x05
#define AT_SET_UATEN 0x06
#define AT_SET_CMDPW 0x07
#define AT_SET_CALEN 0x08
#define AT_SET_NATEN 0x09
#define AT_SET_CACHEN 0x0A
#define AT_SET_SOCKETBDISABLE 0x0B
#define AT_SET_SAVE 0x0C
#define AT_SET_RESET 0x0D
#define AT_SET_OK 0xFF

extern uint8_t AT_MODE_STATUS;
#define AT_MODE_SEND_xxx   0x01
#define AT_MODE_SEND_a   0x02
#define AT_MODE_SUCCESS 0x03

void GPRS_HARDWARE_INIT(void);
void GPRS_LOOPs(void);
void GPRS_HARDWARE_INIT(void);
void GPRS_SendDATA(uint8_t *data, uint8_t len);
void GPRS_AT_INIT_COMMOND(void);
#endif   // _USR7S3_H_

