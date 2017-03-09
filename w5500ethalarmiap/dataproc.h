#ifndef  _DATAPROC_H_
#define  _DATAPROC_H_

#include "platform.h"

#define Alarm_Arming 0x00
#define Alarm_DisArming 0x01
#define Alarm_InPutMismatch 0x02

typedef struct ALARM
{
	uint8_t Alarm_Type;
	uint8_t Alarm_Count;
	bool Alarm_Count_Flag;
	bool Alarm_Confirm_Flag;
}ALARM;

extern ALARM Alarm2Server[3];
extern void data2server_loops(SOCKET s) ;
extern unsigned int len;
#define NONE 0x00
#define UPDATE_FLAG_Address 0x0801FC00 //µÚ127Ò³FLASH¿Õ¼ä
#define GO_UPDATE_FLAG 0x87654321
#define DONE_UPDATE_FLAG 0x12345678
#define FrameHeader 0xAA

#define GotNoData 0xEE
#define GotWrongData 0x01
#define GotWrongDataLenth 0x02
#define GotRightData 0xFF
#define SetPinsStatusFailed 0xAB
#define DATAPROC_SUCCESS 0xDD
#define SAVEFAILED 0xBB
#define IOMISMATCH 0x03
#define LocalDisArming 0x04
#define LocalArming 0x05

#define COMMAND 0x01
#define DATA 0x02
#define UPGRADE 0x03
#define KEEPALIVE 0x04
#define WRONGDATA 0x05
#define ALARM2SERVER 0x06
#define ALARMCONFIRM 0x06

#define ReadPortInput 0x01
#define WritePortOutput 0x02
#define Arming 0x03
#define DisArming 0x04
#define Local_Arming 0x05
#define Local_DisArming 0x06
#define NET_Arming 0x07
#define NET_DisArming 0x08

#define IO 0x05
#define NET 0x06

#define ArmingPinNum 2
#define InputPinNum 14
#define OutputPinNum 8

typedef unsigned char DATATYPE;
typedef unsigned char ACTIONTYPE;
typedef unsigned char ACTION;

extern 	ACTIONTYPE actiontype;
extern DATATYPE recv_type;
extern DATATYPE send_type;

extern void dataSEND(SOCKET s,DATATYPE type);
extern uint8_t dataRECV(SOCKET s);
uint8_t dataPROC(DATATYPE type);
extern void SCAN_PORT(uint8_t scantype, uint8_t scancause);
extern uint8_t Arming_Proc(uint8_t type);
#endif   // _DATAPROC_H_

