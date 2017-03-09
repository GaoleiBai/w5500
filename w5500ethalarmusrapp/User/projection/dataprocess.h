#ifndef  _DATAPROCESS_H_
#define  _DATAPROCESS_H_

#include "platform.h"

#define DATA_BUFF_MAX 64
extern void DATA_PROC_LOOP(void);
extern void PROC_LOOPs(void);
#define BACK_COMMOND 0x01
#define BACK_DATA 0x02
#define BACK_UPGRADE 0x03
#define BACK_KEEPALIVE 0x04
#define BACK_ERROR 0x05
#define BACK_ALARM 0x06
#define BACK_ERROR_NOFRAMEHEAD 0xEE
#define BACK_ERROR_ECC 0x01
#define BACK_ERROR_PACKAGELEN 0x02
#define BACK_ERROR_DATALEN 0x03
#define BACK_ERROR_HARDWAREOP 0xAB
#define BACK_COMMOND_WRITECONFIRM 0x02
#define BACK_COMMOND_ARMINGCONFIRM 0x03
#define BACK_COMMOND_DISARMINGCONFIRM 0x04
#define BACK_COMMOND_REBOOTCONFIRM 0x04
#define BACK_COMMOND_INPUTSTATUS 0x01
#define BACK_COMMOND_VERINFO 0x06
#define BACK_ALARM_IOMISMATCH 0x03
#define BACK_ALARM_LOCALDISARMING 0x04
#define BACK_ALARM_LOCALARMING 0x05
#define BACK_ALARM_CONFIREM 0xFF
#define BACK_DATA_IPMODE 0x01
#define BACK_DATA_IPMODE_DHCP 0x50
#define BACK_DATA_IPMODE_STATICIP 0xA0
#define BACK_DATA_PORTTIME 0x02
#define BACK_DATA_IPSET 0x03
#define BACK_DATA_PORTTIME_IN 0x01
#define BACK_DATA_PORTTIME_OUT 0x02
#define BACK_DATA_PORTTIME_DARMING 0x03

#define RECV_COMMOND 0x01
#define RECV_DATA 0x02
#define RECV_UPGRADE 0x03
#define RECV_KEEPALIVE 0x04
#define RECV_ALARMCONFIRM 0x06
#define RECV_COMMOND_READIN 0x01
#define RECV_COMMOND_WRITEOUT 0x02
#define RECV_COMMOND_ARMING 0x03
#define RECV_COMMOND_DISARMING 0x04
#define RECV_COMMOND_REBOOT 0x05
#define RECV_COMMOND_VERINFO 0x06
#define RECV_COMMOND_SETDEFAULT 0x0F
#define RECV_DATA_IPMODE 0x01
#define RECV_DATA_IPMODE_DHCP 0x50
#define RECV_DATA_IPMODE_STATICIP 0xA0
#define RECV_DATA_TIMESET 0x02
#define RECV_DATA_TIMESET_IN 0x01
#define RECV_DATA_TIMESET_OUT 0x02
#define RECV_DATA_TIMESET_DARMING 0x03
#define RECV_DATA_IPSET 0x03

#define Alarm_Arming 0x00
#define Alarm_DisArming 0x01
#define Alarm_InPutMismatch 0x02

extern void data2server_loops(SOCKET s) ;

#define ArmingPort_Num 0
#define DisArmingPort_Num 1


#define NONE 0x00
#define UPDATE_FLAG_Address 0x0801FC00 //µÚ127Ò³FLASH¿Õ¼ä
#define GO_UPDATE_FLAG 0x87654321
#define DONE_UPDATE_FLAG 0x12345678
#define FrameHeader 0xAA

#define Arming 0x03
#define DisArming 0x04
#define SystemReboot 0x05
#define VerINFO 0x06
#define FactoryReset 0x0F
#define Local_Arming 0x05
#define Local_DisArming 0x06
#define NET_Arming 0x07
#define NET_DisArming 0x08
#define KEEPALIVE 0x04
#define ReadPortInput 0x01
#define OutPut_Write_Confirm 0x02
#define Arming_Confirm 0x03
#define DisArming_Confirm 0x04
#define Factory_IP_MAC 0x55

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
extern uint8_t Eth_dataRECV(SOCKET s);
extern uint8_t Gprs_dataRECV(void);
uint8_t dataPROC(DATATYPE type);
extern void SCAN_PORT(uint8_t scantype, uint8_t scancause);
extern uint8_t Arming_Proc(uint8_t type, uint8_t action);
extern bool INdata2EEPROM(uint8_t _ucRegAddr, uint8_t *_ucRegData, uint8_t len);
extern void DATA_PROC_TASK(SOCKET s);


//#define GotNoData 0xEE
//#define GotWrongData 0x01
//#define GotWrongDataLenth 0x02
//#define GotRightData 0xFF
//#define SetPinsStatusFailed 0xAB
//#define DATAPROC_SUCCESS 0xDD
//#define SAVEFAILED 0xBB
//#define IOMISMATCH 0x03
//#define LocalDisArming 0x04
//#define LocalArming 0x05
//#define HardWare_ERROR 0xAB
//#define COMMAND 0x01
//#define DATA 0x02
//#define UPGRADE 0x03
//#define KEEPALIVE 0x04
//#define WRONGDATA 0x05
//#define ALARM2SERVER 0x06
//#define ALARMCONFIRM 0x06

//#define ReadPortInput 0x01
//#define WritePortOutput 0x02
//#define Arming 0x03
//#define DisArming 0x04
//#define SystemReboot 0x05
//#define VerINFO 0x06
//#define FactoryReset 0x0F
//#define Local_Arming 0x05
//#define Local_DisArming 0x06
//#define NET_Arming 0x07
//#define NET_DisArming 0x08

//#define INPort 0x01
//#define OUTPort 0x02
//#define DIS_Arming_Port 0x03

#endif   // _DATAPROCESS_H_

