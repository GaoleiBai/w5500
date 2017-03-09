/* 接收数据格式
帧头	 长度	   功能  数据     校验和
1Byte	 2Byte	1Byte	 NBytes 	 2Byte
*/

#include "dataprocess.h"
uint8_t data_ch = 0;
#define ETH_CH 0x01
#define GPRS_CH 0x02
unsigned int RunTime = 0;
uint8_t DATA_Get(SOCKET s);
uint8_t dataECC(uint16_t ECCAdd,uint16_t len);
uint8_t dataRECV_check(void) ;
void RECV_DATA_PROCESS(void);
uint8_t dataPROC(DATATYPE type);
bool Action_COMMAND(ACTIONTYPE actiontype, ACTION action);
void dataSEND(SOCKET s,DATATYPE type);
uint8_t len = 0;
uint8_t data_buff[64];
uint8_t send_buff[64];
uint8_t tmp[8];
uint16_t Input_Status;
uint16_t Back_Status;
uint16_t Input_Status_Confirm;
uint8_t Output_Status;
uint16_t Input_Status_ARMING;
DATATYPE recv_type = NONE;
DATATYPE send_type = NONE;
ACTIONTYPE actiontype = NONE;
ACTION action = NONE;
uint8_t send_sub_typeA = NONE , send_sub_typeB = NONE, send_sub_type = NONE;
ALARM Alarm2Server[3] = {
													{.Alarm_Type = Alarm_Arming, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
													{.Alarm_Type = Alarm_DisArming, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
													{.Alarm_Type = Alarm_InPutMismatch, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
												};
#define DATA_Ready2Read 0x01
#define DATA_Validable 0x02
#define DATA_FeedBack 0x03
uint8_t Data_Status = NULL;

uint8_t DATA_Get(SOCKET s) //接收字节不超过256Bytes
{
    uint8_t  cnt;
    switch (data_ch)
    {
        case ETH_CH :
            len = recv(s, data_buff, DATA_BUF_SIZE);
						printf("got lens = %d\r\n",len);
            break;
        case GPRS_CH :
            len = First_Int;
						for(cnt = 0; cnt < First_Int; cnt++)
						{
							data_buff[cnt] = GPRS_RCV_Buf[cnt];
						}
						First_Int = 0;
            break;
        default :
            break;
    }
    return len;
}

uint16_t SumProduct(uint16_t len) //生成发送求和校验数据
{
	uint16_t count,sum=0;
	for(count = 0; count < len; count ++)
	{
		sum += data_buff[count+3];
//		printf("sum[%d] = %02X",count,sum);
	}
//	printf("sum = %02X",sum);
	return sum;
}

uint8_t dataECC(uint16_t ECCAdd,uint16_t len) //输入数据求和校验
{
	uint16_t ECCRes = 0,count = 0;

		for(count = 0;count < len-2;count++)
		{
			ECCRes += data_buff[count+3];
		}
//		printf("ECCRes = %d\r\n",ECCRes);
//		printf("ECCAdd = %d\r\n",ECCAdd);
	if(ECCRes == ECCAdd)
	{return NULL;}
	else {return BACK_ERROR_ECC;}
}

uint8_t dataRECV_check(void) //接收数据判断
{
	unsigned int  packagelen = 0;
	uint16_t count = 0,fh_offset = 0,\
						data_len = 0,ECCAdd = 0,\
						fd = 0;
	packagelen = DATA_Get(0);
    if(packagelen >= 6)
    {
	    for(count = 0; count < packagelen; count++)
    	{
	    	if(data_buff[count] == FrameHeader)
	    	{
	    		fh_offset = count;
	    		goto datachecked;
	    	}
        else {;}
				send_type =  BACK_ERROR;
				send_sub_type = BACK_ERROR_NOFRAMEHEAD;//for debug
				send_sub_typeA = BACK_ERROR_NOFRAMEHEAD;
				Data_Status = DATA_FeedBack;
				return BACK_ERROR_NOFRAMEHEAD;
			}
	}
    else {;}
	send_type =  BACK_ERROR;
	send_sub_type = BACK_ERROR_PACKAGELEN;//for denug
	send_sub_typeA = BACK_ERROR_PACKAGELEN;
	Data_Status = DATA_FeedBack;
	return BACK_ERROR_PACKAGELEN;
	
datachecked:
	for(count = 0; count < packagelen - fh_offset; count++)//获取数据包
	{
		data_buff[count] = data_buff[fh_offset+count];
	}
	data_len = (((uint16_t)data_buff[1] << 8) | (uint16_t)data_buff[2]);//有效数据+功能位 长度
	if((data_len < 3) || (data_len != packagelen - fh_offset -2 -1))
	{	
		send_type =  BACK_ERROR;
		Data_Status = DATA_FeedBack;
		send_sub_type = BACK_ERROR_DATALEN;//for denug
		send_sub_typeA = BACK_ERROR_DATALEN;
		return BACK_ERROR_DATALEN;
	}
	
	ECCAdd = (((uint16_t)data_buff[packagelen - fh_offset - 2])<<8) | ((uint16_t)data_buff[packagelen - fh_offset - 1]);//去除帧头-1
	fd = dataECC(ECCAdd,data_len);
	if( fd == NULL)
	{
		Data_Status = DATA_Validable;
		return fd;
	}
	else {
				send_type =  BACK_ERROR;
				send_sub_typeA = BACK_ERROR_ECC;
				send_sub_type = BACK_ERROR_ECC;//for denug
				Data_Status = DATA_FeedBack;
				return fd;
				}
}

void RECV_DATA_PROCESS(void)
{
	recv_type = data_buff[3];
	dataPROC(recv_type);
}

/*
scantype:
	SCAN_IN 扫描输入端口
	SCAN_OUT 扫描输出端口
	SCAN_ALL 全部端口扫描
scancause：
	SCAN_LOOPS 仅扫描，不保存
	SCAN_ARMING 扫描且保存，作为布防参照数据
*/
void SCAN_PORT(uint8_t scantype, uint8_t scancause) //端口扫描,关中断
{
	uint8_t count;
	uint16_t temp;
	switch (scantype)
	{
		case SCAN_IN:
			Input_Status = 0x0000;
			for(count = 0; count < InputPinNum; count ++)
			{
				IN[count].Pin_Value = GPIO_ReadInputDataBit(IN[count].Port, IN[count].Port_Pin);
				Input_Status |= ((uint16_t)IN[count].Pin_Value) << count ;
			}
//			printf("IN status:0x%04X\r\n",Input_Status);
			if((scancause == SCAN_ARMING)&&(Input_Status_ARMING != Input_Status))
			{
				Input_Status_ARMING = Input_Status;
				temp = Input_Status;
				PortData2EEProm(IN_ARMING_reg_add, (uint8_t)(temp>>8));
				PortData2EEProm(IN_ARMING_reg_add + 1, (uint8_t)temp);
			}
			else if ((scancause == SCAN_LOOPS)&&(Input_Status_ARMING != Input_Status))
						{
							
						}
			break;

		case SCAN_OUT:
			Output_Status = 0x00;
			for(count = 0; count < OutputPinNum; count ++)
			{
				if(GPIO_ReadOutputDataBit(OUT[count].Port, OUT[count].Port_Pin))
				{OUT[count].Pin_Value = 0x01;}
				else {OUT[count].Pin_Value = 0x00;}
				Output_Status |= OUT[count].Pin_Value << count ;
			}
			break;

		case SCAN_ALL:
			Input_Status = 0x0000;
			Output_Status = 0x00;
			for(count = 0; count < InputPinNum; count ++)
			{
				IN[count].Pin_Value = GPIO_ReadInputDataBit(IN[count].Port, IN[count].Port_Pin);
				Input_Status |= ((uint16_t)IN[count].Pin_Value) << count ;
			}
			for(count = 0; count < OutputPinNum; count ++)
			{
				if(GPIO_ReadOutputDataBit(OUT[count].Port, OUT[count].Port_Pin))
				{OUT[count].Pin_Value = true;}
				else {OUT[count].Pin_Value = false;}
				Output_Status |= OUT[count].Pin_Value << count ;
			}
			if((scancause == SCAN_ARMING)&&(Input_Status_ARMING != Input_Status))
			{
				Input_Status_ARMING = Input_Status;
				Input_Status_Confirm = Input_Status_ARMING;
				temp = Input_Status;
				PortData2EEProm(IN_ARMING_reg_add, (uint8_t)(temp>>8));
				PortData2EEProm(IN_ARMING_reg_add + 1, (uint8_t)temp);
			}
			printf("Output_Status: 0x%02X,Input_Status: 0x%04X\r\n",Output_Status,Input_Status);
			break;	

		default :
			break;
	}
}

bool SET_OUT(void) //设置端口输出,关中断
{
	uint8_t count;
	Output_Status = action;
	printf("0x%02X\r\n",Output_Status);
	for(count = 0; count < OutputPinNum; count ++)
		{
			OUT[count].Pin_Value = (Output_Status >> count) & 0x01;
			if(OUT[count].Pin_Value)
				{
					GPIO_SetBits(OUT[count].Port,OUT[count].Port_Pin);
				}
			else {GPIO_ResetBits(OUT[count].Port,OUT[count].Port_Pin);}

		}
	if(Output_Status_EEPROM != action)//如果是新的指令状态，保存至EEPROM
		{	
			Output_Status_EEPROM = action;
			PortData2EEProm(OUT_reg_add, Output_Status_EEPROM);
		}
	for(count = 0; count < OutputPinNum; count ++)
		{
			if(GPIO_ReadOutputDataBit(OUT[count].Port, OUT[count].Port_Pin) != OUT[count].Pin_Value)//检测管脚输出状态
			{
				send_type = BACK_COMMOND;
				return false;
			}
			else{;}
		}
	return true;
}

/*
OPCode:
	Local_DisArming 本地端口撤防
	Local_Arming 本地端口布防
	NET_DisArming 网络撤防
	NET_Arming 网络布防
*/
uint8_t Dis_Arming_OP(uint8_t OPCode) //布撤防操作
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t count;
	if((OPCode == Local_DisArming) && (Arming_Status == Arming))
		{
			
					Arming_Status = DisArming;
					Alarm2Server[Alarm_DisArming].Alarm_Count_Flag = true;
					printf("Platform DisArming by IO...\r\n");

					goto DISARMING;
		}
			
	else if((OPCode == Local_Arming) && (Arming_Status == DisArming))
		{
				Arming_Status = Arming;
				Alarm2Server[Alarm_Arming].Alarm_Count_Flag = true;
				printf("Platform Arming by IO...\r\n");
				goto ARMING;
		}
			
	else if(OPCode ==  NET_DisArming)// && (Arming_Status == Arming))
		{
			Arming_Status = DisArming;
			printf("Platform DisArming by NET...\r\n");
			goto DISARMING;
		}
	
	else if(OPCode == NET_Arming) //&& (Arming_Status == DisArming))
		{
			Arming_Status = Arming;
			printf("Platform Arming by NET...\r\n");
			goto ARMING;
		}
		
	else {return Arming_Status;}
		
DISARMING: //通用撤防操作
					//SCAN_PORT(SCAN_ALL, SCAN_LOOPS);//将disarming前的端口状态扫描并保存
					GPIO_SetBits(OUT[0].Port, OUT[0].Port_Pin);
					GPIO_SetBits(OUT[1].Port, OUT[1].Port_Pin);
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;		 
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
					for(count = 0; count < InputPinNum; count ++)
					{
						GPIO_InitStructure.GPIO_Pin = IN[count].Port_Pin;
						GPIO_Init(IN[count].Port, &GPIO_InitStructure);
						GPIO_ResetBits(IN[count].Port,IN[count].Port_Pin);
					}
					return Arming_Status;

ARMING:  //通用布防操作
					for(count = 0; count < OutputPinNum; count ++)
					{
						OUT[count].Pin_Value = (Output_Status >> count) & 0x01;
						if(OUT[count].Pin_Value)
							{
								GPIO_SetBits(OUT[count].Port,OUT[count].Port_Pin);
							}
						else {GPIO_ResetBits(OUT[count].Port,OUT[count].Port_Pin);}
					}
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		 
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
					for(count = 0; count < InputPinNum; count ++)
					{
						GPIO_InitStructure.GPIO_Pin = IN[count].Port_Pin;
						GPIO_Init(IN[count].Port, &GPIO_InitStructure);
					}
					delay_ms(1);//端口方向发生变化后，稍作延时，避免回读出错
					SCAN_PORT(SCAN_ALL, SCAN_ARMING);
					return Arming_Status;
}

uint8_t Arming_Proc(uint8_t type, uint8_t action) //布撤防处理，调用Dis_Arming_OP
{
	uint8_t fb=0;
	switch(type)
	{
	case IO :
			ARMING_PORT[ArmingPort_Num].Pin_Value = GPIO_ReadInputDataBit(ARMING_PORT[ArmingPort_Num].Port, ARMING_PORT[ArmingPort_Num].Port_Pin);
			ARMING_PORT[1].Pin_Value = GPIO_ReadInputDataBit(ARMING_PORT[1].Port, ARMING_PORT[1].Port_Pin);
			if((!ARMING_PORT[ArmingPort_Num].Pin_Value) && (Arming_Status == Arming))
			{
				SYS_Flags.Count_Flag = true;
				if(TIM_Count.Arming_Count >= ARMING_PORT[ArmingPort_Num].Delay_Time)// 撤防，打开继电器CH1&CH2
				{
					fb = Dis_Arming_OP(Local_DisArming);
					send_sub_typeA = BACK_ALARM_LOCALDISARMING;
					send_type = BACK_ALARM;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Second = Current_Date.Second;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Minute = Current_Date.Minute;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Hour = Current_Date.Hour;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Day = Current_Date.Day;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Month = Current_Date.Month;
					ARMING_PORT[DisArmingPort_Num].Prot_Date.Year = Current_Date.Year;
					Arming_Status = DisArming;
					Alarm2Server[Alarm_DisArming].Alarm_Count_Flag = true;
					Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag = false;
				}
			}
			else if((!ARMING_PORT[1].Pin_Value) && (Arming_Status == DisArming))
			{
				SYS_Flags.Count_Flag = true;
				if(TIM_Count.Arming_Count >= ARMING_PORT[1].Delay_Time)//布防，恢复继电器状态
				{
					fb = Dis_Arming_OP(Local_Arming);
					send_sub_typeA = BACK_ALARM_LOCALARMING;
					send_type = BACK_ALARM;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Second = Current_Date.Second;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Minute = Current_Date.Minute;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Hour = Current_Date.Hour;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Day = Current_Date.Day;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Month = Current_Date.Month;
					ARMING_PORT[ArmingPort_Num].Prot_Date.Year = Current_Date.Year;
					Arming_Status = Arming;
					Alarm2Server[Alarm_Arming].Alarm_Count_Flag = true;
					Alarm2Server[Alarm_Arming].Alarm_Confirm_Flag = false;
				}
			}
			else
				{
					SYS_Flags.Count_Flag = false;
					TIM_Count.Arming_Count = 0 ;
				}
		break;
				
	case NET:

			Arming_Status = Dis_Arming_OP(action);
			fb = Arming_Status;
			if(Arming_Status == Arming)
			{
				SYS_Flags.NET_Arming_Flag = true;
			}
			else {SYS_Flags.NET_Arming_Flag = false;}
		break;
					
	default :
		break;
	}
	return fb;
}


bool Action_COMMAND(ACTIONTYPE actiontype, ACTION action) //硬件操作
{
	uint8_t cnt, buff[4];
	bool fb = false;
	switch(actiontype)
	{
		case RECV_COMMOND_READIN : 
			SCAN_PORT(SCAN_ALL, SCAN_LOOPS);
			fb = true;
			break;		
		case RECV_COMMOND_WRITEOUT :
				fb = SET_OUT();
			break;
		case RECV_COMMOND_ARMING :
			if(Arming_Proc(NET, NET_Arming) == Arming)
				{
					fb = true;
					SYS_Flags.NET_Arming_Flag = true;
				}
			else {fb = false;}
			break;
		case RECV_COMMOND_DISARMING :
			if(Arming_Proc(NET, NET_DisArming) == DisArming)
				{
					fb = true;
					SYS_Flags.NET_Arming_Flag = false;
				}
			else {fb = false;}
			break;
		case RECV_COMMOND_REBOOT :
			SYSRESET();
		break;
		case RECV_COMMOND_VERINFO :
			EEPROM_GetData(HardWare_Version_add, buff, 4);
			printf("HardWare_Version: %d.%d.%02d.%02d\r\n", buff[0],buff[1],buff[2],buff[3]);
			for(cnt = 0; cnt < 4; cnt++)
				{
					tmp[cnt] = buff[cnt];
				}
			EEPROM_GetData(SoftWare_Version_add, buff, 4);
			printf("SoftWare_Version: %d.%d.%02d.%02d\r\n", buff[0],buff[1],buff[2],buff[3]);
			for(cnt = 0; cnt < 4; cnt++)
				{
					tmp[cnt + 4] = buff[cnt];
				}
			send_type = RECV_COMMOND;
			actiontype = RECV_COMMOND_VERINFO;
			fb = true;
		break;
		case RECV_COMMOND_SETDEFAULT :
			EEPROM_WriteByte(STATUS_add, Factory_IP_MAC);
			delay_ms(3);
			if(EEPROM_ReadByte(STATUS_add) == Factory_IP_MAC)
			{
						printf("save indata to eeprom OK.....\r\n");
			}
			else
			{
				printf("save indata to eeprom FAILED....\r\n");
				fb = false;
			}
			if(INdata2EEPROM(Local_IP_add, buff,4) != true || \
				INdata2EEPROM(Local_GateWay_add, buff,4) != true || \
				INdata2EEPROM(Local_SubNet_add, buff,4) != true || \
				INdata2EEPROM(Dest_IP_add, buff,4) != true || \
				INdata2EEPROM(Local_Port_add, buff,2) != true || \
				INdata2EEPROM(Dst_Port_add, buff,2) != true )
				{fb = false;}		
			fb = true;
			send_type = RECV_COMMOND;
			actiontype = RECV_COMMOND_SETDEFAULT;
		break;
		default : 
			break;
	}
	return fb;
}


bool Port_DelayTime_Set(void)
{
	uint8_t Port_type,buff[2];
	bool fb = false;
	uint16_t tmp,temp;
	Port_type = data_buff[5];
	switch (Port_type)
		{
			case RECV_DATA_TIMESET_IN :
				send_sub_typeB = BACK_DATA_PORTTIME_IN;
			break;
			
			case RECV_DATA_TIMESET_OUT :
				send_sub_typeB = BACK_DATA_PORTTIME_OUT;
			break;
			
			case RECV_DATA_TIMESET_DARMING :
				tmp = ((uint16_t)data_buff[6] << 8) | (uint16_t)data_buff[7] ;
				temp = ((uint16_t)data_buff[8] << 8) | (uint16_t)data_buff[9] ;
				if(tmp == ArmingPort_Num)
					{
						ARMING_PORT[ArmingPort_Num].Delay_Time = temp * 100;//100ms起步
						buff[0] = (uint8_t)(ARMING_PORT[ArmingPort_Num].Delay_Time >> 8);
						buff[1] =(uint8_t) ARMING_PORT[ArmingPort_Num].Delay_Time;
						fb = INdata2EEPROM(Arming_Time_add, buff, 2);
					}
				else if(tmp == DisArmingPort_Num)
					{
						ARMING_PORT[DisArmingPort_Num].Delay_Time = temp * 100;
						buff[0] = (uint8_t)(ARMING_PORT[DisArmingPort_Num].Delay_Time >> 8);
						buff[1] =(uint8_t) ARMING_PORT[DisArmingPort_Num].Delay_Time;
						fb = INdata2EEPROM(DisArming_Time_add, buff, 2);
					}
				else{;}
				if(fb == true)
				{send_sub_typeB = BACK_DATA_PORTTIME_DARMING;}
				else{;}
				
			break;

			default :
			break;
				
		}
	return fb;
}

bool INdata_process(void)
{
	uint8_t data_type,IPMODE,temp,buff[2];
	bool fb;
	data_type = data_buff[4];
	switch (data_type)
	{
		case RECV_DATA_IPMODE :
			IPMODE = data_buff[5] & MASK_HIGH;
			send_sub_typeB = IPMODE;
			if((IPMODE == RECV_DATA_IPMODE_DHCP) || (IPMODE == RECV_DATA_IPMODE_STATICIP))
			{
				temp = EEPROM_ReadByte(STATUS_add);
				if((temp & MASK_HIGH) != IPMODE)
				{
					EEPROM_WriteByte(STATUS_add, (temp & MASK_LOW) | IPMODE );
					delay_ms(3);
					if(EEPROM_ReadByte(STATUS_add) == ((temp & MASK_LOW) | IPMODE))
					{
						printf("save indata to eeprom OK.....\r\n");
						fb = true;
					}
					else
					{
						printf("save indata to eeprom FAILED....\r\n");
						fb = false;
					}
				}
				else{
						printf("no need to save indata..\r\n");
						fb = true;
					  }
				send_sub_typeA = BACK_DATA_IPMODE;
			}
			else{;}
		break;
					
		case RECV_DATA_TIMESET :
			fb = Port_DelayTime_Set();
			if(fb == true)
			{send_sub_typeA = BACK_DATA_PORTTIME;}
			else{;}
		break;
		
		case RECV_DATA_IPSET :
			gWIZNETINFO.ip[0] = data_buff[5];
			gWIZNETINFO.ip[1] = data_buff[6];
			gWIZNETINFO.ip[2] = data_buff[7];
			gWIZNETINFO.ip[3] = data_buff[8];
			fb = INdata2EEPROM(Local_IP_add, gWIZNETINFO.ip,4);
			if(!fb) {return fb = false;}
			else{;}
			gWIZNETINFO.gw[0] = data_buff[9];
			gWIZNETINFO.gw[1] = data_buff[10];
			gWIZNETINFO.gw[2] = data_buff[11];
			gWIZNETINFO.gw[3] = data_buff[12];
			fb = INdata2EEPROM(Local_GateWay_add, gWIZNETINFO.gw,4);
			if(!fb) {return fb = false;}
			else{;}
			gWIZNETINFO.sn[0] = data_buff[13];
			gWIZNETINFO.sn[1] = data_buff[14];
			gWIZNETINFO.sn[2] = data_buff[15];
			gWIZNETINFO.sn[3] = data_buff[16];
			fb = INdata2EEPROM(Local_SubNet_add, gWIZNETINFO.sn,4);
			if(!fb) {return fb = false;}
			else{;}
			DstIP[0] = data_buff[17];
			DstIP[1] = data_buff[18];
			DstIP[2] = data_buff[19];
			DstIP[3] = data_buff[20];
			fb = INdata2EEPROM(Dest_IP_add, DstIP,4);
			if(!fb) {return fb = false;}
			else{;}
			buff[0] = data_buff[21];
			buff[1] = data_buff[22];
			LocalPort = (((uint16_t )data_buff[21]) << 8) | ((uint16_t )data_buff[22]);
			fb = INdata2EEPROM(Local_Port_add, buff,2);
			if(!fb) {return fb = false;}
			else{;}
			buff[0] = data_buff[23];
			buff[1] = data_buff[24];
			DstPort = (((uint16_t )data_buff[23]) << 8) | ((uint16_t )data_buff[24]);
			fb = INdata2EEPROM(Dst_Port_add, buff,2);
			if(!fb) {return fb = false;}
			else{;}
			
			printf("mac : %02X.%02X.%02X\r\n", gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
			printf("LocalIP : %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
			printf("GateWay : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
			printf("SubNet : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
			printf("DstIP : %d.%d.%d.%d\r\n", DstIP[0],DstIP[1],DstIP[2],DstIP[3]);
			printf("LocalPort : %d\r\n", LocalPort);
			printf("DstPort : %d\r\n", DstPort);
			send_sub_typeA = BACK_DATA_IPSET;
		break;
		
		default:
		break;
	}
	return fb;
}


uint8_t dataPROC(DATATYPE type) //接收数据后完成规定动作
{
	uint8_t FB = 0;
	Data_Status = DATA_FeedBack;
	actiontype = data_buff[4];
		switch(type)
		{
			case RECV_COMMOND : 
					printf("Got a COMMAND.....\r\n");
					action = data_buff[5];
					FB = Action_COMMAND(actiontype,action);
					if(!FB)
					{
						send_type = BACK_ERROR;
						send_sub_typeA = BACK_ERROR_HARDWAREOP;
						send_sub_type = BACK_ERROR_HARDWAREOP;//for denug
					}
					else {
							send_type = BACK_COMMOND;
						}
				break;
						
			case RECV_DATA :
				printf("Got a data.....\r\n");
					FB = INdata_process();
					if(!FB)
					{
						send_type = BACK_ERROR;
						send_sub_type = BACK_ERROR_HARDWAREOP;//for denug
						send_sub_typeA = BACK_ERROR_HARDWAREOP;
					}
					else {
							send_type = BACK_DATA;
						}
				break;
						
			case RECV_UPGRADE :
				printf("Got a update.....\r\n");
				FLASH_Unlock();
				FLASH_ErasePage(UPDATE_FLAG_Address);
				FLASH_ProgramWord(UPDATE_FLAG_Address,GO_UPDATE_FLAG); 
				FLASH_Lock();
				SYSRESET();
				break;
			
			case RECV_KEEPALIVE :
				printf("Got a keepalive.....\r\n");
				SYS_Flags.Wait_KeepAlive_Back_Flag = false;
				TIM_Count.Wait_KeepAlive_Back_Count = 0;
				if(data_buff[2] > 3)//时间心跳包
				{
					Current_Date.Year = data_buff[4];
					Current_Date.Month = data_buff[5];
					Current_Date.Day = data_buff[6];
					Current_Date.Hour = data_buff[7];
					Current_Date.Minute = data_buff[8];
					Current_Date.Second = data_buff[9];
				}
				else {;}
				FB = true;
//				send_type = BACK_KEEPALIVE;
					//心跳不回复
				recv_type = NONE;
				send_type = NONE;
				send_sub_typeA = NONE;
				Data_Status = NONE;
				break;
					
			case RECV_ALARMCONFIRM :
				printf("Got a ALARMCONFIRM.....\r\n");
				Input_Status_Confirm = Input_Status;
				SYS_Flags.Alarm_Flag = false;
				Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag = true;
				Alarm2Server[Alarm_Arming].Alarm_Confirm_Flag = true;
				Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag = true;
				FB = true;
			//警告确认不回复
				recv_type = NONE;
				send_type = NONE;
				send_sub_typeA = NONE;
				Data_Status = NONE;
//				send_type = BACK_ALARM;
//				send_sub_typeA = BACK_ALARM_CONFIREM;
				break;
			
			default :
				break;
		}
	return FB;
}



void DATA_INIT(void)
{
	recv_type = NONE;
	send_type = NONE;
	actiontype = NONE;
	action = NONE;
	send_sub_typeA = NONE;
	send_sub_typeB = NONE;
	send_sub_type = NONE;
	memset(gDATABUF, 0, DATA_BUF_SIZE);
	memset(data_buff, 0, sizeof(data_buff));
}

void data2server(SOCKET s)
{
	#ifdef DEBUG
	switch (send_sub_type)
	{
		case BACK_ERROR_NOFRAMEHEAD:
			printf("BACK_ERROR_NOFRAMEHEAD\r\n");
		break;
		case BACK_ERROR_PACKAGELEN:
			printf("BACK_ERROR_PACKAGELEN\r\n");
			break;
		case BACK_ERROR_ECC:
			printf("BACK_ERROR_ECC\r\n");
			break;
		case BACK_ERROR_HARDWAREOP:
			printf("BACK_ERROR_HARDWAREOP\r\n");
			break;
		case NULL:
			printf("BACK_NORMAL\r\n");
			break;
		case BACK_ERROR_DATALEN:
			printf("BACK_ERROR_DATALEN\r\n");
			break;
		default :
			break;
	}
	#endif
	dataSEND(s,send_type);
	Data_Status = NULL;
}

void sendprocess(SOCKET s, uint8_t send_lenth)
{
	uint8_t lens;
	if((W5500_Status == W5500_SOCK_OK) && SYS_Flags.W5500_Send_OK)
	{
//		printf("send data by w5500......\r\n");
		lens = send(s,data_buff,send_lenth);
		while((getSn_IR(0)&0x10) != 0x10);
		setSn_IR(s,  getSn_IR(s)&0x1F);
		printf("send lens = %d\r\n",lens);
	//	SYS_Flags.W5500_Send_OK = false;
	}
	else if(GPRS_SEND_STATUS == GPRS_SEND_PREP)
				{
//					printf("send data by GPRS......\r\n");
					GPRS_SendDATA(data_buff, send_lenth);
					SYS_Flags.GPRS_Send_Flag = false;
				}
			else {;}
	DATA_INIT();
}

void dataSEND(SOCKET s,DATATYPE type) //发送数据(是否需要使用结构体填充的方式？)
{
	uint16_t sum = 0, len = 0;
	memset(data_buff,0,32);
	switch(type)
		{
			case BACK_COMMOND : 
				printf("Send a COMMAND.....\r\n");
				if(actiontype == RECV_COMMOND_READIN)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x07;
					data_buff[3] = type;
					data_buff[4] = ReadPortInput;
					data_buff[5] = (uint8_t)(Input_Status >> 8);
					data_buff[6] = (uint8_t)Input_Status;
					data_buff[7] = Output_Status;
					sum = SumProduct(data_buff[2]-2);
					data_buff[8] = (uint8_t)(sum >> 8);
					data_buff[9] = (uint8_t)(sum & 0xFF);
					len = 10;
				}
				else{;}
				if(actiontype == RECV_COMMOND_WRITEOUT)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x04;
					data_buff[3] = type;
					data_buff[4] = OutPut_Write_Confirm;
					sum = SumProduct(data_buff[2]-2);
					data_buff[5] = (uint8_t)(sum >> 8);
					data_buff[6] = (uint8_t)(sum & 0xFF);
					len = 7;
				}
				else{;}
				if((actiontype == RECV_COMMOND_ARMING) || (actiontype == RECV_COMMOND_DISARMING))
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x04;
					data_buff[3] = type;
					if(actiontype == RECV_COMMOND_ARMING)
					{	
						data_buff[4] = Arming_Confirm;
					}
					else if(actiontype == RECV_COMMOND_DISARMING)
							{
								data_buff[4] = DisArming_Confirm;
							}
					sum = SumProduct(data_buff[2]-2);
					data_buff[5] = (uint8_t)(sum >> 8);
					data_buff[6] = (uint8_t)(sum & 0xFF);
					len = 7;
				}
				else{;}
				if(actiontype == RECV_COMMOND_VERINFO)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x0c;
					data_buff[3] = type;
					data_buff[4] = actiontype;
					data_buff[5] = tmp[0];
					data_buff[6] = tmp[1];
					data_buff[7] = tmp[2];
					data_buff[8] = tmp[3];
					data_buff[9] = tmp[4];
					data_buff[10] = tmp[5];
					data_buff[11] = tmp[6];
					data_buff[12] = tmp[7];
					sum = SumProduct(data_buff[2]-2);
					data_buff[13] = (uint8_t)(sum >> 8);
					data_buff[14] = (uint8_t)(sum & 0xFF);
					len = 15;
				}
				else{;}
				if(actiontype == RECV_COMMOND_SETDEFAULT)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x07;
					data_buff[3] = type;
					data_buff[4] = actiontype;
					sum = SumProduct(data_buff[2]-2);
					data_buff[5] = (uint8_t)(sum >> 8);
					data_buff[6] = (uint8_t)(sum & 0xFF);
					len = 7;
				}
				else{;}
				break;
			case BACK_DATA :
				printf("Send a data.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x05;
				data_buff[3] = type;
				data_buff[4] = send_sub_typeA;
				if(send_sub_typeA == BACK_DATA_IPSET)
				{
					data_buff[5] = 0x00;
				}
				else if(send_sub_typeA == BACK_DATA_IPMODE)
							{
								data_buff[5] = send_sub_typeB;
							}
							else if(send_sub_typeA == BACK_DATA_PORTTIME)
										{
											data_buff[5] = send_sub_typeB;
										}
										else {;}
				sum = SumProduct(data_buff[2]-2);
				data_buff[6] = (uint8_t)(sum >> 8);
				data_buff[7] = (uint8_t)(sum & 0xFF);
				len = 8;
				break;
			case BACK_UPGRADE :
				printf("Send a UPGRADE.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x03;
				data_buff[3] = type;
				sum = SumProduct(data_buff[2]-2);
				data_buff[4] = (uint8_t)(sum >> 8);
				data_buff[5] = (uint8_t)(sum & 0xFF);
				len = 6;
				break;
			case BACK_KEEPALIVE :
				if(!SYS_Flags.Wait_KeepAlive_Back_Flag)
				{return;}
				printf("Send a keepalive.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x09;
				data_buff[3] = type;
				data_buff[4] = gWIZNETINFO.mac[0];
				data_buff[5] = gWIZNETINFO.mac[1];
				data_buff[6] = gWIZNETINFO.mac[2];
				data_buff[7] = gWIZNETINFO.mac[3];
				data_buff[8] = gWIZNETINFO.mac[4];
				data_buff[9] = gWIZNETINFO.mac[5];
				sum = SumProduct(data_buff[2]-2);
				data_buff[10] = (uint8_t)(sum >> 8);
				data_buff[11] = (uint8_t)(sum & 0xFF);
				len = 12;
				break;
			case BACK_ERROR :
				printf("Send a dataErro.....\r\n");
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x04;
					data_buff[3] = type;
					data_buff[4] = send_sub_typeA;
					sum = SumProduct(data_buff[2]-2);
					data_buff[5] = (uint8_t)(sum >> 8);
					data_buff[6] = (uint8_t)(sum & 0xFF);
					len = 7;
				break;
			case BACK_ALARM :
				printf("Send a ALARM2SERVER.....\r\n");
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x0C;
					data_buff[3] = type;
					data_buff[4] = send_sub_typeA;
					if(send_sub_typeA == BACK_ALARM_IOMISMATCH)
					{
						data_buff[5] = (uint8_t)(Back_Status >> 8);
						data_buff[6] = (uint8_t)Back_Status;
						data_buff[7] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Year;
						data_buff[8] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Month;
						data_buff[9] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Day;
						data_buff[10] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Hour;
						data_buff[11] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Minute;
						data_buff[12] = Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Second;
					}
					else if((send_sub_typeA == BACK_ALARM_LOCALDISARMING) || (send_sub_typeA == BACK_ALARM_LOCALARMING) || (send_sub_typeA == BACK_ALARM_CONFIREM))
						{
							data_buff[5] = 0x00;
							data_buff[6] = 0x00;
							if(send_sub_typeA == BACK_ALARM_LOCALDISARMING)
							{
								data_buff[7] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Year;
								data_buff[8] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Month;
								data_buff[9] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Day;
								data_buff[10] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Hour;
								data_buff[11] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Minute;
								data_buff[12] = ARMING_PORT[DisArmingPort_Num].Prot_Date.Second;
							}
							else if(send_sub_typeA == BACK_ALARM_LOCALARMING)
									{
										data_buff[7] = ARMING_PORT[ArmingPort_Num].Prot_Date.Year;
										data_buff[8] = ARMING_PORT[ArmingPort_Num].Prot_Date.Month;
										data_buff[9] = ARMING_PORT[ArmingPort_Num].Prot_Date.Day;
										data_buff[10] = ARMING_PORT[ArmingPort_Num].Prot_Date.Hour;
										data_buff[11] = ARMING_PORT[ArmingPort_Num].Prot_Date.Minute;
										data_buff[12] = ARMING_PORT[ArmingPort_Num].Prot_Date.Second;
									}
							else if (send_sub_typeA == BACK_ALARM_CONFIREM)
									{
										data_buff[7] = Current_Date.Year;
										data_buff[8] = Current_Date.Month;
										data_buff[9] = Current_Date.Day;
										data_buff[10] = Current_Date.Hour;
										data_buff[11] = Current_Date.Minute;
										data_buff[12] = Current_Date.Second;
									}
								else{;}
						}
					else {;}
					sum = SumProduct(data_buff[2]-2);
					data_buff[13] = (uint8_t)(sum >> 8);
					data_buff[14] = (uint8_t)(sum & 0xFF);
					len = 15;
						printf("Back_Status:0x%04X............\r\n",Back_Status);
				break;
			default :
				break;
		}
	sendprocess(s, len);
}

void KeepAlive_Proc(SOCKET s)
{
		if(SYS_Flags.KeepAlive_Flag)//心跳数据包发送
		{
			SYS_Flags.KeepAlive_Flag = false;
			SYS_Flags.Wait_KeepAlive_Back_Flag = true;//等待心跳包回复
			dataSEND(s,KEEPALIVE);
			printf("mS:%d,S:%d,M:%d,H:%d,D:%d,M:%d,Y:%d...\r\n",\
					TIM_Count.mSec_Count,Current_Date.Second,Current_Date.Minute,Current_Date.Hour,\
					Current_Date.Day,Current_Date.Month,Current_Date.Year);
			RunTime ++;
			printf("RunTime = %d Mins....\r\n",RunTime/6);
		}
		else {;}
		if(TIM_Count.Wait_KeepAlive_Back_Count >= 30)
		{
			W5500_Status = W5500_begin;
//			printf("W5500_begin.....\r\n");
			SYS_Flags.Wait_KeepAlive_Back_Flag = false;
			TIM_Count.Wait_KeepAlive_Back_Count = 0;
		}
		else{;}
}

void alarm2server_loops(SOCKET s) //未完
{
	if((Arming_Status == Arming) && SYS_Flags.SYS_Interrupt)//布防期间端口扫描及上报,5s无确认继续上报,扫描周期100ms
		{		
			SCAN_PORT(SCAN_IN, SCAN_LOOPS);
			if(Input_Status != Input_Status_Confirm)
				{ 
					Alarm2Server[Alarm_InPutMismatch].Alarm_Count_Flag = true;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag = false;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Second = Current_Date.Second;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Minute = Current_Date.Minute;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Hour = Current_Date.Hour;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Day = Current_Date.Day;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Month = Current_Date.Month;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Date.Year = Current_Date.Year;
				}
				Back_Status = Input_Status ^ Input_Status_ARMING;
		}
		else {;}
//发送告警信息
	if((Alarm2Server[Alarm_InPutMismatch].Alarm_Count >= 2) && !Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag)
		{
			send_type = BACK_ALARM;
			Alarm2Server[Alarm_InPutMismatch].Alarm_Count_Flag = false;
			Alarm2Server[Alarm_InPutMismatch].Alarm_Count = 0;
			send_sub_typeA = BACK_ALARM_IOMISMATCH;
			dataSEND(s,send_type);
		}
		else {;}
		
	if((Alarm2Server[Alarm_DisArming].Alarm_Count >= 2) && !Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag)
		{
			send_type = BACK_ALARM;
			Alarm2Server[Alarm_DisArming].Alarm_Count_Flag = false;
			Alarm2Server[Alarm_DisArming].Alarm_Count = 0;
			send_sub_typeA = BACK_ALARM_LOCALDISARMING;
			dataSEND(s,send_type);
		}
		else {;}
			
		if((Alarm2Server[Alarm_Arming].Alarm_Count >= 2) && !Alarm2Server[Alarm_Arming].Alarm_Confirm_Flag)
		{
			send_type = BACK_ALARM;
			Alarm2Server[Alarm_Arming].Alarm_Count_Flag = false;
			Alarm2Server[Alarm_Arming].Alarm_Count = 0;
			send_sub_typeA = BACK_ALARM_LOCALARMING;
			dataSEND(s,send_type);
		}
		else {;}
}

void DATA_PROC_LOOP(void)
{
    switch (Data_Status)
	{
		case DATA_Ready2Read :
			dataRECV_check();
		break;
		case DATA_Validable :
			RECV_DATA_PROCESS();
		break;
		case DATA_FeedBack :
			data2server(0);
			Data_Status = NULL;
		break;
		default ://查询是否接受到数据,数据通道二选一
			if(SYS_Flags.W5500_Read_OK) 
			{
        data_ch = ETH_CH;
				Data_Status = DATA_Ready2Read;
				SYS_Flags.W5500_Read_OK = false;
//				printf("RECV data by w5500......\r\n");
//				printf("RECV data len = %d......\r\n",len);
			}
			else if (SYS_Flags.GPRS_Read_OK)
							{
									data_ch = GPRS_CH;
									SYS_Flags.GPRS_Read_OK = false;
									Data_Status = DATA_Ready2Read;
//									printf("RECV data by GPRS......\r\n");
							}
			else {Data_Status = NULL;}
		break;
	}
}

void PROC_LOOPs(void)
{
	Arming_Proc(IO, NONE);//本地端口布撤防检测及处理	
	alarm2server_loops(0);//报警提交
	KeepAlive_Proc(0);//心跳数据包发送
	DATA_PROC_LOOP();
}

