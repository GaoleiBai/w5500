/* �������ݸ�ʽ
֡ͷ	 ����	   ����  ����     У���
1Byte	 2Byte	1Byte	 NBytes 	 2Byte
*/

#include "dataproc.h"
uint8_t data_buff[32];
uint16_t Input_Status;
uint16_t Input_Status_Confirm;
uint8_t Output_Status;
uint16_t Input_Status_ARMING;
DATATYPE recv_type = NONE;
DATATYPE send_type = NONE;
ACTIONTYPE actiontype = NONE;
ACTION action = NONE;
unsigned int len;
ALARM Alarm2Server[3] = {
													{.Alarm_Type = Alarm_Arming, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
													{.Alarm_Type = Alarm_DisArming, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
													{.Alarm_Type = Alarm_InPutMismatch, .Alarm_Count = 0, .Alarm_Count_Flag = false, .Alarm_Confirm_Flag = false},
												};

uint8_t dataECC(uint16_t ECCAdd,uint16_t len) //�����������У��
{
	uint16_t ECCRes = 0,count = 0;

		for(count = 0;count < len-2;count++)
		{
			ECCRes += data_buff[count+3];
		}
	if(ECCRes == ECCAdd)
	{return GotRightData;}
	else {return GotWrongData;}
}

uint8_t dataRECV(SOCKET s) //���������ж�
{
	recv_type = NONE;
	send_type = NONE;
	uint16_t count = 0,fh_offset = 0,\
						data_len = 0,ECCAdd = 0,\
						fd = 0;
	len = recv(s,gDATABUF,DATA_BUF_SIZE);
	memset(data_buff,0,32);
	for(count = 0;count < len;count++)
	{
		if(gDATABUF[count] == FrameHeader)
		{
			fh_offset = count;
			goto datachecked;
		}
	}
	send_type =  WRONGDATA;
	return GotNoData;
	
datachecked:
	for(count = 0;count < len - fh_offset;count++)//��ȡ���ݰ�
	{
		data_buff[count] = gDATABUF[fh_offset+count];
	}
	data_len = data_buff[1]+data_buff[2];//��Ч����+����λ ����
	if(data_len < 3)
	{	
		send_type =  WRONGDATA;
		return GotWrongDataLenth;
	}
	recv_type = data_buff[3];
	ECCAdd = (((uint16_t)data_buff[len - fh_offset - 2])<<8) | ((uint16_t)data_buff[len - fh_offset - 1]);//ȥ��֡ͷ-1
	fd = dataECC(ECCAdd,data_len);
	if( fd == GotRightData)
	{
		dataPROC(recv_type);
		send_type = recv_type;
		return fd;
	}
	else {
				send_type =  WRONGDATA;
				return fd;
				}
}

uint8_t Data2EEProm(uint8_t _ucRegAddr, uint8_t _ucRegData) //�����ݴ���Ĵ���
{
	if(Input_Status_ARMING == _ucRegData)//�ж��Ƿ���Ҫ�洢
	{;}
		else 
		{
			EEPROM_WriteByte(_ucRegAddr, _ucRegData);
			delay_ms(5);
			if(EEPROM_ReadByte(_ucRegAddr) == _ucRegData)
			{
				printf("save to eeprom OK.....\r\n");
			}
			else
			{
				printf("save to eeprom FAILED....\r\n");
				send_type = WRONGDATA;
				return SAVEFAILED;
			}
		}
	return DATAPROC_SUCCESS;
}
/*
scantype:
	SCAN_IN ɨ������˿�
	SCAN_OUT ɨ������˿�
	SCAN_ALL ȫ���˿�ɨ��
scancause��
	SCAN_LOOPS ��ɨ�裬������
	SCAN_ARMING ɨ���ұ��棬��Ϊ������������
*/
void SCAN_PORT(uint8_t scantype, uint8_t scancause) //�˿�ɨ��
{
	uint8_t count;
	uint16_t temp;
	__disable_irq() ;
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
				Data2EEProm(IN_ARMING_reg_add, (uint8_t)(temp>>8));
				Data2EEProm(IN_ARMING_reg_add + 1, (uint8_t)temp);
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
				Data2EEProm(IN_ARMING_reg_add, (uint8_t)(temp>>8));
				Data2EEProm(IN_ARMING_reg_add + 1, (uint8_t)temp);
			}
			printf("Output_Status: 0x%02X,Input_Status: 0x%04X\r\n",Output_Status,Input_Status);
			break;	
		default :
			break;
	}
	__enable_irq() ;
}

uint8_t SET_OUT(void) //���ö˿����
{
	uint8_t count;
	__disable_irq() ;
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
	if(Output_Status_EEPROM != action)//������µ�ָ��״̬��������EEPROM
		{	
			Output_Status_EEPROM = action;
			Data2EEProm(OUT_reg_add, Output_Status_EEPROM);
		}
	for(count = 0; count < OutputPinNum; count ++)
		{
			if(GPIO_ReadOutputDataBit(OUT[count].Port, OUT[count].Port_Pin) != OUT[count].Pin_Value)//���ܽ����״̬
			{
				send_type = WRONGDATA;
				__enable_irq() ;
				return SetPinsStatusFailed;
			}
			else{;}
		}
	__enable_irq() ;
	return DATAPROC_SUCCESS;
}
/*
OPCode:
	Local_DisArming ���ض˿ڳ���
	Local_Arming ���ض˿ڲ���
	NET_DisArming ���糷��
	NET_Arming ���粼��
*/
uint8_t Dis_Arming_OP(uint8_t OPCode) //����������
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
		
DISARMING: //ͨ�ó�������
					//SCAN_PORT(SCAN_ALL, SCAN_LOOPS);//��disarmingǰ�Ķ˿�״̬ɨ�貢����
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

ARMING:  //ͨ�ò�������
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
					SCAN_PORT(SCAN_ALL, SCAN_ARMING);
					return Arming_Status;
}

uint8_t Arming_Proc(uint8_t type) //��������������Dis_Arming_OP
{
	switch(type)
	{
	case IO :
		if(SYS_Flags.SYS_Interrupt)//100ms��鲼�����˿��ź�
		{
			ARMING_PORT[0].Pin_Value = GPIO_ReadInputDataBit(ARMING_PORT[0].Port, ARMING_PORT[0].Port_Pin);
			ARMING_PORT[1].Pin_Value = GPIO_ReadInputDataBit(ARMING_PORT[1].Port, ARMING_PORT[1].Port_Pin);
			if((!ARMING_PORT[0].Pin_Value) && (Arming_Status == Arming))
			{
				SYS_Flags.Count_Flag = true;
				if(TIM_Count.Arming_Count >= 5)// 5s�󳷷����򿪼̵���CH1&CH2
				{
					Dis_Arming_OP(Local_DisArming);
					Erro_Flag = LocalDisArming;
					send_type = ALARM2SERVER;
					dataSEND(0,send_type);
					Arming_Status = DisArming;
					Alarm2Server[Alarm_DisArming].Alarm_Count_Flag = true;
					Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag = false;
				}
			}
			else if((!ARMING_PORT[1].Pin_Value) && (Arming_Status == DisArming))
			{
				SYS_Flags.Count_Flag = true;
				if(TIM_Count.Arming_Count >= 10)//30s�󲼷����ָ��̵���״̬
				{
					Dis_Arming_OP(Local_Arming);
					Erro_Flag = LocalArming;
					send_type = ALARM2SERVER;
					dataSEND(0,send_type);
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
		}
		break;
				
	case NET:
		if( !SYS_Flags.NET_Arming_Flag)
		{
			Dis_Arming_OP(NET_Arming);
			SYS_Flags.NET_Arming_Flag = true;
			Arming_Status = Arming;
			return Arming_Status;
		}
		else if(SYS_Flags.NET_Arming_Flag)
					{
						Dis_Arming_OP(NET_DisArming);
						SYS_Flags.NET_Arming_Flag = false;
						Arming_Status = DisArming;
						return Arming_Status;
					}
		else {;}
		break;
					
	default :
		break;
	}
	return Arming_Status;
}

uint8_t HardWareAction_COMMAND(ACTIONTYPE actiontype, ACTION action) //Ӳ����������������
{
	switch(actiontype)
	{
		case ReadPortInput : 
			SCAN_PORT(SCAN_ALL, SCAN_LOOPS);
			break;
					
		case WritePortOutput :
				if( SET_OUT() == SAVEFAILED)
				{return SAVEFAILED;}
			break;
		case Arming :
			SYS_Flags.NET_Arming_Flag = false;
			Arming_Proc(NET);
			break;
		case DisArming :
			SYS_Flags.NET_Arming_Flag = true;
			Arming_Proc(NET);
			break;
		default : 
			break;
	}
	return DATAPROC_SUCCESS;
}

uint32_t Flash_Read(uint32_t iAddress)
{

		//printf("0x%02X\r\n",*( uint32_t*) iAddress);

	return *( uint32_t*) iAddress;
} 

uint8_t dataPROC(DATATYPE type) //�������ݺ���ɹ涨����
{
	uint8_t FB;
	actiontype = data_buff[4];
		switch(type)
		{
			case COMMAND : 
//					printf("Got a COMMAND.....\r\n");
					action = data_buff[5];
					FB = HardWareAction_COMMAND(actiontype,action);
					if(FB == SetPinsStatusFailed)
					{
						send_type = WRONGDATA;
						return SetPinsStatusFailed;
					}
					if(FB == SAVEFAILED)
					{
						send_type = WRONGDATA;
						return SAVEFAILED;
					}
				break;
			case DATA :
//				printf("Got a data.....\r\n");
				break;
			case UPGRADE :
//				printf("Got a update.....\r\n");
				FLASH_Unlock();
				FLASH_ErasePage(UPDATE_FLAG_Address);
				FLASH_ProgramWord(UPDATE_FLAG_Address,GO_UPDATE_FLAG); 
				FLASH_Lock();
				break;
			case KEEPALIVE :
//				printf("Got a keepalive.....\r\n");
				break;
			case ALARMCONFIRM :
//				printf("Got a ALARMCONFIRM.....\r\n");
				Input_Status_Confirm = Input_Status;
				SYS_Flags.Alarm_Flag = false;
				Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag = true;
				Alarm2Server[Alarm_Arming].Alarm_Confirm_Flag = true;
				Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag = true;
				break;
			default :
				break;
		}
	return DATAPROC_SUCCESS;
}

uint16_t SumProduct(uint16_t len) //���ɷ������У������
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

void dataSEND(SOCKET s,DATATYPE type) //��������(�Ƿ���Ҫʹ�ýṹ�����ķ�ʽ��)
{
	uint16_t sum = 0;
	memset(data_buff,0,32);
	switch(type)
		{
			case COMMAND : 
//				printf("Send a COMMAND.....\r\n");
				if(actiontype == ReadPortInput)
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
					send(s,data_buff,10);
				}
				if(actiontype == WritePortOutput)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x03;
					data_buff[3] = type;
					sum = SumProduct(data_buff[2]-2);
					data_buff[4] = (uint8_t)(sum >> 8);
					data_buff[5] = (uint8_t)(sum & 0xFF);
					send(s,data_buff,6);
				}
				if((actiontype == Arming) || (actiontype == DisArming))
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x03;
					data_buff[3] = type;
					sum = SumProduct(data_buff[2]-2);
					data_buff[4] = (uint8_t)(sum >> 8);
					data_buff[5] = (uint8_t)(sum & 0xFF);
					send(s,data_buff,6);
				}
				break;
			case DATA :
//				printf("Send a data.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x03;
				data_buff[3] = type;
				sum = SumProduct(data_buff[2]-2);
				data_buff[4] = (uint8_t)(sum >> 8);
				data_buff[5] = (uint8_t)(sum & 0xFF);
				send(s,data_buff,6);
				break;
			case UPGRADE :
//				printf("Send a UPGRADE.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x03;
				data_buff[3] = type;
				sum = SumProduct(data_buff[2]-2);
				data_buff[4] = (uint8_t)(sum >> 8);
				data_buff[5] = (uint8_t)(sum & 0xFF);
				send(s,data_buff,6);
				break;
			case KEEPALIVE :
//				printf("Send a keepalive.....\r\n");
				data_buff[0] = FrameHeader;
				data_buff[1] = 0x00;
				data_buff[2] = 0x0F;
				data_buff[3] = type;
				data_buff[4] = gWIZNETINFO.mac[0];
				data_buff[5] = gWIZNETINFO.mac[1];
				data_buff[6] = gWIZNETINFO.mac[2];
				data_buff[7] = gWIZNETINFO.mac[3];
				data_buff[8] = gWIZNETINFO.mac[4];
				data_buff[9] = gWIZNETINFO.mac[5];
				data_buff[10] = gWIZNETINFO.ip[0];
				data_buff[11] = gWIZNETINFO.ip[1];
				data_buff[12] = gWIZNETINFO.ip[2];
				data_buff[13] = gWIZNETINFO.ip[3];
				sum = SumProduct(data_buff[2]-2);
				data_buff[14] = (uint8_t)(sum >> 8);
				data_buff[15] = (uint8_t)(sum & 0xFF);
				send(s,data_buff,16);
				break;
			case WRONGDATA :
//				printf("Send a dataErro.....\r\n");
				if(Erro_Flag != GotRightData)
				{
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x04;
					data_buff[3] = type;
					data_buff[4] = Erro_Flag;
					sum = SumProduct(data_buff[2]-2);
					data_buff[5] = (uint8_t)(sum >> 8);
					data_buff[6] = (uint8_t)(sum & 0xFF);
					send(s,data_buff,7);
				}
				break;
			case ALARM2SERVER :
//				printf("Send a ALARM2SERVER.....\r\n");
					data_buff[0] = FrameHeader;
					data_buff[1] = 0x00;
					data_buff[2] = 0x05;
					data_buff[3] = type;
					data_buff[4] = Erro_Flag;
					if(Erro_Flag == IOMISMATCH)
					{
						data_buff[5] = (uint8_t)(Input_Status >> 8);
						data_buff[6] = (uint8_t)Input_Status;
					}
					else if((Erro_Flag == LocalDisArming) || (Erro_Flag == LocalArming))
						{
							data_buff[5] = 0x00;
							data_buff[6] = 0x00;
						}
					else {;}
					sum = SumProduct(data_buff[2]-2);
					data_buff[7] = (uint8_t)(sum >> 8);
					data_buff[8] = (uint8_t)(sum & 0xFF);
					send(s,data_buff,9);
				break;
			default :
				break;
		}
	Erro_Flag = NONE;
	send_type = NONE;
}

void alarm2server_loops(SOCKET s) //δ��
{
	if(Arming_Status == Arming)//�����ڼ�˿�ɨ�輰�ϱ�,10s��ȷ�ϼ����ϱ�
		{		
			SCAN_PORT(SCAN_IN, SCAN_LOOPS);
			if(Input_Status != Input_Status_Confirm)
				{ 
					Alarm2Server[Alarm_InPutMismatch].Alarm_Count_Flag = true;
					Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag = false;
				}
		}
		else {;}
		
	if((Alarm2Server[Alarm_InPutMismatch].Alarm_Count >= 5) && !Alarm2Server[Alarm_InPutMismatch].Alarm_Confirm_Flag)
		{
			send_type = ALARM2SERVER;
			SYS_Flags.W5500_Send_OK = false;
			Alarm2Server[Alarm_InPutMismatch].Alarm_Count_Flag = false;
			Alarm2Server[Alarm_InPutMismatch].Alarm_Count = 0;
			Erro_Flag = IOMISMATCH;
			dataSEND(s,send_type);
			send_type = NONE;
		}
		else {;}
		
	if((Alarm2Server[Alarm_DisArming].Alarm_Count >= 5) && !Alarm2Server[Alarm_DisArming].Alarm_Confirm_Flag)
		{
			send_type = ALARM2SERVER;
			SYS_Flags.W5500_Send_OK = false;
			Alarm2Server[Alarm_DisArming].Alarm_Count = 0;
			Erro_Flag = LocalDisArming;
			dataSEND(s,send_type);
			send_type = NONE;
		}
		else {;}
			
		if((Alarm2Server[Alarm_Arming].Alarm_Count >= 5) && !Alarm2Server[Alarm_Arming].Alarm_Confirm_Flag)
		{
			send_type = ALARM2SERVER;
			SYS_Flags.W5500_Send_OK = false;
			Alarm2Server[Alarm_Arming].Alarm_Count = 0;
			Erro_Flag = LocalArming;
			dataSEND(s,send_type);
			send_type = NONE;
		}
		else {;}
}

void data2server_loops(SOCKET s)
{
	alarm2server_loops(s);
	//����ʱ��ظ���������ֵ
	if(SYS_Flags.W5500_Read_OK)
		{
			SYS_Flags.W5500_Read_OK = false;
			SYS_Flags.Got_Data_Flag = true;
				memset(gDATABUF,0,DATA_BUF_SIZE);
				 Erro_Flag = dataRECV(s);
					switch (Erro_Flag)
					{
						case GotRightData:
							printf("GotRightData\r\n");
							break;
						case GotWrongData:
							printf("GotWrongData\r\n");
							break;
						case GotNoData:
							printf("GotNoData\r\n");
							break;
						case GotWrongDataLenth:
							printf("GotWrongDataLenth\r\n");
							break;
						case SetPinsStatusFailed:
							printf("SetPinsStatusFailed\r\n");
							break;
						case DATAPROC_SUCCESS:
							printf("DATAPROC_SUCCESS\r\n");
							break;
						case SAVEFAILED:
							printf("SAVEFAILED\r\n");
							break;
					}
		}
		
//		if(send_type)
//		{
//			SYS_Flags.W5500_Send_OK = false;
//			dataSEND(s,send_type);
//			send_type = NONE;
////							if(Flash_Read(UPDATE_FLAG_Address) == GO_UPDATE_FLAG) //����������־λ������
////				{
////					SYSRESET();
////				}
//		}
		
		if(SYS_Flags.KeepAlive_Flag)//�������ݰ�����
		{
			SYS_Flags.KeepAlive_Flag = false;
			SYS_Flags.W5500_Send_OK = false;
			dataSEND(s,KEEPALIVE);
			printf("mS:%d,S:%d,M:%d,H:%d\r\n",TIM_Count.mSec_Count,TIM_Count.Sec_Count,TIM_Count.Min_Count,TIM_Count.Hou_Count);
		}
}
