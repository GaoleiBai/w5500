#include "platform.h"
#include "USR7S3.h"
#include <stdlib.h>

char GPRS_RCV_Buf[GPRS_RCV_Buf_Max];
uint8_t First_Int = 0;
uint8_t GPRS_STATUS = NULL;
uint8_t AT_SET_STATUS = NULL;
uint8_t AT_MODE_STATUS = NULL;
uint8_t COMMOND_EXCUTE_STATUS = NULL;
uint8_t retry_time = 0;
bool test_flag = false;

void GPRS_PIN_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPRS_RST_PIN;
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPRS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPRS_LINK_PIN | GPRS_NET_PIN;
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPRS_PORT, &GPIO_InitStructure);
}

void GPRS_RESET(void)
{
		GPRS_RST;
		delay_ms(500);
		GPRS_RST_RELEASE;
		delay_ms(500);
}

void  GPRS_USART_INIT(long BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	 /* PB10 USART3_tx  */
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* PB11 USART1_rx  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;     
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;      
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;      
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART3, &USART_ClockInitStructure);
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
 USART_Cmd(USART3, ENABLE);
 USART_ClearFlag(USART3,USART_FLAG_TC);
 USART_ClearFlag(USART1,USART_FLAG_TC);

 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART3_IRQHandler(void) 
{ 
	char temp;
  	while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
  	{;} 
	#ifdef DEBUG
	USART_SendData(USART1,temp = USART_ReceiveData(USART3));
	#else
	temp = USART_ReceiveData(USART3);
	#endif
			if((GPRS_STATUS == GPRS_CONNECT_OK) && temp == (char)0xAA)
			{SYS_Flags.GPRS_Read_OK = true;First_Int = 0;}
			else{;}
	GPRS_RCV_Buf[First_Int] = temp;  	
	First_Int++;                			
	if(First_Int > GPRS_RCV_Buf_Max)       	
	{
		First_Int = 0;
	}
	else {;}	
  	USART_ClearFlag(USART3, USART_FLAG_RXNE);
}

void SendStr(char *str)//普通字符串发送
{
	while((*str)!='\0')
	{
		USART_SendData(USART3,*str++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
}

void GPRS_SendAT(char *str)//待\r\n的字符串发送
{
	SendStr(str);
	USART_SendData(USART3,0X0D);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3,0X0A);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void GPRS_SendDATA(uint8_t *data, uint8_t len)//数据发送
{
	do
	{
		USART_SendData(USART3,*data++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
	while(--len);
}

void GPRS_HARDWARE_INIT(void)
{
	GPRS_USART_INIT(115200);
	GPRS_PIN_INIT();
}

uint8_t charfind(char str) //查找单个字符
{
	uint16_t cnt;
	for(cnt = 0; cnt < GPRS_RCV_Buf_Max; cnt++)
	{
		if(GPRS_RCV_Buf[cnt] == str)
		{
			return cnt;
		}
		else{;}
	}
	return NULL;
}

char* memstr(char* full_data, int full_data_len, char* substr)  //查找字符串
{  
    if (full_data == NULL || full_data_len <= 0 || substr == NULL) {  
        return NULL;  
    }  
  
    if (*substr == '\0') {  
        return NULL;  
    }  
  
    int sublen = strlen(substr);  
  
    int i;  
    char* cur = full_data;  
    int last_possible = full_data_len - sublen + 1;  
    for (i = 0; i < last_possible; i++) {  
        if (*cur == *substr) {  
            //assert(full_data_len - i >= sublen);  
            if (memcmp(cur, substr, sublen) == 0) {  
                //found  
                return cur;  
            }  
        }  
        cur++;  
    }  
  
    return NULL;  
}  

uint8_t GPRS_RETURN_CHECK(char *str)//AT返回数据中查找单个字符'a'或者字符串
{
	char tmp ;
	if((*str) != 'a')
	{
		if(memstr(GPRS_RCV_Buf, GPRS_RCV_Buf_Max, str) != NULL)
			{
				printf("GPRS MODULE BACK OK....\r\n");
				return  RETURN_OK;
			}
		else {;}
	}
	else if((*str) == 'a')
	{
		tmp = 'a';
		if(charfind(tmp) != NULL)
					{
						printf("GPRS MODULE BACK OK....\r\n");
						return  RETURN_OK;
					}
				else {;}
	}
	else {;}
	return  RETURN_NOTHING;
}

uint8_t GPRS_RETURN(char *str)//查找成功后清除GPRS_RCV_Buf
{
	uint8_t tmp;
	tmp = GPRS_RETURN_CHECK(str);
	if(tmp == RETURN_OK)
	{
		memset(GPRS_RCV_Buf, 0, GPRS_RCV_Buf_Max);
		First_Int = 0;
	}
	else {;}
	return tmp;
}
/**/
uint8_t COMMOND_RETURN_WAIT(uint8_t time_out, char *str) //AT返回数据查找字符
{
	if(TIM_Count.ticks >	time_out)
			{
				TIM_Count.ticks = 0;
				return COMM_STATUS_TIMEOUT;
			}
			else {
							if(GPRS_RETURN(str) == RETURN_OK)
							{
								TIM_Count.ticks = 0;
								return COMM_STATUS_OK;
							}
							else {;}
					}
	return COMM_STATUS_NONE;
}

/*
	GPRS命令执行函数
	time_out 超时时间
	cmp_str 需要返回的字符
	send_str 发送的命令字符
	send_type 发送类型(SEND_TYPE_A、SEND_TYPE_B)
*/
bool COMMOND_EXCUTE(uint8_t time_out, char *cmp_str, char *send_str, uint8_t send_type)
{
	uint8_t tmp = 0;
	bool flag = false;
	switch (COMMOND_EXCUTE_STATUS)
	{
		case COMMOND_EXCUTE_SEND_OK :
			tmp = COMMOND_RETURN_WAIT(time_out, cmp_str);
			if(tmp == COMM_STATUS_NONE) //无返回，继续检测
				{COMMOND_EXCUTE_STATUS = COMMOND_EXCUTE_SEND_OK;}
				else if(tmp == COMM_STATUS_TIMEOUT) //返回超时，重发
							{
								retry_time++;
								COMMOND_EXCUTE_STATUS = NULL;
							}
							else if(tmp == COMM_STATUS_OK)
										{
											flag = true;
											retry_time = 0;
											COMMOND_EXCUTE_STATUS = NULL;
										}
										else {;}
			if(retry_time > 10)//如果重发次数超过10次不成功，重启GPRS
			{
				GPRS_STATUS = NULL;
				retry_time = 0;
			}
			else {;}
			break;
											
		default :
				if(send_type == SEND_TYPE_A)//根据发送类型选择send函数
				{
					SendStr(send_str);
				}
				else if(send_type == SEND_TYPE_B)
						{
							GPRS_SendAT(send_str);
						}
				else {;}
				COMMOND_EXCUTE_STATUS = COMMOND_EXCUTE_SEND_OK;
			break;
	}
	return flag;
}

void GPRS_AT_SET(void)
{
	switch (AT_MODE_STATUS) 
	{
		case AT_MODE_SEND_xxx :
			if(COMMOND_EXCUTE(1, "a", "+++", SEND_TYPE_A))
			{AT_MODE_STATUS = AT_MODE_SEND_a;}
			else {AT_MODE_STATUS = AT_MODE_SEND_xxx;}
			
			break;
			
		case AT_MODE_SEND_a :
			if(COMMOND_EXCUTE(1, "+ok", "a", SEND_TYPE_A))
			{AT_MODE_STATUS = AT_MODE_SUCCESS;}
			else {AT_MODE_STATUS = AT_MODE_SEND_a;}
			break;
									
		case AT_MODE_SUCCESS :
			GPRS_AT_INIT_COMMOND();
			break;
		
		default :
				AT_MODE_STATUS = AT_MODE_SEND_xxx;
			break;
	}
}

/*整数转换为字符串*/  
char *IntToStr(int num, char str[])  
{  
    int i = 0, j = 0;  
    char temp[10];  //IP、端口等字符串最多不超过5个字符
    while(num)  
    {  
        temp[i] = num % 10 + '0';   //转换为字符
        num = num / 10;  
        i++;  
    }  
    temp[i] =	'\0';    //结束符
      
    i = i - 1;     //temp有效字符处
    while(i >= 0)  
    {  
        str[j] = temp[i];  
        i--;  
        j++;  
    }  
    str[j] = '\0';   //字符串结尾
    return str;  
}  

char* GPRS_GET_PARA_STRING(void)//将服务器IP、端口号转换成字符串
{
	static char str[40] = {'A','T','+','S','O','C','K','A','=','"','T','C','P','"',',','"'};
	char tmp[5];
	uint8_t cnt = 16, count = 0;

	count = 0;
	IntToStr(DstIP[0],tmp);
	do{
			str[cnt++] = tmp[count];
		}
	while (tmp[count++] != '\0');
	str[cnt - 1] = '.';
		
	count = 0;
	IntToStr(DstIP[1],tmp);
	do{
			str[cnt++] = tmp[count];
		}
	while (tmp[count++] != '\0');
	str[cnt - 1] = '.';
		
	count = 0;
	IntToStr(DstIP[2],tmp);
	do{
			str[cnt++] = tmp[count];
		}
	while (tmp[count++] != '\0');
	str[cnt - 1] = '.';
		
	count = 0;
	IntToStr(DstIP[3],tmp);
	do{
			str[cnt++] = tmp[count];
		}
	while (tmp[count++] != '\0');
	str[cnt - 1] = '"';
	str[cnt++] = ',';
		
	count = 0;
	IntToStr(DstPort,tmp);
	do{
			str[cnt++] = tmp[count];
		}
	while (tmp[count++] != '\0');
	str[cnt - 1] = '\0';
	
	return str;
}

void GPRS_AT_INIT_COMMOND(void)
{
	switch (AT_SET_STATUS)
	{
		case AT_SET_ATE://关回显
			if(COMMOND_EXCUTE(1, "OK", "ATE1", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_WKMOD;}
			else {AT_SET_STATUS = AT_SET_ATE;}
			break;
						
		case AT_SET_WKMOD ://设置为NET模式
			if(COMMOND_EXCUTE(1, "OK", "AT+WKMOD=\"NET\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_SOCKETAEN;}
			else {AT_SET_STATUS = AT_SET_WKMOD;}
			break;
									
		case AT_SET_SOCKETAEN ://SOCKETA使能
			if(COMMOND_EXCUTE(1, "OK", "AT+SOCKAEN=\"on\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_SOCKETA;}
			else {AT_SET_STATUS = AT_SET_SOCKETAEN;}
			break;
									
		case AT_SET_SOCKETA ://配置SOCKETA
			if(COMMOND_EXCUTE(1, "OK", GPRS_GET_PARA_STRING(), SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_HEART;}
			else {AT_SET_STATUS = AT_SET_SOCKETA;}
			break;
									
			case AT_SET_HEART ://关闭心跳，自行设计心跳包
			if(COMMOND_EXCUTE(1, "OK", "AT+HEARTEN=\"off\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_UATEN;}
			else {AT_SET_STATUS = AT_SET_HEART;}
			break;
			
			case AT_SET_UATEN ://打开串口AT指令
			if(COMMOND_EXCUTE(1, "OK", "AT+UATEN=\"on\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_CMDPW;}
			else {AT_SET_STATUS = AT_SET_UATEN;}
			break;
			
			case AT_SET_CMDPW ://设置串口AT指令密码
			if(COMMOND_EXCUTE(1, "OK", "AT+CMDPW=\"fuck\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_CALEN;}
			else {AT_SET_STATUS = AT_SET_CMDPW;}
			break;
			
			case AT_SET_CALEN ://关闭语音功能
			if(COMMOND_EXCUTE(1, "OK", "AT+CALEN=\"off\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_NATEN;}
			else {AT_SET_STATUS = AT_SET_CALEN;}
			break;
			
			case AT_SET_NATEN ://关闭网络AT功能
			if(COMMOND_EXCUTE(1, "OK", "AT+NATEN=\"off\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_CACHEN;}
			else {AT_SET_STATUS = AT_SET_NATEN;}
			break;
			
			case AT_SET_CACHEN ://关闭数据缓存
			if(COMMOND_EXCUTE(1, "OK", "AT+CACHEN=\"off\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_SOCKETBDISABLE;}
			else {AT_SET_STATUS = AT_SET_CACHEN;}
			break;
			
			case AT_SET_SOCKETBDISABLE ://SOCKETB禁用
			if(COMMOND_EXCUTE(1, "OK", "AT+SOCKBEN=\"off\"", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_SAVE;}
			else {AT_SET_STATUS = AT_SET_SOCKETBDISABLE;}
			break;
									
		case AT_SET_SAVE ://save&reboot
			if(COMMOND_EXCUTE(1, "OK", "AT+S", SEND_TYPE_B))
			{AT_SET_STATUS = AT_SET_RESET;}
			else {AT_SET_STATUS = AT_SET_SAVE;}
			break;
			
		case AT_SET_RESET ://RESET OK
			if(COMMOND_EXCUTE(30, "USR-GM3 V2.1", "", SEND_TYPE_C))
			{AT_SET_STATUS = AT_SET_OK;printf("gprs reboot ok...\r\n");}
			else {AT_SET_STATUS = AT_SET_RESET;}
			break;	
									
		case AT_SET_OK :
			GPRS_STATUS = GPRS_SET_OK;
			TIM_Count.ticks = 0;
			break;
		
		default :
			AT_SET_STATUS = AT_SET_ATE;
			break;
	}
}

uint8_t GPRS_SEND_STATUS = NULL;

void GPRS_CHECK_LOOP(void)
{
	switch (GPRS_STATUS)
	{
		case GPRS_CHECK :
			if(memstr(GPRS_RCV_Buf, GPRS_RCV_Buf_Max, "USR-GM3 V2.1") != NULL)
			{
				GPRS_STATUS = GPRS_RESET_OK;
				memset(GPRS_RCV_Buf, 0, GPRS_RCV_Buf_Max);
			}
			else{GPRS_STATUS = GPRS_CHECK;}
			if(TIM_Count.ticks > 10) //30s无欢迎词，GPRS模块错误
			{
				GPRS_STATUS = GPRS_HardWare_ERROR;
				printf("GPRS_HardWare_ERROR...\r\n");
				TIM_Count.ticks = 0;
			}
			else {;}
			break;
		
		case GPRS_RESET_OK :
			GPRS_AT_SET();
			break;
		
		case GPRS_SET_OK :
			if( (GPRS_NET_STATUS) & (GPRS_LINK_STATUS) ) //检测GPRS及SOCKET是否正常连接
			{
				TIM_Count.ticks = 0;
				GPRS_STATUS = GPRS_CONNECT_OK;
			}
			else {GPRS_STATUS = GPRS_SET_OK;}
			if(TIM_Count.ticks > 60) //120s无法建立正常连接，重新初始化GPRS
			{GPRS_STATUS = NULL;}
			break;
			
		case GPRS_CONNECT_OK :
			if( !((GPRS_NET_STATUS) & (GPRS_LINK_STATUS)) )
			{GPRS_STATUS = GPRS_SET_OK;}
			else {GPRS_STATUS = GPRS_CONNECT_OK;}
			break;
			
			case GPRS_HardWare_ERROR :
//				if(memstr(GPRS_RCV_Buf, GPRS_RCV_Buf_Max, "USR-GM3 V2.1") != NULL)//检测GPRS模块是否工作
//				{
//					GPRS_STATUS = NULL;
//					memset(GPRS_RCV_Buf, 0, GPRS_RCV_Buf_Max);
//				}
//				else{GPRS_STATUS = GPRS_HardWare_ERROR;}
			GPRS_STATUS = NULL;
				break;
			
		default :
			printf("gprs hardware reset...\r\n");
			GPRS_RESET();
			AT_SET_STATUS = NULL;
			AT_MODE_STATUS = NULL;
			GPRS_SEND_STATUS = NULL;
			GPRS_STATUS = GPRS_CHECK;
			break;
	}
}

void GPRS_SEND_LOOP(void)
{
	switch (GPRS_SEND_STATUS)
	{
		case GPRS_SEND_CHECK :
			if(COMMOND_EXCUTE(1, "+SOCKALK:connected", "fuck#AT+SOCKALK?", SEND_TYPE_B))//软件查询模块SOCKET连接是否正常
			{GPRS_SEND_STATUS = GPRS_SEND_PREP;}
			else {GPRS_SEND_STATUS = GPRS_SEND_CHECK;}
			break;
			
		case GPRS_SEND_PREP :
			if(SYS_Flags.GPRS_Send_Flag)
			{
				GPRS_SEND_STATUS = NULL;
			}
			else {
							GPRS_SEND_STATUS = GPRS_SEND_PREP;
							SYS_Flags.GPRS_Send_Flag = false;
						}
			break;
				
		default :
			if( (GPRS_STATUS == GPRS_CONNECT_OK) && !SYS_Flags.GPRS_Send_Flag )
			{
				GPRS_SEND_STATUS = GPRS_SEND_CHECK;
			}
			else {GPRS_SEND_STATUS = NULL;}
			break;
	}
}

void GPRS_LOOPs(void)
{
	GPRS_CHECK_LOOP();
	GPRS_SEND_LOOP();
}

