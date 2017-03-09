/**
  ******************************************************************************
  * @file    platform.c
  * $Author: liaojingjing$
  * $Revision: 17 $
  * $Date:: 2016-11-03 11:16:48 +0800 #$
  * @brief  
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "platform.h"
INITREG  W5500_int_reg;
wiz_NetInfo gWIZNETINFO = { .mac = {0x0c,0x29,0xab,0x00, 0x00, 0x00},
                            .ip = {192, 168, 1, 123},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {0,0,0,0},
                            .dhcp = NETINFO_STATIC };
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t Arming_Status;
bool format_flag = false;										

uint8_t sys_upgradeflag = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;  //应用程序地址指针
#define  ApplicationAddress0     0x08000000    //应用程序起始地址 

#define  ApplicationAddress1     0x08007000    //应用程序起始地址 
														
/************************************
 * @ brief Call back for ip Conflict
 ************************************/
void my_ip_conflict(void)
{
	printf("CONFLICT IP from DHCP\r\n");
	//halt or reset or any...
	while(1); // this example is halt.
}
/**
  * @brief  串口打印输出
  * @param  None
  * @retval None
  */

void W5500_interrupt_config(SOCKET s)
{
	setIMR(IM_IR7 | IM_IR6);
	setSIMR(0x01);//s0 interrupt
	setSn_IMR(s,  Sn_IR_TIMEOUT | Sn_IR_RECV | Sn_IR_SENDOK | Sn_IR_DISCON | Sn_IR_CON);//socket interrupt all
	setSn_MSSR(s,1460);
		setRTR(0x07d0);
		setRCR(8);
}

void socket_start(SOCKET s)
{
		int ret;
		uint8_t DstIP[4]={192,168,1,111};//{122,228,19,57};
socket_reconnect:
	{
		ret = socket(s,Sn_MR_TCP,5000,0x00);
		if(ret != 0){
			printf("%d:Socket Error\r\n",s);
			while(1);
		}else{
			printf("%d:Opened\r\n",s);
		}
		//连接TCP服务器
		ret = connect(s,DstIP,6000);//13435);
		if(ret != SOCK_OK){
			printf("%d:Socket Connect Error\r\n",s);
			goto socket_reconnect;
		}	
	}
	
}

void W5500_function_register(void )
{
		//Host dependent peripheral initialized
	// First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP 
	/* Critical section callback */
	reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);	//注册临界区函数
	/* Chip selection call back */
#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);//注册SPI片选信号函数
#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
#else
   #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
      #error "Unknown _WIZCHIP_IO_MODE_"
   #else
      reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
   #endif
#endif
	/* SPI Read & Write callback function */
	reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);	//注册读写函数
}

void W5500_NETWORK_init(SOCKET s)
{
    // must be set the default mac before DHCP started.
	setSHAR(gWIZNETINFO.mac);
	DHCP_init(SOCK_DHCP, gDATABUF);
	// if you want defiffent action instead defalut ip assign,update, conflict,
	// if cbfunc == 0, act as default.
	reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
	while(DHCP_run() != DHCP_IP_LEASED);
	socket_start(s);
	
}

void W5500_init(SOCKET s)
{
	W5500_function_register();
	W5500_RESET(s);
//	W5500_interrupt_config(s);
	W5500_NETWORK_init(s);
	
}
uint32_t APP_Address = 0x08007000;
void SCAN_LOOP_TASK(SOCKET s)
{
	uint16_t len = 0;
	uint32_t app_data_temp,count;
	if(SYS_Flags.SYS_Interrupt)//100ms检查W5500寄存器
	{

		SYS_Flags.SYS_Interrupt = false;
		W5500_int_reg.IR_REG = getIR();
		W5500_int_reg.SIR_REG = getSIR();
		W5500_int_reg.SnSIR_REG = getSn_IR(0);
		W5500_int_reg.SnSR_REG = getSn_SR(0);	
		W5500_int_reg.PHY_LNK_REG = getPHYCFGR();
//		printf("GOT: IR:0x%02X,SIR:0x%02X,SnSIR:0x%02X,SnCR:0x%02X,PHYCFGR:0x%02X\r\n",W5500_int_reg.IR_REG,W5500_int_reg.SIR_REG,W5500_int_reg.SnSIR_REG,W5500_int_reg.SnSR_REG,W5500_int_reg.PHY_LNK_REG);
		setSIR(W5500_int_reg.SIR_REG);
		setSn_IR(s,  W5500_int_reg.SnSIR_REG&0x0F);//send函数清除0x10;
		setPHYCFGR(W5500_int_reg.PHY_LNK_REG | 0x01);
		if((W5500_int_reg.PHY_LNK_REG & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_OFF)//检查link情况
		{
			SYS_Flags.W5500_Dislink_Status = true;
		}
		if(SYS_Flags.W5500_Dislink_Status)
		{	
			if((getPHYCFGR() & PHYCFGR_LNK_STATUS) != 0)
			{
				DHCP_init(SOCK_DHCP, gDATABUF);
				while(DHCP_run() != DHCP_IP_LEASED){};
				close(s);
				socket_start(s);
				SYS_Flags.W5500_Dislink_Status = false;
			}
		}
		
		if(((W5500_int_reg.IR_REG & 0x80) == 0x80) || ((W5500_int_reg.IR_REG & 0x20) == 0x20))//ip冲突 或 目标不可达
		{
			close(s);
			W5500_init(s);//重启网络
		}
		
		if(W5500_int_reg.SIR_REG == 0x01)//s0中断到来
		{
			if(((W5500_int_reg.SnSIR_REG & 0x02) == 0x02)|| ((W5500_int_reg.SnSIR_REG) == 0x08)) //socket 断开或连接超时
			{
				close(s);
				socket_start(s);//重启网络socket
			}
			if((W5500_int_reg.SnSIR_REG & 0x04) == 0x04)
				{SYS_Flags.W5500_Read_OK = true;}
			if((W5500_int_reg.SnSIR_REG & 0x10) == 0x10)
				{SYS_Flags.W5500_Send_OK = true;}
		}	
	}
	//固件升级
	if(SYS_Flags.W5500_Read_OK)
		{
			SYS_Flags.W5500_Read_OK = false;
			SYS_Flags.Got_Data_Flag = true;
			memset(gDATABUF,0,DATA_BUF_SIZE);
			len = recv(s,gDATABUF,DATA_BUF_SIZE);;
			FLASH_Unlock();
			FLASH_ClearFlag( FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR);
							if(APP_Address % 1024 == 0)//1页1k，先擦后写
				{
					FLASH_ErasePage(APP_Address);
				}
			for(count = 0; count < len; count += 4)
			{
				app_data_temp = (uint32_t)gDATABUF[0 + count];
				app_data_temp |= ((uint32_t)gDATABUF[1 + count]) << 8;
				app_data_temp |= ((uint32_t)gDATABUF[2 + count]) << 16;
				app_data_temp |= ((uint32_t)gDATABUF[3 + count]) << 24;
				while(FLASH_ProgramWord(APP_Address + count,app_data_temp) != FLASH_COMPLETE); 
				if(*( uint32_t*)(APP_Address + count) != app_data_temp) //写入数据校验
				{
					printf("CRC Error......\r\n");
				}
				else {printf(".");}
			}
			FLASH_Lock();
			APP_Address += len;
			if(len < DATA_BUF_SIZE)
			{
				printf("\r\nApp data = %d Bytes...\r\n",APP_Address -= ApplicationAddress1);
				FLASH_Unlock();
				FLASH_ClearFlag( FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR); 
				do
				{
					while(FLASH_ErasePage(UPDATE_FLAG_Address) != FLASH_COMPLETE);
					while(FLASH_ProgramWord(UPDATE_FLAG_Address,DONE_UPDATE_FLAG)== FLASH_COMPLETE);
				}
				while(*( uint32_t*)(UPDATE_FLAG_Address) != DONE_UPDATE_FLAG); //写入数据校验
				FLASH_Lock();
				printf("0x%02X\r\n",*( uint32_t*) UPDATE_FLAG_Address);
				SYSRESET();
			}
		}
}



void LOOPs(SOCKET s)
{
	while(1)
	{
		SCAN_LOOP_TASK(s);
		WDOG_FeedBack_TASK();
		if(sys_upgradeflag == 0x01)
		{
			jump();
		}
	}
}

void go_jump(void)
{
	uint32_t ApplicationAddress=0;

		ApplicationAddress = ApplicationAddress1;
							USART_DeInit(USART1);
			TIM_DeInit( TIM2);
		RCC_DeInit();
		__disable_irq() ;
		#ifdef  VECT_TAB_RAM  
    // Set the Vector Table base location at 0x20000000 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
    // Set the Vector Table base location at 0x08000000 
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x7000);   
    #endif

	if (((*(volatile uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
	{
		JumpAddress = *(volatile uint32_t*)(ApplicationAddress + 4);
		Jump_To_Application = (pFunction)JumpAddress;
		
		__set_MSP(*(volatile uint32_t*)ApplicationAddress);    //初始化用户程序的堆栈指针 
		
		Jump_To_Application();
	}
}

void jump(void)
{
	uint32_t ApplicationAddress=0;
	if(0x01==sys_upgradeflag)
	{
		ApplicationAddress = ApplicationAddress1;
		printf("Execute user Program\r\n");
							USART_DeInit(USART1);
			TIM_DeInit( TIM2);
		RCC_DeInit();
		__disable_irq() ;
		#ifdef  VECT_TAB_RAM  
    // Set the Vector Table base location at 0x20000000 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
    // Set the Vector Table base location at 0x08000000 
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x7000);   
    #endif
	}
	else
	{
		return ;
	}
	if (((*(volatile uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
	{
		JumpAddress = *(volatile uint32_t*)(ApplicationAddress + 4);
		Jump_To_Application = (pFunction)JumpAddress;
		
		__set_MSP(*(volatile uint32_t*)ApplicationAddress);    //初始化用户程序的堆栈指针 
		
		Jump_To_Application();
	}
}
/*******************************************************
 * @ brief Call back for ip assing & ip update from DHCP
 *******************************************************/
void my_ip_assign(void)
{
   getIPfromDHCP(gWIZNETINFO.ip);
   getGWfromDHCP(gWIZNETINFO.gw);
   getSNfromDHCP(gWIZNETINFO.sn);
   getDNSfromDHCP(gWIZNETINFO.dns);
   gWIZNETINFO.dhcp = NETINFO_DHCP;
   /* Network initialization */
   network_init();      // apply from dhcp
//   printf("DHCP LEASED TIME : %d Sec.\r\n", getDHCPLeasetime());
}

/******************************************************************************
 * @brief  Network Init
 * Intialize the network information to be used in WIZCHIP
 *****************************************************************************/
void network_init(void)
{
	uint8_t tmpstr[6] = {0};
	wiz_NetInfo netinfo;

	// Set Network information from netinfo structure
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	// Get Network information
	ctlnetwork(CN_GET_NETINFO, (void*)&netinfo);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

//	if(netinfo.dhcp == NETINFO_DHCP) printf("\r\n=== %s NET CONF : DHCP ===\r\n",(char*)tmpstr);
//	else printf("\r\n=== %s NET CONF : Static ===\r\n",(char*)tmpstr);

//	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],
//			netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
//	printf("SIP: %d.%d.%d.%d\r\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
//	printf("GAR: %d.%d.%d.%d\r\n", netinfo.gw[0],netinfo.gw[1],netinfo.gw[2],netinfo.gw[3]);
//	printf("SUB: %d.%d.%d.%d\r\n", netinfo.sn[0],netinfo.sn[1],netinfo.sn[2],netinfo.sn[3]);
//	printf("DNS: %d.%d.%d.%d\r\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
//	printf("===========================\r\n");

}

void WDOG_FeedBack_TASK(void)
{
	if(SYS_Flags.WDG_Flag && (TIM_Count.WDG_Count_Back == 4))
	{
		IWDG_ReloadCounter();
		SYS_Flags.WDG_Flag = false;
		TIM_Count.WDG_Count_Back = 0;
	}
}

