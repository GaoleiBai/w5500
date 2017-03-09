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
                            .ip = {192, 168, 1, 111},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {0,0,0,0},
                            .dhcp = NETINFO_STATIC };
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t DstIP[4]={103,44,145,247};//{172,17,50,70};//{192,168,1,223};//;
uint16_t DstPort = 10439;//8090;//25853;//11376;
uint16_t LocalPort = 5000;
uint8_t W5500_Status;

uint8_t Arming_Status;

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

bool socket_start(SOCKET s)
{
	int ret;
	ret = socket(s,Sn_MR_TCP,LocalPort,0x00);
	if(ret != 0){
		printf("%d:Socket start Error\r\n",s);
		return false;
	}else{
		printf("%d:Opened\r\n",s);
	}
	//连接TCP服务器
	ret = connect(s,DstIP,DstPort);
	if(ret != SOCK_OK){
		printf("%d:Socket Connect Error\r\n",s);
		W5500_Status = W5500_begin;
	}	
	else if(ret == SOCK_OK)
		{return true;}
	return false;
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
//uint8_t ip[4];
	if(ETH_STATUS == DHCP)
		{
  			  // must be set the default mac before DHCP started.
			setSHAR(gWIZNETINFO.mac);
			DHCP_init(SOCK_DHCP, gDATABUF);
			// if you want defiffent action instead defalut ip assign,update, conflict,
			// if cbfunc == 0, act as default.
			reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
		}
	else if(ETH_STATUS == Static_IP)
		{
			setSHAR(gWIZNETINFO.mac);/*配置Mac地址*/
 			 setSUBR(gWIZNETINFO.sn);/*配置子网掩码*/
 			 setGAR(gWIZNETINFO.gw);/*配置默认网关*/
  			setSIPR(gWIZNETINFO.ip);/*配置Ip地址*/
		}
	else{;}
	
//	getSIPR (ip);
// 	 printf("IP : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
// 	 getSUBR(ip);
//  	printf("SN : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
//  	getGAR(ip);
//  	printf("GW : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
	
}

void W5500_init(SOCKET s)
{
	W5500_function_register();
	W5500_RESET(s);
	while((getMR() & 0x80) == 0x80);
	W5500_interrupt_config(s);
	W5500_NETWORK_init(s);
	TIM_Count.DHCP_ReConnect_Count	= 6;
	SYS_Flags.W5500_Send_OK = true;
}

bool W5500_Check(SOCKET s)
{
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
		setSn_IR(s,  W5500_int_reg.SnSIR_REG&0x1F);
		setPHYCFGR(W5500_int_reg.PHY_LNK_REG | 0x01);

		if(((W5500_int_reg.PHY_LNK_REG & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_OFF) || ((W5500_int_reg.IR_REG & 0x80) == 0x80) || ((W5500_int_reg.IR_REG & 0x20) == 0x20)) //检查link情况,无连接返回false;ip冲突 或 目标不可达
		{
			W5500_Status = W5500_begin;
			return false;
		}
		else{;}
		
		
		if(W5500_int_reg.SIR_REG == 0x01)//s0中断到来
		{
			if(((W5500_int_reg.SnSIR_REG & 0x02) == 0x02)|| ((W5500_int_reg.SnSIR_REG) == 0x08)) //socket 断开或连接超时
			{
				if(ETH_STATUS == DHCP)
				{
					close(s);
					socket_start(s);//重启网络socket
				}
				else if(ETH_STATUS == Static_IP)
					{W5500_Status = W5500_begin;}
					else {;}
			}
			if((W5500_int_reg.SnSIR_REG & 0x04) == 0x04)
				{SYS_Flags.W5500_Read_OK = true;}
			if((W5500_int_reg.SnSIR_REG & 0x10) == 0x10)
				{SYS_Flags.W5500_Send_OK = true;}
		}	
	}
	else{;}
		
	return true;
}

void SCAN_LOOP_TASK(SOCKET s)//状态机实现
{
	uint8_t buff[4];
	if(sys_upgradeflag!=0)//如果发现升级标志，跳转IAP代码，准备远程更新
	{			
		jump();
	}
	
	if(ETH_STATUS == DHCP)
	{
		switch (W5500_Status)//查询5500寄存器标志，完成网络连接状态及中断信息更新
		{
			case W5500_begin :
				close(s);
				W5500_init(s);
				W5500_Status = W5500_Reset_OK;
			break;

			case W5500_Reset_OK :
				if((getPHYCFGR() & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_OFF)//检查link情况,无连接返回继续检测
				{
					W5500_Status = W5500_Reset_OK;
				}
				else if((getPHYCFGR() & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_ON)
						{
							W5500_Status = W5500_PHY_OK;
						}
				else{;}
			break;

			case W5500_PHY_OK ://PHY链接成功，开始DHCP,10s无法获取退出，1分钟后重试
				if(TIM_Count.DHCP_ReConnect_Count > 1)
				{
					TIM_Count.DHCP_ReConnect_Count = 0;
					SYS_Flags.DHCP_ReCount_Flag = false;
					DHCP_init(SOCK_DHCP, gDATABUF);
					delay_ms(1000);delay_ms(1000);//华为荣耀千兆以太网口路由器需要延时才可以自动获取
					do
					{
						if (TIM_Count.DHCP_TimeOut > 10)
							{
								SYS_Flags.DHCP_Count_Flag = false;
								SYS_Flags.DHCP_ReCount_Flag = true;
								W5500_Status = W5500_begin;
								break;
							}
						else {
								SYS_Flags.DHCP_Count_Flag = true;
								W5500_Status = W5500_DHCP_OK;
							 }
					}
					while(DHCP_run() != DHCP_IP_LEASED);//DHCP超时10s，返回连接出错
				}
				else{;}
			break;

			case W5500_DHCP_OK :
				close(s);
				if(socket_start(s))
					{
						W5500_Status = W5500_SOCK_OK;
						EEPROM_GetData(Local_Port_add, buff,2);
						LocalPort = (((uint16_t )buff[0]) << 8) | ((uint16_t )buff[1]);
						EEPROM_GetData(Dst_Port_add, buff,2);
						DstPort = (((uint16_t )buff[0]) << 8) | ((uint16_t )buff[1]);
						printf("mac : %02X.%02X.%02X\r\n", gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
						printf("LocalIP : %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
						printf("GateWay : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
						printf("SubNet : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
						printf("DstIP : %d.%d.%d.%d\r\n", DstIP[0],DstIP[1],DstIP[2],DstIP[3]);
						printf("LocalPort : %d\r\n", LocalPort);
						printf("DstPort : %d\r\n", DstPort);
					}
				else {W5500_Status = W5500_begin;} //socket链接失败，重新处理
			break;

			case W5500_SOCK_OK :
				W5500_Check(s);
			break;
		}
	}
	else if(ETH_STATUS == Static_IP)
		{
			 switch(W5500_Status)
			    {
			    	case W5500_begin :
					close(s);
					W5500_init(s);
					W5500_Status = W5500_Reset_OK;
					break;

					case W5500_Reset_OK :
						if((getPHYCFGR() & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_OFF)//检查link情况,无连接返回继续检测
						{
							W5500_Status = W5500_Reset_OK;
						}
						else if((getPHYCFGR() & PHYCFGR_LNK_STATUS) == PHYCFGR_LNK_ON)
								{
									W5500_Status = W5500_PHY_OK;
								}
						else{;}
					break;
					 
			       case W5500_PHY_OK:
								close(s);
			        	if(socket_start(s))
								{
									W5500_Status = W5500_SOCK_OK;
									dataSEND(s,KEEPALIVE); 
								}
								else {W5500_Status = W5500_PHY_OK;} //socket链接失败，重新处理
			         break;
					 
			       case W5500_SOCK_OK:
								W5500_Check(s);
			       break;
					 
			       default:
			         break;
     }
		}
	else{;}
	
}

void LOOPs(SOCKET s)
{
	while(1)
	{
		SCAN_LOOP_TASK(s);
		PROC_LOOPs();
//		GPRS_LOOPs();
		WDOG_FeedBack_TASK();
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
		IWDG_ReloadCounter();
		SYS_Flags.WDG_Flag = true;
		TIM_Count.WDG_Count_Back = 0;
}

