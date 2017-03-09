/**
  ******************************************************************************
  * @file    main.c
  * $Author: liaojingjing$
  * $Revision: 17 $
  * $Date:: 2016-11-4 11:16:48 +0800 #$
  * @brief   主函数.
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "platform.h"

SOCKET SocketNum = 0;

int main(void)
{
	if(*( uint32_t*)(UPDATE_FLAG_Address) != GO_UPDATE_FLAG)//判断是否需要升级
	{go_jump();}
	else
	{
		platform_init(SocketNum);
		printf("IAP Program started.....\r\n");
		LOOPs(SocketNum);
	}
} 

