/**
  ******************************************************************************
  * @file    main.c
  * $Author: liaojingjing$
  * $Revision: 17 $
  * $Date:: 2016-11-4 11:16:48 +0800 #$
  * @brief   ������.
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "platform.h"

SOCKET SocketNum = 0;

int main(void)
{
	if(*( uint32_t*)(UPDATE_FLAG_Address) != GO_UPDATE_FLAG)//�ж��Ƿ���Ҫ����
	{go_jump();}
	else
	{
		platform_init(SocketNum);
		printf("IAP Program started.....\r\n");
		LOOPs(SocketNum);
	}
} 

