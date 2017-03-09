/**
  ******************************************************************************
  * @file    main.c
  * $Author: liaojingjing$
  * $Revision: 17 $
  * $Date:: 2016-11-4 11:16:48 +0800 #$
  * @brief   Ö÷º¯Êý.
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "platform.h"

SOCKET SocketNum = 0;

int main(void)
{
	platform_init(SocketNum);
	printf("User Program started.....\r\n");
	printf(" 0x1ffff7e8 id:0x%X\n",(*(volatile uint32_t *)0x1ffff7e8));
	printf(" 0x1ffff7ec id:0x%X\n",(*(volatile uint32_t *)0x1ffff7ec));
	printf(" 0x1ffff7f0 id:0x%X\n",(*(volatile uint32_t *)0x1ffff7f0));
	LOOPs(SocketNum);
} 

/*
	GPIO_InitTypeDef GPIO_InitStructure;
	
	SystemInit();	
	delay_init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_2);
	delay_ms(1500);delay_ms(1500);delay_ms(1500);
SYSRESET();

	GPIO_InitTypeDef GPIO_InitStructure;
	
	SystemInit();	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	while(1)
	{
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
	delay_ms(1);
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
		delay_ms(1);
	}

*/


