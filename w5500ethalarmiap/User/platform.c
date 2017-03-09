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
COUNT TIM_Count;
FLAGS SYS_Flags;
PORT IN[14] = {
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_2},{.Port = GPIOC,.Port_Pin = GPIO_Pin_3},
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_0},{.Port = GPIOA,.Port_Pin = GPIO_Pin_1},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_0},{.Port = GPIOB,.Port_Pin = GPIO_Pin_1},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_12},{.Port = GPIOB,.Port_Pin = GPIO_Pin_13},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_14},{.Port = GPIOB,.Port_Pin = GPIO_Pin_15},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_6},{.Port = GPIOC,.Port_Pin = GPIO_Pin_7},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_8},{.Port = GPIOC,.Port_Pin = GPIO_Pin_9},
							};
PORT ARMING_PORT[2] = {
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_0},{.Port = GPIOC,.Port_Pin = GPIO_Pin_1},												
											};
PORT OUT[8] = {
								{.Port = GPIOD,.Port_Pin = GPIO_Pin_2,.Pin_Value = 0x00},{.Port = GPIOC,.Port_Pin = GPIO_Pin_12,.Pin_Value = 0x00},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_11,.Pin_Value = 0x00},{.Port = GPIOC,.Port_Pin = GPIO_Pin_10,.Pin_Value = 0x00},
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_15,.Pin_Value = 0x00},{.Port = GPIOA,.Port_Pin = GPIO_Pin_12,.Pin_Value = 0x00},
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_11,.Pin_Value = 0x00},{.Port = GPIOA,.Port_Pin = GPIO_Pin_8,.Pin_Value = 0x00},
							};
void SYSRESET(void)  {__disable_fault_irq(); NVIC_SystemReset();}
uint8_t Erro_Flag = 0x00;
uint16_t Input_Status_EEPROM;
uint8_t Output_Status_EEPROM;
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void W5500_RESET(SOCKET s)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);//¸´Î»Òý½ÅÀ­µÍ
	delay_ms(50);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);//¸´Î»Òý½ÅÀ­¸ß
	delay_ms(200);
	
	//¹Ø¼ü¼Ä´æÆ÷Çå³ý£¬PHY¸´Î»
	setIR(0xF0);
	setSIR(W5500_int_reg.SIR_REG);
	setSn_IR(s,  W5500_int_reg.SnSIR_REG&0x0F);//sendº¯ÊýÇå³ý0x10;
	wizphy_reset();
	
	while((getPHYCFGR()&0x01) != PHYCFGR_LNK_ON);//µÈ´ýÒÔÌ«ÍøÁ¬½ÓÍê³É
}

void platform_init(SOCKET s)
{
	
	SystemInit();//ÏµÍ³Ê±ÖÓ³õÊ¼»¯
	USART_int(115200);//´®¿Ú1³õÊ¼»¯
	W5500_GPIO_Configuration();
	SPI_Configuration();//Config SPI
	delay_init();	//ÑÓÊ±³õÊ¼»¯
	TIMER_init();
//	IWDG_Config();
	NVIC_init();
	EEPROM_MAC_check();
//	PORTInit();
	W5500_init(s);
}

void USART_int(long BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* PA10 USART1_Rx  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate;//??????
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???????8bit
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//????1
  USART_InitStructure.USART_Parity = USART_Parity_No;//????
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//??????none
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//??????????
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;     
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;      
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;      
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART1, &USART_ClockInitStructure);
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 USART_Cmd(USART1, ENABLE);
}

/*
void USART_SendStr(char *str)
{
   while((*str)!='\0')
	{
			USART_SendData(USART1,*str++);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
}
*/


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
void USART1_IRQHandler(void) 
{ 

  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
  { 
  } 
	
	
USART_SendData(USART1,USART_ReceiveData(USART1));
	if(USART_ReceiveData(USART1) == 's')
	{SYSRESET();}
	if(USART_ReceiveData(USART1) == 'a')
	{sys_upgradeflag = 0x01;}
  USART_ClearFlag(USART1, USART_FLAG_RXNE);

}
/*ÖØ¶¨ÏòÊ¹ÓÃPRINTFº¯Êý*/
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

#pragma import(__use_no_semihosting) 
void _sys_exit(int x) 
{ 
x = x; 
} 
struct __FILE 
{ 
int handle; 
/* Whatever you require here. If the only file you are using is */ 
/* standard output using printf() for debugging, no file handling */ 
/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;


/**********************************************************************************END OF USART************************************************************************************/

//Èç¹ûÊ¹ÓÃucos,Ôò°üÀ¨ÏÂÃæµÄÍ·ÎÄ¼þ¼´¿É.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos Ê¹ÓÃ	  
#endif
static uint8_t  fac_us=0;//usÑÓÊ±±¶³ËÊý
static uint16_t fac_ms=0;//msÑÓÊ±±¶³ËÊý
#ifdef OS_CRITICAL_METHOD 	//Èç¹ûOS_CRITICAL_METHOD¶¨ÒåÁË,ËµÃ÷Ê¹ÓÃucosIIÁË.
//systickÖÐ¶Ï·þÎñº¯Êý,Ê¹ÓÃucosÊ±ÓÃµ½
void SysTick_Handler(void)
{				   
	OSIntEnter();		//½øÈëÖÐ¶Ï
    OSTimeTick();       //µ÷ÓÃucosµÄÊ±ÖÓ·þÎñ³ÌÐò               
    OSIntExit();        //´¥·¢ÈÎÎñÇÐ»»ÈíÖÐ¶Ï
}
#endif

//³õÊ¼»¯ÑÓ³Ùº¯Êý
//µ±Ê¹ÓÃucosµÄÊ±ºò,´Ëº¯Êý»á³õÊ¼»¯ucosµÄÊ±ÖÓ½ÚÅÄ
//SYSTICKµÄÊ±ÖÓ¹Ì¶¨ÎªHCLKÊ±ÖÓµÄ1/8
//SYSCLK:ÏµÍ³Ê±ÖÓ
void delay_init()	 
{

#ifdef OS_CRITICAL_METHOD 	//Èç¹ûOS_CRITICAL_METHOD¶¨ÒåÁË,ËµÃ÷Ê¹ÓÃucosIIÁË.
	uint32_t reload;
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//Ñ¡ÔñÍâ²¿Ê±ÖÓ  HCLK/8
	fac_us=SystemCoreClock/8000000;	//ÎªÏµÍ³Ê±ÖÓµÄ1/8  
	 
#ifdef OS_CRITICAL_METHOD 	//Èç¹ûOS_CRITICAL_METHOD¶¨ÒåÁË,ËµÃ÷Ê¹ÓÃucosIIÁË.
	reload=SystemCoreClock/8000000;		//Ã¿ÃëÖÓµÄ¼ÆÊý´ÎÊý µ¥Î»ÎªK	   
	reload*=1000000/OS_TICKS_PER_SEC;//¸ù¾ÝOS_TICKS_PER_SECÉè¶¨Òç³öÊ±¼ä
							//reloadÎª24Î»¼Ä´æÆ÷,×î´óÖµ:16777216,ÔÚ72MÏÂ,Ô¼ºÏ1.86s×óÓÒ	
	fac_ms=1000/OS_TICKS_PER_SEC;//´ú±íucos¿ÉÒÔÑÓÊ±µÄ×îÉÙµ¥Î»	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//¿ªÆôSYSTICKÖÐ¶Ï
	SysTick->LOAD=reload; 	//Ã¿1/OS_TICKS_PER_SECÃëÖÐ¶ÏÒ»´Î	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//¿ªÆôSYSTICK    
#else
	fac_ms=(uint16_t)fac_us*1000;//·ÇucosÏÂ,´ú±íÃ¿¸ömsÐèÒªµÄsystickÊ±ÖÓÊý   
#endif
}								    

#ifdef OS_CRITICAL_METHOD	//Ê¹ÓÃÁËucos
//ÑÓÊ±nus
//nusÎªÒªÑÓÊ±µÄusÊý.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;	//LOADµÄÖµ	    	 
	ticks=nus*fac_us; 			//ÐèÒªµÄ½ÚÅÄÊý	  		 
	tcnt=0;
	told=SysTick->VAL;        	//¸Õ½øÈëÊ±µÄ¼ÆÊýÆ÷Öµ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//ÕâÀï×¢ÒâÒ»ÏÂSYSTICKÊÇÒ»¸öµÝ¼õµÄ¼ÆÊýÆ÷¾Í¿ÉÒÔÁË.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//Ê±¼ä³¬¹ý/µÈÓÚÒªÑÓ³ÙµÄÊ±¼ä,ÔòÍË³ö.
		}  
	}; 									    
}
//ÑÓÊ±nms
//nms:ÒªÑÓÊ±µÄmsÊý
void delay_ms(uint16_t nms)
{	
	if(OSRunning==TRUE)//Èç¹ûosÒÑ¾­ÔÚÅÜÁË	    
	{		  
		if(nms>=fac_ms)//ÑÓÊ±µÄÊ±¼ä´óÓÚucosµÄ×îÉÙÊ±¼äÖÜÆÚ 
		{
   			OSTimeDly(nms/fac_ms);//ucosÑÓÊ±
		}
		nms%=fac_ms;				//ucosÒÑ¾­ÎÞ·¨Ìá¹©ÕâÃ´Ð¡µÄÑÓÊ±ÁË,²ÉÓÃÆÕÍ¨·½Ê½ÑÓÊ±    
	}
	delay_us((uint32_t)(nms*1000));	//ÆÕÍ¨·½Ê½ÑÓÊ±,´ËÊ±ucosÎÞ·¨Æô¶¯µ÷¶È.
}
#else//²»ÓÃucosÊ±
//ÑÓÊ±nus
//nusÎªÒªÑÓÊ±µÄusÊý.		    								   
  void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //Ê±¼ä¼ÓÔØ	  		 
	SysTick->VAL=0x00;        //Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //¿ªÊ¼µ¹Êý	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL =0X00;       //Çå¿Õ¼ÆÊýÆ÷	 
}
//ÑÓÊ±nms
//×¢ÒânmsµÄ·¶Î§
//SysTick->LOADÎª24Î»¼Ä´æÆ÷,ËùÒÔ,×î´óÑÓÊ±Îª:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLKµ¥Î»ÎªHz,nmsµ¥Î»Îªms
//¶Ô72MÌõ¼þÏÂ,nms<=1864 
void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//Ê±¼ä¼ÓÔØ(SysTick->LOADÎª24bit)
	SysTick->VAL =0x00;           //Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //¿ªÊ¼µ¹Êý  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL =0X00;       //Çå¿Õ¼ÆÊýÆ÷	  	    
} 
#endif
/**********************************************************************************END OF Delay************************************************************************************/


/* ¶¨ÒåI2C×ÜÏßÁ¬½ÓµÄGPIO¶Ë¿Ú*/
#define GPIO_PORT_I2C	GPIOB			/* GPIO¶Ë¿Ú */
#define I2C_SCL_PIN		GPIO_Pin_6			/* Á¬½Óµ½SCLÊ±ÖÓÏßµÄGPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* Á¬½Óµ½SDAÊý¾ÝÏßµÄGPIO */
#define EEPROM_ADDRESS 0xA0
/* ¶¨Òå¶ÁÐ´SCLºÍSDAµÄºê */
#define I2C_SCL_1()  GPIO_SetBits(GPIOB,I2C_SCL_PIN)				/* SCL = 1 */
#define I2C_SCL_0()  GPIO_ResetBits(GPIOB,I2C_SCL_PIN)					/* SCL = 0 */

#define I2C_SDA_1()  GPIO_SetBits(GPIOB,I2C_SDA_PIN)			/* SDA = 1 */
#define I2C_SDA_0()  GPIO_ResetBits(GPIOB,I2C_SDA_PIN)				/* SDA = 0 */

#define I2C_SDA_READ()  GPIOB->IDR  & I2C_SDA_PIN	/* ¶ÁSDA¿ÚÏß×´Ì¬ */
#define I2C_SCL_READ()  GPIOB->IDR  & I2C_SCL_PIN	/* ¶ÁSCL¿ÚÏß×´Ì¬ */

/*
*********************************************************************************************************
*	º¯ Êý Ãû: bsp_InitI2C
*	¹¦ÄÜËµÃ÷: ÅäÖÃI2C×ÜÏßµÄGPIO£¬²ÉÓÃÄ£ÄâIOµÄ·½Ê½ÊµÏÖ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	/* ´ò¿ªGPIOÊ±ÖÓ */
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;		/* ÉèÎªÊä³ö¿Ú */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO¿Ú×î´óËÙ¶È */
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* ¸øÒ»¸öÍ£Ö¹ÐÅºÅ, ¸´Î»I2C×ÜÏßÉÏµÄËùÓÐÉè±¸µ½´ý»úÄ£Ê½ */
	i2c_Stop();
}


/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_Delay
*	¹¦ÄÜËµÃ÷: I2C×ÜÏßÎ»ÑÓ³Ù£¬×î¿ì400KHz
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void i2c_Delay(void)
{/*
	uint16_t i;

	¡¡
		CPUÖ÷Æµ168MHzÊ±£¬ÔÚÄÚ²¿FlashÔËÐÐ, MDK¹¤³Ì²»ÓÅ»¯¡£ÓÃÌ¨Ê½Ê¾²¨Æ÷¹Û²â²¨ÐÎ¡£
		Ñ­»·´ÎÊýÎª5Ê±£¬SCLÆµÂÊ = 1.78MHz (¶ÁºÄÊ±: 92ms, ¶ÁÐ´Õý³££¬µ«ÊÇÓÃÊ¾²¨Æ÷Ì½Í·ÅöÉÏ¾Í¶ÁÐ´Ê§°Ü¡£Ê±Ðò½Ó½üÁÙ½ç)
		Ñ­»·´ÎÊýÎª10Ê±£¬SCLÆµÂÊ = 1.1MHz (¶ÁºÄÊ±: 138ms, ¶ÁËÙ¶È: 118724B/s)
		Ñ­»·´ÎÊýÎª30Ê±£¬SCLÆµÂÊ = 440KHz£¬ SCL¸ßµçÆ½Ê±¼ä1.0us£¬SCLµÍµçÆ½Ê±¼ä1.2us

		ÉÏÀ­µç×èÑ¡Ôñ2.2KÅ·Ê±£¬SCLÉÏÉýÑØÊ±¼äÔ¼0.5us£¬Èç¹ûÑ¡4.7KÅ·£¬ÔòÉÏÉýÑØÔ¼1us

		Êµ¼ÊÓ¦ÓÃÑ¡Ôñ400KHz×óÓÒµÄËÙÂÊ¼´¿É
	
	for (i = 0; i < 2000; i++);*/
	delay_us(10);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_Start
*	¹¦ÄÜËµÃ÷: CPU·¢ÆðI2C×ÜÏßÆô¶¯ÐÅºÅ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* µ±SCL¸ßµçÆ½Ê±£¬SDA³öÏÖÒ»¸öÏÂÌøÑØ±íÊ¾I2C×ÜÏßÆô¶¯ÐÅºÅ */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_Start
*	¹¦ÄÜËµÃ÷: CPU·¢ÆðI2C×ÜÏßÍ£Ö¹ÐÅºÅ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* µ±SCL¸ßµçÆ½Ê±£¬SDA³öÏÖÒ»¸öÉÏÌøÑØ±íÊ¾I2C×ÜÏßÍ£Ö¹ÐÅºÅ */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_SendByte
*	¹¦ÄÜËµÃ÷: CPUÏòI2C×ÜÏßÉè±¸·¢ËÍ8bitÊý¾Ý
*	ÐÎ    ²Î:  _ucByte £º µÈ´ý·¢ËÍµÄ×Ö½Ú
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* ÏÈ·¢ËÍ×Ö½ÚµÄ¸ßÎ»bit7 */
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // ÊÍ·Å×ÜÏß
		}
		_ucByte <<= 1;	/* ×óÒÆÒ»¸öbit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_ReadByte
*	¹¦ÄÜËµÃ÷: CPU´ÓI2C×ÜÏßÉè±¸¶ÁÈ¡8bitÊý¾Ý
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ¶Áµ½µÄÊý¾Ý
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ¶Áµ½µÚ1¸öbitÎªÊý¾ÝµÄbit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: EEPROM_WriteByte
*	¹¦ÄÜËµÃ÷: ÏòEEPROM¼Ä´æÆ÷Ð´ÈëÒ»¸öÊý¾Ý
*	ÐÎ    ²Î: _ucRegAddr : ¼Ä´æÆ÷µØÖ·
*			  _ucRegData : ¼Ä´æÆ÷Êý¾Ý
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void EEPROM_WriteByte(uint8_t _ucRegAddr, uint8_t _ucRegData)
{
    i2c_Start();							/* ×ÜÏß¿ªÊ¼ÐÅºÅ */

    i2c_SendByte(EEPROM_ADDRESS);	/* ·¢ËÍÉè±¸µØÖ·+Ð´ÐÅºÅ */
	i2c_WaitAck();

    i2c_SendByte(_ucRegAddr);				/* ÄÚ²¿¼Ä´æÆ÷µØÖ· */
	i2c_WaitAck();

    i2c_SendByte(_ucRegData);				/* ÄÚ²¿¼Ä´æÆ÷Êý¾Ý */
	i2c_WaitAck();

    i2c_Stop();                   			/* ×ÜÏßÍ£Ö¹ÐÅºÅ */
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: EEPROM_ReadByte
*	¹¦ÄÜËµÃ÷: ¶ÁÈ¡EEPROM¼Ä´æÆ÷µÄÊý¾Ý
*	ÐÎ    ²Î: _ucRegAddr : ¼Ä´æÆ÷µØÖ·
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
uint8_t EEPROM_ReadByte(uint8_t _ucRegAddr)
{
	uint8_t ucData;

	i2c_Start();                  			/* ×ÜÏß¿ªÊ¼ÐÅºÅ */
	i2c_SendByte(EEPROM_ADDRESS);	/* ·¢ËÍÉè±¸µØÖ·+Ð´ÐÅºÅ */
	i2c_WaitAck();
	i2c_SendByte(_ucRegAddr);     			/* ·¢ËÍ´æ´¢µ¥ÔªµØÖ· */
	i2c_WaitAck();

	i2c_Start();                  			/* ×ÜÏß¿ªÊ¼ÐÅºÅ */

	i2c_SendByte(EEPROM_ADDRESS+1); 	/* ·¢ËÍÉè±¸µØÖ·+¶ÁÐÅºÅ */
	i2c_WaitAck();

	ucData = i2c_ReadByte();       			/* ¶Á³ö¼Ä´æÆ÷Êý¾Ý */
	i2c_NAck();
	i2c_Stop();                  			/* ×ÜÏßÍ£Ö¹ÐÅºÅ */
	return ucData;
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_WaitAck
*	¹¦ÄÜËµÃ÷: CPU²úÉúÒ»¸öÊ±ÖÓ£¬²¢¶ÁÈ¡Æ÷¼þµÄACKÓ¦´ðÐÅºÅ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ·µ»Ø0±íÊ¾ÕýÈ·Ó¦´ð£¬1±íÊ¾ÎÞÆ÷¼þÏìÓ¦
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPUÊÍ·ÅSDA×ÜÏß */
	i2c_Delay();
	I2C_SCL_1();	/* CPUÇý¶¯SCL = 1, ´ËÊ±Æ÷¼þ»á·µ»ØACKÓ¦´ð */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU¶ÁÈ¡SDA¿ÚÏß×´Ì¬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_Ack
*	¹¦ÄÜËµÃ÷: CPU²úÉúÒ»¸öACKÐÅºÅ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPUÇý¶¯SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU²úÉú1¸öÊ±ÖÓ */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPUÊÍ·ÅSDA×ÜÏß */
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_NAck
*	¹¦ÄÜËµÃ÷: CPU²úÉú1¸öNACKÐÅºÅ
*	ÐÎ    ²Î:  ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPUÇý¶¯SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU²úÉú1¸öÊ±ÖÓ */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: i2c_CheckDevice
*	¹¦ÄÜËµÃ÷: ¼ì²âI2C×ÜÏßÉè±¸£¬CPUÏò·¢ËÍÉè±¸µØÖ·£¬È»ºó¶ÁÈ¡Éè±¸Ó¦´ðÀ´ÅÐ¶Ï¸ÃÉè±¸ÊÇ·ñ´æÔÚ
*	ÐÎ    ²Î:  _Address£ºÉè±¸µÄI2C×ÜÏßµØÖ·
*	·µ »Ø Öµ: ·µ»ØÖµ 0 ±íÊ¾ÕýÈ·£¬ ·µ»Ø1±íÊ¾Î´Ì½²âµ½
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* ·¢ËÍÆô¶¯ÐÅºÅ */

		/* ·¢ËÍÉè±¸µØÖ·+¶ÁÐ´¿ØÖÆbit£¨0 = w£¬ 1 = r) bit7 ÏÈ´« */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* ¼ì²âÉè±¸µÄACKÓ¦´ð */

		i2c_Stop();			/* ·¢ËÍÍ£Ö¹ÐÅºÅ */

		return ucAck;
	}
	return 1;	/* I2C×ÜÏßÒì³£ */
}

/*******************************************************************************
* º¯ÊýÃû  : ADC_init
* ÃèÊö    : ADC³õÊ¼»¯
* ÊäÈë    : ÎÞ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ÎÞ
*******************************************************************************/
void ADC_init(void){
	GPIO_InitTypeDef   GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  /* ADCCLK = PCLK2/2 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
#else
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
#endif
ADC_DeInit(ADC1);
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	/* Configure PB0 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

}

/*******************************************************************************
* º¯ÊýÃû  : Get_ADC
* ÃèÊö    : ¶ÁÈ¡ADCÔëÉù
* ÊäÈë    : ÎÞ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ÎÞ
*******************************************************************************/
int Get_ADC(void){
	 /* ADC1 regular channel configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
	/* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
//  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
//  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	return ADC_GetConversionValue(ADC1);
}

/*******************************************************************************
* º¯ÊýÃû  : ADC_disable
* ÃèÊö    : ADC¹Ø±Õ
* ÊäÈë    : ÎÞ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ÎÞ
*******************************************************************************/
void ADC_disable(void){
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, DISABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, DISABLE);

}

/*******************************************************************************
* º¯ÊýÃû  : MAC_rand
* ÃèÊö    : Ëæ»úÉú³ÉMACµØÖ·
* ÊäÈë    : ÎÞ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ÎÞ
*******************************************************************************/
void MAC_rand(void)
{
	unsigned char i;
	ADC_init();
	for(i = 0; i < 3 ;i++)
		{  
			srand(Get_ADC());
			gWIZNETINFO.mac[3+i] = rand();
			srand(gWIZNETINFO.mac[3+i]);
			printf("%02x:",gWIZNETINFO.mac[3+i]);
		}
	ADC_disable();
}
/*******************************************************************************
* º¯ÊýÃû  : EEPROM_MAC_check
* ÃèÊö    : MACµØÖ·µÄ´æ´¢
* ÊäÈë    : ÎÞ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ÎÞ
*******************************************************************************/
void EEPROM_MAC_check(void)
{
	unsigned char i;
	InitI2C();
	if(i2c_CheckDevice(EEPROM_ADDRESS)== 0)
	{
//		printf("got 24cxx on iic bus\r\n");
		if(EEPROM_ReadByte(0x00) == 0xbb)
		{
			printf(" MAC already in EEPROM......\r\n");
			for(i = 0; i< 3; i++)
					{
						gWIZNETINFO.mac[3+i] = EEPROM_ReadByte(i+1);
						delay_ms(5);
					}
		}
		else {
					printf(" MAC random & store in EEPROM......\r\n");
					MAC_rand();
					for(i = 0; i< 3; i++)
					{
						EEPROM_WriteByte(MAC_reg_add + i, gWIZNETINFO.mac[3+i]);
						delay_ms(5);
					}
					EEPROM_WriteByte(0x00, 0xbb);
					delay_ms(5);
//					printf("got 24cxx 0x00 0x%02X\r\n",EEPROM_ReadByte(0x00));
				}
//		printf("MAC=");			
//		for(i = 0; i < 6; i++)
//		{
//			if(i == 5)
//			{printf("%02X\r\n",gWIZNETINFO.mac[i]);}
//			else {printf("%02X:",gWIZNETINFO.mac[i]);}
//		}
	}
	else {printf("got nothings on iic bus\r\n");SYSRESET();}
}

/**********************************************************************************END OF MACRand************************************************************************************/


/**
  * @brief  Ê¹ÄÜSPIÊ±ÖÓ
  * @retval None
  */
static void SPI_RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);
}
/**
  * @brief  ÅäÖÃÖ¸¶¨SPIµÄÒý½Å
  * @retval None
  */
static void SPI_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//PA4->CS,PA5->SCK,PA6->MISO,PA7->MOSI		 					 
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//³õÊ¼»¯Æ¬Ñ¡Êä³öÒý½Å
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}
/**
  * @brief  ¸ù¾ÝÍâ²¿SPIÉè±¸ÅäÖÃSPIÏà¹Ø²ÎÊý
  * @retval None
  */
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_RCC_Configuration();

	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1,&SPI_InitStruct);
	
	SPI_GPIO_Configuration();

	SPI_SSOutputCmd(SPI1, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
}
/**
  * @brief  Ð´1×Ö½ÚÊý¾Ýµ½SPI×ÜÏß
  * @param  TxData Ð´µ½×ÜÏßµÄÊý¾Ý
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{				 
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==0);	//µÈ´ý·¢ËÍÇø¿Õ		  
	SPI1->DR=TxData;	 	  									//·¢ËÍÒ»¸öbyte 
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //µÈ´ý½ÓÊÕÍêÒ»¸öbyte  
	SPI1->DR;		
}
/**
  * @brief  ´ÓSPI×ÜÏß¶ÁÈ¡1×Ö½ÚÊý¾Ý
  * @retval ¶Áµ½µÄÊý¾Ý
  */
uint8_t SPI_ReadByte(void)
{			 
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==0);	//µÈ´ý·¢ËÍÇø¿Õ			  
	SPI1->DR=0xFF;	 	  										//·¢ËÍÒ»¸ö¿ÕÊý¾Ý²úÉúÊäÈëÊý¾ÝµÄÊ±ÖÓ 
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //µÈ´ý½ÓÊÕÍêÒ»¸öbyte  
	return SPI1->DR;  						    
}
/**
  * @brief  ½øÈëÁÙ½çÇø
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}
/**
  * @brief  ÍË³öÁÙ½çÇø
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  Æ¬Ñ¡ÐÅºÅÊä³öµÍµçÆ½
  * @retval None
  */
void SPI_CS_Select(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
/**
  * @brief  Æ¬Ñ¡ÐÅºÅÊä³ö¸ßµçÆ½
  * @retval None
  */
void SPI_CS_Deselect(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}
/**************************************************************************END OF SPI************************************************************************************/

void TIMER_init(void)//1ms interrupt
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 9;
  TIM_TimeBaseStructure.TIM_Prescaler = 7200;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 
	/* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void) //TIM2 IRQ
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		//sec min hour
		if(TIM_Count.mSec_Count >= 1000)
		{
			TIM_Count.Sec_Count ++;
			TIM_Count.mSec_Count = 0;
			
			if(TIM_Count.Sec_Count >= 60)
			{
				TIM_Count.Sec_Count = 0;
				TIM_Count.Min_Count ++;
				SYS_Flags.KeepAlive_Flag = true;

				if(TIM_Count.Min_Count >= 60)
				{
					TIM_Count.Min_Count = 0;
					TIM_Count.Hou_Count ++;
				}
				else{;}
			}
			else{;}
		}
		else {TIM_Count.mSec_Count++;}
		
		if(TIM_Count.Interrupt_Count == 100)
		{
			SYS_Flags.SYS_Interrupt=true;
			TIM_Count.Interrupt_Count = 0;
		}
		else {TIM_Count.Interrupt_Count++;}
		
		if(TIM_Count.WDG_Count < 5000)//<5s set WDG flag
		{TIM_Count.WDG_Count++;}
		else {
						TIM_Count.WDG_Count = 0;
						TIM_Count.WDG_Count_Back ++;
					}
		if(TIM_Count.WDG_Count_Back > 3) 
		{
			IWDG_ReloadCounter();
			SYS_Flags.WDG_Flag = false;
			TIM_Count.WDG_Count_Back = 0;
		}
		else{;}
					
		if(TIM_Count.W5500_Reset_Count == 20000)//20s got no data network init
		{
			TIM_Count.W5500_Reset_Count = 0;
			if(SYS_Flags.Got_Data_Flag == true)
				{
					SYS_Flags.Reset_Flag = false;
					SYS_Flags.Got_Data_Flag = false;
				}
			else{
						SYS_Flags.Reset_Flag = true;
					}
		}
		else{
					TIM_Count.W5500_Reset_Count++;
				}
				
		if(TIM_Count.Send_Count > 1000) {TIM_Count.Send_Count = 0;}
			else {TIM_Count.Send_Count++;}
			
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update );
	}
}

void IWDG_Config(void)//26s IWDG
{ 
   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
   IWDG_SetPrescaler(IWDG_Prescaler_256);//40KHz 256Div
   IWDG_SetReload(0xFFF);                 //count
   IWDG_ReloadCounter();          
   IWDG_Enable();                 
}

void W5500_GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* W5500_RSTÒý½Å³õÊ¼»¯ÅäÖÃ(PA3) */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	
	/* W5500_INTÒý½Å³õÊ¼»¯ÅäÖÃ(PA2) */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	/* Connect EXTI Line2 to PA2 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	/* PA2 as W5500 interrupt input */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the EXTI2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
//		SYS_Flags.W5500_Interrupt=true;
//		W5500_int_reg.IR_REG = getIR();
//		W5500_int_reg.SIR_REG = getSIR();
//		W5500_int_reg.SnSIR_REG = getSn_IR(0);
//		W5500_int_reg.SnSR_REG = getSn_SR(0);	
//		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}
/**************************************************************************END OF EXTI2&TIM&WTD&NVIC************************************************************************************/


/*********************************END OF FILE**********************************/
