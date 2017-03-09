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
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);//��λ��������
	delay_ms(50);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);//��λ��������
	delay_ms(200);
	
	//�ؼ��Ĵ��������PHY��λ
	setIR(0xF0);
	setSIR(W5500_int_reg.SIR_REG);
	setSn_IR(s,  W5500_int_reg.SnSIR_REG&0x0F);//send�������0x10;
	wizphy_reset();
	
	while((getPHYCFGR()&0x01) != PHYCFGR_LNK_ON);//�ȴ���̫���������
}

void platform_init(SOCKET s)
{
	
	SystemInit();//ϵͳʱ�ӳ�ʼ��
	USART_int(115200);//����1��ʼ��
	W5500_GPIO_Configuration();
	SPI_Configuration();//Config SPI
	delay_init();	//��ʱ��ʼ��
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
/*�ض���ʹ��PRINTF����*/
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
/* FILE is typedef� d in stdio.h. */ 
FILE __stdout;


/**********************************************************************************END OF USART************************************************************************************/

//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif
static uint8_t  fac_us=0;//us��ʱ������
static uint16_t fac_ms=0;//ms��ʱ������
#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
//systick�жϷ�����,ʹ��ucosʱ�õ�
void SysTick_Handler(void)
{				   
	OSIntEnter();		//�����ж�
    OSTimeTick();       //����ucos��ʱ�ӷ������               
    OSIntExit();        //���������л����ж�
}
#endif

//��ʼ���ӳٺ���
//��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()	 
{

#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
	uint32_t reload;
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;	//Ϊϵͳʱ�ӵ�1/8  
	 
#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
	reload=SystemCoreClock/8000000;		//ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/OS_TICKS_PER_SEC;//����OS_TICKS_PER_SEC�趨���ʱ��
							//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	fac_ms=1000/OS_TICKS_PER_SEC;//����ucos������ʱ�����ٵ�λ	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->LOAD=reload; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK    
#else
	fac_ms=(uint16_t)fac_us*1000;//��ucos��,����ÿ��ms��Ҫ��systickʱ����   
#endif
}								    

#ifdef OS_CRITICAL_METHOD	//ʹ����ucos
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;	//LOAD��ֵ	    	 
	ticks=nus*fac_us; 			//��Ҫ�Ľ�����	  		 
	tcnt=0;
	told=SysTick->VAL;        	//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	}; 									    
}
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(uint16_t nms)
{	
	if(OSRunning==TRUE)//���os�Ѿ�������	    
	{		  
		if(nms>=fac_ms)//��ʱ��ʱ�����ucos������ʱ������ 
		{
   			OSTimeDly(nms/fac_ms);//ucos��ʱ
		}
		nms%=fac_ms;				//ucos�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((uint32_t)(nms*1000));	//��ͨ��ʽ��ʱ,��ʱucos�޷���������.
}
#else//����ucosʱ
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
  void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
} 
#endif
/**********************************************************************************END OF Delay************************************************************************************/


/* ����I2C�������ӵ�GPIO�˿�*/
#define GPIO_PORT_I2C	GPIOB			/* GPIO�˿� */
#define I2C_SCL_PIN		GPIO_Pin_6			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* ���ӵ�SDA�����ߵ�GPIO */
#define EEPROM_ADDRESS 0xA0
/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  GPIO_SetBits(GPIOB,I2C_SCL_PIN)				/* SCL = 1 */
#define I2C_SCL_0()  GPIO_ResetBits(GPIOB,I2C_SCL_PIN)					/* SCL = 0 */

#define I2C_SDA_1()  GPIO_SetBits(GPIOB,I2C_SDA_PIN)			/* SDA = 1 */
#define I2C_SDA_0()  GPIO_ResetBits(GPIOB,I2C_SDA_PIN)				/* SDA = 0 */

#define I2C_SDA_READ()  GPIOB->IDR  & I2C_SDA_PIN	/* ��SDA����״̬ */
#define I2C_SCL_READ()  GPIOB->IDR  & I2C_SCL_PIN	/* ��SCL����״̬ */

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitI2C
*	����˵��: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	/* ��GPIOʱ�� */
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
	i2c_Stop();
}


/*
*********************************************************************************************************
*	�� �� ��: i2c_Delay
*	����˵��: I2C����λ�ӳ٣����400KHz
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_Delay(void)
{/*
	uint16_t i;

	��
		CPU��Ƶ168MHzʱ�����ڲ�Flash����, MDK���̲��Ż�����̨ʽʾ�����۲Ⲩ�Ρ�
		ѭ������Ϊ5ʱ��SCLƵ�� = 1.78MHz (����ʱ: 92ms, ��д������������ʾ����̽ͷ���ϾͶ�дʧ�ܡ�ʱ��ӽ��ٽ�)
		ѭ������Ϊ10ʱ��SCLƵ�� = 1.1MHz (����ʱ: 138ms, ���ٶ�: 118724B/s)
		ѭ������Ϊ30ʱ��SCLƵ�� = 440KHz�� SCL�ߵ�ƽʱ��1.0us��SCL�͵�ƽʱ��1.2us

		��������ѡ��2.2Kŷʱ��SCL������ʱ��Լ0.5us�����ѡ4.7Kŷ����������Լ1us

		ʵ��Ӧ��ѡ��400KHz���ҵ����ʼ���
	
	for (i = 0; i < 2000; i++);*/
	delay_us(10);
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C���������ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
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
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C����ֹͣ�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_SendByte
*	����˵��: CPU��I2C�����豸����8bit����
*	��    ��:  _ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
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
			 I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_ReadByte
*	����˵��: CPU��I2C�����豸��ȡ8bit����
*	��    ��:  ��
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
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
*	�� �� ��: EEPROM_WriteByte
*	����˵��: ��EEPROM�Ĵ���д��һ������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*			  _ucRegData : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EEPROM_WriteByte(uint8_t _ucRegAddr, uint8_t _ucRegData)
{
    i2c_Start();							/* ���߿�ʼ�ź� */

    i2c_SendByte(EEPROM_ADDRESS);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();

    i2c_SendByte(_ucRegAddr);				/* �ڲ��Ĵ�����ַ */
	i2c_WaitAck();

    i2c_SendByte(_ucRegData);				/* �ڲ��Ĵ������� */
	i2c_WaitAck();

    i2c_Stop();                   			/* ����ֹͣ�ź� */
}

/*
*********************************************************************************************************
*	�� �� ��: EEPROM_ReadByte
*	����˵��: ��ȡEEPROM�Ĵ���������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t EEPROM_ReadByte(uint8_t _ucRegAddr)
{
	uint8_t ucData;

	i2c_Start();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte(EEPROM_ADDRESS);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();
	i2c_SendByte(_ucRegAddr);     			/* ���ʹ洢��Ԫ��ַ */
	i2c_WaitAck();

	i2c_Start();                  			/* ���߿�ʼ�ź� */

	i2c_SendByte(EEPROM_ADDRESS+1); 	/* �����豸��ַ+���ź� */
	i2c_WaitAck();

	ucData = i2c_ReadByte();       			/* �����Ĵ������� */
	i2c_NAck();
	i2c_Stop();                  			/* ����ֹͣ�ź� */
	return ucData;
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_WaitAck
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    ��:  ��
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
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
*	�� �� ��: i2c_Ack
*	����˵��: CPU����һ��ACK�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_CheckDevice
*	����˵��: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
*	��    ��:  _Address���豸��I2C���ߵ�ַ
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* ���������ź� */

		/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* ����豸��ACKӦ�� */

		i2c_Stop();			/* ����ֹͣ�ź� */

		return ucAck;
	}
	return 1;	/* I2C�����쳣 */
}

/*******************************************************************************
* ������  : ADC_init
* ����    : ADC��ʼ��
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
* ������  : Get_ADC
* ����    : ��ȡADC����
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
* ������  : ADC_disable
* ����    : ADC�ر�
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void ADC_disable(void){
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, DISABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, DISABLE);

}

/*******************************************************************************
* ������  : MAC_rand
* ����    : �������MAC��ַ
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
* ������  : EEPROM_MAC_check
* ����    : MAC��ַ�Ĵ洢
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
  * @brief  ʹ��SPIʱ��
  * @retval None
  */
static void SPI_RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);
}
/**
  * @brief  ����ָ��SPI������
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
	//��ʼ��Ƭѡ�������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}
/**
  * @brief  �����ⲿSPI�豸����SPI��ز���
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
  * @brief  д1�ֽ����ݵ�SPI����
  * @param  TxData д�����ߵ�����
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{				 
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==0);	//�ȴ���������		  
	SPI1->DR=TxData;	 	  									//����һ��byte 
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //�ȴ�������һ��byte  
	SPI1->DR;		
}
/**
  * @brief  ��SPI���߶�ȡ1�ֽ�����
  * @retval ����������
  */
uint8_t SPI_ReadByte(void)
{			 
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==0);	//�ȴ���������			  
	SPI1->DR=0xFF;	 	  										//����һ�������ݲ����������ݵ�ʱ�� 
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //�ȴ�������һ��byte  
	return SPI1->DR;  						    
}
/**
  * @brief  �����ٽ���
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}
/**
  * @brief  �˳��ٽ���
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  Ƭѡ�ź�����͵�ƽ
  * @retval None
  */
void SPI_CS_Select(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
/**
  * @brief  Ƭѡ�ź�����ߵ�ƽ
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
	/* W5500_RST���ų�ʼ������(PA3) */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	
	/* W5500_INT���ų�ʼ������(PA2) */	
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
