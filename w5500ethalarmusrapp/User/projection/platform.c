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

uint8_t HardWareVersionNum[4] = {16,11,0,2} ;
uint8_t SoftWareVersionNum[4] = {16,12,0,1} ;

uint8_t ETH_STATUS = NULL;
COUNT TIM_Count;
DATE Current_Date;
FLAGS SYS_Flags;
PORT IN[14] = {
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_9},{.Port = GPIOC,.Port_Pin = GPIO_Pin_8},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_7},{.Port = GPIOC,.Port_Pin = GPIO_Pin_6},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_15},{.Port = GPIOB,.Port_Pin = GPIO_Pin_14},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_13},{.Port = GPIOB,.Port_Pin = GPIO_Pin_12},
								{.Port = GPIOB,.Port_Pin = GPIO_Pin_1},{.Port = GPIOB,.Port_Pin = GPIO_Pin_0},
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_1},{.Port = GPIOA,.Port_Pin = GPIO_Pin_0},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_3},{.Port = GPIOC,.Port_Pin = GPIO_Pin_2},
};

PORT ARMING_PORT[2] = {
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_0, .Delay_Time = 0},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_1, .Delay_Time = 0},												
											};

PORT OUT[8] = {
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_8,.Pin_Value = 0x00},{.Port = GPIOA,.Port_Pin = GPIO_Pin_11,.Pin_Value = 0x00},
								{.Port = GPIOA,.Port_Pin = GPIO_Pin_12,.Pin_Value = 0x00},{.Port = GPIOA,.Port_Pin = GPIO_Pin_15,.Pin_Value = 0x00},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_10,.Pin_Value = 0x00},{.Port = GPIOC,.Port_Pin = GPIO_Pin_11,.Pin_Value = 0x00},
								{.Port = GPIOC,.Port_Pin = GPIO_Pin_12,.Pin_Value = 0x00},{.Port = GPIOD,.Port_Pin = GPIO_Pin_2,.Pin_Value = 0x00},
							};

uint16_t Input_Status_EEPROM;
uint8_t Output_Status_EEPROM;
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void SYSRESET(void)  
{
	disconnect(0);
	close(0);
	__disable_fault_irq();
	NVIC_SystemReset();
}


void W5500_RESET(SOCKET s)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);//��λ��������
	delay_ms(50);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);//��λ��������
	delay_ms(200);
	W5500_Status = W5500_begin;
	//�ؼ��Ĵ��������PHY��λ
	setIR(0xF0);
	setSIR(W5500_int_reg.SIR_REG);
	setSn_IR(s,  W5500_int_reg.SnSIR_REG&0x0F);//send�������0x10;
	wizphy_reset();
	
//	while((getPHYCFGR()&0x01) != PHYCFGR_LNK_ON);//�ȴ���̫���������
}

void data_init(void)
{
	memset(&TIM_Count, 0, sizeof(TIM_Count));
	memset(&Current_Date, 0, sizeof(Current_Date));
	memset(&SYS_Flags, false, sizeof(Current_Date));
	memset(GPRS_RCV_Buf, '\0', sizeof(GPRS_RCV_Buf));
	W5500_Status = NULL;
	GPRS_STATUS = NULL;
	GPRS_SEND_STATUS = NULL;
}

void platform_init(SOCKET s)
{
	
	SystemInit();//ϵͳʱ�ӳ�ʼ��
	USART_int(115200);//����1��ʼ��
	W5500_GPIO_Configuration();
	SPI_Configuration();//Config SPI
	TIMER_init();
	IWDG_Config();
	NVIC_init();
	__enable_irq() ;
	data_init();
	EEPROM_check();
	PORTInit();
	W5500_init(s);
//	GPRS_HARDWARE_INIT();
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
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//USART_Mode_Tx;//??????????
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
	
	
//	USART_SendData(USART1,USART_ReceiveData(USART1));
#ifdef DEBUG	
	USART_SendData(USART3,USART_ReceiveData(USART1));
	if(USART_ReceiveData(USART1) == '[')
	{SYSRESET();}
	if(USART_ReceiveData(USART1) == ']')
	{W5500_Status = W5500_begin;}
	if(USART_ReceiveData(USART1) == '/')
	{EEPROM_WriteByte(STATUS_add, 0x55 );}
  USART_ClearFlag(USART1, USART_FLAG_RXNE);
#endif
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
#ifdef DEBUG
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
#endif
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
/* FILE is typedef? d in stdio.h. */ 
FILE __stdout;


/**********************************************************************************END OF USART************************************************************************************/

void delay_us(u32 n)
{
	u8 j;
	while(n--)
	for(j=0;j<10;j++);
}
void delay_ms(u32 n)
{
	while(n--)
	delay_us(1000);
}

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
{
	delay_us(30);
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
* ������  : EEPROM_GetData
* ����    : ������ȡ�ֽڴ������
* ����    : _Address��ַ��*buff���飬len����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/

void EEPROM_GetData(uint8_t _Address, uint8_t  *buff, uint8_t len)
{
	uint8_t cnt;
	for(cnt = 0; cnt < len; cnt++)
		{
			buff[cnt] = EEPROM_ReadByte(_Address + cnt);
		}
}

/*******************************************************************************
* ������  : INdata2EEPROM
* ����    : �������ݱ���
* ����    : _Address��ַ��*buff���飬len����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/

bool INdata2EEPROM(uint8_t _ucRegAddr, uint8_t *_ucRegData, uint8_t len) //�������ݱ���
{
	uint8_t cnt;
	for(cnt = 0; cnt < len; cnt++)
		{
			if(EEPROM_ReadByte( _ucRegAddr + cnt) != _ucRegData[cnt])
				{
					EEPROM_WriteByte(_ucRegAddr + cnt, _ucRegData[cnt]);
					delay_ms(3);
				}
			else{;}
		}
	for(cnt = 0; cnt < len; cnt++)
		{
			if(EEPROM_ReadByte( _ucRegAddr + cnt) != _ucRegData[cnt])
				{
					printf("save indata failed...\r\n");
					return false;
				}
			else{;}
		}
	printf("save indata OK...\r\n");
	return true;
}
/*******************************************************************************
* ������  : PortData2EEProm
* ����    : �������ݱ���
* ����    : _Address��ַ��*buff���飬len����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
bool PortData2EEProm(uint8_t _ucRegAddr, uint8_t _ucRegData) //�¶˿����ݴ���Ĵ���
{
	if(Input_Status_ARMING == _ucRegData)//�ж��Ƿ���Ҫ�洢
	{;}
		else 
		{
			EEPROM_WriteByte(_ucRegAddr, _ucRegData);
			delay_ms(3);
			if(EEPROM_ReadByte(_ucRegAddr) == _ucRegData)
			{
				printf("save portdata to eeprom OK.....\r\n");
			}
			else
			{
				printf("save portdata to eeprom FAILED....\r\n");
				return false;
			}
		}
	return true;
}

/*******************************************************************************
* ������  : EEPROM_check
* ����    : �ؼ�������֤��mac��localip��serverip....
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void EEPROM_check(void)
{
	unsigned char cnt,tmp,buff[4];
	InitI2C();
	if(i2c_CheckDevice(EEPROM_ADDRESS)== 0)
	{
		tmp = EEPROM_ReadByte(STATUS_add);
		if((tmp & 0x05) == 0x05) //mac already
		{
			printf(" MAC already in EEPROM......\r\n");
			for(cnt = 0; cnt < 3; cnt++)
					{
						gWIZNETINFO.mac[3 + cnt] = EEPROM_ReadByte(cnt + MAC_reg_add);
						delay_ms(3);
					}
		}
		else{;}
		if((tmp & 0x0A) == 0x0A) //mac rand
		{
			printf(" MAC random & store in EEPROM......\r\n");
			MAC_rand();
			for(cnt = 0; cnt < 3; cnt++)
			{
				EEPROM_WriteByte(MAC_reg_add + cnt, gWIZNETINFO.mac[3 + cnt]);
				delay_ms(3);
			}
			EEPROM_WriteByte(STATUS_add, 0x05);
		}
		else{;}
		if((tmp & 0x50) == 0x50) //DHCP
		{
			ETH_STATUS = DHCP;
			printf("ETH = DHCP...\r\n");
		}
		else{;}
		if((tmp & 0xA0) == 0xA0) // static IP
		{
			ETH_STATUS = Static_IP;
			printf("ETH = Static IP...\r\n");
			EEPROM_GetData(Local_IP_add, gWIZNETINFO.ip,4);
			EEPROM_GetData(Local_GateWay_add, gWIZNETINFO.gw,4);
			EEPROM_GetData(Local_SubNet_add, gWIZNETINFO.sn,4);
			EEPROM_GetData(Dest_IP_add, DstIP,4);
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
		else{ETH_STATUS = DHCP;}
//		ETH_STATUS = DHCP;
//		printf("mac : %02X.%02X.%02X\r\n", gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
		INdata2EEPROM(HardWare_Version_add, HardWareVersionNum, 4);
		INdata2EEPROM(SoftWare_Version_add, SoftWareVersionNum, 4);
		EEPROM_GetData(HardWare_Version_add, buff, 4);
		printf("HardWare_Version: %d.%d.%02d.%02d\r\n", buff[0],buff[1],buff[2],buff[3]);
		EEPROM_GetData(SoftWare_Version_add, buff, 4);
		printf("SoftWare_Version: %d.%d.%02d.%02d\r\n", buff[0],buff[1],buff[2],buff[3]);
		EEPROM_GetData(DisArming_Time_add, buff, 2);
		ARMING_PORT[DisArmingPort_Num].Delay_Time = (((uint16_t)buff[0]) << 8) | (uint16_t)buff[1];
		EEPROM_GetData(Arming_Time_add, buff, 2);
		ARMING_PORT[ArmingPort_Num].Delay_Time = (((uint16_t)buff[0]) << 8) | (uint16_t)buff[1];
		printf("ARMING_DELAY=%d,DISARMING_DELAY=%d\r\n",ARMING_PORT[1].Delay_Time,ARMING_PORT[0].Delay_Time);
//		EEPROM_GetData(Dest_IP_add, DstIP,4);
//		EEPROM_GetData(Local_Port_add, buff,2);
//		LocalPort = (((uint16_t )buff[0]) << 8) | ((uint16_t )buff[1]);
//		EEPROM_GetData(Dst_Port_add, buff,2);
//		DstPort = (((uint16_t )buff[0]) << 8) | ((uint16_t )buff[1]);
//		printf("DstIP : %d.%d.%d.%d\r\n", DstIP[0],DstIP[1],DstIP[2],DstIP[3]);
//		printf("LocalPort : %d\r\n", LocalPort);
//		printf("DstPort : %d\r\n", DstPort);
	}
	else {printf("got nothings on iic bus\r\n");SYSRESET();}
}

uint32_t Flash_Read(uint32_t iAddress)
{

		printf("0x%02X\r\n",*( uint32_t*) iAddress);

	return *( uint32_t*) iAddress;
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
		if(SYS_Flags.Count_Flag)//�������˿ڵ�ƽ����ʱ��
		{
			TIM_Count.Arming_Count++;
		}
		else {TIM_Count.Arming_Count = 0;}
		
		//sec min hour
		if(TIM_Count.mSec_Count >= 1000)
		{
			Current_Date.Second ++;
			TIM_Count.mSec_Count = 0;
			
						//���ӳ�ʱ����
			if(time_count_flag)
			{
				time_count_flag = false;
				time_out_count ++;
			}
			else {time_out_count = 0;}
			
			TIM_Count.ticks ++;
			
			if(Current_Date.Second % 10 == 0)
			{
				SYS_Flags.KeepAlive_Flag = true;
			}
			else {;}
			if(SYS_Flags.Wait_KeepAlive_Back_Flag)
			{TIM_Count.Wait_KeepAlive_Back_Count++;}
			else {TIM_Count.Wait_KeepAlive_Back_Count = 0;}
				
			if(SYS_Flags.DHCP_Count_Flag) //dhcp ���ͳ�ʱ
			{TIM_Count.DHCP_TimeOut++;}
			else {TIM_Count.DHCP_TimeOut = 0;}
			
			if(Alarm2Server[Alarm_Arming].Alarm_Count_Flag)//�ȴ�ȷ�ϲ�������ʱ��
			{Alarm2Server[Alarm_Arming].Alarm_Count ++;}
			else {Alarm2Server[Alarm_Arming].Alarm_Count = 0;}
			
			if(Alarm2Server[Alarm_DisArming].Alarm_Count_Flag)//�ȴ�ȷ�ϳ�������ʱ��
			{Alarm2Server[Alarm_DisArming].Alarm_Count ++;}
			else {Alarm2Server[Alarm_DisArming].Alarm_Count = 0;}
			
			if(Alarm2Server[Alarm_InPutMismatch].Alarm_Count_Flag)//�ȴ�ȷ�϶˿�ʧ�侯��ʱ��
			{Alarm2Server[Alarm_InPutMismatch].Alarm_Count ++;}
			else {Alarm2Server[Alarm_InPutMismatch].Alarm_Count = 0;}
			
			if(Current_Date.Second >= 60)
			{
				Current_Date.Second = 0;
				Current_Date.Minute ++;
				if(SYS_Flags.DHCP_ReCount_Flag) // DHCP��ȡ��ʱ
				{TIM_Count.DHCP_ReConnect_Count++;}
				else {TIM_Count.DHCP_ReConnect_Count = 0;}

				if(Current_Date.Minute >= 60)
				{
					Current_Date.Minute = 0;
					Current_Date.Hour ++;
				}
				else{;}

				if(Current_Date.Hour >= 24)
				{
					Current_Date.Hour = 0;
					Current_Date.Day ++;
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
		
//		if(TIM_Count.WDG_Count < 5000)//<5s set WDG flag
//		{TIM_Count.WDG_Count++;}
//		else {
//						TIM_Count.WDG_Count = 0;
//						TIM_Count.WDG_Count_Back ++;
//					}
//		if(TIM_Count.WDG_Count_Back > 3) 
//		{
//			IWDG_ReloadCounter();
//			SYS_Flags.WDG_Flag = false;
//			TIM_Count.WDG_Count_Back = 0;
//		}
//		else{;}
					
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
//	EXTI_InitTypeDef  EXTI_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* W5500_RST���ų�ʼ������(PA3) */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	
//	/* W5500_INT���ų�ʼ������(PA2) */	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//		
//	/* Connect EXTI Line2 to PA2 */
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

//	/* PA2 as W5500 interrupt input */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
}

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	#ifdef  VECT_TAB_RAM  
    // Set the Vector Table base location at 0x20000000 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
    // Set the Vector Table base location at 0x08000000 
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x7000);   
    #endif
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the EXTI2 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//void EXTI2_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
//	{
//		SYS_Flags.W5500_Interrupt=true;
//		W5500_int_reg.IR_REG = getIR();
//		W5500_int_reg.SIR_REG = getSIR();
//		W5500_int_reg.SnSIR_REG = getSn_IR(0);
//		W5500_int_reg.SnSR_REG = getSn_SR(0);	
//		EXTI_ClearITPendingBit(EXTI_Line2);
//	}
//}
/**************************************************************************END OF EXTI2&TIM&WTD&NVIC************************************************************************************/

void PORTInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t count;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	RCC_APB2PeriphClockCmd(INPORT_RCC_Periph | OUTPORT_RCC_Periph, ENABLE); 	/* ??GPIO?? */
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		/* ????? */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO????? */
	for(count = 0; count < InputPinNum; count ++)
	{
		GPIO_InitStructure.GPIO_Pin = IN[count].Port_Pin;
		GPIO_Init(IN[count].Port, &GPIO_InitStructure);
	}
	for(count = 0; count < ArmingPinNum; count ++)
	{
		GPIO_InitStructure.GPIO_Pin = ARMING_PORT[count].Port_Pin;
		GPIO_Init(ARMING_PORT[count].Port, &GPIO_InitStructure);
	}
	
	Output_Status_EEPROM = EEPROM_ReadByte(OUT_reg_add);
	Input_Status_ARMING = EEPROM_ReadByte(IN_ARMING_reg_add);
	Input_Status_ARMING = (Input_Status_ARMING << 8) |EEPROM_ReadByte(IN_ARMING_reg_add+1);
	Input_Status = Input_Status_ARMING;
	Input_Status_Confirm = Input_Status_ARMING;
	Output_Status = Output_Status_EEPROM;
	Arming_Status = Arming;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		/* ????? */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO????? */
	for(count = 0; count < OutputPinNum; count ++)
	{
		GPIO_InitStructure.GPIO_Pin = OUT[count].Port_Pin;
		GPIO_Init(OUT[count].Port, &GPIO_InitStructure);
	//	GPIO_ResetBits(OUT[count].Port,OUT[count].Port_Pin);//test code
		OUT[count].Pin_Value = (Output_Status >> count) & 0x01;
			if(OUT[count].Pin_Value)
				{
					GPIO_SetBits(OUT[count].Port,OUT[count].Port_Pin);
				}
			else {GPIO_ResetBits(OUT[count].Port,OUT[count].Port_Pin);}
	}

	printf("OUT init status:0x%02X\r\n",Output_Status);
	printf("IN_ARMING init status:0x%04X\r\n",Input_Status_ARMING);
//	for(count = 0; count < OutputPinNum; count ++)
//		{
//			OUT[count].Pin_Value = (Output_Status >> count) & 0x01;
//			if(OUT[count].Pin_Value)
//				{
//					GPIO_SetBits(OUT[count].Port,OUT[count].Port_Pin);
//				}
//			else {GPIO_ResetBits(OUT[count].Port,OUT[count].Port_Pin);}
//		}
		//Ĭ����������
	SYS_Flags.NET_Arming_Flag = true;
	SYS_Flags.Arming_Flag = true;
	Arming_Status = Arming;
}
/*IAP*/
uint8_t sys_upgradeflag = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;  //Ӧ�ó����ַָ��

void jump(void)
{
	uint32_t ApplicationAddress=0;
	if(0x01==sys_upgradeflag)
	{
		ApplicationAddress = ApplicationAddress0;
		printf("Execute IAP Program\r\n");
							USART_DeInit(USART1);
			TIM_DeInit( TIM2);
		RCC_DeInit();
		__disable_irq() ;
		#ifdef  VECT_TAB_RAM  
    // Set the Vector Table base location at 0x20000000 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
    // Set the Vector Table base location at 0x08000000 
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
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
		
		__set_MSP(*(volatile uint32_t*)ApplicationAddress);    //��ʼ���û�����Ķ�ջָ�� 
		
		Jump_To_Application();
	}
}


/*********************************END OF FILE**********************************/
