/*******************************************************************************
 * ��  ��:  main.c
 *
 * ��  Ҫ:  ��ؿ������Ŀ��Ƴ���'���ļ�'
 *
 * ��  ��:  2019-10-29
 *
 * ��  ��:  ST-Link
 *
 * ��汾:  ST3.5.0
 *****************************************************************************/

/*================================== �ļ����� ================================*/

#include "stm32f10x.h"
#include "target.h"
#include "comd_proc.h"
#include "hc595.h"

/*================================== ��ض��� ================================*/

/*----------------------------------------------------------------------------**
 * ��������
 *----------------------------------------------------------------------------*/

/* SysTick ��� */
unsigned int  guwTickCnt  = 0;
unsigned char gucTickFlag = 0;

/* USART1 ��� */
struct _usart_device gtUsart1Dev = {0};

/* USART3 ��� */
unsigned char gucUart3RxBuf[68] = {0};
unsigned char gucUart3RxLen = 0;
unsigned char gucUart3RxFlag = 0;

/* TIM6 ��� */
volatile unsigned int  gvuw_tim6_cnt; // ��ʱ�� 6 ������1
volatile unsigned char gvuc_tim6_flag = 0;


/*----------------------------------------------------------------------------**
 * ���غ���ԭ��
 *----------------------------------------------------------------------------*/

void NVIC_Configure(void);					// NVIC ��ʼ��
void GPIO_Configure(void);					// GPIO ��ʼ��
void SysTick_Configure(void);				// SysTick ��ʼ��
void USART1_Configure(unsigned int baud);	// USART1 ��ʼ��
void USART1_NVIC_Init(void);				// USART1 NVIC �ж�����

void USART3_Configure(unsigned int baud);	// USART3 ��ʼ��
void USART3_NVIC_Init(void);				// USART3 NVIC �ж�����

void TIM6_Configure(void);					// TIMER6 ��ʼ��
void TIM6_NVIC_Init(void);					// TIMER6 NVIC �ж�����

/*================================== ������� ================================*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Printf �ӿں���
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: fputc
 * ��  ��: �ض���c�⺯��printf��USART4
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

int fputc(int ch, FILE *f)
{
	/* �� Printf ���ݷ�������1 */
	USART_SendData(USART1, (unsigned char)ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);

	return (ch);
}

/*----------------------------------------------------------------------------**
 * ��  ��: mcu_restart
 * ��  ��: CPU �����λ
 * ˵  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void mcu_restart(void)
{
	__set_FAULTMASK(1); // �ر������ж�
	NVIC_SystemReset(); // ��λ
}


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Target Ŀ���
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: target_initial
 * ��  ��: Ŀ����ʼ��
 * ˵  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void target_initial(void)
{
	/* GPIO ��ʼ�� */
	SysTick_Configure();
	GPIO_Configure();
	NVIC_Configure();
	USART1_Configure(115200);
	USART3_Configure(115200);
	TIM6_Configure();
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * NVIC
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: NVIC_Configure
 * ��  Ҫ: ϵͳ�жϹ���
 * ��  ��: ��
 * ��  ��: ��
 *
 * ˵  ��: 1. ʹ�� NVIC �ж�;
 *----------------------------------------------------------------------------*/

void NVIC_Configure(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				// �����ж���Ϊ 2

	/* Enable the EXTI0_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;			// �ж���Ϊ 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// �������ȼ�Ϊ 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			// ��Ӧ���ȼ�Ϊ 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// ���� SysTick_IRQn �ж�
	NVIC_Init(&NVIC_InitStructure);

	USART1_NVIC_Init();
	USART3_NVIC_Init();
	TIM6_NVIC_Init();
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * GPIO
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: GPIO_Configure
 * ��  Ҫ: GPIO ���ó�ʼ��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * SysTick
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: SysTick_Configure
 * ��  ��: SysTick ��ʼ��
 * ˵  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void SysTick_Configure(void)
{
	// SystemCoreClock / 1000    // 1ms		�ж�Ƶ��
	// SystemCoreClock / 100000	 // 10us	�ж�Ƶ��
	// SystemCoreClock / 1000000 // 1us 	�ж�Ƶ��
	while (SysTick_Config( SystemCoreClock  / 1000));	//Systick ����Ϊ 1ms �ж�
}

/*----------------------------------------------------------------------------**
 * ��  ��: systick_callback
 * ��  ��: LED ��˸
 * ˵  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void systick_callback(void)
{
	static unsigned char flag = 0;

	guwTickCnt++;
	if ((guwTickCnt % 500) == 0)
	{
		flag++;
		((flag & 0x01) ? (PCout(13) = 0) : (PCout(13) = 1));
	}
	gucTickFlag = 0x55;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * TIM6
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: TIM6_Configure
 * ��  ��: TIM6 ����.
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void TIM6_Configure(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// ������ʱ��ʱ��,���ڲ�ʱ�� CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); ///ʹ�� TIM6 ʱ��

	// �Զ���װ�ؼĴ����ܵ�ֵ(����ֵ)
	TIM_TimeBaseStructure.TIM_Period = 10000;

	// �ۼ� TIM_Period ��Ƶ�ʺ����һ�����»����ж�
	// ʱ��Ԥ��Ƶ��Ϊ 71����������������ʱ�� CK_CNT = CK_INT / (71+1)=1M
	TIM_TimeBaseStructure.TIM_Prescaler= 71;

	// ��ʼ����ʱ��
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	// ����������жϱ�־λ
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);

	// �����������ж�
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	// ʹ�ܼ�����
	TIM_Cmd(TIM6, ENABLE);

	// ��ʱ�رն�ʱ����ʱ�ӣ��ȴ�ʹ��
	//BASIC_TIM_APBxClock_FUN(TIM6, DISABLE);
}

// �ж����ȼ�����
/*----------------------------------------------------------------------------**
 * ��  ��: TIM6_NVIC_Init
 * ��  ��: TIM6 �ж�����.
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void TIM6_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

	// �����ж���Ϊ 0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;
	// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// ������ռ���ȼ�Ϊ 3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * ��  ��: tmr6_int_update_callback
 * ��  ��: ��ʱ��6 Update �жϻص�����
 * ˵  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void tmr6_int_update_callback(void)
{
	gvuw_tim6_cnt--;
	if (gvuw_tim6_cnt == 0)
	{
		gvuc_tim6_flag = 0x55;
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: tmr6_delay
 * ��  ��: ʹ�ö�ʱ��6 ������ʱ
 * ˵  ��: value - ��ʱ��ʱ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void tmr6_delay(int value)
{
	gvuw_tim6_cnt  = value;
	gvuc_tim6_flag = 0;

	while (gvuc_tim6_flag == 0);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * USART1
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: USART1_Configure
 * ��  ��: USART1 GPIO ����,����ģʽ����.
 * ��  ��: baud - ������
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void USART1_Configure(unsigned int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ���ô���1 ��USART1�� ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� GPIO �˿�����*/
	/* ���ô���1 USART1 Tx (PA.09)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���ô���1 USART1 Rx (PA.10) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ����1����ģʽ��USART1 mode������ */
	//USART_InitStructure.USART_BaudRate = 9600;	//һ������Ϊ9600;
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);	// ʹ�ܴ���

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*----------------------------------------------------------------------------**
 * ��  ��: USART1_NVIC_Init
 * ��  ��: ����1 NVIC�жϳ�ʼ��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void USART1_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		// �����ж���Ϊ 2

	/* Enable the USART1_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	// USART1 IRQ.
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_rcv_process
 * ��  ��: ����1 ���մ�����
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_rcv_process(void)
{
	struct _usart_device *p_uart1 = &gtUsart1Dev;

	if (p_uart1->rx_ok == 0x55)
	{
		command_handler((char *)(p_uart1->rx_buf), p_uart1->rx_len);
		p_uart1->rx_ok  = 0x00;
		p_uart1->rx_len = 0x00;
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_send_char
 * ��  ��: ����1 ���ݷ��ͺ���
 * ˵  ��: ch - ��Ҫ���͵��ַ�����
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_send_char(unsigned char ch)
{
	USART_SendData(USART1, ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_send_data
 * ��  ��: ����1 ���ݷ��ͺ���
 * ˵  ��: buf - �洢��Ҫ���͵����ݵĻ�����
 *         len - ��Ҫ���͵����ݳ���
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_send_data(unsigned char *buf, unsigned char len)
{
	unsigned char i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART1, buf[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_send_string
 * ��  ��: ����1 �ַ������ͺ���
 * ˵  ��: str - ��Ҫ���͵��ַ���
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_send_string(char *str)
{
	while (*str != '\0')
	{
		USART_SendData(USART1, *str);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		str++;
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_rcv_int_callback
 * ��  ��: ����1 �����жϻص�����
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_rcv_int_callback()
{
	unsigned char r_dat;
	struct _usart_device *p_uart1 = &gtUsart1Dev;

	if (p_uart1->rx_idx > USART1_RCV_MEM_SIZE) // ��ֹ���ݹ���, �����ڴ����
	{
		p_uart1->rx_idx  = 0;
		p_uart1->rx_flag = 0;
	}

	r_dat = USART1->DR;

	if (p_uart1->rx_flag == 0)
	{
		p_uart1->rx_buf[p_uart1->rx_idx] = r_dat;
		p_uart1->rx_idx++;
		if (r_dat == '\r')
		{
			p_uart1->rx_flag = 1;
		}
	}
	else if (p_uart1->rx_flag == 1)
	{
		p_uart1->rx_buf[p_uart1->rx_idx] = r_dat;
		p_uart1->rx_idx++;
		if (r_dat == '\n')
		{
			p_uart1->rx_len  = p_uart1->rx_idx;
			p_uart1->rx_idx  = 0;
			p_uart1->rx_flag = 0x00;
			p_uart1->rx_ok   = 0x55;
		}
		else
		{
			p_uart1->rx_idx  = 0;
			p_uart1->rx_flag = 0;
		}
	}

	USART_ClearFlag(USART1, USART_FLAG_RXNE);
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart1_tmx_int_callback
 * ��  ��: ����1 �����жϻص�����
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart1_tmx_int_callback(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * USART3
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * ��  ��: USART3_Configure
 * ��  ��: ����3 GPIO ����,����ģʽ����.
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void USART3_Configure(unsigned int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// USART3_TX:  PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				// PB10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// �����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// ��ʼ�� GPIOB.10

	// USART3_RX:  PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				// PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// ��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// ��ʼ�� GPIOB.11

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// ��Ӳ������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// ���պͷ���
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ��У��
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 1λֹͣλ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 8λ����λ
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);													// ʹ�ܴ���3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);								// ʹ�ܽ����ж�
}

/*----------------------------------------------------------------------------**
 * ��  ��: USART3_NVIC_Init
 * ��  ��: ����3 NVIC�жϳ�ʼ��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void USART3_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				// �����ж���Ϊ 1

	/* Enable the USART3_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; // USART3 IRQ.
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart3_rcv_process
 * ��  ��: ����3 ���մ�����
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart3_rcv_process(void)
{
	unsigned char i;

	if (gucUart3RxFlag)
	{
		for (i = 0; i < gucUart3RxLen; i++)
		{
			printf("rx3=%x\r\n", gucUart3RxBuf[i]);
			usart3_send_char(gucUart3RxBuf[i]);
		}
		gucUart3RxLen = 0;
		gucUart3RxFlag = 0;
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart3_send_char
 * ��  ��: ����3 ���ݷ��ͺ���
 * ˵  ��: ch - ��Ҫ���͵��ַ�����
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart3_send_char(unsigned char ch)
{
	USART_SendData(USART3, ch);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart3_send_data
 * ��  ��: ����3 ���ݷ��ͺ���
 * ˵  ��: buf - �洢��Ҫ���͵����ݵĻ�����
 *         len - ��Ҫ���͵����ݳ���
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart3_send_data(unsigned char *buf, unsigned char len)
{
	unsigned char i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART3, buf[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart3_send_string
 * ��  ��: ����3 �ַ������ͺ���
 * ˵  ��: str - ��Ҫ���͵��ַ���
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart3_send_string(char *str)
{
	while (*str != '\0')
	{
		USART_SendData(USART3, *str);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		str++;
	}
}

/*----------------------------------------------------------------------------**
 * ��  ��: usart3_rcv_int_callback
 * ��  ��: ����3 �����жϻص�����
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void usart3_rcv_int_callback(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)	//�����ж�
	{
		if (gucUart3RxLen >= 64) // ��ֹ���ݹ���, �����ڴ����
		{
			gucUart3RxLen = 0;
		}
		gucUart3RxBuf[gucUart3RxLen] = USART3->DR;
		gucUart3RxLen++;
		gucUart3RxFlag = 0x55;
		USART_ClearFlag(USART3, USART_FLAG_RXNE);
	}
}

/*******************************************************************************
 *                              �ļ�����
 ******************************************************************************/
