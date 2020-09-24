/*******************************************************************************
 * 文  件:  main.c
 *
 * 概  要:  电池控制器的控制程序'主文件'
 *
 * 日  期:  2019-10-29
 *
 * 调  试:  ST-Link
 *
 * 库版本:  ST3.5.0
 *****************************************************************************/

/*================================== 文件包含 ================================*/

#include "stm32f10x.h"
#include "target.h"
#include "comd_proc.h"
#include "hc595.h"

/*================================== 相关定义 ================================*/

/*----------------------------------------------------------------------------**
 * 变量定义
 *----------------------------------------------------------------------------*/

/* SysTick 相关 */
unsigned int  guwTickCnt  = 0;
unsigned char gucTickFlag = 0;

/* USART1 相关 */
struct _usart_device gtUsart1Dev = {0};

/* USART3 相关 */
unsigned char gucUart3RxBuf[68] = {0};
unsigned char gucUart3RxLen = 0;
unsigned char gucUart3RxFlag = 0;

/* TIM6 相关 */
volatile unsigned int  gvuw_tim6_cnt; // 定时器 6 计数器1
volatile unsigned char gvuc_tim6_flag = 0;


/*----------------------------------------------------------------------------**
 * 本地函数原型
 *----------------------------------------------------------------------------*/

void NVIC_Configure(void);					// NVIC 初始化
void GPIO_Configure(void);					// GPIO 初始化
void SysTick_Configure(void);				// SysTick 初始化
void USART1_Configure(unsigned int baud);	// USART1 初始化
void USART1_NVIC_Init(void);				// USART1 NVIC 中断配置

void USART3_Configure(unsigned int baud);	// USART3 初始化
void USART3_NVIC_Init(void);				// USART3 NVIC 中断配置

void TIM6_Configure(void);					// TIMER6 初始化
void TIM6_NVIC_Init(void);					// TIMER6 NVIC 中断配置

/*================================== 程序代码 ================================*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Printf 接口函数
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * 函  数: fputc
 * 概  述: 重定向c库函数printf到USART4
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

int fputc(int ch, FILE *f)
{
	/* 将 Printf 内容发往串口1 */
	USART_SendData(USART1, (unsigned char)ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);

	return (ch);
}

/*----------------------------------------------------------------------------**
 * 函  数: mcu_restart
 * 概  述: CPU 软件复位
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void mcu_restart(void)
{
	__set_FAULTMASK(1); // 关闭所有中断
	NVIC_SystemReset(); // 复位
}


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Target 目标板
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * 函  数: target_initial
 * 概  述: 目标板初始化
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void target_initial(void)
{
	/* GPIO 初始化 */
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
 * 函  数: NVIC_Configure
 * 概  要: 系统中断管理
 * 参  数: 无
 * 返  回: 无
 *
 * 说  明: 1. 使能 NVIC 中断;
 *----------------------------------------------------------------------------*/

void NVIC_Configure(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				// 设置中断组为 2

	/* Enable the EXTI0_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;			// 中断线为 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// 抢先优先级为 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			// 响应优先级为 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// 允许 SysTick_IRQn 中断
	NVIC_Init(&NVIC_InitStructure);

	USART1_NVIC_Init();
	USART3_NVIC_Init();
	TIM6_NVIC_Init();
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * GPIO
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * 函  数: GPIO_Configure
 * 概  要: GPIO 配置初始化
 * 参  数: 无
 * 返  回: 无
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
 * 函  数: SysTick_Configure
 * 概  述: SysTick 初始化
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void SysTick_Configure(void)
{
	// SystemCoreClock / 1000    // 1ms		中断频率
	// SystemCoreClock / 100000	 // 10us	中断频率
	// SystemCoreClock / 1000000 // 1us 	中断频率
	while (SysTick_Config( SystemCoreClock  / 1000));	//Systick 配置为 1ms 中断
}

/*----------------------------------------------------------------------------**
 * 函  数: systick_callback
 * 概  述: LED 闪烁
 * 说  明: 无
 * 返  回: 无
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
 * 函  数: TIM6_Configure
 * 概  述: TIM6 配置.
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void TIM6_Configure(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// 开启定时器时钟,即内部时钟 CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); ///使能 TIM6 时钟

	// 自动重装载寄存器周的值(计数值)
	TIM_TimeBaseStructure.TIM_Period = 10000;

	// 累计 TIM_Period 个频率后产生一个更新或者中断
	// 时钟预分频数为 71，则驱动计数器的时钟 CK_CNT = CK_INT / (71+1)=1M
	TIM_TimeBaseStructure.TIM_Prescaler= 71;

	// 初始化定时器
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	// 清除计数器中断标志位
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);

	// 开启计数器中断
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	// 使能计数器
	TIM_Cmd(TIM6, ENABLE);

	// 暂时关闭定时器的时钟，等待使用
	//BASIC_TIM_APBxClock_FUN(TIM6, DISABLE);
}

// 中断优先级配置
/*----------------------------------------------------------------------------**
 * 函  数: TIM6_NVIC_Init
 * 概  述: TIM6 中断配置.
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void TIM6_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

	// 设置中断组为 0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;
	// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// 设置抢占优先级为 3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * 函  数: tmr6_int_update_callback
 * 概  述: 定时器6 Update 中断回调函数
 * 说  明: 无
 * 返  回: 无
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
 * 函  数: tmr6_delay
 * 概  述: 使用定时器6 进行延时
 * 说  明: value - 延时的时间
 * 返  回: 无
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
 * 函  数: USART1_Configure
 * 概  述: USART1 GPIO 配置,工作模式配置.
 * 参  数: baud - 波特率
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void USART1_Configure(unsigned int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 配置串口1 （USART1） 时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* 串口 GPIO 端口配置*/
	/* 配置串口1 USART1 Tx (PA.09)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置串口1 USART1 Rx (PA.10) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 串口1工作模式（USART1 mode）配置 */
	//USART_InitStructure.USART_BaudRate = 9600;	//一般设置为9600;
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);	// 使能串口

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*----------------------------------------------------------------------------**
 * 函  数: USART1_NVIC_Init
 * 概  述: 串口1 NVIC中断初始化
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void USART1_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		// 设置中断组为 2

	/* Enable the USART1_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	// USART1 IRQ.
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * 函  数: usart1_rcv_process
 * 概  述: 串口1 接收处理函数
 * 参  数: 无
 * 返  回: 无
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
 * 函  数: usart1_send_char
 * 概  述: 串口1 数据发送函数
 * 说  明: ch - 需要发送的字符数据
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void usart1_send_char(unsigned char ch)
{
	USART_SendData(USART1, ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*----------------------------------------------------------------------------**
 * 函  数: usart1_send_data
 * 概  述: 串口1 数据发送函数
 * 说  明: buf - 存储需要发送的数据的缓冲区
 *         len - 需要发送的数据长度
 * 返  回: 无
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
 * 函  数: usart1_send_string
 * 概  述: 串口1 字符串发送函数
 * 说  明: str - 需要发送的字符串
 * 返  回: 无
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
 * 函  数: usart1_rcv_int_callback
 * 概  述: 串口1 接收中断回调函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void usart1_rcv_int_callback()
{
	unsigned char r_dat;
	struct _usart_device *p_uart1 = &gtUsart1Dev;

	if (p_uart1->rx_idx > USART1_RCV_MEM_SIZE) // 防止数据过多, 导致内存溢出
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
 * 函  数: usart1_tmx_int_callback
 * 概  述: 串口1 发送中断回调函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void usart1_tmx_int_callback(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * USART3
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------------------**
 * 函  数: USART3_Configure
 * 概  述: 串口3 GPIO 配置,工作模式配置.
 * 参  数: 无
 * 返  回: 无
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// 初始化 GPIOB.10

	// USART3_RX:  PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				// PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// 初始化 GPIOB.11

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 接收和发送
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 1位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 8位数据位
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);													// 使能串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);								// 使能接收中断
}

/*----------------------------------------------------------------------------**
 * 函  数: USART3_NVIC_Init
 * 概  述: 串口3 NVIC中断初始化
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void USART3_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				// 设置中断组为 1

	/* Enable the USART3_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; // USART3 IRQ.
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------**
 * 函  数: usart3_rcv_process
 * 概  述: 串口3 接收处理函数
 * 参  数: 无
 * 返  回: 无
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
 * 函  数: usart3_send_char
 * 概  述: 串口3 数据发送函数
 * 说  明: ch - 需要发送的字符数据
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void usart3_send_char(unsigned char ch)
{
	USART_SendData(USART3, ch);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

/*----------------------------------------------------------------------------**
 * 函  数: usart3_send_data
 * 概  述: 串口3 数据发送函数
 * 说  明: buf - 存储需要发送的数据的缓冲区
 *         len - 需要发送的数据长度
 * 返  回: 无
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
 * 函  数: usart3_send_string
 * 概  述: 串口3 字符串发送函数
 * 说  明: str - 需要发送的字符串
 * 返  回: 无
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
 * 函  数: usart3_rcv_int_callback
 * 概  述: 串口3 接收中断回调函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void usart3_rcv_int_callback(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)	//接收中断
	{
		if (gucUart3RxLen >= 64) // 防止数据过多, 导致内存溢出
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
 *                              文件结束
 ******************************************************************************/
