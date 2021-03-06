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

/*================================== 相关定义 ================================*/

#include "stm32f10x.h"
#include "target.h"
#include "hc595.h"

/*----------------------------------------------------------------------------**
 * 变量定义
 *----------------------------------------------------------------------------*/

extern unsigned char gucTickFlag;

unsigned int guwLedCount = 0;

/*----------------------------------------------------------------------------**
 * 函数原型声明
 *----------------------------------------------------------------------------*/

void delay(unsigned int count);

/*================================== 程序代码 ================================*/

/*----------------------------------------------------------------------------**
 * 函  数: led0_on
 * 概  述: LED0 点亮函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led0_on(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

/*----------------------------------------------------------------------------**
 * 函  数: led0_off
 * 概  述: LED0 点亮函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led0_off(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/*----------------------------------------------------------------------------**
 * 函  数: led1_on
 * 概  述: LED1 点亮函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led1_on(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/*----------------------------------------------------------------------------**
 * 函  数: led1_off
 * 概  述: LED1 熄灭函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led1_off(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

/*----------------------------------------------------------------------------**
 * 函  数: led2_on
 * 概  述: LED2 点亮函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led2_on(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}

/*----------------------------------------------------------------------------**
 * 函  数: led2_off
 * 概  述: LED2 熄灭函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led2_off(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/*----------------------------------------------------------------------------**
 * 函  数: led_blink
 * 概  述: LED 闪烁函数
 * 说  明: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void led_blink(void)
{
	static unsigned char flag1 = 0;

	if (gucTickFlag == 0x55)
	{
		guwLedCount++;
		if (guwLedCount == 220)
		{
			guwLedCount = 0;
			if (flag1++ & 0x01)
			{
				LED1_ON;
				LED2_OFF;
			}
			else
			{
				LED1_OFF;
				LED2_ON;
			}
		}
		gucTickFlag = 0;
	}
}

/*----------------------------------------------------------------------------**
 * 函  数: main
 * 概  述: 主函数
 * 说  明: 无
 * 返  回: int
 *----------------------------------------------------------------------------*/

int main(void)
{
	target_initial();
	printf("\r\n\r\n");
	printf("STM32F103RE Demo V0.1\r\n\r\n");
	printf("Core_Clock=%dHz\r\n\r\n", SystemCoreClock);
	printf("Waiting 1s ..... ");
	// 延时 1 秒
	tmr6_delay(100);

	printf("Done\r\n");

	while(1)
	{
		led_blink();
		usart1_rcv_process();
	}
}

/*----------------------------------------------------------------------------**
 * 函  数: delay
 * 概  述: 简单延时函数
 * 说  明: count - 延时数
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void delay(unsigned int count)
{
	int i;
	for (i=0; i<count; i++);
}

/*******************************************************************************
 *                              文件结束
 ******************************************************************************/
