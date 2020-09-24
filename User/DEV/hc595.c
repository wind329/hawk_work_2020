/*******************************************************************************
 * 文  件:  hc595.c
 *
 * 概  要:  'HC595'的驱动程序'源文件'
 *
 * 日  期:  2020-05-28
 *
 * 调  试:  ST-Link
 *
 * 库版本:  ST3.5.0
 *****************************************************************************/

/*================================== 文件包含 ================================*/

#include "stm32f10x.h"
#include "target.h"

/*================================== 相关定义 ================================*/

/*----------------------------------------------------------------------------**
 * 宏定义
 *----------------------------------------------------------------------------*/

#define HC595_PORT1			GPIOC

#define HC595_SDAT1			GPIO_Pin_10		// serial data input, pin14
#define HC595_SCLK1			GPIO_Pin_11		// shift register clock input, pin11
#define HC595_RCLK1			GPIO_Pin_12		// storage register clock input, pin12

#define HC595_PORT2			GPIOC

#define HC595_SDAT2			GPIO_Pin_7		// serial data input, pin14
#define HC595_SCLK2			GPIO_Pin_8		// shift register clock input, pin11
#define HC595_RCLK2			GPIO_Pin_9		// storage register clock input, pin12

#define HC595_PORT3			GPIOC

#define HC595_SDAT3			GPIO_Pin_0		// serial data input, pin14
#define HC595_SCLK3			GPIO_Pin_1		// shift register clock input, pin11
#define HC595_RCLK3			GPIO_Pin_2		// storage register clock input, pin12

/*================================== 程序代码 ================================*/

/*----------------------------------------------------------------------------**
 * 函  数: hc595_hw_initial
 * 概  要: HC595 的接口硬件初始化函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_hw_initial(void)
{
	GPIO_InitTypeDef GPIO_InitS;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// 第一片595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12));

	// 第二片595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9));

	// 第三片595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2));
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_write_byte_1
 * 概  要: 第一片 HC595 的字节写入函数
 * 参  数: ch - 写入的字节
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_write_byte_1(unsigned char ch)
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		GPIO_ResetBits(HC595_PORT1, HC595_SCLK1);
		((ch & 0x01) ? GPIO_SetBits(HC595_PORT1, HC595_SDAT1) : GPIO_ResetBits(HC595_PORT1, HC595_SDAT1));
		GPIO_SetBits(HC595_PORT1, HC595_SCLK1);
		ch = ch >> 1;
	}
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_latch_1
 * 概  要: 第一片 HC595 的锁存函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_latch_1(void)
{
	GPIO_ResetBits(HC595_PORT1, HC595_RCLK1);
	GPIO_SetBits(HC595_PORT1, HC595_RCLK1);
	GPIO_ResetBits(HC595_PORT1, HC595_RCLK1);
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_write_byte_2
 * 概  要: 第二片 HC595 的字节写入函数
 * 参  数: ch - 写入的字节
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_write_byte_2(unsigned char ch)
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		GPIO_ResetBits(HC595_PORT2, HC595_SCLK2);
		((ch & 0x01) ? GPIO_SetBits(HC595_PORT2, HC595_SDAT2) : GPIO_ResetBits(HC595_PORT2, HC595_SDAT2));
		GPIO_SetBits(HC595_PORT2, HC595_SCLK2);
		ch = ch >> 1;
	}
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_latch_2
 * 概  要: 第二片 HC595 的锁存函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_latch_2(void)
{
	GPIO_ResetBits(HC595_PORT2, HC595_RCLK2);
	GPIO_SetBits(HC595_PORT2, HC595_RCLK2);
	GPIO_ResetBits(HC595_PORT2, HC595_RCLK2);
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_write_byte_3
 * 概  要: 第三片 HC595 的字节写入函数
 * 参  数: ch - 写入的字节
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_write_byte_3(unsigned char ch)
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		GPIO_ResetBits(HC595_PORT3, HC595_SCLK3);
		((ch & 0x01) ? GPIO_SetBits(HC595_PORT3, HC595_SDAT3) : GPIO_ResetBits(HC595_PORT3, HC595_SDAT3));
		GPIO_SetBits(HC595_PORT3, HC595_SCLK3);
		ch = ch >> 1;
	}
}

/*----------------------------------------------------------------------------**
 * 函  数: hc595_latch_3
 * 概  要: 第三片 HC595 的锁存函数
 * 参  数: 无
 * 返  回: 无
 *----------------------------------------------------------------------------*/

void hc595_latch_3(void)
{
	GPIO_ResetBits(HC595_PORT3, HC595_RCLK3);
	GPIO_SetBits(HC595_PORT3, HC595_RCLK3);
	GPIO_ResetBits(HC595_PORT3, HC595_RCLK3);
}

/*******************************************************************************
 *                              文件结束
 ******************************************************************************/
