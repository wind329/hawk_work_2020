/*******************************************************************************
 * ��  ��:  hc595.c
 *
 * ��  Ҫ:  'HC595'����������'Դ�ļ�'
 *
 * ��  ��:  2020-05-28
 *
 * ��  ��:  ST-Link
 *
 * ��汾:  ST3.5.0
 *****************************************************************************/

/*================================== �ļ����� ================================*/

#include "stm32f10x.h"
#include "target.h"

/*================================== ��ض��� ================================*/

/*----------------------------------------------------------------------------**
 * �궨��
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

/*================================== ������� ================================*/

/*----------------------------------------------------------------------------**
 * ��  ��: hc595_hw_initial
 * ��  Ҫ: HC595 �Ľӿ�Ӳ����ʼ������
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void hc595_hw_initial(void)
{
	GPIO_InitTypeDef GPIO_InitS;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// ��һƬ595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12));

	// �ڶ�Ƭ595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9));

	// ����Ƭ595
	GPIO_InitS.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitS);

	GPIO_ResetBits(GPIOC, (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2));
}

/*----------------------------------------------------------------------------**
 * ��  ��: hc595_write_byte_1
 * ��  Ҫ: ��һƬ HC595 ���ֽ�д�뺯��
 * ��  ��: ch - д����ֽ�
 * ��  ��: ��
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
 * ��  ��: hc595_latch_1
 * ��  Ҫ: ��һƬ HC595 �����溯��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void hc595_latch_1(void)
{
	GPIO_ResetBits(HC595_PORT1, HC595_RCLK1);
	GPIO_SetBits(HC595_PORT1, HC595_RCLK1);
	GPIO_ResetBits(HC595_PORT1, HC595_RCLK1);
}

/*----------------------------------------------------------------------------**
 * ��  ��: hc595_write_byte_2
 * ��  Ҫ: �ڶ�Ƭ HC595 ���ֽ�д�뺯��
 * ��  ��: ch - д����ֽ�
 * ��  ��: ��
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
 * ��  ��: hc595_latch_2
 * ��  Ҫ: �ڶ�Ƭ HC595 �����溯��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void hc595_latch_2(void)
{
	GPIO_ResetBits(HC595_PORT2, HC595_RCLK2);
	GPIO_SetBits(HC595_PORT2, HC595_RCLK2);
	GPIO_ResetBits(HC595_PORT2, HC595_RCLK2);
}

/*----------------------------------------------------------------------------**
 * ��  ��: hc595_write_byte_3
 * ��  Ҫ: ����Ƭ HC595 ���ֽ�д�뺯��
 * ��  ��: ch - д����ֽ�
 * ��  ��: ��
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
 * ��  ��: hc595_latch_3
 * ��  Ҫ: ����Ƭ HC595 �����溯��
 * ��  ��: ��
 * ��  ��: ��
 *----------------------------------------------------------------------------*/

void hc595_latch_3(void)
{
	GPIO_ResetBits(HC595_PORT3, HC595_RCLK3);
	GPIO_SetBits(HC595_PORT3, HC595_RCLK3);
	GPIO_ResetBits(HC595_PORT3, HC595_RCLK3);
}

/*******************************************************************************
 *                              �ļ�����
 ******************************************************************************/
