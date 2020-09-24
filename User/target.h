/*******************************************************************************
 * ��  ��: target.h
 *
 * ��  Ҫ: Ŀ�����Ƴ����ͷ�ļ�
 *
 ******************************************************************************/

#ifndef __TARGET_H__
#define	__TARGET_H__

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * �ļ�����
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * λ������,ʵ��51���Ƶ�GPIO���ƹ���
 * ����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
 * IO�ڲ����궨��
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define BITBAND(addr, bitnum)		((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)				*((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)		MEM_ADDR(BITBAND(addr, bitnum)) 

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * IO�ڵ�ַӳ��
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define GPIOA_ODR_Addr    (GPIOA_BASE + 12) // 0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE + 12) // 0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE + 12) // 0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE + 12) // 0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE + 12) // 0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE + 12) // 0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE + 12) // 0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE + 8)  // 0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE + 8)  // 0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE + 8)  // 0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE + 8)  // 0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE + 8)  // 0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE + 8)  // 0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE + 8)  // 0x40011E08 
 
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * IO�ڲ���,ֻ�Ե�һ��IO��.
 * ȷ��n��ֵС��16! 
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr, n)  // ��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr, n)  // ���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr, n)  // ��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr, n)  // ���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // ��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // ���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // ��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // ���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // ��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // ����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // ��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // ����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // ��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // ����

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * �궨��
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define LED0_OFF	GPIO_SetBits(GPIOC, GPIO_Pin_13);
#define LED0_ON 	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

#define LED1_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define LED1_ON 	GPIO_ResetBits(GPIOB, GPIO_Pin_0);

#define LED2_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_1);
#define LED2_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_1);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * ���Ͷ���
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define USART1_RCV_MEM_SIZE		512
#define USART1_TMX_MEM_SIZE		512

typedef struct _usart_device {
	unsigned char rx_buf[USART1_RCV_MEM_SIZE];
	unsigned int  rx_idx;
	unsigned int  rx_len;
	unsigned char rx_flag;
	unsigned char rx_ok;

	unsigned char tx_buf[USART1_TMX_MEM_SIZE];
	unsigned int  tx_len;
	unsigned char tx_flag;

} st_UsartDevice;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * ����ԭ��
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void mcu_restart(void);

void target_initial(void);

void usart1_rcv_process(void);
void usart1_send_char(unsigned char ch);
void usart1_send_data(unsigned char *buf, unsigned char len);
void usart1_send_string(char *str);

void uart3_rcv_process(void);
void usart3_send_char(unsigned char ch);
void usart3_send_data(unsigned char *buf, unsigned char len);
void usart3_send_string(char *str);

void tmr6_int_update_callback(void);
void tmr6_delay(int value);

#endif /* End of __TARGET_H__ */

/*******************************************************************************
 *                              �ļ�����
 ******************************************************************************/
