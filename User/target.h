/*******************************************************************************
 * 文  件: target.h
 *
 * 概  要: 目标板控制程序的头文件
 *
 ******************************************************************************/

#ifndef __TARGET_H__
#define	__TARGET_H__

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * 文件包含
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * 位带操作,实现51类似的GPIO控制功能
 * 具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
 * IO口操作宏定义
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define BITBAND(addr, bitnum)		((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)				*((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)		MEM_ADDR(BITBAND(addr, bitnum)) 

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * IO口地址映射
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
 * IO口操作,只对单一的IO口.
 * 确保n的值小于16! 
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr, n)  // 输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr, n)  // 输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr, n)  // 输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr, n)  // 输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // 输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // 输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // 输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // 输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // 输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // 输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // 输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // 输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // 输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // 输入

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * 宏定义
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define LED0_OFF	GPIO_SetBits(GPIOC, GPIO_Pin_13);
#define LED0_ON 	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

#define LED1_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define LED1_ON 	GPIO_ResetBits(GPIOB, GPIO_Pin_0);

#define LED2_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_1);
#define LED2_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_1);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * 类型定义
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
 * 函数原型
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
 *                              文件结束
 ******************************************************************************/
