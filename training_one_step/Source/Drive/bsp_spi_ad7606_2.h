/*
*********************************************************************************************************
*	                                  
*	ģ������ : AD7606����ģ�� 
*	�ļ����� : bsp_ad7606.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v0.1    2009-12-27 armfly  �������ļ���ST�̼���汾ΪV3.1.2
*		v1.0    2011-01-11 armfly  ST�̼���������V3.4.0�汾��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "stm32f10x_spi.h"

#ifndef __BSP_AD7606_2_H
#define __BSP_AD7606_2_H

/* ÿ������2�ֽڣ��ɼ�ͨ�� */
#define CH_NUM			8				/* �ɼ�2ͨ�� */
#define FIFO_2_SIZE		1*256*2		/* ��С��Ҫ����48K (CPU�ڲ�RAM ֻ��64K) */

/* ����AD7606��SPI GPIO */
#define AD_SPI_2                           SPI1
#define AD_SPI_CLK_2                       RCC_APB2Periph_SPI1

#define AD_SPI_SCK_2_PIN                   GPIO_Pin_5				
#define AD_SPI_SCK_2_GPIO_PORT             GPIOA						
#define AD_SPI_SCK_2_GPIO_CLK              RCC_APB2Periph_GPIOA

#define AD_SPI_MISO_2_PIN                  GPIO_Pin_6				
#define AD_SPI_MISO_2_GPIO_PORT            GPIOA						
#define AD_SPI_MISO_2_GPIO_CLK             RCC_APB2Periph_GPIOA

/*������AD7606������GPIO */
#define AD_CS_2_PIN                        GPIO_Pin_13
#define AD_CS_2_GPIO_PORT                  GPIOC				
#define AD_CS_2_GPIO_CLK                   RCC_APB2Periph_GPIOC

#define AD_RESET_2_PIN                     GPIO_Pin_14
#define AD_RESET_2_GPIO_PORT               GPIOC				
#define AD_RESET_2_GPIO_CLK                RCC_APB2Periph_GPIOC


#define AD_CONVST_2_PIN                    GPIO_Pin_15
#define AD_CONVST_2_GPIO_PORT              GPIOB		
#define AD_CONVST_2_GPIO_CLK               RCC_APB2Periph_GPIOB


#define AD_RANGE_2_PIN                     GPIO_Pin_8
#define AD_RAGEE_2_GPIO_PORT               GPIOA		
#define AD_RAGEE_2_GPIO_CLK                RCC_APB2Periph_GPIOA


#define AD_CS_2_LOW()     					AD_CS_2_GPIO_PORT->BRR = AD_CS_2_PIN
#define AD_CS_2_HIGH()     					AD_CS_2_GPIO_PORT->BSRR = AD_CS_2_PIN

#define AD_RESET_2_LOW()					AD_RESET_2_GPIO_PORT->BRR = AD_RESET_2_PIN
#define AD_RESET_2_HIGH()					AD_RESET_2_GPIO_PORT->BSRR = AD_RESET_2_PIN
	
#define AD_CONVST_2_LOW()					AD_CONVST_2_GPIO_PORT->BRR = AD_CONVST_2_PIN
#define AD_CONVST_2_HIGH()					AD_CONVST_2_GPIO_PORT->BSRR = AD_CONVST_2_PIN

#define AD_RANGE_2_5V()						AD_RANGE_2_GPIO_PORT->BRR = AD_RANGE_2_PIN
#define AD_RANGE_2_10V()					AD_RANGE_2_GPIO_PORT->BSRR = AD_RANGE_2_PIN


#define ENABLE_2_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_2_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */



/* AD���ݲɼ������� */
typedef struct
{
	uint16_t usRead;
	uint16_t usWrite;
	uint16_t usCount;
	uint16_t usBuf[FIFO_2_SIZE];
}FIFO_2_T;



/* ���ⲿ���õĺ������� */
void ad7606_2_Reset(void);
void bsp_InitAD7606_2(void);
void ad7606_2_IRQSrc(void);
//uint8_t GetAdcFormFifo(uint16_t *_usReadAdc);
int16_t GetAdcFormFifo_2(uint8_t _ch);
void AD7606_2_Mak(void);
void ad7606_2_StopRecord(void);

void AD7606_2_Disp(void);

extern FIFO_2_T  g_tAD_2;

#endif


