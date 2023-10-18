/*
*********************************************************************************************************
*	                                  
*	ģ������ : AD7606����ģ��
*	�ļ����� : bsp_spi_ad7606.c
*	��    �� : V1.0
*	˵    �� : ����AD7606 ADCת���� SPI�ӿ�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2011-06-18 armfly  ����
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_spi_ad7606.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "timer.h"
#include "misc.h"

FIFO_T	g_tAD;	/* ����һ�����������������ڴ洢AD�ɼ����ݣ�������д��SD�� */
static int16_t s_volt[8];
static int16_t s_dat[8];
static int16_t s_adc_now[8];
extern float Force1[8];


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAD7606
*	����˵��: ��ʼ��AD7606 SPI����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;
	
//	
	/* AD_SPI_CS_GPIO, AD_SPI_MOSI_GPIO, AD_SPI_MISO_GPIO, AD_SPI_DETECT_GPIO 
	   and AD_SPI_SCK_GPIO Periph clock enable */
	RCC_APB2PeriphClockCmd(AD_CS_GPIO_CLK | AD_SPI_MISO_GPIO_CLK | AD_SPI_SCK_GPIO_CLK
			, ENABLE);
	
	/* AD_SPI Periph clock enable */
	RCC_APB1PeriphClockCmd(AD_SPI_CLK, ENABLE); 
	
	/* Configure AD_SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = AD_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure AD_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = AD_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(AD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = AD_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CS_GPIO_PORT, &GPIO_InitStructure);
//

	/* AD_SPI Config */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(AD_SPI, &SPI_InitStructure);
	
	SPI_Cmd(AD_SPI, ENABLE); /* AD_SPI enable */
	
	/* ����������GPIO */

	/* ʹ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(AD_RESET_GPIO_CLK | AD_CONVST_GPIO_CLK , ENABLE);

	/* ����RESET GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_RESET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RESET_GPIO_PORT, &GPIO_InitStructure);
	
	/* ����CONVST GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_CONVST_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CONVST_GPIO_PORT, &GPIO_InitStructure);
	
	/* ����Range 1��10V��  0��5V */
	GPIO_InitStructure.GPIO_Pin = AD_RANGE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RAGEE_GPIO_PORT, &GPIO_InitStructure);

	/* ����GPIO�ĳ�ʼ״̬ */
	ad7606_Reset();				/* Ӳ����λ��AD7606 */
	
	AD_CONVST_HIGH();			/* CONVST������Ϊ�ߵ�ƽ */	

	TIM4_Int_Init(49,719);	/* ����TIM4��ʱ�ж� */	
	
	GPIOA->BRR = AD_RANGE_PIN;//10V�ǣ�GPIOA->BSRR = AD_RANGE_PIN;
	
}

/*
*********************************************************************************************************
*	�� �� ��: ad7606_Reset
*	����˵��: Ӳ����λAD7606
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_Reset(void)
{
	/* AD7606�Ǹߵ�ƽ��λ��Ҫ����С����50ns */
	
	AD_RESET_LOW();
	
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	
	AD_RESET_LOW();
}


/*
*********************************************************************************************************
*	�� �� ��: ad7606_StartConv
*	����˵��: ����AD7606��ADCת��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_StartConv(void)
{
	/* �����ؿ�ʼת�����͵�ƽ����ʱ������25ns  */
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();	/* ����ִ��2�Σ��͵�ƽԼ50ns */
	
	AD_CONVST_HIGH();
}


/*
*********************************************************************************************************
*	�� �� ��: ad7606_ReadBytes
*	����˵��: ��ȡAD7606�Ĳ������
*	��    �Σ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint16_t ad7606_ReadBytes(void)
{
  uint16_t usData = 0;
  
  /* Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(AD_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  
  /* Send the byte */
  SPI_I2S_SendData(AD_SPI, 0xFFFF);

  /* Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(AD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  
  /* Get the received data */
  usData = SPI_I2S_ReceiveData(AD_SPI);

  /* Return the shifted data */
  return usData;
}

/*
*********************************************************************************************************
*	�� �� ��: ad7606_IRQSrc
*	����˵��: ��ʱ���ñ����������ڶ�ȡADת��������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_IRQSrc(void)
{
	uint8_t i;
	uint16_t usReadValue;

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	/* ��ȡ���� 
	ʾ������⣬CS�͵�ƽ����ʱ�� 35us 
	*/
	AD_CS_LOW();
	for (i = 0; i < CH_NUM; i++)
	{
		
		s_adc_now[i] = ad7606_ReadBytes();
			
		s_adc_now[i] = s_adc_now[i] * 256 + ad7606_ReadBytes(); /* ������ */	
		
	}		
	
	AD_CS_HIGH();	

	ad7606_StartConv();
}



int16_t GetAdcFormFifo(uint8_t _ch)
{
	int16_t sAdc;
	
	DISABLE_INT();	
	sAdc = s_adc_now[_ch];
	ENABLE_INT();

	return sAdc;
}


/*
*********************************************************************************************************
*	�� �� ��: AD7606_Mak
*	����˵��: ��������������(����AD7606��д����)
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_Mak(void)
{
	uint8_t i;
	int16_t adc;

	for (i = 0;i < CH_NUM; i++)
	{	
		s_dat[i] = GetAdcFormFifo(i);
		
	/* 
		32767 = 5V , ��������ֵ��ʵ�ʿ��Ը���5V��׼��ʵ��ֵ���й�ʽ���� 
		volt[i] = ((int16_t)dat[i] * 5000) / 32767;	����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼            
		volt[i] = dat[i] * 0.3051850947599719
	*/
		adc = s_dat[i];
		s_volt[i] = (adc * 5000) / 32767;//	range=10V: s_volt[i] = (adc * 10000) / 32767;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_Disp
*	����˵��: ��ʾ�����������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_Disp(void)
{
	int16_t i;	
	int16_t iTemp;

	/* ��ӡ�ɼ����� */
//	for (i = 0; i < CH_NUM; i++)
	for (i = 0; i < 8; i++)
	{                
   		iTemp = s_volt[i];	/* uV  */
		
		if (s_dat[i] < 0)
		{
			iTemp = -iTemp;
			Force1[i]=-(iTemp /1000+(iTemp%1000)/100*0.1+(iTemp%100)/10*0.01+iTemp%10*0.001);
//			printf("CH%d=%.3f",i+1,Force[i]);
//            printf(" CH%d = %6d,0x%04X (-%d.%d%d%d V) \r\n", i+1, s_dat[i], (uint16_t)s_dat[i], iTemp /1000, (iTemp%1000)/100, (iTemp%100)/10,iTemp%10);
		}
		else
		{
			Force1[i]=iTemp /1000+(iTemp%1000)/100*0.1+(iTemp%100)/10*0.01+iTemp%10*0.001;
//			printf("CH%d=%.3f",i+1,Force[i]);
//         	printf(" CH%d = %6d,0x%04X ( %d.%d%d%d V) \r\n", i+1, s_dat[i], s_dat[i] , iTemp /1000, (iTemp%1000)/100, (iTemp%100)/10,iTemp%10);                    
		}
	}
//	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n",Force[0],Force[1],Force[2],Force[3],Force[4],Force[5],Force[6],Force[7]);
}



