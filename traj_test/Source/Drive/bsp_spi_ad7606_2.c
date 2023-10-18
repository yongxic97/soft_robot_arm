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
#include "bsp_spi_ad7606_2.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "timer.h"
#include "misc.h"

FIFO_2_T	g_tAD_2;	/* ����һ�����������������ڴ洢AD�ɼ����ݣ�������д��SD�� */
static int16_t s_volt_2[8];
static int16_t s_dat_2[8];
static int16_t s_adc_now_2[8];
extern float Force_2[8];


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAD7606
*	����˵��: ��ʼ��AD7606 SPI����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAD7606_2(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;
	
//	
	/* AD_SPI_CS_GPIO, AD_SPI_MOSI_GPIO, AD_SPI_MISO_GPIO, AD_SPI_DETECT_GPIO 
	   and AD_SPI_SCK_GPIO Periph clock enable */
	RCC_APB2PeriphClockCmd(AD_CS_2_GPIO_CLK | AD_SPI_MISO_2_GPIO_CLK | AD_SPI_SCK_2_GPIO_CLK
			, ENABLE);
	
	/* AD_SPI Periph clock enable */
	RCC_APB2PeriphClockCmd(AD_SPI_CLK_2, ENABLE); 
	
	/* Configure AD_SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = AD_SPI_SCK_2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD_SPI_SCK_2_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure AD_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = AD_SPI_MISO_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(AD_SPI_MISO_2_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = AD_CS_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CS_2_GPIO_PORT, &GPIO_InitStructure);
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
	SPI_Init(AD_SPI_2, &SPI_InitStructure);
	
	SPI_Cmd(AD_SPI_2, ENABLE); /* AD_SPI enable */
	
	/* ����������GPIO */

	/* ʹ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(AD_RESET_2_GPIO_CLK | AD_CONVST_2_GPIO_CLK , ENABLE);

	/* ����RESET GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_RESET_2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RESET_2_GPIO_PORT, &GPIO_InitStructure);
	
	/* ����CONVST GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_CONVST_2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CONVST_2_GPIO_PORT, &GPIO_InitStructure);
	
	/* ����Range 1��10V��  0��5V */
	GPIO_InitStructure.GPIO_Pin = AD_RANGE_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RAGEE_2_GPIO_PORT, &GPIO_InitStructure);

	/* ����GPIO�ĳ�ʼ״̬ */
	ad7606_2_Reset();				/* Ӳ����λ��AD7606 */
	
	TIM2_Int_Init(49,719);	/* ����TIM4��ʱ�ж� */	
	
	AD_CONVST_2_HIGH();			/* CONVST������Ϊ�ߵ�ƽ */	
	
	GPIOA->BRR = AD_RANGE_2_PIN;//10V�ǣ�GPIOA->BSRR = AD_RANGE_PIN;
	
}

/*
*********************************************************************************************************
*	�� �� ��: ad7606_Reset
*	����˵��: Ӳ����λAD7606
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_2_Reset(void)
{
	/* AD7606�Ǹߵ�ƽ��λ��Ҫ����С����50ns */
	
	AD_RESET_2_LOW();
	
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	
	AD_RESET_2_LOW();
}


/*
*********************************************************************************************************
*	�� �� ��: ad7606_StartConv
*	����˵��: ����AD7606��ADCת��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_2_StartConv(void)
{
	/* �����ؿ�ʼת�����͵�ƽ����ʱ������25ns  */
	AD_CONVST_2_LOW();
	AD_CONVST_2_LOW();
	AD_CONVST_2_LOW();	/* ����ִ��2�Σ��͵�ƽԼ50ns */
	
	AD_CONVST_2_HIGH();
}


/*
*********************************************************************************************************
*	�� �� ��: ad7606_ReadBytes
*	����˵��: ��ȡAD7606�Ĳ������
*	��    �Σ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint16_t ad7606_2_ReadBytes(void)
{
  uint16_t usData = 0;
  
  /* Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(AD_SPI_2, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  
  /* Send the byte */
  SPI_I2S_SendData(AD_SPI_2, 0xFFFF);

  /* Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(AD_SPI_2, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  
  /* Get the received data */
  usData = SPI_I2S_ReceiveData(AD_SPI_2);

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
void ad7606_2_IRQSrc(void)
{
	uint8_t i;
	uint16_t usReadValue;

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	/* ��ȡ���� 
	ʾ������⣬CS�͵�ƽ����ʱ�� 35us 
	*/
	AD_CS_2_LOW();
	for (i = 0; i < CH_NUM; i++)
	{
		
		s_adc_now_2[i] = ad7606_2_ReadBytes();
			
		s_adc_now_2[i] = s_adc_now_2[i] * 256 + ad7606_2_ReadBytes(); /* ������ */	
		
	}		
	
	AD_CS_2_HIGH();	

	ad7606_2_StartConv();
}



int16_t GetAdcFormFifo_2(uint8_t _ch)
{
	int16_t sAdc;
	
	DISABLE_2_INT();	
	sAdc = s_adc_now_2[_ch];
	ENABLE_2_INT();

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
void AD7606_2_Mak(void)
{
	uint8_t i;
	int16_t adc;

	for (i = 0;i < CH_NUM; i++)
	{	
		s_dat_2[i] = GetAdcFormFifo_2(i);
		
	/* 
		32767 = 5V , ��������ֵ��ʵ�ʿ��Ը���5V��׼��ʵ��ֵ���й�ʽ���� 
		volt[i] = ((int16_t)dat[i] * 5000) / 32767;	����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼            
		volt[i] = dat[i] * 0.3051850947599719
	*/
		adc = s_dat_2[i];
		s_volt_2[i] = (adc * 5000) / 32767;//	range=10V: s_volt[i] = (adc * 10000) / 32767;
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
void AD7606_2_Disp(void)
{
	int16_t i;	
	int16_t iTemp;

	/* ��ӡ�ɼ����� */
//	for (i = 0; i < CH_NUM; i++)
	for (i = 0; i < 8; i++)
	{                
   		iTemp = s_volt_2[i];	/* uV  */
		
		if (s_dat_2[i] < 0)
		{
			iTemp = -iTemp;
			Force_2[i]=-(iTemp /1000+(iTemp%1000)/100*0.1+(iTemp%100)/10*0.01+iTemp%10*0.001);
//			printf("CH%d=%.3f",i+1,Force[i]);
//            printf(" CH%d = %6d,0x%04X (-%d.%d%d%d V) \r\n", i+1, s_dat[i], (uint16_t)s_dat[i], iTemp /1000, (iTemp%1000)/100, (iTemp%100)/10,iTemp%10);
		}
		else
		{
			Force_2[i]=iTemp /1000+(iTemp%1000)/100*0.1+(iTemp%100)/10*0.01+iTemp%10*0.001;
//			printf("CH%d=%.3f",i+1,Force[i]);
//         	printf(" CH%d = %6d,0x%04X ( %d.%d%d%d V) \r\n", i+1, s_dat[i], s_dat[i] , iTemp /1000, (iTemp%1000)/100, (iTemp%100)/10,iTemp%10);                    
		}
	}
}



