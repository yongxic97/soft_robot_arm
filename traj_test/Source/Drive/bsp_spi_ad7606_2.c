/*
*********************************************************************************************************
*	                                  
*	模块名称 : AD7606驱动模块
*	文件名称 : bsp_spi_ad7606.c
*	版    本 : V1.0
*	说    明 : 驱动AD7606 ADC转换器 SPI接口
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2011-06-18 armfly  创建
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
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

FIFO_2_T	g_tAD_2;	/* 定义一个交换缓冲区，用于存储AD采集数据，并用于写入SD卡 */
static int16_t s_volt_2[8];
static int16_t s_dat_2[8];
static int16_t s_adc_now_2[8];
extern float Force_2[8];


/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 初始化AD7606 SPI口线
*	形    参：无
*	返 回 值: 无
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
	
	/* 配置其它的GPIO */

	/* 使能GPIO时钟 */
	RCC_APB2PeriphClockCmd(AD_RESET_2_GPIO_CLK | AD_CONVST_2_GPIO_CLK , ENABLE);

	/* 配置RESET GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_RESET_2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RESET_2_GPIO_PORT, &GPIO_InitStructure);
	
	/* 配置CONVST GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_CONVST_2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CONVST_2_GPIO_PORT, &GPIO_InitStructure);
	
	/* 配置Range 1：10V；  0：5V */
	GPIO_InitStructure.GPIO_Pin = AD_RANGE_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AD_RAGEE_2_GPIO_PORT, &GPIO_InitStructure);

	/* 设置GPIO的初始状态 */
	ad7606_2_Reset();				/* 硬件复位复AD7606 */
	
	TIM2_Int_Init(49,719);	/* 配置TIM4定时中断 */	
	
	AD_CONVST_2_HIGH();			/* CONVST脚设置为高电平 */	
	
	GPIOA->BRR = AD_RANGE_2_PIN;//10V是：GPIOA->BSRR = AD_RANGE_PIN;
	
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_Reset
*	功能说明: 硬件复位AD7606
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_2_Reset(void)
{
	/* AD7606是高电平复位，要求最小脉宽50ns */
	
	AD_RESET_2_LOW();
	
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	AD_RESET_2_HIGH();
	
	AD_RESET_2_LOW();
}


/*
*********************************************************************************************************
*	函 数 名: ad7606_StartConv
*	功能说明: 启动AD7606的ADC转换
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_2_StartConv(void)
{
	/* 上升沿开始转换，低电平持续时间至少25ns  */
	AD_CONVST_2_LOW();
	AD_CONVST_2_LOW();
	AD_CONVST_2_LOW();	/* 连续执行2次，低电平约50ns */
	
	AD_CONVST_2_HIGH();
}


/*
*********************************************************************************************************
*	函 数 名: ad7606_ReadBytes
*	功能说明: 读取AD7606的采样结果
*	形    参：
*	返 回 值: 无
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
*	函 数 名: ad7606_IRQSrc
*	功能说明: 定时调用本函数，用于读取AD转换器数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_2_IRQSrc(void)
{
	uint8_t i;
	uint16_t usReadValue;

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	/* 读取数据 
	示波器监测，CS低电平持续时间 35us 
	*/
	AD_CS_2_LOW();
	for (i = 0; i < CH_NUM; i++)
	{
		
		s_adc_now_2[i] = ad7606_2_ReadBytes();
			
		s_adc_now_2[i] = s_adc_now_2[i] * 256 + ad7606_2_ReadBytes(); /* 读数据 */	
		
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
*	函 数 名: AD7606_Mak
*	功能说明: 处理采样后的数据(串行AD7606读写例程)
*	形    参：无
*	返 回 值: 无
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
		32767 = 5V , 这是理论值，实际可以根据5V基准的实际值进行公式矫正 
		volt[i] = ((int16_t)dat[i] * 5000) / 32767;	计算实际电压值（近似估算的），如需准确，请进行校准            
		volt[i] = dat[i] * 0.3051850947599719
	*/
		adc = s_dat_2[i];
		s_volt_2[i] = (adc * 5000) / 32767;//	range=10V: s_volt[i] = (adc * 10000) / 32767;
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_Disp
*	功能说明: 显示采样后的数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_2_Disp(void)
{
	int16_t i;	
	int16_t iTemp;

	/* 打印采集数据 */
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



