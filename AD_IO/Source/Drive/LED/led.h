#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define mcp_0 PBout(4)// PB4
#define mcp_1 PBout(5)// PB5	


#define mcp_0_1()			GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define mcp_0_0()			GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define mcp_1_1()			GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define mcp_1_0()			GPIO_ResetBits(GPIOB, GPIO_Pin_5)


void LED_Init(void);//��ʼ��

		 				    
#endif
