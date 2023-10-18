/*
编写者：Kevin
网址：http://RobotControl.taobao.com
作者E-mail：1609370741@qq.com
编译环境：MDK-Lite  Version: 5.17
初版时间: 2016-1-31
测试： 本程序已在【君悦智控】的STM32Core平台上完成测试
功能：
用STM32Core平台串口2读取JY901的数据，然后通过串口1打印到串口助手,串口助手波特率要选为9600。
JY-61的波特率要修改为115200.
注意：示例程序输出的是ASCLL码，用16进制（HEX）显示是不能看到准确数据的。
硬件接线：
USB-TTL工具                 STM32Core               JY61
VCC          -----           VCC        ----         VCC
TX           -----           RX1（管脚10）     
RX           -----           TX1（管脚9）
GND          -----           GND        ----          GND
                             RX2 （管脚3）       ----  TX
														 TX2 （管脚2）       ----  RX
------------------------------------
 */
 
#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "UART1.h"
#include "UART2.h"
#include "UART3.h"
#include "delay.h"
#include "JY61.h"
#include "DIO.h"
#include "timer.h"
#include "key.h"
#include "adc.h"
#include "bsp_mcp23017.h"
#include "bsp_spi_ad7606.h"

struct 	SAcc 		stcAcc1;
struct 	SAcc 		stcAcc2;
struct 	SGyro 		stcGyro1;
struct 	SGyro 		stcGyro2;
struct	SAngle 		stcAngle1;
struct 	SAngle 		stcAngle2;


float yaw=0,pitch=0,roll=0;
int UART_RX_flag=0,i=0;
int mode=0,num_i=0,T_zero=0;
float Force1[8],Force_2[8];
float Vadc[16];
//float Pre_des[13]={1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5};
int Pd[13]={100,100,100,100,100,100,100,100,100,100,100,100,100};
int Td[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};//Td获取数据
int Td0[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};//Td初始数据
int Td_abs[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};//Td数据取绝对值，用于后面排序
int Td_num[13]={0,1,2,3,4,5,6,7,8,9,10,11,12};//Td数据排序调整后的序号顺位顺序
int Td_num0[13]={0,1,2,3,4,5,6,7,8,9,10,11,12};//Td获取数据时的自然排序位

float Pnow[8];
float Pre_des[8]={0,0,0,0,0,0,0,0},Pre_des_r[8];
float Pre_des_0[8]={100,100,100,100,100,100,100,100};
float Pre_flag[8],Pre_Pflag[8];
float Dead_z=0.1;
int flag=1;
int Time_lag=1100,Dead_zone=100;
int mode_clear_flag=0;
float V_Pin,V_Pout;
int number[] = {0,3,6};
int a = 0;
int b = 3;
int c = 6;

int16_t IO_Output1=0x0000,IO_Output2=0x0000;
int IO_num=0;
int Button_Flag=0,start_flag=0,start_input=0,stop_flag=0,input=0;


//用串口2给JY模块发送指令
void sendcmd1(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
		UART2_Put_Char(cmd[i]);
}

void sendcmd2(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
		UART3_Put_Char(cmd[i]);
}

//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data1(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	LED_REVERSE();					//接收到数据，LED灯闪烁一下
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc1,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro1,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle1,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//清空缓存区
	}
}

void CopeSerial2Data2(unsigned char ucData)
{
	static unsigned char ucRxBuffer2[250];
	static unsigned char ucRxCnt2 = 0;	
	
	LED_REVERSE();					//接收到数据，LED灯闪烁一下
	ucRxBuffer2[ucRxCnt2++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer2[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt2=0;
		return;
	}
	if (ucRxCnt2<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer2[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc2,&ucRxBuffer2[2],8);break;
			case 0x52:	memcpy(&stcGyro2,&ucRxBuffer2[2],8);break;
			case 0x53:	memcpy(&stcAngle2,&ucRxBuffer2[2],8);break;

		}
		ucRxCnt2=0;//清空缓存区
	}
}

void CopeSerial1Data1(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}

void CopeSerial1Data2(unsigned char ucData)
{	
	UART3_Put_Char(ucData);//转发串口1收到的数据给串口3（JY模块）
}

void reset_imu()
{
//		if(UART_RX_flag==1)
//		{
//			UART_RX_flag = 0;
//			printf("正在进行加速度校准\r\n");
//			sendcmd1(ACCCMD);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
//			printf("加速度校准完成\r\n");
//			delay_ms(100);
//			printf("进行Z轴角度清零\r\n");
//			sendcmd1(YAWCMD);
//			printf("Z轴角度清零完成\r\n");
//		}
//		if(UART_RX_flag==2)
//		{
//			UART_RX_flag = 0;
//			printf("正在进行加速度校准\r\n");
//			sendcmd2(ACCCMD);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
//			printf("加速度校准完成\r\n");
//			delay_ms(100);
//			printf("进行Z轴角度清零\r\n");
//			sendcmd2(YAWCMD);
//			printf("Z轴角度清零完成\r\n");
//		}
}

void run_mode()
{
	switch(mode)
	{
		case 0:
			Pre_des[0]=100;
			Pre_des[1]=100;
			Pre_des[2]=100;
			Pre_des[3]=100;
			Pre_des[4]=100;
			Pre_des[5]=100;
			Pre_des[6]=100;
			Pre_des[7]=100;
			break;
		case 1:
			Pre_des[0]=30;
			Pre_des[1]=170;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=30;
			Pre_des[5]=170;
			break;	
		case 2:
			Pre_des[0]=170;
			Pre_des[1]=30;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=170;
			Pre_des[5]=30;
			break;
		case 3:
			Pre_des[0]=30;
			Pre_des[1]=170;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=30;
			Pre_des[5]=170;
			break;
		case 4:
			Pre_des[0]=170;
			Pre_des[1]=30;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=170;
			Pre_des[5]=30;
			break;
		case 5:
			Pre_des[0]=30;
			Pre_des[1]=170;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=30;
			Pre_des[5]=170;
			break;	
//		case 5:
//			Pre_des[0]=40;
//			Pre_des[1]=170;
//			Pre_des[2]=170;
//			Pre_des[3]=40;
//			Pre_des[4]=40;
//			Pre_des[5]=170;
//			Pre_des[6]=75;
//			Pre_des[7]=125;
//			break;
//		case 6:
//			Pre_des[0]=40;
//			Pre_des[1]=170;
//			Pre_des[2]=170;
//			Pre_des[3]=40;
//			Pre_des[4]=40;
//			Pre_des[5]=170;
//			Pre_des[6]=125;
//			Pre_des[7]=75;
//			break;	
//		case 7:
//			Pre_des[0]=40;
//			Pre_des[1]=170;
//			Pre_des[2]=170;
//			Pre_des[3]=40;
//			Pre_des[4]=40;
//			Pre_des[5]=170;
//			Pre_des[6]=75;
//			Pre_des[7]=125;
//			break;
//		case 8:
//			Pre_des[0]=40;
//			Pre_des[1]=170;
//			Pre_des[2]=170;
//			Pre_des[3]=40;
//			Pre_des[4]=40;
//			Pre_des[5]=170;
//			Pre_des[6]=125;
//			Pre_des[7]=75;
//			break;	
//		case 9:
//			Pre_des[0]=40;
//			Pre_des[1]=170;
//			Pre_des[2]=170;
//			Pre_des[3]=40;
//			Pre_des[4]=40;
//			Pre_des[5]=170;
//			Pre_des[6]=120;
//			Pre_des[7]=130;
//			break;
		case 6:
			Pre_des[0]=170;
			Pre_des[1]=30;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=170;
			Pre_des[5]=30;
			break;
		case 7:
			Pre_des[0]=30;
			Pre_des[1]=170;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=30;
			Pre_des[5]=170;
			break;
		case 8:
			Pre_des[0]=170;
			Pre_des[1]=30;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=170;
			Pre_des[5]=30;
			break;
		case 9:
			Pre_des[0]=170;
			Pre_des[1]=170;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=30;
			Pre_des[5]=30;
			break;
		case 10:
			Pre_des[0]=30;
			Pre_des[1]=30;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=170;
			Pre_des[5]=170;
			break;
		case 11:
			Pre_des[0]=170;
			Pre_des[1]=170;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=30;
			Pre_des[5]=30;
			break;
		case 12:
			Pre_des[0]=30;
			Pre_des[1]=30;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=170;
			Pre_des[5]=170;
			break;
		case 13:
			Pre_des[0]=170;
			Pre_des[1]=170;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=30;
			Pre_des[5]=30;
			break;
		case 14:
			Pre_des[0]=30;
			Pre_des[1]=30;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=170;
			Pre_des[5]=170;
			break;
		case 15:
			Pre_des[0]=170;
			Pre_des[1]=170;
			Pre_des[2]=170;
			Pre_des[3]=30;
			Pre_des[4]=30;
			Pre_des[5]=30;
			break;
		case 16:
			Pre_des[0]=30;
			Pre_des[1]=30;
			Pre_des[2]=30;
			Pre_des[3]=170;
			Pre_des[4]=170;
			Pre_des[5]=170;
			break;	
		case 17:
			Pre_des[0]=100;
			Pre_des[1]=100;
			Pre_des[2]=100;
			Pre_des[3]=100;
			Pre_des[4]=100;
			Pre_des[5]=100;
			break;
//		case 5:
//			for(i=0;i<13;i++)
//			{
//				Pre_des[i]=Pre_des_0[i];
//			}
//			break;
	}
		
}


float V2P(float V)
{
	float Pressure;
	Pressure=(V-2.5)*50;
	return Pressure;
}

void Pre_read()
{
		AD7606_Mak();
		AD7606_Disp();
		for(i=0;i<8;i++)
			{
				Vadc[i]=Force1[i];
			}
		for(i=0;i<8;i++)
			{
				Pnow[i]=V2P(Vadc[i]);
//				printf("%.1f ",Pnow[i]);
			}
//		printf("\r\n");
}


void judge(int first)
{
	if(first==1){
		for(i=0;i<3;i++)
		{	
			if(Pnow[number[i]]-Pre_des[number[i]]>Dead_z)
			{
				//吸气
				IO_Output1|=(1<<(number[i]+8));
				IO_Output1&=~(1<<number[i]);
				Pre_Pflag[number[i]]=0;

			}else if(Pre_des[number[i]]-Pnow[number[i]]>Dead_z)
			{
				//充气
				IO_Output1&=~(1<<(number[i]+8));
				IO_Output1|=(1<<number[i]);
				Pre_Pflag[number[i]]=0;
			}else if(Pre_des[number[i]]-Pnow[number[i]]<Dead_z && Pnow[number[i]]-Pre_des[number[i]]<Dead_z)
			{
				IO_Output1&=~(1<<(number[i]+8));
				IO_Output1&=~(1<<number[i]);
				Pre_Pflag[number[i]]=1;
			}			
			fn_mcp23017_setports(IO_Output1);
		}
	}
	else{
		for(i=0;i<3;i++)
		{	
			if(Pnow[number[i]]-Pre_des[number[i]]>Dead_z+1.3)
			{
				//吸气
				IO_Output1|=(1<<(number[i]+8));
				IO_Output1&=~(1<<number[i]);
				Pre_Pflag[number[i]]=0;

			}else if(Pre_des[number[i]]-Pnow[number[i]]>Dead_z+1.3)
			{
				//充气
				IO_Output1&=~(1<<(number[i]+8));
				IO_Output1|=(1<<number[i]);
				Pre_Pflag[number[i]]=0;
			}else if(Pre_des[number[i]]-Pnow[number[i]]<Dead_z && Pnow[number[i]]-Pre_des[number[i]]<Dead_z)
			{
				IO_Output1&=~(1<<(number[i]+8));
				IO_Output1&=~(1<<number[i]);
				Pre_Pflag[number[i]]=1;
			}			
			fn_mcp23017_setports(IO_Output1);
		}

	}
}


void Text1()
{
		for(i=0;i<13;i++)
		{
			if(i<8)
			{
				if(Td[i]>0)
				{
					IO_Output1&=~(1<<(i+8));
					IO_Output1|=(1<<i);
				}
				if(Td[i]<0)
				{
					IO_Output1|=(1<<(i+8));
					IO_Output1&=~(1<<i);
				}
				fn_mcp23017_setports(IO_Output1);
				if(Td[i]>0)
				{
					delay_ms(Td[i]);
				}
				if(Td[i]<0)
				{
					delay_ms(-Td[i]);
				}
				Pre_flag[i]=1;
				IO_Output1=0x0000;
				fn_mcp23017_setports(IO_Output1);
			}else
			{
				if(Td[i]>0)
				{
					IO_Output2&=~(1<<i);
					IO_Output2|=(1<<(i-8));
				}
				if(Td[i]<0)
				{
					IO_Output2|=(1<<i);
					IO_Output2&=~(1<<(i-8));
				}	
				fn_mcp23017_2_setports(IO_Output2);
				if(Td[i]>0)
				{
					delay_ms(Td[i]);
				}
				if(Td[i]<0)
				{
					delay_ms(-Td[i]);
				}
				Pre_flag[i]=1;
				IO_Output2=0x0000;
				fn_mcp23017_2_setports(IO_Output2);
			}
		}
}



void bubbleSort(int n)
{
	int i=0,j=0,tmp=0,tmp_t=0;
	for(i =0 ; i< n-1; ++i)
	{
		Td_abs[i]=abs(Td[i]);
		Td_num[i]=Td_num0[i];
	}
	
	  for(i =0 ; i< n-1; ++i)
	  {
		for(j = 0; j < n-i-1; ++j)
		{
		  if(Td_abs[j] > Td_abs[j+1])
		  {
			tmp = Td_abs[j];
			Td_abs[j] = Td_abs[j+1];
			Td_abs[j+1] = tmp;
			
			tmp_t = Td_num[j];
			Td_num[j] = Td_num[j+1];
			Td_num[j+1] = tmp_t;
		  }
		}
	  }
  
	  for(i=0;i<13;i++)
		{
			printf("%d ",Td[i]);
		}
		printf("\r\n");
	  for(i=0;i<13;i++)
		{
			printf("%d ",Td_abs[i]);
		}
		printf("\r\n");
	  for(i=0;i<13;i++)
		{
			printf("%d ",Td_num[i]);
		}
		printf("\r\n");

}


void text_t(int n)
{
	int number=0;
	int t_old=0,t_new=0;
	
//	第一次判断并开启所有电磁阀的命令
	for(i=0;i<n-1;i++)
	{
		number=Td_num[i];
		if(number<8)
		{
			if(Td[number]>0)
			{
				IO_Output1&=~(1<<(number+8));
				IO_Output1|=(1<<number);
			}else if(Td[number]<0)
			{
				IO_Output1|=(1<<(number+8));
				IO_Output1&=~(1<<number);
			}
		}else if(number>7)
		{
			if(Td[number]>0)
			{
				IO_Output2&=~(1<<number);
				IO_Output2|=(1<<(number-8));
			}else if(Td[number]<0)
			{
				IO_Output2|=(1<<number);
				IO_Output2&=~(1<<(number-8));
			}
		}
	}
		fn_mcp23017_setports(IO_Output1);
		fn_mcp23017_2_setports(IO_Output2);
		number=0;
	
//	按排序延时
	for(i=0;i<n-1;i++)
	{
		t_new=Td_num[i];
		t_old=Td_num[i-1];
		if(i>0 && Td_abs[t_new]>Td_abs[t_old])
		{
			delay_ms(Td_abs[t_new]-Td_abs[t_old]);
			
		}else if(i==0 && Td_abs[t_new]>0)
		{
			delay_ms(Td_abs[t_new]);
		}
		
//		每次delay一次后，判断需要关哪一路的电磁阀，然后全关闭；
		if(number<8)
		{
			IO_Output1&=~(1<<(number+8));
			IO_Output1&=~(1<<number);
			fn_mcp23017_setports(IO_Output1);
//			Pre_flag[Td_num[i]]=1;
		}else{
			IO_Output2&=~(1<<number);
			IO_Output2&=~(1<<(number-8));
			fn_mcp23017_2_setports(IO_Output2);
//			Pre_flag[Td_num[i]]=1;
		}
	}
	
	IO_Output1=0x0000;
	IO_Output2=0x0000;
	fn_mcp23017_setports(IO_Output1);
	fn_mcp23017_2_setports(IO_Output2);
}


void Read_Button(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 1 && Button_Flag==0)
	{
		Button_Flag=1;
		start_input=1;
		if(start_input==1)
			{
				mode=0;
				start_flag=1;
				stop_flag=0;
			}else if(start_input==0)
			{
				stop_flag=1;
			}
			printf("button flag: %d \r\n",Button_Flag);
		
	}else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 0 && Button_Flag==1)
	{
		Button_Flag=0;
		start_input=0;
		if(start_input==1)
			{
				mode=0;
				start_flag=1;
				stop_flag=0;
			}else if(start_input==0)
			{
				stop_flag=1;
			}
			printf("button flag: %d \r\n",Button_Flag);
		
	}
	
}

void pump_k(void)
{
	float P_in, P_out;
	//0.2-2.7V 对应 -100~100kpa
	V_Pin=Get_Adc_Average(ADC_Channel_0,10)*(3.3/4095.0);//PA11
	V_Pout=Get_Adc_Average(ADC_Channel_1,10)*(3.3/4095.0);//PA15
	P_in=80*(V_Pin-1.25)+100;
	P_out=80*(V_Pout-1.25)+100;
	printf("%f,%f\r\n",P_in,P_out);
	
	if(P_in<160)
	{
		PAout(11)=1;
	}else if(P_in>170)
	{
		PAout(11)=0;
	}
	
	if(P_out>55)
	{
		PAout(12)=1;
	}else if(P_out<45)
	{
		PAout(12)=0;
	}
}


int main(void)
{  		
	u8 finished = 0; // 0 for not finished, 1 for finished close-loop control
	long delayTime = 1000;    //采取静态数据时每次需要停顿的时长（≥800ms）

	float ctrl_seq[63][3] = {
{	-0.287557443	,	-1.253807577	,	-0.183589055	},
{	-1.148760276	,	-1.439068037	,	-0.292050145	},
{	-2.367269022	,	-2.329443718	,	-0.570411231	},
{	-3.410422654	,	-3.214943491	,	-1.288339028	},
{	-4.383041834	,	-3.911985219	,	-2.090839009	},
{	-5.29548257	,	-4.389911979	,	-2.768271062	},
{	-6.079264277	,	-4.709285295	,	-3.316499064	},
{	-6.705718735	,	-4.925139052	,	-3.751631039	},
{	-7.200286025	,	-5.030218282	,	-4.06680641	},
{	-7.602342585	,	-4.98842355	,	-4.267266591	},
{	-7.940295154	,	-4.783881283	,	-4.377420712	},
{	-8.230899224	,	-4.432358438	,	-4.425376379	},
{	-8.483939895	,	-3.966002825	,	-4.429806646	},
{	-8.704958075	,	-3.415755099	,	-4.397219841	},
{	-8.897100398	,	-2.80334491	,	-4.327663015	},
{	-9.063209602	,	-2.142702688	,	-4.222306468	},
{	-9.207652324	,	-1.445144653	,	-4.08741857	},
{	-9.337019825	,	-0.723307669	,	-3.933846324	},
{	-9.459593961	,	0.008175377	,	-3.77413792	},
{	-9.58411243	,	0.734680198	,	-3.619598161	},
{	-9.718445504	,	1.444633282	,	-3.478390608	},
{	-9.868623862	,	2.132002939	,	-3.354855818	},
{	-9.307494873	,	2.267447405	,	-3.85718708	},
{	-8.801150942	,	1.549540984	,	-5.061662372	},
{	-8.700757401	,	0.037270477	,	-6.356229488	},
{	-8.884367818	,	-1.793514192	,	-7.515183815	},
{	-9.105714971	,	-3.463203426	,	-8.557238874	},
{	-9.19894118	,	-4.714834425	,	-9.448524323	},
{	-9.08992427	,	-5.483190295	,	-10.13446544	},
{	-8.76581438	,	-5.828599124	,	-10.60043646	},
{	-8.252731667	,	-5.879310811	,	-10.8675147	},
{	-7.596837083	,	-5.770109713	,	-10.96712231	},
{	-6.848379397	,	-5.593177247	,	-10.92562008	},
{	-6.05118356	,	-5.381660306	,	-10.76511417	},
{	-5.239539468	,	-5.125726253	,	-10.51018372	},
{	-4.439707422	,	-4.803099465	,	-10.191143	},
{	-3.671689862	,	-4.405025491	,	-9.84198249	},
{	-2.948856292	,	-3.947252372	,	-9.494613656	},
{	-2.276535926	,	-3.46475493	,	-9.172381037	},
{	-1.652141237	,	-2.996545595	,	-8.88647134	},
{	-1.068070259	,	-2.570431061	,	-8.63732872	},
{	-0.515747197	,	-2.194648614	,	-8.420067462	},
{	0.012951404	,	-1.856619853	,	-8.230977485	},
{	-0.199170308	,	-2.105773547	,	-7.52190387	},
{	-1.197880324	,	-3.126362905	,	-6.808945145	},
{	-2.570681987	,	-4.738439292	,	-6.464190746	},
{	-3.859058389	,	-6.567803577	,	-6.572799104	},
{	-4.749720895	,	-8.219287899	,	-6.888668625	},
{	-5.133498804	,	-9.313125083	,	-7.020653826	},
{	-5.079567896	,	-9.659449595	,	-6.722991562	},
{	-4.780386926	,	-9.384247491	,	-6.02460937	},
{	-4.436562264	,	-8.811368188	,	-5.134938691	},
{	-4.13941119	,	-8.221894804	,	-4.274768937	},
{	-3.861923994	,	-7.722387138	,	-3.554004077	},
{	-3.537943555	,	-7.276154214	,	-2.947921628	},
{	-3.130669528	,	-6.797295433	,	-2.361963269	},
{	-2.6454492	,	-6.225931093	,	-1.719035561	},
{	-2.108006263	,	-5.552839011	,	-0.999862652	},
{	-1.538257409	,	-4.799175352	,	-0.226043273	},
{	-0.937811913	,	-3.984677616	,	0.575499153	},
{	-0.294736946	,	-3.112623779	,	1.394666038	},
{	0.40431284	,	-2.173171744	,	2.24150139	},
{	1.166419214	,	-1.152026098	,	3.139494347	},

	};

	SysTick_init(72,10);//设置时钟频率
	uart_init(115200);//接PC的串口
	TIM3_Int_Init(24999,13999);//10Khz的计数频率，计数到5000为500ms 即TimeOut = （（Prescaler + 1） * （Period + 1） ） / TimeClockFren =（7199+1）*（4999+1）/72/1000 = 500ms;
	
	init_iic_mcp23017();
	fn_mcp23017_iodir(0);//0-output/1-inpu
	fn_mcp23017_setports(0x0000);
	
	bsp_InitAD7606();

	KEY_Init();
	Adc_Init();
//	LED_ON();
	delay_ms(1000);delay_ms(1000);//等等JY61初始化完成

	Pre_read();

 	while((USART_RX_STA&0X8000)==0){ // wait until target comes to the serial port
		delay_ms(10);
	}
	USART_RX_STA=0;
	Pre_read();
	printf("1 %.3f %.3f %.3f\r\n", Pnow[a],Pnow[b],Pnow[c]);

	for(int j=0;j<63;j++){
		int first = 1;
		Pre_des[a]=ctrl_seq[j][0];
		Pre_des[b]=ctrl_seq[j][1];
		Pre_des[c]=ctrl_seq[j][2];
		while(1){
			Pre_read();
			judge(first);
			first = 0;
//			printf("3\r\n");
			if(Pre_Pflag[a]==1 && Pre_Pflag[b]==1 && Pre_Pflag[c]==1){
				Pre_read();
				delay_ms(delayTime);
				printf("1 %.3f %.3f %.3f\r\n", Pnow[a],Pnow[b],Pnow[c]);
				break;
			}
		}
	}

	while(1){delay_ms(100);}

}


