#include "bsp_mcp23017_2.h"
#include "bsp_mcp23017.h"
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
 
/*
		data|=(1<<n);					//第n位置1
		data&=~(1<<n);				//第n位置0
		data^=(1<<n);				//第n位取反
		(data>>n)&1;				//取第n位的值
		fn_mcp23017_setports(data);	//使IO扩展模块进行信号输出
*/

void init_iic_mcp23017_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd(MCP_RCC_I2C_2_PORT, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = MCP_I2C_2_SCL_PIN | MCP_I2C_2_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	
	GPIO_Init(MCP_GPIO_PORT_I2C_2, &GPIO_InitStructure);
 
	i2c_2_stop();
}

 
 
void i2c_2_start(void)
{
	MCP_I2C_2_SDA_1();
	MCP_I2C_2_SCL_1();
	i2c_delay();
	MCP_I2C_2_SDA_0();
	i2c_delay();
	MCP_I2C_2_SCL_0();
	i2c_delay();
}
 
 
void i2c_2_stop(void)
{
	MCP_I2C_2_SDA_0();
	MCP_I2C_2_SCL_1();
	i2c_delay();
	MCP_I2C_2_SDA_1();
}
 
void i2c_2_sendbyte(uint8_t _ucByte)
{
	uint8_t i;
 
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			MCP_I2C_2_SDA_1();
		}
		else
		{
			MCP_I2C_2_SDA_0();
		}
		i2c_delay();
		MCP_I2C_2_SCL_1();
		i2c_delay();	
		MCP_I2C_2_SCL_0();
		if (i == 7)
		{
			 MCP_I2C_2_SDA_1();
		}
		_ucByte <<= 1;
		i2c_delay();
	}
}
 
uint8_t i2c_2_readbyte(void)
{
	uint8_t i;
	uint8_t value;
 
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		MCP_I2C_2_SCL_1();
		i2c_delay();
		if (MCP_I2C_2_SDA_READ())
		{
			value++;
		}
		MCP_I2C_2_SCL_0();
		i2c_delay();
	}
	return value;
}
 
uint8_t i2c_2_waitack(void)
{
	uint8_t re;
 
	MCP_I2C_2_SDA_1();
	i2c_delay();
	MCP_I2C_2_SCL_1();
	i2c_delay();
	if (MCP_I2C_2_SDA_READ())
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	MCP_I2C_2_SCL_0();
	i2c_delay();
	return re;
}
 
void i2c_2_ack(void)
{
	MCP_I2C_2_SDA_0();
	i2c_delay();
	MCP_I2C_2_SCL_1();
	i2c_delay();
	MCP_I2C_2_SCL_0();
	i2c_delay();
	MCP_I2C_2_SDA_1();
}
 
void i2c_2_nack(void)
{
	MCP_I2C_2_SDA_1();
	i2c_delay();
	MCP_I2C_2_SCL_1();
	i2c_delay();
	MCP_I2C_2_SCL_0();
	i2c_delay();	
}
 
u8 fn_mcp23017_2_iodir(uint16_t ports)
{
	u8 ADDR_MCP23017 = 0x40;
	i2c_2_start();
	i2c_2_sendbyte(ADDR_MCP23017);
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_sendbyte(0x00);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_sendbyte(ports >> 8);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
//	i2c_sendbyte(0x01);
//	
//	if (i2c_waitack() != 0)
//	{
//		goto cmd_fail;
//	}
	
	i2c_2_sendbyte((uint8_t)ports);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_stop();
	return 0;
	
	cmd_2_fail:
	i2c_2_stop();
	return 1;
}
 
u8 fn_mcp23017_2_setports(uint16_t ports)//A high 8 bits; B low 8 bits
{
	u8 ADDR_MCP23017 = 0x40;
	i2c_2_start();
	i2c_2_sendbyte(ADDR_MCP23017);
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_sendbyte(0x12);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_sendbyte(ports >> 8);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
//	i2c_sendbyte(0x13);
//	
//	if (i2c_waitack() != 0)
//	{
//		goto cmd_fail;
//	}
	
	i2c_2_sendbyte(ports);
	
	if (i2c_2_waitack() != 0)
	{
		goto cmd_2_fail;
	}
	
	i2c_2_stop();
	return 0;
	
	cmd_2_fail:
	i2c_2_stop();
	return 1;
}


