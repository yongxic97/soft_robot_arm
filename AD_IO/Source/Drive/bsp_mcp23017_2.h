
#ifndef __BSP_MCP23017_2_H
#define __BSP_MCP23017_2_H
#include "stm32f10x.h"
 
#define MCP_GPIO_PORT_I2C_2			GPIOB
#define MCP_RCC_I2C_2_PORT 			RCC_APB2Periph_GPIOB
#define MCP_I2C_2_SCL_PIN			GPIO_Pin_10
#define MCP_I2C_2_SDA_PIN			GPIO_Pin_11
 
#define MCP_I2C_2_SCL_1()  GPIO_SetBits(MCP_GPIO_PORT_I2C_2, MCP_I2C_2_SCL_PIN)		/* SCL = 1 */
#define MCP_I2C_2_SCL_0()  GPIO_ResetBits(MCP_GPIO_PORT_I2C_2, MCP_I2C_2_SCL_PIN)		/* SCL = 0 */
 
#define MCP_I2C_2_SDA_1()  GPIO_SetBits(MCP_GPIO_PORT_I2C_2, MCP_I2C_2_SDA_PIN)		/* SDA = 1 */
#define MCP_I2C_2_SDA_0()  GPIO_ResetBits(MCP_GPIO_PORT_I2C_2, MCP_I2C_2_SDA_PIN)		/* SDA = 0 */
 
#define MCP_I2C_2_SDA_READ()  GPIO_ReadInputDataBit(MCP_GPIO_PORT_I2C_2, MCP_I2C_2_SDA_PIN)
 
void init_iic_mcp23017_2(void);
void i2c_2_start(void);
void i2c_2_stop(void);
void i2c_2_sendbyte(uint8_t _ucByte);
uint8_t i2c_2_readbyte(void);
uint8_t i2c_2_waitack(void);
void i2c_2_ack(void);
void i2c_2_nack(void);
 
u8 fn_mcp23017_2_iodir(uint16_t ports);
u8 fn_mcp23017_2_setports(uint16_t ports);
 
#endif


