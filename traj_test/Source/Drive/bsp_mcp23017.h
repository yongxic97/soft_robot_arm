
#ifndef __BSP_MCP23017_H
#define __BSP_MCP23017_H
#include "stm32f10x.h"
 
#define MCP_GPIO_PORT_I2C		GPIOB
#define MCP_RCC_I2C_PORT 		RCC_APB2Periph_GPIOB
#define MCP_I2C_SCL_PIN			GPIO_Pin_6
#define MCP_I2C_SDA_PIN			GPIO_Pin_7
 
#define MCP_I2C_SCL_1()  GPIO_SetBits(MCP_GPIO_PORT_I2C, MCP_I2C_SCL_PIN)		/* SCL = 1 */
#define MCP_I2C_SCL_0()  GPIO_ResetBits(MCP_GPIO_PORT_I2C, MCP_I2C_SCL_PIN)		/* SCL = 0 */
 
#define MCP_I2C_SDA_1()  GPIO_SetBits(MCP_GPIO_PORT_I2C, MCP_I2C_SDA_PIN)		/* SDA = 1 */
#define MCP_I2C_SDA_0()  GPIO_ResetBits(MCP_GPIO_PORT_I2C, MCP_I2C_SDA_PIN)		/* SDA = 0 */
 
#define MCP_I2C_SDA_READ()  GPIO_ReadInputDataBit(MCP_GPIO_PORT_I2C, MCP_I2C_SDA_PIN)
 
void init_iic_mcp23017(void);
void i2c_delay(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_sendbyte(uint8_t _ucByte);
uint8_t i2c_readbyte(void);
uint8_t i2c_waitack(void);
void i2c_ack(void);
void i2c_nack(void);
 
u8 fn_mcp23017_iodir(uint16_t ports);
u8 fn_mcp23017_setports(uint16_t ports);
 
#endif


