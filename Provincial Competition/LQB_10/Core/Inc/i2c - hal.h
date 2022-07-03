#ifndef __I2C_H
#define __I2C_H

#include "main.h"

void I2CStart(void);
void I2CStop(void);
unsigned char I2CWaitAck(void);
void I2CSendAck(void);
void I2CSendNotAck(void);
void I2CSendByte(unsigned char cSendByte);
unsigned char I2CReceiveByte(void);
void I2CInit(void);
void I2C_24C02_Write(uint8_t *pucStr, uint8_t Addr, uint8_t Num);
void I2C_24C02_Read(uint8_t *pucStr, uint8_t Addr, uint8_t Num);
void MCP4017_Write(uint8_t value);
uint8_t MCP4017_Read(void);
#endif
