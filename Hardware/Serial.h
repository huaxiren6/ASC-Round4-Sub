#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern int16_t Serial_TxPacket[];
extern int16_t Serial_RxPacket[];

void Serial_Init(void);
void Serial_SendByte(int16_t Byte);
void Serial_SendArray(int16_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
uint8_t Serial_GetRxFlag(void);

#endif
