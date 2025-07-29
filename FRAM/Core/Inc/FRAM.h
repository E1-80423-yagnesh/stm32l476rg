#ifndef __FRAM_H
#define __FRAM_H

#include "stm32l4xx_hal.h"
#include "main.h"




#define FM_24CL16_Write 0xA0
#define FM_24CL16_Read  0xA1
#define BufferSize 2048

// Operation status definitions
#define Result_Success  0
#define Result_Error    4

// Function prototypes
void FM24CL16B_Write_Data(uint16_t address, uint8_t data);
uint8_t FM24CL16B_Read_Data(uint16_t address);
void Parameter_Write(uint16_t param_address, float param_data);
float Parameter_Read(uint16_t param_address);

#endif
