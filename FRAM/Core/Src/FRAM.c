#include "FRAM.h"

#include "stdio.h"


void FM24CL16B_Write_Data(uint16_t address, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, FM_24CL16_Write, address, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xff);
    HAL_Delay(5); // Short delay for write completion
    printf("\r\n EEPROM 24C02 Write Test OK \r\n");

    FM24CL16B_Read_Data(address);
}


uint8_t FM24CL16B_Read_Data(uint16_t address)
{
    uint8_t read_data;
    HAL_I2C_Mem_Read(&hi2c1, FM_24CL16_Read, address, I2C_MEMADD_SIZE_8BIT, &read_data, 1, 0xff);
    printf("Address: %d    Data: 0x%02X    Decimal: %d \r\n", address, read_data, read_data);
    return read_data;
}


void Parameter_Write(uint16_t param_address, float param_data)
{
    uint16_t integer_part;
    uint8_t high_byte1;
    uint8_t high_byte2;
    uint16_t decimal_part;
    uint8_t low_byte1;
    uint8_t low_byte2;

    // Extract integer part and convert to bytes
    integer_part = (uint16_t)param_data;
    high_byte1 = integer_part / 256;
    high_byte2 = integer_part % 256;

    // Extract decimal part and convert to bytes
    decimal_part = (param_data - integer_part) * 1000.0;
    low_byte1 = decimal_part / 256;
    low_byte2 = decimal_part % 256;

    // Write bytes to consecutive EEPROM addresses
    HAL_I2C_Mem_Write(&hi2c1, FM_24CL16_Write, param_address, I2C_MEMADD_SIZE_8BIT, &high_byte1, 1, 0xff);
    HAL_Delay(5); // Write delay
    HAL_I2C_Mem_Write(&hi2c1, FM_24CL16_Write, param_address+1, I2C_MEMADD_SIZE_8BIT, &high_byte2, 1, 0xff);
    HAL_Delay(5); // Write delay
    HAL_I2C_Mem_Write(&hi2c1, FM_24CL16_Write, param_address+2, I2C_MEMADD_SIZE_8BIT, &low_byte1, 1, 0xff);
    HAL_Delay(5); // Write delay
    HAL_I2C_Mem_Write(&hi2c1, FM_24CL16_Write, param_address+3, I2C_MEMADD_SIZE_8BIT, &low_byte2, 1, 0xff);
    HAL_Delay(5); // Write delay

    // Read back stored values for verification
    FM24CL16B_Read_Data(param_address);
    FM24CL16B_Read_Data(param_address+1);
    FM24CL16B_Read_Data(param_address+2);
    FM24CL16B_Read_Data(param_address+3);
}


float Parameter_Read(uint16_t param_address)
{
    float param_data;
    uint8_t high_byte1;
    uint8_t high_byte2;
   // uint16_t decimal_part;
    uint8_t low_byte1;
    uint8_t low_byte2;

    // Read bytes from consecutive EEPROM addresses
    high_byte1 = FM24CL16B_Read_Data(param_address);
    high_byte2 = FM24CL16B_Read_Data(param_address+1);
    low_byte1 = FM24CL16B_Read_Data(param_address+2);
    low_byte2 = FM24CL16B_Read_Data(param_address+3);

    // Reconstruct float value
    param_data = high_byte1 * 256 + high_byte2 + (low_byte1 * 256 + low_byte2) * 0.001;
    printf("Parameter Value: %f\r\n", param_data);

    return param_data;
}
