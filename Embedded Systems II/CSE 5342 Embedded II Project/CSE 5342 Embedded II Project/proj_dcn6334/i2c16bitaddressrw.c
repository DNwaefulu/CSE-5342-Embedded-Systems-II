// This will only work for devices with 16-bit address for registers
/*

#include "i2c0.h"
#include "tm4c123gh6pm.h"

#define HB(x) (x >> 8) & 0xFF
#define LB(x) (x) & 0xFF

uint8_t readI2c0Register16(uint8_t add, uint16_t reg)
{
    I2C0_MSA_R = add << 1;
    I2C0_MDR_R = (reg >> 8) & 0xFF;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    I2C0_MDR_R = reg & 0xFF;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    I2C0_MSA_R = (add << 1) | 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    return I2C0_MDR_R;
}
*/
/*
    Template for writing the data;

    Let's say you have a data value of 0xA and want to write to address 5 of the EEPROM (0xA0 - notice how I shift the address by 1 to the right).

    uint8_t i2cData = { LB(5), 0xA };
    writeI2c0Registers(0xA0 >> 1, HB(5), i2cData, 2);
*/



