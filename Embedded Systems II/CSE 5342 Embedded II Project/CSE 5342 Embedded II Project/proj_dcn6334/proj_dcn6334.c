

/**
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
#include "eeprom.h"
#include "i2c0.h"
#include "cli.h"
#include "math.h"
#include "hibernation.h"

// The reason I shift left by 1 is because if you look at the library, it takes the address,
// and shift it right by 1

#define WRITE 0x40 >> 1 // Write opcode
#define READ 0x41 >> 1 // Read opcode

#define HB(x) (x >> 8) & 0xFF
#define LB(x) (x) & 0xFF

//Provide information for 24LC512 EEPROM
#define A0      PORTE,0
#define A1      PORTB,7
#define A2      PORTB,6
#define WP      PORTA,4

#define EEPROM_ADDR 0xA0
#define IMU_ADDR    0xD0

// Provide information for MPU
#define MPU9250 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H  0x65
#define TEMP_OUT_L  0x66
#define AK8963 0x0C

//For wEEPROM and rEEPROM
#define MAX_SENSORS 4
#define MAG         0
#define GYRO        1
#define ACCEL       2
#define TEMP        3

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF, 2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4


//Start at the beginning of the EEPROM (Table of contents of where everything is in the EEPROM)
// 0 - 64
typedef struct _metaData
{
    uint16_t addr[MAX_SENSORS];
    uint8_t count[MAX_SENSORS];
} metaData;

//Stores the sensor data for easy access
typedef struct _sensorData
{
    uint32_t timestamp;
    uint8_t x;
    uint8_t y;
    uint8_t z;
} sensorData;

uint8_t eeprom[1024];

//Read I2C for SDA and SCL on EEPROM
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

//Write to EEPROM through metadata
void wEeprom(uint8_t type, sensorData* d)
{
    // Read meta data first
    uint8_t i = 0;
    for(i = 0; i < sizeof(metaData); i++)
    {
        eeprom[i] = readI2c0Register16(0xA0 >> 1, i);
        waitMicrosecond(1000);
    }

    // The name of an array is its address
    metaData* mPtr = (metaData*)eeprom;

    uint16_t offset = mPtr->addr[type] + mPtr->count[type];

    // If we index into the eeprom, then we get a value.
    // By prefixing wiht & we get the address
    sensorData* sPtr = (sensorData*)(eeprom + offset);
    sPtr->timestamp = d->timestamp;
    sPtr->x = d->x;
    sPtr->y = d->y;
    sPtr->z = d->z;

    // Update the count
    mPtr->count[type] = mPtr->count[type] + 1;

    // Write the meta data first to the eeprom
    i = 0;
    for(i = 0; i < sizeof(metaData); i++)
    {
        uint8_t i2cData[2] = { LB(i), eeprom[i] };
        writeI2c0Registers(0xA0 >> 1, HB(i), i2cData, 2);
        waitMicrosecond(1000);
    }

    // Write the data back
    i = 0;
    for(i = offset; i < offset + sizeof(sensorData); i++)
    {
        uint8_t i2cData[2] = { LB(i), eeprom[i] };
        writeI2c0Registers(0xA0 >> 1, HB(i), i2cData, 2);
        waitMicrosecond(10000);
    }
}

//Write to EEPROM through metadata
void rEeprom(uint8_t type, uint8_t* count, uint8_t* offset)
{
    // Read meta data first
    uint8_t i = 0;
    for(i = 0; i < sizeof(metaData); i++)
    {
        eeprom[i] = readI2c0Register16(0xA0 >> 1, i);
        waitMicrosecond(1000);
    }

    // The name of an array is its address
    metaData* mPtr = (metaData*)eeprom;

    *offset = mPtr->addr[type] + mPtr->count[type];
    *count = mPtr->count[type];
}

//Initialize ADC0 for the internal temperature sensor
void initTemp()
{
    //Enable clock to ADC0
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Configure ADC
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

//Initialize MPU for configuration
void initMPU()
{
    writeI2c0Register(MPU9250,0x37,0xA2);
    //Turn on sensors
    writeI2c0Register(MPU9250,PWR_MGMT_1,0x00);
    waitMicrosecond(1000);
    //Gyro configuration
    writeI2c0Register(MPU9250,GYRO_CONFIG,0xFF);
    //Acceleration configuration
    writeI2c0Register(MPU9250,ACCEL_CONFIG,0x18);
    //Magnetometer configuration
    writeI2c0Register(AK8963,0x0A,0x06);
    //Clear interrupt flag
    readI2c0Register(MPU9250, 0x3A);
}

//Initialize RTC to store time values
void initRTC()
{
    //Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 -32);
    while(!HIB_CTL_WRC & HIB_CTL_R);
    return;
}

//Used to change the date and time
void resetRTC()
{
    while(!HIB_CTL_WRC & HIB_CTL_R);
    HIB_RTCLD_R = 0;
}

//Initialize EEPROM 241 512KB
void init24lc512()
{
    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTE);
    enablePort(PORTF);

    selectPinPushPullOutput(A0);
    selectPinPushPullOutput(A1);
    selectPinPushPullOutput(A2);
    selectPinPushPullOutput(WP);

    setPinValue(A0, 0);
    setPinValue(A1, 0);
    setPinValue(A2, 0);

    // Allow writes to the device
    setPinValue(WP, 0);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Days of each month for "date" command
uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

//Samples N
uint32_t N;
uint32_t count1 = 0;

//Internal Sensor on microcontroller
int32_t getTemp()
{
    int32_t temperature;
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);
        temperature = (((147 - ((247.5 * ADC0_SSFIFO3_R) / 4096)) - 32) * 5) / 9;
        return temperature;
}

//Internal temperature of MPU 9250
int16_t getSensorTemp()
{
    int16_t rawData[2];
    rawData[0] = readI2c0Register(MPU9250, 0x41);
    rawData[1] = readI2c0Register(MPU9250, 0x42);

    int16_t res = 0;
    res = rawData[0] << 8;
    res |= rawData[1];

    res = (res - 0)/331 + 21;
    return res;
}

//Read gyroscope data from MPU
void readGyro(int16_t * destination)
{
    uint16_t rawData[6];
    rawData[0] = readI2c0Register(MPU9250, GYRO_XOUT_H);
    rawData[1] = readI2c0Register(MPU9250, GYRO_XOUT_L);
    rawData[2] = readI2c0Register(MPU9250, GYRO_YOUT_H);
    rawData[3] = readI2c0Register(MPU9250, GYRO_YOUT_L);
    rawData[4] = readI2c0Register(MPU9250, GYRO_ZOUT_H);
    rawData[5] = readI2c0Register(MPU9250, GYRO_ZOUT_L);

    destination[0] = (float)(((int16_t)rawData[0] << 8) | rawData[1])/131.0;
    destination[1] = (float)(((int16_t)rawData[2] << 8) | rawData[3])/131.0;
    destination[2] = (float)(((int16_t)rawData[4] << 8) | rawData[5])/131.0;
}

//Read accelerometer data from MPU
void readAcceleration(int16_t * destination)
{
    uint16_t rawData[6];
    rawData[0] = readI2c0Register(MPU9250, ACCEL_XOUT_H);
    rawData[1] = readI2c0Register(MPU9250, ACCEL_XOUT_L);
    rawData[2] = readI2c0Register(MPU9250, ACCEL_YOUT_H);
    rawData[3] = readI2c0Register(MPU9250, ACCEL_YOUT_L);
    rawData[4] = readI2c0Register(MPU9250, ACCEL_ZOUT_H);
    rawData[5] = readI2c0Register(MPU9250, ACCEL_ZOUT_L);

    destination[0] = (float)(((int16_t)rawData[0] << 8) | rawData[1])/16384.0;
    destination[1] = (float)(((int16_t)rawData[2] << 8) | rawData[3])/16384.0;
    destination[2] = (float)(((int16_t)rawData[4] << 8) | rawData[5])/16384.0;
}

//Read compass data from MPU
void readCompass(int16_t * destination)
{
    writeI2c0Register(0x68, 0x37, 0x02);
    writeI2c0Register(0x0C, 0x0A, 0x01);

    while(!(readI2c0Register(0x0C, 0x02) & 1));



    uint16_t x1 = readI2c0Register(0x0C, 0x04);
    x1 = (x1 << 8) | readI2c0Register(0x0C, 0x03);

    uint16_t y = readI2c0Register(0x0C, 0x06);
    y = (y << 8) | readI2c0Register(0x0C, 0x05);

    uint16_t z = readI2c0Register(0x0C, 0x08);
    z = (z << 8) | readI2c0Register(0x0C, 0x07);
}

//Initialize level shift to turn on or off EEPROM
void initLevelShift()
{
    enablePort(PORTF);
    selectPinPushPullOutput(PORTF, 1);
}

int main(void)
{
    //Initialize everything
    initLevelShift();
    initSystemClockTo40Mhz();
    initI2c0();
    initUart0();
    initMPU();
    init24lc512();
    initTemp();
    // initRTC();
    setPinValue(PORTF, 1, 1);

    //Data from user to be stored
    USER_DATA userData;

    //Signal encryption
    uint32_t Encrypt = 0;
    uint32_t key = 17;


    char x[128];
    int16_t values[3];

    // Write the meta data first to the eeprom

    // The name of an array is its address
    uint8_t i = 0;

    metaData* mPtr = (metaData*)eeprom;
    mPtr->addr[MAG] = 64;
    mPtr->addr[GYRO] = 128;
    mPtr->addr[ACCEL] = 192;
    mPtr->addr[TEMP] = 256;

    mPtr->count[MAG] = 0;
    mPtr->count[GYRO] = 0;
    mPtr->count[ACCEL] = 0;
    mPtr->count[TEMP] = 0;

    i = 0;
    for(i = 0; i < sizeof(metaData); i++)
    {
        uint8_t i2cData[2] = { LB(i), eeprom[i] };
        writeI2c0Registers(0xA0 >> 1, HB(i), i2cData, 2);
        waitMicrosecond(1000);
    }

    for(i = 0; i < sizeof(metaData); i++)
    {
        eeprom[i] = readI2c0Register16(0xA0 >> 1, i);
        waitMicrosecond(1000);
    }
    //Temperature gating
    int16_t temperature1 = 20;
    // False is for < and True is for >
    bool tlevel = false;
    //Accelerometer gating
    int16_t accelerate = 1;
    // False is for < and True is for >
    bool alevel = false;
    //Gyroscope gating
    int16_t gyroscope = 20;
    // False is for < and True is for >
    bool glevel = false;
    //Compass gating
    int16_t magneto = 50;
    // False is for < and True is for >
    bool mlevel = false;

    putsUart0("Data logger initialized\n");
    putsUart0("> ");
	while(true)
	{   //Temperature level: If > or <, turn on or off EEPROM
	    if(!tlevel)
	    {
	        if(getSensorTemp() > temperature1)
	            setPinValue(PORTF, 1, 1);
	        else
	            setPinValue(PORTF, 1, 0);
	    }
	    else if(tlevel)
	    {
	        if(getSensorTemp() > temperature1)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
	    }
/*
	    //Accelerometer level
        if(!alevel)
        {
            readAcceleration(values);
            int16_t go = values;
            if(values[0] > accelerate || values[1] > accelerate || values[2] > accelerate)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }
        else if(alevel)
        {
            readAcceleration(values);
            if(values[0] > accelerate || values[1] > accelerate || values[2] > accelerate)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }*/

        //Gyro level: If > or <, turn on or off EEPROM
        if(!glevel)
        {
            readGyro(values);
            if(values[0] > gyroscope || values[1] > gyroscope || values[2] > gyroscope)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }
        else if(glevel)
        {
            readGyro(values);
            if(values[0] > gyroscope || values[1] > gyroscope || values[2] > gyroscope)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }
/*
        //Compass Level
        if(!mlevel)
        {
            readCompass(values);
            if(values[0] > magneto || values[1] > magneto || values[2] > magneto)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }
        else if(mlevel)
        {
            readCompass(values);
            if(values[0] > magneto || values[1] > magneto || values[2] > magneto)
                setPinValue(PORTF, 1, 1);
            else
                setPinValue(PORTF, 1, 0);
        }
        */
	    if(kbhitUart0())
	    {
            getsUart0(&userData);
            parseField(&userData);
            //> levelShift 1 to turn on or off
            if(isCommand(&userData, "levelShift", 1))
            {
                int32_t arg = getFieldInteger(&userData, 1);
                if(arg == 1)
                    setPinValue(PORTF, 1, 1);
                else
                    setPinValue(PORTF, 1, 0);
            }
            //Check if EEPROM and MPU are on
            if(isCommand(&userData, "poll", 0))
            {
                if (pollI2c0Address(0xA0 >> 1))
                {
                    putsUart0("EEPROM found!\r\n");
                    setPinValue(GREEN_LED, 1);
                }

                if(pollI2c0Address(0xD0 >> 1))
                {
                    setPinValue(GREEN_LED, 0);
                    setPinValue(BLUE_LED, 1);
                    putsUart0("IMU Found\r\n");
                }
            }
    /////////////////////Debug/////////////////////////////
            if(isCommand(&userData, "reset", 0))
            {
                putsUart0("Reseting...\n");
                NVIC_APINT_R = NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECTKEY;
            }
            //Receive changing temperature value from MPU
            if(isCommand(&userData, "temp", 0))
            {
                int16_t temp = getSensorTemp();
                sprintf(x, "Temperature value is: %d\r\n", temp);
                putsUart0(x);
                uint32_t temp_encrypt;
                /*
                if(Encrypt == 1)
                {
                    temp_encrypt = temp + key;

                    sprintf(x, "Temperature value is: %d\r\n", temp_encrypt);
                    putsUart0(x);
                }

                else if (Encrypt == 0)
                {
                    temp_encrypt = temp - key;
                    sprintf(x, "Temperature value is: %d\r\n", temp_encrypt);
                    putsUart0(x);
                }
                */
            }
    ////////////////////Configuration/////////////////////////////
            //Check the time stored on RTC
            if(isCommand(&userData, "time", 0))
            {
                //uint16_t chrono = set_time();
                uint16_t day = 29;
                uint16_t hour = 5;
                uint16_t min = 8;
                uint16_t sec = 50;
                uint32_t RTC = HIB_RTCC_R;
                RTC += ((day/86400) + (hour * 3600) + (min * 60) + sec);
                uint16_t dayout = floor(RTC / 86400);
                uint16_t hourout = floor((RTC - (dayout * 86400)) / 3600);
                uint16_t minout = floor((RTC - (dayout * 86400) - (hourout * 3600))/60);
                uint16_t secout = floor((RTC - (dayout * 86400) - (hourout * 3600) - (minout *60)));
                if(Encrypt == 1)
                {
                    sprintf(x, "Time: %d : %d : %d\r\n", hourout + key, minout + key, secout + key);
                    putsUart0(x);
                }

                else if(Encrypt == 0)
                {
                    sprintf(x, "Time: %d : %d : %d\r\n", hourout, minout, secout);
                    putsUart0(x);
                }

                waitMicrosecond(90000);
            }

            //Check the date
            if(isCommand(&userData, "date", 0))
            {
                uint32_t RTC;
                uint16_t i = 0;
                uint16_t month = 12;
                uint16_t day = 7;
                uint16_t temp_day = 0;

                if(month == 2 || month == 9 || month == 11)
                {
                    for(i = 0; i < month-1; i++)
                    {
                        temp_day += daysOfEachMonth[i];
                    }
                }

                else
                {
                    for(i = 0; i <= month-1; i++)
                    {
                        temp_day += daysOfEachMonth[i];
                    }
                }

                RTC = temp_day;
                day += RTC;

                uint16_t monthout = floor(day / daysOfEachMonth[month-1]);
                uint16_t dayout = day - RTC;
                sprintf(x, "Day: %d/%d\r\n", monthout, dayout);
                putsUart0(x);
                waitMicrosecond(90000);
            }

            //Check the compass value from MPU
            if(isCommand(&userData, "compass", 0))
            {
                writeI2c0Register(0x68, 0x37, 0x02);
                writeI2c0Register(0x0C, 0x0A, 0x01);

                while(!(readI2c0Register(0x0C, 0x02) & 1));



                uint16_t x1 = readI2c0Register(0x0C, 0x04);
                x1 = (x1 << 8) | readI2c0Register(0x0C, 0x03);

                uint16_t y = readI2c0Register(0x0C, 0x06);
                y = (y << 8) | readI2c0Register(0x0C, 0x05);

                uint16_t z = readI2c0Register(0x0C, 0x08);
                z = (z << 8) | readI2c0Register(0x0C, 0x07);

                sprintf(x, "Magnetometer: x = %hu, y = %hu, z = %hu\r\n", x1, y, z);
                putsUart0(x);
                waitMicrosecond(90000);
/*
                if(Encrypt == 1)
                {
                    sprintf(x, "Magnetometer: x = %hu, y = %hu, z = %hu\r\n", x1 + key, y + key, z + key);
                    putsUart0(x);
                    waitMicrosecond(90000);
                }

                else if(Encrypt == 0)
                {

                }
*/
            }
            //Read gyroscope data from MPU
            if(isCommand(&userData, "gyro", 0))
            {
                readGyro(values);
                sprintf(x, "Gyro data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                putsUart0(x);
                waitMicrosecond(90000);
                /*if(N != 0)
                {
                    while(count1 < N)
                    {

                        sprintf(x, "Gyro data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                        putsUart0(x);
                        waitMicrosecond(90000);
                        count1++;
                    }
                 }
                else
                {
                    sprintf(x, "Gyro data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                    putsUart0(x);
                    waitMicrosecond(90000);
                }*/

            }
            //Read accelerometer data from MPU
            if(isCommand(&userData, "accel", 0))
            {
                readAcceleration(values);
                sprintf(x, "Acceleration data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                putsUart0(x);
                /*if(N != 0)
                {
                    while(count1 < N)
                    {
                        sprintf(x, "Acceleration data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                        putsUart0(x);
                        count1++;
                    }
                }
                else
                {
                    readAcceleration(values);
                    sprintf(x, "Acceleration data: %d  %d  %d\r\n", values[0], values[1], values[2]);
                    putsUart0(x);
                }*/


            }

            // gating temp GT 12

            //Gate the following information from user to turn on or off
            //if the value is > or < the input
            if(isCommand(&userData, "gating", 3))
            {
                char* arg1 = getFieldString(&userData, 1);
                char* arg2 = getFieldString(&userData, 2);
                int32_t arg3 = getFieldInteger(&userData, 3);
                //Gating for temperature
                if(stringCompare(arg1, "temp", 16))
                {
                    if(stringCompare(arg2, "GT", 16))
                        tlevel = true;
                    else
                        tlevel = false;
                    temperature1 = arg3;
                }

                //Gating for accel
                if(stringCompare(arg1, "accel", 2))
                {
                    if(stringCompare(arg2, "GT", 2))
                        alevel = true;
                    else
                        alevel = false;
                    accelerate = arg3;
                }

                if(stringCompare(arg1, "gyro", 20))
                {
                    if(stringCompare(arg2, "GT", 20))
                        glevel = true;
                    else
                        glevel = false;
                    gyroscope = arg3;
                }

                if(stringCompare(arg1, "compass", 20))
                {
                    if(stringCompare(arg2, "GT", 20))
                        mlevel = true;
                    else
                        mlevel = false;
                    magneto = arg3;
                }
            }

            //Log all compass data by time
            if(isCommand(&userData, "logCompass", 0))
            {
                // Read the compass here first
                // Create the data packet
                // Save it into the EEPROM
                writeI2c0Register(0x68, 0x37, 0x02);
                writeI2c0Register(0x0C, 0x0A, 0x01);

                while(!(readI2c0Register(0x0C, 0x02) & 1));



                uint16_t x1 = readI2c0Register(0x0C, 0x04);
                x1 = (x1 << 8) | readI2c0Register(0x0C, 0x03);

                uint16_t y1 = readI2c0Register(0x0C, 0x06);
                y1 = (y1 << 8) | readI2c0Register(0x0C, 0x05);

                uint16_t z1 = readI2c0Register(0x0C, 0x08);
                z1 = (z1 << 8) | readI2c0Register(0x0C, 0x07);

                sensorData s;
                uint16_t day = 29;
                uint16_t hour = 5;
                uint16_t min = 8;
                uint16_t sec = 50;
                s.timestamp = HIB_RTCC_R;
                s.timestamp += ((day/86400) + (hour * 3600) + (min * 60) + sec);
                uint16_t dayout = floor(s.timestamp / 86400);
                uint16_t hourout = floor((s.timestamp - (dayout * 86400)) / 3600);
                uint16_t minout = floor((s.timestamp - (dayout * 86400) - (hourout * 3600))/60);
                uint16_t secout = floor((s.timestamp - (dayout * 86400) - (hourout * 3600) - (minout *60)));
                s.x = x1;
                s.y = y1;
                s.z = z1;

                N = 0;
                if(N != 0)
                {
                    while(count1 < N)
                    {
                        wEeprom(MAG, &s);
                        uint8_t offset = 0, count = 0;
                        rEeprom(MAG, &count, &offset);

                        uint8_t i = 0;
                        for(i = offset; i < offset + count; i++)
                        {
                            eeprom[i] = readI2c0Register16(0xA0 >> 1, i);
                            waitMicrosecond(1000);
                        }

                        for(i = 0; i < count; i++)
                        {
                            sensorData* sPtr = (sensorData*)(eeprom + (offset - 1)) + i;
                            sprintf(x, "Magnetometer data count = %hhu @%hhu: Timestamp = %d : %d : %d, x = %hhu, y = %hhu, z = %hhu\n", count, offset, hourout, minout, secout, sPtr->timestamp, sPtr->x, sPtr->y, sPtr->z);
                            putsUart0(x);
                        }
                        count1++;
                    }
                }
                else
                {
                    wEeprom(MAG, &s);
                    uint8_t offset = 0, count = 0;
                    rEeprom(MAG, &count, &offset);

                    uint8_t i = 0;
                    for(i = offset; i < offset + count; i++)
                    {
                        eeprom[i] = readI2c0Register16(0xA0 >> 1, i);
                        waitMicrosecond(1000);
                    }

                    for(i = 0; i < count; i++)
                    {
                        sensorData* sPtr = (sensorData*)(eeprom + (offset - 1)) + i;
                        sprintf(x, "Magnetometer data count = %hhu @%hhu: Timestamp = %d : %d : %d, x = %hhu, y = %hhu, z = %hhu\n", count, offset, hourout, minout, secout, sPtr->timestamp, sPtr->x, sPtr->y, sPtr->z);
                        putsUart0(x);
                    }
                }


            }

            //The number of times you want EEPROM to read and write
            if(isCommand(&userData, "samples", 1))
            {
                int32_t arg = getFieldInteger(&userData, 1);
                N = arg;
                putsUart0("Sample entered\n");
            }

            // Parameter will be set to H
            if(isCommand(&userData, "hysteresisPH", 0))
            {

            }

            //Enter hibernation and trigger to wake up
            if(isCommand(&userData, "sleep", 0))
            {
                if(!checkIfConfigured())
                    initHibernationModule();

                setPinValue(RED_LED, 1);

                if(wakePinCausedWakeUp())
                {
                    setPinValue(BLUE_LED, 1);
                    setPinValue(RED_LED, 0);
                    putsUart0("Trigger activated\r\n");
                }


                while(getPinValue(PUSH_BUTTON));
                {
                    setPinValue(RED_LED, 1);
                }

                setPinValue(RED_LED, 0);
                setPinValue(GREEN_LED, 0);
                setPinValue(BLUE_LED, 0);
                hibernate(30);

                setPinValue(RED_LED, 1);

                SYSCTL_SCGCGPIO_R = 0; //GPIO Port F is off

            }


            if(isCommand(&userData, "levelingOff", 0))
            {

            }

            if(isCommand(&userData, "levelingOn", 0))
            {

            }

            if(isCommand(&userData, "encryptOff", 1))
            {
                putsUart0("Encrypt Off\r\n");
                Encrypt = 0;
            }

            //Encryption key by user key input
            if(isCommand(&userData, "encryptKey", 0))
            {
                putsUart0("Encrypt On\r\n");
                Encrypt = 1;

            }


    /////////////////////////Sample Control///////////////
            if(isCommand(&userData, "periodicT", 0))
            {

            }

            if(isCommand(&userData, "trigger", 0))
            {
                putsUart0("Trigger On\r\n");
            }

            if(isCommand(&userData, "stop", 0))
            {

            }
            putsUart0("> ");
	    }
	}

}
