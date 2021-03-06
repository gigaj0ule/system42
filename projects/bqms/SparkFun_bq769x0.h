#include "Arduino.h"
#include "Wire.h"

// Register addresses
#define bq796x0_SYS_STAT        0x00
#define bq796x0_CELLBAL1        0x01
#define bq796x0_CELLBAL2        0x02
#define bq796x0_CELLBAL3        0x03
#define bq796x0_SYS_CTRL1       0x04
#define bq796x0_SYS_CTRL2       0x05
#define bq796x0_PROTECT1        0x06
#define bq796x0_PROTECT2        0x07
#define bq796x0_PROTECT3        0x8
#define bq796x0_OV_TRIP         0x09
#define bq796x0_UV_TRIP         0x0A
#define bq796x0_CC_CFG          0x0B

// Read-only
#define bq796x0_VC1_HI          0x0C
#define bq796x0_VC1_LO          0x0D

// Other VC registers are done with an offset in software
#define bq796x0_BAT_HI          0x2A
#define bq796x0_BAT_LO          0x2B
#define bq796x0_TS1_HI          0x2C
#define bq796x0_TS1_LO          0x2D

// Ofther TS registers are done with an offset in software
#define bq796x0_CC_HI           0x32
#define bq796x0_CC_LO           0x33
#define bq796x0_ADCGAIN1        0x50
#define bq796x0_ADCOFFSET       0x51
#define bq796x0_ADCGAIN2        0x59

// SYS_STAT bit masks
#define bq796x0_CC_READY        1 << 7
#define bq796x0_DEVICE_XREADY   1 << 5
#define bq796x0_OVRD_ALERT      1 << 4
#define bq796x0_UV              1 << 3
#define bq796x0_OV              1 << 2
#define bq796x0_SCD             1 << 1
#define bq796x0_OCD             1 << 0

// SYS_CTRL1 bit masks
#define bq796x0_LOAD_PRESENT    1 << 7
#define bq796x0_ADC_EN          1 << 4
#define bq796x0_TEMP_SEL        1 << 3
#define bq796x0_SHUT_A          1 << 1
#define bq796x0_SHUT_B          1 << 0

// SYS_CTRL2 bit masks
#define bq796x0_DELAY_DIS       1 << 7
#define bq796x0_CC_EN           1 << 6
#define bq796x0_CC_ONESHOT      1 << 5
#define bq796x0_DSG_ON          1 << 1
#define bq796x0_CHG_ON          1 << 0

bool    bq769x0_initBQ(TwoWire &passedWire, byte irqPin);
void    bq769x0_displayVoltages(void);
void    bq769x0_enableBalancing(byte cellNumber, bool enabled);
void    bq769x0_enterSHIPmode(void);
float   bq769x0_readCellVoltage(byte cellNumber);
int     bq769x0_readTemp(byte thermistorNumber);
float   bq769x0_readCoulombCounter(void);
float   bq769x0_readPackVoltage(void);

float   bq769x0_readGAIN(void);

int     bq769x0_readADCoffset(void);
float   bq769x0_readOVtrip(void);
void    bq769x0_writeOVtrip(float tripVoltage);
float   bq769x0_readUVtrip(void);
void    bq769x0_writeUVtrip(float tripVoltage);
byte    bq769x0_tripCalculator(float tripVoltage);
void    bq769x0_registerWrite(byte regAddress, byte regData);
byte    bq769x0_registerRead(byte regAddress);
int     bq769x0_registerDoubleRead(byte regAddress);
int     bq769x0_thermistorLookup(float resistance);

void    bq769x0_reset(void);
void    bq769x0_wake(void);

bool    bq769x0_listen(void);
float   bq76940_roundf(float number, int precision);

float    bq769x0_readDieTemp(int dieNumber);


// My pack is a 15 cell lipo that runs at 48V. Your pack may vary. Read the datasheet!
// This code is written for the bq76940. The bq76940 supports 9 to 15 cells.
#define NUMBER_OF_CELLS 15

// Max number of ms before timeout error. 100 is pretty good
#define MAX_I2C_TIME 100
