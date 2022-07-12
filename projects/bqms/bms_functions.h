#ifndef __BMS_FUNCTIONS_H__
    #define __BMS_FUNCTIONS_H__

    #include "Arduino.h"
    #include "my_io.h"
    
    void data_bus_write (uint8_t address);

    float bq_measureThermistorChannel(byte channel);

    void bq_readCellTemps(void);
    void bq_readCellVoltages(void);
    void bq_balanceCells(void);

#endif // __BMS_FUNCTIONS_H__