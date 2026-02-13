#ifndef INA3221_H

#define INA3221_H

#include <stdint.h>

#define INA3221_REG_CONFIG                     0x00
#define INA3221_REG_SHUNT_V_CH1                0x01
#define INA3221_REG_BUS_V_CH1                  0x02
#define INA3221_REG_SHUNT_V_CH2                0x03
#define INA3221_REG_BUS_V_CH2                  0x04
#define INA3221_REG_SHUNT_V_CH3                0x05
#define INA3221_REG_BUS_V_CH3                  0x06
#define INA3221_REG_MANUFACTURER_ID 0xFE
#define INA3221_REG_DIE_ID                     0xFF

#define INA3221_MANUFACTURER_ID                0x5449
#define INA3221_DEFAULT_CONFIG                 0x7127

typedef struct
{
    uint8_t i2c_addr;

    int8_t (*i2c_write)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

    int8_t (*i2c_read)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

} INA3221_Handle_t;

int8_t INA3221_Init(INA3221_Handle_t *dev);

float INA3221_GetBusVoltage(INA3221_Handle_t *dev, uint8_t channel);

float INA3221_GetShuntVoltage(INA3221_Handle_t *dev, uint8_t channel);

float INA3221_GetCurrent(INA3221_Handle_t *dev, uint8_t channel, float shunt_resistor_ohms);

#endif
