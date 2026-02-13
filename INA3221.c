#include <ina3221.h>

static int16_t merge_bytes(uint8_t msb, uint8_t lsb)
{
      return (int16_t)((msb << 8) | lsb);
}

int8_t INA3221_Init(INA3221_Handle_t *dev)
{
      uint8_t id_data[2];

      if (dev->i2c_read(dev->i2c_addr, INA3221_REG_MANUFACTURER_ID, id_data, 2) != 0)
    {
            return -1;
      }

      uint16_t man_id = (id_data[0] << 8) | id_data[1];

      if (man_id != INA3221_MANUFACTURER_ID)
    {
            return -2;
      }

      uint8_t config_data[2] = {0x71, 0x27};

      dev->i2c_write(dev->i2c_addr, INA3221_REG_CONFIG, config_data, 2);

      return 0;
}

float INA3221_GetBusVoltage(INA3221_Handle_t *dev, uint8_t channel)
{
      uint8_t reg = 0;

      switch(channel)
      {
            case 1: reg = INA3221_REG_BUS_V_CH1; break;
            case 2: reg = INA3221_REG_BUS_V_CH2; break;
            case 3: reg = INA3221_REG_BUS_V_CH3; break;

            default: return 0.0f;
      }

      uint8_t data[2];

      dev->i2c_read(dev->i2c_addr, reg, data, 2);

      int16_t raw_val = merge_bytes(data[0], data[1]);

      raw_val = raw_val >> 3;

      return raw_val * 0.008f;
}

float INA3221_GetShuntVoltage(INA3221_Handle_t *dev, uint8_t channel)
{
      uint8_t reg = 0;
      switch(channel)
    {
            case 1: reg = INA3221_REG_SHUNT_V_CH1; break;
            case 2: reg = INA3221_REG_SHUNT_V_CH2; break;
            case 3: reg = INA3221_REG_SHUNT_V_CH3; break;

            default: return 0.0f;
      }

      uint8_t data[2];

      dev->i2c_read(dev->i2c_addr, reg, data, 2);

      int16_t raw_val = merge_bytes(data[0], data[1]);

      raw_val = raw_val >> 3;

      return raw_val * 0.00004f;
}

float INA3221_GetCurrent(INA3221_Handle_t *dev, uint8_t channel, float shunt_resistor_ohms)
{
      float voltage_drop = INA3221_GetShuntVoltage(dev, channel);
      return voltage_drop / shunt_resistor_ohms;
}
