#include <stdio.h>
#include <unistd.h>
#include "drivers/mss_i2c/mss_i2c.h"
#include "drivers/mss_uart/mss_uart.h"
#include "ina3221.h"

#define INA3221_I2C_ADDR    0x40
#define SHUNT_RES_OHMS      0.1f

int _write(int file, char *ptr, int len)
{
    MSS_UART_polled_tx(&g_mss_uart0, (const uint8_t *)ptr, len);
    return len;
}

int8_t m2s_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint8_t tx_buffer[16];
    
    tx_buffer[0] = reg_addr;
    for(int i = 0; i < len; i++)
    {
        tx_buffer[i+1] = data[i];
    }

    MSS_I2C_write(&g_mss_i2c0, dev_addr, tx_buffer, len + 1, MSS_I2C_RELEASE_BUS);
    
    mss_i2c_status_t status = MSS_I2C_wait_complete(&g_mss_i2c0, MSS_I2C_NO_TIMEOUT);
    
    return (status == MSS_I2C_SUCCESS) ? 0 : -1;
}

int8_t m2s_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    MSS_I2C_write(&g_mss_i2c0, dev_addr, &reg_addr, 1, MSS_I2C_HOLD_BUS);
    
    if (MSS_I2C_wait_complete(&g_mss_i2c0, MSS_I2C_NO_TIMEOUT) != MSS_I2C_SUCCESS)
    {
        return -1;
    }

    MSS_I2C_read(&g_mss_i2c0, dev_addr, data, len, MSS_I2C_RELEASE_BUS);
    
    mss_i2c_status_t status = MSS_I2C_wait_complete(&g_mss_i2c0, MSS_I2C_NO_TIMEOUT);

    return (status == MSS_I2C_SUCCESS) ? 0 : -1;
}

int main()
{
    MSS_UART_init(&g_mss_uart0, 
                  MSS_UART_115200_BAUD, 
                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_I2C_init(&g_mss_i2c0, INA3221_I2C_ADDR, MSS_I2C_PCLK_DIV_960);

    INA3221_Handle_t monitor;
    monitor.i2c_addr = INA3221_I2C_ADDR;
    monitor.i2c_write = m2s_i2c_write;
    monitor.i2c_read  = m2s_i2c_read;

    if (INA3221_Init(&monitor) != 0)
    {
        while(1);
    }

    while(1)
    {
        float bus_voltage = INA3221_GetBusVoltage(&monitor, 1);
        float current     = INA3221_GetCurrent(&monitor, 1, SHUNT_RES_OHMS);

        printf("CH1 Bus: %0.2f V | Current: %0.3f A\r\n", bus_voltage, current);
    }

    return 0;
}
