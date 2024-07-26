#include "PAC193x.h"
#include <string.h>

void PAC193x_init(Microchip_PAC193x* dev, uint32_t resistor_value) {
    dev->rsense = resistor_value;
    dev->error_code = 0;
}

void PAC193x_begin(Microchip_PAC193x* dev) {
    uint8_t data;
    
    data = 0;
    PAC193x_i2c_write(PAC1934_NEG_PWR_ADDR, &data, 1);
    
    data = 2;
    PAC193x_i2c_write(PAC1934_CTRL_ADDR, &data, 1);
    
    data = 20; //14h
    PAC193x_i2c_write(PAC1934_SLOW_ADDR, &data, 1);

    PAC193x_update_slow_status(dev);
    PAC193x_refresh(dev);
    // Delay function should be implemented by the user
    // delay(125);
}

void PAC193x_refresh(Microchip_PAC193x* dev) {
    uint8_t cmd = PAC1934_REFRESH_CMD_ADDR;
    dev->error_code = PAC193x_i2c_write(PAC1934_REFRESH_CMD_ADDR, &cmd, 1);
    
    if (dev->error_code != 0) {
        dev->error_code = -2;
    }

    // Timestamp function should be implemented by the user
    // dev->refresh_timestamp = get_microseconds();
}

int16_t PAC193x_update_overflow_alert(Microchip_PAC193x* dev) {
    uint8_t temp_read;
    
    dev->error_code = PAC193x_i2c_read(PAC1934_CTRL_ACT_ADDR, &temp_read, 1);
    if (dev->error_code == 0) {
        dev->overflow_alert = temp_read & 0x01;
    }
    
    return dev->error_code;
}

int16_t PAC193x_update_slow_status(Microchip_PAC193x* dev) {
    uint8_t temp_read;
    
    dev->error_code = PAC193x_i2c_read(PAC1934_SLOW_ADDR, &temp_read, 1);
    if (dev->error_code == 0) {
        dev->slow_status = (temp_read & 0x80) >> 7;
    }
    
    return dev->error_code;
}

int16_t PAC193x_update_power_on_status(Microchip_PAC193x* dev) {
    uint8_t temp_read, temp_write;
    
    dev->error_code = PAC193x_i2c_read(PAC1934_SLOW_ADDR, &temp_read, 1);
    if (dev->error_code == 0) {
        dev->power_on_status = temp_read & 0x01;
        temp_write = temp_read & 0xFE;
        dev->error_code = PAC193x_i2c_write(PAC1934_SLOW_ADDR, &temp_write, 1);
    }
    
    return dev->error_code;
}

int16_t PAC193x_update_sample_rate_lat(Microchip_PAC193x* dev) {
    uint8_t sample_rate_bits;
    
    dev->error_code = PAC193x_i2c_read(PAC1934_CTRL_LAT_ADDR, &sample_rate_bits, 1);
    if (dev->error_code == 0) {
        sample_rate_bits = (sample_rate_bits & 0xC0) >> 6;
        switch(sample_rate_bits) {
            case 0: dev->sample_rate_lat = 1024; break;
            case 1: dev->sample_rate_lat = 256; break;
            case 2: dev->sample_rate_lat = 64; break;
            case 3: dev->sample_rate_lat = 8; break;
            default: dev->sample_rate_lat = (dev->slow_status) ? 8 : 1024;
        }
    }
    
    return dev->error_code;
}

int16_t PAC193x_set_sample_rate(Microchip_PAC193x* dev, uint16_t value) {
    uint8_t temp_read, temp_write;
    
    dev->error_code = PAC193x_i2c_read(PAC1934_CTRL_ADDR, &temp_read, 1);
    if (dev->error_code == 0) {
        switch(value) {
            case 1024: temp_write = 0; break;
            case 256: temp_write = 1; break;
            case 64: temp_write = 2; break;
            case 8: temp_write = 3; break;
            default:
                dev->error_code = -4;
                return dev->error_code;
        }
        
        temp_write = (temp_write << 6) | (temp_read & 0x3F);
        dev->error_code = PAC193x_i2c_write(PAC1934_CTRL_ADDR, &temp_write, 1);
        if (dev->error_code == 0) {
            PAC193x_refresh(dev);
            // Delay function should be implemented by the user
            // delay(125);
        }
    }
    
    return dev->error_code;
}

int16_t PAC193x_update_product_id(Microchip_PAC193x* dev) {
    dev->error_code = PAC193x_i2c_read(PAC1934_PRODUCT_ID_ADDR, &dev->product_id, 1);
    return dev->error_code;
}

int16_t PAC193x_update_manufacturer_id(Microchip_PAC193x* dev) {
    dev->error_code = PAC193x_i2c_read(PAC1934_MANUFACTURER_ID_ADDR, &dev->manufacturer_id, 1);
    return dev->error_code;
}

int16_t PAC193x_update_revision_id(Microchip_PAC193x* dev) {
    dev->error_code = PAC193x_i2c_read(PAC1934_REVISION_ID_ADDR, &dev->revision_id, 1);
    return dev->error_code;
}
