#ifndef MICROCHIP_PAC193X_H
#define MICROCHIP_PAC193X_H

#include <stdint.h>
#include <stdbool.h>

#define RSENSE 24900000 //microohm
#define I2C_ADDRESS 0x10
#define CHANNEL 1

// Register addresses
#define PAC1932_REFRESH_CMD_ADDR            0x00
#define PAC1932_CTRL_ADDR                   0x01
#define PAC1932_ACC_COUNT_ADDR              0x02
#define PAC1932_VPOWER1_ACC_ADDR            0x03
#define PAC1932_VPOWER2_ACC_ADDR            0x04
#define PAC1932_VBUS1_ADDR                  0x07
#define PAC1932_VBUS2_ADDR                  0x08
#define PAC1932_VSENSE1_ADDR                0x0B
#define PAC1932_VSENSE2_ADDR                0x0C
#define PAC1932_VBUS1_AVG_ADDR              0X0F
#define PAC1932_VBUS2_AVG_ADDR              0X10
#define PAC1932_VSENSE1_AVG_ADDR            0X13
#define PAC1932_VSENSE2_AVG_ADDR            0X14
#define PAC1932_VPOWER1_ADDR                0X17
#define PAC1932_VPOWER2_ADDR                0X18
#define PAC1932_CHANNEL_DIS_ADDR            0X1C
#define PAC1932_NEG_PWR_ADDR                0X1D
#define PAC1932_REFRESH_G_CMD_ADDR          0x1E
#define PAC1932_REFRESH_V_CMD_ADDR          0x1F
#define PAC1932_SLOW_ADDR                   0X20
#define PAC1932_CTRL_ACT_ADDR               0X21
#define PAC1932_CHANNEL_DIS_ACT_ADDR        0X22 
#define PAC1932_NEG_PWR_ACT_ADDR            0X23
#define PAC1932_CTRL_LAT_ADDR               0X24
#define PAC1932_CHANNEL_DIS_LAT_ADDR        0X25
#define PAC1932_NEG_PWR_LAT_ADDR            0x26

#define PAC1932_PRODUCT_ID_ADDR             0xFD
#define PAC1932_MANUFACTURER_ID_ADDR        0xFE
#define PAC1932_REVISION_ID_ADDR            0xFF

typedef struct {
    uint8_t overflow_alert;
    uint8_t slow_status;
    uint8_t power_on_status;
    uint16_t sample_rate_lat;
    uint8_t product_id;
    uint8_t manufacturer_id;
    uint8_t revision_id;
    uint32_t refresh_timestamp;
    uint32_t rsense;
    int16_t error_code;
} Microchip_PAC193x;

// Function prototypes
void PAC193x_init(Microchip_PAC193x* dev, uint32_t resistor_value);
void PAC193x_begin(Microchip_PAC193x* dev);
void PAC193x_refresh(Microchip_PAC193x* dev);
int16_t PAC193x_update_overflow_alert(Microchip_PAC193x* dev);
int16_t PAC193x_update_slow_status(Microchip_PAC193x* dev);
int16_t PAC193x_update_power_on_status(Microchip_PAC193x* dev);
int16_t PAC193x_update_sample_rate_lat(Microchip_PAC193x* dev);
int16_t PAC193x_set_sample_rate(Microchip_PAC193x* dev, uint16_t value);
int16_t PAC193x_update_product_id(Microchip_PAC193x* dev);
int16_t PAC193x_update_manufacturer_id(Microchip_PAC193x* dev);
int16_t PAC193x_update_revision_id(Microchip_PAC193x* dev);

// Low-level I2C communication functions (to be implemented)
int16_t PAC193x_i2c_read(uint8_t reg_address, uint8_t* buffer, uint16_t length);
int16_t PAC193x_i2c_write(uint8_t reg_address, uint8_t* data, uint16_t length);

#endif // MICROCHIP_PAC193X_H
