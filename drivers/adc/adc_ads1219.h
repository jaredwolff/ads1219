
/*
 * Copyright (c) 2022 Circuit Dojo LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ADC_ADS1219_H_
#define ZEPHYR_DRIVERS_ADC_ADS1219_H_

#include <zephyr.h>

/* Commands */
#define ADC_ADS1219_RESET_CMD 0x06
#define ADC_ADS1219_START_SYNC_CMD 0x08
#define ADC_ADS1219_POWERDOWN_CMD 0x02
#define ADC_ADS1219_RDATA_CMD 0x10
#define ADC_ADS1219_RREG_CMD 0x20
#define ADC_ADS1219_WREG_CMD 0x40

/* Registers */
#define ADC_ADS1219_CONFIG_REG 0x00

#define ADC_ADS1219_STATUS_REG 0x04
#define ADC_ADS1219_STATUS_DRDY 0x80

union adc_ads1219_config_reg
{

    struct
    {
        uint8_t vref : 1;
        uint8_t cm : 1;
        uint8_t dr : 2;
        uint8_t gain : 1;
        uint8_t mux : 3;
    };
    uint8_t raw;
};

/* Driver data */
struct adc_ads1219_data
{
    const struct device *i2c;
    struct k_sem wait_sem;
};

int adc_ads1219_reset(const struct device *dev);
int adc_ads1219_power_down(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_ADC_ADS1219_H_ */