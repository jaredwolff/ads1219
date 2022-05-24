#
# Copyright (c) 2022 Circuit Dojo LLC
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ADC_ADS1219 adc_ads1219.c)