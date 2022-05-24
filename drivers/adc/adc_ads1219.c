
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/adc.h>
#include <drivers/i2c.h>

#include <adc_ads1219.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_ads1219);

#define DT_DRV_COMPAT ti_ads1219

static int start_sync(const struct device *i2c)
{

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_START_SYNC_CMD};
	int ret = i2c_write(i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

int adc_ads1219_power_down(const struct device *dev)
{

	struct adc_ads1219_data *data = dev->data;

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_POWERDOWN_CMD};
	int ret = i2c_write(data->i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

static int read_data(const struct device *i2c, uint8_t *data)
{

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_RDATA_CMD};
	int ret = i2c_write(i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	/* Then read! */
	ret = i2c_read(i2c, data, 3,
				   DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to read register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

static int read_register(const struct device *i2c, uint8_t reg, uint8_t *data)
{

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_RREG_CMD | reg};
	int ret = i2c_write(i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	/* Then read! */
	ret = i2c_read(i2c, data, 1,
				   DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to read register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

static int write_config_register(const struct device *i2c, uint8_t data)
{

	LOG_INF("Writing %x to config.", data);

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_WREG_CMD, data};
	int ret = i2c_write(i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

/* Implementation of the ADC driver API function: adc_channel_setup. */
static int adc_ads1219_channel_setup(const struct device *dev,
									 const struct adc_channel_cfg *channel_cfg)
{
	struct adc_ads1219_data *data = dev->data;
	union adc_ads1219_config_reg cfg = {0};

	/* Select the gain */
	switch (channel_cfg->gain)
	{
	case ADC_GAIN_1:
		cfg.gain = 0;
		break;
	case ADC_GAIN_4:
		cfg.gain = 1;
		break;
	default:
		LOG_ERR("Invalid gain: %i", channel_cfg->gain);
		return -EINVAL;
	}

	/* Select the reference */
	switch (channel_cfg->reference)
	{
	case ADC_REF_INTERNAL:
		cfg.vref = 0;
		break;
	case ADC_REF_EXTERNAL0:
		cfg.vref = 1;
		break;
	default:
		LOG_ERR("Invalid reference: %i", channel_cfg->reference);
		return -EINVAL;
	}

	/* Select channel */
	if (channel_cfg->channel_id > 7)
	{
		LOG_ERR("Invalid channel id: %i", channel_cfg->channel_id);
		return -EINVAL;
	}
	else
	{
		cfg.mux = channel_cfg->channel_id;
	}

	/* Aquisition time/data rate*/
	switch (channel_cfg->acquisition_time)
	{
	case 20: /* 20 Samples per sec */
		cfg.dr = 0;
		break;
	case 90: /* 90 Samples per sec */
		cfg.dr = 1;
		break;
	case 330: /* 330 Samples per sec */
		cfg.dr = 2;
		break;
	case 1000: /* 1000 Samples per sec */
		cfg.dr = 3;
		break;
	default:
		LOG_ERR("Invalid data rate: %i", channel_cfg->acquisition_time);
	}

	/* Write the register */
	int err = write_config_register(data->i2c, cfg.raw);
	if (err)
	{
		LOG_ERR("Unable to write config register. Err: %i", err);
		return err;
	}

	return 0;
}

/* Implementation of the ADC driver API function: adc_read. */
static int adc_ads1219_read(const struct device *dev,
							const struct adc_sequence *sequence)
{
	int err = 0;

	struct adc_ads1219_data *data = dev->data;

	/* Start one shot..  */
	err = start_sync(data->i2c);
	if (err)
	{
		LOG_ERR("unable to start err: %i", err);
		return err;
	}

	while (true)
	{
		uint8_t reg = 0;
		err = read_register(data->i2c, ADC_ADS1219_STATUS_REG, &reg);
		if (err)
		{
			LOG_ERR("Unable to read status register. Err: %i", err);
			return err;
		}

		LOG_DBG("Ready: %s", reg & ADC_ADS1219_STATUS_DRDY ? "true" : "false");

		if (reg & ADC_ADS1219_STATUS_DRDY)
		{
			break;
		}

		k_sleep(K_MSEC(100));
	}

	if (sequence->buffer_size != sizeof(int32_t))
	{
		LOG_ERR("Invalid buffer size. Should be sizeof(int32_t).");
		return -EINVAL;
	}

	uint8_t buf[3] = {0};
	int32_t *raw = (int32_t *)sequence->buffer;

	/* Read data */
	err = read_data(data->i2c, buf);
	if (err)
	{
		LOG_ERR("Unable to read data. Err: %i", err);
		return err;
	}

	LOG_DBG("Buf value: %x %x %x", buf[0], buf[1], buf[2]);

	/* If negative need to fill the MSB in int32 */
	if (buf[0] & 0x80)
	{
		*raw = 0xff << 24;
	}

	/* Fill the rest */
	*raw += (buf[0] << 16) + (buf[1] << 8) + buf[2];

	return 0;
}

int adc_ads1219_reset(const struct device *dev)
{

	struct adc_ads1219_data *data = dev->data;

	/* Write the RREG command first */
	uint8_t cmd[] = {ADC_ADS1219_RESET_CMD};
	int ret = i2c_write(data->i2c, cmd, sizeof(cmd), DT_REG_ADDR(DT_DRV_INST(0)));
	if (ret)
	{
		LOG_ERR("Failed to set  register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

static int init_adc(const struct device *dev)
{

	struct adc_ads1219_data *data = dev->data;

	/* Get the i2c device binding*/
	data->i2c = device_get_binding(DT_BUS_LABEL(DT_DRV_INST(0)));

	/* Set I2C Device. */
	if (data->i2c == NULL || !device_is_ready(data->i2c))
	{
		LOG_ERR("Failed to get pointer to %s device!",
				DT_BUS_LABEL(DT_DRV_INST(0)));
		return -EINVAL;
	}

	/* Initialize semaphore */
	k_sem_init(&data->wait_sem, 0, 1);

	uint8_t reg = 0;
	int err = read_register(data->i2c, ADC_ADS1219_CONFIG_REG, &reg);
	if (err)
	{
		LOG_ERR("Unable to read config register. Err: %i", err);
		return err;
	}
	else
	{
		LOG_INF("Configuration register: %x", reg);
	}

	err = read_register(data->i2c, ADC_ADS1219_STATUS_REG, &reg);
	if (err)
	{
		LOG_ERR("Unable to read status register. Err: %i", err);
		return err;
	}
	else
	{
		LOG_INF("Status register: %x", reg);
	}

	return 0;
}

static const struct adc_driver_api adc_ads1219_driver_api = {
	.channel_setup = adc_ads1219_channel_setup,
	.read = adc_ads1219_read,
	.ref_internal = 2048,
};

#define ADC_INIT(inst)                                              \
	static struct adc_ads1219_data adc_ads1219_data_##inst;         \
	DEVICE_DT_INST_DEFINE(0,                                        \
						  init_adc, NULL, &adc_ads1219_data_##inst, \
						  NULL, POST_KERNEL,                        \
						  CONFIG_I2C_INIT_PRIORITY,                 \
						  &adc_ads1219_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_INIT)
