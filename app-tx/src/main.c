#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <esb.h>
#include <logging/log.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/adc.h>
#include <math.h>
#include <drivers/pwm.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_LABEL DT_PROP(ADC_NODE, label)

#define PWM_NODE DT_NODELABEL(pwm0)
#define PWM_LABEL DT_PROP(PWM_NODE, label)
#define PWM_CHANNEL_0 DT_PROP(PWM_NODE, ch0_pin)
#define PWM_CHANNEL_1 DT_PROP(PWM_NODE, ch1_pin)
#define PWM_CHANNEL_2 DT_PROP(PWM_NODE, ch2_pin)
#define PWM_CHANNEL_3 DT_PROP(PWM_NODE, ch3_pin)

static struct adc_channel_cfg adc_cfg[] = {
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 0,
		.input_positive = 2,
	},
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 1,
		.input_positive = 3,
	},
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 2,
		.input_positive = 5,
	},
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 3,
		.input_positive = 6,
	},
};

int16_t adc_buf[ARRAY_SIZE(adc_cfg)];

struct adc_sequence adc_seq = {
	.channels = 0,
	.buffer = adc_buf,
	.buffer_size = sizeof(adc_buf),
	.resolution = 12,
	.calibrate = true,
};

static struct k_poll_signal tx_event = K_POLL_SIGNAL_INITIALIZER(tx_event);
static struct k_poll_event events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &tx_event, 0),
};

static void event_handler(struct esb_evt const *event)
{
	struct esb_payload rx_payload;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		k_poll_signal_raise(&tx_event, 0);
		break;
	case ESB_EVENT_TX_FAILED:
		k_poll_signal_raise(&tx_event, -EAGAIN);
		break;
	case ESB_EVENT_RX_RECEIVED:
		esb_read_rx_payload(&rx_payload);
		break;
	}
}

static int clocks_start(void)
{
	int ret;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	ret = onoff_request(clk_mgr, &clk_cli);
	if (ret < 0) {
		LOG_ERR("Clock request failed: %d", ret);
		return ret;
	}

	do {
		ret = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!ret && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (ret);

	return 0;
}

static int esb_initialize(void)
{
	int ret;

	uint8_t base_addr_0[4] = { 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t base_addr_1[4] = { 0xC2, 0xC2, 0xC2, 0xC2 };
	uint8_t addr_prefix[8] = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

	struct esb_config config = ESB_DEFAULT_CONFIG;
	config.event_handler = event_handler;

	ret = esb_init(&config);
	__ASSERT(!ret, "esb_init: %d", ret);

	ret = esb_set_base_address_0(base_addr_0);
	__ASSERT(!ret, "esb_set_base_address_0: %d", ret);

	ret = esb_set_base_address_1(base_addr_1);
	__ASSERT(!ret, "esb_set_base_address_1: %d", ret);

	ret = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	__ASSERT(!ret, "esb_set_prefixes: %d", ret);

	return 0;
}

static void init_adc(const struct device *adc_dev)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(adc_cfg); i++) {
		ret = adc_channel_setup(adc_dev, &adc_cfg[i]);
		__ASSERT(!ret, "adc_channel_setup failed (%d): %d", i, ret);

		adc_seq.channels |= BIT(adc_cfg[i].channel_id);
	}
}

void main(void)
{
	int ret;

	ret = clocks_start();
	__ASSERT(!ret, "clocks_start: %d", ret);

	ret = esb_initialize();
	__ASSERT(!ret, "esb_initialize: %d", ret);

	const struct device *pwm_dev = device_get_binding(PWM_LABEL);
	__ASSERT_NO_MSG(pwm_dev);

	const struct device *adc_dev = device_get_binding(ADC_LABEL);
	__ASSERT_NO_MSG(adc_dev);

	init_adc(adc_dev);

	while (1) {
		ret = adc_read(adc_dev, &adc_seq);
		__ASSERT(!ret, "adc_read failed: %d", ret);

		uint32_t buffer[ARRAY_SIZE(adc_cfg)];
		for (int i = 0; i < ARRAY_SIZE(adc_cfg); ++i) {
			buffer[i] = adc_buf[i] < 0 ? 0 : adc_buf[i] > 4096 ? 4096 : adc_buf[i];
		}

		ret = pwm_pin_set_usec(pwm_dev, PWM_CHANNEL_0, BIT(12), buffer[0], 0);
		__ASSERT(!ret, "pwm_pin_set_usec failed: %d", ret);
		ret = pwm_pin_set_usec(pwm_dev, PWM_CHANNEL_1, BIT(12), buffer[2], 0);
		__ASSERT(!ret, "pwm_pin_set_usec failed: %d", ret);
		ret = pwm_pin_set_usec(pwm_dev, PWM_CHANNEL_2, BIT(12), buffer[1], 0);
		__ASSERT(!ret, "pwm_pin_set_usec failed: %d", ret);
		ret = pwm_pin_set_usec(pwm_dev, PWM_CHANNEL_3, BIT(12), buffer[3], 0);
		__ASSERT(!ret, "pwm_pin_set_usec failed: %d", ret);

		struct esb_payload tx_payload = {
			.pipe = 0,
			.length = sizeof(buffer),
		};
		memcpy(tx_payload.data, buffer, sizeof(buffer));

		k_poll_signal_reset(&tx_event);

		ret = esb_write_payload(&tx_payload);
		__ASSERT(!ret, "adc_read esb_write_payload: %d", ret);

		k_poll(events, ARRAY_SIZE(events), K_MSEC(20));
		if (events[0].signal->result) {
			esb_flush_tx();
		}
	}
}
