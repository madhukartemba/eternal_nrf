#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

/* BTHome v2 service-data format (https://bthome.io)
 *   uuid     : 0xFCD2 (BTHome SIG-assigned 16-bit UUID), little-endian
 *   info     : 0x40 = v2, unencrypted, not trigger-based
 *   0x01     : battery, uint8, % (derived from VDD: 2.20 V = 0%, 3.10 V = 100%)
 *   0x02     : temperature, int16 LE, 0.01 degC resolution
 *   0x03     : humidity, uint16 LE, 0.01 % resolution
 *   0x0F     : generic boolean, uint8 (button state)
 *   0x11     : opening, uint8 (magnet/contact state)
 */
#define BTHOME_UUID         0xFCD2U
#define BTHOME_INFO_V2      0x40U
#define BTHOME_OID_BATTERY  0x01U
#define BTHOME_OID_TEMP     0x02U
#define BTHOME_OID_HUM      0x03U
#define BTHOME_OID_BOOL     0x0FU
#define BTHOME_OID_OPENING  0x11U

/* Linear V->% mapping for the rail powering the nRF VDD pin. Placeholder
 * range; tune to the actual battery / supercap discharge curve once
 * characterised on hardware.
 */
#define BATT_MV_MIN         2200U
#define BATT_MV_MAX         3100U

#define LED0_PIN            10U
#define INPUT_PIN           12U
#define BUTTON_PIN          20U
#define SHT_EN_PIN          16U

#define SENSOR_PERIOD       K_MINUTES(10)
/* ~1500 ms at the default 100-150 ms fast-adv interval -> ~10-15 adv events
 * per burst. Longer than strictly needed for an ideal scanner, but gives
 * Home Assistant / windowed BLE scanners enough redundancy to avoid the
 * unavailability timeout when a burst is partially missed.
 */
#define ADV_BURST_MS        1500U
#define LED_FLASH_MS        30U

static struct {
	uint16_t uuid;
	uint8_t  info;
	uint8_t  oid_battery;
	uint8_t  battery_pct;
	uint8_t  oid_temp;
	int16_t  temp_centi_c;
	uint8_t  oid_hum;
	uint16_t hum_centi_pct;
	uint8_t  oid_button;
	uint8_t  button_pressed;
	uint8_t  oid_magnet;
	uint8_t  magnet_open;
} __packed svc = {
	.uuid        = sys_cpu_to_le16(BTHOME_UUID),
	.info        = BTHOME_INFO_V2,
	.oid_battery = BTHOME_OID_BATTERY,
	.oid_temp    = BTHOME_OID_TEMP,
	.oid_hum     = BTHOME_OID_HUM,
	.oid_button  = BTHOME_OID_BOOL,
	.oid_magnet  = BTHOME_OID_OPENING,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL),
	BT_DATA_BYTES(BT_DATA_NAME_COMPLETE,
		      'E', 't', 'e', 'r', 'n', 'a', 'l'),
	BT_DATA(BT_DATA_SVC_DATA16, (const uint8_t *)&svc, sizeof(svc)),
};

static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *const shtc3 = DEVICE_DT_GET(DT_NODELABEL(shtc3));
static const struct adc_dt_spec adc_vdd =
	ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static struct gpio_callback input_cb;
static struct gpio_callback button_cb;
static struct k_work        update_work;
static struct k_work_delayable adv_stop_work;
static struct k_work_delayable sensor_work;
static bool advertising;
static bool bt_ready;

/* Every GPIO we don't drive or sense.
 *
 * Includes both QCAA-exposed pins (4,5,6,9,18,25,28,30) and the die-pad
 * pins that aren't bonded out in the QCAA package (2,3,7,8,11,13,17,19,
 * 22,23,24,26,27,29,31). PIN_CNF still controls those die pads; disabling
 * their input buffers cuts any latent pad leakage. Skipped:
 *   0,1   - XL1/XL2 (LFXO, owned by the clock-control driver)
 *   10    - LED
 *   12    - DRV5032 input
 *   14,15 - I2C SCL/SDA (managed by pinctrl)
 *   16    - SHT_VCC
 *   20    - button
 *   21    - RESET
 */
static const uint8_t unused_pins[] = {
	2, 3, 4, 5, 6, 7, 8, 9, 11, 13,
	17, 18, 19, 22, 23, 24, 25, 26, 27, 28,
	29, 30, 31,
};

static int bt_setup(void)
{
	if (bt_ready) {
		return 0;
	}
	int err = bt_enable(NULL);
	if (err && err != -EALREADY) {
		printk("bt_enable failed: %d\n", err);
		return err;
	}
	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		(void)settings_load();
	}
	bt_ready = true;
	return 0;
}

static void bt_teardown(void)
{
	if (!bt_ready) {
		return;
	}
	(void)bt_disable();
	bt_ready = false;
}

static int read_shtc3_sample(int16_t *temp_centi_c, uint16_t *hum_centi_pct)
{
	struct sensor_value temp;
	struct sensor_value hum;
	int rc;

	/* Power-gate the SHTC3: drive SHT_EN high for the read, then low.
	 * Datasheet power-up time is ~240 us; 5 ms gives ample margin and
	 * the CPU just idles during k_sleep().
	 */
	(void)gpio_pin_set(gpio0, SHT_EN_PIN, 1);
	k_sleep(K_MSEC(5));

	rc = sensor_sample_fetch(shtc3);
	if (rc == 0) {
		rc = sensor_channel_get(shtc3, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	}
	if (rc == 0) {
		rc = sensor_channel_get(shtc3, SENSOR_CHAN_HUMIDITY, &hum);
	}

	(void)gpio_pin_set(gpio0, SHT_EN_PIN, 0);

	if (rc) {
		printk("shtc3: read failed (%d)\n", rc);
		return -1;
	}

	int32_t temp_micro = (temp.val1 * 1000000) + temp.val2;
	int32_t hum_micro  = (hum.val1 * 1000000) + hum.val2;

	*temp_centi_c = (int16_t)(temp_micro / 10000);
	if (hum_micro < 0) {
		hum_micro = 0;
	}
	*hum_centi_pct = (uint16_t)(hum_micro / 10000);
	return 0;
}

static uint8_t voltage_to_battery_pct(uint16_t mv)
{
	if (mv <= BATT_MV_MIN) {
		return 0U;
	}
	if (mv >= BATT_MV_MAX) {
		return 100U;
	}
	return (uint8_t)(((uint32_t)(mv - BATT_MV_MIN) * 100U) /
			(BATT_MV_MAX - BATT_MV_MIN));
}

static int read_vdd_millivolts(uint16_t *millivolts)
{
	int16_t raw = 0;
	struct adc_sequence seq = {
		.buffer      = &raw,
		.buffer_size = sizeof(raw),
	};

	int err = adc_sequence_init_dt(&adc_vdd, &seq);
	if (err) {
		return err;
	}
	err = adc_read(adc_vdd.dev, &seq);
	if (err) {
		return err;
	}

	int32_t mv = raw;
	err = adc_raw_to_millivolts_dt(&adc_vdd, &mv);
	if (err) {
		return err;
	}
	if (mv < 0) {
		mv = 0;
	}
	*millivolts = (uint16_t)mv;
	return 0;
}

static void update_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);

	int16_t  t = 0;
	uint16_t h = 0;
	uint16_t v = 0;

	if (read_shtc3_sample(&t, &h) == 0) {
		svc.temp_centi_c  = sys_cpu_to_le16(t);
		svc.hum_centi_pct = sys_cpu_to_le16(h);
	}
	if (read_vdd_millivolts(&v) == 0) {
		svc.battery_pct = voltage_to_battery_pct(v);
	}

	int magnet = gpio_pin_get(gpio0, INPUT_PIN);
	int button = gpio_pin_get(gpio0, BUTTON_PIN);

	svc.button_pressed = (button == 0) ? 1U : 0U;
	svc.magnet_open    = (magnet  > 0) ? 1U : 0U;

	printk("update: t=%d.%02d C  rh=%u.%02u %%  v=%u mV  batt=%u %%  mag=%s  btn=%s\n",
	       t / 100, (t < 0 ? -t : t) % 100,
	       h / 100, h % 100,
	       v, svc.battery_pct,
	       svc.magnet_open ? "open" : "closed",
	       svc.button_pressed ? "down" : "up");

	if (advertising) {
		(void)bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	} else {
		if (bt_setup() != 0) {
			return;
		}
		if (bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad,
				    ARRAY_SIZE(ad), NULL, 0)) {
			printk("adv_start failed\n");
			return;
		}
		advertising = true;
	}

	(void)k_work_reschedule(&adv_stop_work, K_MSEC(ADV_BURST_MS));
}

static void adv_stop_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	if (advertising) {
		(void)bt_le_adv_stop();
		advertising = false;
	}
	/* Brief "going to sleep" blink. Runs on the system workqueue so
	 * k_sleep() just yields the CPU to idle for the flash duration.
	 */
	(void)gpio_pin_set(gpio0, LED0_PIN, 1);
	k_sleep(K_MSEC(LED_FLASH_MS));
	(void)gpio_pin_set(gpio0, LED0_PIN, 0);

	/* Tear down the BLE controller for the long idle window. The next
	 * update_work will rebuild it via bt_setup(). At a 10-min cycle the
	 * one-shot enable cost (~30 ms HFCLK + flash read) is heavily
	 * dominated by the ~1-2 uA continuous saving while disabled.
	 */
	bt_teardown();
}

static void sensor_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	(void)k_work_submit(&update_work);
	(void)k_work_reschedule(&sensor_work, SENSOR_PERIOD);
}

/* Re-arm SENSE on a pin to fire on its NEXT transition. We use level-mode
 * interrupts (which map to GPIOTE PORT/SENSE on the nRF gpio driver, LFCLK-
 * based, ~0 uA idle) rather than edge-mode interrupts (which allocate a
 * GPIOTE channel in IN-event mode and burn ~5-6 uA continuously in System
 * ON LP). After each detection the callback flips the sense polarity, so
 * functionally this behaves identically to GPIO_INT_EDGE_BOTH.
 */
static void rearm_pin_sense(gpio_pin_t pin)
{
	int level = gpio_pin_get(gpio0, pin);
	(void)gpio_pin_interrupt_configure(gpio0, pin,
		level > 0 ? GPIO_INT_LEVEL_LOW : GPIO_INT_LEVEL_HIGH);
}

static void gpio_event_cb(const struct device *dev,
			  struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	if (pins & BIT(INPUT_PIN)) {
		rearm_pin_sense(INPUT_PIN);
	}
	if (pins & BIT(BUTTON_PIN)) {
		rearm_pin_sense(BUTTON_PIN);
	}
	(void)k_work_submit(&update_work);
}

int main(void)
{
	printk("nrf-sensor: boot\n");

	if (!device_is_ready(gpio0)) {
		printk("gpio0 not ready\n");
		return -1;
	}
	if (!device_is_ready(adc_vdd.dev)) {
		printk("adc not ready\n");
		return -1;
	}

	/* LED: drive low (off) by default. Only pulsed briefly after each
	 * BLE burst as a "going to sleep" indicator.
	 */
	if (gpio_pin_configure(gpio0, LED0_PIN, GPIO_OUTPUT_INACTIVE)) {
		printk("led pin config failed\n");
		return -1;
	}

	/* Power up the SHTC3 long enough to run device_init() (the driver
	 * does a wake + ID-check + sleep over I2C). The DT gpio-hog already
	 * holds SHT_EN high at boot; this re-asserts it explicitly so we
	 * own the pin from here on.
	 */
	if (gpio_pin_configure(gpio0, SHT_EN_PIN, GPIO_OUTPUT_ACTIVE)) {
		printk("sht_en config failed\n");
		return -1;
	}
	k_sleep(K_MSEC(20));

	if (device_init(shtc3)) {
		printk("shtc3 init failed\n");
		return -1;
	}
	if (!device_is_ready(shtc3)) {
		printk("shtc3 not ready\n");
		return -1;
	}

	/* Init complete; gate the sensor power off. read_shtc3_sample()
	 * re-asserts SHT_EN around each measurement.
	 */
	(void)gpio_pin_set(gpio0, SHT_EN_PIN, 0);

	if (adc_channel_setup_dt(&adc_vdd)) {
		printk("adc channel setup failed\n");
		return -1;
	}

	/* DRV5032 (push-pull variant) actively drives both rails; no internal
	 * pull needed, and adding one would just burn current fighting the
	 * driver whenever the output sits in the opposite state.
	 */
	if (gpio_pin_configure(gpio0, INPUT_PIN, GPIO_INPUT)) {
		printk("input pin config failed\n");
		return -1;
	}
	if (gpio_pin_configure(gpio0, BUTTON_PIN,
			       GPIO_INPUT | GPIO_PULL_UP)) {
		printk("button pin config failed\n");
		return -1;
	}

	gpio_init_callback(&input_cb,  gpio_event_cb, BIT(INPUT_PIN));
	gpio_init_callback(&button_cb, gpio_event_cb, BIT(BUTTON_PIN));
	gpio_add_callback(gpio0, &input_cb);
	gpio_add_callback(gpio0, &button_cb);

	/* Arm SENSE on the current pin state's opposite polarity. See
	 * rearm_pin_sense() above for why we avoid GPIO_INT_EDGE_*.
	 */
	rearm_pin_sense(INPUT_PIN);
	rearm_pin_sense(BUTTON_PIN);

	/* Disconnect every pin we don't drive or sense. Reset state is
	 * input-with-buffer-enabled, which can burn pad leakage if the pin
	 * floats around mid-rail.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(unused_pins); i++) {
		(void)gpio_pin_configure(gpio0, unused_pins[i],
					 GPIO_DISCONNECTED);
	}

	/* BLE controller is brought up on demand by update_work_handler ->
	 * bt_setup(), and torn down by adv_stop_handler -> bt_teardown().
	 * No need to call bt_enable() here.
	 */

	k_work_init(&update_work, update_work_handler);
	k_work_init_delayable(&adv_stop_work,  adv_stop_handler);
	k_work_init_delayable(&sensor_work,    sensor_work_handler);

	/* First publish, then arm the periodic sensor timer. The CPU stays
	 * in System ON low-power between events; wake sources are the
	 * GPIOTE PORT/SENSE detect on INPUT_PIN + BUTTON_PIN and the RTC
	 * tick driving the delayable workqueue.
	 */
	(void)k_work_submit(&update_work);
	(void)k_work_reschedule(&sensor_work, SENSOR_PERIOD);

	return 0;
}
