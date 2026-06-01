#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

/* BTHome v2 service-data format (https://bthome.io)
 *   uuid     : 0xFCD2 (BTHome SIG-assigned 16-bit UUID), little-endian
 *   info     : 0x40 = v2, unencrypted, not trigger-based
 *   0x02     : temperature, int16 LE, 0.01 degC resolution
 *   0x03     : humidity, uint16 LE, 0.01 % resolution
 *   0x0F     : generic boolean, uint8 (button state)
 *   0x11     : opening, uint8 (magnet/contact state)
 *
 * No battery object: this is a batteryless harvester (solar + supercap via
 * BQ25504). There is no meaningful "charge %" to report - the device simply
 * runs whenever the storage cap holds enough energy and browns out when it
 * doesn't. BTHome object IDs must stay in ascending order, which they are.
 */
#define BTHOME_UUID         0xFCD2U
#define BTHOME_INFO_V2      0x40U
#define BTHOME_OID_TEMP     0x02U
#define BTHOME_OID_HUM      0x03U
#define BTHOME_OID_BOOL     0x0FU
#define BTHOME_OID_OPENING  0x11U

#define LED0_PIN            10U
#define INPUT_PIN           12U
#define BUTTON_PIN          20U
#define SHT_EN_PIN          16U

/* Batteryless duty cycle.
 *
 * There is no deep-sleep schedule: in an energy-harvesting design you spend
 * energy as soon as you have it, because you can't assume you'll still be
 * powered in N minutes. Each pass through the main loop:
 *   1. read the sensor + contact/button pins,
 *   2. fire a short advertising burst of ~3-4 events so a windowed scanner
 *      (Home Assistant) reliably catches at least one,
 *   3. flash the LED briefly as an "alive" heartbeat, then idle for the
 *      rest of IDLE_PERIOD,
 *   4. loop and transmit again if the MCU is still powered.
 *
 * ~450 ms at the 100-150 ms NCONN_IDENTITY fast interval yields ~3-4
 * advertising events per burst.
 */
#define ADV_BURST_MS        450U
#define LED_FLASH_MS        30U
#define IDLE_PERIOD         K_MSEC(3000)

static struct {
	uint16_t uuid;
	uint8_t  info;
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

/* One publish cycle: read sensors + pins, fire a short advertising burst,
 * then hold the LED solid for the idle window. Returns after IDLE_PERIOD;
 * if the cap browns out at any point the MCU simply resets and main()
 * starts the cycle over from scratch.
 */
static void publish_cycle(void)
{
	int16_t  t = 0;
	uint16_t h = 0;

	if (read_shtc3_sample(&t, &h) == 0) {
		svc.temp_centi_c  = sys_cpu_to_le16(t);
		svc.hum_centi_pct = sys_cpu_to_le16(h);
	}

	int magnet = gpio_pin_get(gpio0, INPUT_PIN);
	int button = gpio_pin_get(gpio0, BUTTON_PIN);

	svc.button_pressed = (button == 0) ? 1U : 0U;
	svc.magnet_open    = (magnet  > 0) ? 1U : 0U;

	printk("publish: t=%d.%02d C  rh=%u.%02u %%  mag=%s  btn=%s\n",
	       t / 100, (t < 0 ? -t : t) % 100,
	       h / 100, h % 100,
	       svc.magnet_open ? "open" : "closed",
	       svc.button_pressed ? "down" : "up");

	/* Burst: start fresh each cycle so the just-read data goes out, run
	 * for ~3-4 advertising events, then stop the radio.
	 */
	if (bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad,
			    ARRAY_SIZE(ad), NULL, 0) == 0) {
		k_sleep(K_MSEC(ADV_BURST_MS));
		(void)bt_le_adv_stop();
	} else {
		printk("adv_start failed\n");
	}

	/* Brief LED heartbeat to show the burst went out, then idle for the
	 * rest of the window. If the storage cap can no longer power the MCU
	 * (brownout) the device just resets and starts over once it recharges.
	 */
	(void)gpio_pin_set(gpio0, LED0_PIN, 1);
	k_sleep(K_MSEC(LED_FLASH_MS));
	(void)gpio_pin_set(gpio0, LED0_PIN, 0);
	k_sleep(IDLE_PERIOD);
}

int main(void)
{
	printk("nrf-sensor (batteryless): boot\n");

	if (!device_is_ready(gpio0)) {
		printk("gpio0 not ready\n");
		return -1;
	}

	/* LED: drive low (off) by default. Held solid during each idle window. */
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

	/* Disconnect every pin we don't drive or sense. Reset state is
	 * input-with-buffer-enabled, which can burn pad leakage if the pin
	 * floats around mid-rail.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(unused_pins); i++) {
		(void)gpio_pin_configure(gpio0, unused_pins[i],
					 GPIO_DISCONNECTED);
	}

	/* Bring the BLE controller up once. With CONFIG_BT_SETTINGS the
	 * identity (stable MAC) is loaded from flash so Home Assistant keeps
	 * tracking the same device across the frequent power cycles a
	 * harvester sees. We never tear it down: at a ~3 s cadence the
	 * one-shot enable cost would dominate, and re-enabling on every cycle
	 * means re-reading flash each time.
	 */
	if (bt_setup() != 0) {
		return -1;
	}

	/* Spend energy as it arrives: no sleep schedule, just publish, hold
	 * the LED, and publish again for as long as the cap keeps us alive.
	 */
	while (1) {
		publish_cycle();
	}

	return 0;
}
