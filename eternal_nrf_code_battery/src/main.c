#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>

/* BTHome v2 service-data format (https://bthome.io). Objects must be in
 * ascending object-ID order.
 *   uuid     : 0xFCD2 (BTHome SIG-assigned 16-bit UUID), little-endian
 *   info     : 0x40 = v2, unencrypted, not trigger-based
 *   0x00     : packet id, uint8. Incremented on every payload change; HA
 *              drops repeats, so a 1.5 s burst is one update / one event.
 *   0x01     : battery, uint8, % (per-cell discharge curve, resting VDD)
 *   0x02     : temperature, int16 LE, 0.01 degC resolution
 *   0x03     : humidity, uint16 LE, 0.01 % resolution
 *   0x11     : opening, uint8 (magnet/contact state)
 *   0x16     : battery charging, uint8 (from the resting-VDD trend)
 *   0x3A     : button event, uint8 (0 none, 1 press, 2 double, 3 triple,
 *              4 long press). Replaces the old 0x0F generic-bool state.
 *
 * Total advertisement: flags 3 + name "MT" 4 + service data 21 = 28 of
 * the 31 legacy-advertising bytes.
 */
#define BTHOME_UUID         0xFCD2U
#define BTHOME_INFO_V2      0x40U
#define BTHOME_OID_PACKET   0x00U
#define BTHOME_OID_BATTERY  0x01U
#define BTHOME_OID_TEMP     0x02U
#define BTHOME_OID_HUM      0x03U
#define BTHOME_OID_OPENING  0x11U
#define BTHOME_OID_CHARGING 0x16U
#define BTHOME_OID_BUTTON   0x3AU

#define BTN_EVT_NONE        0x00U
#define BTN_EVT_PRESS       0x01U
#define BTN_EVT_DOUBLE      0x02U
#define BTN_EVT_TRIPLE      0x03U
#define BTN_EVT_LONG        0x04U

/* Button timing. Long press fires while still held so it feels immediate. */
#define BTN_DEBOUNCE_MS     20U
#define BTN_MULTI_GAP_MS    350U
#define BTN_LONG_MS         1000U

/* Resting-VDD discharge curve (mV -> %), piecewise linear between points.
 * Approximation of the datasheet curve at uA-level loads; tune against
 * real logs. Readings taken while the radio is active are ignored for the
 * battery %, since TX sag on these small cells is tens of mV.
 */
struct curve_pt {
	uint16_t mv;
	uint8_t  pct;
};

#if defined(ETERNAL_CELL_CR2032)
/* CR2032 primary cell (e.g. the main-door unit). Build with
 *   west build -d build_cr2032 -- -DEXTRA_CFLAGS=-DETERNAL_CELL_CR2032
 * Long flat plateau ~2.9-3.0 V at uA loads, knee below ~2.7 V. No charging
 * object is sent: the cell can't charge.
 */
static const struct curve_pt batt_curve[] = {
	{3000, 100}, {2900, 80}, {2800, 60}, {2700, 40},
	{2600, 25},  {2500, 15}, {2400, 8},  {2200, 0},
};
#else
/* ML621 (Mn-Li, charged to ~3.1 V by the BQ25504 VBAT_OV setting): drops
 * quickly off full charge, then slopes steadily through 2.9-2.5 V.
 */
static const struct curve_pt batt_curve[] = {
	{3100, 100}, {3000, 95}, {2900, 85}, {2800, 70}, {2700, 50},
	{2600, 32},  {2500, 18}, {2400, 9},  {2300, 4},  {2200, 0},
};
#define ETERNAL_TRACK_CHARGING 1
#endif

#if defined(ETERNAL_TRACK_CHARGING)
/* Charging = smoothed resting VDD rising over the last CHG_WINDOW
 * heartbeats (40 min at a 10-min period). Thresholds in mV; hysteresis
 * stops it flapping on ADC noise (~0.9 mV/LSB at gain 1/6).
 */
#define CHG_WINDOW          4U
#define CHG_ON_MV           6
#define CHG_OFF_MV          0
#endif

#define LED0_PIN            10U
#define INPUT_PIN           12U
#define BUTTON_PIN          20U
#define SHT_EN_PIN          16U

/* Periodic heartbeat (temp/humidity/battery). Kept under Home Assistant's
 * 15-min BLE stale-advertisement fallback so one missed burst doesn't mark
 * the device unavailable. Was 5 min; the periodic bursts dominated the
 * energy budget on low-light (back door) units.
 */
#define SENSOR_PERIOD       K_MINUTES(10)
/* Interval matches nrf_doorbell: 20-30 ms (0x0020/0x0030 in 0.625 ms units)
 * instead of BT_LE_ADV_NCONN_IDENTITY's 100-150 ms. TX is +4 dBm (see
 * prj.conf).
 *
 * Door/button events get the long burst (~50-75 events) because a missed
 * open/close matters. Periodic heartbeats get a short burst (~10-15 events):
 * a lost one is simply replaced by the next. Together with the 10-min period
 * this cuts periodic radio-on time ~10x (288 x 1.5 s -> 144 x 0.3 s per day).
 */
#define ADV_BURST_EVENT_MS     1500U
#define ADV_BURST_PERIODIC_MS  300U
#define LED_FLASH_MS        30U

/* BLE advertising interval units are 0.625 ms; 0x0020=20 ms, 0x0030=30 ms. */
static const struct bt_le_adv_param adv_param = {
	.id = BT_ID_DEFAULT,
	.sid = 0,
	.secondary_max_skip = 0,
	.options = BT_LE_ADV_OPT_USE_IDENTITY,
	.interval_min = 0x0020,
	.interval_max = 0x0030,
	.peer = NULL,
};

static struct {
	uint16_t uuid;
	uint8_t  info;
	uint8_t  oid_packet;
	uint8_t  packet_id;
	uint8_t  oid_battery;
	uint8_t  battery_pct;
	uint8_t  oid_temp;
	int16_t  temp_centi_c;
	uint8_t  oid_hum;
	uint16_t hum_centi_pct;
	uint8_t  oid_magnet;
	uint8_t  magnet_open;
#if defined(ETERNAL_TRACK_CHARGING)
	uint8_t  oid_charging;
	uint8_t  charging;
#endif
	uint8_t  oid_button;
	uint8_t  button_event;
} __packed svc = {
	.uuid         = sys_cpu_to_le16(BTHOME_UUID),
	.info         = BTHOME_INFO_V2,
	.oid_packet   = BTHOME_OID_PACKET,
	.oid_battery  = BTHOME_OID_BATTERY,
	.oid_temp     = BTHOME_OID_TEMP,
	.oid_hum      = BTHOME_OID_HUM,
	.oid_magnet   = BTHOME_OID_OPENING,
#if defined(ETERNAL_TRACK_CHARGING)
	.oid_charging = BTHOME_OID_CHARGING,
#endif
	.oid_button   = BTHOME_OID_BUTTON,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL),
	BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'M', 'T'),
	BT_DATA(BT_DATA_SVC_DATA16, (const uint8_t *)&svc, sizeof(svc)),
};

/* Legacy advertising payload limit. */
BUILD_ASSERT((3 + 4 + 2 + sizeof(svc)) <= 31, "advertisement exceeds 31 bytes");

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
/* Set by GPIO ISR / boot to request the long event burst; consumed by
 * update_work_handler. Periodic updates leave it clear.
 */
static atomic_t event_pending;
/* Uptime (ms) at which the current advertising burst is due to stop. */
static int64_t adv_end_ms;
/* Button event resolved by the button state machine, waiting to be sent. */
static atomic_t button_event_pending;

static struct k_work_delayable btn_debounce_work;
static struct k_work_delayable btn_resolve_work;
static struct k_work_delayable btn_long_work;
static bool    btn_down;        /* debounced state */
static uint8_t btn_clicks;
static bool    btn_long_fired;

#if defined(ETERNAL_TRACK_CHARGING)
static int32_t chg_ema_mv16;    /* EMA of resting VDD, mV * 16 */
static int32_t chg_hist[CHG_WINDOW];
static uint8_t chg_count;
static uint8_t chg_idx;
#endif

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
	const size_t n = ARRAY_SIZE(batt_curve);

	if (mv >= batt_curve[0].mv) {
		return batt_curve[0].pct;
	}
	for (size_t i = 1; i < n; i++) {
		const struct curve_pt *hi = &batt_curve[i - 1];
		const struct curve_pt *lo = &batt_curve[i];

		if (mv >= lo->mv) {
			return (uint8_t)(lo->pct +
				((uint32_t)(mv - lo->mv) * (hi->pct - lo->pct)) /
				(hi->mv - lo->mv));
		}
	}
	return batt_curve[n - 1].pct;
}

#if defined(ETERNAL_TRACK_CHARGING)
/* Feed one resting VDD sample per heartbeat; returns the charging flag. */
static uint8_t charge_track(uint16_t mv)
{
	int32_t s = (int32_t)mv * 16;

	if (chg_count == 0U) {
		chg_ema_mv16 = s;
	} else {
		chg_ema_mv16 += (s - chg_ema_mv16) / 4;	/* alpha = 1/4 */
	}

	int32_t oldest = chg_hist[chg_idx];

	chg_hist[chg_idx] = chg_ema_mv16;
	chg_idx = (chg_idx + 1U) % CHG_WINDOW;
	if (chg_count < CHG_WINDOW) {
		chg_count++;
		return 0U;	/* not enough history yet */
	}

	int32_t delta_mv = (chg_ema_mv16 - oldest) / 16;

	if (delta_mv >= CHG_ON_MV) {
		return 1U;
	}
	if (delta_mv <= CHG_OFF_MV) {
		return 0U;
	}
	return svc.charging;	/* hysteresis band: hold */
}
#endif

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

	bool is_event = atomic_clear(&event_pending) != 0;

	if (read_shtc3_sample(&t, &h) == 0) {
		svc.temp_centi_c  = sys_cpu_to_le16(t);
		svc.hum_centi_pct = sys_cpu_to_le16(h);
	}

	/* Only trust VDD while the radio is idle; mid-burst readings sag. */
	if (!advertising && read_vdd_millivolts(&v) == 0) {
		svc.battery_pct = voltage_to_battery_pct(v);
#if defined(ETERNAL_TRACK_CHARGING)
		if (!is_event) {
			svc.charging = charge_track(v);
		}
#endif
	}

	int magnet = gpio_pin_get(gpio0, INPUT_PIN);

	svc.magnet_open  = (magnet > 0) ? 1U : 0U;
	svc.button_event = (uint8_t)atomic_clear(&button_event_pending);
	svc.packet_id++;

	printk("update: pkt=%u t=%d.%02d C  rh=%u.%02u %%  v=%u mV  batt=%u %%  mag=%s  btn_evt=%u\n",
	       svc.packet_id,
	       t / 100, (t < 0 ? -t : t) % 100,
	       h / 100, h % 100,
	       v, svc.battery_pct,
	       svc.magnet_open ? "open" : "closed",
	       svc.button_event);

	uint32_t burst_ms = is_event ? ADV_BURST_EVENT_MS : ADV_BURST_PERIODIC_MS;
	int64_t  end_ms   = k_uptime_get() + burst_ms;

	if (advertising) {
		(void)bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	} else {
		if (bt_setup() != 0) {
			return;
		}
		if (bt_le_adv_start(&adv_param, ad,
				    ARRAY_SIZE(ad), NULL, 0)) {
			printk("adv_start failed\n");
			return;
		}
		advertising = true;
	}

	/* The host copied the payload above. Clear the one-shot button event
	 * locally so the next packet id doesn't carry it again.
	 */
	svc.button_event = BTN_EVT_NONE;

	/* Never shorten a burst already in flight (e.g. a heartbeat landing
	 * inside a door-event burst); only extend it.
	 */
	if (end_ms <= adv_end_ms && k_work_delayable_is_pending(&adv_stop_work)) {
		return;
	}

	adv_end_ms = end_ms;
	(void)k_work_reschedule(&adv_stop_work, K_MSEC(burst_ms));
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

/* Button state machine (system workqueue). The ISR only kicks the
 * debounce timer; edges are processed once the pin has been stable for
 * BTN_DEBOUNCE_MS. Only resolved events (press / double / triple / long)
 * are sent, so a click costs one event burst instead of one per edge.
 */
static void btn_emit(uint8_t evt)
{
	atomic_set(&button_event_pending, evt);
	atomic_set(&event_pending, 1);
	(void)k_work_submit(&update_work);
}

static void btn_long_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	if (btn_down) {
		btn_long_fired = true;
		btn_clicks = 0U;
		(void)k_work_cancel_delayable(&btn_resolve_work);
		btn_emit(BTN_EVT_LONG);
	}
}

static void btn_resolve_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	uint8_t n = btn_clicks;

	btn_clicks = 0U;
	if (n == 0U) {
		return;
	}
	btn_emit(n == 1U ? BTN_EVT_PRESS :
		 n == 2U ? BTN_EVT_DOUBLE : BTN_EVT_TRIPLE);
}

static void btn_debounce_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	bool down = (gpio_pin_get(gpio0, BUTTON_PIN) == 0);	/* active low */

	if (down == btn_down) {
		return;		/* bounce settled back to the same state */
	}
	btn_down = down;

	if (down) {
		btn_long_fired = false;
		(void)k_work_cancel_delayable(&btn_resolve_work);
		(void)k_work_reschedule(&btn_long_work, K_MSEC(BTN_LONG_MS));
	} else {
		(void)k_work_cancel_delayable(&btn_long_work);
		if (btn_long_fired) {
			return;
		}
		if (btn_clicks < 3U) {
			btn_clicks++;
		}
		(void)k_work_reschedule(&btn_resolve_work,
					K_MSEC(BTN_MULTI_GAP_MS));
	}
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

	if (pins & BIT(BUTTON_PIN)) {
		rearm_pin_sense(BUTTON_PIN);
		(void)k_work_reschedule(&btn_debounce_work,
					K_MSEC(BTN_DEBOUNCE_MS));
	}
	if (pins & BIT(INPUT_PIN)) {
		rearm_pin_sense(INPUT_PIN);
		atomic_set(&event_pending, 1);
		(void)k_work_submit(&update_work);
	}
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

	k_work_init(&update_work, update_work_handler);
	k_work_init_delayable(&adv_stop_work,     adv_stop_handler);
	k_work_init_delayable(&sensor_work,       sensor_work_handler);
	k_work_init_delayable(&btn_debounce_work, btn_debounce_handler);
	k_work_init_delayable(&btn_resolve_work,  btn_resolve_handler);
	k_work_init_delayable(&btn_long_work,     btn_long_handler);
	btn_down = (gpio_pin_get(gpio0, BUTTON_PIN) == 0);

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

	/* First publish, then arm the periodic sensor timer. The CPU stays
	 * in System ON low-power between events; wake sources are the
	 * GPIOTE PORT/SENSE detect on INPUT_PIN + BUTTON_PIN and the RTC
	 * tick driving the delayable workqueue.
	 */
	atomic_set(&event_pending, 1);	/* boot publish: use the long burst */
	(void)k_work_submit(&update_work);
	(void)k_work_reschedule(&sensor_work, SENSOR_PERIOD);

	return 0;
}
