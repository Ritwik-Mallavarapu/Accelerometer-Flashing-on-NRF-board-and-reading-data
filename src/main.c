/*
 * =====================================================================
 * TYRE SENSOR FIRMWARE - v10  (FINAL - signed decode)
 * nRF52810 (IndieSemiC EVK-ISC-nRF52810-A) + GY-45 / MMA8452Q
 * =====================================================================
 *
 * Save as:  C:\ncs\projects\tyre_sensor\src\main.c
 *
 * ---------------------------------------------------------------------
 * THE BUG (found by tracing your recorded data)
 * ---------------------------------------------------------------------
 * The MMA8452Q returns 12-bit data LEFT JUSTIFIED in a 16-bit pair:
 *
 *      msb       lsb
 *    D11..D4   D3..D0 0000
 *
 * Correct decode: read the 16-bit field as SIGNED, then divide by 16.
 *
 * Every earlier version read it as UNSIGNED and then divided. Positive
 * values were fine. Negative values wrapped to 0x8000, and
 * 0x8000 / 16 = 2048 counts = exactly the "8.00 g" seen in the data.
 *
 * Proof, from the recorded bytes:
 *
 *    0xFF00 -> unsigned 4080 (bogus)   signed -16  counts (-0.06 g)
 *    0xF800 -> unsigned 3968 (bogus)   signed -128 counts (-0.50 g)
 *    0x8010 -> unsigned 2049 (bogus)   signed -2047 counts (-8.0 g)
 *    0x7FF0 ->          2047 (ok)      signed  2047 counts (+8.0 g)
 *
 * X axis only survived because it happened to stay positive.
 *
 * ---------------------------------------------------------------------
 * THE FIX
 * ---------------------------------------------------------------------
 *   1. Sign-extend the 16-bit field BEFORE dividing.
 *   2. No clamping. A bad value must LOOK bad, not be disguised as a
 *      plausible 8.00 g. Clamping was a mistake in v9.
 *   3. All three axes go through ONE helper - they cannot diverge.
 *
 * ---------------------------------------------------------------------
 * DESIGN (verified working - unchanged)
 * ---------------------------------------------------------------------
 *   Sensor sampling   : 200 Hz
 *   Averaging         : 10 raw -> 1 value   (low-pass ~10 Hz)
 *   Reported rate     : 20 Hz
 *   Values per packet : 2
 *   Packet size       : 16 bytes            (fits default ATT payload)
 *   Packet rate       : 10 per second
 *
 * A BUILD_ASSERT refuses to compile if the packet ever grows past the
 * 20-byte default ATT payload.
 *
 * ---------------------------------------------------------------------
 * DIAGNOSTIC CHARACTERISTIC (UUID ...0003)
 * ---------------------------------------------------------------------
 *   V10 who=2A err=0 i2c=1 rd=1 cptr=1 pkt=N smp=N avg=N nfail=0 nrc=00
 * =====================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(tyre, LOG_LEVEL_INF);

/* ================================================================== */
/* Sensor                                                             */
/* ================================================================== */

#define MMA8452Q_ADDR            0x1C
#define MMA8452Q_WHO_AM_I        0x0D
#define MMA8452Q_WHO_AM_I_VALUE  0x2A
#define REG_OUT_X_MSB            0x01
#define REG_XYZ_DATA_CFG         0x0E
#define REG_CTRL_REG1            0x2A

#define CTRL_REG1_ACTIVE         0x01
#define CTRL_REG1_DR_200HZ       0x28

/* ================================================================== */
/* Timing and packet geometry                                         */
/* ================================================================== */

#define SAMPLE_PERIOD_MS    5
#define DECIMATE            10
#define REPORT_PERIOD_MS    (SAMPLE_PERIOD_MS * DECIMATE)          /* 50 ms */

#define VALUES_PER_PACKET   2
#define NOTIFY_PERIOD_MS    (REPORT_PERIOD_MS * VALUES_PER_PACKET) /* 100 ms */

#define HEADER_BYTES        4
#define AXIS_BYTES          6
#define PACKET_BYTES        (HEADER_BYTES + (VALUES_PER_PACKET * AXIS_BYTES))
#define MAX_PAYLOAD         20

BUILD_ASSERT(PACKET_BYTES <= MAX_PAYLOAD,
	     "packet exceeds default ATT payload");

#define DIAG_LEN            110

/* ================================================================== */
/* BLE UUIDs                                                          */
/* ================================================================== */

#define BT_UUID_TYRE_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x6b1f0001, 0x7a3c, 0x4d2e, 0x9f10, 0x8a5c1e200001)
#define BT_UUID_TYRE_DATA_VAL \
	BT_UUID_128_ENCODE(0x6b1f0002, 0x7a3c, 0x4d2e, 0x9f10, 0x8a5c1e200002)
#define BT_UUID_TYRE_DIAG_VAL \
	BT_UUID_128_ENCODE(0x6b1f0003, 0x7a3c, 0x4d2e, 0x9f10, 0x8a5c1e200003)

/* ================================================================== */
/* State                                                              */
/* ================================================================== */

static const struct device *i2c_dev;
static struct bt_conn *current_conn;

static uint16_t packet_seq;
static uint8_t  packet[PACKET_BYTES];

static int32_t  acc_x, acc_y, acc_z;
static uint8_t  acc_count;
static uint8_t  values_ready;

static uint8_t  who_value;
static uint8_t  last_err;
static uint8_t  i2c_ok;
static uint8_t  read_ok;
static uint32_t packets_sent;
static uint32_t samples_taken;
static uint32_t averages_made;
static uint32_t notify_fails;
static uint8_t  last_notify_rc;

static char diag[DIAG_LEN];

/* ================================================================== */
/* Diagnostic characteristic                                          */
/* ================================================================== */

static ssize_t diag_read(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	snprintf(diag, sizeof(diag),
		 "V10 who=%02X err=%u i2c=%u rd=%u cptr=%u pkt=%u smp=%u "
		 "avg=%u nfail=%u nrc=%02X",
		 who_value, last_err, i2c_ok, read_ok,
		 (current_conn != NULL) ? 1 : 0,
		 (unsigned)packets_sent,
		 (unsigned)samples_taken,
		 (unsigned)averages_made,
		 (unsigned)notify_fails,
		 (unsigned)last_notify_rc);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, diag, strlen(diag));
}

/* ================================================================== */
/* Sensor                                                             */
/* ================================================================== */

static int sensor_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t b[2] = { reg, value };

	return i2c_write(i2c_dev, b, sizeof(b), MMA8452Q_ADDR);
}

static int sensor_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
	return i2c_write_read(i2c_dev, MMA8452Q_ADDR, &reg, 1, out, len);
}

static void sensor_init(void)
{
	uint8_t who = 0xFF;
	int ret;

	last_err = 0;

	ret = sensor_read_regs(MMA8452Q_WHO_AM_I, &who, 1);
	who_value = who;
	if (ret) {
		last_err = 2;
		return;
	}
	if (who != MMA8452Q_WHO_AM_I_VALUE) {
		last_err = 3;
		return;
	}

	if (sensor_write_reg(REG_CTRL_REG1, 0x00))                   { last_err = 4; return; }
	if (sensor_write_reg(REG_XYZ_DATA_CFG, 0x02))                { last_err = 5; return; }
	if (sensor_write_reg(REG_CTRL_REG1, CTRL_REG1_DR_200HZ))     { last_err = 6; return; }
	if (sensor_write_reg(REG_CTRL_REG1,
			     CTRL_REG1_DR_200HZ | CTRL_REG1_ACTIVE)) { last_err = 7; return; }

	last_err = 0;
}

/*
 * ============================ THE FIX ==============================
 *
 * 12-bit data is LEFT JUSTIFIED inside a 16-bit pair:
 *
 *      msb       lsb
 *    D11..D4   D3..D0 0000
 *
 * Step 1: assemble the 16-bit field, little-endian byte order from the
 *         register pair (msb first on the bus).
 * Step 2: interpret it as SIGNED 16-bit (two's complement).
 * Step 3: divide by 16 to remove the 4 unused low bits.
 *
 * The sign MUST be applied at step 2, before the division. Reading the
 * field as unsigned first is what produced the phantom 8.00 g.
 */
static int16_t decode_axis(uint8_t msb, uint8_t lsb)
{
	int32_t field = ((int32_t)(uint32_t)msb << 8) | (int32_t)(uint32_t)lsb;

	/* Sign-extend the 16-bit field. */
	if (field & 0x8000) {
		field -= 0x10000;
	}

	/* Remove the 4 unused low bits. Result is a signed 12-bit count. */
	return (int16_t)(field / 16);
}

static int sensor_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t raw[AXIS_BYTES];
	int ret = sensor_read_regs(REG_OUT_X_MSB, raw, sizeof(raw));

	if (ret) {
		read_ok = 0;
		last_err = 8;
		return ret;
	}
	read_ok = 1;

	*x = decode_axis(raw[0], raw[1]);
	*y = decode_axis(raw[2], raw[3]);
	*z = decode_axis(raw[4], raw[5]);

	/* No clamping. A value outside +/-2048 counts is reported as-is so
	 * it is visible as bad, rather than disguised as a clean 8.00 g. */

	return 0;
}

/* ================================================================== */
/* Packet helpers                                                     */
/* ================================================================== */

static void put16(uint8_t *dst, uint16_t v)
{
	dst[0] = (uint8_t)(v & 0xFF);
	dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void start_packet(void)
{
	packet_seq++;
	put16(&packet[0], packet_seq);
	put16(&packet[2], 0);
	values_ready = 0;
}

static void append_value(int16_t x, int16_t y, int16_t z)
{
	if (values_ready >= VALUES_PER_PACKET) {
		return;
	}

	uint8_t *slot = &packet[HEADER_BYTES + (values_ready * AXIS_BYTES)];

	put16(&slot[0], (uint16_t)x);
	put16(&slot[2], (uint16_t)y);
	put16(&slot[4], (uint16_t)z);
	values_ready++;
}

/* ================================================================== */
/* GATT                                                               */
/* ================================================================== */

static void tyre_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(value);
}

BT_GATT_SERVICE_DEFINE(tyre_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_TYRE_SERVICE_VAL)),

	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_TYRE_DATA_VAL),
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(tyre_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_TYRE_DIAG_VAL),
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       diag_read, NULL, NULL),
);

/* ================================================================== */
/* Sending                                                            */
/* ================================================================== */

static void send_packet(void)
{
	uint8_t len;
	int rc;

	if (current_conn == NULL || values_ready == 0) {
		return;
	}

	put16(&packet[2], values_ready);
	len = (uint8_t)(HEADER_BYTES + (values_ready * AXIS_BYTES));

	rc = bt_gatt_notify(current_conn, &tyre_svc.attrs[1], packet, len);
	last_notify_rc = (uint8_t)(rc & 0xFF);

	if (rc == 0) {
		packets_sent++;
	} else {
		notify_fails++;
	}

	start_packet();
}

static void send_handshake(void)
{
	uint8_t probe[HEADER_BYTES];

	put16(&probe[0], 0);
	put16(&probe[2], 0);

	(void)bt_gatt_notify(current_conn, &tyre_svc.attrs[1],
			     probe, sizeof(probe));
}

/* ================================================================== */
/* Advertising / connection                                           */
/* ================================================================== */

static void advertising_start(void)
{
	static const struct bt_data ad[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	};

	(void)bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
					      BT_GAP_ADV_FAST_INT_MIN_2,
					      BT_GAP_ADV_FAST_INT_MAX_2,
					      NULL),
			      ad, ARRAY_SIZE(ad), NULL, 0);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		advertising_start();
		return;
	}
	current_conn = bt_conn_ref(conn);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

/* ================================================================== */
/* Sampling / averaging                                               */
/* ================================================================== */

static void sample_once(void)
{
	int16_t x, y, z;

	if (sensor_read_xyz(&x, &y, &z) != 0) {
		return;
	}

	samples_taken++;

	acc_x += x;
	acc_y += y;
	acc_z += z;
	acc_count++;

	if (acc_count >= DECIMATE) {
		int32_t ax = acc_x / DECIMATE;
		int32_t ay = acc_y / DECIMATE;
		int32_t az = acc_z / DECIMATE;

		append_value((int16_t)ax, (int16_t)ay, (int16_t)az);
		averages_made++;

		acc_x = acc_y = acc_z = 0;
		acc_count = 0;
	}
}

/* ================================================================== */
/* Main                                                               */
/* ================================================================== */

int main(void)
{
	who_value = 0; last_err = 0; i2c_ok = 0; read_ok = 0;
	acc_count = 0; values_ready = 0;

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		last_err = 1;
	} else {
		i2c_ok = 1;
		sensor_init();
	}

	if (bt_enable(NULL)) {
		return -1;
	}

	advertising_start();
	start_packet();

	int64_t next_sample = k_uptime_get();
	int64_t next_notify = next_sample + NOTIFY_PERIOD_MS;
	bool handshake_sent = false;

	while (1) {
		int64_t now = k_uptime_get();

		if (current_conn != NULL && !handshake_sent) {
			send_handshake();
			handshake_sent = true;
		}
		if (current_conn == NULL) {
			handshake_sent = false;
		}

		if (now >= next_sample) {
			sample_once();
			next_sample += SAMPLE_PERIOD_MS;
			if (k_uptime_get() - next_sample > SAMPLE_PERIOD_MS) {
				next_sample = k_uptime_get() + SAMPLE_PERIOD_MS;
			}
		}

		if (now >= next_notify) {
			send_packet();
			next_notify += NOTIFY_PERIOD_MS;
		}

		k_sleep(K_MSEC(1));
	}

	return 0;
}
