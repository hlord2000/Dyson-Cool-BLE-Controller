/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dyson Cool BLE Controller
 *
 * BLE peripheral with custom GATT service for IR fan control.
 * Uses TIMER + GPIOTE + DPPI for hardware-based 38kHz carrier generation
 * that doesn't block interrupts (BLE-friendly).
 *
 * Hardware setup:
 * - IR LED/transmitter connected to P1.14 (TX)
 *
 * BLE GATT Service:
 * - Fan Control Service: 12345678-1234-5678-1234-56789abcdef0
 * - Fan Command Char:    12345678-1234-5678-1234-56789abcdef1 (write)
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>

/* BLE includes */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <nrfx_gpiote.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dyson_ble, LOG_LEVEL_INF);

/* GPIO pin - P1.14 for TX */
#define IR_TX_PIN        NRF_GPIO_PIN_MAP(1, 14)

/* GPIOTE instance - use gpiote20 for P1 pins on nRF54L15 */
#define GPIOTE_INST      20
#define GPIOTE_NODE      DT_NODELABEL(gpiote20)

/* Timer instance for 38kHz carrier */
#define TX_TIMER_NODE    DT_NODELABEL(timer21)

BUILD_ASSERT(IS_ENABLED(CONFIG_NRFX_GPIOTE20),
	"CONFIG_NRFX_GPIOTE20 must be enabled");
BUILD_ASSERT(IS_ENABLED(CONFIG_NRFX_TIMER),
	"CONFIG_NRFX_TIMER must be enabled");

/* 38kHz carrier = 26.3us period, 13us half-period for toggle */
#define IR_CARRIER_HZ        38000
#define CARRIER_HALF_PERIOD_US  13

/* Timing constants in microseconds
 * Protocol:
 *   START: LOW for 2.2ms (carrier burst)
 *   Inter-bit: LOW for ~750us (carrier burst)
 *   Bit 0: HIGH for ~750us (space - no carrier)
 *   Bit 1: HIGH for ~1.5ms (space - no carrier)
 */
#define START_LOW_US         2200
#define INTERBIT_LOW_US      750
#define BIT_0_HIGH_US        750
#define BIT_1_HIGH_US        1500

/* Gap between repeated transmissions */
#define TX_REPEAT_GAP_MS     100

/* GPIOTE and Timer instances */
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE_INST_GET(GPIOTE_INST));
static nrfx_timer_t tx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(21));

/* GPIOTE channel for TX output */
static uint8_t gpiote_tx_channel;

/* DPPI handle for timer->gpiote connection */
static nrfx_gppi_handle_t tx_dppi_handle;

/* Flag to track if TX hardware is initialized */
static bool tx_hw_initialized;

/* Statistics */
static uint32_t tx_packets;

/* BLE connection state */
static struct bt_conn *current_conn;
static bool ble_connected;
static uint32_t ble_tx_count;

/*
 * Fan control codes (16-bit IR codes)
 */
#define FAN_POWER       0x4800
#define FAN_OSCILLATE   0x48A8
#define FAN_SPEED_UP    0x4854
#define FAN_SPEED_DOWN  0x48FC
#define FAN_TIME_UP     0x4878
#define FAN_TIME_DOWN   0x48CC

/*
 * BLE Fan Command IDs (single byte over BLE)
 */
#define BLE_CMD_POWER      0x01
#define BLE_CMD_OSCILLATE  0x02
#define BLE_CMD_SPEED_UP   0x03
#define BLE_CMD_SPEED_DOWN 0x04
#define BLE_CMD_TIME_UP    0x05
#define BLE_CMD_TIME_DOWN  0x06

/*
 * Custom GATT Service UUIDs
 */
#define BT_UUID_FAN_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_FAN_COMMAND_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 fan_service_uuid = BT_UUID_INIT_128(BT_UUID_FAN_SERVICE_VAL);
static struct bt_uuid_128 fan_command_uuid = BT_UUID_INIT_128(BT_UUID_FAN_COMMAND_VAL);

/*
 * Advertising data
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_FAN_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/*
 * Start 38kHz carrier (hardware-based, non-blocking)
 */
static void ir_carrier_start(void)
{
	nrfx_timer_clear(&tx_timer);
	nrfx_timer_enable(&tx_timer);
}

/*
 * Stop 38kHz carrier
 */
static void ir_carrier_stop(void)
{
	nrfx_timer_disable(&tx_timer);
	/* Ensure pin is LOW after stopping */
	nrf_gpio_pin_clear(IR_TX_PIN);
}

/*
 * Send carrier burst for specified duration (microseconds)
 * Hardware generates 38kHz - we just wait for the duration
 */
static void ir_carrier_burst(uint32_t duration_us)
{
	ir_carrier_start();
	k_busy_wait(duration_us);
	ir_carrier_stop();
}

/*
 * Space period - no carrier, pin LOW (receiver sees HIGH)
 */
static void ir_space(uint32_t duration_us)
{
	nrf_gpio_pin_clear(IR_TX_PIN);
	k_busy_wait(duration_us);
}

/*
 * Transmit a single packet using 38kHz modulated carrier
 *
 * Protocol (from receiver perspective):
 *   START: LOW for 2.2ms (carrier burst)
 *   Then for each bit:
 *     - Space: HIGH for 750us (bit 0) or 1.5ms (bit 1)
 *     - Mark: LOW for 750us (carrier burst)
 */
static void ir_transmit_packet(const uint8_t *data, uint32_t num_bits)
{
	uint32_t i;

	/* Send start marker - carrier burst for 2.2ms */
	ir_carrier_burst(START_LOW_US);

	/* Send data bits MSB first */
	for (i = 0; i < num_bits; i++) {
		uint32_t byte_idx = i / 8;
		uint32_t bit_idx = 7 - (i % 8);
		uint8_t bit_val = (data[byte_idx] >> bit_idx) & 1;

		/* Space - duration encodes bit value */
		if (bit_val) {
			ir_space(BIT_1_HIGH_US);
		} else {
			ir_space(BIT_0_HIGH_US);
		}

		/* Mark - fixed 750us carrier burst */
		ir_carrier_burst(INTERBIT_LOW_US);
	}

	/* Ensure pin is LOW after transmission */
	nrf_gpio_pin_clear(IR_TX_PIN);
}

/*
 * Transmit data - sends packet twice with 100ms gap
 * No irq_lock needed - hardware carrier generation is BLE-friendly
 */
static void ir_transmit(const uint8_t *data, uint32_t num_bits)
{
	/* First transmission */
	ir_transmit_packet(data, num_bits);

	/* Wait 100ms between packets */
	k_msleep(TX_REPEAT_GAP_MS);

	/* Second transmission */
	ir_transmit_packet(data, num_bits);

	tx_packets++;
}

/*
 * Execute fan command - translate BLE command to IR code
 */
static void execute_fan_command(uint8_t cmd)
{
	uint8_t data[2];
	uint16_t code;

	switch (cmd) {
	case BLE_CMD_POWER:
		code = FAN_POWER;
		LOG_INF("BLE: Power command");
		break;
	case BLE_CMD_OSCILLATE:
		code = FAN_OSCILLATE;
		LOG_INF("BLE: Oscillate command");
		break;
	case BLE_CMD_SPEED_UP:
		code = FAN_SPEED_UP;
		LOG_INF("BLE: Speed up command");
		break;
	case BLE_CMD_SPEED_DOWN:
		code = FAN_SPEED_DOWN;
		LOG_INF("BLE: Speed down command");
		break;
	case BLE_CMD_TIME_UP:
		code = FAN_TIME_UP;
		LOG_INF("BLE: Time up command");
		break;
	case BLE_CMD_TIME_DOWN:
		code = FAN_TIME_DOWN;
		LOG_INF("BLE: Time down command");
		break;
	default:
		LOG_WRN("BLE: Unknown command 0x%02x", cmd);
		return;
	}

	data[0] = (code >> 8) & 0xFF;
	data[1] = code & 0xFF;
	ir_transmit(data, 16);
	ble_tx_count++;
}

/*
 * GATT write callback for fan command characteristic
 */
static ssize_t write_fan_command(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len,
				 uint16_t offset, uint8_t flags)
{
	const uint8_t *data = buf;

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	execute_fan_command(data[0]);

	return len;
}

/*
 * GATT Service Definition
 */
BT_GATT_SERVICE_DEFINE(fan_svc,
	BT_GATT_PRIMARY_SERVICE(&fan_service_uuid),
	BT_GATT_CHARACTERISTIC(&fan_command_uuid.uuid,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, write_fan_command, NULL),
);

/*
 * Start BLE advertising
 */
static int start_advertising(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Advertising started");
	return 0;
}

/*
 * BLE connection callbacks
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connection failed (err 0x%02x): %s", err, addr);
		return;
	}

	LOG_INF("Connected: %s", addr);
	current_conn = bt_conn_ref(conn);
	ble_connected = true;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	ble_connected = false;

	/* Resume advertising after disconnect */
	int adv_err = start_advertising();
	if (adv_err) {
		LOG_ERR("Failed to restart advertising (err %d)", adv_err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/*
 * Initialize BLE
 */
static int init_ble(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth initialized");

	err = start_advertising();
	if (err) {
		return err;
	}

	return 0;
}

/*
 * Timer handler - not used, but required for nrfx_timer_init
 */
static void tx_timer_handler(nrf_timer_event_t event_type, void *context)
{
	/* Timer runs in hardware, no CPU intervention needed */
}

/*
 * Initialize hardware-based 38kHz carrier generation
 * Uses TIMER21 + GPIOTE + DPPI for interrupt-free operation
 */
static int init_tx_hardware(void)
{
	int rv;
	nrfx_err_t err;

	LOG_INF("Initializing IR TX hardware...");

	/* Connect GPIOTE IRQ handler */
	IRQ_CONNECT(DT_IRQN(GPIOTE_NODE), DT_IRQ(GPIOTE_NODE, priority),
		    nrfx_gpiote_irq_handler, &gpiote, 0);

	/* Initialize GPIOTE - may already be initialized by BLE/Zephyr */
	rv = nrfx_gpiote_init(&gpiote, 0);
	LOG_INF("GPIOTE init returned: %d", rv);
	/* Accept 0 (success) or positive error codes that indicate already init */
	if (rv < 0) {
		LOG_ERR("GPIOTE init failed: %d", rv);
		return -EIO;
	}

	/* Allocate GPIOTE channel for TX output */
	err = nrfx_gpiote_channel_alloc(&gpiote, &gpiote_tx_channel);
	if (err != 0) {
		LOG_ERR("GPIOTE channel alloc failed: 0x%x", err);
		return -EIO;
	}
	LOG_INF("GPIOTE TX channel: %d", gpiote_tx_channel);

	/* Configure TX pin as GPIOTE output with toggle task */
	nrfx_gpiote_output_config_t output_cfg = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	nrfx_gpiote_task_config_t task_cfg = {
		.task_ch = gpiote_tx_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
	};

	err = nrfx_gpiote_output_configure(&gpiote, IR_TX_PIN, &output_cfg, &task_cfg);
	if (err != 0) {
		LOG_ERR("GPIOTE output configure failed: 0x%x", err);
		return -EIO;
	}

	nrfx_gpiote_out_task_enable(&gpiote, IR_TX_PIN);
	LOG_INF("GPIOTE output configured on pin %d", IR_TX_PIN);

	/* Initialize Timer21 for 38kHz carrier (1MHz, 13us toggle) */
	IRQ_CONNECT(DT_IRQN(TX_TIMER_NODE), DT_IRQ(TX_TIMER_NODE, priority),
		    nrfx_timer_irq_handler, &tx_timer, 0);

	nrfx_timer_config_t timer_cfg = {
		.frequency = NRFX_MHZ_TO_HZ(1),  /* 1MHz = 1us resolution */
		.mode = NRF_TIMER_MODE_TIMER,
		.bit_width = NRF_TIMER_BIT_WIDTH_16,
		.interrupt_priority = DT_IRQ(TX_TIMER_NODE, priority),
	};

	err = nrfx_timer_init(&tx_timer, &timer_cfg, tx_timer_handler);
	if (err != 0) {
		LOG_ERR("Timer init failed: 0x%x", err);
		return -EIO;
	}

	/* Set CC[0] for half-period toggle (13us for 38kHz) */
	nrfx_timer_extended_compare(&tx_timer, NRF_TIMER_CC_CHANNEL0,
				    CARRIER_HALF_PERIOD_US,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				    false);  /* No interrupt needed */

	LOG_INF("Timer21 configured for %d Hz carrier", IR_CARRIER_HZ);

	/* Connect Timer CC[0] event to GPIOTE toggle task via DPPI */
	rv = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&tx_timer, NRF_TIMER_CC_CHANNEL0),
		nrfx_gpiote_out_task_address_get(&gpiote, IR_TX_PIN),
		&tx_dppi_handle);
	if (rv != 0) {
		LOG_ERR("DPPI conn alloc failed: %d", rv);
		return -EIO;
	}

	nrfx_gppi_conn_enable(tx_dppi_handle);
	LOG_INF("DPPI connected: Timer CC[0] -> GPIOTE toggle");

	tx_hw_initialized = true;
	LOG_INF("IR TX hardware initialized (38kHz carrier via TIMER+GPIOTE+DPPI)");

	return 0;
}

/*
 * Shell command helpers
 */
static void tx_fan_code(const struct shell *sh, uint16_t code, const char *name)
{
	uint8_t data[2];

	data[0] = (code >> 8) & 0xFF;
	data[1] = code & 0xFF;
	shell_print(sh, "Fan %s (0x%04X)", name, code);
	ir_transmit(data, 16);
	shell_print(sh, "Done");
}

static int cmd_fan_power(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_POWER, "power");
	return 0;
}

static int cmd_fan_oscillate(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_OSCILLATE, "oscillate");
	return 0;
}

static int cmd_fan_speed_up(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_SPEED_UP, "speed up");
	return 0;
}

static int cmd_fan_speed_down(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_SPEED_DOWN, "speed down");
	return 0;
}

static int cmd_fan_time_up(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_TIME_UP, "time up");
	return 0;
}

static int cmd_fan_time_down(const struct shell *sh, size_t argc, char **argv)
{
	tx_fan_code(sh, FAN_TIME_DOWN, "time down");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Dyson Cool BLE Controller Status");
	shell_print(sh, "=========================================");
	shell_print(sh, "BLE:");
	shell_print(sh, "  Device:     %s", CONFIG_BT_DEVICE_NAME);
	shell_print(sh, "  Connected:  %s", ble_connected ? "Yes" : "No");
	shell_print(sh, "  BLE TX:     %u", ble_tx_count);
	shell_print(sh, "");
	shell_print(sh, "IR TX:");
	shell_print(sh, "  Pin:        P1.14 (nrf pin %d)", IR_TX_PIN);
	shell_print(sh, "  Carrier:    %d Hz (hardware)", IR_CARRIER_HZ);
	shell_print(sh, "  HW Init:    %s", tx_hw_initialized ? "Yes" : "No");
	shell_print(sh, "  Packets:    %u", tx_packets);
	return 0;
}

static int cmd_ir_tx(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t data[8];
	uint32_t num_bits;
	uint32_t val;
	size_t len;
	size_t i;

	if (argc < 2) {
		shell_print(sh, "Usage: ir tx <value>");
		shell_print(sh, "  ir tx 0xAB      - transmit hex byte");
		shell_print(sh, "  ir tx 0xABCD    - transmit hex word");
		shell_print(sh, "  ir tx 10101010  - transmit binary string");
		return 0;
	}

	memset(data, 0, sizeof(data));

	if (argv[1][0] == '0' && (argv[1][1] == 'x' || argv[1][1] == 'X')) {
		val = strtoul(argv[1], NULL, 16);

		if (val <= 0xFF) {
			data[0] = val;
			num_bits = 8;
		} else if (val <= 0xFFFF) {
			data[0] = (val >> 8) & 0xFF;
			data[1] = val & 0xFF;
			num_bits = 16;
		} else {
			data[0] = (val >> 24) & 0xFF;
			data[1] = (val >> 16) & 0xFF;
			data[2] = (val >> 8) & 0xFF;
			data[3] = val & 0xFF;
			num_bits = 32;
		}
	} else {
		len = strlen(argv[1]);

		if (len > 64) {
			shell_error(sh, "Binary string too long (max 64)");
			return -EINVAL;
		}

		num_bits = len;
		for (i = 0; i < len; i++) {
			if (argv[1][i] != '0' && argv[1][i] != '1') {
				shell_error(sh, "Invalid binary char: %c", argv[1][i]);
				return -EINVAL;
			}
			if (argv[1][i] == '1') {
				uint32_t byte_idx = i / 8;
				uint32_t bit_idx = 7 - (i % 8);
				data[byte_idx] |= (1 << bit_idx);
			}
		}
	}

	shell_print(sh, "Transmitting %u bits...", num_bits);
	ir_transmit(data, num_bits);
	shell_print(sh, "Done");

	return 0;
}

/* Fan subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(fan_cmds,
	SHELL_CMD(power, NULL, "Power on/off", cmd_fan_power),
	SHELL_CMD(osc, NULL, "Oscillate side to side", cmd_fan_oscillate),
	SHELL_CMD(up, NULL, "Speed up", cmd_fan_speed_up),
	SHELL_CMD(down, NULL, "Speed down", cmd_fan_speed_down),
	SHELL_CMD(tup, NULL, "Timer up", cmd_fan_time_up),
	SHELL_CMD(tdown, NULL, "Timer down", cmd_fan_time_down),
	SHELL_SUBCMD_SET_END
);

/* IR subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(ir_cmds,
	SHELL_CMD_ARG(tx, NULL, "Transmit: ir tx <0xHH|binary>", cmd_ir_tx, 1, 1),
	SHELL_CMD(fan, &fan_cmds, "Fan control commands", NULL),
	SHELL_CMD(status, NULL, "Show status", cmd_status),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ir, &ir_cmds, "IR commands", NULL);

int main(void)
{
	int ret;

	printk("\n");
	printk("===========================================\n");
	printk("  Dyson Cool BLE Controller\n");
	printk("  nRF54L15-DK\n");
	printk("===========================================\n");
	printk("BLE Device: %s\n", CONFIG_BT_DEVICE_NAME);
	printk("IR TX Pin:  P1.14 (38kHz hardware carrier)\n");
	printk("===========================================\n");
	printk("BLE Commands (write to characteristic):\n");
	printk("  0x01 = Power      0x02 = Oscillate\n");
	printk("  0x03 = Speed Up   0x04 = Speed Down\n");
	printk("  0x05 = Time Up    0x06 = Time Down\n");
	printk("===========================================\n");
	printk("Shell: ir fan <power|osc|up|down|tup|tdown>\n");
	printk("===========================================\n\n");

	/* Initialize hardware-based IR TX */
	ret = init_tx_hardware();
	if (ret < 0) {
		LOG_ERR("IR TX hardware init failed: %d", ret);
		return ret;
	}

	/* Initialize BLE */
	ret = init_ble();
	if (ret < 0) {
		LOG_ERR("BLE init failed: %d", ret);
		return ret;
	}

	LOG_INF("Dyson Cool BLE Controller ready");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
