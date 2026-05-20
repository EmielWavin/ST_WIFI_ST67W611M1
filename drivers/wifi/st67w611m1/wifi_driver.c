/*
 * ST67W611M1 Wi-Fi NCP HAL driver.
 *
 * All control (scan, connect, disconnect, AP start/stop, version query) and
 * data traffic (L2 Ethernet frames) travel over a binary-framed SPI bus using
 * the ST67W611M1 T02 "host LwIP" NCP firmware profile. The host (Zephyr) runs
 * the IP stack; the NCP is a pure Wi-Fi radio bridging SPI↔802.11.
 *
 * SPI frame format (8-byte header, little-endian, followed by payload):
 *   [magic:u16_le=0x55AA][len:u16_le][flags:u8][type:u8][rsvd:u16]
 *   flags = version(2b) | rx_stall(1b) | flags_rest(5b) — all zero for host
 *
 * Traffic types:
 *   0 = AT command / response
 *   1 = STA Ethernet frame
 *   2 = AP Ethernet frame
 *
 * All SPI transactions are FULL-DUPLEX: host sends header+payload while
 * simultaneously receiving header+payload from the NCP.
 *
 * DATA_READY (PE13):
 *   Rising edge  → NCP ready for a SPI transaction (TXN_RDY)
 *   Falling edge → NCP has processed host header (HDR_ACK)
 *
 * AT commands follow the x-cube-st67w61 SDK "CW" command set (ESP-AT based):
 *   AT+GMR          — firmware version query
 *   AT+CIPSTAMAC?   — STA MAC address query
 *   AT+CWJAP        — join access point (STA connect)
 *   AT+CWQAP        — quit access point (STA disconnect)
 *   AT+CWMODE       — WiFi mode (1=STA, 2=AP, 3=STA+AP)
 *   AT+CWSAP        — soft-AP configuration
 *   AT+CWLAP        — list access points (scan)
 *
 * Architecture: CONFIG_ZTEST replaces all hardware with in-memory simulation.
 * The Zephyr device/net_if registration only exists in the non-ZTEST path.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_driver, CONFIG_WIFI_ST67W611M1_LOG_LEVEL);

#include "wifi_driver.h"

#ifdef CONFIG_ZTEST

/* =========================================================================
 * SIMULATION PATH — unit tests
 * =========================================================================*/

#include <string.h>

static bool     s_init;
static bool     s_connected;
static bool     s_ap_active;
static uint8_t  s_mac[6]        = {0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
static char     s_ip[WIFI_DRV_IP_MAX_LEN]   = "192.168.1.200";
static char     s_version[32]               = "sim-v2.0.106";

int wifi_driver_init(void)
{
	s_init = true;
	s_connected = false;
	s_ap_active = false;
	return 0;
}

int wifi_driver_connect(const char *ssid, const char *password)
{
	ARG_UNUSED(ssid);
	ARG_UNUSED(password);
	s_connected = true;
	return 0;
}

int wifi_driver_disconnect(void)
{
	s_connected = false;
	return 0;
}

int wifi_driver_ap_start(const char *ssid, const char *password)
{
	ARG_UNUSED(ssid);
	ARG_UNUSED(password);
	s_ap_active = true;
	return 0;
}

int wifi_driver_ap_stop(void)
{
	s_ap_active = false;
	return 0;
}

int wifi_driver_get_mac(uint8_t *mac_out)
{
	memcpy(mac_out, s_mac, 6);
	return 0;
}

const char *wifi_driver_get_ip(void)
{
	return s_ip;
}

const char *wifi_driver_get_ncp_version(void)
{
	return s_version;
}

bool wifi_driver_is_connected(void)
{
	return s_connected;
}

struct net_if *wifi_driver_get_iface(void)
{
	return NULL;
}

int wifi_driver_diag(char *buf, size_t len)
{
	return snprintf(buf, len, "WiFi diag not available in test mode");
}

/* Test control: set > 0 to make the next N scan calls return -EIO */
static int wifi_test_scan_fail_count;

void wifi_test_set_scan_fail_count(int n)
{
	wifi_test_scan_fail_count = n;
}

int wifi_driver_scan_json(const struct device *dev, char *out, size_t out_len)
{
	ARG_UNUSED(dev);
	if (wifi_test_scan_fail_count > 0) {
		wifi_test_scan_fail_count--;
		return -EIO;
	}
	static const char fake[] =
		"[{\"ssid\":\"TestNet\",\"rssi\":-60,\"channel\":6,\"security\":\"WPA2\"}]";

	strncpy(out, fake, out_len - 1);
	out[out_len - 1] = '\0';
	return (int)strlen(fake);
}

void wifi_driver_scan_start_refresh(void) {}

#else /* CONFIG_ZTEST */

/* =========================================================================
 * REAL HARDWARE PATH
 * =========================================================================*/

#define DT_DRV_COMPAT st_st67w611m1

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* --------------------------------------------------------------------------
 * SPI frame protocol (ST67W611M1 binary SPI protocol)
 *
 * Header (8 bytes, packed, little-endian):
 *   [magic:u16][len:u16][flags:u8][type:u8][rsvd:u16]
 *   magic = 0x55AA  (0xAA 0x55 on wire, LE)
 *   flags = version(2b)|rx_stall(1b)|flags_rest(5b) — host always sends 0
 *
 * Transactions are full-duplex: master TX and slave TX happen simultaneously.
 * --------------------------------------------------------------------------*/

#define SPI_HEADER_MAGIC      0x55AAU
#define SPI_FRAME_HDR_SIZE    8U
#define SPI_XFER_MAX_PAYLOAD  1520U   /* W61_MAX_SPI_XFER in SDK */

#define SPI_TRAFFIC_AT_CMD    0U  /* AT command / response */
#define SPI_TRAFFIC_STA_DATA  1U  /* STA L2 Ethernet frame */
#define SPI_TRAFFIC_AP_DATA   2U  /* AP L2 Ethernet frame */

#define AT_RSP_BUF_SIZE       512U
#define RX_THREAD_STACK_SIZE  2048
#define RX_THREAD_PRIORITY    K_PRIO_COOP(7)

/*
 * Scan architecture (CWMODE=3, AP+STA dual mode):
 *
 * The NCP scans while beaconing.  CWMODE=3 is kept throughout (CWMODE=1
 * triggers unpredictable automatic background re-scans).
 *
 *   KEEPALIVE: A background work item fires AT+CWLAP every 20 s.  Results
 *              are parsed and cached in scan_results_buf.  The NCP needs
 *              sustained periodic CWLAP commands to keep its scan database
 *              populated — a single trigger is not enough.  During warm-up
 *              (first 5 kicks), the interval is 10 s for faster priming.
 *
 *   ON-DEMAND: wifi_driver_scan_json() reads from the keepalive's cache.
 *              If the cache is fresh (< 60 s old), results are returned
 *              immediately.  If stale/empty, it waits up to 45 s for the
 *              keepalive to populate the cache.  This avoids sending
 *              competing CWLAPs that interfere with the NCP's scan engine.
 *
 *   DIRECTED:  `neptune wifi scan_ssid` sends a targeted AT+CWLAP with
 *              SSID/channel filter for connecting to a specific network.
 */
#define SCAN_MAX_ENTRIES          32

/* SPI word size: 8-bit, MSB first */
#define WIFI_SPI_OPER  (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8))

/* --------------------------------------------------------------------------
 * Driver configuration (from DTS)
 * --------------------------------------------------------------------------*/

struct wifi_st67_cfg {
	struct spi_dt_spec  bus;
	struct gpio_dt_spec data_ready;
	struct gpio_dt_spec boot;
	struct gpio_dt_spec chip_en;
};

/* --------------------------------------------------------------------------
 * Driver instance data
 * --------------------------------------------------------------------------*/

struct wifi_st67_data {
	struct net_if            *iface;
	struct gpio_callback      dr_cb;
	struct k_sem              dr_sem;       /* rising edge: TXN_RDY */
	struct k_sem              hdr_ack_sem;  /* falling edge: HDR_ACK */
	struct k_mutex            spi_lock;
	struct k_thread           rx_thread;
	char                      ncp_version[32];
	char                      ip_str[WIFI_DRV_IP_MAX_LEN];
	uint8_t                   mac[6];
	bool                      connected;
	bool                      ap_active;
	bool                      init_done;   /* RX thread stays idle until init finishes */
	const struct device      *dev;
	/* Packet counters for diagnostics */
	uint32_t                  rx_frames;
	uint32_t                  tx_frames;
	uint32_t                  rx_bytes;
	uint32_t                  tx_bytes;
	uint32_t                  rx_errors;
};

K_THREAD_STACK_DEFINE(wifi_rx_stack, RX_THREAD_STACK_SIZE);

/* Forward declarations */
static int wifi_st67_at_cmd(const struct device *dev,
			    const char *cmd, char *rsp, size_t rsp_len);

/* --------------------------------------------------------------------------
 * SPI frame helpers
 * --------------------------------------------------------------------------*/

static void spi_hdr_build(uint8_t hdr[SPI_FRAME_HDR_SIZE],
			  uint8_t type, uint16_t payload_len)
{
	hdr[0] = (uint8_t)(SPI_HEADER_MAGIC & 0xFFU);
	hdr[1] = (uint8_t)((SPI_HEADER_MAGIC >> 8) & 0xFFU);
	hdr[2] = (uint8_t)(payload_len & 0xFFU);
	hdr[3] = (uint8_t)((payload_len >> 8) & 0xFFU);
	hdr[4] = 0U; /* flags: version=0, rx_stall=0 */
	hdr[5] = type;
	hdr[6] = 0U; /* rsvd */
	hdr[7] = 0U;
}

static bool spi_hdr_parse(const uint8_t hdr[SPI_FRAME_HDR_SIZE],
			  uint8_t *type_out, uint16_t *len_out)
{
	uint16_t magic = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);

	if (magic != SPI_HEADER_MAGIC) {
		return false;
	}
	*len_out  = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);
	*type_out = hdr[5];
	return true;
}

/*
 * Full-duplex SPI transaction. Caller must hold spi_lock.
 *
 * Sends [header + tx_payload] while simultaneously receiving
 * [header + rx_payload] from the NCP. If tx_payload is NULL or tx_len == 0,
 * an empty header (len=0) is sent (receive-only mode).
 *
 * On return: rx_type_out/rx_len_out reflect what the NCP sent (0 if nothing).
 */
static int spi_transact(const struct device *dev,
			uint8_t tx_type, const uint8_t *tx_payload, uint16_t tx_len,
			uint8_t *rx_type_out, uint8_t *rx_buf, uint16_t rx_max,
			uint16_t *rx_len_out)
{
	const struct wifi_st67_cfg *cfg = dev->config;
	struct wifi_st67_data *data = dev->data;
	int ret;

	/*
	 * ST67W611M1 SPI two-phase protocol (matches SDK spi_iface.c):
	 *
	 * 1. Assert CS manually → NCP raises DATA_READY (TXN_RDY) in response
	 * 2. Wait for DATA_READY HIGH (or it may already be HIGH for NCP-initiated)
	 * 3. Phase 1: full-duplex transfer of [8-byte header + tx_payload] together
	 * 4. NCP drops DATA_READY (HDR_ACK) — processed our header, ready for Phase 2
	 * 5. Phase 2 (optional): rx-only for NCP bytes beyond Phase 1
	 * 6. Deassert CS
	 *
	 * CS is controlled manually so it stays asserted across Phase 1→Phase 2.
	 * Zephyr spi_transceive() is called with CS disabled (auto-CS skipped).
	 * Static buffers are safe because callers hold spi_lock.
	 */
	static uint8_t tx_frame[SPI_FRAME_HDR_SIZE + SPI_XFER_MAX_PAYLOAD + 4U];
	static uint8_t rx_frame[SPI_FRAME_HDR_SIZE + SPI_XFER_MAX_PAYLOAD + 4U];
	static uint8_t tx_phase2_dummy[SPI_XFER_MAX_PAYLOAD + 4U];

	if (tx_len > SPI_XFER_MAX_PAYLOAD) {
		return -EMSGSIZE;
	}

	/* Phase 1 size: header + tx_payload, 4-byte aligned */
	uint16_t phase1_len = (uint16_t)((SPI_FRAME_HDR_SIZE + tx_len + 3U) & ~3U);

	/* Build combined TX frame: [header][payload][zero-padding] */
	memset(tx_frame, 0U, phase1_len);
	spi_hdr_build(tx_frame, tx_type, tx_len);
	if (tx_len > 0U && tx_payload != NULL) {
		memcpy(tx_frame + SPI_FRAME_HDR_SIZE, tx_payload, tx_len);
	}

	/* SPI config with auto-CS disabled — we drive CS manually */
	struct spi_config no_cs_cfg = cfg->bus.config;
	no_cs_cfg.cs.gpio.port = NULL;

	const struct gpio_dt_spec *cs = &cfg->bus.config.cs.gpio;

	/* Assert CS: NCP senses this and will raise DATA_READY (TXN_RDY) */
	(void)gpio_pin_set_dt(cs, 1);

	/* Wait for DATA_READY HIGH via GPIO poll (1000 ms).
	 * For NCP-initiated transactions DR is already HIGH → returns immediately.
	 * For host-initiated: NCP raises DR ~1ms after CS assertion. */
	{
		int64_t deadline = k_uptime_get() + 1000LL;

		while (!gpio_pin_get_dt(&cfg->data_ready) &&
		       k_uptime_get() < deadline) {
			k_msleep(1);
		}
		if (!gpio_pin_get_dt(&cfg->data_ready)) {
			(void)gpio_pin_set_dt(cs, 0);
			LOG_ERR("[spi] TIMEOUT: DR never HIGH after CS");
			return -ETIMEDOUT;
		}
	}

	if (tx_len > 0U) {
		LOG_DBG("[spi] sending %u bytes, p1=%u", tx_len, phase1_len);
	}

	/* Phase 1: transfer [header + tx_payload] together; receive NCP header +
	 * start of NCP payload simultaneously. */
	struct spi_buf    tx_p1b  = {.buf = tx_frame,  .len = phase1_len};
	struct spi_buf    rx_p1b  = {.buf = rx_frame,  .len = phase1_len};
	struct spi_buf_set tx_p1s = {.buffers = &tx_p1b, .count = 1};
	struct spi_buf_set rx_p1s = {.buffers = &rx_p1b, .count = 1};

	ret = spi_transceive(cfg->bus.bus, &no_cs_cfg, &tx_p1s, &rx_p1s);
	if (ret) {
		LOG_ERR("SPI phase1 error: %d", ret);
		(void)gpio_pin_set_dt(cs, 0);
		if (rx_len_out)  { *rx_len_out  = 0U; }
		if (rx_type_out) { *rx_type_out = 0U; }
		return ret;
	}

	/* Parse NCP header (first 8 bytes of rx_frame) */
	uint8_t  rx_type = 0U;
	uint16_t rx_len  = 0U;
	bool ncp_hdr_valid = spi_hdr_parse(rx_frame, &rx_type, &rx_len);
	bool ncp_has_data  = ncp_hdr_valid && rx_len > 0U;

	if (tx_len > 0U) {
		LOG_DBG("[spi] ncp hdr: valid=%d len=%u type=%u",
			(int)ncp_hdr_valid, rx_len, rx_type);
	}

	/* Wait for HDR_ACK (DATA_READY goes LOW): NCP processed Phase 1 header.
	 * Only needed before Phase 2. Skip entirely if Phase 2 is not required —
	 * holding CS while waiting for HDR_ACK that may never come confuses the NCP. */
	uint16_t already_rx = (phase1_len > SPI_FRAME_HDR_SIZE)
			    ? (phase1_len - SPI_FRAME_HDR_SIZE) : 0U;

	if (ncp_has_data && rx_len > already_rx) {
		/* Phase 2 needed: wait briefly for HDR_ACK */
		if (k_sem_take(&data->hdr_ack_sem, K_MSEC(10)) != 0) {
			/* Fallback: poll DR for up to 20ms */
			int64_t deadline = k_uptime_get() + 20LL;

			while (k_uptime_get() < deadline &&
			       gpio_pin_get_dt(&cfg->data_ready)) {
				k_msleep(1);
			}
		}

		/* Phase 2: rx-only for NCP bytes beyond Phase 1 capacity */
		uint16_t extra = (uint16_t)((rx_len - already_rx + 3U) & ~3U);
		uint16_t avail = (uint16_t)(sizeof(rx_frame) - phase1_len);

		if (extra > avail) {
			LOG_WRN("phase2 overflow: clamping %u=>%u", extra, avail);
			extra = avail;
		}

		memset(tx_phase2_dummy, 0U, extra);

		struct spi_buf    tx_p2b  = {.buf = tx_phase2_dummy,       .len = extra};
		struct spi_buf    rx_p2b  = {.buf = rx_frame + phase1_len, .len = extra};
		struct spi_buf_set tx_p2s = {.buffers = &tx_p2b, .count = 1};
		struct spi_buf_set rx_p2s = {.buffers = &rx_p2b, .count = 1};

		ret = spi_transceive(cfg->bus.bus, &no_cs_cfg, &tx_p2s, &rx_p2s);
		if (ret) {
			LOG_ERR("SPI phase2 error: %d", ret);
		}
	}

	/* Deassert CS */
	(void)gpio_pin_set_dt(cs, 0);

	/* Return results.
	 * NCP payload is contiguous at rx_frame[SPI_FRAME_HDR_SIZE..]. */
	if (rx_type_out) { *rx_type_out = rx_type; }

	if (!ncp_has_data || rx_len == 0U) {
		if (rx_len_out) { *rx_len_out = 0U; }
		return 0;
	}

	uint16_t copy_len = (rx_len < rx_max) ? rx_len : rx_max;

	if (rx_buf != NULL && copy_len > 0U) {
		memcpy(rx_buf, rx_frame + SPI_FRAME_HDR_SIZE, copy_len);
	}
	if (rx_len_out) { *rx_len_out = copy_len; }

	return 0;
}

/* --------------------------------------------------------------------------
 * AT command interface
 *
 * Flow per AT command:
 *   1. Acquire spi_lock (blocks RX thread)
 *   2. spi_transact (send): assert CS → wait DR HIGH → Phase 1 (header+cmd) →
 *      HDR_ACK → Phase 2 (optional) → CS deassert
 *   3. Check for immediate response in the same transaction
 *   4. Loop: wait TXN_RDY (NCP signals response ready) → spi_transact (receive)
 *      → check OK/ERROR
 *   5. Release spi_lock
 * --------------------------------------------------------------------------*/

static int wifi_st67_at_cmd(const struct device *dev,
			    const char *cmd, char *rsp, size_t rsp_len)
{
	struct wifi_st67_data *data = dev->data;
	uint16_t cmd_len = (uint16_t)strlen(cmd);
	uint8_t rx_buf[AT_RSP_BUF_SIZE];
	uint8_t  rx_type;
	uint16_t rx_len;
	size_t   rsp_off = 0;
	int ret;

	k_mutex_lock(&data->spi_lock, K_FOREVER);

	if (rsp && rsp_len > 0U) {
		rsp[0] = '\0';
	}

	/* Drain stale TXN_RDY events from previous commands */
	while (k_sem_take(&data->dr_sem, K_NO_WAIT) == 0) {}

	/* Brief inter-command gap: NCP needs settling time after processing
	 * the previous AT command before it can accept a new CS assertion. */
	k_msleep(50);

	/* Send AT command.  spi_transact asserts CS, waits for DATA_READY HIGH
	 * (NCP acknowledges CS), transfers header+cmd, waits HDR_ACK, releases CS. */
	ret = spi_transact(dev, SPI_TRAFFIC_AT_CMD,
			   (const uint8_t *)cmd, cmd_len,
			   &rx_type, rx_buf, sizeof(rx_buf) - 1U, &rx_len);

	if (ret) {
		k_mutex_unlock(&data->spi_lock);
		return ret;
	}

	/* Check for immediate response in the same transaction */
	if (rx_len > 0U && rx_type == SPI_TRAFFIC_AT_CMD) {
		rx_buf[rx_len] = '\0';
		if (rsp && rsp_len > 0U) {
			size_t n = MIN((size_t)rx_len, rsp_len - 1U);

			memcpy(rsp, rx_buf, n);
			rsp_off = n;
			rsp[rsp_off] = '\0';
		}
		if (strstr((char *)rx_buf, "\nERROR")) {
			k_mutex_unlock(&data->spi_lock);
			LOG_WRN("AT error: %.32s => %.64s", cmd, (char *)rx_buf);
			return -EIO;
		}
		if (strstr((char *)rx_buf, "\nOK")) {
			k_mutex_unlock(&data->spi_lock);
			return 0;
		}
	}

	if (!rsp || rsp_len == 0U) {
		k_mutex_unlock(&data->spi_lock);
		return 0;
	}

	/* Response may arrive across multiple SPI transactions — accumulate
	 * fragments into rsp and poll until the final \nOK or \nERROR terminator. */
	for (int tries = 0; tries < 20; tries++) {
		bool got_it = (k_sem_take(&data->dr_sem, K_MSEC(100)) == 0);

		if (!got_it) {
			int64_t deadline = k_uptime_get() + 5000;

			while (k_uptime_get() < deadline) {
				if (gpio_pin_get_dt(&((const struct wifi_st67_cfg *)dev->config)->data_ready)) {
					got_it = true;
					break;
				}
				k_msleep(10);
			}
		}
		if (!got_it) {
			LOG_ERR("AT response timeout for: %s", cmd);
			k_mutex_unlock(&data->spi_lock);
			return -ETIMEDOUT;
		}

		ret = spi_transact(dev, SPI_TRAFFIC_AT_CMD, NULL, 0U,
				   &rx_type, rx_buf, sizeof(rx_buf) - 1U, &rx_len);

		if (ret || rx_len == 0U || rx_type != SPI_TRAFFIC_AT_CMD) {
			continue;
		}

		rx_buf[rx_len] = '\0';
		LOG_DBG("AT rsp (try %d): %.64s", tries, (char *)rx_buf);

		/* Append fragment to accumulated response buffer */
		if (rsp_off < rsp_len - 1U) {
			size_t avail = rsp_len - 1U - rsp_off;
			size_t n = MIN((size_t)rx_len, avail);

			memcpy(rsp + rsp_off, rx_buf, n);
			rsp_off += n;
			rsp[rsp_off] = '\0';
		}

		if (strstr(rsp, "\nERROR")) {
			k_mutex_unlock(&data->spi_lock);
			LOG_WRN("AT error: %.32s => %.64s", cmd, rsp);
			return -EIO;
		}
		if (strstr(rsp, "\nOK")) {
			k_mutex_unlock(&data->spi_lock);
			return 0;
		}
	}

	k_mutex_unlock(&data->spi_lock);
	return -ETIMEDOUT;
}

/* --------------------------------------------------------------------------
 * DATA_READY GPIO ISR
 *
 * Rising edge  → TXN_RDY: NCP ready for a SPI transaction
 * Falling edge → HDR_ACK: NCP has processed the host header
 * --------------------------------------------------------------------------*/

static void data_ready_isr(const struct device *port,
			   struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	struct wifi_st67_data *data =
		CONTAINER_OF(cb, struct wifi_st67_data, dr_cb);
	const struct wifi_st67_cfg *cfg = data->dev->config;

	if (gpio_pin_get_dt(&cfg->data_ready)) {
		k_sem_give(&data->dr_sem);      /* rising: TXN_RDY */
	} else {
		k_sem_give(&data->hdr_ack_sem); /* falling: HDR_ACK */
	}
}

/* --------------------------------------------------------------------------
 * RX thread — handles incoming SPI frames from the NCP
 *
 * Polls DATA_READY GPIO directly (no semaphore) to avoid race with the AT
 * command sender which owns dr_sem during synchronous command execution.
 * Uses K_NO_WAIT on spi_lock so AT commands always take priority.
 * --------------------------------------------------------------------------*/

static void wifi_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	const struct device *dev = p1;
	struct wifi_st67_data *data = dev->data;
	const struct wifi_st67_cfg *cfg = dev->config;
	static uint8_t frame_buf[SPI_XFER_MAX_PAYLOAD];

	while (1) {
		k_msleep(100); /* ~10 Hz poll rate — keeps NCP responsive without spamming */

		/* Don't touch SPI until hw_init completes */
		if (!data->init_done) {
			continue;
		}

		/* Skip if DATA_READY not asserted */
		if (!gpio_pin_get_dt(&cfg->data_ready)) {
			continue;
		}

		/* Skip if AT command in progress (it owns spi_lock) */
		if (k_mutex_lock(&data->spi_lock, K_NO_WAIT) != 0) {
			continue;
		}

		uint8_t  rx_type  = 0U;
		uint16_t rx_len   = 0U;

		int ret = spi_transact(dev, SPI_TRAFFIC_AT_CMD, NULL, 0U,
				       &rx_type, frame_buf, sizeof(frame_buf),
				       &rx_len);

		k_mutex_unlock(&data->spi_lock);

		if (ret || rx_len == 0U) {
			continue;
		}

		if (rx_type == SPI_TRAFFIC_AT_CMD) {
			frame_buf[MIN(rx_len, sizeof(frame_buf) - 1U)] = '\0';
			LOG_DBG("async AT: %s", (char *)frame_buf);
			continue;
		}

		if (rx_type != SPI_TRAFFIC_STA_DATA && rx_type != SPI_TRAFFIC_AP_DATA) {
			LOG_DBG("unknown traffic type: %u", rx_type);
			continue;
		}

		if (!data->iface) {
			continue;
		}

		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, rx_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("RX: no buffer (%u bytes)", rx_len);
			continue;
		}

		if (net_pkt_write(pkt, frame_buf, rx_len)) {
			net_pkt_unref(pkt);
			continue;
		}

		ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("RX: net_recv_data failed (%d) len=%u", ret, rx_len);
			data->rx_errors++;
			net_pkt_unref(pkt);
		} else {
			data->rx_frames++;
			data->rx_bytes += rx_len;
		}
	}
}

/* --------------------------------------------------------------------------
 * net_if send callback (outgoing Ethernet frames → SPI type 1 or 2)
 * --------------------------------------------------------------------------*/

static int wifi_st67_send(const struct device *dev, struct net_pkt *pkt)
{
	struct wifi_st67_data *data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	static uint8_t tx_buf[NET_ETH_MTU + NET_ETH_MAX_HDR_SIZE];

	if (len > sizeof(tx_buf)) {
		return -EMSGSIZE;
	}

	if (net_pkt_read(pkt, tx_buf, len)) {
		return -EIO;
	}

	uint8_t traffic_type = data->ap_active ? SPI_TRAFFIC_AP_DATA
					       : SPI_TRAFFIC_STA_DATA;

	int ret;

	k_mutex_lock(&data->spi_lock, K_FOREVER);
	ret = spi_transact(dev, traffic_type, tx_buf, (uint16_t)len,
			   NULL, NULL, 0U, NULL);
	k_mutex_unlock(&data->spi_lock);

	if (ret == 0) {
		data->tx_frames++;
		data->tx_bytes += len;
	}

	return ret;
}

/* --------------------------------------------------------------------------
 * wifi_mgmt ops
 * --------------------------------------------------------------------------*/

static int wifi_st67_mgmt_connect(const struct device *dev,
				  struct wifi_connect_req_params *params)
{
	char cmd[128];
	char rsp[AT_RSP_BUF_SIZE];

	/* Switch to STA-only mode for association.  The NCP sometimes fails
	 * to complete WPA2 handshake while AP is simultaneously active. */
	(void)wifi_st67_at_cmd(dev, "AT+CWMODE=1,0\r\n", rsp, sizeof(rsp));

	/* AT+CWJAP="ssid","password",,0
	 * Empty BSSID field, WEP=0 (not WEP). WPA2 is auto-negotiated.
	 * Returns immediately with +CW:CONNECTING\r\nOK — actual
	 * association is async.  Caller must poll wifi_driver_poll_sta_state()
	 * to determine when connection completes. */
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%.*s\",\"%.*s\",,0\r\n",
		 params->ssid_length, params->ssid,
		 params->psk_length, params->psk);

	int ret = wifi_st67_at_cmd(dev, cmd, rsp, sizeof(rsp));
	if (ret) {
		wifi_mgmt_raise_connect_result_event(dev->data ?
			((struct wifi_st67_data *)dev->data)->iface : NULL, -1);
	}
	/* Do NOT set connected=true here — CWJAP is async.
	 * wifi_driver_poll_sta_state() will report when association succeeds. */
	return ret;
}

static int wifi_st67_mgmt_disconnect(const struct device *dev)
{
	char rsp[AT_RSP_BUF_SIZE];

	/* AT+CWQAP=0: quit AP, restore=0 keeps saved credentials */
	int ret = wifi_st67_at_cmd(dev, "AT+CWQAP=0\r\n", rsp, sizeof(rsp));

	struct wifi_st67_data *data = dev->data;

	data->connected = false;
	net_if_carrier_off(data->iface);
	wifi_mgmt_raise_disconnect_result_event(data->iface, ret ? -1 : 0);
	return ret;
}

/* --------------------------------------------------------------------------
 * wifi_driver_poll_sta_state — query AT+CWSTATE? for async connect result
 *
 * Returns:  0 = idle/disconnected
 *           1 = connected to AP, no IP (L2 up, waiting for DHCP)
 *           2 = connected (got IP)
 *          <0 = AT command error
 * --------------------------------------------------------------------------*/
int wifi_driver_poll_sta_state(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct wifi_st67_data *data = dev->data;
	char rsp[256];

	int ret = wifi_st67_at_cmd(dev, "AT+CWSTATE?\r\n", rsp, sizeof(rsp));
	if (ret) {
		return -EIO;
	}

	/* Response format: +CWSTATE:<state>,"<ssid>"\r\nOK
	 * state: 0=idle, 1=connected(no IP), 2=connected(has IP),
	 *        3=disconnecting, 4=disconnected */
	const char *p = strstr(rsp, "+CWSTATE:");
	if (!p) {
		return -EINVAL;
	}
	int state = atoi(p + 9);

	/* State 1 or 2: L2 link is up — signal carrier_on so Zephyr's
	 * network stack can start DHCP on the WiFi interface. */
	if ((state == 1 || state == 2) && !data->connected) {
		data->connected = true;
		net_if_carrier_on(data->iface);
		wifi_mgmt_raise_connect_result_event(data->iface, 0);
		LOG_INF("STA L2 connected (NCP state=%d)", state);
	}

	return state;
}

/* --------------------------------------------------------------------------
 * AT+CWLAP response parser
 * Format: +CWLAP:(ecn,"ssid",rssi,"mac",channel)
 * --------------------------------------------------------------------------*/

/*
 * Returns true if rsp contains at least one CWLAP entry with a non-empty SSID
 * and non-zero channel/RSSI. Filters out the +CWLAP:(0,"",0,"00:...",0) placeholder
 * the NCP emits when the scan result cache is not yet populated.
 */
static bool scan_rsp_has_real_entry(const char *rsp)
{
	const char *p = rsp;

	while ((p = strstr(p, "+CWLAP:(")) != NULL) {
		/* Quick pattern check: +CWLAP:(ecn,"<non-empty>", */
		const char *q = p + 8;        /* skip "+CWLAP:(" */
		const char *comma = strchr(q, ',');

		if (comma && *(comma + 1) == '"' && *(comma + 2) != '"') {
			return true; /* SSID field is non-empty */
		}
		p += 8;
	}
	return false;
}

static int parse_cwlap_line(const char *line, int *ecn_out, char *ssid_out,
			    size_t ssid_len, int *rssi_out, int *channel_out)
{
	const char *p = strstr(line, "+CWLAP:(");

	if (!p) {
		return -1;
	}
	p += 8; /* skip "+CWLAP:(" */

	/* ecn: integer up to first comma */
	char *endp;
	long ecn = strtol(p, &endp, 10);

	if (endp == p || *endp != ',') {
		return -1;
	}
	p = endp + 1;

	/* ssid: quoted string — handle escaped quotes defensively */
	if (*p != '"') {
		return -1;
	}
	p++;
	const char *ssid_start = p;

	while (*p && *p != '"') {
		if (*p == '\\' && *(p + 1)) {
			p += 2;
		} else {
			p++;
		}
	}
	size_t slen = (size_t)(p - ssid_start);

	if (*p == '"') {
		p++;
	}
	if (*p != ',') {
		return -1;
	}
	p++;

	/* rssi: signed integer up to next comma */
	long rssi = strtol(p, &endp, 10);

	if (endp == p || *endp != ',') {
		return -1;
	}
	p = endp + 1;

	/* mac: quoted string — skip it */
	if (*p != '"') {
		return -1;
	}
	p++;
	while (*p && *p != '"') {
		p++;
	}
	if (*p == '"') {
		p++;
	}
	if (*p != ',') {
		return -1;
	}
	p++;

	/* channel: integer */
	long channel = strtol(p, &endp, 10);

	if (endp == p) {
		return -1;
	}

	*ecn_out = (int)ecn;
	size_t copy_len = MIN(slen, ssid_len - 1U);

	memcpy(ssid_out, ssid_start, copy_len);
	ssid_out[copy_len] = '\0';
	*rssi_out    = (int)rssi;
	*channel_out = (int)channel;
	return 0;
}

static int wifi_st67_mgmt_scan(const struct device *dev,
			       struct wifi_scan_params *params,
			       scan_result_cb_t cb)
{
	ARG_UNUSED(params);

	char rsp[2048];
	int ret = wifi_st67_at_cmd(dev, "AT+CWLAP\r\n", rsp, sizeof(rsp));

	if (ret) {
		return ret;
	}

	struct wifi_st67_data *data = dev->data;
	char *line = rsp;

	while ((line = strstr(line, "+CWLAP:(")) != NULL) {
		int ecn, rssi, channel;
		char ssid[WIFI_DRV_SSID_MAX_LEN + 1];

		if (parse_cwlap_line(line, &ecn, ssid, sizeof(ssid),
				     &rssi, &channel) == 0) {
			struct wifi_scan_result result = {0};
			size_t slen = strlen(ssid);

			memcpy(result.ssid, ssid,
			       MIN(slen, sizeof(result.ssid)));
			result.ssid_length = (uint8_t)MIN(slen,
							  sizeof(result.ssid));
			result.rssi    = (int8_t)rssi;
			result.channel = (uint8_t)channel;

			switch (ecn) {
			case 0:
				result.security = WIFI_SECURITY_TYPE_NONE;
				break;
			case 1:
				result.security = WIFI_SECURITY_TYPE_WEP;
				break;
			case 2:
			case 3:
			case 4:
				result.security = WIFI_SECURITY_TYPE_PSK;
				break;
			case 6:
				result.security = WIFI_SECURITY_TYPE_SAE;
				break;
			default:
				result.security = WIFI_SECURITY_TYPE_UNKNOWN;
				break;
			}

			cb(data->iface, 0, &result);
		}

		/* Advance past current prefix to avoid re-matching same entry */
		line += 8;
	}

	cb(data->iface, 0, NULL);
	return 0;
}

/* Forward declarations for scan keepalive (defined later with scan logic) */
#define SCAN_KEEPALIVE_INTERVAL_MS  25000  /* every 25s — gives NCP time to scan */
#define SCAN_KEEPALIVE_INITIAL_MS   20000  /* first kick 20s after AP (post-RST warm-up) */
static void scan_keepalive_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(scan_keepalive_work, scan_keepalive_work_fn);
static int scan_keepalive_kicks;

static int wifi_st67_mgmt_ap_enable(const struct device *dev,
				    struct wifi_connect_req_params *params)
{
	char cmd[192];
	char rsp[AT_RSP_BUF_SIZE];
	int ret;

	/* Step 1: switch to AP+STA dual mode (STA not connected → sta_state=0)
	 * AT+CWMODE=3,0 */
	ret = wifi_st67_at_cmd(dev, "AT+CWMODE=3,0\r\n", rsp, sizeof(rsp));
	if (ret) {
		LOG_ERR("CWMODE=3 failed: %d", ret);
		return ret;
	}

	/* Set country code and scan options once the mode is established.
	 * NL allows channels 1-13.  CWLAPOPT for NCP v2.0.106 takes 5 args:
	 *   sort=1 (RSSI desc), mask=1695, rssi_floor=-100, scan_type=255,
	 *   max_count=50  (raised from the NCP default — fewer caps means
	 *   strong nearby APs that arrive late in the scan still get listed).
	 * Old 3-arg syntax silently applied a small default max_count. */
	if (wifi_st67_at_cmd(dev, "AT+CWCOUNTRY=1,\"NL\",1,13\r\n", rsp,
			     sizeof(rsp))) {
		LOG_WRN("CWCOUNTRY not supported — scan limited to NCP default channels");
	}
	if (wifi_st67_at_cmd(dev, "AT+CWLAPOPT=1,1695,-100,255,50\r\n", rsp,
			     sizeof(rsp))) {
		LOG_WRN("CWLAPOPT (5-arg) failed; falling back to 3-arg form");
		(void)wifi_st67_at_cmd(dev, "AT+CWLAPOPT=1,127,-100\r\n",
				       rsp, sizeof(rsp));
	}

	/* Step 2: configure soft-AP
	 * AT+CWSAP="ssid","password",channel,security,max_conn,hidden
	 * security 3 = WPA2-PSK, max_conn 4, hidden 0 (visible) */
	snprintf(cmd, sizeof(cmd),
		 "AT+CWSAP=\"%.*s\",\"%.*s\",6,3,4,0\r\n",
		 params->ssid_length, params->ssid,
		 params->psk_length, params->psk);

	ret = wifi_st67_at_cmd(dev, cmd, rsp, sizeof(rsp));
	if (!ret) {
		struct wifi_st67_data *data = dev->data;

		data->ap_active = true;
		net_if_carrier_on(data->iface);

		/* Start scan keepalive — first kick shortly after AP is up
		 * so the NCP begins building its scan database.  Warm-up
		 * phase fires more frequently (3 kicks at 15s intervals). */
		scan_keepalive_kicks = 0;
		k_work_reschedule(&scan_keepalive_work,
				  K_MSEC(SCAN_KEEPALIVE_INITIAL_MS));
	}
	return ret;
}

static int wifi_st67_mgmt_ap_disable(const struct device *dev)
{
	char rsp[AT_RSP_BUF_SIZE];

	/* AT+CWMODE=1,0: switch to STA-only mode, do not reconnect */
	int ret = wifi_st67_at_cmd(dev, "AT+CWMODE=1,0\r\n", rsp, sizeof(rsp));

	if (!ret) {
		struct wifi_st67_data *data = dev->data;

		data->ap_active = false;
		k_work_cancel_delayable(&scan_keepalive_work);
		if (!data->connected) {
			net_if_carrier_off(data->iface);
		}
	}
	return ret;
}

static const struct wifi_mgmt_ops wifi_st67_mgmt_ops = {
	.scan       = wifi_st67_mgmt_scan,
	.connect    = wifi_st67_mgmt_connect,
	.disconnect = wifi_st67_mgmt_disconnect,
	.ap_enable  = wifi_st67_mgmt_ap_enable,
	.ap_disable = wifi_st67_mgmt_ap_disable,
};

/* --------------------------------------------------------------------------
 * ethernet_api capabilities
 * --------------------------------------------------------------------------*/

static enum ethernet_hw_caps wifi_st67_get_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (enum ethernet_hw_caps)0;
}

/* --------------------------------------------------------------------------
 * net_if init callback
 * --------------------------------------------------------------------------*/

static void wifi_st67_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct wifi_st67_data *data = dev->data;

	data->iface = iface;
	net_if_set_link_addr(iface, data->mac, sizeof(data->mac),
			     NET_LINK_ETHERNET);

	/* Carrier off until STA connects or AP starts */
	net_if_carrier_off(iface);

	LOG_INF("WiFi interface initialised, MAC %02X:%02X:%02X:%02X:%02X:%02X",
		data->mac[0], data->mac[1], data->mac[2],
		data->mac[3], data->mac[4], data->mac[5]);
}

/* --------------------------------------------------------------------------
 * Device hardware init (called by Zephyr at boot via NET_DEVICE_DT_INST_DEFINE)
 * --------------------------------------------------------------------------*/

static int wifi_st67_hw_init(const struct device *dev)
{
	const struct wifi_st67_cfg *cfg = dev->config;
	struct wifi_st67_data *data = dev->data;
	int ret;

	data->dev = dev;
	k_sem_init(&data->dr_sem, 0, 16);      /* up to 16 pending TXN_RDY events */
	k_sem_init(&data->hdr_ack_sem, 0, 16); /* up to 16 pending HDR_ACK events */
	k_mutex_init(&data->spi_lock);

	/* CHIP_EN and BOOT: configure and do a clean power cycle.
	 * SDK pattern: drive BOTH low first, then CHIP_EN high after BOOT is confirmed LOW.
	 * This ensures a clean NCP boot regardless of previous state. */

	/* BOOT pin — must be LOW before and during NCP power-on for normal operation */
	if (cfg->boot.port) {
		if (!gpio_is_ready_dt(&cfg->boot)) {
			LOG_WRN("BOOT GPIO not ready — skipping");
		} else {
			gpio_pin_configure_dt(&cfg->boot, GPIO_OUTPUT_INACTIVE);
			LOG_DBG("BOOT PE%u LOW", cfg->boot.pin);
		}
	}

	if (cfg->chip_en.port) {
		if (!gpio_is_ready_dt(&cfg->chip_en)) {
			LOG_ERR("CHIP_EN GPIO not ready");
			return -ENODEV;
		}
		/* Drive LOW first to ensure a clean power-off → power-on cycle */
		gpio_pin_configure_dt(&cfg->chip_en, GPIO_OUTPUT_INACTIVE);
		LOG_DBG("CHIP_EN PE%u LOW (power off NCP)", cfg->chip_en.pin);
		k_msleep(200);

		/* Now power on */
		gpio_pin_set_dt(&cfg->chip_en, 1);
		LOG_DBG("CHIP_EN PE%u HIGH (power on NCP)", cfg->chip_en.pin);
	}

	/* SPI bus */
	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure CS GPIO as output, initially deasserted.
	 * We manage CS manually so auto-CS is disabled in spi_transceive calls. */
	if (cfg->bus.config.cs.gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&cfg->bus.config.cs.gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure CS GPIO: %d", ret);
			return ret;
		}
		LOG_DBG("CS PD%u configured as output (deasserted)",
			cfg->bus.config.cs.gpio.pin);
	}

	/* DATA_READY GPIO — interrupt on BOTH edges:
	 *   rising  → TXN_RDY (NCP ready for a SPI transaction)
	 *   falling → HDR_ACK (NCP processed our header) */
	if (!gpio_is_ready_dt(&cfg->data_ready)) {
		LOG_ERR("DATA_READY GPIO not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&cfg->data_ready, GPIO_INPUT);
	gpio_init_callback(&data->dr_cb, data_ready_isr,
			   BIT(cfg->data_ready.pin));
	gpio_add_callback(cfg->data_ready.port, &data->dr_cb);
	int irq_ret = gpio_pin_interrupt_configure_dt(&cfg->data_ready,
						      GPIO_INT_EDGE_BOTH);
	LOG_DBG("DATA_READY PE%u EXTI BOTH: ret=%d initial=%d",
		cfg->data_ready.pin, irq_ret,
		gpio_pin_get_dt(&cfg->data_ready));

	/* Wait for NCP to boot — T02 firmware may take up to 2s to assert DATA_READY */
	k_msleep(2000);

	/* Poll and log DATA_READY state every 100ms for 2s */
	{
		int prev = -1;

		for (int i = 0; i <= 20; i++) {
			int cur = gpio_pin_get_dt(&cfg->data_ready);

			if (cur != prev) {
				LOG_DBG("DATA_READY @ +%dms: %s (irq_cnt=%u)",
					i * 100, cur ? "HIGH" : "LOW",
					k_sem_count_get(&data->dr_sem));
				prev = cur;
			}
			if (i == 0 && cur) {
				/* Already HIGH — good, interrupt should have fired */
				break;
			}
			if (cur && i > 0) {
				k_sem_give(&data->dr_sem);
				break;
			}
			if (i < 20) {
				k_msleep(100);
			}
		}
		if (prev == 0) {
			LOG_WRN("DATA_READY stayed LOW for 2s — NCP not responding");
		}
	}

	/* Drain any spurious semaphore counts accumulated during NCP boot */
	while (k_sem_take(&data->hdr_ack_sem, K_NO_WAIT) == 0) {}
	while (k_sem_take(&data->dr_sem, K_NO_WAIT) == 0) {}

	/* Seed one TXN_RDY if DATA_READY is already HIGH */
	if (gpio_pin_get_dt(&cfg->data_ready)) {
		k_sem_give(&data->dr_sem);
	}

	/* Read NCP boot notification.
	 * After power-on the NCP asserts DATA_READY and sends "\r\nready\r\n".
	 * We must receive this before sending AT commands — the NCP ignores AT
	 * commands sent during its boot sequence.
	 *
	 * Loop: poll DATA_READY → do a receive-only SPI transaction → check for
	 * "ready" in response. Retry up to 10 times with 200ms delays (~2s total).
	 */
	{
		static uint8_t boot_buf[64];
		bool ncp_ready = false;

		for (int attempt = 0; attempt < 10 && !ncp_ready; attempt++) {
			/* Wait up to 500ms for DATA_READY to assert */
			int64_t dr_deadline = k_uptime_get() + 500LL;

			while (!gpio_pin_get_dt(&cfg->data_ready) &&
			       k_uptime_get() < dr_deadline) {
				k_msleep(20);
			}

			if (!gpio_pin_get_dt(&cfg->data_ready)) {
				k_msleep(200);
				continue;
			}

			uint8_t  boot_rx_type = 0U;
			uint16_t boot_rx_len  = 0U;

			k_mutex_lock(&data->spi_lock, K_FOREVER);
			(void)spi_transact(dev, SPI_TRAFFIC_AT_CMD, NULL, 0U,
					   &boot_rx_type, boot_buf,
					   sizeof(boot_buf) - 1U, &boot_rx_len);
			k_mutex_unlock(&data->spi_lock);

			if (boot_rx_len > 0U) {
				boot_buf[boot_rx_len] = '\0';
				LOG_INF("NCP boot drain [%d]: %.32s",
					attempt, (char *)boot_buf);
				if (strstr((char *)boot_buf, "ready")) {
					ncp_ready = true;
				}
			} else {
				LOG_DBG("NCP boot drain [%d]: empty", attempt);
				k_msleep(200);
			}
		}

		if (!ncp_ready) {
			LOG_WRN("NCP did not send 'ready' — proceeding anyway");
		}
	}

	/* Query NCP firmware version: AT+GMR */
	char rsp[AT_RSP_BUF_SIZE];

	ret = wifi_st67_at_cmd(dev, "AT+GMR\r\n", rsp, sizeof(rsp));

	if (!ret) {
		/* Prefer component_version_sdk_ (overall NCP version) */
		const char *tag = "component_version_sdk_";
		const char *p   = strstr(rsp, tag);

		if (!p) {
			/* Fallback to macsw version for older firmware */
			tag = "component_version_macsw_";
			p   = strstr(rsp, tag);
		}

		if (p) {
			p += strlen(tag);
			/* Copy version digits until whitespace or end */
			size_t i = 0;

			while (*p && *p != '\r' && *p != '\n' && *p != ' ' &&
			       i < sizeof(data->ncp_version) - 1U) {
				data->ncp_version[i++] = *p++;
			}
			data->ncp_version[i] = '\0';
		} else {
			/* Fallback: use first line of response */
			strncpy(data->ncp_version, rsp,
				sizeof(data->ncp_version) - 1U);
			data->ncp_version[sizeof(data->ncp_version) - 1U] = '\0';
		}
	} else {
		strncpy(data->ncp_version, "unknown",
			sizeof(data->ncp_version));
		LOG_WRN("NCP version query failed (%d) — continuing", ret);
	}

	/* Read STA MAC address: AT+CIPSTAMAC?
	 * Response format: +CIPSTAMAC:"AA:BB:CC:DD:EE:FF" */
	ret = wifi_st67_at_cmd(dev, "AT+CIPSTAMAC?\r\n", rsp, sizeof(rsp));
	if (!ret) {
		const char *p = strstr(rsp, "+CIPSTAMAC:\"");
		bool parsed   = false;

		if (p) {
			p += 12; /* skip '+CIPSTAMAC:"' */
			uint8_t m[6];
			bool ok = true;

			for (int i = 0; i < 6 && ok; i++) {
				char hi = *p++;
				char lo = *p++;
				uint8_t h = (uint8_t)((hi >= 'a') ? (hi - 'a' + 10) :
						      (hi >= 'A') ? (hi - 'A' + 10) :
						      (hi - '0'));
				uint8_t l = (uint8_t)((lo >= 'a') ? (lo - 'a' + 10) :
						      (lo >= 'A') ? (lo - 'A' + 10) :
						      (lo - '0'));

				m[i] = (h << 4) | l;
				if (i < 5 && *p++ != ':') {
					ok = false;
				}
			}
			if (ok) {
				memcpy(data->mac, m, 6);
				parsed = true;
			}
		}
		ARG_UNUSED(parsed);
	}
	if (data->mac[0] == 0U && data->mac[1] == 0U) {
		static const uint8_t fallback[6] = {
			0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

		memcpy(data->mac, fallback, 6);
		LOG_WRN("MAC query failed — using fallback");
	}

	/* Start RX thread — now init is done, allow it to poll SPI */
	data->init_done = true;
	k_thread_create(&data->rx_thread, wifi_rx_stack, RX_THREAD_STACK_SIZE,
			wifi_rx_thread, (void *)dev, NULL, NULL,
			RX_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "wifi_rx");

	LOG_INF("ST67W611M1 init OK, NCP fw: %s", data->ncp_version);
	return 0;
}

/* --------------------------------------------------------------------------
 * Public HAL API (called by wifi_manager)
 * --------------------------------------------------------------------------*/

int wifi_driver_init(void)
{
	/* Device is registered automatically by NET_DEVICE_DT_INST_DEFINE.
	 * This function is a no-op at firmware level but kept for symmetry
	 * with the CONFIG_ZTEST path. */
	return 0;
}

int wifi_driver_connect(const char *ssid, const char *password)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	struct wifi_connect_req_params params = {
		.ssid        = (const uint8_t *)ssid,
		.ssid_length = (uint8_t)strlen(ssid),
		.psk         = (const uint8_t *)password,
		.psk_length  = (uint8_t)strlen(password),
		.security    = WIFI_SECURITY_TYPE_PSK,
		.channel     = WIFI_CHANNEL_ANY,
		.mfp         = WIFI_MFP_OPTIONAL,
	};
	return wifi_st67_mgmt_connect(dev, &params);
}

int wifi_driver_disconnect(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	return wifi_st67_mgmt_disconnect(dev);
}

int wifi_driver_ap_start(const char *ssid, const char *password)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	struct wifi_connect_req_params params = {
		.ssid        = (const uint8_t *)ssid,
		.ssid_length = (uint8_t)strlen(ssid),
		.psk         = (const uint8_t *)password,
		.psk_length  = (uint8_t)strlen(password),
		.security    = WIFI_SECURITY_TYPE_PSK,
		.channel     = 6,
		.mfp         = WIFI_MFP_OPTIONAL,
	};
	return wifi_st67_mgmt_ap_enable(dev, &params);
}

int wifi_driver_ap_stop(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	return wifi_st67_mgmt_ap_disable(dev);
}

int wifi_driver_get_mac(uint8_t *mac_out)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	struct wifi_st67_data *data = dev->data;

	memcpy(mac_out, data->mac, 6);
	return 0;
}

const char *wifi_driver_get_ip(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return "";
	}
	struct wifi_st67_data *data = dev->data;

	if (!data->iface) {
		return "";
	}

	struct in_addr *addr = net_if_ipv4_get_global_addr(data->iface,
							   NET_ADDR_PREFERRED);

	if (addr) {
		net_addr_ntop(AF_INET, addr,
			      data->ip_str, sizeof(data->ip_str));
		return data->ip_str;
	}
	return "";
}

const char *wifi_driver_get_ncp_version(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return "unavailable";
	}
	return ((struct wifi_st67_data *)dev->data)->ncp_version;
}

bool wifi_driver_is_connected(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return false;
	}
	return ((struct wifi_st67_data *)dev->data)->connected;
}

struct net_if *wifi_driver_get_iface(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return NULL;
	}
	return ((struct wifi_st67_data *)dev->data)->iface;
}

int wifi_driver_diag(char *buf, size_t len)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return snprintf(buf, len, "WiFi driver not ready");
	}

	struct wifi_st67_data *data = dev->data;

	/* Query connected stations via AT+CWLIF */
	char at_rsp[256];
	int ret = wifi_st67_at_cmd(dev, "AT+CWLIF\r\n", at_rsp, sizeof(at_rsp));
	const char *clients = (ret == 0 && at_rsp[0] != '\0') ? at_rsp : "(none)";

	return snprintf(buf, len,
		"WiFi interface: %s\n"
		"MAC:       %02X:%02X:%02X:%02X:%02X:%02X\n"
		"AP active: %s\n"
		"RX frames: %u (%u bytes)\n"
		"TX frames: %u (%u bytes)\n"
		"RX errors: %u\n"
		"Clients:\n%s",
		data->iface ? "up" : "down",
		data->mac[0], data->mac[1], data->mac[2],
		data->mac[3], data->mac[4], data->mac[5],
		data->ap_active ? "yes" : "no",
		data->rx_frames, data->rx_bytes,
		data->tx_frames, data->tx_bytes,
		data->rx_errors,
		clients);
}

/* Serialises concurrent wifi_driver_scan_json calls (web API + CLI can both call
 * this; the Phase-2 sleep must not be interrupted by a second caller
 * resetting the shared static rsp[] buffer). */
static K_MUTEX_DEFINE(scan_mutex);

/* Per-network entry accumulated across scan cycles */
struct scan_result_t {
	char ssid[WIFI_DRV_SSID_MAX_LEN + 1];
	int  rssi;
	int  channel;
	int  ecn;
};

/* Forward declaration */
static void scan_accumulate(const char *resp,
			    struct scan_result_t *results, int *count, int max);

/* --------------------------------------------------------------------------
 * Scan result cache — populated by the keepalive work item.  On-demand scans
 * read from this cache.  If the cache is stale, the on-demand scan waits for
 * the keepalive to refresh it rather than sending competing CWLAPs.
 * Protected by scan_mutex.
 * --------------------------------------------------------------------------*/
static struct scan_result_t scan_results_buf[SCAN_MAX_ENTRIES];
static int  scan_results_count;
static int64_t scan_cache_uptime;  /* k_uptime_get() when cache was last filled */

/* Cache is considered fresh if populated within the last 60s */
#define SCAN_CACHE_MAX_AGE_MS  60000

/* --------------------------------------------------------------------------
 * Scan keepalive — periodic background CWLAP that caches results.
 *
 * The NCP requires two consecutive AT+CWLAP commands to produce results:
 * the first triggers an internal scan, the second (after a short delay)
 * reads the cached results.  This work item fires every 25s and stores
 * any results in scan_results_buf for on-demand reads.
 * --------------------------------------------------------------------------*/

static void scan_keepalive_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct wifi_st67_data *data = dev->data;

	if (!data->init_done || (!data->ap_active && !data->connected)) {
		return; /* NCP not ready or neither AP nor STA up */
	}

	static char keepalive_rsp[2048];

	if (k_mutex_lock(&scan_mutex, K_MSEC(100)) == 0) {
		/* On the very first kick in AP mode, re-assert CWMODE=3 to prime
		 * the NCP's STA scan engine.  Skip this when in pure STA mode —
		 * CWLAP works directly in CWMODE=1 and setting CWMODE=3 would
		 * restart the AP. */
		if (scan_keepalive_kicks == 0 && data->ap_active) {
			(void)wifi_st67_at_cmd(dev, "AT+CWMODE=3,0\r\n",
					       keepalive_rsp,
					       sizeof(keepalive_rsp));
			LOG_INF("keepalive: primed STA scan engine (CWMODE=3)");
			k_msleep(5000);
		}

		/* Send first CWLAP to trigger a scan cycle in the NCP. */
		memset(keepalive_rsp, 0, sizeof(keepalive_rsp));
		(void)wifi_st67_at_cmd(dev, "AT+CWLAP\r\n",
				       keepalive_rsp,
				       sizeof(keepalive_rsp));

		/* If the first already returned results, cache them. */
		if (scan_rsp_has_real_entry(keepalive_rsp)) {
			scan_results_count = 0;
			scan_accumulate(keepalive_rsp, scan_results_buf,
					&scan_results_count,
					SCAN_MAX_ENTRIES);
			scan_cache_uptime = k_uptime_get();
			LOG_INF("keepalive: cached %d networks",
				scan_results_count);
		} else {
			/* Wait for NCP scan to complete, then read results
			 * with a second CWLAP.  3-5s is enough for NCP to
			 * return cached results from the trigger above. */
			k_msleep(3000);

			memset(keepalive_rsp, 0, sizeof(keepalive_rsp));
			int ret = wifi_st67_at_cmd(dev, "AT+CWLAP\r\n",
						   keepalive_rsp,
						   sizeof(keepalive_rsp));

			if ((ret == 0 || ret == -ETIMEDOUT) &&
			    scan_rsp_has_real_entry(keepalive_rsp)) {
				scan_results_count = 0;
				scan_accumulate(keepalive_rsp, scan_results_buf,
						&scan_results_count,
						SCAN_MAX_ENTRIES);
				scan_cache_uptime = k_uptime_get();
				LOG_INF("keepalive: cached %d networks",
					scan_results_count);
			} else {
				LOG_DBG("keepalive[%d]: no results (ret=%d)",
					scan_keepalive_kicks, ret);
			}
		}

		k_mutex_unlock(&scan_mutex);
	}

	scan_keepalive_kicks++;
	k_work_reschedule(&scan_keepalive_work,
			  K_MSEC(SCAN_KEEPALIVE_INTERVAL_MS));
}

/* Parse one CWLAP response buffer and merge any new APs into results[].
 * Deduplicates by SSID, keeping the entry with the strongest RSSI. */
static void scan_accumulate(const char *resp,
			    struct scan_result_t *results, int *count, int max)
{
	const char *line = resp;
	int total_lines = 0;
	int parsed_ok = 0;
	int filtered = 0;

	while ((line = strstr(line, "+CWLAP:(")) != NULL) {
		int ecn, rssi, channel;
		char ssid[WIFI_DRV_SSID_MAX_LEN + 1];

		total_lines++;

		if (parse_cwlap_line(line, &ecn, ssid, sizeof(ssid),
				     &rssi, &channel) == 0) {
			parsed_ok++;
			LOG_DBG("  raw AP: SSID='%s' rssi=%d ch=%d",
				ssid[0] ? ssid : "<hidden>", rssi, channel);
			if (ssid[0] == '\0') {
				filtered++;  /* hidden network */
				line += 8;
				continue;
			}
			if (rssi == 0 && channel == 0) {
				filtered++;  /* placeholder */
				line += 8;
				continue;
			}

			/* Check for existing entry with same SSID */
			bool found = false;

			for (int i = 0; i < *count; i++) {
				if (strcmp(results[i].ssid, ssid) == 0) {
					found = true;
					/* Keep the stronger signal */
					if (rssi > results[i].rssi) {
						results[i].rssi    = rssi;
						results[i].channel = channel;
						results[i].ecn     = ecn;
					}
					break;
				}
			}
			if (!found && *count < max) {
				strncpy(results[*count].ssid, ssid,
					WIFI_DRV_SSID_MAX_LEN);
				results[*count].ssid[WIFI_DRV_SSID_MAX_LEN] = '\0';
				results[*count].rssi    = rssi;
				results[*count].channel = channel;
				results[*count].ecn     = ecn;
				(*count)++;
			}
		} else {
			LOG_WRN("wifi scan: parse failed: %.80s", line);
		}
		line += 8;
	}

	LOG_INF("wifi scan: raw=%d parsed=%d filtered=%d unique=%d",
		total_lines, parsed_ok, filtered, *count);
}

/* Helper: build JSON output from a results array */
static int scan_results_to_json(const struct scan_result_t *results, int count,
				char *out, size_t out_len)
{
	char *p   = out;
	char *end = out + out_len - 1U;

	if (p < end) {
		*p++ = '[';
	}

	bool first   = true;
	int  entries = 0;

	for (int i = 0; i < count && p < end; i++) {
		const char *sec;

		switch (results[i].ecn) {
		case 0:  sec = "Open";       break;
		case 1:  sec = "WEP";        break;
		case 2:  sec = "WPA";        break;
		case 3:  sec = "WPA2";       break;
		case 4:  sec = "WPA/WPA2";   break;
		case 5:  sec = "WPA2-Ent";   break;
		case 6:  sec = "WPA3";       break;
		case 7:  sec = "WPA2/WPA3";  break;
		case 8:  sec = "WAPI";       break;
		case 9:  sec = "OWE";        break;
		default: sec = "Unknown";    break;
		}

		int n = snprintf(p, (size_t)(end - p),
			"%s{\"ssid\":\"%s\",\"rssi\":%d,"
			"\"channel\":%d,\"security\":\"%s\"}",
			first ? "" : ",",
			results[i].ssid, results[i].rssi,
			results[i].channel, sec);

		if (n > 0 && n < (int)(end - p)) {
			p    += n;
			first = false;
			entries++;
		} else {
			break;
		}
	}

	if (p < end) {
		*p++ = ']';
	}
	*p = '\0';
	return (int)(p - out);
}

int wifi_driver_scan_json(const struct device *dev, char *out, size_t out_len)
{
	if (!out || out_len == 0) {
		return -EINVAL;
	}

	const struct device *d = dev ? dev : DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(d)) {
		return -ENODEV;
	}

	/*
	 * Cache-first scan strategy:
	 *
	 * The keepalive work fires AT+CWLAP every 20s and caches any results
	 * it finds.  On-demand scans first check the cache.  If the cache is
	 * fresh, return it immediately (no SPI traffic, no race conditions).
	 *
	 * If the cache is empty/stale, release the mutex and wait for the
	 * keepalive to populate it (up to ~45s).  This avoids sending
	 * competing CWLAPs that interfere with the NCP's scan engine.
	 */

	/* Fast path: check if cache is fresh */
	k_mutex_lock(&scan_mutex, K_FOREVER);

	if (scan_results_count > 0 && scan_cache_uptime > 0) {
		int64_t age = k_uptime_get() - scan_cache_uptime;

		if (age < SCAN_CACHE_MAX_AGE_MS) {
			int len = scan_results_to_json(scan_results_buf,
						       scan_results_count,
						       out, out_len);
			LOG_INF("scan_json: cache hit (%d networks, age %d ms)",
				scan_results_count, (int)age);
			k_mutex_unlock(&scan_mutex);
			return len;
		}
	}
	k_mutex_unlock(&scan_mutex);

	/* Slow path: cache is empty or stale.  Wait for keepalive to
	 * populate it.  Poll every 5s for up to 45s total. */
	LOG_INF("scan_json: cache empty/stale, waiting for keepalive...");

	for (int i = 0; i < 9; i++) {
		k_msleep(5000);

		k_mutex_lock(&scan_mutex, K_FOREVER);
		if (scan_results_count > 0 && scan_cache_uptime > 0) {
			int64_t age = k_uptime_get() - scan_cache_uptime;

			if (age < SCAN_CACHE_MAX_AGE_MS) {
				int len = scan_results_to_json(
					scan_results_buf,
					scan_results_count, out, out_len);
				LOG_INF("scan_json: got %d networks after %d s wait",
					scan_results_count, (i + 1) * 5);
				k_mutex_unlock(&scan_mutex);
				return len;
			}
		}
		k_mutex_unlock(&scan_mutex);
	}

	LOG_WRN("scan_json: no results after 45s wait");
	return -ETIMEDOUT;
}

/* Restart scan keepalive (e.g. after STA connects or AP restarts). */
void wifi_driver_scan_start_refresh(void)
{
#ifndef CONFIG_ZTEST
	scan_keepalive_kicks = 0;
	k_work_reschedule(&scan_keepalive_work, K_MSEC(3000));
	LOG_INF("Scan keepalive restarted");
#endif
}

void wifi_driver_scan_stop_refresh(void)
{
#ifndef CONFIG_ZTEST
	k_work_cancel_delayable(&scan_keepalive_work);
	LOG_INF("Scan keepalive stopped");
#endif
}

/* --------------------------------------------------------------------------
 * Driver registration
 * --------------------------------------------------------------------------*/

int wifi_driver_at_cmd(const char *cmd, char *rsp, size_t rsp_len)
{
#ifdef CONFIG_ZTEST
	ARG_UNUSED(cmd);
	if (rsp && rsp_len > 0) {
		rsp[0] = '\0';
	}
	return 0;
#else
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	/* Append \r\n if caller omitted it */
	char buf[128];
	size_t len = strlen(cmd);

	if (len >= 2 && cmd[len - 2] == '\r' && cmd[len - 1] == '\n') {
		return wifi_st67_at_cmd(dev, cmd, rsp, rsp_len);
	}
	snprintf(buf, sizeof(buf), "%s\r\n", cmd);
	return wifi_st67_at_cmd(dev, buf, rsp, rsp_len);
#endif
}

static const struct net_wifi_mgmt_offload wifi_st67_api = {
	.wifi_iface = {
		.iface_api.init  = wifi_st67_iface_init,
		.send            = wifi_st67_send,
		.get_capabilities = wifi_st67_get_caps,
	},
	.wifi_mgmt_api = &wifi_st67_mgmt_ops,
};

#define WIFI_ST67_DEFINE(n)							\
	static const struct wifi_st67_cfg wifi_cfg_##n = {			\
		.bus        = SPI_DT_SPEC_INST_GET(n, WIFI_SPI_OPER, 0),	\
		.data_ready = GPIO_DT_SPEC_INST_GET(n, data_ready_gpios),	\
		.boot       = GPIO_DT_SPEC_INST_GET_OR(n, boot_gpios, {0}),	\
		.chip_en    = GPIO_DT_SPEC_INST_GET_OR(n, chip_en_gpios, {0}),	\
	};									\
	static struct wifi_st67_data wifi_data_##n;				\
	NET_DEVICE_DT_INST_DEFINE(n, wifi_st67_hw_init, NULL,			\
				  &wifi_data_##n, &wifi_cfg_##n,		\
				  CONFIG_WIFI_INIT_PRIORITY,			\
				  &wifi_st67_api,				\
				  ETHERNET_L2,					\
				  NET_L2_GET_CTX_TYPE(ETHERNET_L2),		\
				  NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(WIFI_ST67_DEFINE)

#endif /* CONFIG_ZTEST */
