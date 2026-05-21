/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST67W611M1 Wi-Fi NCP HAL driver — public API header.
 */

#ifndef WIFI_DRIVER_H
#define WIFI_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct device;
struct net_if;

/** Maximum SSID length (matches Zephyr WIFI_SSID_MAX_LEN). */
#define WIFI_DRV_SSID_MAX_LEN 32

/** Maximum passphrase length (matches Zephyr WIFI_PSK_MAX_LEN). */
#define WIFI_DRV_PSK_MAX_LEN  64

/** Maximum IP address string length (NNN.NNN.NNN.NNN\0). */
#define WIFI_DRV_IP_MAX_LEN   16

/**
 * @brief Initialise the Wi-Fi HAL driver.
 *
 * Configures SPI bus, DATA_READY GPIO interrupt, and queries the NCP
 * firmware version. Must be called once before any other wifi_driver_*
 * function. In CONFIG_ZTEST mode initialises simulation state only.
 *
 * @return 0 on success, negative errno on failure.
 */
int wifi_driver_init(void);

/**
 * @brief Connect to a Wi-Fi network in STA mode.
 *
 * Sends the appropriate AT command to the NCP to join the specified
 * network. The call is asynchronous — connection state changes are
 * signalled via Zephyr's wifi_mgmt events. In CONFIG_ZTEST mode the
 * simulated connected state is set immediately.
 *
 * @param ssid     Network SSID (NUL-terminated, max WIFI_DRV_SSID_MAX_LEN).
 * @param password WPA2 passphrase (NUL-terminated, max WIFI_DRV_PSK_MAX_LEN).
 * @return 0 on success (AT command accepted), negative errno on error.
 */
int wifi_driver_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from the current Wi-Fi network.
 *
 * @return 0 on success, negative errno on error.
 */
int wifi_driver_disconnect(void);

/**
 * @brief Start a Wi-Fi soft access point.
 *
 * @param ssid     AP broadcast SSID.
 * @param password WPA2 passphrase (empty string for open network).
 * @return 0 on success, negative errno on error.
 */
int wifi_driver_ap_start(const char *ssid, const char *password);

/**
 * @brief Stop the Wi-Fi soft access point.
 *
 * @return 0 on success, negative errno on error.
 */
int wifi_driver_ap_stop(void);

/**
 * @brief Read the NCP module MAC address.
 *
 * @param mac_out Buffer of at least 6 bytes to receive the MAC address.
 * @return 0 on success, negative errno on error.
 */
int wifi_driver_get_mac(uint8_t *mac_out);

/**
 * @brief Get the current IP address as a string.
 *
 * Returns a pointer to an internal buffer. Valid until the next call.
 * Returns an empty string if no IP is assigned.
 */
const char *wifi_driver_get_ip(void);

/**
 * @brief Get the NCP firmware version string.
 *
 * Returns a pointer to an internal buffer populated at init time.
 * Format: "vMAJOR.MINOR.PATCH" (e.g. "v2.0.106").
 */
const char *wifi_driver_get_ncp_version(void);

/**
 * @brief Return true if the STA interface is connected to an AP.
 */
bool wifi_driver_is_connected(void);

/**
 * @brief Return the Wi-Fi net_if pointer.
 *
 * Used by wifi_manager to assign an AP-mode static IP and start a DHCP
 * server on the Wi-Fi interface. Returns NULL in CONFIG_ZTEST mode or
 * if the driver is not yet initialised.
 */
struct net_if *wifi_driver_get_iface(void);

/**
 * @brief Write WiFi diagnostic info into buf.
 *
 * Includes packet counters, AP state, and connected clients.
 *
 * @param buf Output buffer.
 * @param len Buffer size.
 * @return Number of characters written (snprintf semantics).
 */
int wifi_driver_diag(char *buf, size_t len);

/**
 * @brief Trigger a Wi-Fi scan and return results as a JSON array.
 *
 * Sends AT+CWLAP to the NCP, parses the response, and writes a JSON array
 * of nearby networks into @p out. Blocks until the NCP responds (~2-3 seconds).
 *
 * @param dev   WiFi device (pass NULL to use the first registered wifi device).
 * @param out   Output buffer for JSON string.
 * @param out_len  Size of @p out.
 * @return Length of JSON string written (excluding NUL), or negative on error.
 */
int wifi_driver_scan_json(const struct device *dev, char *out, size_t out_len);

/**
 * @brief Start periodic background scan refresh.
 *
 * Called automatically after AP is up. Triggers AT+CWLAP every 25s so
 * the scan cache stays warm and user-triggered scans return instantly.
 */
void wifi_driver_scan_start_refresh(void);

/**
 * @brief Stop periodic background scan refresh.
 *
 * Should be called before STA connect attempts to avoid SPI contention
 * between CWLAP and CWJAP.
 */
void wifi_driver_scan_stop_refresh(void);

/**
 * @brief Send a raw AT command to the NCP and return the response.
 *
 * Intended for diagnostics only. Appends \r\n if the caller omits it.
 *
 * @param cmd     AT command string (e.g. "AT+CWCOUNTRY?").
 * @param rsp     Output buffer for NCP response.
 * @param rsp_len Size of @p rsp.
 * @return 0 on OK, negative on error.
 */
int wifi_driver_at_cmd(const char *cmd, char *rsp, size_t rsp_len);

/**
 * @brief Poll STA connection state via AT+CWSTATE?.
 *
 * Non-blocking query to determine async CWJAP result.
 *
 * @return 0 = idle/disconnected, 1 = connecting, 2 = connected,
 *         3 = disconnecting, 4 = disconnected, <0 = AT error.
 */
int wifi_driver_poll_sta_state(void);

/* -----------------------------------------------------------------------
 * NCP Firmware Update (FWU) over SPI
 * ----------------------------------------------------------------------- */

/**
 * @brief Start or stop an NCP firmware update session.
 *
 * Sends AT+OTASTART=1 to begin or AT+OTASTART=0 to abort.
 * Must be called before wifi_driver_fwu_send().
 *
 * @param enable  true to start, false to abort/terminate.
 * @return 0 on success, negative errno on failure.
 */
int wifi_driver_fwu_start(bool enable);

/**
 * @brief Send a firmware chunk to the NCP.
 *
 * Sends AT+OTASEND=<len>, waits for '>' prompt, then sends raw binary data.
 * The first chunk must be a 512-byte OTA header. Subsequent chunks should
 * be 256-byte aligned for optimal NCP flash writes.
 *
 * @param data  Firmware binary chunk.
 * @param len   Length of chunk in bytes (max SPI_XFER_MAX_PAYLOAD - 32).
 * @return 0 on success, negative errno on failure.
 */
int wifi_driver_fwu_send(const uint8_t *data, size_t len);

/**
 * @brief Finish FWU and reboot the NCP into new firmware.
 *
 * Sends AT+OTAFIN. The NCP will reboot — allow ~5s before re-initialising
 * the wifi driver.
 *
 * @return 0 on success, negative errno on failure.
 */
int wifi_driver_fwu_finish(void);

#endif /* WIFI_DRIVER_H */
