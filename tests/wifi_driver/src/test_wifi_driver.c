/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the ST67W611M1 Wi-Fi NCP driver (simulation path).
 */

#include <string.h>
#include <zephyr/ztest.h>
#include "wifi_driver.h"

extern void wifi_test_set_scan_fail_count(int n);

ZTEST(wifi_scan_suite, test_scan_json_returns_array)
{
	char buf[256];
	int n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n > 0, "scan should return positive length");
	zassert_equal(buf[0], '[', "result should start with '['");
	zassert_equal(buf[n - 1], ']', "result should end with ']'");
}

ZTEST(wifi_scan_suite, test_scan_json_contains_ssid)
{
	char buf[256];
	wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_not_null(strstr(buf, "\"ssid\""), "result should contain ssid key");
	zassert_not_null(strstr(buf, "TestNet"), "result should contain TestNet from stub");
}

ZTEST(wifi_scan_suite, test_scan_json_contains_rssi)
{
	char buf[256];
	wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_not_null(strstr(buf, "\"rssi\""), "result should contain rssi key");
	zassert_not_null(strstr(buf, "-60"), "result should contain rssi value from stub");
}

ZTEST(wifi_scan_suite, test_scan_json_small_buffer)
{
	char buf[4] = {0};
	int n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n >= 0, "should not return error with small buffer");
}

ZTEST(wifi_scan_suite, test_scan_json_null_terminated)
{
	char buf[256];
	memset(buf, 0xFF, sizeof(buf));
	int n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n > 0, "should have written data");
	zassert_equal(buf[n], '\0', "result should be null terminated");
}

ZTEST(wifi_scan_suite, test_scan_fail_injection_works)
{
	char buf[256];

	wifi_test_set_scan_fail_count(1);
	int n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n < 0, "injected failure should return error");
	n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n > 0, "should succeed after fail count exhausted");
	zassert_not_null(strstr(buf, "TestNet"), "should return fake data");
}

ZTEST(wifi_scan_suite, test_scan_zero_fail_count_succeeds_immediately)
{
	char buf[256];

	wifi_test_set_scan_fail_count(0);
	int n = wifi_driver_scan_json(NULL, buf, sizeof(buf));
	zassert_true(n > 0, "should succeed with zero fail count");
}

ZTEST_SUITE(wifi_scan_suite, NULL, NULL, NULL, NULL, NULL);
