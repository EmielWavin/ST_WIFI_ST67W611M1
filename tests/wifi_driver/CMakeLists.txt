# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(wifi_driver_test)

# The driver source is compiled by the module's own CMakeLists.txt
# (triggered by CONFIG_WIFI_ST67W611M1=y in prj.conf).
# We only need to add the test source here.
target_sources(app PRIVATE src/test_wifi_driver.c)
