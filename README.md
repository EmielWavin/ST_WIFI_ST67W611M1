# ST67W611M1 Wi-Fi NCP Driver (Zephyr Module)

Zephyr west module providing a HAL driver for the **ST67W611M1** Wi-Fi NCP
(Network Co-Processor). The driver communicates with the NCP over SPI using the
T02 "host-LwIP" firmware profile — the host runs the full IP stack while the
NCP handles 802.11 radio operations.

## Features

- SPI binary-framed protocol with DATA_READY interrupt
- AT command interface (ESP-AT compatible subset)
- STA connect/disconnect, AP start/stop
- Wi-Fi scan (broadcast + directed SSID)
- MAC address and IP queries
- Raw AT passthrough for diagnostics
- Full `#ifdef CONFIG_ZTEST` simulation for unit testing

## Hardware

- **NCP module:** ST67W611M1 (X-NUCLEO-67W61M1 shield)
- **NCP firmware:** T02 host-LwIP profile (v2.0.106+ recommended)
- **Interface:** SPI (full-duplex, 8-byte framed header) + GPIO DATA_READY interrupt
- **Reference board:** NUCLEO-H563ZI (any Zephyr-supported board with SPI works)

## Requirements

- Zephyr RTOS v3.7.x
- Board devicetree overlay with `compatible = "st,st67w611m1"` node
- `CONFIG_SPI=y` and `CONFIG_GPIO=y` in project Kconfig

## Integration

### 1. Add to your `west.yml`

```yaml
manifest:
  projects:
    - name: ST_WIFI_ST67W611M1
      url: https://github.com/EmielWavin/ST_WIFI_ST67W611M1
      revision: v1.0.0
      path: modules/lib/st_wifi_st67w611m1
```

### 2. Enable in your project's Kconfig (`prj.conf`)

```
CONFIG_WIFI_ST67W611M1=y
```

### 3. Add devicetree overlay

```dts
&spi1 {
    status = "okay";

    st67w611m1: wifi@0 {
        compatible = "st,st67w611m1";
        reg = <0>;
        spi-max-frequency = <24000000>;
        data-ready-gpios = <&gpioe 13 GPIO_ACTIVE_HIGH>;
        boot-gpios = <&gpioe 9 GPIO_ACTIVE_HIGH>;
        chip-en-gpios = <&gpioe 11 GPIO_ACTIVE_HIGH>;
    };
};
```

### 4. Include the header

```c
#include "wifi_driver.h"

int main(void) {
    wifi_driver_init();
    wifi_driver_connect("MyNetwork", "password123");
}
```

## Kconfig Options

| Symbol | Default | Description |
|--------|---------|-------------|
| `CONFIG_WIFI_ST67W611M1` | `n` | Enable the driver |
| `CONFIG_WIFI_ST67W611M1_LOG_LEVEL_*` | `INF` | Log verbosity (DBG/INF/WRN/ERR) |

## Unit Tests

Tests use the `CONFIG_ZTEST` simulation path (no hardware needed):

```sh
west build -b native_sim tests/wifi_driver -- -DCONFIG_WIFI_ST67W611M1=y
./build/zephyr/zephyr.exe
```

## License

Apache-2.0. See [LICENSE](LICENSE).
