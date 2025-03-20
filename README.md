# ZMK 74HC164 Shift Register Matrix Driver

This module provides a keyboard matrix scanning driver for ZMK that uses 74HC164 shift registers to control the columns of a keyboard matrix. This approach allows for keyboards with many columns while using only a few microcontroller pins.

## How It Works

The 74HC164 is an 8-bit serial-in, parallel-out shift register that can be daisy-chained to create larger outputs. In a keyboard matrix, this shift register can drive the columns while the rows are connected directly to GPIO pins on the microcontroller.

### Hardware Configuration

- **Data Pin**: Serial data input to the shift register
- **Clock Pin**: Clock signal for the shift register
- **Reset Pin**: Reset pin for the shift register (active low)
- **Row Pins**: Connected directly to the microcontroller as inputs (with pull-up resistors)
- **Power Pins**: Optional control pins for powering the matrix or sections of it

## Installation

To use this driver in your ZMK keyboard, you'll need to include this module in your ZMK config. Add the following to your `west.yml` file:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
  self:
    path: config
    
  # Add this section to include the module
  import:
    - name: zmk-modules
    
    # Add this in zmk-modules/west.yml
    projects:
      - name: zmk-driver-kscan-74hc164-matrix
        path: zmk-driver-kscan-74hc164-matrix
        url: https://github.com/your-username/zmk-driver-kscan-74hc164-matrix
        revision: main
```

## Device Tree Configuration

Add the following to your keyboard's `.dts` file:

```dts
/ {
    kscan0: kscan_0 {
        compatible = "zmk,kscan-74hc164-matrix";
        data-gpios = <&gpio0 1 GPIO_ACTIVE_HIGH>;
        clk-gpios = <&gpio0 2 GPIO_ACTIVE_HIGH>;
        reset-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;
        power-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
        row-gpios
            = <&gpio0 5 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>
            , <&gpio0 6 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>
            , <&gpio0 7 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>
            , <&gpio0 8 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>
            ;
        rows = <4>;
        columns = <8>;  // 8 columns from a single 74HC164
        debounce-press-ms = <5>;
        debounce-release-ms = <5>;
        polling-interval-ms = <10>;
    };
};
```

## Properties

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| `data-gpios` | phandle-array | Yes | GPIO for the data pin of the 74HC164 shift register |
| `clk-gpios` | phandle-array | Yes | GPIO for the clock pin of the 74HC164 shift register |
| `reset-gpios` | phandle-array | Yes | GPIO for the reset pin of the 74HC164 shift register |
| `power-gpios` | phandle-array | Yes | GPIOs for power control |
| `row-gpios` | phandle-array | Yes | GPIOs for row scanning of the matrix |
| `rows` | int | Yes | Number of rows in the matrix |
| `columns` | int | Yes | Number of columns in the matrix |
| `debounce-press-ms` | int | No | Debounce time for key press in milliseconds |
| `debounce-release-ms` | int | No | Debounce time for key release in milliseconds |
| `debounce-scan-period-ms` | int | No | Time between matrix scans while debouncing (default: 1ms) |
| `poll-period-ms` | int | No | Time between matrix scans when no keys are pressed (default: 10ms) |
| `wakeup-source` | boolean | No | Whether the matrix scan can be used as a wakeup source |

## プロパティ（日本語）

| プロパティ名 | 型 | 必須 | 説明 |
|----------|------|----------|-------------|
| `data-gpios` | phandle-array | はい | 74HC164シフトレジスタのデータピン用GPIO |
| `clk-gpios` | phandle-array | はい | 74HC164シフトレジスタのクロックピン用GPIO |
| `reset-gpios` | phandle-array | はい | 74HC164シフトレジスタのリセットピン用GPIO |
| `power-gpios` | phandle-array | はい | 電源制御用GPIO |
| `row-gpios` | phandle-array | はい | 行スキャン用GPIO |
| `rows` | int | はい | マトリックスの行数 |
| `columns` | int | はい | マトリックスの列数 |
| `debounce-press-ms` | int | いいえ | キー押下時のデバウンス時間（ミリ秒） |
| `debounce-release-ms` | int | いいえ | キー解放時のデバウンス時間（ミリ秒） |
| `debounce-scan-period-ms` | int | いいえ | デバウンス中のマトリックススキャン間隔（デフォルト: 1ms） |
| `poll-period-ms` | int | いいえ | キー未押下時のスキャン間隔（デフォルト: 10ms） |
| `wakeup-source` | boolean | いいえ | マトリックススキャンをウェイクアップソースとして使用するかどうか |

## Power Management

The driver supports power management through Zephyr's PM system. When the device enters a low-power state, the driver will:

1. Disable scanning and interrupts
2. Power off the matrix
3. Disconnect GPIOs to save power

When resuming, the driver will reinitialize the hardware and continue scanning.

## 電源管理（日本語）

このドライバはZephyrの電源管理システムをサポートしています。デバイスが低電力状態に入ると、ドライバは以下の処理を行います：

1. スキャンと割り込みを無効化
2. マトリックスの電源をオフ
3. 省電力のためにGPIOを切断

復帰時には、ハードウェアを再初期化し、スキャンを再開します。

## License

This driver is released under the MIT License. See the LICENSE file for details.

## ライセンス（日本語）

このドライバはMITライセンスの下でリリースされています。詳細はLICENSEファイルを参照してください。
