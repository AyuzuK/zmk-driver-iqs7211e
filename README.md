# ZMK IQS7211E ドライバ

このドライバは、Azoteq IQS7211E静電容量式タッチセンサーをZMKフレームワークで使用できるようにします。

## 概要

IQS7211Eは、トラックパッドアプリケーション向けに設計された低消費電力(1.5mA,データシートより)のタッチコントローラーです。このドライバはI2Cインターフェースを介してIQS7211Eセンサーと通信し、ZMKにマウス移動入力を提供します。

## 機能

- 指位置検出による静電容量式タッチトラッキング
- 2本指までのマルチタッチ
  - 1本指の移動でカーソル移動
  - 1本指のタップで左クリック
  - 2本指の移動でスクロール
  - 2本指のタップで右クリック

## インストール

1. ZMKモジュールとして追加：

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: sekigon-gonnoc
      url-base: https://github.com/sekigon-gonnoc
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-driver-iqs7211e
      remote: sekigon-gonnoc
```

## デバイスツリー設定

シールドまたはボード設定ファイル（.overlayまたは.dtsi）で設定：

```dts
&pinctrl {
    i2c0_default: i2c0_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 17)>,
                    <NRF_PSEL(TWIM_SCL, 0, 20)>;
            bias-pull-up;
        };
    };

    i2c0_sleep: i2c0_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 17)>,
                    <NRF_PSEL(TWIM_SCL, 0, 20)>;
            bias-pull-up;
            low-power-enable;
        };
    };
};

&i2c0 {
    status = "okay";
    compatible = "nordic,nrf-twim";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";
    clock-frequency = <I2C_BITRATE_FAST>;
    zephyr,concat-buf-size = <32>;

    trackpad: trackpad@56 {
        status = "okay";
        compatible = "azoteq,iqs7211e";
        reg = <0x56>;
        irq-gpios = <&gpio0 21 GPIO_PULL_UP>;
        power-gpios = <&gpio0 22 GPIO_ACTIVE_HIGH>;
        scroller-mode;
        // v-invert;
    };
};
```

## キーボードのKconfigファイルでモジュールを有効化

キーボードの `Kconfig.defconfig` に以下を追加：

```kconfig
if ZMK_KEYBOARD_YOUR_KEYBOARD

config ZMK_POINTING
    default y

config IQS7211E
    default y

endif
```

## プロパティ

- `irq-gpios`: RDY/モーションピンに接続されたGPIO（必須、アクティブロー）
- `power-gpios`: 電源制御ピンに接続されたGPIO（任意、アクティブハイ）
- `scroller-mode`: 指移動を `INPUT_REL_WHEEL`, `INPUT_REL_HWHEEL` として送出する（任意）
- `gesture-mode`: タップとスワイプを、クリックではなくジェスチャーの合図（`INPUT_BTN_5`〜`INPUT_BTN_8`）として送出する（任意）
- `v-invert`: `scroller-mode` 時の縦スクロール方向を反転する（任意）
- `h-invert`: `scroller-mode` 時の横スクロール方向を反転する（任意）

### Scroller mode と慣性スクロール

- `scroller-mode` を指定すると、指移動はカーソル移動ではなくホイール入力として送出されます。
- タップによるクリック（1本指タップ=左クリック、2本指タップ=右クリック）はそのまま有効です。
- `scroller-mode` では、`HWheel` ゾーン内で横移動すると `INPUT_REL_HWHEEL` を送出します。
- `HWheel` ゾーンは Kconfig の `MIN/MAX` で帯域を指定し、上側基準/下側基準を切り替えできます。
- `scroller-mode` では、スクロール開始後の軸を固定します。
  `V scroll` 中に H ゾーンへ入っても `H scroll` へ切り替わらず、逆も同様です。
- 慣性スクロールはKconfigで制御します。

```kconfig
CONFIG_IQS7211E_TAP_EDGE_MARGIN_PERMILLE=80
CONFIG_IQS7211E_SCROLLER_HWHEEL_ZONE_MIN_PERMILLE=0
CONFIG_IQS7211E_SCROLLER_HWHEEL_ZONE_MAX_PERMILLE=150
CONFIG_IQS7211E_SCROLLER_HWHEEL_ZONE_FROM_BOTTOM=n
```

- `IQS7211E_TAP_EDGE_MARGIN_PERMILLE`: 周辺タップ無効のマージン（permille）。
- `IQS7211E_SCROLLER_HWHEEL_ZONE_MIN_PERMILLE`, `MAX`: `HWheel` 帯域を `[MIN, MAX)` で指定。

```kconfig
CONFIG_IQS7211E_SCROLLER_INERTIA=y
CONFIG_IQS7211E_SCROLLER_INERTIA_DECAY_PERMILLE=920
CONFIG_IQS7211E_SCROLLER_INERTIA_STOP_THRESHOLD_Q8=96
```

- 慣性スクロールの送出間隔は 10ms 固定です。
- 慣性は `WHEEL` と `HWHEEL` の両方に同じ減衰特性で適用されます。
- 実装は固定小数点（Q8）で、FPUを使いません。

### カーソルの移動開始しきい値

```kconfig
CONFIG_IQS7211E_MOTION_START_THRESHOLD=25
```

- 指を置いた位置からの距離（`|dx| + |dy|`、センサー座標）がこの値に達するまで、`INPUT_REL_X` / `INPUT_REL_Y` を送出しません。
- しきい値に達するまでの移動量（達した回を含む）は捨てるため、カーソルが急に動くことはありません。
- 2本指から1本指に戻ったときは、その位置を開始位置として取り直します。
- `scroller-mode` と2本指スクロールには適用しません。
- 既定値は `0`（無効）です。

### Gesture mode

`gesture-mode` を指定すると、タップとスワイプを次の合図として送出します。
合図は press の後、一定時間で release します。

| 操作 | 合図 |
|---|---|
| 1本指タップ（2タップ待ち時間が過ぎたとき） | `INPUT_BTN_5` |
| 2タップ（待ち時間内に2回目のタップ） | `INPUT_BTN_6` |
| 上スワイプ | `INPUT_BTN_7` |
| 下スワイプ | `INPUT_BTN_8` |

- タップの左クリック（`INPUT_BTN_0`）、2本指タップの右クリック（`INPUT_BTN_1`）、ダブルタップの押しっぱなしは送出しません。
- カーソル移動と2本指スクロールは、これまでどおり送出します。`scroller-mode` とは独立しています。
- 1タップの合図は、2タップ待ち時間の分だけ遅れて送出されます。
- 2回目のタッチがタップで終わらなかった場合（スワイプなど）は、先に1タップの合図を送ってから、次の判定を行います。
- 途中で2本指になったタッチでは、合図を送りません。
- スワイプは、指を置いてから離すまでの縦の移動量で判定します（縦の移動量が横より大きいこと）。edge margin はタップにだけ適用し、スワイプには適用しません。
- 合図はマウスボタンとしては扱われないため、keymap 側で `zmk,input-processor-behaviors` を使って任意の behavior に割り当てます。

```kconfig
CONFIG_IQS7211E_GESTURE_DOUBLE_TAP_WAIT_MS=300
CONFIG_IQS7211E_GESTURE_TAP_TIME_MS=200
CONFIG_IQS7211E_GESTURE_TAP_DISTANCE=50
CONFIG_IQS7211E_GESTURE_SWIPE_DISTANCE_PERMILLE=200
CONFIG_IQS7211E_GESTURE_SWIPE_MAX_TIME_MS=500
CONFIG_IQS7211E_GESTURE_SIGNAL_RELEASE_MS=30
```

- 上記の値は既定値です。`gesture-mode` を指定したときだけ使われます。
- `IQS7211E_GESTURE_SWIPE_DISTANCE_PERMILLE`: スワイプと判定する縦の移動量（Y解像度に対する割合）。

## ハードウェアセットアップ

### I2Cアドレス
IQS7211Eはデフォルトで I2Cアドレス `0x56` を使用します。

### ピン接続
- **VDD**: 3.3V電源供給
- **GND**: グランド
- **SDA**: I2Cデータライン
- **SCL**: I2Cクロックライン
- **RDY**: Ready/Motionピン（アクティブロー、IRQ GPIOに接続）

### 電源制御（任意）
電源制御GPIOを使用する場合：
- センサーへの電源供給を制御するGPIOピンに接続
- スリープモード中の完全電源遮断に有用

## 設定

ドライバは `iqs7211e_init.h` の事前定義された設定を使用します：
- タッチ感度閾値
- ATI（自動調整実装）パラメータ
- 電源モード設定
- レポートレートとタイミング

これらの設定は使用するトラックパッドの構成に合わせて変更してください。デフォルトでは[低消費電力円形トラックパッド](https://nogikes.booth.pm/items/7254791)用の設定になっています。

インスタンスごとの初期化データ（シンボル指定）

- 各デバイスツリーのノードに `init-symbol` と `init-length` プロパティを追加することで、そのノード専用の初期化データ（C シンボル）を指定できます。
- `init-symbol` は C 側で定義された `const uint8_t` 配列の識別子（C識別子）を指し、`init-length` はその配列のバイト長を指定します。
- 例（`.overlay` または board `.dtsi`）:

```dts
&i2c0 {
  trackpad0: trackpad@56 {
    status = "okay";
    compatible = "azoteq,iqs7211e";
    reg = <0x56>;
    irq-gpios = <&gpio0 21 GPIO_ACTIVE_LOW>;
    init-symbol = "trackpad0_iqs7211e_init";
  };
};

&i2c1 {
  trackpad1: trackpad@56 {
    status = "okay";
    compatible = "azoteq,iqs7211e";
    reg = <0x56>;
    irq-gpios = <&gpio0 22 GPIO_ACTIVE_LOW>;
    init-symbol = "trackpad1_iqs7211e_init";
  };
};
```

- 指定したシンボルはボード/プロジェクト側で次のように定義してください（例）:

```c
// board/init_data.c
#include <stdint.h>
const uint8_t trackpad0_iqs7211e_init[] = { /* レコード形式の初期化データ */ };
```

- フォールバック: `init-symbol` を指定しない場合、ドライバはモジュール内で提供する既定シンボル `iqs7211e_init_default`を使用します。

- 初期化データ形式:
  - ドライバは各レコードを `[register_addr, length, data...]` の繰り返しで処理します。
  - 各レコードはそのレジスタアドレスへ `length` バイトのブロック書き込みを行います。

## トラブルシューティング

### デバイスが見つからない
- I2C配線を確認
- 電源供給（3.3V）を確認
- I2Cアドレス（デフォルト0x56）を確認

### 移動検出しない
- IRQ GPIO接続と設定を確認
- RDYピンが接続されているか確認（アクティブロー）

### 初期化失敗
- 十分な電源供給電流容量を確保
- I2Cバス競合を確認
- タイミング要件が満たされているか確認

### タッチ感度が悪い
- 初期化ファイルでATIパラメータの調整を検討

## 使用例

設定が完了すると、トラックパッドは自動的に移動を相対マウス入力としてZMK入力システムにレポートします。キーマップでの追加設定は不要です。
