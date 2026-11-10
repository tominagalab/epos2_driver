# epos2_driver

## 1. 概要

Maxon EPOS2をUSBで操作するROS 2 Humble向けC++パッケージです。
MaxonのEPOS Command Libraryを使い、モータの有効化・無効化と速度指令（Profile Velocity Mode）を行います。

> **重要：安全に関する注意**
>
> 初回は、モータに接続した機構を動かしても人や物に危険がない状態にし、
> 可能であれば負荷を外して試してください。モータを動かすときは、すぐに
> 電源を切れる人が近くにいてください。このパッケージは非常停止装置ではなく、
> 安全を保証するものでもありません。

### ノード構成

| 項目 | 内容 |
|---|---|
| パッケージ名 | `epos2_driver` |
| 実行ファイル | `epos2_driver_node` |
| ノード名 | `epos2_driver` |
| 起動ファイル | [`launch/epos2_driver.launch.py`](./launch/epos2_driver.launch.py) |
| 制御モード | Profile Velocity Mode |

### トピック・サービス一覧

名前はすべて相対名です。ノードを名前空間付きで起動した場合は、その名前空間が先頭に付きます。
このノードはパブリッシャ（状態の配信）を持ちません。

#### Subscribe（購読）

| トピック名 | 型 | QoS | 内容 |
|---|---|---|---|
| `cmd_vel_rpm` | `std_msgs/msg/Int32` | Reliable、Volatile、深さ10（既定） | 目標速度 [rpm]。符号で回転方向が決まります。 |

- モータが無効のときに受信した指令は無視されます（5秒に1回、警告ログを出力）。
- 指令は受信するたびにEPOS2へ送られます。タイムアウトや指令値の上下限による制限はありません。
- 加減速は `profile_acceleration` / `profile_deceleration` に従います。

#### Service（サービス）

| サービス名 | 型 | 内容 |
|---|---|---|
| `enable` | `std_srvs/srv/SetBool` | `data: true` でモータを有効化、`data: false` で停止して無効化します。 |

`enable` のリクエストとレスポンス：

| `data` | 動作 | `success` | `message` の例 |
|---|---|---|---|
| `true` | 有効化済みなら何もしません。未有効なら、フォルトのクリア、Profile Velocity Modeの有効化、加減速の設定、Enable stateへの遷移を順に行います。 | 成功時 `true`、いずれかの手順が失敗したら `false` | `EPOS2 enabled`、`EPOS2 is already enabled`、失敗時は `<API名> failed (error 0x........)` |
| `false` | 無効化済みなら何もしません。有効なら速度指令を停止（Halt）してから無効化します。 | 成功時 `true`、停止または無効化に失敗したら `false` | `EPOS2 halted and disabled`、`EPOS2 is already disabled`、失敗時は `<API名> failed (error 0x........)` |

#### パラメータ

| パラメータ | 型 | 初期値 | 説明 |
|---|---|---:|---|
| `device_name` | string | `EPOS2` | EPOS Command Libraryに渡す機器名 |
| `protocol_stack_name` | string | `MAXON SERIAL V2` | 通信プロトコルスタック名 |
| `interface_name` | string | `USB` | 接続方式 |
| `port_name` | string | `USB0` | ライブラリに渡すUSBポート名 |
| `node_id` | int | `1` | EPOS2のノードID（1〜127） |
| `baudrate` | int | `1000000` | 通信速度（正の値） |
| `timeout_ms` | int | `500` | 通信タイムアウト [ms]（正の値） |
| `profile_acceleration` | int | `1000` | 速度制御の加速度（正の値）。有効化時に設定されます。 |
| `profile_deceleration` | int | `1000` | 速度制御の減速度（正の値）。有効化時に設定されます。 |

- パラメータは起動時にのみ読み込まれ、実行中に変更しても反映されません。
- 範囲外の値や接続失敗の場合、ノードはエラーを表示して終了します（終了コード1）。
- 初期値が、使用するEPOS2やモータに適しているとは限りません。値の意味、単位、許容範囲はMaxonの機器マニュアルで確認してください。

#### 状態遷移

| タイミング | モータの状態 |
|---|---|
| ノード起動時 | EPOS2へ接続し、Disable stateにします。 |
| `enable(true)` 成功後 | 有効。`cmd_vel_rpm` が受け付けられます。 |
| `enable(false)` 後 | 無効。`cmd_vel_rpm` は無視されます。 |
| ノード終了時 | 有効だった場合は停止して無効化し、デバイスを閉じます。 |

## 2. 必要なもの

- Ubuntu 22.04（Jammy）
- ROS 2 Humble（インストール済みであること）
- ビルドツール：`colcon`、`libusb-1.0-0-dev`、`unzip`、`wget`
- Maxon EPOS2と、PCに接続するためのUSBケーブル
- MaxonのEPOS Command Library（Linux版。入手方法は次章）
- EPOS2に接続するモータ、電源、エンコーダなどの機器

EPOS2本体の設定（ノードIDや通信設定など）が必要です。モータとEPOS2の配線、電源、
モータの種類に応じた設定は、本体のマニュアルに従ってください。

## 3. インストール・ビルド方法

### 3.1 ビルドツールの準備

```bash
sudo apt update
sudo apt install ros-dev-tools libusb-1.0-0-dev unzip wget
```

### 3.2 EPOS Command Libraryの入手と展開

EPOS Command Libraryは、EPOS2と通信するためにMaxonが提供しているライブラリです。
ROSのパッケージとは別にインストールします。次のコマンドで、Maxon公式サイトからLinux版の
ZIPファイルをダウンロードして展開します。

```bash
mkdir -p ~/opt/epos-command-library
wget -O ~/opt/epos-command-library/EPOS-Linux-Library-En.zip \
  'https://www.maxongroup.co.uk/medias/sys_master/root/9443687202846/EPOS-Linux-Library-En.zip'

unzip ~/opt/epos-command-library/EPOS-Linux-Library-En.zip \
  -d ~/opt/epos-command-library
```

このZIPファイルのリンクは[Maxonのダウンロードページ](https://www.maxongroup.co.uk/maxon/view/product/326343)
に掲載されている「EPOS Linux Library (ARM/Intel 32/64-Bit)」です。

展開後、次のファイルがあることを確認します。

```text
~/opt/epos-command-library/EPOS_Linux_Library/include/Definitions.h
~/opt/epos-command-library/EPOS_Linux_Library/lib/intel/x86_64/libEposCmd.so.6.8.1.0
```

2番目のファイルはIntel/AMD 64-bit PC向けの例です。コンピュータの種類は
`uname -m` で確認できます。`x86_64`以外の場合は、展開した `lib` フォルダから対応する
ライブラリを選んでください（`intel/x86`、`arm/v6`、`arm/v7`、`arm/v8` など）。

### 3.3 USB機器へのアクセス権

USB機器を開けない場合は、Maxonのアーカイブに含まれるUSBアクセス用ルールを登録します。
内容を確認してから、必要な場合だけ実行してください。

```bash
sudo cp ~/opt/epos-command-library/EPOS_Linux_Library/misc/99-ftdi.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

登録後、EPOS2のUSBケーブルを抜き差ししてください。

Maxonのアーカイブには `install.sh` も含まれますが、システム全体へのファイル配置に加え、
`ip` コマンドをパスワードなしで実行できる設定も追加するため、本手順では実行しません。

### 3.4 パッケージのビルド

このリポジトリをワークスペースの `src` 以下に置き、ワークスペースのルートでビルドします。
CMakeは、ホームディレクトリに展開したMaxonライブラリを自動で参照し、CPUの種類に応じた
ライブラリを選びます。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select epos2_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

異なる場所にワークスペースを作成した場合は、`~/ros2_ws` を読み替えてください。

## 4. How to use

### 4.1 ノードの起動

実行時にもライブラリを見つけられるよう、`LD_LIBRARY_PATH` に追加して起動します。
次はIntel/AMD 64-bit PC（`x86_64`）の例です。ARM 64-bit PC（`aarch64`）では
`intel/x86_64` を `arm/v8` に置き換えてください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export EPOS_LIB_DIR="$HOME/opt/epos-command-library/EPOS_Linux_Library/lib/intel/x86_64"
export LD_LIBRARY_PATH="$EPOS_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ros2 launch epos2_driver epos2_driver.launch.py
```

起動時、ノードはEPOS2に接続しますが、モータは無効のままです。
パラメータを変更するには [`launch/epos2_driver.launch.py`](./launch/epos2_driver.launch.py) を編集します。

### 4.2 モータを有効にする

別の端末でROS 2とワークスペースの環境を読み込み、サービスを呼び出します。
モータが動作可能になるため、機構の周囲に人や障害物がないことを確認してください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /enable std_srvs/srv/SetBool "{data: true}"
```

### 4.3 速度を指令する

`/cmd_vel_rpm` に目標速度 [rpm] を送ります。使用するモータの安全な速度範囲を確認し、
小さな値から試してください。

```bash
ros2 topic pub --once /cmd_vel_rpm std_msgs/msg/Int32 "{data: 100}"
```

継続して指令する場合は、`--once` の代わりに `-r 10`（10 Hz）などを指定します。
停止するには `{data: 0}` を送るか、モータを無効化します。

### 4.4 モータを停止・無効化する

```bash
ros2 service call /enable std_srvs/srv/SetBool "{data: false}"
```

## 5. トラブルシューティング

- **`Maxon EPOS Command Library was not found`**  
  `EPOS_COMMAND_INCLUDE_DIR` と `EPOS_COMMAND_LIBRARY` が実際の `Definitions.h` と
  ライブラリファイルを指しているか確認して、もう一度ビルドします。
- **`error while loading shared libraries`**  
  起動前に、使用中のPCに合ったライブラリのフォルダを `LD_LIBRARY_PATH` に追加したか
  確認します。
- **USBデバイスを開けない（`VCS_OpenDevice failed`）**  
  USBケーブルと接続を確認し、必要なら「3.3 USB機器へのアクセス権」を行います。
  USBルールを変更した場合は、ケーブルを抜き差しします。
- **ノードIDや通信設定のエラー（`VCS_SetProtocolStackSettings failed` など）**  
  EPOS2に保存されたノードID、通信速度と、起動設定の値が一致しているか確認します。
- **速度を送ってもモータが動かない**  
  `enable` サービスで `data: true` を呼び出し、`success: true` が返っているか確認します。
  無効のときの指令は無視され、警告ログが出ます。
- **`enable` が `success: false` を返す**  
  `message` のエラーコード（`error 0x........`）を、Maxon EPOS Command Libraryのマニュアルで
  確認します。電源、モータの配線、EPOS2のエラー状態を確認してください。

詳しい関数や通信設定については、[Maxon EPOS Command Libraryのマニュアル（英語）](https://www.maxongroup.ch/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf)
も参照してください。
