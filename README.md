# epos2_driver

## 1. 概要

Maxon EPOS2をUSBおよびUSB-to-CANゲートウェイ経由で操作するROS 2 Humble向けC++パッケージです。
1つのノードで、USB接続の1台からUSB-to-CAN経由の複数台まで操作できます。
MaxonのEPOS Command Libraryを使い、モータの有効化・無効化と速度または位置の指令を行います。

> **重要：安全に関する注意**
>
> 初回は、モータに接続した機構を動かしても人や物に危険がない状態にし、
> 可能であれば負荷を外して試してください。モータを動かすときは、すぐに
> 電源を切れる人が近くにいてください。このパッケージは非常停止装置ではなく、
> 安全を保証するものでもありません。

### ノード構成

| 項目 | 内容 |
|---|---|
| 実行ファイル・ROSノード名 | `epos_driver_node` |
| 起動ファイル | [`launch/epos2_driver.launch.py`](./launch/epos2_driver.launch.py) |
| 設定ファイル | [`config/config.yaml`](./config/config.yaml) |
| 接続 | PC — USB — EPOS2ノード4 — CAN — ノード1、2、3、5 |
| 制御モード | モータごとに `velocity` または `position` |

1つのROSノードが共有USB接続とCANサブデバイスを管理し、各モータに個別の名前空間を設けます。
設定例では全モータが `velocity` です。名前空間は設定ファイルの各 `namespace` を編集して
変更できます。以下の `<ns>` は各モータの設定値（例：`/epos2/node_1`）です。

| 種別 | 名前 | 型 | 用途 |
|---|---|---|---|
| Subscribe | `<ns>/cmd_vel_rpm` | `std_msgs/msg/Int32` | 速度指令 [rpm]。`velocity` モードのみ |
| Subscribe | `<ns>/cmd_position_counts` | `std_msgs/msg/Int32` | 絶対位置指令 [エンコーダカウント]。`position` モードのみ |
| Service | `<ns>/enable` | `std_srvs/srv/SetBool` | `true` で有効化、`false` で停止・無効化 |

購読のQoSはReliable、Volatile、深さ10です。`enable` は成功時に `success: true`、
失敗時に `success: false` とエラー内容を返します。起動時に各モータを無効化し、
有効なモータのみに指令を受け付けます。終了時は有効なモータの停止・無効化を試みます。
このノードはモータ状態や指令結果を配信しません。指令処理のエラーはログに出ます。

位置指令はEPOS2の現在の基準座標に対する絶対位置で、新しい指令を直ちに適用します。
この基準は永続的な絶対座標とは限りません。電源投入時に希望する位置がゼロになること、
エンコーダの分解能・方向・基準の再現性を実機設定で確認してください。原点復帰や角度単位への
変換は行いません。

#### 設定項目

| 項目 | 説明 |
|---|---|
| `device_name`, `protocol_stack_name`, `interface_name`, `port_name` | USB接続のEPOS Command Library設定 |
| `usb_baudrate`, `timeout_ms` | USBプロトコルの通信速度とタイムアウト [ms] |
| `can_baudrate` | ゲートウェイのCAN通信速度の期待値 [bit/s]。例は `1000000` |
| `gateway_node_id` | USBで直接接続されたEPOS2のノードID。例は `4` |
| `motor_names` | 制御するモータ設定名のリスト（1台以上） |
| `motors.<名前>.node_id` | EPOS2のノードID（1〜127、重複不可） |
| `motors.<名前>.namespace` | モータの絶対名前空間（`/` で開始、重複不可） |
| `motors.<名前>.mode` | `velocity` または `position` |
| `motors.<名前>.velocity.acceleration`, `deceleration` | 速度制御時の加速度・減速度 |
| `motors.<名前>.velocity.max_velocity` | 速度制御時の速度上限 |
| `motors.<名前>.position.acceleration`, `deceleration` | 位置制御時の加速度・減速度 |
| `motors.<名前>.position.profile_velocity` | 位置制御時のプロファイル速度 |
| `motors.<名前>.position.max_velocity` | 位置制御時の速度上限 |

設定例の加減速度は `10000`、速度上限は `8000`、位置プロファイル速度は `6000` です。
想定単位は速度がrpm、加減速度がrpm/sですが、EPOS2の速度単位設定とモータの許容値を必ず
確認してください。速度上限は選択されたモードの値をEPOS2に設定します。速度指令が上限を
超えた場合は送信せず、ログにエラーを出します。位置プロファイル速度は位置速度上限以下に
設定してください。パラメータは起動時に読み込み、実行中の変更は反映しません。

制御台数は `motor_names` に列挙した設定の数で決まります。選ばれていない `motors` の
設定は使用しません。たとえばUSB接続のノード4だけを制御する場合は、次のようにします。

```yaml
gateway_node_id: 4
motor_names: [node_4]
```

USBで接続したEPOS2のIDが1の場合は、`gateway_node_id: 1` と `motor_names: [node_1]`
に変更してください。USB接続のモータのみを選んだ場合、CANサブデバイスは開きません。
CAN側だけを制御する場合は `motor_names: [node_1, node_2]` のように設定できますが、
`gateway_node_id` は実際にUSB接続されたEPOS2のIDのままにします。ゲートウェイのモータを
リストに含めない場合、そのモータには指令を出しません。

設定例のCAN速度 `1 Mbit/s` はEPOS2の最大値です。全ノードのCAN速度と配線長、
終端抵抗などが適切であることを事前に確認してください。起動時にゲートウェイの設定速度を
読み取り、期待値と異なる場合はCAN側の操作を受け付けません（CAN側を選択した場合のみ）。
CAN速度を自動変更することは
ありません。USBゲートウェイの再接続やCAN速度の修正後はノードを再起動してください。

CAN側の1台が応答しない場合、そのモータのエラーをログとサービス応答に出し、他のモータの
操作を継続します。起動時の個別モータの初期化失敗は、そのモータのサービス呼び出し時に
再試行します。ただしUSBゲートウェイやCANバス自体の障害は複数台に影響します。
共有通信は順次処理され、通信タイムアウト中は他のモータの応答も遅れることがあります。

**安全上の制限：** 指令の受信タイムアウト、位置のソフトリミット、非常停止機能はありません。
通信途絶やプロセス異常終了時の停止も保証しません。上位システムおよび独立した安全回路で
対策してください。速度上限も安全機能の代わりにはなりません。

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

使用する設定ファイルの `motor_names`、接続設定、各モータの
名前空間・モード・プロファイルを実機に合わせて設定し、ビルドし直してください。
従来の単体USB用トピック `/enable` と `/cmd_vel_rpm` は使用せず、
設定した各モータの名前空間を付けて操作します。
統合後のノードは、単体USBの場合も設定された速度上限をEPOS2に設定し、
上限を超える速度指令を拒否します。以前の設定ファイルを使用する場合は、
ROSパラメータの先頭キーも `epos_driver_node` に変更してください。
既存のインストール先には古い実行ファイルや起動ファイルが残る場合がありますが、
この起動ファイルからは新しい `epos_driver_node` のみが起動します。

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
現在はノード4（USB）とノード5（CAN）用の
[`config/config_node4_node5.yaml`](./config/config_node4_node5.yaml) を読み込みます。
このファイルは5台用設定と同じ速度上限・加減速度を使用します。

設定を切り替えるには、[`launch/epos2_driver.launch.py`](./launch/epos2_driver.launch.py)
の `DEFAULT_CONFIG_FILE` を編集し、ビルドし直してください。例えば5台用にする場合は
次のように変更します。

```python
DEFAULT_CONFIG_FILE = 'config.yaml'
```

切り替え後も同じ `ros2 launch epos2_driver epos2_driver.launch.py` で起動できます。
一時的に別の設定ファイルを使用する場合は、従来どおり絶対パスを指定することもできます。

```bash
ros2 launch epos2_driver epos2_driver.launch.py config:=/absolute/path/to/config.yaml
```

ノード4・5の構成では、それぞれ `/epos2/node_4`・`/epos2/node_5` の名前空間を使用します。
以下の操作例はノード1の場合なので、使用するモータの名前空間に読み替えてください。

### 4.2 モータを有効にする

別の端末でROS 2とワークスペースの環境を読み込み、サービスを呼び出します。
モータが動作可能になるため、機構の周囲に人や障害物がないことを確認してください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /epos2/node_1/enable std_srvs/srv/SetBool "{data: true}"
```

### 4.3 速度を指令する

例としてノード1の `/epos2/node_1/cmd_vel_rpm` に目標速度 [rpm] を送ります。
事前にサービス応答の `success: true` を確認してください。モータの安全な速度範囲を確認し、
小さな値から試してください。

```bash
ros2 topic pub --once /epos2/node_1/cmd_vel_rpm std_msgs/msg/Int32 "{data: 100}"
```

継続して指令する場合は、`--once` の代わりに `-r 10`（10 Hz）などを指定します。
停止するには `{data: 0}` を送るか、モータを無効化します。

### 4.4 モータを停止・無効化する

```bash
ros2 service call /epos2/node_1/enable std_srvs/srv/SetBool "{data: false}"
```

### 4.5 位置を指令する

例えばノード3の `mode` を `position` に変更した場合は、同じサービスで有効化し、
位置をエンコーダカウント単位で指定します。実際の可動範囲と基準を確認してから送信してください。

```bash
ros2 service call /epos2/node_3/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /epos2/node_3/cmd_position_counts std_msgs/msg/Int32 "{data: 100}"
ros2 service call /epos2/node_3/enable std_srvs/srv/SetBool "{data: false}"
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
- **`CAN baudrate mismatch`**
  全EPOS2とノード4のゲートウェイのCAN速度を確認し、`can_baudrate` と一致させてから
  ノードを再起動します。速度は本ノードから変更されません。

詳しい関数や通信設定については、[Maxon EPOS Command Libraryのマニュアル（英語）](https://www.maxongroup.ch/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf)
も参照してください。
