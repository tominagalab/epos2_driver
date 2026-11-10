# epos2_driver

Maxon EPOS2をUSBで操作するROS 2 Humble向けC++パッケージです。
MaxonのEPOS Command Libraryを使い、モータの有効化・停止と速度指令を行います。

> **重要：安全に関する注意**
>
> 初回は、モータに接続した機構を動かしても人や物に危険がない状態にし、
> 可能であれば負荷を外して試してください。モータを動かすときは、すぐに
> 電源を切れる人が近くにいてください。このパッケージは非常停止装置ではなく、
> 安全を保証するものでもありません。

## 1. 必要なもの

- Ubuntu 22.04（Jammy）
- ROS 2 Humble
- Maxon EPOS2と、PCに接続するためのUSBケーブル
- MaxonのEPOS Command Library（Linux版）
- EPOS2に接続するモータ、電源、エンコーダなどの機器

EPOS2本体の設定（ノードIDや通信設定など）が必要です。モータとEPOS2の
配線、電源、モータの種類に応じた設定は、本体のマニュアルに従ってください。

## 2. ROS 2 Humbleの準備

すでにROS 2 Humbleをインストール済みなら、この節は読み飛ばせます。
HumbleのDebianパッケージはUbuntu 22.04向けです。まだインストールして
いない場合は、ROS公式手順の「Ubuntu (deb packages)」に従ってROS 2 Humbleを
インストールしてください。

- [ROS 2 HumbleのUbuntuインストール手順](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

C++パッケージのビルドに必要なツールをインストールします。

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools libusb-1.0-0-dev unzip wget
```

以降は、新しく開いた端末でも作業を始めるたびにROS 2の環境を読み込みます。

```bash
source /opt/ros/humble/setup.bash
```

## 3. EPOS Command Libraryの入手と展開

EPOS Command Libraryは、EPOS2と通信するためにMaxonが提供しているライブラリです。
ROSのパッケージとは別にインストールします。

以下のコマンドで、ホームディレクトリ内に専用フォルダを作り、Maxon公式サイトから
Linux版ライブラリのZIPファイルをそのフォルダにダウンロードして展開します。
ZIPファイルには、ライブラリ、ヘッダファイル、サンプル、USBアクセス用の
ルールファイルなどが含まれます。

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
次のコマンドで確認できます。

```bash
uname -m
```

`x86_64`以外の場合は、展開した `lib` フォルダから対応するライブラリを選んで
ください。Maxonのアーカイブには、たとえば `intel/x86`、`arm/v6`、`arm/v7`、
`arm/v8` 用のライブラリが含まれています。

### USB機器へのアクセス権

USB機器を開けないエラーが出る場合、ログイン中のユーザーにEPOS2へのアクセス権が
ない可能性があります。展開したアーカイブの `misc/99-ftdi.rules` は、Maxonの
Linuxライブラリに含まれるUSBアクセス用ルールです。必要な場合だけ、次のように
システムへ登録します。

```bash
sudo cp ~/opt/epos-command-library/EPOS_Linux_Library/misc/99-ftdi.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

登録後、EPOS2のUSBケーブルを抜き差ししてください。このルールは対象USB機器への
アクセスを許可する設定です。内容を確認してから登録してください。

Maxonのアーカイブには `install.sh` も含まれますが、システム全体へのファイル配置、
USBルール登録、ライブラリのシンボリックリンク作成に加え、`ip` コマンドをパスワード
なしで実行できる設定も追加するスクリプトです。この設定は本パッケージの動作には
必要ありません。本手順ではシステムへの変更を限定するため、スクリプトは実行せず、
展開したライブラリを直接指定し、必要な場合だけUSBルールを個別に登録します。

## 4. ROS 2パッケージのビルド

ROS 2では、パッケージをワークスペースの `src` フォルダに置いてビルドします。
このパッケージのフォルダ名が `epos2_driver` であることを確認し、次の場所に置いて
ください。すでにこの場所にある場合は、そのままで構いません。

```text
~/ros2_ws/src/epos2_driver
```

以降のコマンドは、このワークスペースを使う例です。異なる場所に作成した場合は、
`~/ros2_ws` を自分のワークスペースの場所に読み替えてください。

このパッケージのCMake設定は、ホームディレクトリに展開したMaxonライブラリを自動で
参照し、CPUの種類に応じたライブラリを選びます。また、Maxonライブラリが必要とする
共有ライブラリの名前リンクも自動で作成します。ライブラリの場所をビルドコマンドに
指定する必要はありません。ワークスペースのルートで、次のコマンドを実行します。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select epos2_driver --symlink-install
```

ビルドが成功したら、同じ端末でワークスペースの環境を読み込みます。

```bash
source ~/ros2_ws/install/setup.bash
```

## 5. EPOS2の起動

ライブラリを実行時にも見つけられるように、ライブラリのフォルダを環境変数に追加し、
起動します。次はIntel/AMD 64-bit PC（`uname -m`が`x86_64`）の場合です。
ARM 64-bit PC（`aarch64`）では、パスの`intel/x86_64`を`arm/v8`に置き換えてください。

```bash
export EPOS_LIB_DIR="$HOME/opt/epos-command-library/EPOS_Linux_Library/lib/intel/x86_64"
export LD_LIBRARY_PATH="$EPOS_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ros2 launch epos2_driver epos2_driver.launch.py
```

起動時、ノードはEPOS2に接続しますが、モータは無効のままです。接続に失敗した場合は、
USBケーブル、USBアクセス権、EPOS2のノードID、ライブラリの場所を確認してください。

## 6. モータの有効化と速度指令

別の端末を開き、ROS 2とワークスペースの環境を読み込みます。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### モータを有効にする

次のサービス呼び出しでモータを有効にします。`true` を送るとモータが動作可能に
なるため、機構の周囲に人や障害物がないことを確認してください。

```bash
ros2 service call /enable std_srvs/srv/SetBool "{data: true}"
```

### 速度を指令する

`/cmd_vel_rpm` に `std_msgs/msg/Int32` 型で目標速度を送ります。値の単位はrpmです。
次の例は100 rpmを一度だけ送ります。使用するモータの安全な速度範囲を確認し、
小さな値から試してください。

```bash
ros2 topic pub --once /cmd_vel_rpm std_msgs/msg/Int32 "{data: 100}"
```

### モータを停止・無効化する

```bash
ros2 service call /enable std_srvs/srv/SetBool "{data: false}"
```

## 7. 主な設定値

起動設定は [`launch/epos2_driver.launch.py`](./launch/epos2_driver.launch.py) にあります。
EPOS2の設定と実際のUSB接続に合わせて値を確認してください。

| パラメータ | 初期値 | 説明 |
|---|---:|---|
| `device_name` | `EPOS2` | EPOS Command Libraryに渡す機器名 |
| `protocol_stack_name` | `MAXON SERIAL V2` | 通信プロトコルスタック名 |
| `interface_name` | `USB` | 接続方式 |
| `port_name` | `USB0` | ライブラリに渡すUSBポート名 |
| `node_id` | `1` | EPOS2に設定されたノードID |
| `baudrate` | `1000000` | 通信速度 |
| `timeout_ms` | `500` | 通信タイムアウト（ミリ秒） |
| `profile_acceleration` | `1000` | 速度制御の加速度設定 |
| `profile_deceleration` | `1000` | 速度制御の減速度設定 |

これらの初期値が、使用するEPOS2やモータに適しているとは限りません。値の意味、
単位、許容範囲を、Maxonの機器マニュアルおよび設定内容で確認してください。

## 8. うまく動かないとき

- **`Maxon EPOS Command Library was not found`**  
  `EPOS_COMMAND_INCLUDE_DIR` と `EPOS_COMMAND_LIBRARY` が実際の `Definitions.h` と
  ライブラリファイルを指しているか確認して、もう一度ビルドします。
- **`error while loading shared libraries`**  
  起動前に、使用中のPCに合ったライブラリのフォルダを `LD_LIBRARY_PATH` に追加したか
  確認します。
- **USBデバイスを開けない**  
  USBケーブルと接続を確認し、必要なら「USB機器へのアクセス権」の手順を行います。
  USBルールを変更した場合は、ケーブルを抜き差しします。
- **ノードIDや通信設定のエラー**  
  EPOS2に保存されたノードID、通信速度、モードと、起動設定の値が一致しているか
  確認します。

詳しい関数や通信設定については、[Maxon EPOS Command Libraryのマニュアル（英語）](https://www.maxongroup.ch/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf)
も参照してください。
