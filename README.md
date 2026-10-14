# robot_port

ROS 2とSTM32の間で、ロボットの操作要求・動作指令・フィードバックをUDPで送受信するパッケージです。
2026NHKロボコンAチームのために作成されました。

## 目次

1. [概要](#1-概要)
2. [ドキュメント](#2-ドキュメント)
3. [コントリビューション](#3-コントリビューション)
4. [ビルド・使い方](#4-ビルド使い方)
5. [システム構成](#5-システム構成)
6. [ライセンス](#6-ライセンス)

## 1. 概要

`udp_node`は操作要求とフィードバックをROSトピックへ配信し、`RobotOperation`の動作指令をUDPパケットへ変換します。通信状態、指令の鮮度、送受信エラーは`BridgeHealth`で確認できます。

## 2. ドキュメント

| ドキュメント | 説明 |
| :--- | :--- |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 開発フロー・コミット規約 |
| [コーディング規約](docs/coding-rules.md) | コーディング規約の詳細 |
| [通信仕様](docs/new_mainboard_contract.md) | パケット構成、接続条件、実機で確認が必要な項目 |
| [通信試験](docs/verification_new_udp.md) | 単体試験・UDP結合試験の結果 |

## 3. コントリビューション

[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

## 4. ビルド・使い方

ROS 2 Humbleを使用します。ワークスペースで依存関係をインストールしてからビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/robot-port --ignore-src -y --rosdistro humble
colcon build --symlink-install --packages-select robot_port
source install/setup.bash
ros2 launch robot_port robot_port.launch.py
```

接続設定は[config/udp.yaml](config/udp.yaml)にあります。別の設定を使う場合はファイルの絶対パスを指定します。

```bash
ros2 launch robot_port robot_port.launch.py config:=/absolute/path/udp.yaml
```

既定では送信が無効です。実機へ送信するには`enable_transmission: true`と`physical_contract_confirmed: true`が必要です。後者は接続条件の確認を記録する設定であり、機構の停止や校正を検証する機能ではありません。試験用の`test_mode: true`では、送信元・宛先ともIPv4 loopbackに限ります。

型のみをビルドする場合は`--cmake-args -DBUILD_RUNTIME_NODES=OFF`を指定します。通信試験は次のコマンドで実行します。

```bash
colcon test --packages-select robot_port
colcon test-result --verbose
```

## 5. システム構成

### トピック

| 方向 | トピック | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 入力 | `/robot/operation` | `robot_port/msg/RobotOperation` | 動作指令の全スナップショット |
| 出力 | `/robot/command` | `robot_port/msg/RobotCommand` | 操作要求 |
| 出力 | `/robot/feedback` | `robot_port/msg/RobotFeedback` | 基板のフィードバック |
| 出力 | `/robot/communication_status` | `robot_port/msg/BridgeHealth` | 鮮度、エラー、最後の送信結果 |

### UDP通信

| ポート | 用途 | パケット長 |
| :--- | :--- | :--- |
| `26574` | 操作要求の受信 | 8 byte |
| `39244` | 動作指令の送信／フィードバックの受信 | 32 byte／44 byte |

パケットは明示的なlittle endianで変換します。受信元IP・ポート、完全長、header、enum、bool、浮動小数点の有限値を検査し、不正なスナップショットは全体を拒否します。動作指令の予約byteは0です。角度配列はmoving、bucket1、bucket2、bucket3、flag、deskの順で、int8の範囲は[-20, 20]です。ROS側のsession・世代などのメタデータはUDPパケットへ含めません。

ソケットは非ブロッキングで使用し、bindに失敗した場合は終了コード1で終了します。

### 指令の鮮度監視

Operationの期限は既定で100msです。生成時刻と単調時計を使い、古い指令・重複更新・世代の逆行・不正値を拒否します。ROS時計が停止しても期限は失効し、時計の巻き戻りや大きな進みでも送信許可を失効させます。

送信有効時の起動・期限切れでは、自動速度ゼロのFailを送ります。動作開始には全ゼロのStandbyまたはFailによる中立状態の確認が必要で、期限切れ後は新しい世代を要求します。別sessionによる実行中指令の横取りや、引退済みsessionの指令を受け付けません。

`BridgeHealth`は最後のUDP送信成功と、その指令のsession・世代・状態を公開します。送信成功は基板での受理や物理停止の確認とは別です。

## 6. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
