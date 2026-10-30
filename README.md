# robot_port

ROS 2の標準メッセージとメイン基板のUDP通信を中継するパッケージです。
2026NHKロボコンAチームのために作成されました。

## 目次

1. [概要](#1-概要)
2. [ドキュメント](#2-ドキュメント)
3. [コントリビューション](#3-コントリビューション)
4. [ビルド・使い方](#4-ビルド使い方)
5. [システム構成](#5-システム構成)
6. [ライセンス](#6-ライセンス)

## 1. 概要

`udp_node`は速度・状態・機構の各トピックで値を更新し、最新値を100ms周期で送信します。
ROS入力が途絶えても最新値を保持します。独自のROSメッセージ定義はありません。

`NavigationStatus`でROS側の状態を基板へ通知します。明示された自己位置推定またはナビゲーションの失敗だけでFailへ遷移し、Failへの遷移時は走行速度とベルト速度を破棄してゼロを即時送信します。

## 2. ドキュメント

| ドキュメント | 説明 |
| :--- | :--- |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 開発フロー・コミット規約 |
| [コーディング規約](docs/coding-rules.md) | コーディング規約の詳細 |
| [NavigationState状態遷移図](docs/navigation_state.puml) | ナビゲーション状態の遷移とFail復帰条件 |

## 3. コントリビューション

[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

## 4. ビルド・使い方

ROS 2 Humbleを使用します。以前のカスタム型の生成物を混ぜないよう、初回は新しいbuild/installを使います。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/robot-port --ignore-src -y --rosdistro humble
colcon build --packages-select robot_port --build-base build-standard --install-base install-standard
source install-standard/setup.bash
ros2 launch robot_port robot_port.launch.py
```

接続設定は[config/udp.yaml](config/udp.yaml)のパラメータで指定します。通常起動からUDPを送信します。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `local_address` | `192.168.3.1` | 自機のbind先IPv4 |
| `peer_address` | `192.168.3.2` | メイン基板のIPv4 |
| `command_port` | `26574` | 基板要求の受信ポート |
| `operation_port` | `39244` | 動作指令送信・フィードバック受信ポート |
| `operation_freq` | `20` | 動作指令の送信周期[Hz] |
| `receive_timer_freq` | `100` | 受信処理と入力タイムアウト判定の周期[Hz] |
| `localization_timeout_ms` | `1000` | 自己位置推定結果のタイムアウト[ms]。超過時は`false` |
| `navigation_status_timeout_ms` | `1000` | ナビゲーション状態のタイムアウト[ms]。超過時は`Fail` |

別の設定を使う場合は絶対パスを指定します。

```bash
ros2 launch robot_port robot_port.launch.py config:=/absolute/path/udp.yaml
ros2 topic echo /robot/feedback/navigation_status
```

ローカルに`test/`を配置した場合は、同じbuild/installを指定して試験できます。
公開checkoutではビルド・lint・CIの起動確認を実施し、ローカル追加試験とは区別します。

```bash
colcon test --packages-select robot_port --build-base build-standard --install-base install-standard
colcon test-result --test-result-base build-standard --verbose
```

## 5. システム構成

### 入力トピック

全入力はReliable・Volatile・最新1件です。`cmd_vel`はbase_link基準のlinear.x/yとangular.zを使います。

| トピック | 型 | 内容 |
| :--- | :--- | :--- |
| `/robot/command/localization_valid` | `std_msgs/Bool` | 現在の自己位置推定結果が利用可能か |
| `/robot/command/navigation_status` | `std_msgs/UInt8` | ナビゲーションの状態、0〜5 |
| `/robot/command/cmd_vel` | `geometry_msgs/Twist` | 並進速度[m/s]・Yaw角速度[rad/s] |
| `/robot/command/target_id` | `std_msgs/UInt8` | 対象ID、0〜7 |
| `/robot/command/belt_launcher_speed` | `std_msgs/Float32` | ベルト速度[m/s] |
| `/robot/command/bucket_arm_height` | `std_msgs/Float32` | アーム高さ[m] |
| `/robot/command/bucket_arm_width_extract` | `std_msgs/Bool` | アーム展開 |
| `/robot/command/bucket_arm_hold` | `std_msgs/Bool` | アーム保持 |
| `/robot/command/angles_deg` | `std_msgs/Int8MultiArray` | 角度6要素、各−20〜20度 |
| `/robot/command/can_move` | `std_msgs/Bool` | LED用の移動可否。速度抑制には使用しない |

角度順は移動バケツ、固定バケツ1、2、3、旗、机です。角度はそのまま符号付き度数として送ります。
不正な要素数・範囲・NaN・無限値は入力全体を拒否し、最後の正常値を保持します。不正入力自体をFail原因にはしません。

### 出力トピック

フィードバックは受信したパケットごとに個別配信します。`bucket_arm_hight`は既存の綴りを維持しています。

| トピック | 型 | 内容 |
| :--- | :--- | :--- |
| `/robot/feedback/sequence` | `std_msgs/UInt8` | 基板フィードバックのsequence |
| `/robot/feedback/emergency_stop` | `std_msgs/Bool` | 非常停止状態 |
| `/robot/feedback/over_current` | `std_msgs/Bool` | 過電流状態 |
| `/robot/feedback/drive_battery_voltage` | `std_msgs/Float32` | 電圧[V] |
| `/robot/feedback/drive_current` | `std_msgs/Float32` | 電流[A] |
| `/robot/feedback/belt_launcher_target` | `std_msgs/Float32` | 射出時リリース目標値[m/s]（1射出1値） |
| `/robot/feedback/belt_launcher/command` | `std_msgs/Float32` | 連続指令速度[m/s]（VESCへの連続指令値） |
| `/robot/feedback/belt_launcher_release` | `std_msgs/Float32` | 直前の射出初速[m/s] |
| `/robot/feedback/wheel_angular_velocity` | `std_msgs/Float32MultiArray` | 前・左・右の車輪角速度[rad/s] |
| `/robot/feedback/belt_launcher_velocity` | `std_msgs/Float32` | ベルト速度[m/s] |
| `/robot/feedback/loading_belt_angle` | `std_msgs/Float32` | 装填機構の角度[rad] |
| `/robot/feedback/bucket_arm_hight` | `std_msgs/Float32` | アーム高さ[m] |
| `/robot/feedback/navigation_status` | `std_msgs/UInt8` | UDPへ送信する実効状態。基板の受理確認ではない |
| `/robot/request` | `std_msgs/UInt8MultiArray` | 基板要求7項目 |

要求配列の順序は`[sequence, shutdown, localization, logging, target_id, navigation_command, cloth_collect]`です。
bool項目は0/1。対象IDは0=開始位置、1〜3=固定バケツ1〜3、4/5=机1/2、6=旗、7=移動バケツです。
`navigation_command`はSleep=0、Stanby=1、Move=2です。要求の中継だけを行い、OS終了やゴール管理は行いません。

### 状態遷移

状態値は`Stanby=0, Ready=1, Moving=2, Tracking=3, Goal=4, Fail=5`です。
詳細な遷移は[NavigationState状態遷移図](docs/navigation_state.puml)を参照してください。

- 起動時はStanby・速度ゼロ。自己位置とナビ状態が未受信なら走行指令を出しません。
- 両入力が正常ならナビ状態に従います。走行速度はMoving/Tracking時だけ受け付けます。
- `localization_valid=false`またはナビ状態FailでFailを保持します。位置設定は保持し、走行/ベルト速度は破棄してゼロを即時送信します。Fail中の速度入力は保存しません。
- 両入力が正常へ戻った後、実際に受信したSleep/Stanbyから新しいMoveへ変化したときだけFailを解除します。同一Moveの再送、sequenceや対象IDの変更では解除しません。
- 解除直後はReady・速度ゼロです。解除後の新しいナビ状態を受け取り、その後の新しい速度入力を待ちます。

- 自己位置推定結果またはナビゲーション状態の受信が対応するタイムアウトを超えると、それぞれ`false`または`Fail`へ遷移します。
- 未受信・通知途絶（タイムアウト前）・ROS時計変化・LED用移動不可・正常終了・取消はFail原因にしません。
通常時の入力停止では最後の正常値を送り続けます。基板要求の取消はgoal managerへ中継し、ナビ側が停止状態を通知します。
自己位置/ナビの状態を標準トピックへ通知する側の接続は必要です。従来のカスタム型を使うmanagerとは互換性がありません。

### UDP通信

| ポート | 方向・用途 | 長さ・ヘッダー |
| :--- | :--- | :--- |
| `26574` | 基板→ROSの要求 | 8 byte・`0x38` |
| `39244` | ROS→基板の動作指令 | 32 byte・`0xAB` |
| `39244` | 基板→ROSの計測値 | 44 byte・`0x55` |

既存のlittle endian配置を維持し、Operationのbyte位置3へNavigationStatus、予約byte位置1へLED用can_move(0/1)を格納します。
LED表示には基板側の対応が必要です。送信100ms、受信処理10ms周期で、不正な送信元・長さ・ヘッダー・bool・enum・非有限値を拒否します。
ソケットは非ブロッキングで所有し、bind失敗などの起動エラーは終了コード1、通信エラーはROSログで通知します。
UDP送信成功と基板の受理・物理停止・実機受入は別に確認してください。

## 6. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
