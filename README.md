# robot_port

ROS 2 HumbleとSTM32間の通信パッケージ。新UDP対応は `feat/10-stm32-new-udp` で段階的に実装中。
通信契約・未確定仕様・実機の保留条件は [new_mainboard_contract.md](docs/new_mainboard_contract.md) を参照。

型生成と実行ノードのビルドは `-DBUILD_RUNTIME_NODES=OFF` で分離可能。

## 新通信型・codec（T02a）

`RobotCommand`、`RobotOperation`、`RobotFeedback`、`BridgeHealth`を所有する。
`wire_codec`は完全長・header・enum・bool・有限float32・[-20,20]のint8角度を検査し、
明示的little endianの32byteを生成する。予約byteは0。ROSメタデータはwireに含まれない。
角度配列はmoving, bucket1, bucket2, bucket3, flag, deskの順。
無効値をクリップせず、スナップショット全体を拒否する。物理範囲・角度無効値は未確定。

## 新UDPノード（T03）

旧44byte指令と部分指令Topicは廃止。入力は `/robot/operation` の全スナップショットのみ。
出力は `/robot/command`、`/robot/feedback`、`/robot/communication_status`。
旧feedback分解Topicの代わりに型付き全フィールドを配信する。
26574はcommand受信専用、39244はoperation送信・feedback受信の共用ソケット。
受信元IP・ポート・元の完全長・切り詰め・codecをすべて検査し、一括更新する。
ソケットは非ブロッキング・RAII・排他的bindで、起動時のbind失敗は終了コード1。

`ros2 launch robot_port robot_port.launch.py config:=/absolute/path/udp.yaml`。
既定設定は送信無効。実機送信は `enable_transmission: true` と
`physical_contract_confirmed: true` を必要とする。これは確認の記録用であり物理停止の証拠ではない。
fixtureは `test_mode: true` に加えlocal/peerともIPv4 loopbackが必須。

## Operation watchdog（T04）

起動時・期限切れは自動速度ゼロのFailを送信。単調時計で生成時点から100msを監視する。
生成stampの過去・未来、重複stamp・更新番号、世代逆行、不正値を拒否し、古い再配信で期限を延長しない。
ROS時計の停止でも単調期限は失効し、巻戻り・大きい進みでも許可を失効する。
producer_sessionは再起動で変更し、最初に全ゼロStandby/Failを送る。
期限切れ後はneutral handshakeの後に新しい世代が必要で、同じ世代の走行は再開しない。
別sessionによるactive指令の横取りや引退済みsessionの再受信を拒否する。

`BridgeHealth`はcommand/feedback/Operationそれぞれの鮮度、拒否数、受信・送信エラー、
最後のUDP送信成功を区別する。Fail送信は基板受理・手動運動・物理停止を保証しない。

## 通信結合・故障注入試験（T18a）

`colcon test --packages-select robot_port` と `colcon test-result --verbose`。
loopback専用pytestで実DDS・実UDPを使用し、packet長・送信元・空packet・旧形式・NaN、
片方向受信断、publisher停止、期限切れ後の旧世代、重複再配信、試験宛先制限、bind競合を検証する。
C++単体試験で生成時刻・時計変化・session交代と実際のUDP送信失敗も確認する。
整形検査は既存 `.clang-format` を使用し、今回の通信コードを対象とする。
旧CANファイルの整形を本機能に混在させない。CIもビルド後にテストを実行する。

## 従来の環境資料

# base-ros2

ROS2の開発環境がまとめられたdockerfileです。
2026NHKロボコンAチームのために作成されました。

## 目次

1. [概要](#1-概要)
2. [ドキュメント](#2-ドキュメント)
3. [コントリビューション](#3-コントリビューション)
4. [ビルド・使い方](#4-ビルド使い方)
5. [システム構成](#5-システム構成)
6. [ライセンス](#6-ライセンス)

## 1. 概要
・ros2のパッケージ関連が一つにまとめられたdockerfile

## 2. ドキュメント

| ドキュメント | 説明 |
| :-: | :-: |
| [CONTRIBUTING.md](./CONTRIBUTING.md) | 開発フロー・コミット規約・コーディング規約 |
| [docs/coding-rules.md](./docs/coding-rules.md) | コーディング規約の詳細 |
| [docs/uml/](./docs/uml/) | UML図 |

## 3. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

## 4. ビルド・使い方

<!-- ビルドのやり方 -->
<!-- 使い方 -->

## 5. システム構成

<!-- ROS2ノード構成・STM32との通信方式・ハードウェア構成 等 -->
<!-- 依存クラスが3つ以上ある場合は docs/uml/ にUML図を作成し、ここにリンクする -->

## 6. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されています。
