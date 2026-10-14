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
