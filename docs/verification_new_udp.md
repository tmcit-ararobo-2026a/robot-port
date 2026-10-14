# 新UDP通信基盤の検証（T02a〜T04・T18a）

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
基準: robot-port `ce821b9ca75e7e6aebcde92ccec710e3f5e05343` → `feat/10-stm32-new-udp`。
検証対象: T00文書化、T01依存、T02a通信型・codec、T03送受信、T04watchdog、T18a試験・整形・CI。
STM32参照: `e66ca03305666d2c53ba97a3e5c855201e2f7e53`。変更せず定義とバイト一致を照合。

## コマンドと結果

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/stm32-ros2/log-t18a-repair build --base-paths robot-port   --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install   --packages-select robot_port   --cmake-args -DCLANG_FORMAT_BIN=/tmp/stm32-tools/clang_format/data/bin/clang-format
source /tmp/stm32-ros2/install/setup.bash
colcon --log-base /tmp/stm32-ros2/log-t18a-final test   --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install   --packages-select robot_port
colcon test-result --test-result-base /tmp/stm32-ros2/build/robot_port/test_results --verbose
```

- ビルド: 成功／終了コード0。ROS 2 Humble、GCC11、Python3.10、C++20。`ros2 interface show robot_port/msg/RobotOperation` も成功。
- 基板構造体との8/32/44byte・offset・operation golden packetのmemcmp: 成功／終了コード0。`/tmp/stm32_wire_layout_check.cpp` をGCC11で基板参照includeとcodecにリンクし実行。
- C++ codec5件・watchdog5件・UDP4件、pytest結合1件、整形・Python・CMake・XML検査: 成功。colcon結果35件、エラー0・失敗0・skip15。
- skip15件はament_cppcheckの既知cppcheck2.7性能問題による自動skip。静的解析の成功と扱わない。
- 通信先は127.0.0.0/8限定。ROS_LOCALHOST_ONLY=1・試験専用ROS_DOMAIN_ID。UDP元長・送信元・片方向断・Operation入力断・重複・旧世代非再開を検証。ソケット/DDSはsandbox外で実施。

## 失敗と修正

- sandbox内UDP試験: 失敗／ctest終了コード8。socket作成のEPERM。loopback限定でsandbox外へ切替後に成功。
- 初回全試験: 失敗／終了コード8。pytest環境変数APPEND_ENVの先頭コロン、manifestのmember_of_group配置、ROS標準uncrustifyとの規約不一致。
- ENVへの変更、manifest順修正、既存.clang-formatに基づく新通信コードの整形検査へ変更。廃止したuncrustifyの失敗結果は `/tmp/stm32-ros2/obsolete-test-results/uncrustify-initial-failure.xunit.xml` に保存し、現行結果と分離。
- 結合試験再試行: 失敗／終了コード8。明示contextに対して暗黙global executorを利用。専用executorへ修正。
- 次の再試行: 失敗／終了コード8。QoS深さ1の入力3件連続送信で上書きされ、拒否数の期待値に達しなかった。1件ずつ拒否を観測する試験へ修正後、結合試験成功／終了コード0。
- 初回colcon test-resultは古いuncrustify結果残留で終了コード1。上記へ保存後、現行結果は終了コード0。

## CI初回失敗と修正

PR #11の初回GitHub CI: 失敗。実機能の単体・結合試験は成功し、clang-format14とローカル18の2箇所の版差、ソース直下のbuild/install生成物をflake8・CMake lintが走査したため失敗した。
ビルド・install・logをrunner.temp配下へ移動し、ローカル整形もUbuntu22.04標準のclang-format14へ合わせた。
修正後のHumbleビルドと全CTest10件: 成功／終了コード0。CIの再実行結果は別途確認する。
修正時のコマンドは上記と同様、整形ツールの指定は `/tmp/stm32-tools14/clang_format/data/bin/clang-format`。
ログ: `/tmp/stm32-ros2/robot-ci-repair-build.log`、`robot-ci-repair-tests.log`。初回CI失敗ログも保持。

## 未実施・残条件

実機、HIL、基板受理、物理停止・負荷試験: 未実施。U1〜U15の外部条件・校正は未確定。
送信成功は基板受理ACK・全機構停止ではない。既定実機送信は無効。
未確定契約は [new_mainboard_contract.md](new_mainboard_contract.md) を参照。

## インストール後の通信launch検証と修正

2026-10-14（Asia/Tokyo）、5714916からのCMake差分。managerの通信試験launchで初回・再試行は失敗（ctest終了コード8）。
udp_nodeの共有依存libudp_channel.soがinstallされていなかった。udp_channelのinstall/exportを追加。
上記Humble colconビルド（robot_portとauto_mode_manager）、インストールsetupをsourceしたmanagerのCTest2件は成功／終了コード0。
ログ: `/tmp/stm32-ros2/runtime-fix-build.log`、`manager-t17a-launch-retry.log`（失敗）、`manager-t17a-final-tests.log`（成功）。
従来のbuildディレクトリ上のcodec/UDP試験に加えて、インストール後のros2 launch送受信を確認した。

## manager契約v1のbridgeメタデータと送信記録

2026-10-14 Asia/Tokyo、feat/12-bridge-contract-v1。3364e6fからの型・通信ノード・試験差分。
bridge_session/update_sequence/test_mode/loopback_onlyと、最後に成功したUDP送信のproducer/session/generation/statusを追加。wireは8/32/44 byteを維持。
Humble、`/home/lambda/ros2_ws/src`でcolcon build --packages-select robot_port --build-base /tmp/manager-v1/build --install-base /tmp/manager-v1/install。
ビルド成功／終了コード0。install setup後ctest --test-dir /tmp/manager-v1/build/robot_port --output-on-failure、10件成功／終了コード0。
実DDSでメタデータ・進行・ノード再起動のsession変化と実Fail送信statusを確認。board ACK／物理停止は未実施・unknown。
ログ `/tmp/manager-v1/runtime-build.log`、bridge-final-tests.log。clang-format14。
