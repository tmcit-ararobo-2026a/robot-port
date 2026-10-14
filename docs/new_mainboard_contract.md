# STM32新UDP通信契約・外部条件（T00）

更新日: 2026-10-14（Asia/Tokyo）。確認基準: gn10-mainboard `feat/new-teleop-api` / `e66ca03305666d2c53ba97a3e5c855201e2f7e53`（同日リモート先頭を照合）。

## 確認済みwire契約

| 種類 | 方向 | ポート | header | 完全長 |
| --- | --- | --- | --- | --- |
| command | 基板→Jetson | 26574 | 0x38 | 8 byte |
| operation | Jetson→基板 | 39244 | 0xAB | 32 byte |
| feedback | 基板→Jetson | 39244 | 0x55 | 44 byte |

基板 `app/include/app/robot_config.hpp` と `app/src/robot_ethernet.cpp` を照合。
Jetsonは192.168.3.1:39244にbindした同じソケットでoperation送信・feedback受信する。
基板の送信元IP・ポートはcommandが192.168.3.2:26574、feedbackが192.168.3.2:39244。
wireはlittle endian / IEEE754 float32、boolは0または1、operation予約byteは0。
TargetIdはStartZone=0、FixedBucket1〜3=1〜3、Desk1〜2=4〜5、Flag=6、MovingBucket=7。
NavigationStatusはStanby=0、Ready=1、Moving=2、Tracking=3、Goal=4、Fail=5。
NavigationCommandはSleep=0、Stanby=1、Move=2。Stanbyはwire側の綴り。
operationコメントの44byte表記と定義・static_assertの32byteに差があり、定義を採用。
ROS内部の時刻・session・世代・更新番号・validityはwireに追加しない。
feedbackはoperationの受理ACKではない。

## 未確定事項・受入保留条件

担当区分は相談先を示すもので、担当者の引受・実機承認は未確認。
2026-10-14にROS側の契約v1を確定し、現STM32の制限付き要求識別をユーザーが選択した。
[manager契約v1](../../docs/auto_mode_contract.md) のC1〜C7が要求・取消・品質・停止・校正の現行規範。下表の根拠と実機の外部残件は引き続き有効。実測値・基板適合は未取得。今回の更新はローカル文書で、未commit・未push。

| ID | 問題と根拠 | 必要な扱い |

|---|---|---|
| U1 | 旧operationと新operationはサイズ・内容が非互換。ヘッダーは共通 | 新形式へ一括移行。ポートだけ変更しない |
| U2 | wireに版・operation連番・受理ACKがない。W5500の受信は長いデータを分割し、旧44 byteの先頭32 byteを受理し得る。[受信処理](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/wiznet_ether/src/socket.cpp#L633-L645) | ROS側で全長・型・送信元を検査。基板の受信保証と完全な版識別は外部条件 |
| U3 | Stanbyの取消意味に仕様と型コメントの差がある。[仕様](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/docs/auto_mode_specification.md#L114-L118)、[型コメント](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/include/app/robot_config.hpp#L65-L68) | 適用状態・完了後の意味を確定 |
| U4 | command sequenceは0のまま、固定Moveは保持。同じ対象への再押下を識別できない。[要求生成](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/robot_state_machine.cpp#L83-L105) | 新規要求の識別、同一要求の再送、完了後の再要求、再起動・順序逆転の扱いを確定 |
| U5 | 取消後も追従ボタンを保持すると次のteleopでMoveに戻る。[入力処理](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/robot_state_machine.cpp#L58-L80) | 取消の優先順位、解除までの保持、再開条件を確定 |
| U6 | operation断は手動移譲で、全停止ではない。teleop断は仕様上全停止。[安全仕様](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/docs/auto_mode_specification.md#L47-L49) | 自動指令停止・Fail送信・手動移譲・物理停止を別々に判定 |
| U7 | 全停止関数でアーム系ESC停止がコメントアウトされ、ベルト中断もない。[停止関数](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L414-L422) | 全機構停止をROS2だけで保証しない |
| U8 | 出力更新はteleop受信時で、operation・timeout処理はその後。[ループ](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L524-L550) | timeout・復帰時の旧指令適用をHILで確認 |
| U9 | Ready/Goalの手動威力、Failの手動ハンドに必要な入力生成が手動モード内に限定。[入力生成](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L219-L246) | 基板側の外部修正・検証条件 |
| U10 | 高さ[m]・展開・把持の物理変換と安全ロックに不一致。target=0は停止とは限らない。[調停](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/control_arbitrator.cpp#L34-L69) | 単位・原点・範囲・ロック・停止・手順を確定 |
| U11 | 射出初速とベルト速度が混同され、速度0も射出禁止にならない。[ベルト制御](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/include/app/belt_launcher_controller.hpp) | 校正と無効解時の扱いを確定 |
| U12 | 角度の無効値・丸め・符号が未定義。LEDには度をradフィールドへ代入。[LED転送](https://github.com/tmcit-ararobo-2026a/gn10-mainboard/blob/e66ca03305666d2c53ba97a3e5c855201e2f7e53/app/src/app.cpp#L186-L192) | 99rad流用や無条件クリップを禁止 |
| U13 | Ready品質、Goal離脱、Fail復帰、危険領域、ハンド手順の詳細不足 | 必須設定・受入条件として確定 |
| U14 | 実モード・teleop断・operation受理状態がJetsonへ報告されない | Jetson要求と基板実状態を区別。feedbackを受理ACKと扱わない |
| U15 | 参照文書・リンクに不一致。shutdown/logging/cloth_collectの意味が未定義 | 未定義の副作用を実装しない |


## 有効化と停止の契約

- codec・loopback通信・模擬watchdog試験は上の確認済みwire契約で進める。
- shutdown / logging / cloth_collectの副作用は実装しない。commandの値は型で公開するだけ。
- 角度は既に承認されたint8度値のみ受け付け、[-20,20]外を拒否。未定義の無効値・99rad流用・radの丸め・クリップを行わない。
- Ready品質・再要求識別・取消優先順位・再開・Goal離脱・危険領域・ハンド原点／停止／手順・ベルト校正の契約が欠ける機能は実機で有効化しない。
- ROS初期設計値: Operation周期20ms、Operation期限100ms、command/feedback期限150ms。受信期限は単調時計。基板のoperation200ms/teleop100msと別であり、実機負荷・応答は未検証。
- Operation断・manager停止では自動速度をゼロ化したFailの送信を試みる。これは手動移譲であり、全機構停止の保証ではない。
- 試験fixtureはloopback専用。限定HILは不要機構が実際に駆動しない構成を機構担当が確認した場合のみ。全体実機はU1〜U15・各校正・機構停止経路の確定と個別測定が必要。

## 要求処理の必須確定条件

1. sequenceが要求単位か送信単位か、新規要求と周期再送の識別。
2. Goal／Fail／取消後の同一対象再要求。
3. Sleep／Stanby／新Move／追従保持の取消優先順位。
4. 解除・再押下・通信復帰時の再開条件。
5. 基板再起動、番号周回、順序逆転・古い要求。

ROS内部世代で排除できるのは遅延Service・Action結果などであり、wireにない識別は復元できない。
現行契約v1ではsequenceを要求IDとして扱わず、Sleep/Stanbyの150ms以上の連続観測を再受付条件にする。保持Moveの再実行、取消中の別Move予約、通信復旧だけの再開は禁止。取消ROS終了確認の期限は200msで、物理停止時間とは別。
T00のROS仕様策定は完了。完全要求識別、取消保持、実測品質、全機構停止・原点・校正と担当者照合・実機受入は未完了。今回の契約文書変更で通信コードやwireを変更していない。
