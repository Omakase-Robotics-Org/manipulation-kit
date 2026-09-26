# Teach: hand-taught gestures for omakaseos (`mkit-teach`)

`manipulation_kit.teach` and the `mkit-teach` CLI record a D1 gesture by moving
the arms by hand, reduce it to keyframes, check it, and write the omakaseos
gesture CSV. They replace d1-sdk `gesture_record` and omakase-core's
`/d1_teach` panel, which stopped working when `d1-firmwared` took over the arm
link (see *Where the old tool went*). Everything goes through the daemon's REST
API — the generated client and `FirmwareExecutor` — so it works with omakaseos
running, over `ssh -L 4750:127.0.0.1:4750`.

---

## 運用手順（オペレーター向け）

前提: ロボット上で `d1-firmwared` が動いている。手元 PC から
`ssh -L 4750:127.0.0.1:4750 d1-2` でトンネルし、`pip install
'manipulation-kit[firmware]'` 済みの環境で実行する（`--url` の既定は
`$D1FW_URL` または `http://127.0.0.1:4750`）。**腕を動かすのは record と play
だけ**。export / check / register / keyframes はロボットに触れない。

### 全体の流れ

```sh
mkit-teach record                                   # 名前 → 腕 → HOLDING → 記録 → ~/teach/<name>.json
mkit-teach export ~/teach/<name>.json               # → ~/teach/<name>_motion.csv
mkit-teach check  ~/teach/<name>_motion.csv --ascii
mkit-teach play   ~/teach/<name>_motion.csv --dry-run
mkit-teach play   ~/teach/<name>_motion.csv
mkit-teach register ~/teach/<name>_motion.csv <gesture.yaml のパス>
```

**各コマンドは最後に `Next:` として次に打つコマンドを絶対パス入りで表示する**。
失敗したときは `To fix:` として直し方（recover の方法、再実行のコマンド）を出す。
`To fix:` が出るのは本当に直すものがある時だけ。**ヒントが `--yes` を勧めることは
無い**（`--yes` は HOLDING の確認を飛ばすスクリプト専用）。

### 1. 録る（record）

```sh
mkit-teach record                              # 名前と腕を聞く。既定: ブレーキ解放, 20 Hz
mkit-teach record --name wave --arms left      # 聞かずに始める（片腕: 反対側は位置保持）
mkit-teach record --name wave --mode keyframe  # Enter ごとに 1 ポーズ、q+Enter で終了
mkit-teach record --name wave --compliance     # 旧 gesture_record の force_compliance
mkit-teach record --name wave --no-brake       # サーボ OFF のみ、ブレーキに触らない
mkit-teach record /path/take.json --arms both  # 保存先を直接指定（スクリプト向け）
```

1. **名前**: `Gesture name (a-z, 0-9, _, -):`。テイクは `<teach dir>/<name>.json`
   （teach dir = `$MKIT_TEACH_DIR`、既定 `~/teach`）。同名があれば上書きか別名かを
   聞く（端末でなければ拒否、`--yes` なら上書き）。名前は記録に残るので export に
   `--name` は要らない。
2. **腕**: `Arms to teach [both/left/right] (default both):`（b / l / r も可）。
   `--arms` を付ければ聞かない。選択は記録の `arms` に残る。
3. **安全契約**（下記）を、選んだ腕だけを名指しして表示し、**`HOLDING` と 1 回だけ**
   打つ（両腕でも 1 回）。
4. アームリースを `operator` クラスで取る。**入口の recover**: 位置モードでない腕
   （idle / error。手で動かして idle のまま置いた腕を含む。教えない側の腕も）を
   計測姿勢で recover して位置保持にする（`recovering arm b (idle, 40.2 deg from
   its command)` と表示）。これは teach だけの明示的な選択で、エージェント実行の
   経路では従来どおり拒否する。
5. 位置モード（比 0.15）でまっすぐ HOME へ（記録しない）。**ジェスチャーは必ず
   HOME から始まる**（`--no-home-start` なら、教える腕が HOME から 2 deg 超で拒否）。
6. **サーボ OFF → 確認 → 3, 2, 1 → ブレーキ解放**: `idle` を要求し、腕が idle と
   **報告するまで待つ**（ブレーキ保持中なので安全。error と報告されたら故障として
   止め、解放しない）。カウントダウン（`--countdown N`）の "0" で `brake_release`
   （確認語 `RELEASE_BRAKE`、時限窓 20 s、記録中は 1/3 ごとに再送）。
   **解放が受理された瞬間が t = 0**。`--compliance` / `--no-brake` はカウントダウンの
   後にそのモードへ入り、モードを確認してから記録する（compliance はさらに 1.5 s
   安定待ち。ツール未登録の空フランジは拒否、`--allow-bare-flange` で許可）。
7. 手で腕を動かす。**Enter** で停止。**録画中の Ctrl-C も停止で、テイクは保存
   される**。`--duration-s` / `--stationary-s` / `--stop-file` でも止まる。サンプルが
   2 未満ならテイクは破棄し、録り直しのコマンド
   （`mkit-teach record --name <name> --arms <arms>`）を出す。録画開始前の Ctrl-C は
   何も保存しない。
8. **停止時は（例外時も必ず）まずブレーキ締結** → 腕が静止し同じモードを 0.3 s
   保つまで待つ → `recover`（計測姿勢で位置保持、比 0.05。position の報告まで確認。
   デーモンの確認ループのため文書の `x-timeout-seconds` = 65 s まで待つ）。拒否
   されたら 1 秒おきに計 3 回。キット側が待ちきれなかった場合は再送しない。それでも
   駄目なら腕は **idle・ブレーキ締結のまま（安全）** 残し、「デーモンが拒否した
   （DAEMON REFUSED）」か「キットが待つのをやめた（KIT GAVE UP）」かを区別して
   `To fix:` に復帰方法（コンソールの Arms → Recover、`POST /v1/arm/<side>/recover`）
   を出す。
9. `[recorded] <take>: N samples, T s` と `Next: mkit-teach export <take>`。

```
lease → [入口 recover] → position（確認）→ HOME
      → idle 要求 → idle の報告を確認（ブレーキ保持中）→ 3, 2, 1 → brake_release（t = 0）→ 記録
      → 停止（Enter / Ctrl-C）: brake_engage → 静止＋モード安定 0.3 s → recover（最大 3 回）
      → position を確認 → リース返却
```

記録ファイル（JSON）は生データのまま（平滑化も手首固定もしていない）なので、
設定を変えて何度でも export をやり直せる。グリッパーの閉度も記録されるが、CSV には
列が無い（omakaseos のプレーヤーは 15 列以外を拒否する）。

### ブレーキ解放の安全契約（record の既定）

* サーボ OFF・ブレーキ強制開放: **支えていない腕はその瞬間に重力で落ちる。**
* 解放する腕は、解放前から記録終了まで**必ず人が支える**。落下経路に手や顔を
  入れない。確認（`HOLDING`）はセッションに 1 回、選んだ腕すべてに対して。
* 解放は時限式: デーモン自身が窓（既定 20 s、記録中は 1/3 ごとに再送）の
  終わりに締結する。このツールが落ちても窓で必ず締まる。
* 締結されるのは: 記録終了（このツール）、窓切れ、`brake_engage`、estop、
  ソフトキル、デーモン停止。解放中はその腕への mode / recover / trajectory を
  デーモンが拒否する。
* ブレーキ状態はデーモンの指令記録でありセンサではない。ブレーキ経路は
  2026-09-23 に d1-2 で実機確認済み（BRAK1=2 で解放 → Enter で締結 → recover）。
* d1-firmware PR #92 より古いデーモンは `OperationUnavailable` で止まる（その時は
  `--compliance` か `--no-brake`）。

### 2. 書き出す（export）

```sh
mkit-teach export ~/teach/wave.json                   # → ~/teach/wave_motion.csv
mkit-teach export take.json out.csv --name wave --sentiment neutral --usage filler
```

出力先を省くと**テイクと同じディレクトリ**の `<name>_motion.csv`（名前は記録の
ものか `--name`）で、絶対パスで表示する。sentiment / usage を省くと端末なら聞き、
そうでなければ neutral / filler。処理は旧パネルの Review & Save と同じ順:

| 手順 | 既定 | オプション |
|---|---|---|
| 開始時の落ち込みを捨てる（ブレーキ解放直後、最初の 0.5 s 以内で 8 deg/s を超えた最後のサンプルまで） | on | `--sag-max-s` / `--sag-vel` / `--no-sag-trim` |
| 手首 J5–J7 は**教えたまま残す**。可動範囲が 2 deg 未満の手首関節だけ（垂れ・ノイズ）HOME に固定し、その旨を表示 | 固定しない | `--pin-wrist`（旧 gesture_record の全固定） |
| 平滑化: 1 サンプルだけの飛び（両側が平ら）を除去 → 2 次の Savitzky–Golay。**速い折り返しの高さを削らない**（旧 gesture_record の中央値 → 平均は task7 の L7 −70.0 deg を −58.4 deg に削っていた） | 5 サンプル | `--smooth-window N` / `--no-smooth` |
| HOME から最初の姿勢へつなぐ区間を一定の関節速度で追加 | テイク自身のピーク関節速度（平滑化後）を 20–90 deg/s に収めた値（キーフレームモードは 20） | `--home-speed` / `--no-home` |
| 最後の姿勢から HOME へ戻る区間を**専用の控えめなプロファイル**で追加: 最後の姿勢で一旦止まる dwell ノット → min-jerk（0.1 s ごとのノット）。ジェスチャーの速さとは無関係。時間 = max(最短, 15/8 × 最大関節距離 / ピーク速度, √(5.77 × 距離 / ピーク加速度)) | ピーク 40 deg/s・90 deg/s²・最短 2 s（速度上限より速くはしない） | `--home-return-vel` / `--home-return-acc` / `--home-return-min-s` / `--no-home` |
| キーフレーム削減（直線から ε 以内を間引き） | 1.5 deg, collinear | `--epsilon-deg` / `--method dp` / `--min-spacing-s` |
| 停止区間の短縮 | off | `--max-idle-s 0.25/0.5/1` |
| 速度上限まで時間だけ延ばす（下記「速度」） | 150 deg/s・600 deg/s² | `--max-joint-vel` / `--max-joint-acc` / `--no-speed-limit` |
| 安全チェック（check と同じ）→ 関節リミット・速度・時刻が NG なら書かない | — | `--force`（UNSAFE 印付き、play は `--no-safety` が必要） |
| 干渉の所見は警告のみ。`# mkit-teach: min_clearance=…` / `guard_advisory=…` を残す | — | — |

export は必ず**時間の内訳**と**関節ごとの可動範囲**を表示する:

```
timing: recorded 5.24 s -> body 5.10 s (speed cap stretched 3 knot(s), +0.20 s) + HOME connect 0.40 s (at 45 deg/s) + return 2.60 s (min-jerk, peak 40 deg/s, 90 deg/s^2, at least 2 s) = 8.10 s
joint range recorded -> exported [deg]: L1 40.0 -> 39.2, L7 35.7 -> 35.1, R5 1.2 -> 0.0 (pinned: under 2 deg, sag/noise)
```

半分未満に減った関節には `(LOST)`、3 deg 以上小さくなった関節には
`(peak shaved X deg)` が付く（記録側の範囲はエンコーダの 1 サンプルの飛びを除いて測る）。速度上限で延ばした時だけ `speed:
stretched …` の行も出る。`[saved]` と check の時間はどちらも**デーモンの
スプラインの長さ**（0 行目の duration はプレーヤーが無視するので含めない）。

#### 速度（SpeedPolicy）

**ジェスチャーの速度を制限しているのはキットのこの上限だけ**（デーモンは 1 ステップ
350 deg/s を見るだけ、omakaseos のプレーヤーは何も見ない）。`process.SpeedPolicy`
の 1 か所で、export（延ばす）・check（判定）・play（事前チェック）が同じものを使う。

* 既定 **150 deg/s・600 deg/s²**（Shu 2026-09-23「案 c」: 教えた速さで再生し、
  それより速い部分だけ延ばす）。旧 gesture_record の 25 deg/s・120 deg/s² は
  `process.LEGACY_SPEED`。
* 使った上限は CSV に `# mkit-teach: max_joint_vel=… max_joint_acc=…`（延ばした
  場合は `speed_stretch=…` も）として残り、**check と play はその CSV 自身の上限で
  判定する**（`--max-joint-vel` / `--max-joint-acc` で上書き可）。キーが無い CSV は
  既定値で判定。

### 3. 確認する（check）

```sh
mkit-teach check ~/teach/wave_motion.csv --ascii
```

デーモンが実際に再生するスプライン（HOME から始まる Catmull-Rom）を 10 ms 刻みで
たどる。

* **不合格（exit 1）**: 関節リミット（クリップせず違反扱い）、手首ロールの連成
  リミット（J6 に応じた |J7| 上限）、速度・加速度（CSV の上限）、時刻と角度の
  有限性。
* **HOME への復帰区間は別に表示**（`HOME return: … s over N segment(s), peak
  velocity … deg/s, peak acceleration … deg/s^2, … deg/s at HOME; profile 40
  deg/s, 90 deg/s^2`）。CSV が `# mkit-teach: home_return_frames/vel/acc` で
  復帰区間を宣言していれば、そのプロファイル超過と HOME 到着時に動いている
  こと（2 deg/s 超）は不合格。宣言の無い古い CSV は最後の区間を測り、既定
  プロファイルより速い・HOME に動いたまま着く場合に警告だけ出す（テイクから
  export し直せば直る）。
* **警告だけ（exit 0）**: MotionGuard の胴体・胸・両腕間・自己干渉。最も近づいた
  距離・フレーム名・時刻を出す。教示は手で実際に通った姿勢なので、キットの
  カプセルモデルは裁かない（Shu 2026-09-23）。ただし**デーモンは同じ違反で再生を
  拒否する**（下記「ガード」）。

`--ascii` は関節ごとの帯グラフ。動画は `python examples/preview_gesture.py
wave_motion.csv`（mujoco）。

### 4. 再生する（play）

```sh
mkit-teach play ~/teach/wave_motion.csv --dry-run       # 事前チェックだけ（ロボットに触れない）
mkit-teach play ~/teach/wave_motion.csv                 # policy リース, 比はジェスチャーから自動
mkit-teach play ~/teach/wave_motion.csv --vel-ratio 0.5 # 比を固定する
```

UNSAFE 印・チェック NG・コントローラ異常のいずれでも何も動かさない
（`--no-safety` はキットの事前チェックだけを飛ばす。デーモンの検査は飛ばせない）。
位置モードでない腕は入口で recover する（表示あり）。HOME から 2 deg 以上離れて
いれば先に HOME へ移動し、再生後に HOME 到着（計測値）と静止を確認する。

**速度比**: 位置モードのコントローラーはデーモンの 1 ms ごとの目標を最大
140 deg/s × vel_ratio でしか追わない（旧固定 0.15 では約 21 deg/s で、task6 の
118 deg/s の手首の振りが丸められた）。**比が隠れたブレーキになってはいけない**。
速度の関門は CSV の SpeedPolicy だけ。

* ジェスチャー本体の比 = `clamp(1.3 × スプライン上のピーク / 140, 0.3, 1.0)`
  （118 deg/s → 1.0、25 deg/s の旧 CSV → 0.3）。加速度比も同じ値。
* 選んだ比と理由を動かす前に表示する（`playback ratio 1.00: gesture peak 117.8
  deg/s x 1.3 …; HOME approach at 0.30`）。
* ジェスチャー前の HOME への事前移動は**落ち着いた 0.3**。ジェスチャー内の HOME
  接続・復帰区間は本体と同じ比（比は 1 回のトラジェクトリにつき 1 つ）。復帰区間が
  遅いのは比ではなく**タイミング**による（ピーク 40 deg/s の min-jerk）。
  `--vel-ratio` は両方をその値に固定する。

### 5. 登録する（register）

```sh
mkit-teach register ~/teach/wave_motion.csv <omakaseos の checkout>/robot_stack/robots/omakase/d1/gesture.yaml
mkit-teach gestures <gesture.yaml のパス>      # 登録済みの一覧（source と CSV の有無）
```

CSV を渡すと、その `gesture.yaml` の `csv_base_dir`（無ければ横の `csv/`）へ
`<name>_motion.csv` としてコピーし、CSV のヘッダーにある name / sentiment / usage で
エントリ（`name: d1_<name>`, `source: teach`）を追加または置換する。UNSAFE 印の
CSV は入れない。キットは omakaseos の場所を知らないので、`gesture.yaml` のパスは
毎回明示する（omakase-core 側に Makefile のショートカットを置く予定）。
名前だけを渡す `mkit-teach register wave [gesture.yaml]` はエントリの表示・追加だけ。
登録後は omakaseos 側で `git status` を見てコミットする。

### ガード: デーモンの干渉検査は常に有効

Shu 2026-09-23 21:14Z: 干渉ガードはどこでも常に有効。`play` は
`POST /v1/arm/trajectory/start` に `guard` フィールドを**送らない**。再生だけ
干渉検査を緩める経路は無い（d1-firmware PR #106 でデーモン API からも削除）。

* デーモンはアップロード時と再生中の 1 ms ごとに、実形状に基づくモデルとその
  マージン（PR #106）で干渉を検査し、関節リミット・350 deg/s・時刻・最初の点
  （計測から 3 deg 以内）も検査する。違反があれば拒否 / 停止する。
* キットの `check` の干渉所見は警告のままだが、デーモンは同じ違反を拒否する。
  警告が出たら、再生前に教え直すか姿勢を見直す。

### Known limits (2026-09-23)

* **再生の忠実度はコントローラの位置モードのプランナーで頭打ち**。task7 は比 1.00
  で再生したが（キットは設計どおり）、112 deg/s の振りは位置モードで丸められ、
  終了 3.3 s 後でも 16.2 deg 遅れていた。これはデーモン／コントローラ側の制約で、
  PD モードのファームウェアとデーモンのストリーミング再生が入るまで残る。追跡:
  d1-firmware issue #96（TrajectoryStatus should report playback position and
  per-joint tracking/final error）と、PD モード再生について別エージェントが起票
  する issue（controller PD mode / daemon-side streaming for taught gestures）。
* **加速度上限 600 deg/s² が、とても切れのよいテイクを延ばす**。キーフレームの角で
  Catmull-Rom の加速度が大きくなるため、速度が 150 deg/s 未満でも延びることがある
  （例: 113 deg/s の振りが 2.95 s → 3.18 s）。`timing:` 行に何秒延びたかが出る。
  必要なら `--max-joint-acc` で上げる。
* **J6/J7 の連成リミットを見ているのはキットだけ**（d1-firmware issue #99）。
  `--no-safety` は使わないこと。
* 平滑化とキーフレーム化の後の再生スプライン（デーモンの一様 Catmull-Rom）は、
  間隔の違うキーフレームの隣で速度が跳ねたり、折り返しをわずかに行き過ぎたり
  する。跳ねは速度上限で時間を延ばして抑える（`timing:` 行に出る）。
* ブレーキ解放の経路は 2026-09-23 に実機で確認済み。mode の報告待ち（#103）と
  締結直後の recover 拒否（#104）はキット側で吸収している。

### 実機ログ（2026-09-23, d1-2）

| # | kit | 起きたこと | 対応 |
|---|---|---|---|
| 1 | 3fad724 | idle 要求の 5 ms 後の brake_release が拒否（コントローラの idle 報告は 11 ms 後）。締結 2 ms 後の recover が RESET1 code 8 で拒否。手で idle にした腕（指令と 40.2 deg 差）で開始不能 | dc51dbe: モードの報告を待つ、安定待ち＋recover 再試行、入口 recover |
| 2 | dc51dbe | 記録成功（63 サンプル, 5.44 s）。recover がクライアントの 2 s タイムアウトで取り消された | 5956e9e: 文書の `x-timeout-seconds` を使う |
| 3 | 5956e9e | take2: CSV がカレントディレクトリへ。25/120 の上限で 3.83 → 5.52 s | 40a0b6d: テイク横に書く、SpeedPolicy 150/600、上限を CSV に |
| 4 | ba9a642 | Ctrl-C 後のヒントが `--yes` を勧めた、不要な `To fix:`、腕を聞いてほしい | 279d877 |
| 5 | ba9a642 | task3/task4: 手首固定で L7 35.7 deg → 0、5.24 → 6.35 s | ba4c931: 手首は教えたまま、HOME 区間をテイクの速さで、内訳表示 |
| 6 | ba4c931 | task6: CSV は正しいが比 0.15（約 21 deg/s）で丸められた | 2f9f7ad: 比をジェスチャーから決める |
| 7 | 2f9f7ad | task7: 比 1.00 で再生。位置モードの丸めは残る（16.2 deg 遅れ）。export の平滑化も L7 の −70.0 deg を −58.4 deg に削っていた | 平滑化を山を削らない方式に（下記 0.16.0）。追従の遅れは Known limits（コントローラ側） |

---

## Developer notes (English)

### The CSV contract (pinned from the reader, not the prose)

Reader of record: omakase-core `robot_stack/robots/omakase/d1/firmware_session.py::trajectory_from_csv`
(commit f62faf24). Writer of record: d1-sdk `gesture_csv.h::toCsv`.

* `#` lines are comments; rows whose first cell is `duration`/`kp`/`kd` are
  skipped. Header `duration,R1..R7,L1..L7`.
* Exactly 15 finite numbers per row: `duration` (s, > 0, previous row → this
  row) and 14 **degrees**. No gripper column — the firmware reader refuses any
  other count.
* `R1..R7` = SDK ArmSide A = **physical LEFT** = kit `left`; `L1..L7` = B =
  physical RIGHT. The R/L letters are the vendored mesh trees' names.
* The player overwrites row 0 and the last row with HOME
  (`config/home_pose.json`), starts the spline at HOME at t = 0 and ignores row
  0's duration; row k plays at `sum(duration_1..k)`. The daemon samples those
  knots as uniform Catmull-Rom on a linear time base and checks every 1 ms
  (limits, speed and clearances, always).
* Teach metadata travels as `# mkit-teach: key=value` comments (name,
  sentiment, usage, source, `home_sha` = fingerprint of the HOME pinned,
  `max_joint_vel` / `max_joint_acc` = the speed ceiling, `speed_stretch` when
  it slowed the take, `min_clearance`, and `guard_advisory` when a guard
  margin was not met);
  a force-saved file carries `# mkit-teach: UNSAFE=<violation>` lines —
  only for HARD findings (limits, rates, timing), never for the guard.

### Where the old tool went

`gesture_record` (d1-sdk `devices/omakase_arm/example/gesture_record.cpp`)
opened the arm controller over UDP itself; omakase-core's `/d1_teach` panel
(`status_server/d1/teach.py`, `static/d1_teach.html`) spawned it. The firmwared
migration (omakase-core 2050ca49 / faa32a71 / fda96c19, 2026-09-08..10)
ported **playback** to daemon trajectories and left teach on the subprocess:
the panel still renders, but `gesture_record` cannot connect while
`d1-firmwared` owns UDP 4730 (d1-sdk PR #103 makes it refuse, exit 3). Nothing
was deleted; it simply stopped being able to reach the arm.

### HOME rules for a hand-guided take

* `t = 0` of a brake-release take is the instant the release is acknowledged;
  the countdown and the pre-release pose are not in it.
* The start sag: `process.settle_index` takes, within the first `sag_max_s`
  (0.5 s), the LAST sample whose max joint speed (3-sample median filtered)
  exceeds `sag_vel_deg_s` (8 deg/s), and the stream starts after it. It is the
  last fast sample, not the first slow one, because a drop-and-catch has a
  zero-speed turnaround at its bottom. A take with no fast start loses nothing.
* The motion then starts at HOME (row 0) and reaches the first kept sample in
  `max|q - HOME| / home_speed_deg_s`; the stream is smoothed after the cut, so
  the join is continuous in position, and the daemon's C1 Catmull-Rom plus the
  limiter keep it continuous in velocity and under the caps.
* The player replaces the LAST row with HOME, so the last recorded pose is kept
  as a real row and the return is appended after it on its own profile
  (`process.HomeReturn`, not the take's speed): a dwell knot at the last pose
  (at least as long as the last body segment, up to 1 s, and long enough for
  the dwell's peak speed/acceleration to be within the profile), then min-jerk
  knots every 0.1 s to HOME, so the daemon's Catmull-Rom leaves the pose and
  reaches HOME at rest. Duration
  `max(min_s, 15/8 * max|q_last - HOME| / peak_vel, sqrt(5.77 * max|q_last -
  HOME| / peak_acc))`, default 40 deg/s, 90 deg/s^2, 2 s, never above the
  gesture's speed ceiling. The CSV declares it (`home_return_vel`,
  `home_return_acc`, `home_return_frames`) and `check` reports and holds it
  separately. Idle trimming leaves it alone. Keyframe-mode takes return the
  same way. (Before: one HOME row at the take's clamped peak speed; a 60 deg
  return peaked at 113 deg/s on the spline and reached HOME at 45 deg/s.)
* A start/end already within `epsilon_deg` of HOME is snapped to HOME, as
  gesture_record did (the golden wave take is such a take).
* The wrist is kept as taught: gesture_record pinned J5–J7 to HOME against
  compliance sag, which erased deliberate wrist motion under brake release
  (task4). `pin_wrist` restores that; otherwise only a wrist joint whose
  recorded range is under 2 deg is pinned.

### What was ported, and the two deviations

`teach/process.py` ports `smoothSamples`, `reduceSamples`, `buildGesture`,
`limitJointDynamics` (gesture_csv.h) and `_trim_idle_keyframes` (teach.py)
with gesture_record's defaults except the speed ceiling (150 / 600, see
*速度*) and the wrist (free). `teach/record.py` ports the capture sequence
(HOME first, compliance with `xAxisCompliance(2.0)` at 5 %, 1.5 s settle,
sampling, stationary auto-stop, hold on exit); brake release is the default
guide, gesture_record's compliance is `--compliance`. Deviations, both
because the player changed:

1. Row 0 **is** HOME. gesture_record wrote the first kept sample as row 0
   (gesture_play prepended HOME); the firmware player overwrites row 0, which
   would silently drop that keyframe.
2. Dynamics are limited, and `check` samples, on the **daemon's** spline
   (`trajectory_points`), not gesture_play's `[HOME, kf0, …]` path.
   Cross-check: every one of omakase-core's 30 library CSVs passes `check`
   with peaks of exactly 25.0 deg/s and ≤ 120 deg/s² (gesture_record's caps,
   `process.LEGACY_SPEED`), and `limit_joint_dynamics` at those caps is a
   no-op on all 30 (they were repaired to them). The teach default ceiling
   is 150 deg/s, 600 deg/s² since 2026-09-23 (Shu), so those files pass it
   with room to spare.

Also: timestamps are real (the daemon read is not perfectly periodic), the raw
capture is kept raw, and Douglas–Peucker (`--method dp`) is offered beside the
greedy collinear walk.

### Confirmed mode transitions (first live run, d1-2 2026-09-23)

`POST /v1/arm/{side}/mode` returns when the daemon has ACCEPTED the request,
not when the controller has switched; the daemon's own confirmed transitions
(`arm_recovery.rs`, `confirmed(…, 500, 8, …)`) poll for the report. The kit
now does the same, in one place:

* `FirmwareExecutor.wait_for_mode(side, modes, timeout_s=3.0, poll_s=0.02,
  steady_s=0, stationary=False)` polls the generated `ArmState` until `mode`
  is in `modes` (any mode for `None`), optionally unchanged for `steady_s`
  and `stationary`. Timeout → `ModeUnconfirmed` naming the last mode; an
  `error` report when `error` was not asked for → `ModeUnconfirmed` at once.
* `position_mode()` confirms `position` after each request (the executor's
  "completed means arrived" contract, like the end-of-motion barrier).
* `record` confirms `idle` before `brake_release` (the daemon's preflight
  accepts idle|error; an error after an idle request is a fault, surfaced),
  and for `force_compliance` waits for `COMPLIANCE_REPORTS` (`torque`, `pvt`,
  `release`, `unknown`) + `stationary`. The document maps no
  `ArmModeCommandMode` onto the feedback `ArmMode`, so "servo-on, not the
  position hold it left, not idle, at rest" is the derivable check.
* `FirmwareExecutor.recover_arm(side, steady_s=…)`: optional steady wait,
  `arm_recover` at 0.05, confirmed `position`; 3 attempts, 1 s apart; a lost
  lease is never retried. Teardown uses it with `steady_s=0.3`, after the
  brakes are engaged.
* **Every request honours the document's `x-timeout-seconds`.**
  `FirmwareClient._send` (under every generated operation and `request()`)
  waits `timeout_for(method, path)`: the bound declared for the route the
  concrete path instantiates (literal routes outrank templated ones), never
  below the client's `timeout` floor (2 s). On the bundled document:
  `recover` 65 s, `mode` / `brake_release` / `brake_engage` 10 s,
  `move_joints*` 30 s, gripper strokes 40 s; `lease` declares 0.5 s and keeps
  the 2 s floor. Second live run (dc51dbe, 20:46Z): the recover was
  cancelled by the daemon at exactly 2.0 s because the client hung up. A
  request that outlives even the documented bound raises `ClientTimeout`
  ("the kit gave up", not a `FirmwareError` refusal); `recover_arm` never
  retries after one (the daemon may still be running it) and raises
  `RecoverFailed` with a reason per attempt (`refused`, `client_timeout`,
  `unconfirmed`, `not_steady`, `error`), which `exit_problems` spells out.
* `FirmwareExecutor(recover_on_entry=True, announce=…)` recovers every
  non-position arm before `position_mode` (`recover_idle_arms()`, lines kept
  in `entry_recoveries` and in the recording's `meta.entry_recoveries`).
  Only `mkit-teach` sets it; the default executor still refuses an idle arm
  whose command is more than 3 deg from its measurement.

### Daemon surface used (generated client only)

`GET /v1/arm/{side}/state`, `GET /v1/gripper/{side}/state`,
`GET /v1/arm/{side}/tool`, `POST /v1/arm/{side}/mode` (`ArmModeCommand`),
`POST /v1/arm/{side}/recover`, `POST /v1/arm/{side}/brake_release` /
`brake_engage`, `GET /v1/arm/{side}/brake` (PR #92; `OperationUnavailable`
against an older daemon), and — through `FirmwareExecutor` — the lease,
`/v1/arm/trajectory/*` (no `guard` field). `FirmwareExecutor.play_waypoints`
is the public entry for an already-timed trajectory (the plan path `_play`
derives its own timing); `FirmwareExecutor.set_ratios` re-installs position
mode at the ratio a gesture needs. The bundled client is generated from
d1-firmware PR #106's document (spec `3b354c25…`, the always-on guard); `ensure`
regenerates from whatever the daemon serves at connect.

### Known sharp edge: HOME near the body margin — the daemon's call

At HOME the left arm's closest link is 30.3 mm from `torso_belly` against
the kit guard's 30 mm margin, and only J2+ / J1− (A; mirrored on B) eat into
it. A Catmull-Rom returning to HOME from an outward J2 swing overshoots HOME
by a fraction of a degree, which the kit's `check` reports as an advisory
`guard (advisory) body … torso_belly` warning. Whether such a gesture plays
is the daemon's decision: its guard is always on, with its realistic model
and margins (d1-firmware PR #106, which gives HOME 26.7 mm of headroom), and
`play` says before it moves that the daemon refuses such violations.
