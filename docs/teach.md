# Teach: hand-taught gestures for omakaseos (`mkit-teach`)

`manipulation_kit.teach` + the `mkit-teach` CLI record a D1 gesture by moving
the arms by hand, reduce it to keyframes, check it, and write the omakaseos
gesture CSV. It replaces d1-sdk `gesture_record` and omakase-core's
`/d1_teach` panel, which stopped working when `d1-firmwared` took over the arm
link (see *Where the old tool went* below). Everything goes through the
daemon's REST API — the generated client and `FirmwareExecutor` — so it works
with omakaseos running, over `ssh -L 4750:127.0.0.1:4750`.

---

## 運用手順（オペレーター向け）

前提: ロボット上で `d1-firmwared` が動いている。手元 PC から
`ssh -L 4750:127.0.0.1:4750 d1-2` でトンネルし、`pip install
'manipulation-kit[firmware]'` 済みの環境で実行する（`--url` の既定は
`$D1FW_URL` または `http://127.0.0.1:4750`）。**腕を動かすのは record と play
だけ**。keyframes / export / check / register はロボットに触れない。

### 1. 録る（record）

```sh
mkit-teach record take.json                 # 既定: 両腕, ブレーキ解放（手で導く）, 20 Hz
mkit-teach record take.json --arms left     # 片腕だけ解放する（反対側は位置保持）
mkit-teach record take.json --mode keyframe # Enter ごとに 1 ポーズ、q+Enter で終了
mkit-teach record take.json --compliance    # 旧 gesture_record の force_compliance で録る
```

既定は**ブレーキ解放（ハンドガイド）**。Shu 2026-09-23:「コンプライアンスだと
動かすのが難しかった」ため、サーボ OFF + 保持ブレーキ解放で直接手で動かす方式を
既定にした。

流れ:

1. 下記「ブレーキ解放の安全契約」を表示し、**`HOLDING` と 1 回だけ**打つ
   （両腕でも 1 回。腕ごとには聞かない。`--yes` はスクリプト専用）。
2. アームリースを `operator` クラスで取る（テレオペ・コンソールと同格。
   `policy` の自律動作より優先）。
3. 位置モードでまっすぐ HOME へ移動（記録しない）。**ジェスチャーは必ず HOME
   から始まる**: `--no-home-start` で移動を省いた場合、教える腕のどれかの関節が
   HOME から 2 deg を超えていれば開始を拒否する。
4. **カウントダウン 3, 2, 1**（`--countdown N`、0 で無し）。
5. 柔らかくする:
   * 既定（ブレーキ解放）: `idle`（サーボ OFF）→ `brake_release`
     （確認語 `RELEASE_BRAKE`、時限窓 20 s、記録中は窓の 1/3 ごとに再送）。
     **解放が受理された瞬間が t = 0**。そこから即記録開始（解放前の姿勢は記録
     しない）。
   * `--compliance`（旧 gesture_record と同じ）: `force_compliance`
     （力方向 `[1,0,0,0,0,0]`、目標力 0、調整上限 2 mm、速度/加速度比 0.05）。
     サーボは入ったまま、手で押すと逃げる。入ってから 1.5 秒待ってから記録開始。
     デーモンにツール（エンドエフェクタ）が登録されていないと拒否する
     （空フランジとして重力補償され手首が垂れるため。何も付けていない時だけ
     `--allow-bare-flange`）。
   * `--no-brake`: サーボ OFF のみ、ブレーキには触らない
     （idle で手で動かせる個体向け）。
6. 手で腕を動かす。**Enter** で停止（Ctrl-C でも停止し保存される）。
   `--duration-s N` で自動停止、`--stationary-s N` で N 秒静止したら停止、
   `--stop-file PATH` でファイルが現れたら停止。
7. 停止時は（例外時も必ず）**まずブレーキ締結**（他の何よりも先。再送スレッドは
   ロックで止めるので、締結の後に解放が飛ぶことはない）→ `recover`（計測姿勢で
   位置保持、比 0.05 = 旧 `lockCurrentPositionMode(5,5)`）→ リース返却 →
   テイク確定。

記録ファイル（JSON）は生データのまま（平滑化も手首固定もしていない）なので、
設定を変えて何度でも 2 以降をやり直せる。グリッパーの閉度も記録されるが、
CSV には列が無い（omakaseos のプレーヤーは 15 列以外を拒否する）。

### 2. キーフレーム化して書き出す（export）

```sh
mkit-teach export take.json wave_motion.csv --name wave \
    --sentiment neutral --usage filler
```

旧パネルの Review & Save と同じ処理を順に行う:

| 手順 | 既定 | オプション |
|---|---|---|
| 開始時の「落ち込み」を捨てる（ブレーキ解放直後、最初の 0.5 s 以内で関節速度 8 deg/s を超えた最後のサンプルまで） | on | `--sag-max-s` / `--sag-vel` / `--no-sag-trim` |
| 手首 J5–J7 を HOME に固定（重力で垂れた手首を記録しない） | on | `--free-wrist` |
| 平滑化（中央値→平均、ジッタ除去） | 5 サンプル | `--smooth-window N` / `--no-smooth` |
| HOME から始め、落ち込み後の最初の姿勢へ一定速度でつなぐ | 20 deg/s | `--home-speed` |
| 最後の姿勢を残し、そこから HOME へ戻る区間を**一定の関節速度**で追加（時間 = 最大関節差 / 20 deg/s。遠くても近くても同じ速さ） | 20 deg/s | `--home-speed` / `--no-home` |
| キーフレーム削減（直線から ε 以内を間引き） | 1.5 deg, collinear | `--epsilon-deg` / `--method dp` / `--min-spacing-s` |
| 停止区間の短縮（Trim idle pauses） | Off | `--max-idle-s 0.25/0.5/1` |
| 再生可能化（速度 25 deg/s・加速度 120 deg/s² 以下になるまで時間だけ延ばす） | on | `--max-joint-vel` / `--max-joint-acc` / `--no-speed-limit` |
| 安全チェック（下記 check）→ 関節リミット・速度・時刻が NG なら書かない | — | `--force`（UNSAFE 印付きで保存、play は `--no-safety` が必要） |
| ガード（干渉）の所見は**警告のみ**。UNSAFE にはしない。最小クリアランスを `# mkit-teach: min_clearance=…`、マージン割れを `# mkit-teach: guard_advisory=…` として CSV に残す | — | — |

始点・終点が HOME から ε（1.5 deg）以内ならその姿勢を HOME にスナップする
（旧 gesture_record と同じ。区間を足さない）。

`--name` を付けると omakaseos の `gesture.yaml` 用エントリを表示する。
`--register <omakase-core>/robot_stack/robots/omakase/d1/gesture.yaml` で
そのファイルに追加/置換まで行う（CSV は同じ階層の `csv/` にコピーする）。

### 3. 確認する（check）

```sh
mkit-teach check wave_motion.csv --ascii
```

デーモンが実際に再生するスプライン（HOME から始まる Catmull-Rom）を 10 ms
刻みでたどる。

* **不合格（exit 1）になるもの**: 関節リミット（クリップせず違反扱い）、
  手首ロールの連成リミット（J6 に応じた |J7| 上限）、速度 25 deg/s・加速度
  120 deg/s²、時刻（有限・0 始まり・単調増加）と角度の有限性。
* **警告だけのもの（exit 0）**: MotionGuard の胴体・胸・両腕間・自己干渉。
  `WARNING: guard (advisory) body: closest -40.0 mm at t=2.60s — arm A link
  Link2_R within … of body box torso_belly …` のように、最も近づいた距離・
  フレーム名・時刻・マージン割れのサンプル数を出す。

ガードを警告に下げた理由: 教示はオペレーターが**手で腕を導いて実際に通った姿勢**
なので、ガードのカプセルモデルがその姿勢の可否を裁く立場にない（Shu
2026-09-23「teaching に関しては、ガードを外した方がいいかも」）。デーモン側も
`play` の既定（`guard: speed_only`）では干渉検査をしない。下の「ガード」節を参照。

`--ascii` は関節ごとの帯グラフ。動画で見たい時は
`python examples/preview_gesture.py wave_motion.csv`（mujoco）。

### 4. 再生する（play）

```sh
mkit-teach play wave_motion.csv --dry-run     # ロボットに触れず事前チェックだけ
mkit-teach play wave_motion.csv               # policy リース, vel_ratio 0.15, guard speed_only
mkit-teach play wave_motion.csv --guard full  # デーモンの干渉検査も有効にする
```

UNSAFE 印・チェック NG（リミット/速度/時刻）・コントローラ異常のいずれでも何も
動かさない（`--no-safety` でキットの事前チェックを飛ばせるが、デーモンの検査は
飛ばせない）。キットのガード警告は**動かす前に表示**してから再生に進む
（`speed_only` のときは「デーモンは干渉を検査しない＝教示どおりに再生する」旨も
表示する）。HOME から
2 deg 以上離れていれば先に HOME へ移動し、再生後に HOME 到着（計測値）と静止を
確認する。

### ガード: ジェスチャー再生は speed_only（d1-firmware PR #102）

Shu 2026-09-23 17:50Z「gesture 再生の時はスピードガードはあっても、範囲のガードは
オフにしていいかと」。`mkit-teach play` はジェスチャー本体を
`POST /v1/arm/trajectory/start` に **`guard: "speed_only"`** 付きで送る
（`--guard full` で従来どおり）。HOME への事前移動は通常の計画移動なので
`guard` を付けず、デーモン既定の `full` のまま。

| デーモンの検査（1 ms ごと、アップロード時と再生中） | `full`（デーモン既定） | `speed_only`（play 既定） |
| --- | --- | --- |
| 干渉: 胴体/胸 30 mm、両腕間 60 mm、自己干渉 | 拒否 / 停止 | **検査しない** |
| URDF 関節リミット（PR #102 で追加。以前はクリップ後の姿勢しか見ていなかった） | 拒否 / 停止 | 拒否 / 停止 |
| 関節速度 350 deg/s | 拒否 / 停止 | 拒否 / 停止 |
| 点数 2..10000・t=0 始まり・単調増加・120 s 以下・有限 | 拒否 | 拒否 |
| 最初の点が計測姿勢から 3 deg 以内、エラー無しの position/torque 保持 | 拒否 | 拒否 |
| cancel・estop・ソフトキル・古いフィードバック検出 | 停止 | 停止 |

* `speed_only` を頼めるのは**アームリースの保持者だけ**（リース無しは 409、
  他人がリース中はリース拒否）。`play` は executor がリースを取るので該当する。
* デーモンは受理した軌道ごとに `arm_trajectory_guard`（guard と holder）を INFO で
  ログし、`GET /v1/arm/trajectory/{id}/status` の `guard` に記録する。
* 手首の連成リミット（J6 に応じた |J7| 上限）はデーモンではまだ検査していない
  （d1-firmware issue #99）。これを止めているのはキットの `check` だけなので、
  `--no-safety` は使わないこと。
* **デーモンが PR #102 より古い場合**（d1-2 は再デプロイまで `a9c8b0d2…` を配信）、
  そのドキュメントには `TrajectoryGuard` が無い。`play` は「このデーモンは
  guard=speed_only を提供していない」と警告し、`guard` を付けずに送る。つまり
  デーモンの `full` で再生され、HOME の 30.3 mm 問題のような干渉所見がある
  ジェスチャーは従来どおり拒否される。同梱クライアントは PR #102 のドキュメント
  （`1ec29096…`）から生成してあるが、`ensure.py` は接続時にデーモンが配信する
  ドキュメントから生成し直す。そのため、何を送れるかは常に接続先で決まる。

### ブレーキ解放の安全契約（record の既定）

* サーボ OFF・ブレーキ強制開放: **支えていない腕はその瞬間に重力で落ちる。**
* 解放する腕は、解放前から記録終了まで**必ず人が支える**。落下経路に手や顔を
  入れない。確認（`HOLDING`）はセッションに 1 回、教える腕すべてに対して。
* カウントダウン 3, 2, 1 の後にブレーキが開き、その瞬間から記録。停止（Enter）
  で**まずブレーキ締結**、その後に位置保持。
* 解放は時限式: デーモン自身が窓（既定 20 s、記録中は 1/3 ごとに再送）の
  終わりに締結する。このツールが落ちても窓で必ず締まる。
* 締結されるのは: 記録終了（このツール）、窓切れ、`brake_engage`、estop、
  ソフトキル、デーモン停止。
* 解放中はその腕への mode / recover / trajectory をデーモンが拒否する。
* ブレーキ状態はデーモンの指令記録でありセンサではない。キットのブレーキ経路は
  2026-09-23 時点で実機未検証。
* バンドルしたクライアントは d1-2 が配信する文書（spec `a9c8b0d2…`、PR #92
  入り）から生成済み。PR #92 より古いデーモンでは `OperationUnavailable` で
  止まる（その時は `--compliance` か `--no-brake`）。

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
  (limits + speed always; clearances only under `guard: full`).
* Teach metadata travels as `# mkit-teach: key=value` comments (name,
  sentiment, usage, source, `home_sha` = fingerprint of the HOME pinned,
  `min_clearance`, and `guard_advisory` when a guard margin was not met);
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
  as a real row and a HOME row is appended after it, lasting
  `max|q_last - HOME| / home_speed_deg_s` (20 deg/s < the 25 deg/s cap): the
  return is at the same joint speed whether it is long or short. The limiter
  may still stretch it (acceleration), never shorten it. Keyframe-mode takes
  time their last move the same way.
* A start/end already within `epsilon_deg` of HOME is snapped to HOME, as
  gesture_record did (the golden wave take is such a take and is unchanged
  apart from its new `min_clearance` line).

### What was ported, and the two deviations

`teach/process.py` ports `smoothSamples`, `reduceSamples`, `buildGesture`,
`limitJointDynamics` (gesture_csv.h) and `_trim_idle_keyframes` (teach.py)
with gesture_record's defaults. `teach/record.py` ports the capture sequence
(HOME first, compliance with `xAxisCompliance(2.0)` at 5 %, 1.5 s settle,
sampling, stationary auto-stop, hold on exit); brake release is the default
guide now, gesture_record's compliance is `--compliance`. Deviations, both because the
player changed:

1. Row 0 **is** HOME. gesture_record wrote the first kept sample as row 0
   (gesture_play prepended HOME); the firmware player overwrites row 0, which
   would silently drop that keyframe.
2. Dynamics are limited, and `check` samples, on the **daemon's** spline
   (`trajectory_points`), not gesture_play's `[HOME, kf0, …]` path.
   Cross-check: every one of omakase-core's 30 library CSVs passes `check`
   with peaks of exactly 25.0 deg/s and ≤ 120 deg/s², and `limit_joint_dynamics`
   is a no-op on all 30 (they were repaired to those caps).

Also: timestamps are real (the daemon read is not perfectly periodic), the raw
capture is kept raw, and Douglas–Peucker (`--method dp`) is offered beside the
greedy collinear walk.

### Daemon surface used (generated client only)

`GET /v1/arm/{side}/state`, `GET /v1/gripper/{side}/state`,
`GET /v1/arm/{side}/tool`, `POST /v1/arm/{side}/mode` (`ArmModeCommand`),
`POST /v1/arm/{side}/recover`, `POST /v1/arm/{side}/brake_release` /
`brake_engage`, `GET /v1/arm/{side}/brake` (PR #92 — generated operations in
the bundled snapshot since spec `a9c8b0d2…`; `OperationUnavailable` against an
older daemon), and — through `FirmwareExecutor` — the lease,
`/v1/arm/trajectory/*`. `FirmwareExecutor.play_waypoints` is the new public
entry for an already-timed trajectory (the plan path `_play` derives its own
timing).

### Known sharp edge: HOME sits on the body margin — now the daemon's call

At HOME the left arm's closest link is **30.3 mm** from `torso_belly`
against the guard's 30 mm margin, and only J2+ / J1− (A; mirrored on B) eat
into it (+2 deg on J2 → 28.2 mm). A Catmull-Rom that returns to HOME from an
outward J2 swing overshoots HOME by a fraction of a degree. The kit's `check`
now reports that as an advisory `guard (advisory) body … Link2_R …
torso_belly` warning and lets it pass. Resolved by d1-firmware PR #102 (issue
#101, Shu 2026-09-23): `play` uploads with `guard: speed_only`, so a daemon
with PR #102 does not refuse it on clearance. A daemon without it still
refuses the upload under its full guard (`arm_trajectory.rs::validate`), and
`play` says so before it moves.
