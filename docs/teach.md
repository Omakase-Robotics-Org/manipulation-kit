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
mkit-teach record take.json                 # 既定: 両腕, compliance, 20 Hz
mkit-teach record take.json --arms left     # 片腕だけ柔らかくする（反対側は位置保持）
mkit-teach record take.json --mode keyframe # Enter ごとに 1 ポーズ、q+Enter で終了
```

流れ（旧パネルの idle → recording (compliance) → recorded と同じ）:

1. アームリースを `operator` クラスで取る（テレオペ・コンソールと同格。
   `policy` の自律動作より優先）。
2. 位置モードでまっすぐ HOME へ移動（記録しない。`--no-home-start` で省略）。
3. 柔らかくする — `--guide` で選ぶ:
   * `compliance`（既定・gesture_record と同じ）: `force_compliance`
     （力方向 `[1,0,0,0,0,0]`、目標力 0、調整上限 2 mm、速度/加速度比 0.05）。
     サーボは入ったまま、手で押すと逃げる。入ってから 1.5 秒待ってから記録開始。
     デーモンにツール（エンドエフェクタ）が登録されていないと拒否する
     （空フランジとして重力補償され手首が垂れるため。何も付けていない時だけ
     `--allow-bare-flange`）。
   * `--hand-guide`（= `--guide brake`）: サーボ OFF + **保持ブレーキ解放**。
     下記の安全契約を表示し、`HOLDING` と打つまで何もしない。
   * `--no-brake`（= `--guide idle`）: サーボ OFF のみ、ブレーキには触らない
     （idle で手で動かせる個体向け）。
4. 手で腕を動かす。**Enter** で停止（Ctrl-C でも停止し保存される）。
   `--duration-s N` で自動停止、`--stationary-s N` で N 秒静止したら停止、
   `--stop-file PATH` でファイルが現れたら停止。
5. 終了時は必ず（例外時も）: ブレーキ締結（brake のみ）→
   `recover`（計測姿勢で位置保持、比 0.05 = 旧 `lockCurrentPositionMode(5,5)`）
   → リース返却。

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
| 手首 J5–J7 を HOME に固定（重力で垂れた手首を記録しない） | on | `--free-wrist` |
| 平滑化（中央値→平均、ジッタ除去） | 5 サンプル | `--smooth-window N` / `--no-smooth` |
| 最初と最後を HOME に固定 | on | `--no-home`（プレーヤーはどのみち HOME に置き換える） |
| キーフレーム削減（直線から ε 以内を間引き） | 1.5 deg, collinear | `--epsilon-deg` / `--method dp` / `--min-spacing-s` |
| 停止区間の短縮（Trim idle pauses） | Off | `--max-idle-s 0.25/0.5/1` |
| 再生可能化（速度 25 deg/s・加速度 120 deg/s² 以下になるまで時間だけ延ばす） | on | `--max-joint-vel` / `--max-joint-acc` / `--no-speed-limit` |
| 安全チェック（下記 check）→ NG なら書かない | — | `--force`（UNSAFE 印付きで保存、play は `--no-safety` が必要） |

`--name` を付けると omakaseos の `gesture.yaml` 用エントリを表示する。
`--register <omakase-core>/robot_stack/robots/omakase/d1/gesture.yaml` で
そのファイルに追加/置換まで行う（CSV は同じ階層の `csv/` にコピーする）。

### 3. 確認する（check）

```sh
mkit-teach check wave_motion.csv --ascii
```

デーモンが実際に再生するスプライン（HOME から始まる Catmull-Rom）を 10 ms
刻みでたどり、関節リミット（クリップせず違反扱い）、手首ロールの連成リミット
（J6 に応じた |J7| 上限）、胴体・胸・両腕間・自己干渉（デーモンと同じ
MotionGuard）、速度・加速度をチェックする。`--ascii` は関節ごとの帯グラフ。
動画で見たい時は `python examples/preview_gesture.py wave_motion.csv`（mujoco）。

### 4. 再生する（play）

```sh
mkit-teach play wave_motion.csv --dry-run     # ロボットに触れず事前チェックだけ
mkit-teach play wave_motion.csv               # policy リース, vel_ratio 0.15
```

UNSAFE 印・チェック NG・コントローラ異常のいずれでも何も動かさない。HOME から
2 deg 以上離れていれば先に HOME へ移動し、再生後に HOME 到着（計測値）と静止を
確認する。

### ブレーキ解放の安全契約（`--hand-guide`）

* サーボ OFF・ブレーキ強制開放: **支えていない腕はその瞬間に重力で落ちる。**
* 解放する腕は、解放前から記録終了まで**必ず人が支える**。落下経路に手や顔を
  入れない。
* 解放は時限式: デーモン自身が窓（既定 20 s、記録中は 1/3 ごとに再送）の
  終わりに締結する。このツールが落ちても窓で必ず締まる。
* 締結されるのは: 記録終了（このツール）、窓切れ、`brake_engage`、estop、
  ソフトキル、デーモン停止。
* 解放中はその腕への mode / recover / trajectory をデーモンが拒否する。
* ブレーキ状態はデーモンの指令記録でありセンサではない。2026-09-23 時点で
  実機未検証（d1-firmware PR #92 自身の注記）。
* 実行にはデーモンの OpenAPI 文書に `brake_release` が載っている必要がある
  （0.3.0 のバンドル文書には無い）。無ければ `OperationUnavailable` で止まる。

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
  knots as uniform Catmull-Rom on a linear time base and guards every 1 ms.
* Teach metadata travels as `# mkit-teach: key=value` comments (name,
  sentiment, usage, source, `home_sha` = fingerprint of the HOME pinned);
  a force-saved file carries `# mkit-teach: UNSAFE=<violation>` lines.

### Where the old tool went

`gesture_record` (d1-sdk `devices/omakase_arm/example/gesture_record.cpp`)
opened the arm controller over UDP itself; omakase-core's `/d1_teach` panel
(`status_server/d1/teach.py`, `static/d1_teach.html`) spawned it. The firmwared
migration (omakase-core 2050ca49 / faa32a71 / fda96c19, 2026-09-08..10)
ported **playback** to daemon trajectories and left teach on the subprocess:
the panel still renders, but `gesture_record` cannot connect while
`d1-firmwared` owns UDP 4730 (d1-sdk PR #103 makes it refuse, exit 3). Nothing
was deleted; it simply stopped being able to reach the arm.

### What was ported, and the two deviations

`teach/process.py` ports `smoothSamples`, `reduceSamples`, `buildGesture`,
`limitJointDynamics` (gesture_csv.h) and `_trim_idle_keyframes` (teach.py)
with gesture_record's defaults. `teach/record.py` ports the capture sequence
(HOME first, compliance with `xAxisCompliance(2.0)` at 5 %, 1.5 s settle,
sampling, stationary auto-stop, hold on exit). Deviations, both because the
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
`brake_engage`, `GET /v1/arm/{side}/brake` (PR #92; `OperationUnavailable`
when the document lacks them), and — through `FirmwareExecutor` — the lease,
`/v1/arm/trajectory/*`. `FirmwareExecutor.play_waypoints` is the new public
entry for an already-timed trajectory (the plan path `_play` derives its own
timing).

### Known sharp edge: HOME sits on the body margin

At HOME the left arm's closest link is **30.3 mm** from `torso_belly`
against the guard's 30 mm margin, and only J2+ / J1− (A; mirrored on B) eat
into it (+2 deg on J2 → 28.2 mm). A Catmull-Rom that returns to HOME from an
outward J2 swing overshoots HOME by a fraction of a degree — enough for
`check` (and the daemon, which uses the same model and margin) to refuse the
take. If a teach fails near its start or end with `Link2_R … torso_belly`,
that is this; it is a HOME/margin question for the robot profile, not
something the teach tool should paper over.
