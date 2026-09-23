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
mkit-teach record                           # 名前を聞かれる → ~/teach/<name>.json。既定: 両腕, ブレーキ解放, 20 Hz
mkit-teach record --name wave --arms left   # 片腕だけ解放する（反対側は位置保持）
mkit-teach record --name wave --mode keyframe  # Enter ごとに 1 ポーズ、q+Enter で終了
mkit-teach record --name wave --compliance  # 旧 gesture_record の force_compliance で録る
mkit-teach record /path/take.json           # 保存先を直接指定（スクリプト向け）
```

**名前が最初**（Shu 2026-09-23）: 引数なしで起動すると、安全契約より前に
`Gesture name (a-z, 0-9, _, -):` と聞く。テイクは `<teach dir>/<name>.json`
（teach dir = `$MKIT_TEACH_DIR`、既定 `~/teach`）、CSV の既定は
`<teach dir>/<name>_motion.csv`。同名のテイクがあれば上書きか別名かを聞く
（端末でなければ拒否、`--yes` なら上書き）。名前は記録に残るので export で
`--name` は不要。

続けて**どちらの腕を教えるか**を聞く: `Arms to teach [both/left/right] (default
both):`（b / l / r も可）。`--arms` を付ければ聞かない。安全契約は選んだ腕だけを
名指しして表示する。選択は記録の `arms` に残る。

**録画中の Ctrl-C は Enter と同じ「停止」**で、テイクは保存される（ブレーキ締結 →
位置保持の順も同じ）。サンプルが 2 未満ならテイクは破棄され、録り直しの
コマンドを表示する。録画開始前（HOME 移動・カウントダウン中）の Ctrl-C は何も
保存しない。

`To fix:` が出るのは本当に直すものがある時だけ（位置保持に戻せなかった腕、
デーモンの拒否、キット側のタイムアウトなど）。**ヒントが `--yes` を勧めることは
無い**（`--yes` は HOLDING の確認を飛ばすスクリプト専用）。録り直しは
`mkit-teach record --name <name> --arms <arms>`。

**各コマンドは最後に「Next:」として次に打つコマンドを絶対パス入りで表示する**
（record → `export <take>`、export → `check <csv> --ascii` と `play <csv> --dry-run`、
check OK → `play --dry-run`、dry-run OK → `play`、play OK → `register` と次の
テイク）。失敗時は「To fix:」として直し方（recover の方法・再実行コマンド）を出す。

既定は**ブレーキ解放（ハンドガイド）**。Shu 2026-09-23:「コンプライアンスだと
動かすのが難しかった」ため、サーボ OFF + 保持ブレーキ解放で直接手で動かす方式を
既定にした。

流れ:

1. 下記「ブレーキ解放の安全契約」を表示し、**`HOLDING` と 1 回だけ**打つ
   （両腕でも 1 回。腕ごとには聞かない。`--yes` はスクリプト専用）。
2. アームリースを `operator` クラスで取る（テレオペ・コンソールと同格。
   `policy` の自律動作より優先）。
3. **入口の recover**: 位置モードでない腕（idle / error など。手で動かして
   idle のまま置いた腕を含む）があれば、その腕を**計測姿勢で** recover して
   位置保持にする（`[starting] recovering arm b (idle, 40.2 deg from its
   command)` と表示）。教えない側の腕も対象（位置モードと HOME 移動は両腕を
   動かすため）。これは teach の明示的な選択で、エージェント実行の経路では
   従来どおり拒否する。
4. 位置モードでまっすぐ HOME へ移動（記録しない）。**ジェスチャーは必ず HOME
   から始まる**: `--no-home-start` で移動を省いた場合、教える腕のどれかの関節が
   HOME から 2 deg を超えていれば開始を拒否する。
5. 柔らかくする（順序は下の「動作の順序」）:
   * 既定（ブレーキ解放）: `idle`（サーボ OFF）を要求し、**腕が idle と報告する
     まで待つ**（ブレーキは保持したままなので安全）→ **カウントダウン 3, 2, 1**
     （`--countdown N`、0 で無し）→ "0" で `brake_release`（確認語
     `RELEASE_BRAKE`、時限窓 20 s、記録中は窓の 1/3 ごとに再送）。
     **解放が受理された瞬間が t = 0**。そこから即記録開始（解放前の姿勢は記録
     しない）。idle の代わりに error と報告されたら故障として止める
     （ブレーキは解放しない）。
   * `--compliance`（旧 gesture_record と同じ）: カウントダウンの後に
     `force_compliance`（力方向 `[1,0,0,0,0,0]`、目標力 0、調整上限 2 mm、
     速度/加速度比 0.05）。モード遷移と静止を確認してから 1.5 秒待って記録開始。
     サーボは入ったまま、手で押すと逃げる。
     デーモンにツール（エンドエフェクタ）が登録されていないと拒否する
     （空フランジとして重力補償され手首が垂れるため。何も付けていない時だけ
     `--allow-bare-flange`）。
   * `--no-brake`: カウントダウンの後にサーボ OFF のみ（idle を確認）、
     ブレーキには触らない（idle で手で動かせる個体向け）。
6. 手で腕を動かす。**Enter** で停止（Ctrl-C でも停止し保存される）。
   `--duration-s N` で自動停止、`--stationary-s N` で N 秒静止したら停止、
   `--stop-file PATH` でファイルが現れたら停止。
7. 停止時は（例外時も必ず）**まずブレーキ締結**（他の何よりも先。再送スレッドは
   ロックで止めるので、締結の後に解放が飛ぶことはない）→ 腕が**静止し、同じ
   モードを 0.3 s 保つまで待つ** → `recover`（計測姿勢で位置保持、比 0.05 =
   旧 `lockCurrentPositionMode(5,5)`。position と報告されるまで確認）。
   recover はデーモン内で確認ループ（最大 500 ms × 8）を回すので、キットは
   文書の `x-timeout-seconds`（recover は 65 s）まで答えを待つ。
   コントローラが拒否したら 1 秒おきに計 3 回まで試す（キット側が待ちきれずに
   諦めた場合は、デーモンがまだ実行中かもしれないので再送しない）。
   メッセージは「デーモンが拒否した（DAEMON REFUSED）」と「キットが待つのを
   やめた（KIT GAVE UP、拒否ではない）」を区別する。それでも駄目なら腕は
   **idle・ブレーキ締結のまま（安全）**残し、`WARNING: arm b was not put back
   in a position hold …` と表示して記録の `meta.exit_problems` に残す。復帰は
   コンソールの Arms → Recover か `POST /v1/arm/b/recover`。→ リース返却 →
   テイク確定。

### 動作の順序（2026-09-23 d1-2 初回実機で確定）

```
lease → [入口 recover: idle/error の腕] → position（報告を確認）→ HOME
      → idle 要求 → idle と報告されるまで待つ（ブレーキ保持中＝安全）
      → 3, 2, 1 → brake_release（"0"＝t = 0）→ 記録
      → 停止: brake_engage → 静止＋モード安定 0.3 s → recover（最大 3 回, 1 s 間隔）
      → position を確認 → リース返却
```

初回（kit 3fad724）で起きたこと: `POST /v1/arm/{side}/mode` はデーモンが要求を
**受け付けた時点で**返る（0 ms）。コントローラが idle と報告したのは 11 ms 後で、
キットはその 5 ms 前に `brake_release` を送り、デーモンは「位置モードの腕は
解放しない」と 409 で拒否した。続く `recover` はブレーキ締結の 2 ms 後で、
モードが idle/error を往復する間にコントローラが `RESET1`（code 8）を拒否した。
コンソールの Brakes 画面で解放できたのは、その時点で腕がすでに idle だった
から（デーモンの規則は一貫しており、キットが競走していただけ）。もう 1 回は、
オペレーターが手で idle にして動かした腕（指令と計測が 40.2 deg ずれていた）で
`FirmwareExecutor.__enter__` が停止した。今は 3. の入口 recover で始められる。

2 回目（kit dc51dbe, 20:46Z）: 記録自体は成功（idle 確認 → 解放 20:46:13 →
5.44 s / 63 サンプル → Enter → brake_engage 20:46:19.037）。ところが recover
（20:46:19.363 受信）が 20:46:21.366 にデーモン側で `phase=cancelled`。ちょうど
2.0 s = クライアントの既定タイムアウトで接続を切ったため、デーモンが確認ループの
途中で取り消した。文書は recover に `x-timeout-seconds: 65` を宣言している。
今は全リクエストがルートごとの文書の値（下限は既定の 2 s）まで待つ。

記録ファイル（JSON）は生データのまま（平滑化も手首固定もしていない）なので、
設定を変えて何度でも 2 以降をやり直せる。グリッパーの閉度も記録されるが、
CSV には列が無い（omakaseos のプレーヤーは 15 列以外を拒否する）。

### 2. キーフレーム化して書き出す（export）

```sh
mkit-teach export ~/teach/wave.json          # → ~/teach/wave_motion.csv（絶対パスで表示）
mkit-teach export take.json out.csv --name wave --sentiment neutral --usage filler
```

出力先を省くと**テイクと同じディレクトリ**の `<name>_motion.csv`（名前は記録の
ものか `--name`）。sentiment / usage を省くと端末なら聞き、そうでなければ
neutral / filler。

旧パネルの Review & Save と同じ処理を順に行う:

| 手順 | 既定 | オプション |
|---|---|---|
| 開始時の「落ち込み」を捨てる（ブレーキ解放直後、最初の 0.5 s 以内で関節速度 8 deg/s を超えた最後のサンプルまで） | on | `--sag-max-s` / `--sag-vel` / `--no-sag-trim` |
| 手首 J5–J7 は**教えたまま残す**。記録中の可動範囲が 2 deg 未満の手首関節だけ（垂れ・ノイズ）HOME に固定し、その旨を表示する（Shu 2026-09-23: task4 の L7 35.7 deg が旧既定の固定で 0 になった） | off | `--pin-wrist`（旧 gesture_record の固定） |
| 平滑化（中央値→平均、ジッタ除去） | 5 サンプル | `--smooth-window N` / `--no-smooth` |
| HOME から始め、落ち込み後の最初の姿勢へ一定速度でつなぐ | テイク自身のピーク関節速度（平滑化後）を 20–90 deg/s に収めた値 | `--home-speed` |
| 最後の姿勢を残し、そこから HOME へ戻る区間を**一定の関節速度**で追加（時間 = 最大関節差 / HOME 速度。戻りがジェスチャーより遅く感じないように） | 同上（キーフレームモードは 20 deg/s） | `--home-speed` / `--no-home` |
| キーフレーム削減（直線から ε 以内を間引き） | 1.5 deg, collinear | `--epsilon-deg` / `--method dp` / `--min-spacing-s` |
| 停止区間の短縮（Trim idle pauses） | Off | `--max-idle-s 0.25/0.5/1` |
| 再生可能化（速度 150 deg/s・加速度 600 deg/s² 以下になるまで時間だけ延ばす。下記「速度」） | on | `--max-joint-vel` / `--max-joint-acc` / `--no-speed-limit`（教えたタイミングのまま。上限超えは check で NG） |
| 安全チェック（下記 check）→ 関節リミット・速度・時刻が NG なら書かない | — | `--force`（UNSAFE 印付きで保存、play は `--no-safety` が必要） |
| ガード（干渉）の所見は**警告のみ**。UNSAFE にはしない。最小クリアランスを `# mkit-teach: min_clearance=…`、マージン割れを `# mkit-teach: guard_advisory=…` として CSV に残す | — | — |

始点・終点が HOME から ε（1.5 deg）以内ならその姿勢を HOME にスナップする
（旧 gesture_record と同じ。区間を足さない）。

### 速度（SpeedPolicy）

**ジェスチャーの速度を制限しているのはキットのこの上限だけ**。デーモンは
1 ステップ 350 deg/s を上限にするだけで、omakaseos のプレーヤーは速度を一切
検査しない。上限は `process.SpeedPolicy` の 1 か所で、export（伸ばす）・check
（判定）・play（事前チェック）が同じものを使う。

* 既定 **150 deg/s・600 deg/s²**（Shu 2026-09-23 21:09Z「案 c」: 教えた速さで
  再生し、それより速い部分だけ上限まで伸ばす）。旧 gesture_record の 25 deg/s・
  120 deg/s² では d1-2 take2（J1 の振りが平滑化後で約 130 deg/s）が 3.83 s →
  5.52 s に伸び、目に見えて遅くなった。
* 伸ばしたときだけ export が `speed: stretched 3.83 s -> 5.52 s to meet … (J1
  peak 130 deg/s … recorded)` と表示する。加速度上限が効くことが多い
  （キーフレームの角で Catmull-Rom の加速度が大きくなるため）。
* 使った上限は CSV に `# mkit-teach: max_joint_vel=… max_joint_acc=…`
  （伸ばした場合は `speed_stretch=…` も）として残り、**check と play はその
  CSV 自身の上限で判定する**（`--max-joint-vel` / `--max-joint-acc` で上書き
  可）。キーが無い古い CSV は既定値で判定。
* HOME への接続・復帰は一定の関節速度で、既定はそのテイク自身のピーク関節速度
  （平滑化後）を 20–90 deg/s に収めた値（`--home-speed` で指定可）。
* export（と keyframes）は時間の内訳と関節ごとの可動範囲を必ず表示する:
  `timing: recorded 5.24 s -> body 5.10 s (speed cap stretched 3 knot(s),
  +0.20 s) + HOME connect 0.40 s + return 0.60 s (at 45 deg/s) = 6.35 s`、
  `joint range recorded -> exported [deg]: L1 40.0 -> 39.2, L7 35.7 -> 35.1, …`。
  固定した関節には `(pinned: …)`、半分未満に減った関節には `(LOST)` が付く。
* `[saved]` と `check` が表示する時間はどちらも**デーモンのスプラインの長さ**
  （0 行目の duration はプレーヤーが無視するので含めない）。

### gesture.yaml

`--name`（または記録の名前）があれば omakaseos の `gesture.yaml` 用エントリを表示する。
`--register <omakase-core>/robot_stack/robots/omakase/d1/gesture.yaml` で
そのファイルに追加/置換まで行う（CSV は同じ階層の `csv/` にコピーする）。

### 3. 確認する（check）

```sh
mkit-teach check wave_motion.csv --ascii
```

デーモンが実際に再生するスプライン（HOME から始まる Catmull-Rom）を 10 ms
刻みでたどる。

* **不合格（exit 1）になるもの**: 関節リミット（クリップせず違反扱い）、
  手首ロールの連成リミット（J6 に応じた |J7| 上限）、速度・加速度（その CSV
  の `max_joint_vel` / `max_joint_acc`、無ければ 150 deg/s・600 deg/s²）、
  時刻（有限・0 始まり・単調増加）と角度の有限性。
* **警告だけのもの（exit 0）**: MotionGuard の胴体・胸・両腕間・自己干渉。
  `WARNING: guard (advisory) body: closest -40.0 mm at t=2.60s — arm A link
  Link2_R within … of body box torso_belly …` のように、最も近づいた距離・
  フレーム名・時刻・マージン割れのサンプル数を出す。

ガードを警告に下げた理由: 教示はオペレーターが**手で腕を導いて実際に通った姿勢**
なので、ガードのカプセルモデルがその姿勢の可否を裁く立場にない（Shu
2026-09-23「teaching に関しては、ガードを外した方がいいかも」）。ただし
**デーモンは同じ違反で再生を拒否する**（干渉ガードは常に有効）。下の「ガード」節を参照。

`--ascii` は関節ごとの帯グラフ。動画で見たい時は
`python examples/preview_gesture.py wave_motion.csv`（mujoco）。

### 4. 再生する（play）

```sh
mkit-teach play wave_motion.csv --dry-run     # ロボットに触れず事前チェックだけ
mkit-teach play wave_motion.csv               # policy リース, 比はジェスチャーから自動
mkit-teach play wave_motion.csv --vel-ratio 0.5  # 比を固定する
```

UNSAFE 印・チェック NG（リミット/速度/時刻）・コントローラ異常のいずれでも何も
動かさない（`--no-safety` でキットの事前チェックを飛ばせるが、デーモンの検査は
飛ばせない）。キットのガード警告は**動かす前に表示**してから再生に進む
（「デーモンは同じ違反でアップロードを拒否する」旨も表示する）。HOME から
2 deg 以上離れていれば先に HOME へ移動し、再生後に HOME 到着（計測値）と静止を
確認する。record と同じく、位置モードでない腕（idle / error）は入口で計測姿勢に
recover してから始める（表示あり）。

### 再生の速度比（位置モードの vel_ratio）

位置モードのコントローラーは、デーモンの 1 ms ごとの目標を**最大
140 deg/s × vel_ratio** でしか追わない。旧既定 0.15 では約 21 deg/s で、d1-2
task6（CSV は正しく L7 70.6 deg・ピーク 117.8 deg/s）はゆっくり丸められて再生され、
J7 はほとんど動かなかった。**比が隠れたブレーキになってはいけない**。速度の
関門は CSV の SpeedPolicy だけ。

* `play` はジェスチャー本体の比を、再生するスプライン上のピーク関節速度から
  決める: `clamp(1.3 × peak / 140, 0.3, 1.0)`（118 deg/s → 1.0、旧 25 deg/s の
  CSV → 0.3）。加速度比はデーモンに物理的な尺度が無いので同じ値にする。
* 選んだ比と理由を動かす前に表示する（`playback ratio 1.00: gesture peak
  117.8 deg/s x 1.3 …; HOME approach at 0.30`）。
* ジェスチャー前の HOME への事前移動は教示動作ではないので**落ち着いた 0.3**。
  ジェスチャー内の HOME 接続・復帰区間はジェスチャーと同じ比で再生される。
* `--vel-ratio` を付けると事前移動・本体とも、その値に固定する。
* record の HOME 移動は従来どおり 0.15。

### ガード: デーモンの干渉検査は常に有効

Shu 2026-09-23 21:14Z: 干渉ガードはどこでも常に有効。`mkit-teach play` は
ジェスチャーを `POST /v1/arm/trajectory/start` に **`guard` フィールド無しで**
送り、再生だけ干渉検査を緩める経路は無い（デーモン API からも削除予定:
d1-firmware PR #106 とその後続）。

* デーモンはアップロード時と再生中の 1 ms ごとに、**実形状に近いジオメトリ +
  20 mm マージン**で干渉を検査し、関節リミット・350 deg/s・時刻・最初の点
  （計測から 3 deg 以内）も検査する。違反があれば拒否 / 停止する。
* キットの `check` の干渉所見は**警告（advisory）**のままだが、デーモンは同じ
  違反を拒否する。警告が出たジェスチャーは、再生前に教え直すか HOME 付近の
  姿勢を見直すこと。
* 手首の連成リミット（J6 に応じた |J7| 上限）はデーモンではまだ検査していない
  （d1-firmware issue #99）。これを止めているのはキットの `check` だけなので、
  `--no-safety` は使わないこと。
* 同梱クライアントのスナップショット（`1ec29096…`）には旧 `guard` フィールドが
  まだ残っているが、キットは使わない。`ensure.py` は接続時にデーモンが配信する
  ドキュメントから生成し直す。

### ブレーキ解放の安全契約（record の既定）

* サーボ OFF・ブレーキ強制開放: **支えていない腕はその瞬間に重力で落ちる。**
* 解放する腕は、解放前から記録終了まで**必ず人が支える**。落下経路に手や顔を
  入れない。確認（`HOLDING`）はセッションに 1 回、教える腕すべてに対して。
* サーボ OFF（idle と報告されるまで確認）→ カウントダウン 3, 2, 1 → ブレーキ
  が開き、その瞬間から記録。停止（Enter）で**まずブレーキ締結**、腕が静止して
  から位置保持（recover）。recover できなければ idle・ブレーキ締結のまま残す。
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
  `max|q_last - HOME| / home_speed_deg_s` (20 deg/s, well under the ceiling): the
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
torso_belly` warning and lets it pass. The daemon decides: its clearance
guard is always on (Shu 2026-09-23 21:14Z; the per-job relaxation of
d1-firmware PR #102 is being removed), with its realistic geometry and 20 mm
margin, so whether a gesture that grazes HOME plays is the daemon's call, and
`play` says before it moves that the daemon will refuse such violations.
