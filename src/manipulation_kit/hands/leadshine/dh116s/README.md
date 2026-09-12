# DH116S — Leadshine (Leadtron) dexterous hand

Pure-python CANFD driver for the Leadshine DH116S 11-DoF / 6-active-axis
dexterous hand. No ROS, no vendor `.so` — the wire protocol is implemented
directly over SocketCAN-FD (`python-can`).

Protocol source: Leadshine public Feishu wiki (灵巧驱控产品公开资料库),
"Dexterous Drive & Control & DH116(S) Series Dexterous Hand CANFD
Communication Protocol". A mined offline copy (protocol text, pin/bitfield
screenshots, coupling tables, SDK archives) lives in the bring-up notes
directory referenced by the PR that added this driver.

**Status: written from the vendor protocol doc, verified against a
protocol-faithful mocked transport only — not yet run against hardware.**
The hardware bring-up checklist is at the bottom.

## Layout

| file | what |
|---|---|
| `driver.py` | frame encode/decode + `Dh116sDriver` (enable/home/move/stream/SDO) + `SimulatedHand` |
| `transport.py` | `Bus` seam: `RealBus` (python-can SocketCAN-FD) / `MockBus` / `DryRunBus` |
| `coupling.py` | vendor passive↔active lookup tables (linkage→knuckle→fingertip per finger) |
| `data/coupling_*.csv` | bundled tables extracted from the vendor xlsx (`tools/extract_coupling.py`) |
| `descriptions/` | vendor MJCF + decimated meshes — single source of truth for sim (see its README for provenance) |
| `scripts/wiggle.py` | bench CLI: enable → home → per-finger sweep → 50 Hz stream print |
| `tests/` | `pytest tests/` — 47 tests, all on `MockBus` |

## Hand summary

- 11 DoF, 6 active axes (axis order used everywhere in this driver):
  1. thumb lateral swing (0–91°), 2. thumb flexion (0–59°), 3. index (0–72°),
  4. middle (0–72°), 5. ring (0–72°), 6. pinky (0–74°)
- Encoders on every active axis → position/velocity/current feedback on the
  base model (no tactile needed for joint-state recording).
- Wire units: position 0–10000 (fraction of full stroke), velocity
  10/stroke-coefficient per s (20000 ≈ full stroke in 0.5 s), current ‰ of
  rated (0–3000).

### Model naming rules (order the right variant!)

`DH 11 6 S EC – L 1 – S` = DH + total-DoF + active-DoA + form (blank=Universal,
`S`=Lite, `H`=Force-Enhanced) + bus (blank=**CANFD default**, `EC`=EtherCAT,
`RS`=RS485) + L/R + generation (1=Gen-1 MP) + sensors (blank=base,
`-S`=standard tactile, `-M`=multimodal). So **DH116S-L1 = CANFD, left, no
tactile**. The bus variant is factory-set — CANFD and RS485 share pins 3/4
but you must choose at ordering time.

**RS485 note:** there is NO public byte-level RS485 protocol doc; RS485 is
supported only through the vendor LHandProLib SDK. Order CANFD.

## Wiring & power

6-pin connector (see `02_pin_definitions.jpeg` in the bring-up notes):

| pin | signal |
|---|---|
| 1, 2 | VDC in, 12 V (−10 %) – 48 V (+10 %) |
| 3 | CANH (or 485+ on RS485 models) |
| 4 | CANL (or 485−) |
| 5, 6 | GND |

Static 0.1 A, max 2.0 A → a 24 V / 2.5 A bench supply is plenty. Check
polarity twice before first power-on. 120 Ω bus termination if the hand is
not internally terminated (verify on hardware — open question).

## SocketCAN-FD adapter setup

Any SocketCAN CANFD adapter works (PEAK PCAN-USB FD, candleLight/canable2
with FD firmware, …). Bitrates: arbitration 1 Mbps (80 % sample point), data
5 Mbps (75 % sample point).

```sh
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
ip -details link show can0        # verify FD + bitrates
candump can0                      # leave running in a second terminal
```

## Driver usage

```python
from manipulation_kit.hands.leadshine.dh116s import Dh116sDriver

drv = Dh116sDriver.connect(channel="can0", node_id=1)  # default node id = 1
print(drv.identify())        # SDO: model/handedness/versions sanity check
drv.enable()                 # 0x81 (0x20, 0x01) x6
drv.home()                   # 0x81 (0x25, 0x04) x6, polls until homed bit set
drv.set_positions([1500] * 6)               # 15 % stroke, 0x8B pos+vel+cur
drv.set_positions([0.5] * 6, unit="frac")   # or fractions / degrees

# 50 Hz autonomous feedback stream (0x00 config → 0xDB push, no polling):
for fb in drv.start_feedback_stream(hz=50):
    print(fb.pos, fb.vel, fb.cur, fb.faults)

drv.read_torques()           # 0xDC poll: signed torque, 0.01 mN·m
drv.close()
```

Error handling: every 0xDB/0xDC frame carries per-axis status + error code;
`Feedback.faults` gives human-readable non-zero errors (`driver.ERROR_CODES`),
`drv.clear_alarm()` sends control code 3. Malformed frames are dropped and
counted (`drv.decode_errors`), never fatal.

### Coupling tables (sim / URDF mimic joints)

```python
from manipulation_kit.hands.leadshine import dh116s
from manipulation_kit.hands.leadshine.dh116s import coupling
coupling.linkage_to_knuckle("index", 45.0)    # active drive angle → knuckle deg
coupling.linkage_to_fingertip("index", 45.0)  # composed through the linkage
```

Vendor tables are strictly monotonic; inverses are exact. The thumb ships
only knuckle→fingertip. MJCF for sim: bundled under `descriptions/`
(`dh116s.description_path()` resolves it from an installed package);
upstream is `sorrowfeng/leadtron_hand_descriptions` (DH116S-L000-A1 /
R000-A1) — only the decimated right-hand set is vendored here.

## Bench bring-up checklist (first power-on)

1. 24 V ≥ 2.5 A on pins 1/2 + 5/6, polarity checked. CANFD adapter on 3/4,
   termination checked.
2. `ip link` setup as above; `candump can0` running.
3. `python3 scripts/wiggle.py` — **dry-run**, eyeball the frames.
4. Identity: `python3 scripts/wiggle.py --execute --skip-sweep
   --stream-seconds 0` → expect `DH116S CANFD` + correct handedness. If SDO
   times out, wrong node id (default 1) or bus problem.
5. Enable + home with fingers free (torque homing, no hard stop needed).
6. `python3 scripts/wiggle.py --execute --finger index_flex` — one finger,
   30 % stroke, 30 % current (stoppable by hand).
7. Full sweep + stream: `python3 scripts/wiggle.py --execute` — verify
   positions track commands and 50 Hz effective rate.
8. Verifier for the driver on hardware: 60 s sinusoid script, commanded vs
   measured position RMS < 2 % of stroke at 50 Hz.

## Tests

```sh
cd hands/leadshine/dh116s
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

All tests run on the mocked transport: byte-exact golden frames from the
vendor protocol examples, enable→home→move sequences, stream reassembly,
error-bitfield surfacing, garbled-frame robustness, coupling known rows.

## Open hardware questions

- Internal 120 Ω termination? (measure)
- Max 0x8B command rate (protocol suggests 1 kHz internal loop; unstated)
- Whether `stop_feedback_stream()` (empty 0x00 config) actually clears the
  autonomous stream — only the set path is documented.
- Manual ROM (`ACTIVE_ROM_DEG`) vs coupling-table linkage ranges differ by a
  few degrees per finger — calibrate `unit="deg"` scaling on hardware.
