# O30 — LinkerBot LinkerHand O30 dexterous hand

Pure-python CANFD driver for the LinkerHand O30, a 20-DoF / 20-joint
direct-drive five-finger hand. No ROS, no vendor `.so` — the HOP wire protocol
is implemented directly over SocketCAN-FD (`python-can`) or the D1 arm's
end-module CAN passthrough.

**Status: written from the vendor protocol specification and verified against
a protocol-faithful mocked transport only — not yet run against hardware.**
The bench bring-up procedure is at the bottom.

## Sources

| source | what it is |
|---|---|
| `HandProtocol_v1.0.pdf` (protocol version **0.0.3**, exported 2026-06-09) | **primary.** The vendor specification, shipped inside `Linker_Hand_O30_ROS2_SDK` next to the control source. Every constant, sub-index table and golden frame in `driver.py` and `tests/test_frames.py` comes from it. |
| `linker_hand_o30_control.py` (same SDK; public mirror: [`linker-bot/linkerhand-o30-ros2`](https://github.com/linker-bot/linkerhand-o30-ros2)) | the vendor's working implementation, written against a **0.0.2** firmware. Where it disagrees with the spec, both readings are recorded — see [Where the sources disagree](#where-the-sources-disagree). |
| Official product manual (Chinese spec sheet, 2026-08) | mechanical/electrical ratings quoted below. |

## Layout

| file | what |
|---|---|
| `driver.py` | frame encode/decode + `O30Driver` (identify/enable/move/read/config) + `SimulatedHand` |
| `transport.py` | `Bus` seam: `RealBus` (SocketCAN-FD) / `MockBus` / `DryRunBus` / `ArmPassthroughBus` (D1 arm end module) |
| `toolconfig.py` | mass / COM / TCP registration for the arm controller — **provisional, only the mass is measured**; says so at runtime via `ToolConfig.provisional` / `.caveat()`, because mounting this hand moves the registered tool point from the DH116S's 100 mm to the flange origin |
| `retarget.py` | deliberate `NotImplementedError` stub: the glove→20-joint map needs hardware calibration first |
| `scripts/smoke.py` | bench CLI: `info` (read-only), `move` (one slow open/close), `cycle` (repeated grip/release), `janken` (rock/scissors/paper, a DEEP grip on root1+root2+tip), `sweep` (one horizontal joint at a time around its own neutral), `--groups` to pick which joint groups are addressed at all; dry-run by default; SocketCAN FD or CAN 2.0 (`--can-classic`), or the arm passthrough |
| `tests/` | `pytest hands/linkerbot/o30/tests/` — all on `MockBus`, no hardware |

## Protocol summary (HOP v0.0.3)

- **11-bit standard frames.** Request id = the hand's configured `frame_id`
  (spec default `0x001`; the vendor SDK's convention is right `0x01`, left
  `0x02`), reply id = `frame_id | 0x400`, valid range `0x001`–`0x3FE`. A 29-bit
  extended mode also exists (default `0x0000001`, reply `| 0x10000000`); this
  driver speaks standard frames only.
- **Payload = 3-byte header + data**: `byte0 = RTS(bit7) | MI(bit6-0)`,
  `byte1 = SI`, `byte2 = EDL`, `byte3+ = data`. MI selects the register
  (object), SI is a byte offset inside it, EDL is how many bytes matter.
  Little-endian, LSB0, UTF-8 strings.
- **Read vs write is implicit**: no payload = read, payload = write. A frame
  padded up to a legal CANFD length is still a write — the hand takes the
  first EDL bytes.
- **RTS** picks which value a read returns: `0` = live measurement, `1` = the
  last value written. This is the only way to confirm a write, because…
- **…writes are not acked.** Contrast the DH116S, where every write returns a
  3-byte state frame the driver blocks on.
- **Replies echo the header**, then the payload: `41 69 08` →
  `41 69 08 | "HOP"`. A read whose object has fewer bytes left than EDL
  returns what is there *without* an error; an out-of-range SI returns an
  error frame (`MI 0x4F`, payload[0] = error code, 16 records kept).
- **Single frame ≤ 64 bytes** (61 payload). Longer responses are
  auto-fragmented with increasing SI; every request this driver sends fits in
  one frame, so it never reassembles.

### Main indices used here

| MI | register | this driver |
|---|---|---|
| `0x00` | joint map (payload byte → slot) | read only, to verify the factory default |
| `0x01` | position, u8 vector | `set_positions` / `read_positions` |
| `0x02` | velocity | `set_velocities` / `read_velocities` |
| `0x04` / `0x07` | current / temperature | `read_currents` / `read_temperatures` |
| `0x08` | move time (10 ms per count) | `set_move_time` |
| `0x0C` | joint enable bitmask | `enable` / `disable` / `read_joint_enable` |
| `0x0D` | joint capability/fault mask | `read_valid_joints` |
| `0x20` | position, u16 vector | **not used** — reads back all zero on the 0.0.2 firmware these hands run (the vendor found the same). Constants and packers are kept for a later firmware |
| `0x30` | sparse `(joint, position)` pairs | `set_sparse_positions` — **numbering unresolved, see below** |
| `0x35` | configuration | control mode, frame id; flash/reset writes are fenced |
| `0x41` | product info (read-only) | `identify` / `dump_product_info` |
| `0x42` | engineering service | hand side, heartbeat |
| `0x4F` | last communication error (read-only) | `read_error_code` |

### Joint layout — 20 motors in a 36-slot object

Every single-byte joint object is **36 bytes**. Slots are grouped **by joint
type**, five per type, finger order thumb → index → middle → ring → little:

```
0x00-0x04 roll   0x05-0x09 yaw    0x0A-0x0E root1
0x0F-0x13 root2  0x14-0x18 tip    0x19-0x1B wrist
0x1C-0x1E palm   0x1F-0x23 reserved
```

The O30 fills 20 of the hand slots — roll exists on the thumb only, root2 has
no thumb — so the five holes (`0x01`-`0x04`, `0x0F`) are written as zero and
ignored by the hardware. `AXIS_NAMES` is that 20-value order and every vector
in the driver uses it. Commands and feedback are **0-255 per joint**;
velocity likewise.

A per-payload remap (`MI 0x00`) can change which slot each payload byte hits.
It is disabled from the factory (`map[k] = k`) and this driver never enables
it — but it **persists across power cycles**, so `read_joint_map()` /
`assert_default_joint_map()` exist to check a hand whose history you do not
know. `smoke.py info` reports it.

### Where the sources disagree

1. **Two joint numberings.** The spec's 「逻辑关节子索引表」 (p15) numbers the
   36 joints *finger-major* (thumb's five at `0x00`-`0x04`, index's at
   `0x05`-`0x09`, …). Its own memory-layout table, the `MI 0x00` mapping
   chapter ("map[k] takes the values of the single-byte vector memory layout,
   e.g. `0x00` thumb roll, `0x05` thumb yaw, `0x14` thumb tip") and every
   `MI 0x01` example (`01 14 05 C8 C8 C8 C8 C8` = five *fingertips*) use the
   *type-major* layout above. **This driver implements type-major**, which the
   vendor SDK also does. The finger-major table is kept as
   `LOGICAL_JOINT_NUMBERS` so the conflict lives in code rather than in
   someone's memory.
2. **Sparse (`MI 0x30`) numbering — settled in favour of the sub-index.**
   The spec's example `30 00 04 04 C8 09 80` labels `0x04` as the thumb tip,
   which is its finger-major table. The vendor's implementation — the one
   that has run on hardware — addresses sparse joints by the same type-major
   sub-index as every other register (`[(0x0A, 200), …]` = all five root1),
   and so does this driver. Read the spec example as a documentation slip.
3. **Product-info offsets moved.** In 0.0.3 the device UID grew from 32 to 48
   bytes, shifting every field from `protocol_name` on by `0x10`. **The hands
   delivered on D1 run 0.0.2**, so that table is tried FIRST; both ship here
   and the driver probes for `"HOP"` to decide (`detect_product_layout`). If
   neither matches, `dump_product_info()` prints the raw object.

4. **The joint probe `0D 00 24` errors on this firmware** (the vendor
   disabled it in their own SDK for the same reason). The joint set is the
   static 20-entry table; `read_valid_joints()` is a diagnostic for other
   models only, and nothing depends on it.

## Differences from the DH116S

| | Leadshine DH116S | LinkerBot O30 |
|---|---|---|
| protocol | vendor CANFD protocol (CANopen-flavoured COB-IDs + SDO) | HOP v0.0.3 (MI/SI/EDL register access) |
| request id | `0x500 + node` (SDO `0x600 + node`) | the hand's stored `frame_id` (default `0x001`; 1 = right, 2 = left by convention) |
| reply id | `0x480 + node` (SDO `0x580 + node`) | `frame_id | 0x400` |
| id collisions | — | none: O30 ids ≤ `0x3FE`, replies ≤ `0x7FE`, so `0x481`/`0x501`/`0x581`/`0x601` stay free — the two hands can share a bus |
| bitrate | arbitration 1 Mbps, data 5 Mbps, **BRS on** | arbitration 1 Mbps; **BRS off** (factory default, and BRS + fast data phase drives this hand's bus to BUS-OFF) |
| axes | 6 active (11 DoF, coupled) | 20 independent joints |
| command unit | 0-10000 stroke fraction (+ velocity, current, gesture) | 0-255 per joint (velocity 0-255, move time 10 ms/count) |
| feedback | one 62-byte `0xDB` frame: position + velocity + current + per-axis status/error | one register per quantity; 2-4 reads per sample. Errors are one device-level code (`MI 0x4F`), not per joint |
| streaming | 50 Hz poll thread (firmware ignores the autonomous-push config) | poll thread, ≤ 500 Hz interface budget shared with commands |
| writes | acked with a state frame; driver blocks and raises `AckError` | unacked; driver verifies by reading back |
| homing | torque homing, driver polls the homed bit | none — absolutely referenced joints. `home()` commands the open pose (root1 + tip only) |
| power-on | ready when enabled | ~5 s self-check during which **the fingers move by themselves** and writes may be ignored |
| retarget | 6-axis map migrated from d1-vr-teleop | not written yet (explicit `NotImplementedError`) |
| tests | 47, script-style imports (`import driver`) | package imports (`manipulation_kit.hands.linkerbot.o30.driver`) — the top-level name `driver` is already claimed by the DH116S tests in the same pytest process |

## Wiring & power

Per the official spec sheet:

| item | value |
|---|---|
| supply | **wire for 24 V.** The official manual says DC 24-48 V; the device's own product-info string reports `12V~24V`. 24 V is the only value both agree on, and it is what the DH116S bench supply already provides |
| current | 0.4 A idle, 1.1 A average unloaded (at 24 V) |
| **peak power** | **150 W ≈ 6.3 A at 24 V** |
| interface | CAN / CAN FD, 500 Hz |
| performance | repeatability 0.2 mm, 0.8 s to open/close, fingertip force 24 N (thumb) / 30 N (others), 70 N five-finger grip |
| tactile | 6×11 array, 9.6 × 16.47 mm active area, 50 g trigger, 20 N/cm², 200 FPS |
| envelope / mass | 200 × 92 × 47 mm, 730 g (bare hand; 750 g weighed with mounting hardware) |

⚠️ **The DH116S peaked at 2.0 A; the O30 peaks at ~6.3 A.** Check the supply
rating and the wiring gauge along the whole path (including the arm's
end-module harness if the hand is fed through it) before the first grip. The
connector pinout is not documented here — confirm it against the hand's own
label before powering anything.

## SocketCAN-FD setup

```sh
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
ip -details link show can0
candump can0
```

The arbitration rate must be 1 Mbps (the hand's factory default). The
`dbitrate` value is only cosmetic here: the hand ships with **BRS disabled**
(`MI 0x35 / SI 0x07 = 0`), and this driver sends `bitrate_switch=False`, so
the data phase runs at the arbitration rate. Do not "fix" that — the vendor
records BUS-OFF with BRS on.

## Driver usage

```python
from manipulation_kit.hands import get_hand

hand = get_hand("linkerbot/o30", channel="can0", node_id=1, execute=True)
print(hand.identify())            # model / protocol / hand side / frame id
hand.startup()                    # THE VENDOR'S POWER-ON SEQUENCE — fingers move
hand.set_positions([...])         # 0-255 per joint, AXIS_NAMES order
print(hand.read_feedback().pos)
hand.close()
```

`startup()` is the sequence hardware confirmed on 2026-08-13: wait out the
~5 s self-check → position mode + joint enable → write the velocity vector →
command `INIT_POSE`. `enable()` alone does the mode and the enable.

The enable comes before the pose deliberately. A position written to a
disabled hand moves nothing — but the hand STORES it, so the fingers snap to
it later, whenever the enable happens to land. That is real motion at a
moment nobody chose, and it is what moved a hand here once. Enabling first
makes the pose command the first movement.

Two things about the units that read wrong if you assume:

- **velocity 0 is not a stop.** The 0-255 wire value maps onto motor units
  8000-12000 (`velocity_motor_units`), so a joint commanded at velocity 0
  still moves, at about two thirds of full speed. Hold a joint by commanding
  its position, never by zeroing its velocity.
- **`INIT_POSE` is the reference, not zero.** The bending joints (root1,
  tip) rest around 20-37, curl towards ~200 and straighten towards 0. For
  roll, yaw and root2 neither end is calibrated, which is why `home()` and
  the smoke CLI move only the bending joints.

`execute=False` (the default) runs the same driver against `SimulatedHand`
over a `MockBus`: every frame is built and parsed, nothing reaches a wire.

### A command never waits behind a read

Transactions are serialized — a reply carries no sequence number, only the
echoed MI, so two requests in flight could take each other's answers. A read
therefore holds the bus for its whole reply wait, and on this hardware a reply
**does** go missing (single reply slot in the end module). That put a full
`REQUEST_TIMEOUT` in front of whatever came next: measured, one command frame
waited **582 ms** at `REQUEST_TIMEOUT = 0.6`, twice that on the passthrough
where the read is retried. A 50 Hz stream cannot absorb that, and neither can
an operator's STOP, which is also just a write.

So a read gives the bus **up** when a motion command is waiting for it: it
raises `BusYielded` (an `O30Error`, so existing pollers already catch it) and
the command goes out within `COMMAND_PRIORITY_SLICE` (5 ms, measured 1 ms).
`BusYielded` means "sample skipped, nothing is wrong with the hand" — a
feedback poller should drop it and wait for its next tick. The trade is
deliberate: under a live stream some polls are lost, and a stalled command path
is the worse failure of the two. Set `drv.reads_yield_to_commands = False` for a
bench session where the reads ARE the job and no stream is running.

### Asking whether this package is new enough

`manipulation_kit.hands` is installed **editable** on the robots, so which code runs is
decided by which branch that checkout is on — and the failures from an old one
do not look like version skew, they look like broken hardware. `__version__`
does not move when a branch does, so the thing to check is the capability set:

```python
from manipulation_kit.hands.linkerbot import o30

REQUIRED = {"arm-passthrough", "enable-verify-setpoint", "command-priority"}
missing = REQUIRED - getattr(o30, "FEATURES", frozenset())
if missing:
    raise ImportError(f"this manipulation_kit.hands checkout lacks {sorted(missing)}")
```

Tokens are added when a fix changes what a consumer may assume, and are never
renamed or removed — the point is that an old consumer can keep asking.

## Bench bring-up

1. Power the hand (24 V, ≥ 7 A headroom) and **wait ~5 s** for the self-check
   — the fingers move on their own. Keep hands and objects clear.
2. Bring `can0` up as above; leave `candump can0` running in another terminal.
3. `python3 scripts/smoke.py info` — dry run, eyeball the frames.
4. `python3 scripts/smoke.py info --execute --channel can0 --node 1`
   Expect model `O30`, protocol `HOP` and the right hand side. The joint-map
   probe (MI 0x00) times out on the firmware these hands run and MI 0x4F then
   reports `0x09` — the register is not implemented, the same way the MI 0x0D
   capability probe is not, so the CLI says so and assumes the factory
   default. If nothing answers at all, the frame id is the first suspect: it is a
   stored value, not a wiring fact. Try `--node 2`, then probe the broadcast
   id (`DISCOVERY_STD_ID`, `0x7FF`) with `candump`/`cansend`.
5. `python3 scripts/smoke.py move --execute --velocity 30` — one slow
   open → close → open on the bending joints only.
6. `python3 scripts/smoke.py cycle --execute --velocity 30 --cycles 3` — the
   same motion repeated: grip, open, grip, open. Still root1 + tip only, and
   the error register is read at every half cycle, so a hand that starts
   rejecting commands stops the run instead of grinding through it.
7. **Identify root2 (the PIP knuckle).** A grip driven from root1 + tip alone
   comes out shallow — the middle knuckle stays straight, which is visible in
   a photograph of the closed hand. root2's direction has never been
   identified, so find out with the smallest motion that can show it:

   ```sh
   python3 scripts/smoke.py cycle --execute --channel arm:A \
       --groups root2 --target 60 --velocity 20 --cycles 1
   ```

   `--groups` addresses ONLY the groups it names, so root1 and tip are not
   commanded at all here and the fingers stay where startup left them; the
   only thing that can move is the joint under test. The CLI warns before it
   moves, because a value that reads like "curl" may extend instead.

   **Watch the hand.** Note which way the middle knuckles went at 60. If they
   FLEXED, root2 shares root1's polarity and can be raised towards the same
   ~160-200 range in steps. If they EXTENDED, the group is inverted and needs
   a mapping of its own before anything drives it from a glove. Either way,
   write down what you saw before increasing `--target`.

   **2026-08-13, LEFT hand (build 2026-07-22): root2 shares root1's
   polarity.** `--target 60` and `--target 100` both flexed the middle
   knuckles, and `--groups root1,root2,tip --target 120` closed the hand to
   roughly 90° of finger flexion with no interference between fingers. The
   right hand has not been checked, so it is still an open question there —
   this is one hand's answer, not the model's.
8. **Look at a real grip** — `janken`, the one stage that closes the hand
   deeply, on the three groups whose direction is known:

   ```sh
   python3 scripts/smoke.py janken --execute --channel arm:A --rounds 2
   ```

   Rock → scissors → paper, `--rounds` times, `--hold` seconds each. Defaults
   are its own: `--target 160` because the point is the SHAPE
   of a closed hand, and `--velocity 30` because it travels further than
   `move` or `cycle` do. Scissors is the interesting posture — index and
   middle stay at the resting pose while the others curl — and it is what
   shows whether neighbouring fingers foul each other. Transitions command the
   fingers that are OPENING before the ones that are closing, so a curling
   finger cannot catch one still on its way out of the palm.

   Still root1 + root2 + tip only; roll and yaw are never written.
9. **Identify the HORIZONTAL joints** — finger spread (yaw) and thumb
   opposition (roll) — with `sweep`. These cannot be driven the way the
   bending ones are: they have no common "open", and their neutral is a
   different number on every finger (INIT_POSE: thumb yaw 23, index 96,
   middle 176, ring 212, little 162). Writing all five to one value is not a
   posture, it is five unrelated displacements. So `sweep` moves ONE joint at
   a time, out from its own neutral by `--delta` and back, then the other way.

   Do the thumb first — it is the one that can reach the palm:

   ```sh
   python3 scripts/smoke.py sweep --execute --channel arm:A --joints thumb
   python3 scripts/smoke.py sweep --execute --channel arm:A \
       --joints index,middle,ring,little
   ```

   Defaults: `--groups yaw`, `--delta 20`, `--velocity 20`. For the thumb's
   opposition use `--groups roll --joints thumb`. `--delta` is clamped per
   joint so `neutral ± delta` stays in 0-255, which narrows the `+` side on a
   joint that already sits high (ring yaw at 212 gets `+43 / -60` at
   `--delta 60`) — the CLI says so when it happens.

   **Record which way each joint went at `+`** (away from the other fingers,
   or towards them). That table is what a yaw/roll retarget map needs and the
   only reason this stage exists.

   **2026-08-13, LEFT hand, swept over the full range (`--delta 255`):**

   | joint | neutral | `+` (towards 255) direction | notes |
   |---|---|---|---|
   | `thumb_yaw` (CM yaw) | 23 | **palm side — opposition** | healthy over the whole range |
   | `index_yaw` | 96 | **inwards (closing)** | 0 side = spread open |
   | `middle_yaw` | 176 | **inwards (closing)** | 0 side = spread open |
   | `ring_yaw` | 212 | **inwards (closing)** | 0 side = spread open |
   | `little_yaw` | 162 | **inwards (closing)** | 0 side = spread open |
   | `thumb_roll` | 33 | moves, but the ends are hard to see | left alone in v1 |

   All four finger yaws agree, and every joint reached its ends without
   complaint. The thumb did not touch the index finger anywhere in the sweep.
   One hand's answer, as ever — the right hand has not been swept.
10. Only then: per-joint range calibration for the rest, which is what the
    retarget map is waiting on.

Through the arm instead of a USB adapter:

```sh
python3 scripts/smoke.py info --execute --channel arm:A --robot-ip 192.168.9.100
```

The arm SDK link is process-exclusive: stop teleop first.

## Bring-up log

**2026-08-13 — the two hands are different BUILDS, and they refuse MI 0x00
differently.** The left hand (build 2026-07-22) has no joint-map register at
all: every sub-index is silent and MI 0x4F says `0x09`, main index does not
exist. The right hand (build 2026-07-17) HAS the register and guards it:

| sub-index | right hand (07-17) | left hand (07-22) |
|---|---|---|
| `SI 0x00` map table, 36 B | silent, MI 0x4F = `0x10` insufficient permission | silent, `0x09` |
| `SI 0x24` map length, 2 B | silent, `0x10` | silent, `0x09` |
| `SI 0x25` remap enable, 1 B | **answers `0x00`** | silent, `0x09` |

`0x10` is not `0x09` and must not be read as one — but it does not matter,
because the right hand answers the question directly. `SI 0x25` reading zero
says the remap is off, which IS the factory mapping; the table was only ever
a way of finding that out. So `assert_default_joint_map()` now tries three
things in order — read the table, else read the flag, else classify the error
code — and returns which one answered, because the first two are checks and
the third is only a reasoned assumption. The CLI prints them differently for
the same reason: an operator should be able to see at a glance whether the
hand confirmed something or we inferred it.

**2026-08-13 — MI 0x0C has no live channel, so `enable()` was checking the
one register that cannot answer.** With the passthrough understood, `startup`
finally got as far as enabling, and then failed three attempts in a row with
every mask reading zero. The masks were fine. Read the same register on the
same hand in the same second:

| read | answer |
|---|---|
| MI 0x0C RTS=1 (stored, i.e. last written) | `0f 00 00 00 00 0f 0f 0f 0f 0f 0f 0f 0f 0f 0f 00 0f 0f 0f 0f 0f 0f 0f 0f 0f` |
| MI 0x0C RTS=0 (live) | 25 zero bytes |

Both answered; only one of them means anything. All-zero from the live
channel is "not implemented", the same family as MI 0x00 and MI 0x0D — not
"every joint is disabled". `enable()` now verifies against RTS=1 (and watches
MI 0x4F across the write, so a REJECTED write is reported as a rejection
rather than as a slow one), and `info` prints both channels side by side and
says outright which one is lying.

Ruled out first, on paper, before touching the hand: the enable frame is
byte-for-byte identical to the vendor SDK's (`0c 00 19 0f 00 00 00 00 0f …`,
same `_pack_u8` layout, same EDL, same DLC padding), and the vendor never
writes the engineering password for mode or enable — it appears in their
source only as a comment on `RESTORE_ALL`.

**2026-08-13 — the end module only hands over a reply that CHANGED.** With
BRS off the hand answers everything, and yet `smoke.py cycle` was silent from
its first frame, every single time, while `info` run seconds earlier was
perfect. The stage was not the cause; the frame sequence was.

Measured over arm A with reads only:

| what was sent | result |
|---|---|
| the same position read (`01 00 19`) six times | `R . . . . .` |
| the same read with the length alternating 25 / 26 | `. R R R R R`, payload byte-for-byte identical |
| the heartbeat (`42 45 04`, a counter, so every answer differs) repeated | keeps answering |
| `OnClearChData` before every request | *worse* — all silent |

So the suppression is on the **reply**, not the request: the heartbeat counter
advances once per request even while only every other answer is visible, which
means every request reaches the hand. What decides is whether the reply's bytes
differ from the frame the module handed over last — and the reply echoes the
3-byte request header, so one byte of the request is enough to make it differ.

That is the whole bug. `info` reads a different register almost every frame, so
its answers always differ. `cycle` opened with `4F 00 01` (warm-up), then the
joint-map probe this firmware never answers, then `4F 00 01` again — and the
answer to that was byte-identical to the one the module had already delivered
**to the previous `info` process**, because the slot outlives the process that
filled it. Nothing was late, nothing was dropped on the wire: the answer was
simply never handed over, three frames in a row, and the run looked dead.

The driver now asks for one byte more when the answer would otherwise repeat,
and retries once with a different length when a read comes back silent — the
retry is the part that matters, because a fresh process cannot know what the
module already gave to the last one. `Bus.suppresses_repeat_frames` marks the
transports this applies to; it is a no-op on SocketCAN.

**Root cause found: the end module transmits with BRS on; the hand cannot
answer that.** Fix is on the ARM BOARD side (disable BRS on the end-module
channel) — requested from the arm engineer 2026-08-12. Detail below.

**2026-08-11, D1 (arm B passthrough, no USB-CANFD adapter present) — no
answer.** `smoke.py info --execute --channel arm:B` was run at frame ids
`0x001` and `0x002`. In both cases:

- the arm SDK link came up (`Robot connected IP=192.168.9.100`) and every
  request was accepted by the end module — the vendor library's own TX dump
  shows `01 00 00 00 | 41 69 08`, i.e. the CAN id little-endian in 4 bytes
  followed by our 3-byte HOP header, `data size=7`. So the wire format, the
  passthrough call and the frame construction are doing what they should;
- **nothing ever came back on `frame_id | 0x400`.** Every read timed out;
  no error frames, no malformed frames, no partial replies.

Nothing moved: `info` writes nothing. `arm:A` was not tried (the run was
blocked by a permission gate), so the *other* arm is still untested and is
the first thing to try next.

What was established, and what it rules out:

| test | result |
|---|---|
| `info --execute` on arm A and arm B, frame ids `0x001` and `0x002` | silent |
| the same at the broadcast id `0x7FF` | silent |
| **full sweep of every legal frame id** (`0x001`-`0x3FE`, 1022 read requests, both arms) | silent — **a changed frame id is ruled out** |
| the same probe through d1-vr-teleop's own DH116S-proven `D1ArmChannelBus` instead of this package's port | silent — **this package's transport is ruled out** |
| raw `OnGetChData*` polling for 2 s, bypassing every abstraction | `n == 0` throughout — nothing is arriving at the end module at all |

Known-good context that narrows it further: both hands are O30s, wired
through the same connector and cable as the DH116S that worked over this
same passthrough; the hands were verified working on CAN FD standalone
before mounting; the end module is configured FD-only.

Note also that "RX mirrors TX" in the DH116S notes means the RX *layout*
matches the TX layout, not that sent frames are echoed back — so the silence
at `0x7FF` is not by itself evidence about the receive path.

### Root cause (confirmed 2026-08-12 from the vendor SDK)

`linker_hand_o30_control.py` sends every frame with `bitrate_switch=False`
and says why, from measurement:

> 关闭 BRS：实测 BRS 开启 + 高速数据段会把总线打入 BUS-OFF，设备无法应答;
> BRS 关闭时数据段保持仲裁波特率，稳定收发。
> *(BRS off: measured — BRS on with a high-speed data phase throws the bus
> into BUS-OFF and the device cannot answer. With BRS off the data phase
> stays at the arbitration rate and transfers are stable.)*

So the configuration this hand is known to work in is **CAN FD, 11-bit
standard frames, arbitration 1 Mbps, BRS OFF** (data phase therefore also
1 Mbps). The arm's end module drove the DH116S with BRS on at 5 Mbps, and a
board that keeps doing so cannot be answered by an O30. **The fix is to
disable BRS on the end-module channel**, which is an arm-board setting made
with the vendor's tooling — not something any code here can do. This driver
already sends `bitrate_switch=False`, so nothing changes on our side.

The evidence that led there, kept because the reasoning is reusable:

**1. Data-phase / BRS mismatch.** Three facts line up:

- The hand's factory CAN config is arbitration 1 Mbps, **data phase 1 Mbps**
  (`MI 0x35 / SI 0x09 = 0x06`) with **BRS disabled** (`SI 0x07 = 0x00`), in
  **CAN 2.0** mode (`SI 0x06 = 0x01`).
- The vendor's own SDK sends every frame with `bitrate_switch=False`, and
  says why, from measurement: *"BRS on + a high-speed data phase throws the
  bus into BUS-OFF and the device cannot answer."*
- The DH116S ran this same end-module bus at 1 Mbps / **5 Mbps with BRS on**
  (`RealBus` sets `bitrate_switch=True`), and worked — its feedback CSVs in
  `~/hand_logs` are timestamped the day before the swap.

If the module transmits BRS frames with a 5 Mbps data phase, a hand that
cannot decode that data phase never receives the frame, so it never ACKs.
And a CAN frame that no node ACKs never completes — which also explains the
missing "mirror": the end module reports frames that made it onto the bus,
so a transmission that never completes produces nothing to report. Every
observation above is consistent with this, on both arms, at every id.

Note the trap in the obvious fix: matching the module means turning the
hand's BRS ON at 5 Mbps, which is exactly what the vendor measured as
BUS-OFF. The END MODULE is the side to change — but the DH116S shares that
bus, so the change has to be checked against both hands.

**2. The hand is not speaking HOP.** `MI 0x42 / SI 0xDD` switches the CAN
path between HOP (`0x00`) and the **legacy LinkerHand CAN protocol**
(`0x01`), persists across power cycles, and takes effect after a reboot. A
hand left in legacy mode ignores every HOP frame — including all 1022 of the
scan's — while working perfectly with the vendor's older tooling. Still
worth confirming while the board is open, since it costs one read.

### Re-test procedure once BRS is disabled on the board

No code change is needed — the driver has always sent `bitrate_switch=False`.

1. Confirm with the engineer what the channel now runs: expect arbitration
   1 Mbps, **BRS off**, CAN FD, 11-bit standard frames.
2. Power the hands, wait ~5 s for the self-check (the fingers move by
   themselves), keep clear.
3. `smoke.py info --execute --channel arm:B --node 1`, then `--node 2`, then
   the same on `arm:A`. Expect model `O30`, protocol `HOP`, a hand side, and
   a factory joint map. Record which arm answers at which frame id — that is
   the left/right mapping, and it has never been observed.
4. If still silent, sweep the ids again (the scan script is described above)
   before suspecting anything else; then check `42 DD 01` (HOP vs legacy
   protocol) and the physical-layer list below.
5. Only then `smoke.py move --execute` — one arm at a time, fingers clear.

### What could not be determined from software

- **Whether the end module's transmission is ACKed on the bus.**
  `OnSetChData*` returns a bool for the UDP command reaching the arm, not
  for the CAN frame completing (`robot_control.h`; the vendor SDK exposes no
  bus-error, error-counter or BUS-OFF state at all).
- **The end module's bit timing.** Nothing in d1-sdk sets or reports the
  channel's arbitration/data rate, BRS or sample points — `set_ch == 1` is
  the entire interface. It is configured on the board itself with the arm
  vendor's tooling. (The channel does carry both classic CAN 2.0 — the stock
  gripper's 8-byte frames in `example/gripper.cpp` — and 64-byte CANFD, so
  "FD-only" is a configuration, not a property of the hardware.)
- **Whether a TX frame is ever echoed back.** The "RX mirrors TX" note in
  d1-vr-teleop's `arm_hand_bus.py` and the DH116S bring-up doc means the
  RX *wire layout* matches TX (`[id 4 bytes LE][payload]`), not that sent
  frames come back. Both were written from a session where the hands were
  connected and answering (2026-07-19), so there is no record of a mirror
  without a responsive node — the absence of one here proves nothing on its
  own.

### Physical-layer checklist (power off, hand disconnected)

The bench test that passed and the robot test that failed differ in the
cable, so rule the cable out before touching any configuration:

1. Continuity, flange connector → hand connector, pin by pin, against the
   adapter cable's pinout.
2. **CANH/CANL polarity end to end.** A swapped pair is the classic
   "everything looks fine, nothing answers" fault, and it is invisible from
   software.
3. Termination: 60 Ω across CANH/CANL with the bus powered down and both
   ends connected (two 120 Ω terminators in parallel). 120 Ω means only one
   end is terminated; open means neither.
4. Ground and supply continuity to the hand, and that the supply holds up
   for a 6.3 A peak.
5. With a scope or a CAN analyser on the flange pair: does the module's
   frame appear at all, and does anything ACK it?

### Questions for the engineer who ran the standalone bench test

1. Exactly which bus settings worked: arbitration rate, data rate, BRS
   on/off, CAN 2.0 vs CAN FD, sample points.
2. Which SDK/protocol was used — the HOP-based O30 SDK, or the older
   LinkerHand CAN tooling (i.e. is `MI 0x42 / SI 0xDD` still `0x00`)?
3. The frame id each hand ended up with, and whether anything was saved to
   flash (`35 1E 01 01`) during that session.
4. Which adapter was used (transparent-case SocketCAN device vs the
   `libcanbus.so` metal-case one).
5. For the arm's end module: when was it set to "CANFD only", what exactly
   was changed, with which tool, and what are its current arbitration/data
   rates and BRS setting?
6. The adapter cable's pin table (flange side ↔ hand side).

## Bench configuration procedure (NOT yet run — needs approval and an operator)

For reading and, if approved, correcting the hand's CAN configuration with a
direct USB-CANFD adapter and the hand's own supply. **Every step up to
"if a change is approved" is read-only.**

1. Wire the adapter to the hand's CAN pair and power the hand from its own
   supply (24 V, ≥ 7 A headroom). Bring the interface up at 1 Mbps with a
   1 Mbps data phase — matching the hand's factory config, not the DH116S's:

   ```sh
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on
   candump -td can0        # leave running
   ```

2. **Read first, in CAN 2.0** — the mode the hand ships in:

   ```sh
   V=~/d1-vr-teleop/.venv/bin/python
   $V hands/linkerbot/o30/scripts/smoke.py info --execute \
       --channel can0 --node 1 --can-classic
   ```

   If that is silent, retry without `--can-classic` (FD), then sweep the ids
   with the same scan used on the arm. Anything that answers settles the
   frame id, the CAN mode and the protocol in one shot.

3. **Record the configuration before changing anything.** The interesting
   registers, as raw frames (`cansend can0 <id>#<data>`, id = the hand's
   frame id):

   | read | meaning |
   |---|---|
   | `35 06 01` | CAN interface type: `01` = CAN 2.0, `02` = CAN FD |
   | `35 07 01` | BRS enable: `00` = off |
   | `35 08 01` | arbitration baud: `01` = 1 Mbps |
   | `35 09 01` | data baud: `06` = 1 Mbps, `02` = 5 Mbps |
   | `35 00 02` | standard frame id |
   | `42 DD 01` | protocol path: `00` = HOP, `01` = legacy CAN |
   | `41 69 08` | protocol name (expect `HOP`) |

4. **Only if a change is approved**, with the operator present. To match the
   arm's end module (FD, 1 Mbps arbitration, 5 Mbps data, BRS on):

   ```
   35 06 01 02      # CAN FD
   35 08 01 01      # arbitration 1 Mbps
   35 09 01 02      # data phase 5 Mbps
   35 07 01 01      # BRS on
   35 1E 01 01      # SAVE TO FLASH  ← the irreversible step
   ```

   then power-cycle and re-read step 3 to confirm. Note the ordering trap:
   changing the bitrate breaks the link you are changing it over, so the
   save is the last frame you will get an answer to.

   The driver deliberately cannot send the save frame by accident —
   `save_to_flash()` requires `confirm=True`, and the factory-reset
   sub-indices (`0x1C`, `0x1D`) are refused outright.

5. The alternative is to change the END MODULE instead (1 Mbps data phase or
   BRS off), which needs the arm vendor's tooling and would have to be
   checked against the DH116S, which shares that bus configuration.

## Open questions

- **Frame id of the actual hands.** Convention says right `0x01` / left `0x02`,
  the spec's default is `0x001`. Confirm per hand with `smoke.py info` and
  record it; two hands on one bus must differ.
- **Power connector pinout and the harness rating** for 6.3 A peaks.
- **Sparse (`MI 0x30`) joint numbering** — see above.
- **Per-joint open/closed direction and range.** Only root1/tip are known
  (0 ≈ straight, ~200 bent, resting readback ~25-37). roll/yaw/root2 are
  uncalibrated, which is why `home()` leaves them alone and there is no
  `unit="deg"`.
- **Velocity units.** 0-255 with no documented mapping to °/s.
- **Which CAN path the hands are actually configured for** — see the bring-up
  log above. Until that is settled nothing else can be measured.
- **Firmware version of the delivered hands** (0.0.2 vs 0.0.3 product-info
  layout, and whether `MI 0x20` half-word positions work).
- **Tactile sensors** (`MI 0x31`-`0x34`) are not implemented here at all.

## Not in this package

- **Feedback CSV logging.** The DH116S's per-hand command/feedback CSVs come
  from d1-vr-teleop (`server/hand_channel.py`, gated by `D1_TELEOP_HAND_LOG`),
  not from this repo. An O30 backend there gets the same treatment for free
  once it exists.
- **The teleop arm-passthrough bus.** d1-vr-teleop's
  `server/arm_hand_bus.py` hardcodes the DH116S response COB-IDs; it needs
  `frame_id | 0x400` before it can carry an O30. `transport.ArmPassthroughBus`
  here is the O30-generic sibling used by `smoke.py`.
- **Tool config JSON deployment.** Export it into the robot repo:
  `python -m manipulation_kit.hands.export_tool_config linkerbot/o30 <out.json>`.
