# Writing an end-effector provider

**CURRENT.** How to drive your own end effector on a D1 from a process of
your own, so that every client of the robot reaches it through
`d1-firmwared`'s `/v1/end_effectors/{side}` resource exactly as it reaches
the stock gripper.

## The model

`d1-firmwared` (the daemon) drives the end effectors that ship with the D1
itself. For any other end effector, the daemon's configuration declares the
side `driver = "shm"`, and a **provider** — your process — drives the device.
The two talk over iceoryx2 0.9.3 shared memory under the contract
`end_effector.shm/1`. Its normative text is `spec/end_effector.shm/1.md` in
the d1-firmware repository, and its byte layout is
`spec/end_effector.shm/1.layout.json`, which this package vendors verbatim as
`manipulation_kit/end_effectors/end_effector.shm.1.layout.json`.

The provider owns everything about the device: its bus, its protocol, its
calibration, its startup probe. The daemon keeps everything that is the
robot's: it validates every command against your declaration (openness range,
grips and `extra` keys you list, joint count and limits), enforces the
joint-command timestamp rule, never sends faster than your `max_command_hz`,
refuses commands under the soft-kill latch, and stops you on a soft kill.

```
client --REST/WS--> d1-firmwared --iceoryx2 (omakase/ee/<side>/...)--> your provider --> device
```

## What you implement

Subclass `manipulation_kit.end_effectors.shm_provider.Provider`:

| method | called from | what it does |
| --- | --- | --- |
| `descriptor()` | once, at start | your driver name, `family`, model, joints (name, `rad` or `fraction`, limits) and capabilities |
| `set(command)` | a worker thread | drive to `command.openness` (`1.0` open, `0.0` closed); with `wait` and `stroke_completion`, return when the stroke concludes |
| `joints(targets, timestamp_us)` | a worker thread | drive every joint; never wait for motion |
| `read()` | the port thread, every state period | the current `StateSample`; must return at once |
| `stop()` | a worker thread | take the actuators' authority away **without opening**; idempotent |
| `clear_fault()`, `registers(ids)` | a worker thread | optional, if you declare them |
| `tick()` | the port thread | optional control-loop tick, if you declare `ticks` |
| `daemon_presence(present)` | the port thread | the daemon appeared or went away: keep holding, start nothing |

Raise `ProviderError(kind, message)` to refuse or fail; `kind` is one of
`refused`, `unprocessable`, `unavailable`, `device`, `timeout`. The REST
client sees the same kind.

Then serve it:

```python
from manipulation_kit.end_effectors.shm_provider import ServeOptions, serve

serve(MyGripper(), ServeOptions(service="omakase/ee/b"))
```

`serve` owns every port, publishes your descriptor (every 500 ms), heartbeat
(every 50 ms) and state (every 20 ms), and applies the contract's rules
before your methods are called: a `set` addressed to an earlier instance of
your process, or arriving after its 250 ms start deadline, is answered
`stale_target` and never executed; joint targets older than 200 ms are
dropped and only the newest is delivered; a rise of the daemon's stop
generation calls `stop()` even if the stop request itself was lost. A
`set` that blocks for a long stroke never delays a heartbeat or a stop.

## The family and its detail

`family` names the class of your end effector and the schema of the `detail`
your state carries (`{"family", "schema_version", "data"}`, at most 8 KiB of
JSON). Clients read `detail.data` only for a family and version they know.
Two families exist:

- `parallel_gripper` (schema 1): `data` carries at least `model`, `kind`
  (the newest stroke's outcome: `grasp`, `contact`, `empty`, `open`,
  `timeout`, `fault`, `overload`, `halted`, `blind`, `lost`),
  `torque_cap_nm` and `stroke_interrupted`; the robot's telemetry reads these.
- `dexterous_hand` (schema 1): `data` is a `HandState` (`model`, `side`,
  `positions`, `positions_wire`, `enabled`, `all_enabled`, `error_code`,
  `faults`, `live`, `features`).

A device that is neither declares a family of its own; clients then treat its
detail as opaque.

## Deploying on a robot

1. Declare the side in `/etc/d1-firmwared/real.toml` and restart the daemon
   once (a configuration change is a restart key):

   ```toml
   [end_effector.b]
   driver = "shm"
   model = "acme/two-finger-g2"   # optional: refuse any other model
   ```

2. Install `manipulation-kit[shm]` (iceoryx2 is pinned to exactly 0.9.3,
   the daemon's version) and run your provider **as the account the daemon
   runs as** (`doraemon` on a provisioned D1): iceoryx2 0.9.3 creates its
   shared memory readable by its owner only. A systemd unit:

   ```ini
   [Unit]
   Description=ACME gripper provider (end_effector.shm/1)
   After=d1-firmwared.service

   [Service]
   User=doraemon
   ExecStart=/usr/bin/python3 -m acme_gripper.provider --service omakase/ee/b
   Restart=on-failure
   RestartSec=1s

   [Install]
   WantedBy=multi-user.target
   ```

   Do not give the unit `DynamicUser=`, `PrivateIPC=`, `PrivateDevices=` or
   `PrivateUsers=`; with `ProtectSystem=strict`, add
   `ReadWritePaths=/run/d1-firmwared/iceoryx2`.

3. `GET /v1/end_effectors/b` answers `available: true` and your descriptor.
   While your provider is down it answers `available: false` with the reason
   in `error`, and commands answer 502.

A provider declares no arm tool. Register your device's mass, centre of mass,
inertia and TCP with `POST /v1/arm/{side}/tool` after every daemon start, or
the arm compensates gravity for the wrong load.

Your declaration is fixed for the daemon's lifetime: the daemon keeps the
first valid one it receives, takes back a restarted provider that declares
the same, and refuses a changed declaration until it is restarted itself.

## Trying it without hardware

`mkit-ee-provider` serves a simulated end effector:

```sh
pip install -e '.[shm]'
mkit-ee-provider --model gripper --service omakase/ee/b            # on a robot
mkit-ee-provider --model hand --root /tmp/ee/ --prefix try_ --service omakase/ee/b
```

Against a d1-firmware checkout, the daemon's own driver tests run against it:

```sh
D1_FIRMWARE_DIR=~/d1-firmware pytest tests/end_effectors
```

which compares this package's layout with the checkout's and runs the
daemon's `an_external_provider_command` test against both simulated models.

## Checking your own implementation

A provider in another language implements the same thirteen messages. Check
your definitions against `end_effector.shm.1.layout.json` in a test — every
message's iceoryx2 type name, size, alignment and every field's offset, size
and type, and the SHA-256 of the file itself
(`e66b7890142f92fe2429d48847a906e313e0836b3558fc4806a7b3eb0c527a49`; version 1
is frozen, so a changed file is a new contract version). That is what
`tests/end_effectors/test_shm_contract.py` does for the ctypes structures
here.
