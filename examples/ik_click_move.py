#!/usr/bin/env python3
"""Interactive click-to-IK teleop preview for the D1 arms (GUI, no robot).

Open a 3D window with the full-body D1, pick a hand, Ctrl+click any point in
the scene — the selected arm's end-effector moves there via IK, live. Every
accepted target is validated by pyguard (limits + torso keep-out + arm-arm +
self-collision); a rejected click flashes the status line and nothing moves.
Export the whole session as a standard gesture CSV and play IT on the robot
through ``gesture_play`` (pre-flight gate + runtime collision guard):

    # host (needs a display): pip install mujoco "imageio[ffmpeg]"
    python3 scripts/ik_click_move.py

    # repeatable, no-GUI regression test over a list of Cartesian targets:
    python3 scripts/ik_click_move.py --targets scripts/ik_click_targets_example.csv

Controls
--------
    left-drag            rotate camera      right-drag   pan
    scroll               zoom
    L / R                select LEFT / RIGHT hand (physical side)
    Ctrl + left-click    move the selected hand to the clicked surface point
                         (macOS: use Cmd + left-click — Ctrl+click is often
                         remapped to right-click by the OS). Auto-commits a
                         keyframe when the move animation finishes.
    W / S                jog the selected hand +/- 1 cm along world X
    A / D                jog +/- 1 cm along world Y   (hold any jog key to repeat)
    UP / DOWN            jog +/- 1 cm along world Z
                         Jogs stay LOCAL: a step that would swing the shoulder/
                         elbow >60 deg to another arm configuration is refused —
                         command such moves deliberately via Ctrl/Cmd+click
                         (animated, path-linted on commit), or switch hands.
    SPACE / ENTER        commit the current (jogged) pose as a keyframe —
                         jog moves are live but NOT auto-committed
    H                    both arms back to HOME (commits a keyframe)
    U                    undo the last keyframe
    E                    export the session as ik_click_motion.csv (starts/ends at HOME)
    ESC / Q              quit

Markers: green sphere = last accepted target, red sphere = last rejected
target, yellow dot = the selected hand's end-effector. Workflow: Ctrl/Cmd+click
a surface near the goal (fixes x,y), then jog W/S/A/D/UP/DOWN to dial the pose
in, then SPACE to commit.

``--targets FILE`` runs headless: each CSV row is ``side,x,y,z[,expect]``
(expect one of ok / unreachable / guard, default ok; ``#`` comments allowed).
Each target is solved from HOME, checked against pyguard, and classified; the
exit code is non-zero if any classification differs from ``expect``.

Belly-front workspace (measured against pyguard, see FALLBACK_SEEDS)
--------------------------------------------------------------------
Single hand: solvable from x >= 0.22 m in front of the torso, full centerline
coverage (y = 0) from x >= 0.24, z 0.19-0.45. Anything x < ~0.20 is genuinely
inside the conservative safety envelope (torso_belly box + link clearances)
and is rejected no matter the elbow configuration. Both hands simultaneously:
keep >= 0.28 m between the hands (e.g. y = +/-0.14 at x = 0.28, z = 0.30);
tighter spacing trips the 0.06 m arm-arm margin or the torso boxes. IK
solutions here need the elbow flared out — the solver retries from
FALLBACK_SEEDS automatically when the home-seeded solve hits the guard.

IK: damped-least-squares on the MuJoCo model (position-only, seeded from the
current pose with a nullspace bias toward HOME, joint limits clamped). For an
assembly test the exact elbow configuration is not critical — the exported CSV
is joint-space keyframes, each pyguard-validated, and ``gesture_play``
re-validates at pre-flight. To re-solve waypoints with the VENDOR
collision-aware IK instead, see ``make_cartesian_test_gesture.py``.

Physical-side mapping (pyguard ARM_SIDES): LEFT hand = ArmSide A = URDF
``Joint1_R..7_R``; RIGHT = B = ``_L``.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
#: Package-relative anchors. This script lives OUTSIDE the wheel (examples/),
#: so the anchor comes from the import rather than from this file's location;
#: the assets themselves still ship inside manipulation_kit, so this works from
#: a checkout and from an installed wheel alike.
KIT = os.path.dirname(os.path.abspath(__import__("manipulation_kit").__file__))
DEFAULT_URDF = os.path.join(KIT, "description", "d1", "d1.urdf")
DEFAULT_HOME = os.path.join(KIT, "config", "home_pose.json")

#: physical side -> (SDK ArmSide, URDF joint suffix). LEFT hand = A = "_R" tree.
SIDES = {"left": ("A", "R"), "right": ("B", "L")}

#: Guard-clean "elbow flared out" postures (deg) used as extra IK seeds when the
#: home-seeded solve lands on a guard violation — typically Link2 (upper arm)
#: grazing the torso_belly keep-out while reaching toward the body centerline.
#: Found by rejection-sampling joint space against pyguard; each puts the EE in
#: the belly-front band (x 0.22-0.30, z 0.19-0.45) with all links clear. With
#: these, single-hand belly-front work is solvable from x >= 0.22 m (x < 0.20
#: is genuinely inside the conservative safety envelope and stays blocked);
#: dual-hand simultaneous work needs >= ~0.28 m between the hands.
FALLBACK_SEEDS = {
    "R": [
        [-107.9, 49.0, 140.1, -123.4, -139.9, 14.0, 18.4],
        [-70.1, 62.1, 103.5, -119.6, 166.5, -5.3, 29.8],
        [-110.4, 74.2, 131.6, -105.8, 13.0, -16.0, -41.7],
        [-62.6, 86.4, 91.6, -115.4, 80.3, -4.1, -20.7],
        [-115.8, 65.1, 123.5, -125.4, 30.1, -32.9, -54.3],
    ],
    "L": [
        [88.9, 67.5, -122.7, -107.3, -136.0, 2.5, -36.7],
        [63.6, 79.6, -96.4, -114.9, -143.5, -11.2, -31.9],
        [135.7, 59.8, -157.7, -113.6, -51.2, -9.8, 31.4],
        [62.6, 86.4, -91.6, -115.4, -80.3, -4.1, 20.7],
        [34.4, -56.4, -42.4, -131.6, -112.0, 3.6, 18.7],
    ],
}


# --------------------------------------------------------------------------- #
# model helpers (pure; unit-testable without a window)
# --------------------------------------------------------------------------- #
class D1Model:
    def __init__(self, urdf: str = DEFAULT_URDF, home_json: str = DEFAULT_HOME):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(urdf)
        self.data = mujoco.MjData(self.model)
        d = json.load(open(home_json))
        vals = dict(zip(d["joint_order"], d["home_pose"]))
        self.home = {"A": [vals[f"A{i}"] for i in range(1, 8)],
                     "B": [vals[f"B{i}"] for i in range(1, 8)]}
        # per URDF suffix: joint qpos addresses, dof addresses, limits (rad)
        self.qadr, self.dofadr, self.jrange = {}, {}, {}
        for suf in ("R", "L"):
            names = [f"Joint{i}_{suf}" for i in range(1, 8)]
            jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                    for n in names]
            if min(jids) < 0:
                raise RuntimeError(f"missing arm joints for suffix {suf}")
            self.qadr[suf] = [self.model.jnt_qposadr[j] for j in jids]
            self.dofadr[suf] = [self.model.jnt_dofadr[j] for j in jids]
            self.jrange[suf] = [tuple(self.model.jnt_range[j]) for j in jids]
        # EE bodies: the two finger bodies per hand; EE point = their midpoint,
        # jacobian on their (fused hand) parent body.
        self.fingers, self.hand_body = {}, {}
        for suf in ("R", "L"):
            f = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                   f"yubi_{suf}_{s}_finger_link")
                 for s in ("right", "left")]
            self.fingers[suf] = f
            self.hand_body[suf] = self.model.body_parentid[f[0]]
        self.go_home()

    # -- pose I/O ----------------------------------------------------------- #
    def set_arm_deg(self, suf: str, joints_deg):
        for adr, deg in zip(self.qadr[suf], joints_deg):
            self.data.qpos[adr] = np.deg2rad(deg)
        self.mj.mj_forward(self.model, self.data)

    def get_arm_deg(self, suf: str):
        return [float(np.rad2deg(self.data.qpos[a])) for a in self.qadr[suf]]

    def go_home(self):
        self.set_arm_deg("R", self.home["A"])
        self.set_arm_deg("L", self.home["B"])

    def ee_point(self, suf: str):
        xs = [self.data.xpos[f] for f in self.fingers[suf]]
        return (np.array(xs[0]) + np.array(xs[1])) / 2.0

    # -- IK ------------------------------------------------------------------ #
    def solve_ik(self, suf: str, target, *, iters: int = 200, tol: float = 2e-3,
                 damping: float = 1e-3, posture_gain: float = 0.05,
                 bias_deg=None, early_stop: bool = True):
        """Position-only DLS IK for one arm toward ``target`` (world, m).

        Seeds from the CURRENT pose, biases the nullspace toward ``bias_deg``
        (default: the seed — stays near the seed configuration), clamps joint
        limits every step. With ``early_stop=False`` the loop runs all iters,
        walking the nullspace toward the bias while holding the target — used
        to polish a solution back toward a reference posture. Returns
        (joints_deg, err_m)."""
        mujoco, m, d = self.mj, self.model, self.data
        dofs = self.dofadr[suf]
        seed = np.array([d.qpos[a] for a in self.qadr[suf]])
        bias = seed if bias_deg is None else np.deg2rad(np.asarray(bias_deg, float))
        target = np.asarray(target, dtype=float)
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        for _ in range(iters):
            point = self.ee_point(suf)
            err = target - point
            if early_stop and np.linalg.norm(err) < tol:
                break
            mujoco.mj_jac(m, d, jacp, jacr, point, self.hand_body[suf])
            J = jacp[:, dofs]                                   # 3x7
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + damping * np.eye(3), err)
            # nullspace bias toward the reference posture (keeps the elbow sane)
            q = np.array([d.qpos[a] for a in self.qadr[suf]])
            dq += posture_gain * (np.eye(7) - np.linalg.pinv(J) @ J) @ (bias - q)
            dq = np.clip(dq, -0.15, 0.15)                        # rad per step
            for k, adr in enumerate(self.qadr[suf]):
                lo, hi = self.jrange[suf][k]
                d.qpos[adr] = float(np.clip(d.qpos[adr] + dq[k], lo, hi))
            mujoco.mj_forward(m, d)
        err_m = float(np.linalg.norm(target - self.ee_point(suf)))
        return self.get_arm_deg(suf), err_m

    def solve_ik_checked(self, suf: str, target, check, *, err_tol: float = 0.03,
                         retry_on_guard_only: bool = False):
        """IK with guard-aware seed retry, preferring minimal motion.

        Solve from the current pose first; if the result misses ``target`` by
        more than ``err_tol`` or fails ``check()`` (a callable evaluating the
        CURRENT model state, e.g. a pyguard check), retry from each
        FALLBACK_SEEDS posture. Every clean retry is polished back toward the
        start posture (nullspace walk holding the target) and the candidate
        closest to the start wins — a reconfiguration only happens when no
        nearby solution passes the guard. ``retry_on_guard_only`` skips the
        retries when the first attempt failed on reach rather than on the
        guard (jog: don't tunnel through workspace limits). Returns
        (joints_deg, err_m, clean); on ``clean=False`` the model is left at
        the first attempt."""
        start = self.get_arm_deg(suf)
        joints, err = self.solve_ik(suf, target)
        if err <= err_tol and check():
            return joints, err, True
        first = (joints, err)
        if retry_on_guard_only and err > err_tol:
            return first[0], first[1], False
        cands = []
        for seed in FALLBACK_SEEDS[suf]:
            self.set_arm_deg(suf, seed)
            j, e = self.solve_ik(suf, target)
            if e > err_tol or not check():
                continue
            jp, ep = self.solve_ik(suf, target, bias_deg=start,
                                   early_stop=False, iters=80, posture_gain=0.1)
            if ep <= err_tol and check():
                j, e = jp, ep
            else:
                self.set_arm_deg(suf, j)        # polish broke it — keep the raw solution
            cands.append((max(abs(a - b) for a, b in zip(j, start)), j, e))
        if cands:
            _, j, e = min(cands, key=lambda c: c[0])
            self.set_arm_deg(suf, j)
            return j, e, True
        self.set_arm_deg(suf, first[0])
        return first[0], first[1], False


# --------------------------------------------------------------------------- #
# session -> gesture CSV
# --------------------------------------------------------------------------- #
class Session:
    """Accepted keyframes (A7+B7 deg). Exports the shared gesture-CSV contract."""

    def __init__(self, model: D1Model, seg_s: float = 2.5):
        self.m = model
        self.seg_s = seg_s
        self.frames: list[tuple[list, list]] = []
        from manipulation_kit.guard import MotionGuard
        self.guard = MotionGuard()

    def current(self):
        return self.m.get_arm_deg("R"), self.m.get_arm_deg("L")   # (A, B)

    def check_current(self):
        a, b = self.current()
        return self.guard.check(joints_a_deg=a, joints_b_deg=b)

    def push(self):
        """Append the current pose as a keyframe.

        Also lint the joint-space path from the previous keyframe (gesture_play
        pre-flights the INTERPOLATED trajectory, so two clean keyframes with a
        dirty path between them would be refused at playback). Returns a
        warning string if any intermediate pose violates the guard, else None."""
        a1, b1 = self.current()
        a0, b0 = self.frames[-1] if self.frames else (self.m.home["A"], self.m.home["B"])
        warn = self._segment_violation(a0, b0, a1, b1)
        self.frames.append((a1, b1))
        return warn

    def _segment_violation(self, a0, b0, a1, b1, steps: int = 25):
        for t in np.linspace(0.0, 1.0, steps)[1:-1]:
            ai = [p + (q - p) * t for p, q in zip(a0, a1)]
            bi = [p + (q - p) * t for p, q in zip(b0, b1)]
            rep = self.guard.check(joints_a_deg=ai, joints_b_deg=bi)
            if not rep.ok:
                return f"path @{t*100:.0f}%: {rep.violations[0]}"
        return None

    def path_warnings(self):
        """Lint every segment of the would-be export (incl. the home legs)."""
        home = (self.m.home["A"], self.m.home["B"])
        seq = [home] + self.frames + [home]
        return [(i, w) for i, ((a0, b0), (a1, b1)) in enumerate(zip(seq, seq[1:]))
                if (w := self._segment_violation(a0, b0, a1, b1))]

    def undo(self):
        if self.frames:
            self.frames.pop()
        a, b = self.frames[-1] if self.frames else (self.m.home["A"], self.m.home["B"])
        self.m.set_arm_deg("R", a)
        self.m.set_arm_deg("L", b)

    def export(self, path: str) -> int:
        home = (self.m.home["A"], self.m.home["B"])
        seq = [(0.0, *home)] + [(self.seg_s, a, b) for a, b in self.frames] \
            + [(self.seg_s, *home)]
        with open(path, "w") as f:
            f.write("# D1 dual-arm gesture (omakaseos keyframe format, angles in DEGREES)\n")
            f.write("# joint order: R1..R7 (ArmSide::A = physical LEFT), L1..L7 (ArmSide::B = physical RIGHT)\n")
            f.write("# IK CLICK SESSION: exported by scripts/ik_click_move.py\n")
            f.write("duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7\n")
            for dur, a, b in seq:
                f.write(",".join([f"{dur:.4f}"] + [f"{v:.4f}" for v in a]
                                 + [f"{v:.4f}" for v in b]) + "\n")
        return len(seq)


# --------------------------------------------------------------------------- #
# GUI
# --------------------------------------------------------------------------- #
def run_gui(urdf: str, home_json: str, out_csv: str):
    import glfw
    import mujoco

    dm = D1Model(urdf, home_json)
    sess = Session(dm)
    m, d = dm.model, dm.data
    m.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
    m.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]

    if not glfw.init():
        raise RuntimeError("glfw init failed (needs a display)")
    win = glfw.create_window(1280, 960, "D1 click-to-IK (L/R hand · Ctrl/Cmd+click target)", None, None)
    glfw.make_context_current(win)
    glfw.swap_interval(1)

    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 115, -18, 2.4
    cam.lookat[:] = [0, 0, 0.45]
    opt = mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = 1
    scene = mujoco.MjvScene(m, maxgeom=2000)
    ctx = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
    pert = mujoco.MjvPerturb()

    GREEN, RED = (0.1, 0.9, 0.2, 0.55), (1.0, 0.15, 0.1, 0.7)
    state = {"side": "left", "msg": "L/R: hand · Ctrl/Cmd+click: move · WASD/UP/DOWN: jog",
             "anim": None, "last": (0.0, 0.0), "btn": None, "target": None}

    def _suf():
        return SIDES[state["side"]][1]

    def add_marker(pos, rgba, size):
        """Decorative sphere appended to the current frame's scene."""
        if scene.ngeom >= scene.maxgeom:
            return
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([size, 0, 0], dtype=np.float64),
                            np.asarray(pos, dtype=np.float64),
                            np.eye(3).flatten(),
                            np.asarray(rgba, dtype=np.float32))
        scene.ngeom += 1

    def world_click(x, y, w, h):
        """3D point under the cursor (mjv_select on the rendered scene)."""
        aspect = w / max(1, h)
        relx, rely = x / w, 1.0 - y / h
        selpnt = np.zeros(3)
        geomid = np.zeros(1, dtype=np.int32)
        flexid = np.zeros(1, dtype=np.int32)
        skinid = np.zeros(1, dtype=np.int32)
        body = mujoco.mjv_select(m, d, opt, aspect, relx, rely, scene,
                                 selpnt, geomid, flexid, skinid)
        return selpnt.copy() if body >= 0 or geomid[0] >= 0 else None

    guard_ok = lambda: sess.check_current().ok

    def command_move(pt):
        suf = _suf()
        before = dm.get_arm_deg(suf)
        joints, err, clean = dm.solve_ik_checked(suf, pt, guard_ok)
        if not clean:
            rep = sess.check_current()
            dm.set_arm_deg(suf, before)                # revert — guard says no
            state["target"] = (np.asarray(pt, float), RED)
            if err > 0.03:
                state["msg"] = f"REJECTED: unreachable (residual {err*100:.1f} cm)"
            else:
                state["msg"] = f"REJECTED (guard): {rep.violations[0][:70]}"
            return
        # animate from before -> joints in the render loop (commits a keyframe)
        state["anim"] = (suf, np.array(before), np.array(joints), time.time(), 1.2, True)
        state["target"] = (np.asarray(pt, float), GREEN)
        state["msg"] = f"{state['side']} hand -> ({pt[0]:.2f}, {pt[1]:.2f}, {pt[2]:.2f})  err {err*1000:.0f} mm"

    def jog_move(axis, sign):
        if state["anim"] is not None:                  # don't fight a click animation
            return
        suf = _suf()
        before = dm.get_arm_deg(suf)
        target = dm.ee_point(suf).copy()
        target[axis] += sign * 0.01                    # 1 cm per press
        joints, err, clean = dm.solve_ik_checked(suf, target, guard_ok, err_tol=0.005,
                                                 retry_on_guard_only=True)
        if not clean:
            rep = sess.check_current()
            dm.set_arm_deg(suf, before)
            state["target"] = (target, RED)
            state["msg"] = ("JOG blocked: workspace limit" if err > 0.005 else
                            f"JOG blocked (guard): {rep.violations[0][:70]}")
            return
        # a jog must stay a LOCAL move: refuse solutions that swing the
        # shoulder/elbow to another configuration class (the arm would sweep a
        # big arc through space). Deliberate reconfigs go through Ctrl/Cmd+click.
        dprox = max(abs(a - b) for a, b in zip(joints[:4], before[:4]))
        if dprox > 60.0:
            dm.set_arm_deg(suf, before)
            state["target"] = (target, RED)
            state["msg"] = (f"JOG blocked: needs a large arm reconfiguration "
                            f"(shoulder/elbow swing {dprox:.0f} deg) — Ctrl/Cmd+click "
                            "to command it deliberately, or use the other hand")
            return
        state["target"] = (target, GREEN)
        if max(abs(a - b) for a, b in zip(joints, before)) > 20.0:
            # wrist-dominant reconfiguration — animate instead of snap
            dm.set_arm_deg(suf, before)
            state["anim"] = (suf, np.array(before), np.array(joints), time.time(), 0.6, False)
            state["msg"] = "JOG: elbow reconfiguration (guard-clean)"
            return
        p = dm.ee_point(suf)
        state["msg"] = (f"{state['side']} EE ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
                        "  — SPACE/ENTER commits keyframe")

    def on_mouse_button(w_, button, act, mods):
        x, y = glfw.get_cursor_pos(win)
        state["last"] = (x, y)
        # Cmd+click on macOS (Ctrl+click may be remapped to right-click by the OS)
        if act == glfw.PRESS and button == glfw.MOUSE_BUTTON_LEFT \
                and (mods & (glfw.MOD_CONTROL | glfw.MOD_SUPER)):
            fbw, fbh = glfw.get_framebuffer_size(win)
            sx = fbw / max(1, glfw.get_window_size(win)[0])
            pt = world_click(x * sx, y * sx, fbw, fbh)
            if pt is None:
                state["msg"] = "click hit nothing (aim at ground/robot)"
            else:
                command_move(pt)
            state["btn"] = None
        elif act == glfw.PRESS:
            state["btn"] = button
        else:
            state["btn"] = None

    def on_cursor(w_, x, y):
        lx, ly = state["last"]
        dx, dy = x - lx, y - ly
        state["last"] = (x, y)
        if state["btn"] == glfw.MOUSE_BUTTON_LEFT:
            mujoco.mjv_moveCamera(m, mujoco.mjtMouse.mjMOUSE_ROTATE_H,
                                  dx / 800, dy / 800, scene, cam)
        elif state["btn"] == glfw.MOUSE_BUTTON_RIGHT:
            mujoco.mjv_moveCamera(m, mujoco.mjtMouse.mjMOUSE_MOVE_H,
                                  dx / 800, dy / 800, scene, cam)

    def on_scroll(w_, dx, dy):
        mujoco.mjv_moveCamera(m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * dy, scene, cam)

    JOG_KEYS = {glfw.KEY_W: (0, +1), glfw.KEY_S: (0, -1),   # world X
                glfw.KEY_A: (1, +1), glfw.KEY_D: (1, -1),   # world Y
                glfw.KEY_UP: (2, +1), glfw.KEY_DOWN: (2, -1)}  # world Z

    def on_key(w_, key, sc, act, mods):
        if act not in (glfw.PRESS, glfw.REPEAT):
            return
        if key in JOG_KEYS:                            # jog keys allow hold-to-repeat
            jog_move(*JOG_KEYS[key])
            return
        if act != glfw.PRESS:
            return
        if key in (glfw.KEY_ESCAPE, glfw.KEY_Q):
            glfw.set_window_should_close(win, True)
        elif key == glfw.KEY_L:
            state["side"] = "left"; state["msg"] = "LEFT hand selected"
        elif key == glfw.KEY_R:
            state["side"] = "right"; state["msg"] = "RIGHT hand selected"
        elif key in (glfw.KEY_SPACE, glfw.KEY_ENTER, glfw.KEY_KP_ENTER):
            warn = sess.push()
            state["msg"] = (f"keyframe {len(sess.frames)} committed"
                            + (f" — PATH WARNING {warn[:50]}" if warn else ""))
        elif key == glfw.KEY_H:
            dm.go_home()
            warn = sess.push()
            state["msg"] = "HOME" + (f" — PATH WARNING {warn[:50]}" if warn else "")
        elif key == glfw.KEY_U:
            sess.undo(); state["msg"] = f"undo ({len(sess.frames)} keyframes)"
        elif key == glfw.KEY_E:
            n = sess.export(out_csv)
            bad = sess.path_warnings()
            state["msg"] = (f"exported {out_csv} ({n} keyframes)"
                            + (f" — {len(bad)} PATH WARNINGS, gesture_play may refuse it"
                               if bad else " — preview_gesture.py it, then gesture_play"))

    glfw.set_mouse_button_callback(win, on_mouse_button)
    glfw.set_cursor_pos_callback(win, on_cursor)
    glfw.set_scroll_callback(win, on_scroll)
    glfw.set_key_callback(win, on_key)

    while not glfw.window_should_close(win):
        # animation: interpolate the accepted IK move
        if state["anim"] is not None:
            suf, q0, q1, t0, T, commit = state["anim"]
            s = min(1.0, (time.time() - t0) / T)
            dm.set_arm_deg(suf, list(q0 + (q1 - q0) * s))
            if s >= 1.0:
                state["anim"] = None
                if commit:
                    warn = sess.push()          # keyframe accepted into the session
                    if warn:
                        state["msg"] = f"keyframe OK but PATH WARNING {warn[:60]}"
        fbw, fbh = glfw.get_framebuffer_size(win)
        viewport = mujoco.MjrRect(0, 0, fbw, fbh)
        mujoco.mjv_updateScene(m, d, opt, pert, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, scene)
        if state["target"] is not None:
            add_marker(*state["target"], size=0.015)
        add_marker(dm.ee_point(_suf()), (1.0, 0.85, 0.1, 0.9), size=0.008)
        mujoco.mjr_render(viewport, scene, ctx)
        ee = dm.ee_point(_suf())
        overlay = (f"hand: {state['side'].upper()}   keyframes: {len(sess.frames)}"
                   f"   EE: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")
        mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150,
                           mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport,
                           overlay, state["msg"], ctx)
        glfw.swap_buffers(win)
        glfw.poll_events()
    glfw.terminate()


# --------------------------------------------------------------------------- #
def run_targets(urdf: str, home_json: str, path: str) -> int:
    """Headless regression run over a CSV of Cartesian targets.

    Rows: ``side,x,y,z[,expect]`` — expect in {ok, unreachable, guard},
    default ok. Each target is solved from HOME (order-independent) and
    classified with the same thresholds as the GUI. Non-zero exit if any
    classification differs from its expectation."""
    dm = D1Model(urdf, home_json)
    sess = Session(dm)
    rows = []
    for ln, raw in enumerate(open(path), 1):
        line = raw.split("#")[0].strip()
        if not line:
            continue
        parts = [p.strip().lower() for p in line.split(",")]
        if parts[0] not in SIDES or len(parts) not in (4, 5):
            raise SystemExit(f"{path}:{ln}: expected side,x,y,z[,expect] with side left|right")
        expect = parts[4] if len(parts) == 5 else "ok"
        if expect not in ("ok", "unreachable", "guard"):
            raise SystemExit(f"{path}:{ln}: expect must be ok|unreachable|guard")
        rows.append((parts[0], [float(v) for v in parts[1:4]], expect))
    if not rows:
        raise SystemExit(f"{path}: no targets")
    print(f"{'#':>3} {'side':5} {'target (m)':^24} {'err mm':>7} {'result':^11} {'expect':^11} verdict")
    all_ok = True
    guard_ok = lambda: sess.check_current().ok
    for i, (side, tgt, expect) in enumerate(rows, 1):
        dm.go_home()
        _, err, clean = dm.solve_ik_checked(SIDES[side][1], tgt, guard_ok)
        result = "ok" if clean else ("unreachable" if err > 0.03 else "guard")
        ok = result == expect
        all_ok &= ok
        print(f"{i:3d} {side:5s} ({tgt[0]:6.3f},{tgt[1]:6.3f},{tgt[2]:6.3f})  {err*1000:7.1f}"
              f" {result:^11} {expect:^11} {'PASS' if ok else 'FAIL'}")
    print(f"targets: {'PASS' if all_ok else 'FAIL'} ({len(rows)} targets)")
    return 0 if all_ok else 1


def selftest(urdf: str, home_json: str) -> int:
    """Headless verification: IK reaches offset targets, guard gates, CSV round-trips."""
    dm = D1Model(urdf, home_json)
    sess = Session(dm)
    ok = True
    for side, (sdk, suf) in SIDES.items():
        home_pt = dm.ee_point(suf)
        for off in ([0.05, 0, 0], [0, -0.04 if suf == "L" else 0.04, 0], [0, 0, 0.05]):
            dm.go_home()
            target = home_pt + np.array(off)
            joints, err = dm.solve_ik(suf, target)
            rep = sess.check_current()
            print(f"  {side:5s} {suf} target+{off}: err {err*1000:5.1f} mm "
                  f"guard={'OK' if rep.ok else 'REJECT'}")
            ok &= err < 0.01 and rep.ok
            sess.push()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_selftest_ik_click.csv")
    n = sess.export(out)
    rows = [l for l in open(out) if l.strip() and not l.startswith("#")
            and not l.startswith("duration")]
    ok &= len(rows) == n
    # exported keyframes re-validate through pyguard
    for l in rows:
        vals = [float(x) for x in l.split(",")]
        ok &= sess.guard.check(joints_a_deg=vals[1:8], joints_b_deg=vals[8:15]).ok
    bad = sess.path_warnings()
    if bad:
        print(f"  path warnings: {bad}")
    ok &= not bad
    os.unlink(out)
    print(f"selftest: {'PASS' if ok else 'FAIL'} ({n} keyframes exported+validated)")
    return 0 if ok else 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--urdf", default=DEFAULT_URDF)
    p.add_argument("--home", default=DEFAULT_HOME)
    p.add_argument("--out", default="ik_click_motion.csv")
    p.add_argument("--selftest", action="store_true",
                   help="headless check (IK convergence + guard + CSV), no window")
    p.add_argument("--targets", metavar="CSV",
                   help="headless run over side,x,y,z[,expect] targets, no window")
    a = p.parse_args()
    if a.selftest:
        return selftest(a.urdf, a.home)
    if a.targets:
        return run_targets(a.urdf, a.home, a.targets)
    run_gui(a.urdf, a.home, a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
