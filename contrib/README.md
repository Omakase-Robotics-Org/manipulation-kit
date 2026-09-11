# contrib

Research code that is **not part of the shipped package**: it is not installed
by the wheel, no runtime module imports it, and it carries dependencies the kit
itself refuses (`mujoco`, torch, a dataset on disk).

It lives here rather than being deleted because the work is real and the notes
in it are the record of what was tried.

| | |
|---|---|
| [`wholebody/`](wholebody) | Whole-body IK for D1 — arms + 0.30 m lift + differential-drive base in one chain. P0 (lift) and P1 (base) are benchmarks/demos against `d1_wholebody.urdf`; P2 is a ΔEE-action policy experiment and expects a dataset. See its README. |

Runnable, supported scripts are in [`../examples/`](../examples); CAD vendoring
tools are in [`../tools/vendoring/`](../tools/vendoring).
