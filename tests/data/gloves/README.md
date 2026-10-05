# Glove stream fixtures

Test inputs for `tests/gloves/`. They are derived from reference recordings
of two glove drivers' `teleop_gloves.pose_stream.v1` output. Every key, the
key order and every number are the recordings'. The following fields were
replaced with neutral values:

- free text (`note`, `caveat`, `source`, `label`);
- the publishing driver's name (`driver`, now `glove-driver`);
- the per-hand driver's sensor node names (`node-1` .. `node-10`, with the
  same node always mapped to the same replacement);
- that driver's model id (now `litchibot/glove`).

| file | what it is |
|---|---|
| `device_wide.pose_stream.v1.jsonl` | golden vectors of the device-wide dialect: a hello, a UDCAP glove's channel declaration (one for the whole device) and one pose frame. Each line is `{"name", "family", "role", "wire", "expect"}`, and `wire` is the packet. |
| `per_hand.session_head.jsonl` | the LitchiBot glove driver's per-hand dialect: a hello, the right hand's declaration and five pose frames. The frames were solved from a recorded IMU session. |
| `per_hand.dropout.jsonl` | the same driver with the ring finger's distal sensor node silent, so `ring.pip_flex`, `ring.dip_flex` and `ring.total_flex` are absent from every frame. |

`tests/gloves/test_v1.py::test_derived_fixtures_decode_like_the_originals`
checks that each fixture decodes exactly like its reference recording. Node
names are compared by count only, because they were replaced. The test runs
when `MKIT_GLOVE_REFERENCE_DIR` holds the references under these names, with
the full session as `per_hand.session.jsonl`; otherwise it skips. The same
directory's `channels.rs` (the encoder's channel table) feeds
`tests/gloves/test_channels.py::test_channels_match_the_rust_table`.
