# Frozen reference artifacts

Files here are **not part of the kit**. They are copies of retired sources,
kept only because a test in this repo still pins live numbers against them.
Nothing imports, compiles or ships them.

| file | what it is | why it is still here | when it goes |
|---|---|---|---|
| `safety_zones.h` | The authored C++ D1 dual-arm safety geometry, from d1-sdk `devices/omakase_arm/include/omakase_arm/safety_zones.h` @ `47a6337`. | `src/manipulation_kit/config/safety_zones.json` is a **derived export** of these numbers, and `tests/guard/test_safety_zones_export.py` asserts field by field that it still is. This header is the only place the numbers were ever authored; without it the JSON becomes a hand-maintained copy nobody can check. | When `d1-firmwared`'s Rust guard publishes its own authored source, repoint `HEADER` in that test at it and delete this copy. |

What deliberately did **not** come with it: the C++ wrapper, the examples,
`numeric_ik`, `collision_model.h` and the vendor SDK. Hardware and the compiled
guard are `exp--d1-firmware`'s job now.

**Redaction.** The arm vendor's name and model designation were removed from
this copy on 2026-09-10, along with every other occurrence in the repository
(Shu, CTO: the public kit does not name the arm vendor). Only comment prose
changed — every number, type and function here is still the header's, which is
what `test_safety_zones_export.py` reads.
