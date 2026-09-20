"""The committed snapshot of the generated d1-firmwared client.

Everything under here is **machine output**. Do not hand-edit it, and do not
import ``_client.d1fw_api`` directly: go through
:func:`manipulation_kit.executors.firmware.ensure.ensure_client`, which is what
decides whether this snapshot is the client that matches the daemon in front of
you or whether one has to be regenerated from that daemon's own document.

* ``openapi/d1-firmwared.v1.json`` — the document it was generated from, a
  vendored copy; ``SNAPSHOT.json`` records its sha256 and where it came from.
  The firmware serves this document byte for byte at ``GET /openapi.json``,
  which is what makes the hash comparison meaningful.
* ``d1fw_api/`` — ``openapi-python-client`` output, lowered to Python 3.10 by
  ``_gen/compat.py``.

To refresh both after a firmware release::

    mkit-firmware-client refresh --url http://d1-2:4750
    mkit-firmware-client refresh --spec path/to/d1-firmwared.v1.json
"""
