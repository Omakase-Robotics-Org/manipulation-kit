"""manipulation_kit.gloves — the glove side of the glove/hand seam.

Data gloves (LitchiBot, UDCAP, …) teleoperate robot hands (Leadshine DH116S,
RobotEra XHAND1, …) through three layers, so that N gloves and M hands cost
N + M pieces of code instead of N x M:

1. **Glove driver** (outside this package). Publishes
   ``teleop_gloves.pose_stream.v1``: anatomical channels in RADIANS, plus a
   ``channel_declaration`` saying which channels it has and how each is
   obtained. :mod:`.v1` decodes it into :class:`HandAngles`.
2. **Normalisation, per operator and per glove.**
   :class:`ChannelCalibration` turns radians into flexion in [0, 1] with one
   :class:`ChannelRange` per channel (lo -> 0, hi -> 1, linear, clamped,
   optional invert; a bipolar swing channel lands neutral at 0.5). The result
   is the canonical :class:`HandPose`. A missing reading stays ``None`` and
   never becomes 0.0, which is a fully open finger.
3. **Retarget, per hand** (:mod:`manipulation_kit.hands`). A hand's map asks
   for the channels it needs (``required_channels()``) and turns a
   :class:`HandPose` into that hand's joint targets, in the order and units of
   the ``d1-firmwared`` end-effector descriptor, holding (``None``) any joint
   whose inputs are missing.

Like the rest of the kit this package opens no socket: it decodes packets a
caller has already received. :mod:`.channels` is the vocabulary both sides
share.
"""
from .calibration import (  # noqa: F401
    CALIBRATION_SCHEMA,
    ChannelCalibration,
    ChannelRange,
    UnknownChannels,
    load_ranges,
    save_ranges,
)
from .channels import (  # noqa: F401
    ALL_CHANNELS,
    CHANNEL_NAMES,
    CHANNELS,
    EXTENSION_CHANNELS,
    FINGERS,
    ChannelKind,
    ChannelSpec,
    spec_of,
)
from .pose import HandAngles, HandPose  # noqa: F401
from .v1 import (  # noqa: F401
    Declaration,
    DeclarationError,
    PoseStreamDecoder,
    Provenance,
    parse_declaration,
)
