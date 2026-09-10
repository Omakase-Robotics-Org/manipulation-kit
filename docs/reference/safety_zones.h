#ifndef OMAKASE_ARM_SAFETY_ZONES_H_
#define OMAKASE_ARM_SAFETY_ZONES_H_

// D1 dual-arm SELF-COLLISION / BODY-PROXIMITY safety validator.
//
// This is the C++ side of a validator that is mirrored byte-for-byte (same FK,
// same capsule model, same checks, same default zones) in the omakaseos
// authoring viewer (d1_motion_preview.html, JS). It lets the d1-sdk playback
// tools (gesture_play, loadCsv) REFUSE to run a CSV whose dual-arm trajectory
// would drive a link into the other arm or into the torso keep-out volume.
//
// Header-only and std-library-only (like gesture_csv.h) so it builds & unit-
// tests on any host with no robot / no vendor .so.
//
// ---------------------------------------------------------------------------
// MODEL (frame: viewer/URDF frame, z up, y lateral, x forward)
// ---------------------------------------------------------------------------
//   * Each arm is a chain of links; each link is a CAPSULE (segment + radius).
//   * The two arm bases are mounted at the MEASURED shoulder half-width
//     (+/-0.037 m on y, z=0.50 m) -- from the STEP assembly d1_face.step.
//   * Checks (a) arm-arm minimum capsule distance >= min_arm_arm_distance,
//             (b) no arm capsule intersects the torso keep-out box (with
//                 min_body_clearance margin),
//             (c) every joint angle within its D1 arm limit.
//
// The kinematic chain below is kept IDENTICAL to d1_dual.urdf. If the URDF
// mount/joints change, update both (and the JS mirror).
// ---------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "omakase_arm/gesture_csv.h"  // Pose, kJointsPerArm, kDualJoints

namespace omakase_arm {
namespace safety {

using gesture::Pose;
using gesture::kDualJoints;
using gesture::kJointsPerArm;

// --- tiny 3-vector / 4x4 homogeneous transform (column-major-free, row math) -
struct Vec3 {
    double x = 0, y = 0, z = 0;
};
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

// 4x4 transform stored as 3x3 R + t.
struct Tf {
    std::array<std::array<double, 3>, 3> R{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Vec3 t{};
};

inline Tf mul(const Tf& a, const Tf& b)
{
    Tf o;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            o.R[i][j] = a.R[i][0] * b.R[0][j] + a.R[i][1] * b.R[1][j] +
                        a.R[i][2] * b.R[2][j];
        }
    }
    o.t.x = a.R[0][0] * b.t.x + a.R[0][1] * b.t.y + a.R[0][2] * b.t.z + a.t.x;
    o.t.y = a.R[1][0] * b.t.x + a.R[1][1] * b.t.y + a.R[1][2] * b.t.z + a.t.y;
    o.t.z = a.R[2][0] * b.t.x + a.R[2][1] * b.t.y + a.R[2][2] * b.t.z + a.t.z;
    return o;
}

inline Vec3 apply(const Tf& a, const Vec3& p)
{
    return {a.R[0][0] * p.x + a.R[0][1] * p.y + a.R[0][2] * p.z + a.t.x,
            a.R[1][0] * p.x + a.R[1][1] * p.y + a.R[1][2] * p.z + a.t.y,
            a.R[2][0] * p.x + a.R[2][1] * p.y + a.R[2][2] * p.z + a.t.z};
}

// Fixed-axis URDF rpy (roll about x, pitch about y, yaw about z): R = Rz*Ry*Rx.
inline Tf fromRpyXyz(double r, double p, double y, double tx, double ty, double tz)
{
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    Tf o;
    o.R[0][0] = cy * cp;
    o.R[0][1] = cy * sp * sr - sy * cr;
    o.R[0][2] = cy * sp * cr + sy * sr;
    o.R[1][0] = sy * cp;
    o.R[1][1] = sy * sp * sr + cy * cr;
    o.R[1][2] = sy * sp * cr - cy * sr;
    o.R[2][0] = -sp;
    o.R[2][1] = cp * sr;
    o.R[2][2] = cp * cr;
    o.t = {tx, ty, tz};
    return o;
}

// Rotation about local z by angle a (revolute joint actuation).
inline Tf rotZ(double a)
{
    Tf o;
    const double c = std::cos(a), s = std::sin(a);
    o.R = {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
    return o;
}

// --- one joint's fixed origin transform (xyz + rpy), from d1_dual.urdf ---
struct JointDef {
    double rx, ry, rz;  // rpy
    double tx, ty, tz;  // xyz
};

// D1 arm arm chain Base -> Link1..Link7 (one arm). IDENTICAL to the URDF
// Joint*_R / Joint*_L origins. radius is the capsule radius of the link that
// this joint's origin starts (used for the capsule between consecutive frames).
inline const std::array<JointDef, kJointsPerArm>& armChain()
{
    static const std::array<JointDef, kJointsPerArm> chain = {{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.1586},          // Joint1: Base->Link1
        {1.5708, 0.0, 0.0, 0.0, 0.0, 0.0},          // Joint2
        {-1.5708, 0.0, 0.0, 0.0, 0.264, 0.0},       // Joint3
        {-1.5708, 0.0, 3.1416, 0.018, 0.0, 0.0},    // Joint4
        {-1.5708, 0.0, 3.1416, 0.018, -0.264, 0.0}, // Joint5
        {1.5708, -1.5708, 0.0, 0.0, 0.0, 0.0},      // Joint6
        {1.5708, -1.5708, 0.0, 0.0, 0.0, 0.0},      // Joint7
    }};
    return chain;
}

// Per-link capsule radius (m), index 0 = Base, 1..7 = Link1..Link7.
inline const std::array<double, kJointsPerArm + 1>& capsuleRadii()
{
    static const std::array<double, kJointsPerArm + 1> r = {
        0.055, 0.05, 0.045, 0.04, 0.04, 0.035, 0.03, 0.03};
    return r;
}

// Mount transform for each arm base in the torso/viewer frame.
//   ArmSide::A (R tree): y=+0.037, z=0.50, rpy(-pi/2,0,0)
//   ArmSide::B (L tree): y=-0.037, z=0.50, rpy(+pi/2,0,0)
inline Tf mountA() { return fromRpyXyz(-1.5708, 0, 0, 0.0, 0.037, 0.50); }
inline Tf mountB() { return fromRpyXyz(1.5708, 0, 0, 0.0, -0.037, 0.50); }

struct Capsule {
    Vec3 a, b;        // segment endpoints (m)
    double radius;    // m
    int arm;          // 0 = ArmSide A, 1 = ArmSide B
    int link;         // 0 = Base..Link1 segment index
};

// Forward-kinematics: produce the joint-origin points of one arm in the torso
// frame given the 7 joint angles (DEGREES). Returns 8 points: Base origin then
// Link1..Link7 origins.
inline std::array<Vec3, kJointsPerArm + 1> armPoints(const Tf& mount,
                                                     const double* deg7)
{
    std::array<Vec3, kJointsPerArm + 1> pts;
    Tf cur = mount;
    pts[0] = cur.t;  // base origin
    const auto& chain = armChain();
    for (int i = 0; i < kJointsPerArm; ++i) {
        const JointDef& jd = chain[i];
        const Tf fixed = fromRpyXyz(jd.rx, jd.ry, jd.rz, jd.tx, jd.ty, jd.tz);
        cur = mul(cur, mul(fixed, rotZ(deg7[i] * M_PI / 180.0)));
        pts[i + 1] = cur.t;
    }
    return pts;
}

// Build the capsule list for a full dual-arm pose (14 deg).
inline std::vector<Capsule> capsulesForPose(const Pose& pose)
{
    std::vector<Capsule> caps;
    const auto& rad = capsuleRadii();
    auto build = [&](const Tf& mount, const double* deg7, int arm) {
        const auto pts = armPoints(mount, deg7);
        for (int i = 0; i < kJointsPerArm; ++i) {
            // capsule for the link between pts[i] and pts[i+1]; use the larger
            // of the two adjacent radii (conservative).
            const double r = std::max(rad[i], rad[i + 1]);
            caps.push_back({pts[i], pts[i + 1], r, arm, i});
        }
    };
    build(mountA(), pose.data(), 0);
    build(mountB(), pose.data() + kJointsPerArm, 1);
    return caps;
}

// --- geometry: distance between two segments, and segment-to-AABB ------------
inline double clamp01(double v) { return std::min(1.0, std::max(0.0, v)); }

inline double segSegDistance(const Vec3& p1, const Vec3& q1, const Vec3& p2,
                             const Vec3& q2)
{
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);
    double s, t;
    const double kEps = 1e-12;
    if (a <= kEps && e <= kEps) {
        return norm(p1 - p2);
    }
    if (a <= kEps) {
        s = 0.0;
        t = clamp01(f / e);
    } else {
        const double c = dot(d1, r);
        if (e <= kEps) {
            t = 0.0;
            s = clamp01(-c / a);
        } else {
            const double b = dot(d1, d2);
            const double denom = a * e - b * b;
            s = (denom > kEps) ? clamp01((b * f - c * e) / denom) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = clamp01(-c / a);
            } else if (t > 1.0) {
                t = 1.0;
                s = clamp01((b - c) / a);
            }
        }
    }
    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    return norm(c1 - c2);
}

struct Box {
    Vec3 lo, hi;
};

inline double pointBoxDistance(const Vec3& p, const Box& box)
{
    double dx = std::max({box.lo.x - p.x, 0.0, p.x - box.hi.x});
    double dy = std::max({box.lo.y - p.y, 0.0, p.y - box.hi.y});
    double dz = std::max({box.lo.z - p.z, 0.0, p.z - box.hi.z});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Min distance from a segment to an AABB (sampled; conservative & branch-free).
inline double segBoxDistance(const Vec3& a, const Vec3& b, const Box& box,
                             int samples = 16)
{
    double best = 1e9;
    for (int i = 0; i <= samples; ++i) {
        const double s = static_cast<double>(i) / samples;
        const Vec3 p = a + (b - a) * s;
        best = std::min(best, pointBoxDistance(p, box));
    }
    return best;
}

// --- zones config ------------------------------------------------------------
struct Zones {
    Box torso{{-0.075, -0.075, 0.0}, {0.075, 0.075, 0.49}};
    double min_arm_arm = 0.06;
    double min_body_clearance = 0.03;
    // joint limits (deg): J1..J7
    std::array<std::array<double, 2>, kJointsPerArm> limits = {{
        {-173, 173}, {-118, 118}, {-173, 173}, {-145, 45},
        {-173, 173}, {-60, 60}, {-90, 90}}};
};

struct Violation {
    std::string kind;   // "arm-arm" | "torso" | "joint-limit"
    std::string detail; // human-readable
    double clearance = 0.0;
};

struct CheckResult {
    bool safe = true;
    std::vector<Violation> violations;
};

// Validate a single dual-arm pose against the zones. Skips the base capsule
// (the arm's own base sits inside/next to the torso by construction).
inline CheckResult checkPose(const Pose& pose, const Zones& z = Zones())
{
    CheckResult res;

    // (c) joint limits
    for (int arm = 0; arm < 2; ++arm) {
        for (int j = 0; j < kJointsPerArm; ++j) {
            const double v = pose[arm * kJointsPerArm + j];
            if (v < z.limits[j][0] - 1e-6 || v > z.limits[j][1] + 1e-6) {
                res.safe = false;
                res.violations.push_back(
                    {"joint-limit",
                     "arm " + std::string(arm == 0 ? "A" : "B") + " J" +
                         std::to_string(j + 1) + "=" + std::to_string(v) +
                         " deg out of [" + std::to_string(z.limits[j][0]) +
                         "," + std::to_string(z.limits[j][1]) + "]",
                     0.0});
            }
        }
    }

    const std::vector<Capsule> caps = capsulesForPose(pose);

    // (a) arm-arm: every capsule of arm A vs every capsule of arm B.
    //     Skip the Base->Link1 segment (link 0) of either arm: those are the
    //     fixed shoulder flanges, mounted 74 mm apart by construction, so they
    //     "overlap" within the conservative capsule radii but are not a moving
    //     collision risk. The actuated links (1..6) are the real hazard.
    double min_arm_arm = 1e9;
    for (const auto& ca : caps) {
        if (ca.arm != 0 || ca.link == 0) continue;
        for (const auto& cb : caps) {
            if (cb.arm != 1 || cb.link == 0) continue;
            const double d =
                segSegDistance(ca.a, ca.b, cb.a, cb.b) - ca.radius - cb.radius;
            min_arm_arm = std::min(min_arm_arm, d);
        }
    }
    if (min_arm_arm < z.min_arm_arm) {
        res.safe = false;
        res.violations.push_back(
            {"arm-arm",
             "arm-arm clearance " + std::to_string(min_arm_arm) + " m < " +
                 std::to_string(z.min_arm_arm) + " m",
             min_arm_arm});
    }

    // (b) torso keep-out: every link capsule (skip the Base segment, link 0)
    //     must stay min_body_clearance outside the box.
    double min_body = 1e9;
    for (const auto& c : caps) {
        if (c.link == 0) continue;  // base segment lives at the shoulder mount
        const double d = segBoxDistance(c.a, c.b, z.torso) - c.radius;
        min_body = std::min(min_body, d);
    }
    if (min_body < z.min_body_clearance) {
        res.safe = false;
        res.violations.push_back(
            {"torso",
             "body clearance " + std::to_string(min_body) + " m < " +
                 std::to_string(z.min_body_clearance) + " m",
             min_body});
    }

    return res;
}

// Validate every keyframe of a gesture. Returns the first failure (with the
// keyframe index baked into the detail) or a safe result.
inline CheckResult checkGesture(const gesture::Gesture& g, const Zones& z = Zones())
{
    for (size_t k = 0; k < g.keyframes.size(); ++k) {
        CheckResult r = checkPose(g.keyframes[k].positions, z);
        if (!r.safe) {
            for (auto& v : r.violations) {
                v.detail = "keyframe " + std::to_string(k) + ": " + v.detail;
            }
            return r;
        }
    }
    return CheckResult{};
}

// Validate the SMOOTH playback path by densely sampling it (the same path the
// robot actually streams). This catches collisions that appear BETWEEN
// keyframes (e.g. the Catmull-Rom overshoot), not just at the keyframes.
inline CheckResult checkSmoothPath(const gesture::SmoothPath& path,
                                   double sample_hz = 50.0,
                                   const Zones& z = Zones())
{
    const double total = path.totalDuration();
    if (total <= 0.0) {
        return CheckResult{};
    }
    const double dt = 1.0 / sample_hz;
    for (double t = 0.0; t <= total + 1e-9; t += dt) {
        const Pose p = gesture::samplePath(path, std::min(t, total));
        CheckResult r = checkPose(p, z);
        if (!r.safe) {
            for (auto& v : r.violations) {
                v.detail = "t=" + std::to_string(t) + "s: " + v.detail;
            }
            return r;
        }
    }
    return CheckResult{};
}

}  // namespace safety
}  // namespace omakase_arm

#endif  // OMAKASE_ARM_SAFETY_ZONES_H_
