#ifndef IR_CALIBRATION_H
#define IR_CALIBRATION_H

#include <Arduino.h>

// IR sensor distance calibration table.
// Captured 2026-05-17 via test/sensor-cal-ble.cpp `d 1`..`d 9` commands.
// Robot placed nose-toward front wall in corridor (side walls also
// present, hence non-zero L/R columns). 32-sample mean per cell.
//
// Use IR_DIST_FRONT_AVG[N-1] for "(LF+RF)/2 expected when front wall is
// at N cm". Use IR_DIST_TABLE for per-sensor raw values.
//
// Front sensors saturate near 3900 at d<3cm — interpolation reliable
// from ~3 cm out. Below 3 cm treat as "very close".

namespace IRCal {

constexpr int IR_DIST_MIN_CM   = 1;
constexpr int IR_DIST_MAX_CM   = 9;
constexpr int IR_DIST_ROWS     = IR_DIST_MAX_CM - IR_DIST_MIN_CM + 1;

// Per-sensor, indexed [cm-1][sensor]; sensor order: LF, L, R, RF.
constexpr int IR_DIST_TABLE[IR_DIST_ROWS][4] = {
    /* 1 cm */ { 3921, 1753, 2364, 3906 },
    /* 2 cm */ { 3880, 1785, 1714, 3827 },
    /* 3 cm */ { 3819, 1680, 1637, 3521 },
    /* 4 cm */ { 3515, 1635, 1559, 2760 },
    /* 5 cm */ { 2866, 1547, 1469, 2237 },
    /* 6 cm */ { 2428, 1525, 1426, 1957 },
    /* 7 cm */ { 2123, 1422, 1511, 1755 },
    /* 8 cm */ { 1882, 1437, 1478, 1619 },
    /* 9 cm */ { 1719, 1421, 1500, 1504 },
};

// Pre-averaged front pair (LF+RF)/2 per cm — what wallFront() effectively
// integrates over its OR threshold.
constexpr int IR_DIST_FRONT_AVG[IR_DIST_ROWS] = {
    3914, 3854, 3670, 3138, 2552, 2193, 1939, 1751, 1612
};

// Convert measured (lf+rf)/2 -> distance in mm via piecewise-linear
// interpolation on IR_DIST_FRONT_AVG. Monotonically decreasing table,
// so reading above table[0] -> clamp 10mm, below table[last] -> clamp 90mm.
// Returns mm, not cm, so caller can do arithmetic without rescale.
inline float estimateFrontDistMM(int lf, int rf) {
    float avg = 0.5f * (lf + rf);
    if (avg >= IR_DIST_FRONT_AVG[0])                return 10.0f;
    if (avg <= IR_DIST_FRONT_AVG[IR_DIST_ROWS - 1]) return 90.0f;
    for (int i = 0; i < IR_DIST_ROWS - 1; i++) {
        int hi = IR_DIST_FRONT_AVG[i];
        int lo = IR_DIST_FRONT_AVG[i + 1];
        if (avg <= hi && avg >= lo) {
            float frac = (float)(hi - avg) / (float)(hi - lo);
            float cm   = (IR_DIST_MIN_CM + i) + frac;
            return cm * 10.0f;
        }
    }
    return 90.0f;
}

} // namespace IRCal

#endif
