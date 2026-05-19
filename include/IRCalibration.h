#ifndef IR_CALIBRATION_H
#define IR_CALIBRATION_H

#include <Arduino.h>

// IR sensor distance calibration table.
// Captured 2026-05-17 via test/sensor-cal-ble.cpp `d 1`..`d 9` commands.
// Robot placed nose-toward front wall in corridor (side walls also
// present, hence non-zero L/R columns). 32-sample mean per cell.
//
// LF (col 0) and RF (col 3) columns are monotonically decreasing in raw
// count vs distance — usable for piecewise-linear interp. L (col 1) and
// R (col 2) columns are NOT monotonic (sensor was at ~30° forward-bias
// during capture and geometry changed 2026-05-19 to ~90° perpendicular;
// columns are stale anyway).
//
// Front sensors saturate near 3900 at d<3cm — interpolation reliable
// from ~3 cm out. Below 3 cm treat as "very close" (clamp 10mm).

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

// Interp a single front-sensor raw reading -> mm via piecewise-linear
// lookup on its own column of IR_DIST_TABLE. sensorIdx MUST be 0 (LF)
// or 3 (RF); L/R columns are non-monotonic and unsafe to interp.
inline float interpFrontSensorMM(int raw, int sensorIdx) {
    int top = IR_DIST_TABLE[0][sensorIdx];
    int bot = IR_DIST_TABLE[IR_DIST_ROWS - 1][sensorIdx];
    if (raw >= top) return 10.0f;
    if (raw <= bot) return 90.0f;
    for (int i = 0; i < IR_DIST_ROWS - 1; i++) {
        int hi = IR_DIST_TABLE[i][sensorIdx];
        int lo = IR_DIST_TABLE[i + 1][sensorIdx];
        if (raw <= hi && raw >= lo) {
            float frac = (float)(hi - raw) / (float)(hi - lo);
            float cm   = (IR_DIST_MIN_CM + i) + frac;
            return cm * 10.0f;
        }
    }
    return 90.0f;
}

// Per-sensor interp of LF and RF, fused as min — closer sensor wins.
// Handles off-axis approach (one sensor saturated, other not) and skewed
// front walls (returns nearest-corner mm, safer for crash brake than mean).
// Returns mm so callers can do arithmetic without rescale.
inline float estimateFrontDistMM(int lf, int rf) {
    float lfMm = interpFrontSensorMM(lf, 0);
    float rfMm = interpFrontSensorMM(rf, 3);
    return (lfMm < rfMm) ? lfMm : rfMm;
}

} // namespace IRCal

#endif
