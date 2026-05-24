#ifndef IR_CALIBRATION_H
#define IR_CALIBRATION_H

#include <Arduino.h>

// IR sensor distance calibration table — LF and RF only.
// Anchors: 1, 2, 5, 8, 12 cm captured 2026-05-19; 4 cm anchor added
// 2026-05-23. Rows 3, 6, 7, 9, 10, 11 cm are linear interp between anchors.
//
// LF and RF columns are monotonically decreasing in raw count vs distance,
// so piecewise-linear interp is safe. L/R sensors are NOT in this table —
// they're side-aimed (~90° perpendicular as of 2026-05-19), used only for
// centering bias in driveChain(), not for distance estimation.
//
// Front sensors saturate near 3914 at d<2cm — interp reliable from ~2 cm
// out. Below 2 cm treat as "very close" (clamp 10 mm).

namespace IRCal {

constexpr int IR_DIST_MIN_CM   = 1;
constexpr int IR_DIST_MAX_CM   = 12;
constexpr int IR_DIST_ROWS     = IR_DIST_MAX_CM - IR_DIST_MIN_CM + 1;

// Per-sensor, indexed [cm-1][sensor]; sensor order: LF, RF.
constexpr int IR_DIST_TABLE[IR_DIST_ROWS][2] = {
    /*  1 cm */ { 3914, 3914 },
    /*  2 cm */ { 3878, 3878 },
    /*  3 cm */ { 3841, 3684 },   // interp 2cm↔4cm
    /*  4 cm */ { 3803, 3489 },
    /*  5 cm */ { 2982, 2464 },
    /*  6 cm */ { 2623, 2212 },   // interp
    /*  7 cm */ { 2264, 1960 },   // interp
    /*  8 cm */ { 1905, 1708 },
    /*  9 cm */ { 1757, 1607 },   // interp
    /* 10 cm */ { 1609, 1505 },   // interp
    /* 11 cm */ { 1460, 1404 },   // interp
    /* 12 cm */ { 1312, 1302 },
};

// Interp a single front-sensor raw reading -> mm via piecewise-linear
// lookup on its own column of IR_DIST_TABLE. sensorIdx MUST be 0 (LF)
// or 1 (RF).
inline float interpFrontSensorMM(int raw, int sensorIdx) {
    int top = IR_DIST_TABLE[0][sensorIdx];
    int bot = IR_DIST_TABLE[IR_DIST_ROWS - 1][sensorIdx];
    if (raw >= top) return (float)(IR_DIST_MIN_CM * 10);
    if (raw <= bot) return (float)(IR_DIST_MAX_CM * 10);
    for (int i = 0; i < IR_DIST_ROWS - 1; i++) {
        int hi = IR_DIST_TABLE[i][sensorIdx];
        int lo = IR_DIST_TABLE[i + 1][sensorIdx];
        if (raw <= hi && raw >= lo) {
            float frac = (float)(hi - raw) / (float)(hi - lo);
            float cm   = (IR_DIST_MIN_CM + i) + frac;
            return cm * 10.0f;
        }
    }
    return (float)(IR_DIST_MAX_CM * 10);
}

// Per-sensor interp of LF and RF, fused as min — closer sensor wins.
// Handles off-axis approach (one sensor saturated, other not) and skewed
// front walls (returns nearest-corner mm, safer for crash brake than mean).
// Returns mm so callers can do arithmetic without rescale.
inline float estimateFrontDistMM(int lf, int rf) {
    float lfMm = interpFrontSensorMM(lf, 0);
    float rfMm = interpFrontSensorMM(rf, 1);
    return (lfMm < rfMm) ? lfMm : rfMm;
}

} // namespace IRCal

#endif
