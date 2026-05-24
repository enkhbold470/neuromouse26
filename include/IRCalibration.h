#ifndef IR_CALIBRATION_H
#define IR_CALIBRATION_H

#include <Arduino.h>

// IR sensor distance calibration table — LF and RF only.
// Re-captured 2026-05-19 at 5 anchor points (1, 2, 5, 8, 12 cm); rows for
// 3, 4, 6, 7, 9, 10, 11 cm are linear interp between anchors.
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
    /*  1 cm */ { 1895, 2260 },
    /*  2 cm */ { 1235, 1288 },
    /*  3 cm */ { 865, 839 },   // interp
    /*  4 cm */ { 651, 613 },   // interp
    /*  5 cm */ { 495, 492 },
    /*  6 cm */ { 407, 371 },   // interp
    /*  7 cm */ { 341, 297 },   // interp
    /*  8 cm */ { 281, 247 },
    /*  9 cm */ { 233, 211 },   // interp
    /* 10 cm */ { 191, 177 },   // interp
    /* 11 cm */ { 165, 150 },   // interp
    /* 12 cm */ { 141, 123 },
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
