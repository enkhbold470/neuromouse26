// include/OLED.h — 128×64 SSD1306 menu + run + diagnostic screens.
//
// Owns the U8G2 instance. Setup-time `oled.begin()` is called from main.cpp
// after Wire.begin sets the bus speed.
//
// The encoder objects (leftEnc, rightEnc) are defined in main.cpp; this
// header extern's them for the encoder-test screen.

#ifndef MM26_OLED_H
#define MM26_OLED_H

#include <Arduino.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "Tuning.h"
#include "Battery.h"
#include "IMU.h"
#include "IRSensors.h"
#include "IRCalibration.h"
#include "MotionScript.h"
#include "Pose.h"
#include "MicromouseMaze.h"
#include "MicromouseEncoderPCNT.h"

extern MicromouseMaze maze;

extern MicromouseEncoderPCNT leftEnc;
extern MicromouseEncoderPCNT rightEnc;

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

enum MenuItem {
    M_EXPLORE = 0,
    M_FAST,
    M_FAST_SPEED,
    M_SMOOTH,
    M_ENC,
    M_IR_TEST,
    M_CAL_IR,
    M_MOTOR_TEST,
    M_NVS_CLR,
    M_DUMP_MAZE,
    M_COUNT
};

static const char* MENU_LABELS[M_COUNT] = {
    "Explore",
    "Fast Run",
    "Fast Speed",
    "Mode",            // M_SMOOTH — label rendered dynamically (Smooth ON/OFF)
    "Encoder Test",
    "IR Test",
    "Cal IR",
    "Motor Test",
    "Clear NVS",
    "Dump Maze"
};

static int      menuSel    = M_EXPLORE;
static long     menuEncRef = 0;
constexpr long     ENC_PER_STEP       = 80;
constexpr uint32_t COUNTDOWN_DELAY_MS = 500;

static void drawBatteryTopRight() {
    int pct = batPct();
    char b[8]; snprintf(b, sizeof(b), "%d%%", pct);
    oled.setFont(u8g2_font_5x7_tf);
    int w = oled.getStrWidth(b);
    int xText = 128 - 16 - w - 1;
    int yText = 7;
    oled.drawStr(xText, yText, b);
    int xIcon = 128 - 14, yIcon = 1;
    oled.drawFrame(xIcon, yIcon, 12, 6);
    oled.drawBox  (xIcon + 12, yIcon + 2, 2, 2);
    int fillW = (pct * 10 + 50) / 100;
    if (fillW > 0) oled.drawBox(xIcon + 1, yIcon + 1, fillW, 4);
}

static void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "mm26 flood");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);

    const int LH = 10;
    for (int i = 0; i < VIS; i++) {
        int idx = top + i;
        if (idx >= M_COUNT) break;
        int y = 12 + i * LH;
        const char* lbl = MENU_LABELS[idx];
        char dynBuf[20];
        if (idx == M_SMOOTH) {
            snprintf(dynBuf, sizeof(dynBuf), "Smooth: %s", g_smoothMode ? "ON" : "OFF");
            lbl = dynBuf;
        }
        if (idx == menuSel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, lbl);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, lbl);
        }
    }
    oled.sendBuffer();
}

static void oledRun(long avg, long tgt, long tL, long tR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    const char* phN = (runPhase == PH_FORWARD)         ? "FWD"
                    : (runPhase == PH_CURVE)           ? "CRV"
                    : (runPhase == PH_PIVOT)           ? "PIV"
                    : (runPhase == PH_ALIGN_FRONT)     ? "ALN"
                    : (runPhase == PH_REVERSE_TO_BACK) ? "RTB"
                    : "SPOT";
    const char* dN  = (runTurnDir == TURN_RIGHT) ? "R"
                    : (runTurnDir == TURN_LEFT)  ? "L" : "-";
    const char* modeN = returnHomeMode ? "HOME"
                      : (exploreFwdGoalSaved ? "SWEEP" : "EXPLORE");
    char hdr[28];
    snprintf(hdr, sizeof(hdr), "%s %s/%s %d/%d", modeN, phN, dN, scriptIdx + 1, scriptLen);
    oled.drawStr(0, 8, hdr);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
    snprintf(buf, sizeof(buf), "cell (%d,%d) hd=%c",
             robotRow, robotCol, "NESW"[robotHeading]);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "tL%+5ld tR%+5ld", tL, tR);
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "tgt %ld avg %ld", tgt, avg);
    oled.drawStr(0, 42, buf);
    snprintf(buf, sizeof(buf), "yaw %+.1f", yawDeg);
    oled.drawStr(0, 52, buf);
    int pw = (tgt > 0) ? (int)((labs(avg) * 124L) / tgt) : 0;
    if (pw < 0) pw = 0; if (pw > 124) pw = 124;
    oled.drawFrame(0, 54, 126, 8);
    oled.drawBox(1, 55, pw, 6);
    oled.sendBuffer();
}

static void oledTerminal(const char* title, const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, title);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    if (msg) oled.drawStr(0, 32, msg);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
    snprintf(buf, sizeof(buf), "cell (%d,%d)", robotRow, robotCol);
    oled.drawStr(0, 50, buf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

// Shown during EXPLORE_THINK between moves (always-on explore indication).
static void oledExploreThink(const char* status) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    const char* modeN = returnHomeMode ? "HOME"
                      : (exploreFwdGoalSaved ? "SWEEP" : "EXPLORE");
    oled.drawStr(0, 8, modeN);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    if (status) oled.drawStr(0, 30, status);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "(%d,%d) hd=%c", robotRow, robotCol, "NESW"[robotHeading]);
    oled.drawStr(0, 46, buf);
    int uv = maze.countUnvisited(MAZE_ROWS, MAZE_COLS);
    snprintf(buf, sizeof(buf), "unvisited=%d", uv);
    oled.drawStr(0, 58, buf);
    oled.sendBuffer();
}

static void oledEncTest() {
    long tL = leftEnc.getTicks();
    long tR = rightEnc.getTicks();
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Enc Test");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L %ld", tL); oled.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "R %ld", tR); oled.drawStr(0, 44, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=reset+back");
    oled.sendBuffer();
}

static void oledCalIrResult(int newCalL, int newCalR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Cal IR Done");
    oled.drawHLine(0, 10, 128);
    char buf[32];
    snprintf(buf, sizeof(buf), "L45: %4d", newCalL);
    oled.drawStr(0, 26, buf);
    snprintf(buf, sizeof(buf), "R45: %4d", newCalR);
    oled.drawStr(0, 40, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "paste->PinConfig.h btn=ok");
    oled.sendBuffer();
}

static void oledIrTest() {
    // irVal[] filled by sampleIR() before this is called.
    // PAIRS order: 0=LF  1=L45  2=R45  3=RF
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "IR Test");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    char buf[32];
    snprintf(buf, sizeof(buf), "LF:%4d  RF:%4d", irVal[0], irVal[3]);
    oled.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "R45:%4d L45:%4d", irVal[2], irVal[1]);
    oled.drawStr(0, 36, buf);
    float frontMm = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
    snprintf(buf, sizeof(buf), "front %.0f mm", frontMm);
    oled.drawStr(0, 48, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

static void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "STARTING");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}

static void oledFastSpeedEdit() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Fast Speed");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d tps", (int)fastRunCruiseTps);
    oled.drawStr(0, 30, buf);
    float mmps = fastRunCruiseTps * 180.0f / (float)CELL_TICKS;
    snprintf(buf, sizeof(buf), "%d mm/s", (int)mmps);
    oled.drawStr(0, 48, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "spin=adj  btn=save");
    oled.sendBuffer();
}

// Auto-calibrate gyro bias before every run. Robot must be still during the
// ~600 ms sample window; the post-button-press 300 ms settle delay handles
// the hand vibration from the button release.
static void autoCalGyroBeforeStart() {
    if (!imuReady) {
        Serial.println("[GCAL/AUTO] skipped (no IMU)");
        return;
    }
    oledTerminal("CAL GYRO", "stay still");
    delay(300);
    calibrateGyroBias(300, 2);
    yawDeg       = 0.0f;
    yawTargetDeg = 0.0f;
    Serial.printf("[GCAL/AUTO] bias=%.4f deg/s\n", gyroBiasZ);
}

#endif
