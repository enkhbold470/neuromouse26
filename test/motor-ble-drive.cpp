// test/motor-ble-drive.cpp
//
// OLED menu — right encoder scrolls, BUTTON_1 selects.
// No BLE. No Serial commands.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── OLED ──────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ── Motors + Encoders ─────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, true);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, true);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── Status line (for in-action feedback) ──────────────────────────────────────
static char statusLine[22] = "";

static void setStatus(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vsnprintf(statusLine, sizeof(statusLine), fmt, args);
    va_end(args);
}

// ── Helpers ───────────────────────────────────────────────────────────────────
void encodersEnable()  { leftEnc.reset(); rightEnc.reset(); }
void stopMotors()      { leftMotor.brake(); rightMotor.brake(); }

bool buttonEdge() {
    static bool last = HIGH;
    bool cur = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

// ── Motion (same as before but logs to statusLine) ────────────────────────────
static void moveCells(int n) {
    long target = TICKS_PER_CELL * (long)n - COAST_COMP_TICKS;
    encodersEnable();
    setStatus("MOVE %d", n);

    unsigned long startMs = millis();
    while (true) {
        long tL = leftEnc.getTicks();
        long tR = rTicks();
        long remL = target - tL;
        long remR = target - tR;
        bool lDone = (remL <= 0);
        bool rDone = (remR <= 0);

        if (lDone && rDone) {
            stopMotors();
            setStatus("done L%ld R%ld", tL, tR);
            return;
        }

        int err = (int)(tL - tR);
        int baseL = lDone ? 0 : (remL > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remL, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int baseR = rDone ? 0 : (remR > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remR, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int pwmL, pwmR;
        if (!lDone && !rDone && remL > DECEL_TICKS && remR > DECEL_TICKS) {
            pwmL = constrain(baseL - (int)(err * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(baseR + (int)(err * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        } else {
            pwmL = baseL; pwmR = baseR;
        }
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * n)) {
            stopMotors();
            setStatus("TIMEOUT");
            return;
        }
    }
}

void turnRight() {
    encodersEnable();
    setStatus("TURN R");
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    unsigned long t = millis();
    while (leftEnc.getTicks() < TURN_TICKS_90_R) {
        if (millis() - t > 2000) break;
    }
    stopMotors();
    setStatus("R done L%ld", (long)leftEnc.getTicks());
}

void turnLeft() {
    encodersEnable();
    setStatus("TURN L");
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    unsigned long t = millis();
    while (rTicks() < TURN_TICKS_90_L) {
        if (millis() - t > 2000) break;
    }
    stopMotors();
    setStatus("L done R%ld", rTicks());
}

void runSequence() {
    setStatus("SEQ");
    moveCells(5); delay(250);
    turnRight();  delay(250);
    moveCells(1); delay(250);
    turnRight();  delay(250);
    moveCells(5); delay(250);
    turnLeft();   delay(250);
    moveCells(1); delay(250);
    turnLeft();   delay(250);
    moveCells(5);
    setStatus("SEQ done");
}

void freeRun() {
    encodersEnable();
    setStatus("FREE btn=stop");
    while (true) {
        if (buttonEdge()) {
            stopMotors();
            setStatus("FREE stop");
            return;
        }
        long tL = leftEnc.getTicks(), tR = rTicks();
        int  err = (int)(tL - tR);
        leftMotor.drive( constrain(DRIVE_PWM - (int)(err * BALANCE_KP), 200, MOTOR_PWM_MAX));
        rightMotor.drive(constrain(DRIVE_PWM + (int)(err * BALANCE_KP), 200, MOTOR_PWM_MAX));
    }
}

void encoderSpinTest() {
    encodersEnable();
    setStatus("ENC spin 1s");
    leftMotor.drive(220); rightMotor.drive(220);
    delay(1000);
    stopMotors();
    long tL = leftEnc.getTicks(), tR = rTicks();
    setStatus("L%ld R%ld", tL, tR);
}

void diagL(int pwm)  { leftMotor.drive(pwm);  delay(800); leftMotor.coast();  setStatus("L done"); }
void diagR(int pwm)  { rightMotor.drive(pwm); delay(800); rightMotor.coast(); setStatus("R done"); }
void resetEnc()      { encodersEnable(); setStatus("enc=0"); }
void showStatus() {
    setStatus("L%ld R%ld", (long)leftEnc.getTicks(), rTicks());
}

// ── Menu ──────────────────────────────────────────────────────────────────────
enum {
    M_MOVE1=0, M_MOVE2, M_MOVE3, M_MOVE4, M_MOVE5,
    M_FREE, M_STOP, M_RESET, M_SEQ,
    M_TURN_R, M_TURN_L,
    M_LF, M_LB, M_RF, M_RB,
    M_ENC_TEST, M_STATUS,
    M_COUNT
};

static const char* MENU[M_COUNT] = {
    "Move 1 cell", "Move 2 cells", "Move 3 cells", "Move 4 cells", "Move 5 cells",
    "Free run", "Stop", "Reset enc", "Sequence",
    "Turn R 90", "Turn L 90",
    "L fwd", "L rev", "R fwd", "R rev",
    "Enc spin test", "Status"
};

static int  menuSel    = 0;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;

static void dispatch(int sel) {
    switch (sel) {
        case M_MOVE1: moveCells(1); break;
        case M_MOVE2: moveCells(2); break;
        case M_MOVE3: moveCells(3); break;
        case M_MOVE4: moveCells(4); break;
        case M_MOVE5: moveCells(5); break;
        case M_FREE:  freeRun(); break;
        case M_STOP:  stopMotors(); setStatus("STOP"); break;
        case M_RESET: resetEnc(); break;
        case M_SEQ:   runSequence(); break;
        case M_TURN_R: turnRight(); break;
        case M_TURN_L: turnLeft(); break;
        case M_LF: setStatus("L fwd"); diagL( 500); break;
        case M_LB: setStatus("L rev"); diagL(-500); break;
        case M_RF: setStatus("R fwd"); diagR( 500); break;
        case M_RB: setStatus("R rev"); diagR(-500); break;
        case M_ENC_TEST: encoderSpinTest(); break;
        case M_STATUS: showStatus(); break;
    }
}

// 5-row viewport with selected item visible.
static void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "menu %d/%d", menuSel + 1, M_COUNT);
    oled.drawStr(0, 8, hdr);
    oled.drawHLine(0, 10, 128);

    const int LH = 10;
    for (int i = 0; i < VIS; i++) {
        int idx = top + i;
        if (idx >= M_COUNT) break;
        int y = 12 + i * LH;
        if (idx == menuSel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, MENU[idx]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU[idx]);
        }
    }

    // bottom strip
    oled.drawHLine(0, 64 - 10, 128);
    oled.drawStr(0, 63, statusLine);
    oled.sendBuffer();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    menuEncRef = rightEnc.getTicks();
    setStatus("ready");
    oledMenu();   // render once at boot
    Serial.println("[INIT] menu UI — scroll w/ right wheel, btn = select");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
// IDLE-only: menu scroll + select. Dispatch runs blocking action; no OLED
// refresh happens during action (no periodic re-render at all here).
// OLED redrawn only on real change: scroll step or selection.
void loop() {
    long delta = rightEnc.getTicks() - menuEncRef;

    if (delta >= ENC_PER_STEP) {
        menuSel = (menuSel + 1) % M_COUNT;
        menuEncRef += ENC_PER_STEP;
        oledMenu();
        return;
    }
    if (delta <= -ENC_PER_STEP) {
        menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
        menuEncRef -= ENC_PER_STEP;
        oledMenu();
        return;
    }

    if (buttonEdge()) {
        dispatch(menuSel);                 // blocking — no display calls inside
        menuEncRef = rightEnc.getTicks();  // discard wheel motion during action
        oledMenu();                        // one render after action returns
    }
}
