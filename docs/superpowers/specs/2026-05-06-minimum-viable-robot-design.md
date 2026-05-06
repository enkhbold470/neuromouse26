# Minimum Viable Robot (MVR) Design Spec

**Date:** 2026-05-06
**Topic:** Minimum Viable Maze Solving Robot (Ruthless MVP & Physics-First)
**Status:** DRAFT (Awaiting User Review)

---

## 1. Overview
The goal is to deliver a functional, maze-solving robot as quickly as possible by removing all non-essential "bloat" and grounding the motion logic in physical laws (kinematics). We prioritize reliability and "maze-solving success" over advanced features.

---

## 2. Ruthless MVP Constraints
To ensure speed and portability across any microcontroller (including low-end chips), the following items are **removed**:
- **No RGB LEDs:** FastLED and WS2812B logic are deleted.
- **No IMU (for now):** BUG-2 is postponed until the base MVP is successful.
- **No Custom PWM Calculus:** Removed `ledc` (ESP32-specific) in favor of universal `analogWrite()`.
- **No Motor Stiction Mapping:** Removed `MIN_POWER` logic; using raw 8-bit PWM (0-255).
- **No Advanced UI:** Simplified buzzer logic using standard `tone()`.

---

## 3. Physics-First Kinematics
To prevent tire slip and momentum overshoot, the robot uses a **Trapezoidal Velocity Profile**:
- **Acceleration Ramp:** Starts at a low duty cycle (above stall) and increments linearly to `BASE_PWM`.
- **Deceleration Ramp:** Decrements PWM starting at 75% of the target distance to allow for a precise stop.
- **Momentum Settling:** `stopMotors()` uses an electronic brake followed by a 100ms settling delay.

---

## 4. Architectural Refactoring (Modular Cleanup)
The code is moved from a monolithic `main.cpp` into modular headers to support testability and follow established design patterns:
- `include/PinConfig.h`: Centralized pins and physics constants.
- `include/PID.h`: Generic PID with anti-windup and state isolation (Fixes BUG-1, BUG-3).
- `include/MicromouseMaze.h`: 16x16 Flood-fill with robust queue indexing (Fixes BUG-4).
- `include/MicromouseMotor.h`: Lean, universal 8-bit motor driver.
- `include/MicromouseEncoder.h`: Interrupt-driven feedback.

---

## 5. Success Criteria
- [ ] Robot moves forward one cell and stops within ±5mm of center.
- [ ] Robot performs a 90-degree turn with ±5 degree accuracy.
- [ ] Maze solver correctly identifies walls and updates the flood map.
- [ ] Robot reaches the center goal of a 16x16 maze.

---

## 6. Self-Review
- **Placeholder scan:** None. All constants are defined in `PinConfig.h`.
- **Consistency:** API uses 8-bit PWM (0-255) across all modules.
- **Scope check:** Strictly limited to basic maze solving.
- **Ambiguity check:** Kinematic ramps are clearly defined as linear increments/decrements.
