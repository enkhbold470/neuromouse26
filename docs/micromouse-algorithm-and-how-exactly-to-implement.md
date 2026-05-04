<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# micromouse algorithm and how exactly to implement with imu, encoder, 4 ir sensor, n20 500rpm motor, buzzer, esp32s3, come up with very niche technical plan

You can treat this as three stacked problems: 1) motion primitives that are metrically accurate using IMU+encoders+IR, 2) a wall map plus floodfill search, 3) a real‑time scheduler on ESP32‑S3 tying it together.[^1][^2][^3][^4][^5]

Below is a very implementation‑oriented plan for exactly your sensor set.

***

## High‑level architecture

- Use floodfill for maze solving and path optimization.[]()[^2][^3][^6]
- Use:
    - Encoders for distance and speed.
    - IMU gyro for heading and turn accuracy.
    - 4 IR sensors for wall detection and lateral alignment.
    - Buzzer for state/debug feedback.
    - ESP32‑S3 dual core: one core control loop, one core high‑level planning.[^4][^5][^7]

Main software blocks:

1. `sensors_task` (Core 0, high priority, 1 kHz)
2. `motion_control_task` (Core 0, 1 kHz)
3. `planner_task` (Core 1, 100 Hz)
4. `ui_task` for buzzer / debugging (Core 1, low priority)

***

## Hardware \& pin planning

Take cues from existing ESP32/ESP32‑S3 micromouse projects and ESP32+encoder examples.[^8][^5][^4]

- Motors (2× N20 500 rpm with encoders):
    - 2 PWM outputs (left, right) to H‑bridge EN pins.
    - 4 direction pins (IN1…IN4).
    - 4 encoder inputs (A/B left, A/B right) on interrupt‑capable GPIOs.[^9][^10][^8]
- IR sensors:
    - 4 analog inputs on ADC1.
    - Arrange as: front‑left, front‑right, side‑left, side‑right.[^11][^10]
- IMU:
    - I²C on a dedicated bus (400 kHz), e.g. SDA/SCL on pins not shared with encoders.[^7][^4]
- Buzzer:
    - One GPIO with LEDC PWM so you can play tones.
- Optional:
    - One button for start/stop.
    - UART or Wi‑Fi logging for calibration.

Keep all sensors on a solid analog ground region and route encoder lines differentially and short to reduce noise.[^12][^10]

***

## Coordinate frames and units

- Maze coordinates:
    - Integer cell coordinates $(i,j)$ with cell size $L$ (e.g. 180 mm or whatever the contest uses).[^6][^2]
- Robot body frame:
    - Origin at center between wheels.
    - $x$ forward, $y$ left.
- Track:
    - Maintain $(x, y, \theta)$ in meters and radians in maze frame using encoders + gyro.[^5][^4][^7]

***

## Encoder \& IMU fusion for odometry

Use a simple complementary filter: encoders give distance, gyro gives heading.[^13][^4][^5][^7]

### Encoders

- From motor+encoder calibration:
    - Measure ticks for a known distance (e.g. 18 cm) and set `TICKS_PER_M` as in ESP32 micromouse examples.[^9][^4]
- Every control tick (1 kHz):
    - Read `left_ticks_delta`, `right_ticks_delta`.
    - Convert to wheel angular displacement and then to linear distance.
    - Compute:
        - `ds = (dl + dr) / 2`
        - `dtheta_enc = (dr - dl) / WHEELBASE`


### Gyro

- Calibrate gyro bias at startup with robot still, as is standard in micromouse builds.[^13][^7]
- Integrate angular rate to get `theta_gyro`.
- Complementary filter:

```c
theta = alpha * (theta + dtheta_enc) + (1 - alpha) * theta_gyro;
```

with alpha around 0.98.

Position update each loop:

```c
x += ds * cos(theta);
y += ds * sin(theta);
```

Use this for debugging and rough longitudinal correction, but for maze logic you mostly operate on cell counts (number of completed “move one cell” primitives).[^4][^5]

***

## IR sensor model \& wall detection

Use IR distance sensors as wall presence detectors and for fine lateral correction.[^14][^10][^15][^11]

### Static calibration

For each of the 4 IR sensors:

1. Put robot at:
    - “No wall” distance (far away).
    - “Wall present” distance (centered in a cell).
2. Record ADC values to get:
    - `ir_no_wall`, `ir_wall` for that sensor.[^10][^11][^14]
3. Define thresholds:
    - `th_wall = (ir_no_wall + ir_wall) / 2`
    - Possibly add hysteresis.

Store in a table:

```c
typedef struct {
    uint16_t no_wall;
    uint16_t wall;
    uint16_t th_wall;
} IrCal;
IrCal ir_front_left, ir_front_right, ir_side_left, ir_side_right;
```


### Runtime processing

At 1 kHz:

- Read all 4 ADC channels.
- Low‑pass filter:

```c
ir[i].filtered += beta * (raw - ir[i].filtered);
```

with beta small (0.1–0.2).
- Generate booleans:
    - `front_wall = (FL > th || FR > th)`
    - `left_wall = side_left > th_side`
    - `right_wall = side_right > th_side`.[^16][^11][^10]

Also compute a lateral error when running down a corridor:

- `err_side = (side_left_normalized - side_right_normalized)` where normalized is `(val - no_wall) / (wall - no_wall)`.[^17][^14]

You will use `err_side` in lateral PID while moving forward so you stay centered between walls.[^17][^10]

***

## Motion primitives built on PID

Your algorithm will work best if you define high‑level motion primitives:

1. `move_cell_forward(1)`
2. `turn_left_90()`, `turn_right_90()`
3. `pivot_180()`

Floodfill just sequences these primitives; they must be metrically precise.[^3][^1][^2]

### Low‑level speed loop

Use RPM ~500 motors with PID speed control per wheel using encoder feedback.[^8][^5][^9][^4]

- Inner loop at 1 kHz:
    - For each wheel:
        - Measure current ticks per period → angular speed.
        - PID to reach target speed `v_left_cmd`, `v_right_cmd`.
- Outer loop:
    - For `move_cell_forward`, maintain:
        - Longitudinal target distance `s_target = CELL_LENGTH`.
        - Heading target `theta_target` from current cell orientation.
    - Use:
        - Forward speed profile (trapezoidal).
        - Heading correction from gyro + side IR:

```c
heading_error = theta_target - theta;
lateral_error = k_ir * err_side;
correction = Kp_h * heading_error + Kp_lat * lateral_error;
v_left_cmd  = v_forward - correction;
v_right_cmd = v_forward + correction;
```[^17][^13][^10][^18]

```


### Turn 90 degrees

Use IMU gyro as the primary signal, not encoders.[^7][^13]

- Entry:
    - Zero relative angle `theta_turn = 0`.
- Loop:
    - Set `v_left_cmd = -v_turn`, `v_right_cmd = v_turn` for on‑spot yaw.
    - Integrate gyro to get `theta_turn`.
    - Stop when \(|\theta_turn| >= \pi/2` then decelerate.
- Use a small PD on angle error for smoother stop.

Refine turns by using front IR to finalize alignment when a front wall is present.[][]

***

## Maze representation and floodfill

Use standard 16×16 grid with walls stored per cell.[][][][]

### Data structures

```c
#define N 16

typedef struct {
    uint8_t walls;   // bit 0=N, 1=E, 2=S, 3=W
    uint8_t dist;    // floodfill distance
} Cell;

Cell maze[N][N];
```

- Maintain current cell indices `ci, cj` and heading `dir` in {N,E,S,W}.
- On entering a new cell, use IR booleans to update walls:
    - If facing North:
        - front_wall → wall in `maze[ci][cj].walls |= (1<<N)` and matching south wall in neighbor.[][]
- Use recognized goal cells as center 2×2 or 4 center cells depending on rule set.[][]


### Floodfill procedure

You can implement the “reverse from goal” BFS‑style floodfill as given by IEEE UCI / UCLA docs.[][][]

Basic algorithm:

```c
void floodfill() {
    init distances to INF;
    queue all goal cells with distance 0;

    while queue not empty:
        c = pop();
        for each neighbor n of c with no wall between:
            if dist[n] > dist[c] + 1:
                dist[n] = dist[c] + 1;
                push(n);
}
```

On exploration:

1. After you update walls at the new cell, rerun floodfill or a local update version.[][][]
2. To choose next move:
    - Look at neighbors of current cell that are reachable (no wall).
    - Pick the one with minimum distance; use a tie‑breaking policy (e.g. forward > left > right > back) to reduce oscillations.[][]

You normally do:

- First phase: explore until all cells on the optimal path to goal are fully known.
- Second phase: “speed run” by just replaying shortest path at higher velocity.[][]

***

## State machine on top of primitives

Define a clear state machine that the planner drives.

States:

- `IDLE`
- `SEARCHING`
- `AT_GOAL`
- `RETURNING`
- `SPEED_RUN`

Planner loop (100 Hz):

```c
switch(state) {
  case IDLE:
    if (button_pressed) {
      reset_maze();
      state = SEARCHING;
    }
    break;

  case SEARCHING:
    if (!primitive_active) {
      update_walls_from_sensors();
      floodfill();
      next_dir = choose_next_direction();
      schedule_primitive(next_dir);
      if (is_goal(ci, cj)) state = AT_GOAL;
    }
    break;

  case AT_GOAL:
    save_best_path();
    plan_return_to_start();
    state = RETURNING;
    break;

  case RETURNING:
    // Similar to SEARCHING but target is start.
    break;

  case SPEED_RUN:
    execute_cached_best_path_with_higher_speed();
    break;
}
```

Primitive scheduling just sets a goal struct consumed by the motion control task:

```c
typedef enum { PRIM_NONE, PRIM_FORWARD, PRIM_TURN_LEFT, PRIM_TURN_RIGHT } PrimType;

typedef struct {
    PrimType type;
    float target;    // distance or angle
    bool active;
} PrimitiveGoal;
```

`motion_control_task` drives wheels until the primitive objective is met, then clears `active`.

***

## ESP32‑S3 specific runtime structure

ESP32‑S3 micromouse and ESP32 robots tend to use FreeRTOS tasks plus hardware timers to get reliable control rates.[][][]

Suggested layout:

- Hardware timer interrupt at 1 kHz:
    - Quickly latch encoder counts into volatile variables.
    - Optionally trigger ADC sampling.
- `sensors_task` (Core 0, high priority):
    - Runs every 1 ms, reads latched ticks, reads latest ADC buffer and IMU values, updates filtered values and odometry.
- `motion_control_task` (Core 0, medium):
    - Also every 1 ms.
    - Uses primitive goal + odometry + IR errors to update `v_left_cmd`, `v_right_cmd`.
    - Runs speed PID loops and writes PWM duty to motors.[][]
- `planner_task` (Core 1):
    - 100 Hz.
    - Uses wall booleans at cell boundaries to update maze and run floodfill.[][][]
- `ui_task`:
    - Drives buzzer patterns depending on state or errors.

This separation avoids long floodfill runs from blocking motor control.

***

## Calibration and tuning flow

Borrow ideas from micromouse calibration guides: first sensors, then straight motion, then turns, then floodfill.[][][][][][][]

Order:

1. **IMU**
    - Bias: average gyro output for a few seconds at rest and store offset.[][]
    - Scale: rotate robot by a known angle using a jig or a template and adjust scale factor so integrated angle matches.[]
2. **Encoders**
    - Command robot to move a precise distance (e.g. 18 cm) at low speed several times, average ticks → `TICKS_PER_M`.[][]
3. **IR sensors**
    - Do wall/no‑wall measurements for each sensor at known distances to get thresholds and optionally a distance curve.[][][]
    - Validate lateral error signal by plotting `side_left - side_right` down a straight corridor.[]
4. **Speed PID**
    - Start with low Kp, then increase until you get acceptable response without oscillation, following micromouse / line follower PID tuning approaches.[][][]
5. **Heading / lateral control**
    - Tune `Kp_h` on heading error using IMU.
    - Then tune `Kp_lat` on IR lateral error while running in a straight hallway.[][]
6. **Turn primitives**
    - Tune turn speed and IMU angle target until 90° turns land you centered in next cell with minimal correction from IR.[][]
7. **Floodfill**
    - First validate in simulator as suggested by UCLA’s micromouse docs, then port same logic to hardware.[][][]

***

## Extra niche tricks with your sensor set

These are the “niche” bits that use all your sensors together.

1. **Wall‑based longitudinal correction without front wall**
Use side sensors to detect “post‑to‑post” or wall‑to‑gap transitions and clamp longitudinal position when you pass cell boundaries, a common advanced micromouse trick.[][]
2. **Dynamic trust between gyro and encoders**
    - During fast straight runs where side IR shows you centered and stable, slowly correct `theta` toward the geometric direction of the corridor, reducing accumulated gyro drift.[][]
    - During sharp turns on slippery tiles, trust the gyro more than encoders, since wheels may slip.[][]
3. **IR‑assisted cornering**
When approaching an intersection at speed, use rising front IR reading as a trigger to start braking for turns earlier than pure distance estimates, improving repeatability at higher speeds.[][][]
4. **Buzzer as “tuner HUD”**
Encode different calibration modes and error conditions as distinct beep patterns so you can tune without a laptop, similar to how some micromouse teams debug in competition arenas.[][]
5. **Wi‑Fi debug dashboard on ESP32‑S3**
Follow the ESP32‑S3 micromouse project pattern that exposes status over Wi‑Fi, and stream IR, IMU, encoder, and floodfill grids to a web UI for aggressive tuning between runs.[][]

***

If you want, I can sketch out a concrete module layout with function names and a rough C/CPP header structure targeting ESP‑IDF or PlatformIO so you can drop it straight into a project.
<span style="display:none">[^19][^20][^21][^22][^23][^24][^25][^26][^27][^28][^29][^30]</span>

<div align="center">⁂</div>

[^1]: https://ieee.ics.uci.edu/micromouse/floodfill.html

[^2]: https://projects.ieeebruins.com/micromouse/floodfill-module

[^3]: https://github.com/ricdizio/Micromouse

[^4]: https://github.com/Thisen-Ekanayake/Micro-Mouse

[^5]: https://github.com/opatiny/micromouse

[^6]: https://www.physics.unlv.edu/~bill/ecg497/Drew_Tondra_report.pdf

[^7]: https://www.trine.edu/design-expo/documents/2024/EE - Micromouse SrProject poster.pdf

[^8]: https://www.youtube.com/watch?v=T0kihDt2m08

[^9]: https://projects.ieeebruins.com/micromouse/motor-setup-and-pid-module

[^10]: https://www.marginallyclever.com/2016/05/building-testing-micromouse-sensors/

[^11]: https://projects.ieeebruins.com/micromouse/ir-sensors-module

[^12]: https://github.com/gautam-dev-maker/mushak

[^13]: https://micromouseonline.com/2011/05/08/setting-up-a-gyro-on-the-micromouse/

[^14]: https://micromouseonline.com/2011/05/07/easier-wall-sensor-calibration/

[^15]: https://www.youtube.com/watch?v=9kM4Q6i0zTE

[^16]: https://forum.arduino.cc/t/micromouse/595925

[^17]: http://micromouseusa.com/?p=828

[^18]: https://www.youtube.com/watch?v=qKoPRacXk9Q

[^19]: https://forum.arduino.cc/t/esp32-c3-pid-tuning-for-higher-speed-without-oscillation-on-2xn20-motors/1382231

[^20]: https://hackaday.io/project/197835-micromouse

[^21]: https://www.youtube.com/watch?v=knkXkCBqha8

[^22]: https://www.youtube.com/watch?v=3N5E1Ai-aI8

[^23]: https://www.waveshare.com/wiki/ESP32-S3-Tiny

[^24]: https://www.elecrow.com/sharepj/esp32s3-diy-fully-automated-smart-greenhouse-757.html

[^25]: https://www.cambridge.org/core/journals/robotica/article/lowcost-depthimu-intelligent-sensor-fusion-for-indoor-robot-navigation/E9952B7C695EA15E3F13F480D32E868E

[^26]: https://www.instructables.com/Head-Mouse-With-MPU6050-and-Arduino-Micro/

[^27]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12390187/

[^28]: https://www.youtube.com/watch?v=JaCi-V2e2kA

[^29]: https://www.sciencepublishinggroup.com/article/10.11648/j.ajist.20250904.11

[^30]: https://www.youtube.com/watch?v=tRtfpnu4LjE

