#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PID {
    float kp, ki, kd, maxOut;
    float integral  = 0;
    float prevError = 0;
    unsigned long prevUs = 0;

    PID(float p, float i, float d, float mx) : kp(p), ki(i), kd(d), maxOut(mx) {}

    float compute(float error) {
        unsigned long now = micros();
        // BUG-1 fix: state is in the struct instance, not a static variable.
        // BUG-3 note: This returns a generic correction value.
        float dt = (prevUs == 0) ? 0.001f
                                 : (float)(now - prevUs) / 1000000.0f;
        
        // Clamp dt to prevent massive jumps during debug pauses or delays
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.0001f) dt = 0.0001f;

        prevUs = now;
        integral += error * dt;
        
        // Anti-windup: clamp integral to a reasonable range
        // If maxOut is 400, and Ki is 50, then 8 is the max useful integral.
        float iClamp = (ki > 0) ? (maxOut / ki) : 2.0f;
        if (integral > iClamp) integral = iClamp;
        if (integral < -iClamp) integral = -iClamp;

        float deriv = (error - prevError) / dt;
        prevError = error;
        
        float output = kp * error + ki * integral + kd * deriv;
        return (output > maxOut) ? maxOut : (output < -maxOut ? -maxOut : output);
    }

    void reset() {
        integral = 0;
        prevError = 0;
        prevUs = 0;
    }
};

#endif
