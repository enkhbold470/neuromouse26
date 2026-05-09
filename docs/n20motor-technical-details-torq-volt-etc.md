No worries at all! Let's swap out the specs.

A 1:30 gear ratio at 6V spinning at 500 RPM means this motor is geared for **speed over brute strength** compared to the 1:50 version. Because the gear reduction is lower, you get a much faster output shaft, but the trade-off is lower torque.

Here are the exact details and torque estimates for your specific **1:30, 500 RPM, 6V N20 motor**:

### Motor & Gearbox Specifications

* **Operating Voltage:** 6V DC (Optimal)
* **Gear Reduction Ratio:** 1:30
* **No-Load Speed:** 500 RPM (@ 6V)
* **No-Load Current:** 0.15A (150 mA)
* **Stall Current:** ~1.2A to 1.6A (depending on the exact internal windings of the high-torque base motor)
* **Overall Dimensions:** 40.5mm x 12mm x 10mm (Length x Width x Height)
* **Output Shaft:** 3mm diameter, 10mm length, D-type cut for secure mounting.

### Torque Details

Because this is a 1:30 ratio (less reduction than the 1:50), your torque numbers will be lower, but it spins much faster.

* **Rated / Continuous Torque:** ~0.10 kg·cm to 0.15 kg·cm (approx. 1.4 to 2.1 oz·in). *This is the safe zone for continuous running without burning out the motor.*
* **Stall Torque:** ~0.45 kg·cm to 0.55 kg·cm (approx. 6.2 to 7.6 oz·in).
* *Note on Stalling:* If the motor is forced to stop completely while 6V is applied, it will draw maximum current and can strip the 1:30 brass gears or burn the motor coils very quickly. Avoid operating anywhere near the stall torque.

### Encoder Details (Updated for 1:30 Ratio)

The magnetic quadrature encoder on the back works the exact same way as the 1:50, but because the gear ratio is different, your code will count fewer pulses per full revolution of the wheel.

* **Operating Voltage:** 3.3V to 5V DC (Power this from your microcontroller, not the 6V motor supply).
* **Base Motor Resolution:** Usually 11, 12, or 14 Counts Per Revolution (CPR) on the bare motor shaft, depending on the specific magnetic disc used.
* **Total Output Resolution (What your code reads):** Multiply the motor CPR by your 1:30 gear ratio.
* *If it has a 12 CPR disc:* 12 × 30 = **360 counts per wheel revolution.**
* *If it has a 14 CPR disc:* 14 × 30 = **420 counts per wheel revolution.**
* *If it has an 11 CPR disc:* 11 × 30 = **330 counts per wheel revolution.**


* You will use these counts in your code (like an Arduino interrupt) to calculate exactly how far the robot has traveled or to run PID speed control.

### Standard 6-Pin Wiring (Same as before)

1. **M1 (Motor + / Red):** To Motor Driver
2. **M2 (Motor - / Black or White):** To Motor Driver
3. **VCC (Encoder Power / Blue):** 3.3V or 5V
4. **GND (Encoder Ground / Green):** Ground
5. **OUT A (Channel A / Yellow):** To Microcontroller interrupt pin
6. **OUT B (Channel B / White):** To Microcontroller interrupt pin *(Note: Wire colors can sometimes vary by manufacturer, but the pin order is almost always standard).*