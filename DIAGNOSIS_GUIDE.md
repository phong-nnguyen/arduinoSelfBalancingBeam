# Seesaw Balancing: The Simple Guide

Welcome! This guide will walk you through setting up and tuning your balancing robot. We'll do this in three simple steps.

---

**💥💥 SAFETY FIRST! 💥💥**

**REMOVE PROPELLERS for Steps 1 and 2.** Only attach them in Step 3 when explicitly told to. Propellers are dangerous. Wear safety glasses.

---

### Step 1: Calibrate the Sensor's "Sense of Balance"

Your MPU6050 sensor needs to learn what "perfectly level" feels like. We do this by finding its unique "offset" numbers.

1.  **Find the Example Code:** In PlatformIO, look in the file explorer on the left. Navigate to `lib/MPU6050/examples/IMU_Zero/IMU_Zero.ino`. This is a special program just for calibration.
2.  **Run the Calibration:**
    - Temporarily **delete all the code** in your `src/main.cpp` file.
    - **Copy all the code** from `IMU_Zero.ino` and **paste it** into your empty `src/main.cpp`.
    - Make sure the seesaw beam is **perfectly level and still**.
    - Upload this code to your Arduino.
3.  **Get Your Numbers:**
    - Open the **Serial Monitor**.
    - Wait for it to print a bunch of text. It will eventually tell you to send any character. Do that.
    - It will then spit out lines that look like `[ X GYRO ] OFFSETS: [ 123 ]`.
    - Write down the final offset numbers for **X, Y, Z Gyro** and **Z Accel**. They are your sensor's magic numbers.
4.  **Update Your Main Code:**
    - **Delete the `IMU_Zero` code** from `src/main.cpp`.
    - **Copy and paste the full code for `src/main.cpp`** provided in the next section of the tutorial. It has a special "Tuning Area" at the top.
    - Enter the magic numbers you just wrote down into that tuning area.

---

### Step 2: Finding the "Hover" Speed

Now we find the minimum power the motors need to have enough lift to tilt the beam.

1.  **Check the Code:** Make sure your `src/main.cpp` is loaded with the main balancing code (not the calibration code). Double-check your sensor offsets are correct. In the tuning area, set `Kp_roll`, `Ki_roll`, and `Kd_roll` all to `0`.
2.  **BE CAREFUL: ATTACH PROPELLERS NOW.** Make sure they are secure. Wear safety glasses.
3.  **Start Low:** In the `main.cpp` tuning area, set `baseThrottle = 1100`.
4.  **Upload and Test:**
    - Upload the code.
    - Open the Serial Monitor. It will say "DISARMED".
    - Hold the rig securely. Type `a` and hit Enter to **ARM** the motors. They will spin up to the `baseThrottle` speed.
    - See if they have enough power to tilt the beam when you give it a little nudge.
    - Type `d` and hit Enter to **DISARM**.
5.  **Adjust and Repeat:** If it's too weak, increase `baseThrottle` by 25 (e.g., to `1125`, `1150`). Upload and test again. Repeat until you find a speed that can comfortably move the beam. This is your ideal `baseThrottle`.

---

### Step 3: Tune the "Reflexes" (The PID Knobs)

This is the fun part. We're tuning the robot's brain. `P` is the main power, `D` is the brakes, and `I` is for tiny corrections.

**For every test below, you will:**

1.  Change the Kp, Ki, or Kd value in `src/main.cpp`.
2.  Upload the code.
3.  Place the rig on its pivot.
4.  Open the Serial Monitor, type `a` to ARM it, and give the beam a small push.
5.  Type `d` to DISARM it when you're done observing.

#### **Tune P (Proportional): The Muscle**

- Keep `Ki_roll = 0.0` and `Kd_roll = 0.0`.
- Start with **`Kp_roll = 2.0`**.
- Arm the rig and give it a small push.
  - If it's slow and lazy to correct itself -> **Increase `Kp_roll`**.
  - If it shakes violently (oscillates) -> **Decrease `Kp_roll`**.
- Find a `Kp_roll` value that is responsive but might overshoot a little.

#### **Tune D (Derivative): The Brakes**

- Keep your best `Kp_roll` value.
- Start with a small **`Kd_roll = 0.5`**.
- Arm the rig and test.
  - If it still overshoots and wobbles after correcting -> **Increase `Kd_roll`**.
  - If the motors sound jittery or twitchy -> **Decrease `Kd_roll`**.
- Find a `Kd_roll` that stops the wobble without making it twitchy.

#### **Tune I (Integral): The Fine-Tuner (Optional)**

- You might not even need this one.
- If your rig balances well but is always slightly off-center, add a tiny **`Ki_roll = 0.02`**.
- If this introduces a slow wobble, the value is too high. Set it back to 0.

Once you have values that result in a stable, balanced beam, you've done it!
