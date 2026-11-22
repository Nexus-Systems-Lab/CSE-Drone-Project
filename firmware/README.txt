ESP32 PID BALANCE — PHYSICAL PWM PROTOTYPE

This firmware drives four standard 50 Hz PWM ESC inputs. It implements roll/pitch
angle feedback, not a complete flight controller. It has no yaw-rate control,
altitude control, position control or automatic takeoff. Initial gains are not
flight-validated. Roll/pitch correction direction follows the user's measured axes.

OPEN AND UPLOAD
1. Extract the ZIP.
2. Open ESP32_PID_Balance/ESP32_PID_Balance.ino inside the package.
   Keep control_core.h and dashboard_html.h beside the .ino.
3. Board: ESP32 Dev Module (classic ESP32). Espressif Arduino-ESP32 core 3.x
   recommended; a 2.x LEDC compatibility branch is included.
4. Library Manager: install Adafruit SSD1306 and Adafruit GFX Library,
   accepting dependency installation. MPU6050 is read directly over Wire;
   Adafruit MPU6050 is not needed for this version.
5. Select the correct serial port and upload. Serial Monitor: 115200 baud.
6. Keep the assembled frame LEVEL and STILL at boot. Calibration needs 400
   consecutive samples (~4 seconds) and restarts its sample count if moved.
   Startup arming is blocked for at least five seconds.
7. Join WiFi ESP32-Drone-Test, password drone12345.
8. Open http://192.168.4.1. Stay connected despite the no-internet notice.
9. Wait for DISARMED with calibrated telemetry. If needed use RESET / CALIBRATE.
10. Press ARM while level and still. Base starts at 1000 us: motors remain stopped.
11. On a restrained roll/pitch test stand, start with base 1100 us and verify
    correction directions below. First verify indicated outputs without propellers.
    Available torque depends on actual motors, props, supply and operating throttle.
12. STOP ALL or Space stops/disarms. An explicit new ARM is required.

WIRING — USER-VERIFIED POSITIONS
M1 FRONT RIGHT : GPIO25 -> ESC M1 signal
M2 REAR RIGHT  : GPIO26 -> ESC M2 signal
M3 FRONT LEFT  : GPIO27 -> ESC M3 signal
M4 REAR LEFT   : GPIO14 -> ESC M4 signal
ESP32 GND -> ESC signal GND
MPU6050 SDA and OLED SDA -> GPIO21
MPU6050 SCL and OLED SCL -> GPIO22
MPU6050 VCC and OLED VCC -> 3.3V; both GND -> ESP32 GND
MPU6050 address 0x68; SSD1306 OLED 128x64 address 0x3C.
ESC battery power goes to the ESC, not an ESP32 GPIO.
ESC must accept standard 1000–2000 us, 50 Hz input; not DShot-only firmware.

SENSOR CONVENTIONS
User's readings using atan2(ay,az) roll and atan2(-ax,sqrt(ay^2+az^2)) pitch:
Level: R +4, P -5 degrees.
Front down: R +1, P -27 degrees -> pitch decreases.
Right down: R -20, P -5 degrees -> roll decreases.
This code preserves those conventions. It measures fresh level offsets instead
of hardcoding +4/-5. Do not change IMU mounting without rechecking directions.
Gyro X and Y correspond to those roll/pitch sensor axes near level.

CONTROL MODEL
Sample target: 100 Hz; complementary gyro/accelerometer attitude filter.
MPU6050: +/-4g accel, +/-500 degrees/s gyro, ~20 Hz DLPF, 100 Hz sample rate.
Motor PWM: hardware LEDC, 50 Hz, 16-bit duty. OLED refresh: 5 Hz;
web request/heartbeat loop: approximately 6.7 Hz. HTTP and OLED share the main
loop with sensing, so this is a bench prototype rather than deterministic
flight firmware. Actual timing depends on hardware and network behavior.
Roll target = pitch target = 0 degrees relative to calibrated level.
For each axis:
  error = -angle
  correction = Kp*error + Ki*integral(error) - Kd*measured_gyro_rate
Default Kp=1.5 us/degree, Ki=0, Kd=0.05 us/(degree/second).
Ki=0 means the integral contribution starts disabled; the PID implementation
supports integral gain and bounded anti-windup. Derivative uses filtered gyro rate.
Correction limited to +/-100 us per axis. Integrators reset on stop/minimum
throttle; accumulation freezes at previous actuator saturation.
The same gains apply to roll and pitch in this prototype. Gains reset on reboot.
UI allows gains only while stopped: Kp 0..5, Ki 0..1, Kd 0..1.
These limits are prototype choices, not measured flight tuning recommendations.

MIXER (r and p are roll/pitch corrections)
M1 front right = T + r + p + trim1
M2 rear right  = T + r - p + trim2
M3 front left  = T - r + p + trim3
M4 rear left   = T - r - p + trim4
Each final output clamps to 1000..1400 us.
Base throttle T: 1000..1300 us. Independent trims: -40..+40 us.
When DISARMED, FAULT or base T=1000, all outputs are exactly 1000 us, irrespective
of tilt or trims. ZERO also clears trims. STOP clears trims, throttle and PID.

EXPECTED RESPONSE — VERIFY BEFORE TUNING
Front down -> pitch negative -> front M1/M3 increase, rear M2/M4 decrease.
Rear down -> pitch positive -> rear M2/M4 increase, front M1/M3 decrease.
Right down -> roll negative -> right M1/M2 increase, left M3/M4 decrease.
Left down -> roll positive -> left M3/M4 increase, right M1/M2 decrease.
Gyro damping also reacts to tilt velocity, so steady hand-held tilt is the
clearest first direction check. If outputs oppose this table, STOP and resolve
sensor/motor mapping before increasing gains.
Correct motor command direction alone does not prove thrust direction, motor
rotation, propeller pairing or adequate thrust. Those need hardware verification.

KEYBOARD — PAGE MUST HAVE FOCUS
Q / A : increase / decrease M1 front-right trim by 5 us
W / S : increase / decrease M2 rear-right trim by 5 us
E / D : increase / decrease M3 front-left trim by 5 us
R / F : increase / decrease M4 rear-left trim by 5 us
T / G : increase / decrease BASE throttle for all motors by 5 us
Space : STOP ALL and disarm
One keypress is one step; holding a key does not repeat. Text/number fields do
not intercept typing as motor commands. Click outside an input before shortcuts.
Individual trim keys do not replace PID outputs; they add/subtract manual trim.
The UI labels actual acknowledged outputs separately from requested base throttle.

TUNING ORDER
Verify level calibration and all four steady tilt directions first. Start with
Ki=0. Make one small gain adjustment at a time while stopped, then repeat the
restrained test. Kp produces restoring correction; too much can oscillate.
Kd damps rotational movement but excessive derivative/noisy gyro can cause jitter.
Add integral only after proportional/derivative behavior is understood.
The program does not auto-tune. No fixed gains can be guaranteed from the three
static angle readings; inertia, thrust response, delay and vibration also matter.
Roll/pitch target zero does not hold altitude or prevent yaw/translation.

FAULTS AND STOP
ARM requires calibrated IMU, fresh data, PWM initialization, no latched fault,
angles within +/-8 degrees and rates within +/-10 degrees/s.
While armed, tilt over 35 degrees, sensor read errors/nonfinite data or a control
sample gap over 60 ms stop outputs and latch FAULT. RESET while level clears it
by starting a new calibration. Read errors can reflect wiring/noise/power problems.
No valid session heartbeat/command for 1500 ms or no WiFi client disarms.
The heartbeat and timing checks run in firmware's main loop; network STOP is
not an independent hardware power cutoff or a hard real-time guarantee.
Browser hiding/closing requests STOP; reconnect never automatically arms.
Generation tokens reject delayed commands and stale ARM requests after STOP.
Serial ! or 0 = STOP. Serial c = stop and recalibrate at current level.

OLED
Page A: monochrome artificial horizon, corrected roll/pitch, base throttle.
Page B: physical output commands M1-M4 in us.
Pages alternate every 2.5 seconds; top status ARM/OFF/CAL/FAULT.
If OLED is absent, sensor/control/web still work; check 0x3C and wiring.

FILES
ESP32_PID_Balance/ESP32_PID_Balance.ino : complete embedded firmware
ESP32_PID_Balance/control_core.h : PID math and verified motor mixer
ESP32_PID_Balance/dashboard_html.h : embedded web UI (required)
dashboard.html : separate editable UI source (offline preview only)
tests/control_core_test.cpp : native C++ regression test
README.txt : setup, behavior and practical limits

VALIDATION DONE IN THIS ENVIRONMENT
PASS: native g++ tests for all four correction directions, stop/minimum gating,
output clamping, gyro damping sign, anti-windup, reset and angle wrap.
PASS: JavaScript syntax checked with node --check.
PASS: HTML source matches the embedded header.
Arduino-ESP32 toolchain is unavailable here: full board compilation was not run.
No physical ESP32, MPU6050, ESC or motor test was possible here.

REFERENCES
Espressif LEDC API:
https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
TDK/InvenSense MPU6000/6050 register map:
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf


UPDATED DASHBOARD INTEGRATION (2026-10-16)
- Replaced embedded dashboard_html.h with the user-supplied dashboard.html.
- Standalone dashboard.html mirrors the ESP32-served UI. Open the UI through http://192.168.4.1 while connected to ESP32-Drone-Test; opening the standalone file directly cannot call /api endpoints.
- Added request latency indicator and reduced canvas work when the page is hidden.
- Retained original firmware control, API routes, heartbeat expiration, tilt and sensor protections, motor pin mapping and STOP behavior.
- Note: Existing firmware intentionally disarms when Wi-Fi client is lost or the heartbeat expires. No Wi-Fi link can be guaranteed permanent. A dropped link requires manual ARM again.
- The new UI includes 3D visualizer, live graphs, motor trims, gains and DEMO mode. DEMO is visualization only.
- IMPORTANT: Changes have been statically inspected, not verified on the physical ESP32. Validate firmware build and bench behavior before flight.

