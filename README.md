# ESP32 Thrust Dashboard

Standalone Arduino sketch for an ESP32 web dashboard with HX711 load cell input, motor PWM output, test controls, and optional tachometer input.

## Wiring

| Signal | ESP32 pin |
| --- | ---: |
| HX711 DT / DOUT | GPIO 32 |
| HX711 SCK / CLK | GPIO 33 |
| Motor driver PWM input | GPIO 25 |
| Optional tachometer output | GPIO 27 |
| Grounds | Common GND |

The HX711 is powered from 3.3 V as supplied. The sketch configures GPIO 25 as 20 kHz, 8-bit PWM, intended for a PWM input on a suitable motor driver. A hobby ESC usually expects servo-style pulses; do not connect this generic PWM configuration to an ESC unless its input specification supports it. Change `PWM_FREQUENCY_HZ` and the output implementation to match the motor controller.

The optional RPM input uses one falling edge per pulse by default. Set `TACH_PULSES_PER_REV` to match the sensor and set `TACH_ENABLED` to `false` if no tachometer is connected. The default tachometer pin (GPIO 27) is a suggested input because no RPM sensor wiring was provided.

## Build and connect

1. Install the Arduino-ESP32 board package (3.x) and the `HX711` library by Bogdan Necula (`bogde/HX711`).
2. Open `ESP32_Thrust_Dashboard.ino`, select the ESP32 board, and upload.
3. Change `AP_PASSWORD` near the top of the sketch before using it.
4. Connect to the ESP32 Wi-Fi network `Thrust-Rig` (password is the value in `AP_PASSWORD`).
5. Open `http://192.168.4.1` in a browser.

The page has manual throttle, a timed hold, a linear 0-to-100% sweep (60 seconds by default), an emergency stop and reset, HX711 zero/tare and known-mass calibration, a load-cell ignore switch, and live thrust/RPM and thrust/time charts. Timed runs finish at 0% throttle. If the dashboard stops polling for 2.5 seconds while a command is active, the sketch cuts PWM and latches the emergency stop. Load-cell readings use a 5-sample median filter followed by an exponential smoother (`THRUST_EMA_ALPHA = 0.25`); reduce the alpha for steadier but slower readings, or increase it for a quicker response.

## Load cell setup

With the test stand unloaded, use **Zero / tare**. Then place a known mass on the load cell, enter its mass in grams, and press **Calibrate**. The factor is held in RAM and needs to be set again after restarting the ESP32. Thrust readings are displayed in grams; convert to newtons if your test reports use SI units.

The emergency stop is a software command that sets the PWM output to zero and prevents new runs until reset. Keep a physical power disconnect available for the motor system.
