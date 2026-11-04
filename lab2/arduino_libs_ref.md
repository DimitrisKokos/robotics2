# Course libraries — student reference

Everything you need to **use** the four course libraries in your sketches (Labs 1–4), in one place: what each
library does, the functions you call, the conventions, the serial protocol and the log, and the tools that read
it. Keep it open while you program.

| Library | Include | What it is for | Used in |
|---|---|---|---|
| `AS5600_lar` | `<AS5600_lar.h>` | the magnetic angle sensor: angle, continuous position, velocity, magnet status | Lab 1 (also inside `FOC_lar`) |
| `FOC_lar` | `<FOC_lar.h>` | the BLDC motor + driver + sensor: start-up and alignment, voltage and torque commands | Labs 1–4 |
| `LoadCell_lar` | `<LoadCell_lar.h>` | the load cell (HX711): tare, calibration, mass, force, torque | Lab 1 |
| `Pendulum_lar` | `<Pendulum_lar.h>` | the pendulum: model, measured state, timing, start/stop, log, swing-up/balance switch | Labs 2–4 |

The **controllers** (PID, LQR, MPC, TVLQR) and the **energy swing-up** are *not* in the libraries: you write them.

---

## 0. Installation and conventions

Copy the four folders of `lab_code/libs/` into your Arduino libraries folder (`~/Arduino/libraries/` on Linux/macOS,
`Documents\Arduino\libraries\` on Windows) and restart the IDE. Install **Simple FOC** and **HX711_ADC** from the
Library Manager.

`Pendulum_lar` comes with **TODOs**: some functions of `Pendulum_lar.cpp` and `PendulumModel.cpp` are yours to write,
with step-by-step instructions in the code, each marked with the lab that needs it (Lab 2 Part 1, Lab 2 Part 2,
Lab 3). Edit them in the copy in your libraries folder: the IDE recompiles the library with your sketch. Until you
complete a TODO it returns a harmless placeholder (zeros, 0 V, `updateMode()` always `BALANCE`), so the earlier
parts keep working. The tables below describe what each function does once it is complete.

```
libraries/
├── AS5600_lar/      AS5600_lar.h   AS5600_lar.cpp
├── FOC_lar/         FOC_lar.h      FOC_lar.cpp
├── LoadCell_lar/    LoadCell_lar.h LoadCell_lar.cpp
└── Pendulum_lar/    Pendulum_lar.h Pendulum_lar.cpp PendulumModel.h PendulumModel.cpp
```

* **Units:** SI everywhere: rad, rad/s, s, V, N·m, kg, m. (The load cell reports grams.)
* **Angles:** `θ = 0` the pendulum hangs down, `θ = π` upright; near the top `φ = θ − π`. A positive voltage, a
  positive torque and a positive `θ` are the same direction (SimpleFOC's alignment sees to it).
* **Serial:** 115200 baud. Lines starting with `#` are messages; the other lines are data (CSV).
* **Never block `loop()`:** `foc.update()` must run as often as possible. No `delay()` in a motor sketch; do the
  slow work once per control period (`Pendulum_lar` tells you when).

---

## 1. `Pendulum_lar` — the pendulum sketches (Labs 2–4)

One object, `Pendulum_lar pendulum;`, gives you the model of the pendulum, the measured state, the timing of the
control loop, the serial start/stop keys, the CSV log, the torque command and the swing-up/balance switch.

### 1.1 The model

$$
J\ddot\theta = \tau - b\dot\theta - \tau_c\,\mathrm{sign}(\dot\theta) - G\sin\theta,\qquad
J=\Big(\frac{m_r}{3}+m_p\Big)l^2,\qquad G=\Big(\frac{m_r}{2}+m_p\Big)lg
$$

`J` is the inertia about the shaft, `G` the largest gravity torque (pendulum horizontal), `b` the viscous and `τ_c`
the Coulomb friction (Lab 1, ring-down). With the back-EMF compensation on (`USE_BACK_EMF_COMP 1` in the sketches)
the model uses the mechanical `b`; without it, the motor adds an electrical damping `k·Ke`.

### 1.2 A complete pendulum sketch

```cpp
#include <FOC_lar.h>
#include <Pendulum_lar.h>

FOC_lar foc(7, 11, 10, 9, 8);                 // pole pairs, IN1, IN2, IN3, EN
Pendulum_lar pendulum;

void setup()
{
    Serial.begin(115200);
    if (!foc.begin(12.0f, 3.0f, 1.0f, STORED_ZERO_ELECTRIC_ANGLE, STORED_SENSOR_DIRECTION)) while (1);
    foc.setTorqueConstant(0.0082f);           // k, N*m/V      (Lab 1)
    foc.setBackEmfConstant(0.0286f);          // Ke, V*s/rad   (Lab 1)

    pendulum.begin(foc, 0.01f, 1.0f, 60.0f);                        // Ts = 10 ms, velocity filter, run time
    pendulum.setParameters(0.020f, 0.050f, 0.090f, 2.2e-5f, 5.7e-4f, 9.80665f);   // m_r, m_p, l, b, tau_c, g
    pendulum.setCoulombCompensation(true);                          // + tau_c*sign(omega) on every command
}

void loop()
{
    foc.update();                             // the FOC algorithm: every loop, as fast as possible

    bool new_sample = pendulum.update();      // keys, start / stop, run time, timing, theta and omega
    if (pendulum.started())
    {
        // a run has just started: reset your controller (integrals, ...)
    }
    if (!new_sample)
    {
        return;                               // not yet time for the next control step
    }

    float tau = 0.0f;                         // <- your controller, from pendulum.theta and pendulum.omega
    pendulum.applyTorque(tau);
    pendulum.log();
}
```

The lab sketches have this structure already; you write the controller.

### 1.3 Set-up (in `setup()`, after `foc.begin()`)

| Call | What it does |
|---|---|
| `pendulum.begin(foc, control_period_s, velocity_alpha, run_time_s, log_period_ms = 20)` | Connects the motor; control period `Ts` (s); velocity filter `α` in (0, 1] (1 = no filtering, smaller = smoother but slower); the run stops by itself after `run_time_s`; one log line every `log_period_ms` |
| `pendulum.setParameters(rod_mass, point_mass, length, viscous_friction, coulomb_friction = 0)` | The model: kg, kg, m, N·m·s/rad, N·m |
| `pendulum.setCoulombCompensation(bool on)` | Adds the feed-forward `τ_c·sign(ω)` to every command (default off), so your controller sees the linear plant |
| `pendulum.setCaptureAngles(capture, release)` | Thresholds of `updateMode()` (default 0.4 rad and 1.0 rad) |

### 1.4 Every `loop()`

| Call / member | What it gives you |
|---|---|
| `bool pendulum.update()` | Call right after `foc.update()`. Handles the keys `s`, `x`, `p` and the timing. Returns **`true` once per control period** while a run is active, with a fresh measurement |
| `pendulum.theta` | Angle, rad, **continuous** (it does not wrap: one turn adds 2π). 0 = the position at the start command |
| `pendulum.omega` | Angular velocity, rad/s (finite difference of the angle + first-order filter) |
| `pendulum.dt` | The actual control period of this sample, s (use it to integrate) |
| `bool pendulum.started()` | `true` once, right after `s`: reset your controller here |
| `bool pendulum.isRunning()` | A run is active |
| `char pendulum.readCommand()` | The next key that `update()` did not use (`h` and the keys of your sketch); 0 if none |

### 1.5 Commands

| Call | What it does |
|---|---|
| `pendulum.applyTorque(float torque_nm)` | Applies a torque (N·m): converted to voltage with `k`, plus the back-EMF compensation `Ke·ω` and the Coulomb feed-forward. The voltage is limited to `±MOTOR_VOLTAGE_LIMIT` |
| `float pendulum.maxTorque()` | The largest torque the motor can give, `τ_max = k·U_max` (N·m) |
| `pendulum.stop()` | 0 V, ends the run (the `x` key does this) |
| `pendulum.log()` | Prints one CSV line (rate-limited), with the present mode |
| `pendulum.log(int mode)` | The same, with a mode number of your choice |

### 1.6 The model functions (members of `pendulum`)

| Call | Returns |
|---|---|
| `pendulum.inertia()` | `J` (kg·m²) |
| `pendulum.gravityTorque()` | `G` (N·m) |
| `pendulum.viscousFriction()` | `b` (N·m·s/rad) |
| `pendulum.coulombFriction()` | `τ_c` (N·m) |
| `pendulum.naturalFrequency()` | `√(G/J)` (rad/s): the small-swing frequency at the bottom, the unstable pole at the top |
| `pendulum.canLiftDirectly(pendulum.maxTorque())` | `true` if `τ_max > G` (the motor can hold the pendulum horizontal) |
| `pendulum.energy(theta, omega)` | `E = ½Jω² + G(1 − cos θ)` (J): 0 at rest at the bottom |
| `pendulum.energyUpright()` | `E_up = 2G`, the energy at rest at the top |
| `pendulum.acceleration(theta, omega, torque)` | `θ''` of the nonlinear model |
| `pendulum.step(theta, omega, torque, dt)` | One Runge-Kutta step of the model (updates `theta`, `omega`) |
| `pendulum.linearizeUpright(A, B)` / `pendulum.discretizeUpright(Ts, Ad, Bd)` | Linear model at the top, `x = [φ, φ']`, continuous / discrete (zero-order hold): `float A[2][2], B[2]` |
| `pendulum.linearizeAt(theta, A, B)` / `pendulum.discretizeAt(theta, Ts, Ad, Bd)` | The same at any angle, `x = [Δθ, Δω]` (`theta = 0` hanging, `π` upright) |

A `pendulum` can be passed to any function that takes a `const PendulumModel &` (the model class it inherits from).

### 1.7 Swing-up / balance switching

For the upright equilibrium, `updateMode()` tells you whether the pendulum is close enough to the top for the
balance controller:

| Call | What it does |
|---|---|
| `Pendulum_lar::Mode pendulum.updateMode()` | From the present `theta`, `omega`: switches to `Pendulum_lar::BALANCE` when `\|θ − π\| <` capture angle **and** `\|ω\| < 8 rad/s` (`PENDULUM_CAPTURE_SPEED`); back to `Pendulum_lar::SWING_UP` when `\|θ − π\| >` release angle (the pendulum fell). Returns the new mode |
| `pendulum.mode()` | The present mode, without updating it. Read it **before** `updateMode()` to detect the moment of the switch |
| `pendulum.resetMode()` / `pendulum.resetMode(Pendulum_lar::BALANCE)` | Sets the mode (the start command resets it to `SWING_UP`) |

The two thresholds are different on purpose (hysteresis): the controller does not chatter between the two modes
at the edge. The mode is also the last column of the log.

### 1.8 Helpers and constants

| Name | What it is |
|---|---|
| `pendulum_wrap_angle(a)` | `a` wrapped to [−π, π). Use it on angle errors: `pendulum_wrap_angle(PENDULUM_LAR_PI - pendulum.theta)` |
| `pendulum_clamp(x, low, high)` | `x` limited to [low, high] |
| `pendulum_sign(x)` | −1, 0 or +1 |
| `pendulum_mat_mul(a, b, out)` | 2×2 matrix product `out = a·b` (for your `discretizeAt()`, Lab 3) |
| `PENDULUM_LAR_PI`, `PENDULUM_LAR_GRAVITY` | π and 9.80665 m/s² |
| `fabsf`, `sinf`, `cosf`, `sqrtf`, ... | `<math.h>`, already included |

### 1.9 Serial keys and the log

| Key | Handled by | Action |
|---|---|---|
| `s` | `Pendulum_lar` | start a run (when idle). **The pendulum must hang at rest**: its position becomes `θ = 0` |
| `x` | `Pendulum_lar` | stop the run: motor at 0 V |
| `p` | `Pendulum_lar` | pause / resume the CSV log |
| `h` and others | your sketch, via `readCommand()` | help, equilibrium selection, ... (see the header of each sketch) |

At start-up the sketch prints `#READY: pendulum hanging at rest, then send 's'`. After `s` it prints the CSV header
and one line every 20 ms:

```
t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode        (mode 0 = swing-up / motor off, 1 = balance)
```

`torque_Nm` is the torque your controller asked for (without the Coulomb feed-forward); `voltage_V` is the voltage
really applied. At the end (`x` or run time) the sketch prints `#DONE` or `#ABORTED` and the **longest controller
computation**, which must stay well below the control period.

---

## 2. `FOC_lar` — the motor

```cpp
FOC_lar foc(pole_pairs, pin_a, pin_b, pin_c, pin_en);     // SimpleFOCMini + UNO: (7, 11, 10, 9, 8)
```

### 2.1 Start-up

| Call | What it does |
|---|---|
| `bool foc.begin(supply_V, limit_V, align_V)` | Sensor + driver + motor init, then the SimpleFOC alignment (**the shaft moves**). Call after `Serial.begin()`. `false` on failure |
| `bool foc.begin(supply_V, limit_V, align_V, zero_electric_angle, sensor_direction)` | The same with the **stored alignment** (the shaft does not move). Use the two `#define` lines printed by `motor_AS5600`: `STORED_ZERO_ELECTRIC_ANGLE`, `STORED_SENSOR_DIRECTION`. `NOT_SET` / `Direction::UNKNOWN` measure that part |
| `bool foc.beginHardware(supply_V, limit_V, align_V)` + `int foc.findPolePairs(voltage = 1.0f)` + `bool foc.alignFOC()` | Unknown motor: init without alignment, estimate the pole pairs (shaft free), then align |
| `foc.printSummary(Serial)` / `foc.printAlignment(Serial)` | Pole pairs, direction, zero angle / the two `#define` lines to paste |

The stored alignment stays valid while the magnet, the sensor mounting, the phase wiring and the pole pairs do not
change. With the pendulum mounted always use it: a normal alignment is disturbed by the weight of the arm.

### 2.2 Running

| Call | What it does |
|---|---|
| `foc.update()` | `motor.loopFOC()` + `motor.move(target)`. **Every loop, no `delay()`** |
| `foc.setTargetVoltage(V)` / `foc.getTargetVoltage()` | Target voltage, clamped to `±voltage_limit` |
| `foc.setTorqueConstant(k)` / `foc.getTorqueConstant()` | `k` in N·m/V (Lab 1, `torque_fit.py`). 0 = not calibrated |
| `foc.setBackEmfConstant(Ke)` / `foc.getBackEmfConstant()` | `Ke` in V·s/rad (Lab 1, `velocity_fit.py`). 0 = no compensation |
| `bool foc.setTargetTorque(torque)` | `V = torque / k` (clamped). `false` if `k` is not set |
| `bool foc.setTargetTorque(torque, speed)` | `V = torque / k + Ke·speed` (back-EMF compensation) |
| `foc.getTargetTorque()` / `foc.getMaxTorque()` | Torque of the present voltage / `k·voltage_limit` |
| `foc.target_voltage` | The present target (V) |
| `bool foc.receiveSerialCommand()` | Lab 1 sketches: reads a number from the Serial Monitor as the target voltage; `p` pauses the status line, `h` help |
| `foc.getReportedPosition()` / `foc.resetReportedZero()` | Position relative to the zero taken after the alignment (rad) |
| `foc.getStatusString()` / `foc.printSensorInfo(Serial)` | One status line / raw angle and magnet status |
| `foc.isPrinting()` / `foc.setPrinting(bool)` | Printing flag toggled by `p` |
| `foc.motor`, `foc.driver`, `foc.sensor` | The SimpleFOC objects, e.g. `foc.motor.sensor_direction` |

In voltage mode the real torque is `τ = k·(V − Ke·ω)`: the back-EMF eats part of the voltage at speed. That is
why the pendulum sketches always command torque with the speed (`pendulum.applyTorque()` does it for you).

---

## 3. `AS5600_lar` — the sensor

Free functions (one sensor). `FOC_lar` already reads it; call these directly only in sensor sketches.

| Call | What it does |
|---|---|
| `bool AS5600_lar_begin()` | Starts I2C (400 kHz) and the sensor. `false` if no sensor answers |
| `bool update_sensor()` | Reads the angle and updates position, turns and velocity (velocity every 10 ms). Call often: the shaft must turn less than 0.8 rev between two calls |
| `double get_mechanical_angle()` | Angle in [0, 2π) |
| `double get_position()` | Continuous position `turns·2π + angle` (rad) |
| `signed long get_full_rotations()` | Signed full turns |
| `unsigned int get_raw_angle()` | Last raw count, 0…4095 (1 count = 1.53 mrad) |
| `double get_velocity_rad_sec()` / `get_velocity_rpm()` | Velocity, EMA-filtered if the filter is on |
| `double get_velocity_raw_rad_sec()` / `get_velocity_raw_rpm()` | Velocity, unfiltered |
| `bool set_velocity_ema_filter(bool on, double alpha)` | EMA filter, `0 < α ≤ 1` (1 = none). Call before `AS5600_lar_begin()` / `foc.begin()` |
| `bool print_magnet_status(Serial)` / `bool read_sensor_status(status)` | Magnet detected / too weak / too strong |
| `void print_sensor_status(Serial, position_zero = 0)` | One line: raw, angle, position, turns, velocities |
| `double raw2rad(raw)` / `double raw2deg(raw)` | Count → rad / deg |

---

## 4. `LoadCell_lar` — the load cell (Lab 1)

```cpp
LoadCell_lar cell(dout_pin, sck_pin);
```

| Call | What it does |
|---|---|
| `bool cell.begin(cal_factor = 1.0f, stabilizing_ms = 2000, do_tare = true)` | Starts the HX711, waits, tares. **Blocking, start-up only** |
| `bool cell.update()` / `bool cell.read(grams)` | Non-blocking; `true` on a new sample (every 100 ms). Call every loop |
| `float cell.getGrams()` / `getForceN()` / `getTorque(arm_length_m)` | Mass (g) / force (N) / torque (N·m) for an arm from the shaft axis |
| `cell.tare()` / `cell.tareNoDelay()` + `cell.isTareDone()` | Zero (blocking ~2 s / in the background) |
| `float cell.calibrate(known_mass_g)` | Tare first, place the known mass, then call it: sets and returns the factor (blocking ~2 s) |
| `cell.setCalFactor(f)` / `getCalFactor()` / `saveCalFactor(addr)` / `loadCalFactor(addr)` | Calibration factor, in EEPROM |
| `cell.handleSerialCommands()` | Non-blocking keys: `t` tare, `r` calibrate, `c` factor, `s`/`l` save/load, `p` pause, `h` help |
| `cell.setReverseOutput()`, `cell.setSamplesInUse(n)`, `cell.hasError()` | Sign, averaging length, HX711 stopped answering |

The reading is a moving average: wait ~2 s after a load change. The interactive helpers (`waitForKey`,
`calibrateInteractive`, ...) block `loop()`: do not use them while the motor runs.

---

## 5. Tools (Python)

`pip install numpy matplotlib pyserial` (pyserial only to read the serial port). **Close the Serial Monitor** before
a script opens the port.

| Tool | Use |
|---|---|
| `labN/tools/plot_log.py --port /dev/ttyUSB0 --out run.csv` | Waits for `#READY`, asks you to press Enter, sends `s`, saves the CSV of the run and plots angle, velocity, torque, voltage and the phase plane, with a summary (capture time, falls, RMS angle error, peak torque and voltage). On Windows `--port COM3` |
| `labN/tools/plot_log.py --csv run.csv` | Plots a saved log (also the CSV exported by the GUI) |
| `lab1/tools/torque_fit.py`, `velocity_fit.py`, `pendulum_ringdown_fit.py` | Lab 1 fits: `k`, `Ke`, `b`, `τ_c` (`--port` or `--csv`) |
| `./gui/run_gui.sh` | The pendulum simulator (Labs 2–4): try your gains, see the plots and the animation, export a single plot or a GIF |

---

## 6. Common mistakes

| Symptom | Cause |
|---|---|
| Nothing happens after `s` | Check the baud rate (115200), that a run is not already active, and that `pendulum.update()` is called on every loop |
| The pendulum turns a full revolution to reach the reference | The error is not wrapped: use `pendulum_wrap_angle()` |
| The motor pushes away from the reference | Sign error in the controller, or wrong stored sensor direction |
| The controller works in simulation, not on the rig | Wrong `ROD_MASS`, `POINT_MASS`, `PENDULUM_LENGTH` (they set `J`, `G`), or `TORQUE_CONSTANT` |
| Upright reference never reached when starting near the top | `s` was sent with the pendulum held at the top: the zero is taken at `s`, so start hanging and lift it afterwards |
| Jerky motor, "longest computation" close to the period | Too much work per step (printing inside the controller, `delay()`) |
| Integral keeps growing while the torque is at the limit | Missing anti-windup: integrate only while not saturated (`pendulum.maxTorque()`) |
