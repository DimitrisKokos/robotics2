#include "Pendulum_lar.h"

// STUDENTS' VERSION: the parts marked TODO are yours (read the instructions in each one):
//   Lab 2, Part 1 : in update() the control period dt, the angle theta and the velocity omega;
//                   in applyTorque() the Coulomb feed-forward and the torque command
//   Lab 2, Part 2 : updateMode(), the swing-up <-> balance switching
// Until you complete a TODO it returns a harmless placeholder, so that the sketch compiles and runs.

Pendulum_lar::Pendulum_lar()
    : theta(0.0f), omega(0.0f), dt(0.0f), _foc(0), _Ts(0.01f), _alpha(1.0f), _theta_prev(0.0f),
      _run_time(20.0f), _log_period_ms(20), _zero(0.0), _running(false), _started(false),
      _ready_printed(false), _printing(true), _coulomb_comp(false),
      _t_start_ms(0), _t_last_us(0), _t_tick_us(0), _t_log_ms(0), _max_compute_us(0),
      _torque_cmd(0.0f), _voltage_cmd(0.0f), _capture_angle(0.4f), _release_angle(1.0f), _mode(SWING_UP),
      _command_head(0), _command_tail(0)
{
}

// Setup

void Pendulum_lar::begin(FOC_lar &foc,
                         float control_period_s,
                         float velocity_alpha,
                         float run_time_s,
                         unsigned long log_period_ms)
{
    _foc = &foc;
    _Ts = control_period_s;
    _alpha = pendulum_clamp(velocity_alpha, 0.01f, 1.0f);
    _run_time = run_time_s;
    _log_period_ms = log_period_ms;
    _running = false;
    _ready_printed = false;
}

void Pendulum_lar::setCoulombCompensation(bool enabled)
{
    _coulomb_comp = enabled;
}

void Pendulum_lar::setCaptureAngles(float capture_angle, float release_angle)
{
    _capture_angle = capture_angle;
    _release_angle = release_angle;
}

// Serial keys

void Pendulum_lar::pushCommand(char c)
{
    unsigned char next = (unsigned char)((_command_tail + 1) % sizeof(_commands));
    if (next != _command_head)
    {
        _commands[_command_tail] = c;
        _command_tail = next;
    }
}

char Pendulum_lar::readCommand(void)
{
    if (_command_head == _command_tail)
    {
        return 0;
    }
    char c = _commands[_command_head];
    _command_head = (unsigned char)((_command_head + 1) % sizeof(_commands));
    return c;
}

void Pendulum_lar::printHelp(Print &out)
{
    out.println(F("s : start a run (when idle; the pendulum must hang at rest)"));
    out.println(F("x : stop the run (motor 0 V)"));
    out.println(F("p : pause / resume the CSV log"));
    out.println(F("h : this help"));
}

void Pendulum_lar::serviceSerial(void)
{
    bool start_requested = false;
    bool abort_requested = false;

    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n')
        {
            continue;
        }
        if (c == 's')
        {
            start_requested = !_running;
        }
        else if (c == 'x')
        {
            abort_requested = _running;
        }
        else if (c == 'p')
        {
            _printing = !_printing;
            Serial.println(_printing ? F("#printing resumed") : F("#printing paused"));
        }
        else
        {
            pushCommand(c);
        }
    }

    if (_running)
    {
        bool time_over = (millis() - _t_start_ms) >= (unsigned long)(_run_time * 1000.0f);
        if (abort_requested || time_over)
        {
            endRun(abort_requested);
        }
        return;
    }

    if (!_ready_printed)
    {
        _ready_printed = true;
        Serial.println(F("#READY: pendulum hanging at rest, then send 's'"));
    }
    if (start_requested)
    {
        startRun();
    }
}

void Pendulum_lar::startRun(void)
{
    _zero = get_position();
    theta = 0.0f;
    omega = 0.0f;
    dt = 0.0f;
    _theta_prev = 0.0f;
    _max_compute_us = 0;
    _torque_cmd = 0.0f;
    _voltage_cmd = 0.0f;
    _mode = SWING_UP;
    _foc->setTargetVoltage(0.0f);

    _running = true;
    _started = true;
    _t_start_ms = millis();
    _t_last_us = micros();
    _t_log_ms = 0;

    Serial.println(F("t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode"));
}

void Pendulum_lar::endRun(bool aborted)
{
    stop();
    Serial.println(aborted ? F("#ABORTED") : F("#DONE"));
    Serial.print(F("#Longest controller computation: "));
    Serial.print(_max_compute_us);
    Serial.print(F(" us (control period "));
    Serial.print((unsigned long)(_Ts * 1e6f));
    Serial.println(F(" us)"));
    Serial.println(F("#Send 's' to repeat."));
}

// Loop

bool Pendulum_lar::update(void)
{
    serviceSerial();
    if (!_running)
    {
        return false;
    }

    // wait for the control period
    unsigned long now = micros();
    if (now - _t_last_us < (unsigned long)(_Ts * 1e6f))
    {
        return false;
    }
    // TODO (Lab 2, Part 1): the actual control period dt, in seconds (s).
    //   now and _t_last_us are times in microseconds, as returned by micros() (unsigned long): now is the time
    //   of this control step, _t_last_us the time of the previous one (or of the start command).
    //   - dt is their difference, converted to seconds (1 us = 1e-6 s), as a float.
    //   - Subtract first, in unsigned long, and convert the difference afterwards: micros() overflows back to 0
    //     about every 70 minutes, and the unsigned subtraction still gives the right difference then.
    //   - Compute dt BEFORE the line _t_last_us = now below (it overwrites the previous time).
    //   dt is close to the control period Ts but not equal: loop() is late by up to one foc.update().
    dt = 0.0f; // <- replace
    _t_last_us = now;
    _t_tick_us = now;

    // TODO (Lab 2, Part 1): the angle theta (rad), continuous, 0 = the position at the start command.
    //   - get_position() (AS5600_lar) returns the continuous position of the sensor (rad, double): the number
    //     of full turns * 2*pi + the angle within the turn, so it does not jump when the angle crosses 2*pi.
    //   - _zero is get_position() at the start command ('s', the pendulum hanging): theta = position - _zero.
    //   - The sensor may count in the direction opposite to a positive voltage. The alignment of SimpleFOC
    //     found the direction, _foc->motor.sensor_direction: Direction::CW (the same direction, sign +1) or
    //     Direction::CCW (opposite, sign -1). Multiply by that sign, so that a positive torque increases theta.
    //   - Do the subtraction in double (the position grows with every turn, and float has only ~7 digits),
    //     then convert the result to float: theta = (float)(...).
    theta = 0.0f; // <- replace

    // TODO (Lab 2, Part 1): the angular velocity omega (rad/s), from the angle (the sensor gives no velocity
    // synchronous with the control period):
    //   1. raw = (theta - _theta_prev) / dt: the finite difference over this control period; _theta_prev is
    //      the angle of the previous step (0 at the start command, where theta is 0 too);
    //   2. a first-order low-pass filter: omega = _alpha * raw + (1 - _alpha) * omega, with the previous omega
    //      on the right; _alpha is VELOCITY_ALPHA of the sketch (1 = no filtering, smaller = smoother but late);
    //   3. _theta_prev = theta, for the next step.
    omega = 0.0f; // <- replace
    return true;
}

bool Pendulum_lar::started(void)
{
    bool s = _started;
    _started = false;
    return s;
}

bool Pendulum_lar::isRunning(void) const
{
    return _running;
}

float Pendulum_lar::maxTorque(void) const
{
    return _foc ? _foc->getMaxTorque() : 0.0f;
}

// Commands

void Pendulum_lar::recordComputeTime(void)
{
    unsigned long used = micros() - _t_tick_us;
    if (used > _max_compute_us)
    {
        _max_compute_us = used;
    }
}

void Pendulum_lar::applyTorque(float torque_nm)
{
    _torque_cmd = torque_nm;

    // TODO (Lab 2, Part 1): the Coulomb friction feed-forward friction_ff (N*m).
    //   The friction torque of the shaft opposes the motion, tau_c * sign(omega); the motor supplies it on top of
    //   the controller's torque, so that the controller sees a plant without Coulomb friction.
    //   - When the compensation is on (_coulomb_comp, set by setCoulombCompensation()):
    //     friction_ff = tau_c * sign(omega), with tau_c = coulombFriction() (the model) and pendulum_sign(omega).
    //   - When it is off: friction_ff = 0.
    float friction_ff = 0.0f; // <- replace

    // TODO (Lab 2, Part 1): send the total torque to the motor.
    //   Look at FOC_lar::setTargetTorque(float torque_nm, float speed_rad_s) in FOC_lar.h and FOC_lar.cpp: it
    //   converts a torque into the voltage V = torque / k + Ke * speed (the torque constant k and the back-EMF
    //   compensation Ke * speed, both set in setup()) and limits V to the voltage limit.
    //   Call it through the pointer _foc (_foc->setTargetTorque(...)) with the total torque, torque_nm +
    //   friction_ff, and the shaft speed omega. Replace the line below, which keeps the motor at 0 V.
    _foc->setTargetVoltage(0.0f); // <- replace
    (void)friction_ff;            // <- remove once you use friction_ff
    _voltage_cmd = _foc->target_voltage;
    recordComputeTime();
}

void Pendulum_lar::stop(void)
{
    _foc->setTargetVoltage(0.0f);
    _running = false;
}

// Swing-up <-> balance switching

Pendulum_lar::Mode Pendulum_lar::updateMode(void)
{
    // TODO (Lab 2, Part 2): the swing-up <-> balance switching, with hysteresis, from theta and omega.
    //   1. The distance from the top, error = |theta - pi|. theta is continuous: after a full turn the top is at
    //      3*pi, -pi, ..., so wrap theta - PENDULUM_LAR_PI to [-pi, pi) with pendulum_wrap_angle() before
    //      taking the absolute value (fabsf()).
    //   2. In SWING_UP (_mode == SWING_UP): switch _mode to BALANCE when the pendulum is close to the top,
    //      error < _capture_angle, AND slow enough to be caught, |omega| < PENDULUM_CAPTURE_SPEED (8 rad/s).
    //   3. In BALANCE: switch _mode back to SWING_UP only when the pendulum is far from the top,
    //      error > _release_angle (it fell). Since _release_angle > _capture_angle, between the two angles the
    //      mode does not change (hysteresis): it does not chatter at the edge of the capture region.
    //   4. Return _mode.
    //   _capture_angle and _release_angle are set by setCaptureAngles(); the start command resets _mode to SWING_UP.
    // Until you write it (Part 1), the balance controller is always in control:
    _mode = BALANCE; // <- replace
    return _mode;
}

Pendulum_lar::Mode Pendulum_lar::mode(void) const
{
    return _mode;
}

void Pendulum_lar::resetMode(Mode mode)
{
    _mode = mode;
}

// Log

void Pendulum_lar::log(void)
{
    log((int)_mode);
}

void Pendulum_lar::log(int mode)
{
    if (!_printing)
    {
        return;
    }

    unsigned long now = millis();
    if (now - _t_log_ms < _log_period_ms)
    {
        return;
    }
    _t_log_ms = now;

    Serial.print((now - _t_start_ms) / 1000.0f, 3);
    Serial.print(',');
    Serial.print(theta, 4);
    Serial.print(',');
    Serial.print(omega, 3);
    Serial.print(',');
    Serial.print(_torque_cmd, 5);
    Serial.print(',');
    Serial.print(_voltage_cmd, 3);
    Serial.print(',');
    Serial.println(mode);
}
