#ifndef PENDULUM_LAR_H
#define PENDULUM_LAR_H

// Pendulum_lar
//
// Everything a pendulum sketch needs besides FOC_lar and the controller itself, in ONE object:
//
//   - the model of the pendulum (inherited from PendulumModel, see PendulumModel.h):
//         J * theta'' = tau - b * theta' - tau_c * sign(theta') - G * sin(theta)
//     inertia(), gravityTorque(), energy(), linearizeAt(), discretizeUpright(), ...
//   - the measured state: theta (rad, 0 = hanging at the start command, pi = upright), omega (rad/s)
//   - the timing of the control loop, the serial start / stop protocol and the CSV log
//   - the torque command (back-EMF compensation by FOC_lar + optional Coulomb friction feed-forward)
//   - the swing-up <-> balance switching (updateMode()), for the sketches that need it
//
// The controllers (PID, LQR, MPC, TVLQR) and the energy swing-up are NOT here: you write them in your
// sketch. Instructor reference implementations: lab_code/scripts/PendulumReference.h.
//
// Students' version (lab_code/libs): some functions of Pendulum_lar.cpp and PendulumModel.cpp are TODOs,
// with instructions in the code, marked with the lab that needs them (Lab 2 Part 1, Lab 2 Part 2, Lab 3).
// Until a later-lab TODO is completed it returns a harmless placeholder, so the earlier parts still run.
// Complete version (instructors): lab_code/libs_solution.
//
// Typical use:
//
//   FOC_lar foc(7, 11, 10, 9, 8);
//   Pendulum_lar pendulum;
//
//   void setup() {
//       Serial.begin(115200);
//       foc.begin(...);  foc.setTorqueConstant(...);  foc.setBackEmfConstant(...);
//       pendulum.begin(foc, 0.01f, 0.5f, 30.0f);                       // Ts, velocity filter, run time
//       pendulum.setParameters(ROD_MASS, POINT_MASS, LENGTH, B, TAU_C); // the model
//       pendulum.setCoulombCompensation(true);
//   }
//   void loop() {
//       foc.update();
//       if (!pendulum.update()) return;            // true once per control period while a run is active
//       float tau = ... your controller, from pendulum.theta and pendulum.omega ...;
//       pendulum.applyTorque(tau);
//       pendulum.log();
//   }
//
// Serial protocol (115200 baud): '#READY' is printed once; 's' starts a run, 'x' stops it (0 V),
// 'p' pauses / resumes the CSV log. Every other key (e.g. 'h', help) is passed to the sketch through readCommand().
// CSV log: t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode      (mode 0 = swing-up, 1 = balance)
// (torque_Nm is the controller's torque, without the Coulomb compensation; voltage_V is the voltage applied)

#include <Arduino.h>
#include <FOC_lar.h>
#include "PendulumModel.h"

#ifndef PENDULUM_CAPTURE_SPEED
#define PENDULUM_CAPTURE_SPEED 8.0f // rad/s: above this |omega| the pendulum is too fast to be caught
#endif

class Pendulum_lar : public PendulumModel
{
public:
    enum Mode
    {
        SWING_UP = 0,
        BALANCE = 1
    };

    Pendulum_lar();

    // ---- setup() ------------------------------------------------------------------------------------

    // control_period_s : control period Ts (s)
    // velocity_alpha   : velocity filter, in (0, 1]: 1 = no filtering, smaller = smoother but slower
    // run_time_s       : the run stops by itself after this time
    // log_period_ms    : period of the CSV log
    void begin(FOC_lar &foc,
               float control_period_s,
               float velocity_alpha,
               float run_time_s,
               unsigned long log_period_ms = 20);

    // setParameters(rod_mass, point_mass, length, viscous_friction, coulomb_friction) sets the model:
    // inherited from PendulumModel.

    // Coulomb friction feed-forward (default: off). When on, applyTorque() adds tau_c * sign(omega)
    // (tau_c = coulombFriction()) to the torque, so that the motor supplies the friction torque.
    void setCoulombCompensation(bool enabled);

    // Thresholds of updateMode() (default 0.4 rad and 1.0 rad), see below
    void setCaptureAngles(float capture_angle, float release_angle);

    // ---- loop() -------------------------------------------------------------------------------------

    // Call on every loop(), right after foc.update(). Handles the serial keys ('s', 'x', 'p'), the run
    // time and the timing. Returns true once per control period while a run is active: theta, omega and
    // dt have just been updated, compute and apply your command now.
    bool update(void);

    // true once, right after the start command (reset your controller here)
    bool started(void);

    bool isRunning(void) const;

    // Next key that update() did not handle itself ('s', 'x', 'p' and newlines are consumed there),
    // or 0 when no key is pending. Use it for the keys of your sketch.
    char readCommand(void);

    // Prints the keys of Pendulum_lar ('s', 'x', 'p') and 'h'; the sketch adds its own keys
    void printHelp(Print &out = Serial);

    // Measured state, updated by update()
    float theta; // rad, continuous (no wrapping), 0 = hanging at the start command
    float omega; // rad/s, finite difference of theta + first-order filter
    float dt;    // actual control period of the last update (s)

    // Largest torque the motor can apply with the voltage limit (FOC_lar::getMaxTorque())
    float maxTorque(void) const;

    // ---- commands -----------------------------------------------------------------------------------

    // Torque (N*m) through FOC_lar (torque constant + back-EMF compensation), + the Coulomb friction
    // compensation if it is on
    void applyTorque(float torque_nm);

    void stop(void); // motor at 0 V, end of the run

    // ---- swing-up <-> balance switching (hysteresis) ------------------------------------------------

    // From the present theta and omega:
    //   SWING_UP -> BALANCE when |theta - pi| < capture_angle and |omega| < PENDULUM_CAPTURE_SPEED
    //   BALANCE -> SWING_UP when |theta - pi| > release_angle (the pendulum fell)
    // The mode is reset to SWING_UP at every start command.
    Mode updateMode(void);
    Mode mode(void) const;
    void resetMode(Mode mode = SWING_UP);

    // ---- log ----------------------------------------------------------------------------------------

    // Prints one CSV line (rate limited by log_period_ms): log() writes mode(), log(m) writes m
    void log(void);
    void log(int mode);

private:
    FOC_lar *_foc;
    float _Ts;
    float _alpha;
    float _theta_prev;
    float _run_time;
    unsigned long _log_period_ms;
    double _zero;
    bool _running;
    bool _started;
    bool _ready_printed;
    bool _printing;
    bool _coulomb_comp;
    unsigned long _t_start_ms;
    unsigned long _t_last_us;
    unsigned long _t_tick_us;
    unsigned long _t_log_ms;
    unsigned long _max_compute_us;
    float _torque_cmd;
    float _voltage_cmd;
    float _capture_angle;
    float _release_angle;
    Mode _mode;
    char _commands[8];
    unsigned char _command_head;
    unsigned char _command_tail;

    void pushCommand(char c);
    void serviceSerial(void);
    void startRun(void);
    void endRun(bool aborted);
    void recordComputeTime(void);
};

#endif
