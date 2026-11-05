// Lab 2: PID control of the pendulum, TORQUE command.   (student sketch: complete the TODOs)
//
//   theta = 0 : hanging down      theta = pi : upright      (positive theta = positive torque)
//
// PART 1 - PID only (USE_SWING_UP 0). Your PID holds the selected equilibrium (keys '1' and '2'):
//   1 HANGING (stable)   : reference theta = 0. The PID is always in control.
//   2 UPRIGHT (unstable) : reference theta = pi. From the bottom the PID cannot lift the pendulum (the torque
//                          saturates and the pendulum only tilts): YOU bring it by hand close to the top, in
//                          the direction the motor pushes; then your PID balances it.
// PART 2 - PID + swing-up (USE_SWING_UP 1, or key 'u'). Upright only: instead of zero torque, your energy
//   swing-up pumps the pendulum from the bottom up to the top, where the PID takes over.
//
// Given: the settings below, the print helpers and the serial keys ('1', '2', 'u', 'h').
// Yours (the TODOs): the PID/controller functions below, and the start-up of the motor and pendulum in
// setup() and the control loop in loop(). The library Pendulum_lar handles timing, the start/stop keys and
// the CSV log. The applied torque goes through back-EMF compensation (when enabled) and Coulomb-friction
// feed-forward (when enabled), so the model uses the mechanical viscous friction when back-EMF compensation
// is enabled.
// Yours too, in the library (lab_code/libs/Pendulum_lar, the copy in your Arduino libraries folder), with
// instructions in the code:
//   Part 1: Pendulum_lar.cpp   update(): dt, theta, omega; applyTorque(): Coulomb feed-forward, torque command
//           PendulumModel.cpp  setParameters(): J, G, b, tau_c
//   Part 2: Pendulum_lar.cpp   updateMode() (the Part 1 stub always returns BALANCE)
//           PendulumModel.cpp  energy()
//   The other TODOs of PendulumModel.cpp are for Lab 3: leave them for now.
//
// Procedure:
//   1. Fill in the calibration results and pendulum parameters below (Lab 1 rig card).
//   2. Paste the stored alignment values printed by motor_AS5600 (Lab 1), so that the pendulum does not move
//      at power-up.
//   3. Let the pendulum hang at rest, open the Serial Monitor (115200) and send 's'. Send 'x' to stop (0 V).
//
// Log (CSV): t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode   (mode 0 = motor off / swing-up, 1 = balance)
//   capture and plot it with tools/plot_log.py
//
// WARNING: the pendulum moves fast and can hit things. Keep the area clear.

#include <FOC_lar.h>
#include <Pendulum_lar.h>

#define EQUILIBRIUM 1  // equilibrium at power-up: 1 HANGING, 2 UPRIGHT (keys '1' / '2')
#define USE_SWING_UP 0 // 0: PART 1 (PID only), 1: PART 2 (swing-up + PID) (key 'u')

// Motor definitions

#define MOTOR_POLE_PAIRS 7
#define MOTOR_SUPPLY_VOLTAGE 12.0f
#define MOTOR_VOLTAGE_LIMIT 3.0f // limits the torque: max torque = TORQUE_CONSTANT * this
#define SENSOR_ALIGN_VOLTAGE 1.0f

#define STORED_ZERO_ELECTRIC_ANGLE NOT_SET // paste the values printed by motor_AS5600
#define STORED_SENSOR_DIRECTION Direction::UNKNOWN

// Calibration results (Lab 1)

#define TORQUE_CONSTANT NOT_SET // N*m/V    <- REPLACE
#define MOTOR_KE NOT_SET        // V*s/rad  <- REPLACE (low-speed Ke)
#define USE_BACK_EMF_COMP 1     // 1: the torque command includes the back-EMF compensation

// Pendulum parameters (Lab 1)

#define ROD_MASS NOT_SET         // kg   <- REPLACE
#define POINT_MASS NOT_SET       // kg   <- REPLACE
#define PENDULUM_LENGTH NOT_SET  // m    <- REPLACE
#define VISCOUS_FRICTION NOT_SET // N*m*s/rad  <- REPLACE (ring-down)
#define COULOMB_FRICTION NOT_SET // N*m, tau_c  <- REPLACE (ring-down)
#define USE_COULOMB_COMP 1       // 1: add tau_c*sign(omega) to the torque (Coulomb friction compensation)

// Run

#define CONTROL_PERIOD_S 0.01f
#define VELOCITY_ALPHA 1.0f
#define RUN_TIME_S 60.0f
#define CAPTURE_ANGLE 0.4f // rad: pendulum.updateMode() switches to BALANCE closer than this to the top ...
#define RELEASE_ANGLE 1.0f // rad: ... and back to SWING_UP farther than this (the pendulum fell)

// PID design (Part 1)

#define PID_WN_HANGING NOT_SET         // rad/s  closed-loop natural frequency, hanging
#define PID_WN_UPRIGHT NOT_SET         // rad/s  closed-loop natural frequency, upright
#define PID_ZETA NOT_SET               // damping ratio
#define PID_KI_OVER_KP_HANGING NOT_SET // ki = this * kp   (1/s); 0 = PD
#define PID_KI_OVER_KP_UPRIGHT NOT_SET

// Energy swing-up (Part 2)

#define SWING_ENERGY_RATIO NOT_SET // target energy / energy at the top
#define SWING_ENERGY_BAND NOT_SET  // energy error, as a fraction of E_up, over which the torque ramps to full
#define SWING_KICK_SPEED NOT_SET   // below this |omega| (rad/s) the last push direction is kept

// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(MOTOR_POLE_PAIRS, 11, 10, 9, 8);
Pendulum_lar pendulum; // model, measured state, timing, log, swing-up/balance switching

int equilibrium = EQUILIBRIUM;    // 1 hanging, 2 upright
bool use_swing_up = USE_SWING_UP; // upright only: false = Part 1 (PID only), true = Part 2 (swing-up + PID)

// ============================== print helpers ==============================

void printGains(float kp, float ki, float kd)
{
    Serial.print(F("#PID: kp = "));
    Serial.print(kp, 4);
    Serial.print(F(" N*m/rad, ki = "));
    Serial.print(ki, 4);
    Serial.print(F(" N*m/(rad*s), kd = "));
    Serial.print(kd, 5);
    Serial.println(F(" N*m*s/rad"));
}

void printModel(void)
{
    Serial.print(F("#J = "));
    Serial.print(pendulum.inertia() * 1e6f, 1);
    Serial.print(F("e-6 kg*m^2, G = "));
    Serial.print(pendulum.gravityTorque() * 1000.0f, 2);
    Serial.print(F(" mN*m, max torque = "));
    Serial.print(pendulum.maxTorque() * 1000.0f, 2);
    Serial.print(F(" mN*m, sqrt(G/J) = "));
    Serial.print(pendulum.naturalFrequency(), 2);
    Serial.println(F(" rad/s"));
}

// ============================== library reference ==============================
// What the libraries give you (complete list: FOC_libraries_student_reference.md, FOC_lar and Pendulum_lar sections):
//   foc.begin(...), foc.update(), foc.setTorqueConstant(...), foc.setBackEmfConstant(...)
//   pendulum.begin(...), pendulum.theta, pendulum.omega, pendulum.dt
//   pendulum.inertia(), pendulum.gravityTorque(), pendulum.viscousFriction(), pendulum.naturalFrequency()
//   pendulum.energy(...), pendulum.energyUpright(), pendulum.maxTorque()
//   pendulum.updateMode(), pendulum.mode()   Pendulum_lar::SWING_UP / Pendulum_lar::BALANCE
//   pendulum.resetMode(), pendulum.started(), pendulum.readCommand(), pendulum.printHelp()
//   pendulum.setParameters(...), pendulum.setCoulombCompensation(...), pendulum.setCaptureAngles(...)
//   pendulum.applyTorque(...), pendulum.log()
//   pendulum_wrap_angle(a), pendulum_clamp(x, low, high), pendulum_sign(x), PENDULUM_LAR_PI
//   fabsf(x), sinf(x), cosf(x), sqrtf(x)     <math.h>
//
// Library TODOs for this lab:
//   Part 1: Pendulum_lar.cpp / update() and applyTorque(); PendulumModel.cpp / setParameters().
//   Part 2: Pendulum_lar.cpp / updateMode(); PendulumModel.cpp / energy().
//     - updateMode(): in SWING_UP, capture only when the pendulum is within the capture angle of upright
//       AND |omega| is below 8 rad/s. In BALANCE, release back to SWING_UP when it falls beyond the release angle.
//     - energy(): implement the pendulum's total mechanical energy using the model parameters.
//   Leave the other PendulumModel.cpp TODOs for Lab 3.

// ============================== YOUR CONTROLLER ==============================

// PID controller: keep its gains and accumulated error together.
struct Pid
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f; // integral of the error (rad*s)

    void reset()
    {
        // TODO: clear the controller's stored integral state.
    }

    // error = reference - theta (rad), omega = measured velocity (rad/s).
    // Returns the commanded torque (N*m), limited to +/- the supplied torque limit.
    float update(float error, float omega, float dt, float limit)
    {
        // TODO (Part 1): calculate the PID torque using the current error, stored integral and measured velocity.
        //   tau = kp*error + ki*integral - kd*omega
        //   Use measured omega for the derivative term (do not differentiate the error).

        // TODO (Part 1): integrate the error using the actual sample interval: integral += error * dt.
        // TODO (Part 2): add anti-windup: update the integral only when the calculated torque is not saturated
        //   (fabsf(tau) < limit). After integrating, if ki > 0, bound the integral so its torque contribution
        //   is at most half the available torque limit in either direction.
        // TODO (Part 2): clamp the calculated torque to +/- limit and return it.
        return 0.0f;
    }
};

Pid pid;
float swing_direction = 1.0f; // remembered push direction; initial direction is positive

void designController(void)
{
    // TODO (Part 1): obtain J, G and b from the pendulum model and select the settings for the chosen equilibrium.
    //   Calculate kp and kd using the model and the PID design settings from the handout.
    //   Calculate ki from the selected integral-to-proportional gain ratio.
    //   The hanging and upright gains differ in the sign of the gravity term. Do not allow negative gains.
    //   Store the gains in the PID structure, then print them with printGains() and compare with your hand result.
}

void clearControllerState(void)
{
    // TODO (Part 1): reset the PID controller's stored state.
    // TODO (Part 2): reset the remembered swing-up push direction to +1.
}

float swingUpTorque(float theta, float omega)
{
    // TODO (Part 2): update the persistent push direction only when |omega| is greater than the configured
    //   low-speed threshold; use pendulum_sign() for its sign. Otherwise, keep the previous direction.
    //   The direction must be stored between calls (do not make it a local variable). It is initialized to +1
    //   and restored to +1 by clearControllerState().
    // TODO (Part 2): get the current energy, upright energy and maximum torque from the pendulum model.
    //   Calculate the normalized energy error using the configured energy ratio and energy band.
    //   Clamp this coefficient to [-1, +1] with pendulum_clamp(), then multiply by the maximum torque and
    //   the remembered push direction. Return the resulting torque.
    return 0.0f;
}

float controlTorque(void)
{
    // TODO (Part 1): get the maximum available torque from the pendulum model.
    //
    // HANGING (selected equilibrium 1):
    //   - Always use the PID, with reference angle 0.
    //   - Calculate reference - measured angle and wrap the error to [-pi, pi) with pendulum_wrap_angle().
    //   - Call the PID update with the error, measured velocity, actual sample interval and torque limit.
    //
    // UPRIGHT (selected equilibrium 2):
    //   - Save the current mode BEFORE calling pendulum.updateMode(), then call it.
    //   - If the resulting mode is BALANCE, use the PID with reference angle pi and a wrapped error.
    //   - If the mode has just changed from SWING_UP to BALANCE, reset the PID integral before the PID update.
    //   - If the resulting mode is SWING_UP, return zero torque for Part 1. For Part 2, when swing-up is enabled,
    //     return swingUpTorque() instead.
    //
    // In Part 1, updateMode() in the library is still a stub that always returns BALANCE. In Part 2, your
    //   library implementation makes the SWING_UP branch active.
    return 0.0f;
}

// ============================== serial commands ==============================

void printState(void)
{
    Serial.print(equilibrium == 1 ? F("#equilibrium 1: hanging") : F("#equilibrium 2: upright"));
    if (equilibrium == 2)
    {
        Serial.print(use_swing_up ? F(", swing-up + PID (Part 2)") : F(", PID only: lift the pendulum by hand (Part 1)"));
    }
    Serial.println();
}

void printHelp(void)
{
    Serial.println(F("#Serial commands:"));
    pendulum.printHelp();
    Serial.println(F("1 : equilibrium 1: hanging, hold theta = 0"));
    Serial.println(F("2 : equilibrium 2: upright, balance theta = pi"));
    Serial.println(F("u : upright: swing-up helper on / off (Part 2 / Part 1)"));
    Serial.println(F("***"));
}

// Applies the keys read by Pendulum_lar::update()
void handleCommands(void)
{
    char c;
    while ((c = pendulum.readCommand()) != 0)
    {
        switch (c)
        {
        case '1':
        case '2':
            equilibrium = c - '0';
            pendulum.resetMode();
            designController();
            clearControllerState();
            printState();
            break;

        case 'u':
            use_swing_up = !use_swing_up;
            pendulum.resetMode();
            clearControllerState();
            printState();
            break;

        case 'h':
            printHelp();
            break;

        default:
            break;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("#Lab 2: PID control of the pendulum"));

    // TODO (Part 1): initialize the motor and pendulum in this order (use the settings above):
    //   1. Call foc.begin() with the supply voltage, voltage limit, alignment voltage, stored electrical zero
    //      and sensor direction. If it returns false, stop here with while (1).
    //   2. Call foc.setTorqueConstant() with the calibrated torque constant. Call foc.setBackEmfConstant()
    //      with the measured back-EMF constant when compensation is enabled, otherwise with zero.
    //   3. Call pendulum.begin() with the motor, control period, velocity-filter coefficient and run time.
    //   4. Call pendulum.setParameters() with the rod mass, point mass, length, viscous friction and Coulomb
    //      friction. With back-EMF compensation disabled, add the electrical damping (torque constant times
    //      back-EMF constant) to the measured mechanical viscous friction; otherwise use the measured value.
    //   5. Call pendulum.setCoulombCompensation() and pendulum.setCaptureAngles() with the selected settings.
    //   6. Call designController() to calculate the gains for the selected equilibrium.

    // Given: keep these print calls after model initialization; printModel() needs the model parameters.
    printModel();
    printState();
    printHelp();
}

void loop()
{
    // TODO (Part 1): complete the real-time control loop. loop() runs thousands of times per second; never use delay().
    //   1. Call foc.update() on EVERY call to loop(), first; the FOC algorithm must keep running continuously.
    //   2. Call pendulum.update(). It reads 's', 'x' and 'p', handles run time and timing, and returns true once
    //      per control period while a run is active, with fresh angle, velocity and dt measurements.
    //   3. Call handleCommands() immediately after pendulum.update(), so '1', '2', 'u' and 'h' are handled.
    //   4. If pendulum.started() is true, clear the controller state.
    //   5. If there is no new control sample, return; do not calculate or apply a new torque.
    //   6. Calculate the torque with controlTorque(), apply it with pendulum.applyTorque(), and write the CSV
    //      sample with pendulum.log().
}
