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
// Yours (the TODOs): the section "YOUR CONTROLLER", and the start-up of the motor and of the pendulum in
// setup() and the control loop in loop(). The library Pendulum_lar does the timing, the start / stop keys and
// the CSV log for you; the torque you apply goes through the back-EMF compensation (USE_BACK_EMF_COMP 1) and the
// Coulomb-friction feed-forward (USE_COULOMB_COMP 1), so the model your controller sees has the mechanical
// friction b only.
// Yours too, in the library (lab_code/libs/Pendulum_lar, the copy in your Arduino libraries folder), with
// instructions in the code:
//   Part 1: Pendulum_lar.cpp   update(): dt, theta, omega;  applyTorque(): Coulomb feed-forward, torque command
//           PendulumModel.cpp  setParameters(): J, G, b, tau_c
//   Part 2: Pendulum_lar.cpp   updateMode()  (until then it always returns Pendulum_lar::BALANCE)
//           PendulumModel.cpp  energy()
//   The other TODOs of PendulumModel.cpp are for Lab 3: leave them for now.
//
// Procedure:
//   1. Fill in the calibration results and the pendulum parameters below (Lab 1 rig card).
//   2. Paste the STORED_... alignment lines printed by motor_AS5600 (Lab 1) so that the pendulum does not move at power-up.
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

// ============================== YOUR CONTROLLER ==============================
//
// The rest of the sketch calls the four functions at the end of this section. Keep their names and
// signatures; write their bodies, and add any variables, structs or helper functions you need here.
//
//   void  designController()      compute the gains for the selected equilibrium (equilibrium == 1 or 2).
//                                 Called at power-up and when '1' / '2' is pressed.
//   void  clearControllerState()  clear the memory of your controller (the integral, ...).
//                                 Called at every start ('s') and when '1', '2' or 'u' is pressed.
//   float controlTorque()         called once per control period, with a fresh measurement:
//                                 return the torque (N*m) to apply now.
//   float swingUpTorque(theta, omega)   (Part 2) the energy swing-up torque (N*m); call it from controlTorque().
//
// PART 1 - a PID that holds the selected equilibrium
//   * Keep the PID's data (kp, ki, kd and the integral of the error) together, e.g. in a small struct with
//     a reset() and an update(error, omega, dt) that returns the torque.
//   * The law (handout, section 1): tau = kp*e + ki*integral(e) - kd*omega, with e = reference - theta.
//       - the derivative acts on the MEASURED velocity omega, not on the error
//       - wrap the error to [-pi, pi) with pendulum_wrap_angle(), otherwise a full turn looks like a 2*pi error
//       - integrate with the actual control period: integral += e * dt
//   * designController(): kp and kd from the model and the PID_... defines (handout, section 1), ki = ratio * kp.
//     The two equilibria differ only in the sign of G. Do not allow negative gains. Print the result with
//     printGains(kp, ki, kd) and compare it with your hand calculation.
//   * controlTorque():
//       - hanging (equilibrium == 1): the PID with reference 0, always.
//       - upright (equilibrium == 2): call pendulum.updateMode(). While it returns Pendulum_lar::BALANCE use the
//         PID with reference PENDULUM_LAR_PI; when it returns Pendulum_lar::SWING_UP, return 0 for now.
//         Reset the integral at the moment the mode becomes BALANCE (compare with pendulum.mode() BEFORE
//         calling updateMode()).
//         In Part 1 updateMode() is still a TODO of the library that always returns BALANCE, so the PID is
//         always in control; in Part 2 you write it, and the SWING_UP branch starts to work.
//
// PART 2 - the swing-up
//   * First complete updateMode() (Pendulum_lar.cpp) and energy() (PendulumModel.cpp) in the library.
//   * swingUpTorque(theta, omega): the energy swing-up law (handout, Part 2). In controlTorque(), return it
//     instead of 0 when use_swing_up is true and the mode is SWING_UP.
//       - E = pendulum.energy(theta, omega),  E_up = pendulum.energyUpright(),  tau_max = pendulum.maxTorque()
//       - the push direction is sign(omega) (pendulum_sign()); keep the last direction while
//         |omega| < SWING_KICK_SPEED, so that the pendulum starts from rest (initial direction: +1)
//       - saturate with pendulum_clamp()
//   * Now the torque saturates (at the capture, after a push): make the PID saturation-aware. Clamp its output
//     to +/- pendulum.maxTorque() and update the integral only while the output is not saturated (anti-windup).
//
// What the library gives you (complete list: FOC_libraries_student_reference.md, Pendulum_lar section):
//   pendulum.theta, pendulum.omega       measured angle (rad, continuous) and velocity (rad/s)
//   pendulum.dt                          actual control period of this sample (s)
//   pendulum.inertia(), pendulum.gravityTorque(), pendulum.viscousFriction()     J, G, b of the model
//   pendulum.energy(theta, omega), pendulum.energyUpright(), pendulum.maxTorque()      (energy(): your Part 2 TODO)
//   pendulum.updateMode(), pendulum.mode()   Pendulum_lar::SWING_UP / Pendulum_lar::BALANCE  (updateMode(): Part 2)
//   pendulum_wrap_angle(a), pendulum_clamp(x, low, high), pendulum_sign(x), PENDULUM_LAR_PI
//   fabsf(x), sinf(x), cosf(x), sqrtf(x)     <math.h>

void designController(void)
{
    // TODO (Part 1)
}

void clearControllerState(void)
{
    // TODO (Part 1)
}

float swingUpTorque(float theta, float omega)
{
    // TODO (Part 2)
    return 0.0f;
}

float controlTorque(void)
{
    // TODO (Part 1, then Part 2)
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

    // TODO (Part 1): start the motor and the pendulum, in this order (use the defines at the top):
    //   1. The motor, with the stored alignment of Lab 1, so that the pendulum does not move:
    //        foc.begin(supply voltage, voltage limit, align voltage, stored zero angle, stored direction)
    //      It returns false if the motor or the sensor does not answer: then stop here with  while (1);
    //   2. The calibration of Lab 1: foc.setTorqueConstant(k) and foc.setBackEmfConstant(Ke).
    //      Ke only when USE_BACK_EMF_COMP is 1 (otherwise 0: no back-EMF compensation).
    //   3. The pendulum: pendulum.begin(foc, control period, velocity filter alpha, run time).
    //   4. Its model: pendulum.setParameters(rod mass, point mass, length, b, tau_c).
    //      b is VISCOUS_FRICTION with the back-EMF compensation; without it the motor adds the electrical
    //      damping TORQUE_CONSTANT * MOTOR_KE to it.
    //   5. pendulum.setCoulombCompensation(USE_COULOMB_COMP) and pendulum.setCaptureAngles(CAPTURE_ANGLE,
    //      RELEASE_ANGLE) (the thresholds of pendulum.updateMode()).
    //   6. The gains: designController().

    // given: what the sketch prints at power-up (keep it after your code: printModel() needs step 4)
    printModel();
    printState();
    printHelp();
}

void loop()
{
    // TODO (Part 1): the control loop. loop() runs thousands of times per second; never use delay() here.
    //   1. foc.update(): the FOC algorithm. It must run on EVERY call of loop(), first.
    //   2. bool new_sample = pendulum.update(): reads the keys 's', 'x', 'p', keeps the run time and the
    //      timing, and returns true once per control period (while a run is active) with a fresh
    //      pendulum.theta, pendulum.omega and pendulum.dt.
    //   3. handleCommands() (given, below): right after pendulum.update(), so that the other keys are applied.
    //   4. If a run has just started (pendulum.started()), reset your controller.
    //   5. If there is no new sample, return: there is nothing to compute until the next control period.
    //   6. Compute the torque with controlTorque(), apply it with pendulum.applyTorque() and log the sample
    //      with pendulum.log().

    handleCommands(); // given: keys '1', '2', 'u', 'h' (move it to step 3)
}
