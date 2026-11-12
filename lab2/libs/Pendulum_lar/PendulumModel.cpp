#include "PendulumModel.h"

// STUDENTS' VERSION: the functions marked TODO are yours. Each TODO says in which lab it is needed:
//   Lab 2, Part 1 : setParameters()
//   Lab 2, Part 2 : energy()
//   Lab 3 (and later, not needed in Lab 2): acceleration(), step(), linearizeUpright(), linearizeAt(),
//                   discretizeAt(), discretizeUpright()
// Until you complete a TODO it returns a harmless placeholder (zeros), so that the rest of the code compiles
// and runs. The model is
//
//     J * theta'' = tau - b * theta' - tau_c * sign(theta') - G * sin(theta)
//
// with theta = 0 hanging down, theta = pi upright, and the members _J (J, kg*m^2), _G (G, N*m),
// _b (b, N*m*s/rad) and _tc (tau_c, N*m) set by setParameters().

// Helpers (given)

float pendulum_wrap_angle(float angle)
{
    float two_pi = 2.0f * PENDULUM_LAR_PI;
    angle = fmodf(angle + PENDULUM_LAR_PI, two_pi);
    if (angle < 0.0f)
    {
        angle += two_pi;
    }
    return angle - PENDULUM_LAR_PI;
}

float pendulum_clamp(float x, float low, float high)
{
    return (x < low) ? low : ((x > high) ? high : x);
}

float pendulum_sign(float x)
{
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

// 2x2 matrix product, used by the discretization

void pendulum_mat_mul(const float a[2][2], const float b[2][2], float out[2][2])
{
    float r[2][2];
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j];
        }
    }
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            out[i][j] = r[i][j];
        }
    }
}

// PendulumModel

PendulumModel::PendulumModel() : _J(1.0f), _G(0.0f), _b(0.0f), _tc(0.0f)
{
}

void PendulumModel::setParameters(float rod_mass,
                                  float point_mass,
                                  float length,
                                  float viscous_friction,
                                  float coulomb_friction,
                                  float gravity)
{
    // TODO (Lab 2, Part 1): the four constants of the model, from the arguments (SI units).
    //   _J : the moment of inertia about the shaft (kg*m^2). The pendulum is a uniform rod of mass rod_mass
    //        and length `length`, turning about one of its ends, plus a point mass point_mass at its end, at
    //        the distance `length` from the shaft. Add the two inertias (rod about its end: m*l^2/3; point
    //        mass at distance l: m*l^2).
    //   _G : the largest gravity torque (N*m), when the pendulum is horizontal: for each part, its weight
    //        (mass * gravity) times the horizontal distance from the shaft to its centre of mass (the rod's
    //        centre of mass is at length/2, the point mass at length). Then G*sin(theta) is the gravity
    //        torque at any angle.
    //   _b, _tc : the viscous and the Coulomb friction coefficients, as given (viscous_friction, coulomb_friction).
    // The sketch prints J, G and sqrt(G/J) at power-up: compare them with your Section 2 calculations.
    _J = (rod_mass / 3.0f + point_mass) * length * length;
    _G = (rod_mass / 2.0f + point_mass) * length * gravity;
    _b = viscous_friction;
    _tc = coulomb_friction;
}

float PendulumModel::inertia(void) const
{
    return _J;
}

float PendulumModel::gravityTorque(void) const
{
    return _G;
}

float PendulumModel::viscousFriction(void) const
{
    return _b;
}

float PendulumModel::coulombFriction(void) const
{
    return _tc;
}

float PendulumModel::naturalFrequency(void) const
{
    return sqrtf(_G / _J);
}

bool PendulumModel::canLiftDirectly(float max_torque) const
{
    return max_torque > _G;
}

float PendulumModel::acceleration(float theta, float omega, float torque) const
{
    // TODO (Lab 3 and later; not needed in Lab 2): the angular acceleration theta'' (rad/s^2) of the
    // nonlinear model, for the angle theta, the velocity omega and the motor torque `torque`.
    //   Solve the model equation at the top of this file for theta'': the motor torque minus the viscous
    //   friction torque (b*omega), minus the Coulomb friction torque (tau_c times the sign of omega:
    //   pendulum_sign()), minus the gravity torque (G*sin(theta): use sinf()), all divided by J.
    (void)theta;
    (void)omega;
    (void)torque;
    return 0.0f; // <- replace
}

float PendulumModel::energy(float theta, float omega) const
{
    // TODO (Lab 2, Part 2): the mechanical energy E (J), relative to the hanging rest position (E = 0 there).
    //   E = kinetic energy + potential energy:
    //   - kinetic: 0.5 * J * omega^2 (rotation about the shaft);
    //   - potential: the centres of mass rise as the pendulum turns. Their height above the hanging position,
    //     times weight, adds up to G*(1 - cos(theta)) (cosf()): 0 hanging, G at horizontal, 2*G upright.
    //   Check: at rest at the top E = energyUpright() = 2*G.
    return 0.5f * _J * omega * omega + _G * (1.0f - cosf(theta));
}

float PendulumModel::energyUpright(void) const
{
    return 2.0f * _G;
}

void PendulumModel::step(float &theta, float &omega, float torque, float dt) const
{
    // TODO (Lab 3 and later; not needed in Lab 2): advance the state (theta, omega) by one time step dt with
    // the classical 4th-order Runge-Kutta method, the torque held constant during the step. Needs
    // acceleration(). The state x = [theta, omega] obeys x' = f(x) = [omega, acceleration(theta, omega, torque)].
    //   k1 = f(x)
    //   k2 = f(x + dt/2 * k1)
    //   k3 = f(x + dt/2 * k2)
    //   k4 = f(x + dt * k3)
    //   x  = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    //   Each k has an angle part (a velocity) and a velocity part (an acceleration): e.g. k1 = [omega,
    //   acceleration(theta, omega, torque)], k2 = [omega + dt/2*k1w, acceleration(theta + dt/2*k1t,
    //   omega + dt/2*k1w, torque)], ... Write the new values into theta and omega (they are references).
    (void)theta;
    (void)omega;
    (void)torque;
    (void)dt;
    // <- write the step here (until then the state does not change)
}

void PendulumModel::linearizeUpright(float A[2][2], float B[2]) const
{
    // TODO (Lab 3; not needed in Lab 2): the continuous-time linear model at the top, x' = A x + B tau, with
    // the state x = [phi, phi'] and phi = theta - pi.
    //   Near the top sin(theta) = sin(pi + phi) ~ -phi, so the model becomes
    //       J * phi'' = tau - b * phi' + G * phi          (no Coulomb term: it is compensated, see the header)
    //   Row 0 of A, B: phi' = phi'         -> A[0][0], A[0][1], B[0]
    //   Row 1 of A, B: phi'' = ... / J     -> A[1][0] (coefficient of phi), A[1][1] (of phi'), B[1] (of tau)
    A[0][0] = 0.0f; // <- replace the eight values
    A[0][1] = 0.0f;
    A[1][0] = 0.0f;
    A[1][1] = 0.0f;
    B[0] = 0.0f;
    B[1] = 0.0f;
}

void PendulumModel::linearizeAt(float theta, float A[2][2], float B[2]) const
{
    // TODO (Lab 3; not needed in Lab 2): the continuous-time linear model at ANY angle theta (the velocity does
    // not matter), state x = [dtheta, domega], the small deviations from (theta, omega), x' = A x + B dtau.
    //   Differentiate the model equation: the derivative of G*sin(theta) with respect to theta is
    //   G*cos(theta) (cosf()), of b*omega with respect to omega is b. So
    //       J * dtheta'' = dtau - b * domega - G * cos(theta) * dtheta
    //   Fill A and B as in linearizeUpright(). Check: at theta = pi (cos = -1) it must give the same A and B
    //   as linearizeUpright(); at theta = 0 (hanging) A[1][0] = -G/J (gravity pulls back).
    (void)theta;
    A[0][0] = 0.0f; // <- replace the eight values
    A[0][1] = 0.0f;
    A[1][0] = 0.0f;
    A[1][1] = 0.0f;
    B[0] = 0.0f;
    B[1] = 0.0f;
}

void PendulumModel::discretizeAt(float theta, float Ts, float Ad[2][2], float Bd[2]) const
{
    // TODO (Lab 3; not needed in Lab 2): the discrete-time model of linearizeAt(theta) for the control period
    // Ts, with the torque held constant over each period (zero-order hold): x[k+1] = Ad x[k] + Bd tau[k].
    //   Ad = exp(A*Ts) and Bd = (integral from 0 to Ts of exp(A*s) ds) * B. Compute both with the series
    //       Ad = I + (A Ts) + (A Ts)^2/2! + (A Ts)^3/3! + ...
    //       Bd = Ts * [ I + (A Ts)/2! + (A Ts)^2/3! + ... ] * B
    //   Steps:
    //   1. get A and B with linearizeAt(theta, A, B); M = A * Ts (element by element);
    //   2. start with term = I (the 2x2 identity), Ad = I, Bd = Ts * B;
    //   3. for k = 1, 2, ..., 14: term = term * M / k (use pendulum_mat_mul(term, M, term), then divide every
    //      element by k), Ad += term, Bd += Ts / (k + 1) * term * B.
    //      With |A Ts| < 1 (here about 0.1) 14 terms are far more than float precision needs.
    (void)theta;
    (void)Ts;
    Ad[0][0] = 0.0f; // <- replace
    Ad[0][1] = 0.0f;
    Ad[1][0] = 0.0f;
    Ad[1][1] = 0.0f;
    Bd[0] = 0.0f;
    Bd[1] = 0.0f;
}

void PendulumModel::discretizeUpright(float Ts, float Ad[2][2], float Bd[2]) const
{
    // TODO (Lab 3; not needed in Lab 2): the discrete-time model at the top, state [phi, phi'].
    //   It is the model of discretizeAt() at theta = pi (PENDULUM_LAR_PI): one line.
    (void)Ts;
    Ad[0][0] = 0.0f; // <- replace
    Ad[0][1] = 0.0f;
    Ad[1][0] = 0.0f;
    Ad[1][1] = 0.0f;
    Bd[0] = 0.0f;
    Bd[1] = 0.0f;
}
