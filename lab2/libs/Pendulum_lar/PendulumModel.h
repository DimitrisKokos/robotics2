#ifndef PENDULUM_MODEL_H
#define PENDULUM_MODEL_H

// PendulumModel (part of the Pendulum_lar library)
//
// Mechanical model of the pendulum (point mass at the end of a uniform rod, viscous + Coulomb shaft friction):
//
//     J * theta'' = tau - b * theta' - tau_c * sign(theta') - G * sin(theta)
//
//     J = (m_r/3 + m_p) * l^2          inertia about the shaft
//     G = (m_r/2 + m_p) * l * g        maximum gravity torque
//     b                                viscous friction coefficient
//     tau_c                            Coulomb friction torque
//
// theta = 0 : hanging straight down.   theta = pi : upright.
// Positive theta and positive torque are in the same direction.
//
// The linearizations (LQR, MPC, TVLQR) contain b only: the derivative of tau_c*sign(theta') is zero
// for theta' != 0. The Coulomb friction is cancelled instead by a feed-forward torque tau_c*sign(omega)
// added to the command (Pendulum_lar::setCoulombCompensation), in the same way as the back-EMF is
// compensated by FOC_lar.
//
// This file only needs <math.h>: it is used by the Arduino class Pendulum_lar (which inherits from
// PendulumModel) and it also compiles on a PC (lab_code/scripts/pendulum_sim/pendulum_sim.cpp).
// In a sketch you do not include it yourself: #include <Pendulum_lar.h> includes it.

#include <math.h>

#define PENDULUM_LAR_GRAVITY 9.80665f // m/s^2
#define PENDULUM_LAR_PI 3.14159265358979f

// Helpers

float pendulum_wrap_angle(float angle); // to [-pi, pi)
float pendulum_clamp(float x, float low, float high);
float pendulum_sign(float x); // -1, 0 or +1

// 2x2 matrix product out = a * b (out may be a or b). Used by PendulumModel::discretizeAt().
void pendulum_mat_mul(const float a[2][2], const float b[2][2], float out[2][2]);

// Model

class PendulumModel
{
public:
    PendulumModel();

    // rod_mass, point_mass in kg, length in m, viscous_friction in N*m*s/rad, coulomb_friction in N*m
    void setParameters(float rod_mass,
                       float point_mass,
                       float length,
                       float viscous_friction,
                       float coulomb_friction = 0.0f,
                       float gravity = PENDULUM_LAR_GRAVITY);

    float inertia(void) const;          // J (kg*m^2)
    float gravityTorque(void) const;    // G (N*m): maximum gravity torque
    float viscousFriction(void) const;  // b
    float coulombFriction(void) const;  // tau_c
    float naturalFrequency(void) const; // sqrt(G/J): the unstable pole at the top (rad/s)

    // true if max_torque is enough to hold the pendulum horizontal (needed to lift it directly)
    bool canLiftDirectly(float max_torque) const;

    // theta'' of the nonlinear model, (tau - b*omega - tau_c*sign(omega) - G*sin(theta)) / J
    float acceleration(float theta, float omega, float torque) const;

    // Energy relative to the hanging rest position: 0.5*J*w^2 + G*(1 - cos(theta)).
    // It is 2*G at the top. Friction removes it at the rate b*w^2 + tau_c*|w|.
    float energy(float theta, float omega) const;
    float energyUpright(void) const;

    // One Runge-Kutta 4 step of the nonlinear model (torque held constant). With tau_c > 0 the jump of
    // sign(omega) makes the step inaccurate when omega changes sign within dt: use a small dt there.
    void step(float &theta, float &omega, float torque, float dt) const;

    // Linearization at the top, state x = [phi, dphi] with phi = theta - pi:
    //   phi'' = (G/J)*phi - (b/J)*dphi + tau/J          (no Coulomb term: compensate it, see above)
    void linearizeUpright(float A[2][2], float B[2]) const;
    void discretizeUpright(float Ts, float Ad[2][2], float Bd[2]) const; // zero-order hold

    // Linearization at any angle theta (the velocity does not matter), state x = [dtheta, domega]:
    //   dtheta'' = -(G/J)*cos(theta)*dtheta - (b/J)*domega + dtau/J
    // At theta = pi this is linearizeUpright().
    void linearizeAt(float theta, float A[2][2], float B[2]) const;
    void discretizeAt(float theta, float Ts, float Ad[2][2], float Bd[2]) const; // zero-order hold

private:
    float _J;
    float _G;
    float _b;
    float _tc;
};

#endif
