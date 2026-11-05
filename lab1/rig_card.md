| Item | Symbol / sketch setting | Value | Unit |
|---|---|---:|---|
| Pole pairs | `MOTOR_POLE_PAIRS` | 7 | — |
| Stored zero electrical angle | `STORED_ZERO_ELECTRIC_ANGLE` | 5.9 | rad |
| Stored sensor direction | `STORED_SENSOR_DIRECTION` | CCW | — |
| Torque constant | `TORQUE_CONSTANT`, $k$ | 0.009429 | N·m/V |
| Back-EMF constant, low-speed estimate | `MOTOR_KE`, $K_e$ | 0.02689 | V·s/rad |
| Rod mass | `ROD_MASS`, $m_r$ | 0.02 | kg |
| Point mass | `POINT_MASS`, $m_p$ | 0.05 | kg |
| Length, axis to point-mass centre | `PENDULUM_LENGTH`, $l$ | 0.09  | m |
| Inertia | $J$ | 4.590e-04 | kg·m² |
| Gravity-torque coefficient | $G$ | 0.05297 | N·m |
| Natural frequency | $\sqrt{G/J}$ | 10.74 | rad/s |
| Predicted period | $2\pi\sqrt{J/G}$ | 0.585 | s |
| Mechanical viscous friction | `VISCOUS_FRICTION`, $b$ | 1.6850e-05 | N·m·s/rad |
| Mechanical Coulomb friction | `COULOMB_FRICTION`, $\tau_c$ | 4.9812e-04 | N·m |
| Voltage limit planned for controller labs | `MOTOR_VOLTAGE_LIMIT` | **3 (suggested)** | V |
| Estimated maximum torque at 3 V | $\tau_{max}=3k$ | 0.02829 | N·m |
| Torque-to-gravity ratio | $\tau_{max}/G$ | 0.53 | — |

Notes:
- $k$: only one torque run (`torque2.csv`); the fitted slope came out negative (−0.009429 N·m/V, R² = 0.998), i.e. a load-cell / `VOLTAGE_SIGN` sign issue. The magnitude of the fit with offset is recorded. Through-origin fit: $|k_0|$ = 0.00872 N·m/V.
- $K_e$: linear low-speed fit gives 0.0266 (+) and 0.0294 (−) V·s/rad, about 10 % directional difference.
- $R_{est}=3K_e/(2k)\approx 4.3\ \Omega$ (consistency check only).
- Friction from `free1.csv`, viscous + Coulomb fit (RMS residual 0.83°). Run `free11.csv` did not fit (RMS 22.9°), so it is not used.
- Ring-down check: $G_{fit}=\omega_0^2J$ = 0.0497 N·m vs 0.0530 N·m from the masses (≈6 % lower); measured period ≈ 0.61 s vs predicted 0.585 s.
- $\tau_{max}/G<1$: at 3 V the motor cannot hold the pendulum horizontal; reaching upright needs swing-up.
