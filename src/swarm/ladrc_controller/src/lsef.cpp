#include "ladrc_controller/lsef.hpp"

namespace ladrc_controller
{

LSEF::LSEF()
  : kp_(25.0), kd_(10.0)
{
}

LSEF::LSEF(double kp, double kd)
  : kp_(kp), kd_(kd)
{
}

double LSEF::calculate(double ref_pos, double ref_vel, double ref_acc, double z1, double z2, double z3, double b0)
{
  // State error feedback (PD controller on tracking error)
  double u_fb = kp_ * (ref_pos - z1) + kd_ * (ref_vel - z2);

  // Acceleration feedforward
  double u_ff = ref_acc;

  // Combine feedback and feedforward to get desired acceleration
  double u0 = u_fb + u_ff;
  
  // Disturbance compensation and scaling
  // Model is: a_actual = b0*u + z3
  // We want: a_actual = u0 (our desired acceleration)
  // So: b0*u + z3 = u0  =>  b0*u = u0 - z3  =>  u = (u0 - z3) / b0
  double u = (u0 - z3) / b0;
  
  return u;
}

void LSEF::setGains(double kp, double kd)
{
  kp_ = kp;
  kd_ = kd;
}

}  // namespace ladrc_controller