#ifndef LADRC_CONTROLLER__LSEF_HPP_
#define LADRC_CONTROLLER__LSEF_HPP_

namespace ladrc_controller
{

/**
 * @brief Linear State Error Feedback (LSEF) control law
 * 
 * u0 = kp * (r - z1) - kd * z2
 * u = (u0 - z3) / b0
 */
class LSEF
{
public:
  LSEF();
  LSEF(double kp, double kd);
  ~LSEF() = default;

  // Calculate control output
  double calculate(double ref_pos, double ref_vel, double ref_acc, double z1, double z2, double z3, double b0);
  
  // Setters
  void setGains(double kp, double kd);
  void setProportionalGain(double kp) { kp_ = kp; }
  void setDerivativeGain(double kd) { kd_ = kd; }
  
  // Getters
  double getProportionalGain() const { return kp_; }
  double getDerivativeGain() const { return kd_; }

private:
  double kp_;  // Proportional gain
  double kd_;  // Derivative gain
};

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__LSEF_HPP_