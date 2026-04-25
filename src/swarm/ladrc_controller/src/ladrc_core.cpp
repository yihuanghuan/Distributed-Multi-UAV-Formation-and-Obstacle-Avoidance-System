#include "ladrc_controller/ladrc_core.hpp"
#include <algorithm>

namespace ladrc_controller
{

LADRCController::LADRCController()
  : initialized_(false)
{
}

LADRCController::LADRCController(const LADRCParams& params)
  : params_(params), initialized_(false)
{
  initialize(params);
}

void LADRCController::initialize(const LADRCParams& params)
{
  params_ = params;
  
  // Create observer and controller
  observer_ = std::make_unique<LESO>(params_.omega_o, params_.b0, params_.dt);
  controller_ = std::make_unique<LSEF>(params_.kp, params_.kd);
  
  initialized_ = true;
}

double LADRCController::update(double ref_pos, double ref_vel, double ref_acc, double measurement)
{
  if (!initialized_) {
    return 0.0;
  }
  
  // Get estimated states from observer
  auto states = observer_->getStates();
  double z1 = states[0];  // Estimated position
  double z2 = states[1];  // Estimated velocity
  double z3 = states[2];  // Estimated disturbance
  
  // Calculate control output using LSEF with feedforward
  double u = controller_->calculate(ref_pos, ref_vel, ref_acc, z1, z2, z3, params_.b0);
  
  // Saturate control output
  u = saturate(u, params_.min_output, params_.max_output);
  
  // Update observer with measurement and control input
  observer_->update(measurement, u);
  
  return u;
}

void LADRCController::reset()
{
  if (observer_) {
    observer_->reset();
  }
}

void LADRCController::setParameters(const LADRCParams& params)
{
  params_ = params;
  
  if (observer_) {
    observer_->setObserverBandwidth(params_.omega_o);
    observer_->setControlGain(params_.b0);
    observer_->setSamplingTime(params_.dt);
  }
  
  if (controller_) {
    controller_->setGains(params_.kp, params_.kd);
  }
}

void LADRCController::setObserverBandwidth(double omega_o)
{
  params_.omega_o = omega_o;
  if (observer_) {
    observer_->setObserverBandwidth(omega_o);
  }
}

void LADRCController::setControllerBandwidth(double omega_c)
{
  params_.omega_c = omega_c;
  // Update kp and kd based on bandwidth parameterization
  params_.kp = omega_c * omega_c;
  params_.kd = 2.0 * omega_c;
  if (controller_) {
    controller_->setGains(params_.kp, params_.kd);
  }
}

void LADRCController::setControlGain(double b0)
{
  params_.b0 = b0;
  if (observer_) {
    observer_->setControlGain(b0);
  }
}

double LADRCController::getEstimatedDisturbance() const
{
  if (observer_) {
    return observer_->getEstimatedDisturbance();
  }
  return 0.0;
}

std::vector<double> LADRCController::getEstimatedStates() const
{
  if (observer_) {
    return observer_->getStates();
  }
  return {0.0, 0.0, 0.0};
}

double LADRCController::saturate(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

}  // namespace ladrc_controller