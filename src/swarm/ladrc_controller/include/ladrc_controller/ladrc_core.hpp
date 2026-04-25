#ifndef LADRC_CONTROLLER__LADRC_CORE_HPP_
#define LADRC_CONTROLLER__LADRC_CORE_HPP_

#include <vector>
#include <memory>
#include "ladrc_controller/leso.hpp"
#include "ladrc_controller/lsef.hpp"

namespace ladrc_controller
{

struct LADRCParams
{
  // LESO parameters
  double omega_o;  // Observer bandwidth
  
  // LSEF parameters
  double omega_c;  // Controller bandwidth
  double kp;       // Proportional gain
  double kd;       // Derivative gain
  
  // System parameters
  double b0;       // Control gain (estimate of system gain)
  double dt;       // Sampling time
  
  // Limits
  double max_output;
  double min_output;
  
  LADRCParams()
    : omega_o(10.0), omega_c(5.0), kp(25.0), kd(10.0),
      b0(1.0), dt(0.01), max_output(10.0), min_output(-10.0) {}
};

class LADRCController
{
public:
  LADRCController();
  explicit LADRCController(const LADRCParams& params);
  ~LADRCController() = default;

  // Initialize controller
  void initialize(const LADRCParams& params);
  
  // Update controller (main control loop)
  double update(double ref_pos, double ref_vel, double ref_acc, double measurement);
    
  // Reset controller state
  void reset();
  
  // Setters
  void setParameters(const LADRCParams& params);
  void setObserverBandwidth(double omega_o);
  void setControllerBandwidth(double omega_c);
  void setControlGain(double b0);
  
  // Getters
  LADRCParams getParameters() const { return params_; }
  double getEstimatedDisturbance() const;
  std::vector<double> getEstimatedStates() const;

private:
  LADRCParams params_;
  std::unique_ptr<LESO> observer_;
  std::unique_ptr<LSEF> controller_;
  bool initialized_;
  
  double saturate(double value, double min_val, double max_val);
};

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__LADRC_CORE_HPP_