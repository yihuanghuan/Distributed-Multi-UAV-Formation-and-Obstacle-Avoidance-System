#include "ladrc_controller/leso.hpp"

namespace ladrc_controller
{

LESO::LESO()
  : omega_o_(10.0), b0_(1.0), dt_(0.01)
{
  z_.setZero();
  updateMatrices();
}

LESO::LESO(double omega_o, double b0, double dt)
  : omega_o_(omega_o), b0_(b0), dt_(dt)
{
  z_.setZero();
  updateMatrices();
}


void LESO::update(double y, double u)
{
  // 标准离散时间 LESO (欧拉法)
  double e = y - z_(0); // 估计误差 (y - z1)

  // 存储上一时刻的状态
  double z1 = z_(0);
  double z2 = z_(1);
  double z3 = z_(2);

  // 更新状态
  z_(0) = z1 + dt_ * (z2 + beta1_ * e);          
  z_(1) = z2 + dt_ * (z3 + b0_ * u + beta2_ * e); 
  z_(2) = z3 + dt_ * (beta3_ * e);            
}


void LESO::reset()
{
  z_.setZero();
}

void LESO::setObserverBandwidth(double omega_o)
{
  omega_o_ = omega_o;
  updateMatrices();
}

void LESO::setControlGain(double b0)
{
  b0_ = b0;
}

void LESO::setSamplingTime(double dt)
{
  dt_ = dt;
}

std::vector<double> LESO::getStates() const
{
  return {z_(0), z_(1), z_(2)};
}

void LESO::updateMatrices()
{
  // Observer gain using Butterworth pole placement
  // (s + omega_o)^3 = s^3 + 3*omega_o*s^2 + 3*omega_o^2*s + omega_o^3
  beta1_ = 3.0 * omega_o_;
  beta2_ = 3.0 * omega_o_ * omega_o_;
  beta3_ = omega_o_ * omega_o_ * omega_o_;
  
}

}  // namespace ladrc_controller