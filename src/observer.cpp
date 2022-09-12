#include "ddt-motor/observer.hpp"

// #include <iostream>

namespace ddt {
Observer::Observer(double K, double pole0, double pole1) : K(K) {
  A << 0, 0, 0, -K;
  B << K, 0;
  C << 1, 0;

  double k0 = pole0 + pole1;
  double k1 = pole0 * pole1 / K;
  Gain << k0, k1;

  x_est << 0, 0;
}

std::tuple<double, double> Observer::Update(double current, double velocity,
                                            Observer::second dt) {
  Eigen::Vector2d dxdt_pred = A * x_est + B * current;
  Eigen::Vector2d x_pred = x_est + dxdt_pred * dt.count();
  x_est = x_pred - Gain * (velocity - C * x_pred) * dt.count();
  return std::make_tuple(x_est(0), x_est(1));
}

}  // namespace ddt
