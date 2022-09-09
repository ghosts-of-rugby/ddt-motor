#include "ddt-motor/observer.hpp"

#include <iostream>

namespace ddt {
Observer::Observer(double period, double K, double pole0, double pole1)
    : period(period), K(K) {
  A << 0, 0, 0, -K;
  B << K, 0;
  C << 1, 0;

  double k0 = pole0 + pole1;
  double k1 = pole0 * pole1 / K;
  Gain << k0, k1;

  x_est << 0, 0;
}

std::tuple<double, double> Observer::Update(double current, double velocity) {
  Eigen::Vector2d dxdt_pred =
      A * x_est + B * current - Gain * (velocity - C * x_est);
  // std::cout << (velocity - C * x_est) << std::endl;

  x_est += dxdt_pred * period;

  return std::make_tuple(x_est(0), x_est(1));
}

}  // namespace ddt
