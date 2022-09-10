#ifndef DDT_MOTOR_OBSERVER_HPP_
#define DDT_MOTOR_OBSERVER_HPP_

#include <chrono>
#include <tuple>

#include "eigen3/Eigen/Dense"

namespace ddt {

class Observer {
 private:
  using second = std::chrono::duration<double>;

  double K;  // dω/dt = K * i

  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  Eigen::Matrix<double, 1, 2> C;
  Eigen::Vector2d Gain;
  Eigen::Vector2d x_est;

 public:
  explicit Observer(double K, double pole0, double pole1);
  std::tuple<double, double> Update(double current,      // current input
                                    double velocity,     // observed velocity
                                    Observer::second dt  // duration
  );
};

}  // namespace ddt

#endif  // DDT_MOTOR_OBSERVER_HPP_