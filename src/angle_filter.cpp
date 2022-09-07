#include "ddt-motor/angle_filter.hpp"

#include <cmath>
namespace ddt {

AngleFilter::AngleFilter() : step(0), counter(0) {}

double AngleFilter::Update(double angle) {
  if (step == 0) {
    angle_offset = angle;
    pre_angle = angle;
    step++;
    return 0.0;
  }

  if (pre_angle > M_PI_2 * 3 && angle < M_PI_2) {
    counter++;
  } else if (angle > M_PI_2 * 3 && pre_angle < M_PI_2) {
    counter--;
  }
  pre_angle = angle;
  step++;

  return angle + counter * 2 * M_PI - angle_offset;
}

}  // namespace ddt
