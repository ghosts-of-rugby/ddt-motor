#ifndef DDT_MOTOR_ANGLE_FILTER_HPP_
#define DDT_MOTOR_ANGLE_FILTER_HPP_
namespace ddt {
class AngleFilter {
 private:
  int step;
  int counter;
  double angle_offset;
  double pre_angle;

 public:
  AngleFilter(/* args */);
  double Update(double angle);
};
}  // namespace ddt
#endif  // DDT_MOTOR_ANGLE_FILTER_HPP_
