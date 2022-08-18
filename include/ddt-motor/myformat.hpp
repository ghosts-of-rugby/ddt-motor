#ifndef DDT_MOTOR_MYFORMAT_HPP_
#define DDT_MOTOR_MYFORMAT_HPP_
// copy from https://gist.github.com/toffaletti/3760605

#include <iostream>
#include <sstream>

namespace ddt {
struct formatter {
  std::string format;

  formatter(const std::string &format_) : format(format_) {}
  formatter(const formatter &other) = delete;
  formatter(formatter &&other) : format(std::move(other.format)) {}

  template <typename Iter, typename Arg, typename... Args>
  void vformat(std::stringstream &ss, Iter &i, Arg arg) {
    while (i < std::end(format)) {
      switch (*i) {
        case '{':
          ++i;
          ++i;  // eat the '}'
          ss << arg;
          break;
        default:
          ss << *i;
          ++i;
          break;
      }
    }
  }

  template <typename Iter, typename Arg, typename... Args>
  void vformat(std::stringstream &ss, Iter &i, Arg arg, Args... args) {
    while (i < std::end(format)) {
      switch (*i) {
        case '{':
          ++i;
          ++i;  // eat the '}'
          ss << arg;
          vformat(ss, i, args...);
          return;
        default:
          ss << *i;
          ++i;
          break;
      }
    }
  }

  template <typename... Args>
  std::string operator()(Args... args) {
    std::stringstream ss;
    auto i = std::begin(format);
    vformat(ss, i, args...);
    return ss.str();
  }
};

formatter operator"" _fmt(const char *p, size_t n) {
  return std::move(formatter(std::string(p, n)));
}
}  // namespace ddt

#endif