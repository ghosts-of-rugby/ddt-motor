#include <iostream>

#include "ddt-motor/observer.hpp"

using namespace std::literals;  // NOLINT

int main(int argc, char const *argv[]) {
  ddt::Observer observer(15.0, -60, -40);

  for (int i = 0; i < 100; i++) {
    auto [velocity, disturbance] = observer.Update(1.0, 0.0, 5ms);
    std::cout << velocity << std::endl;
  }

  return 0;
}
