#include <iostream>

#include "ddt-motor/observer.hpp"

using namespace std::literals;  // NOLINT

int main(int argc, char const *argv[]) {
  ddt::Observer observer(7.2, -10, -20);

  for (int i = 0; i < 1000; i++) {
    auto [velocity, disturbance] = observer.Update(1.0, 0.0, 5ms);
    std::cout << disturbance << std::endl;
  }

  return 0;
}
