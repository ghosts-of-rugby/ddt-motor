#include <iostream>

#include "ddt-motor/observer.hpp"

int main(int argc, char const *argv[]) {
  ddt::Observer observer(0.05, 7.2, -10, -20);

  for (int i = 0; i < 100; i++) {
    auto [velocity, disturbance] = observer.Update(1.0, 0.0);
    std::cout << disturbance << std::endl;
  }

  return 0;
}
