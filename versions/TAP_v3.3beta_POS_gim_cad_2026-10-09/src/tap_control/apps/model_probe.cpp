#include <cstdlib>
#include <iomanip>
#include <iostream>
#include "robot_control/WholeBodyModel.hpp"

int main(int argc, char** argv) {
  if (argc != 5) return 2;
  robot_control::WholeBodyModel model;
  const auto r = model.compute(std::strtof(argv[1], nullptr),
      std::strtof(argv[2], nullptr), std::strtof(argv[3], nullptr),
      std::strtof(argv[4], nullptr));
  std::cout << std::setprecision(10) << r.com_offset_x_m << ' '
            << r.com_height_m << ' ' << r.pitch_inertia_at_axle_kg_m2 << ' '
            << r.axle_height_difference_m << '\n';
}
