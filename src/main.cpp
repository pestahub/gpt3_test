#include <joint_imitator.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int, char**) {
  JointImitator joint_imitator;

  double setpoint = 1;
  joint_imitator.setMode(JointMode::POSITION);
  joint_imitator.setRate(100);
  joint_imitator.setProfileVelocity(0.5);
  joint_imitator.setProfileAcceleration(0.7);
  joint_imitator.setPosition(setpoint);

  joint_imitator.setPID(10, 1, 0.1);

  std::vector<double> x{0.0};
  std::vector<double> y{0.0};
  plt::plot(x, y);
  plt::xlim(0, 10);
  plt::ylim(-1, 1);

  auto start = std::chrono::high_resolution_clock::now();

  plt::title("position");
  plt::xlim(0, 10);
  plt::ylim(0.0, 1.1);
  plt::axis("equal");

  plt::Plot plot("position");

  std::vector<double> xt{0.0};
  std::vector<double> yt{0.0};

  while (true) {
    // Get the current position
    double position = joint_imitator.getPosition();
    auto now = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
    double seconds = duration.count() / 1000.0;
    xt.push_back(seconds);
    yt.push_back(position);
    // Update the plot data

    plot.update(xt, yt); 

    plt::pause(0.1);
    // Wait for 100ms
  }

  return 0;
}
