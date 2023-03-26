#include <joint_imitator.h>

JointImitator::JointImitator()
    : circle_thread_(), mutex_(), joint_(), rate_(500) {
  last_position_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
  circle_thread_ = std::thread(&JointImitator::circle, this);
}

void JointImitator::circle() {
  while (true) {
    update();
    int32_t circle_time = 1000000 / rate_;  // microseconds
    std::this_thread::sleep_for(std::chrono::microseconds(circle_time));
  }
}

void JointImitator::update() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (joint_.mode_ == JointMode::STOPPED) {
    return;
  }
  if (joint_.mode_ == JointMode::EFFORT) {
    // TODO: Skip, implement later.
    return;
  }
  if (joint_.mode_ == JointMode::POSITION) {
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count() -
                last_position_time;
    dt = dt / 1000000;
    calculateInPositionMode(dt);
    last_position_time =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    return;
  }
  if (joint_.mode_ == JointMode::VELOCITY) {
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count() -
                last_position_time;
    dt = dt / 1000000;
    calculateVelocity(dt);
    last_position_time =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    return;
  }
  throw std::runtime_error("Unknown joint mode.");
}

void JointImitator::calculateInPositionMode(double dt) {
  double target_position = joint_.command_.position_;
  double current_position = joint_.state_.position_;

  // PID gains
  double Kp = 1;
  double Ki = 0.01;
  double Kd = 0.01;

  // Compute error and update PID terms
  static double integral_error = 0.0;
  static double prev_error = 0.0;
  double error = target_position - current_position;
  integral_error += error * dt;
  double derivative_error = (error - prev_error) / dt;
  prev_error = error;

  // Compute control effort
  double target_velocity =
      Kp * error + Ki * integral_error + Kd * derivative_error;

  if (target_velocity > joint_.profile_velocity_) {
    target_velocity = joint_.profile_velocity_;
  }

  // Apply control effort to joint state
  joint_.command_.velocity_ = target_velocity;
  calculateVelocity(dt);
}

void JointImitator::calculateVelocity(double dt) {
  double target_velocity = joint_.command_.velocity_;
  double current_velocity = joint_.state_.velocity_;
  double acceleration = joint_.profile_acceleration_;

  double velocity;
  if (target_velocity > current_velocity) {
    // accelerating
    velocity = current_velocity + acceleration * dt;
    if (velocity > target_velocity) {
      velocity = target_velocity;
    }
  } else if (target_velocity < current_velocity) {
    // decelerating
    velocity = current_velocity - acceleration * dt;
    if (velocity < target_velocity) {
      velocity = target_velocity;
    }
  } else {
    // maintaining velocity
    velocity = target_velocity;
  }
  joint_.state_.velocity_ = velocity;
  joint_.state_.position_ += joint_.state_.velocity_ * dt;
}

void JointImitator::setMode(JointMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.mode_ = mode;
}

void JointImitator::setRate(double rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  rate_ = rate;
}

void JointImitator::setProfileVelocity(double profile_velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.profile_velocity_ = profile_velocity;
}

void JointImitator::setProfileAcceleration(double profile_acceleration) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.profile_acceleration_ = profile_acceleration;
}

void JointImitator::setPosition(double position) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.command_.position_ = position;
}

void JointImitator::setVelocity(double velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.command_.velocity_ = velocity;
}

void JointImitator::setEffort(double effort) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_.command_.effort_ = effort;
}

void JointImitator::setPID(double p, double i, double d) {
  std::lock_guard<std::mutex> lock(mutex_);
  Kp = p;
  Ki = i;
  Kd = d;
}

JointMode JointImitator::getMode() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.mode_;
}

double JointImitator::getRate() {
  std::lock_guard<std::mutex> lock(mutex_);
  return rate_;
}

double JointImitator::getProfileVelocity() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.profile_velocity_;
}

double JointImitator::getProfileAcceleration() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.profile_acceleration_;
}

double JointImitator::getPosition() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.state_.position_;
}

double JointImitator::getVelocity() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.state_.velocity_;
}

double JointImitator::getEffort() {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_.state_.effort_;
}

double JointImitator::getPGain() {
  std::lock_guard<std::mutex> lock(mutex_);
  return Kp;
}

double JointImitator::getIGain() {
  std::lock_guard<std::mutex> lock(mutex_);
  return Ki;
}

double JointImitator::getDGain() {
  std::lock_guard<std::mutex> lock(mutex_);
  return Kd;
}
