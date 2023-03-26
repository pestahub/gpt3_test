#pragma once
#include <math.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

class JointState {
 public:
  JointState() : position_(0), velocity_(0), effort_(0) {}
  ~JointState(){};

  double position_;
  double velocity_;
  double effort_;
};

class JointCommand {
 public:
  JointCommand() : position_(0), velocity_(0), effort_(0) {}
  ~JointCommand(){};

  double position_;
  double velocity_;
  double effort_;
};

enum class JointMode { POSITION, VELOCITY, EFFORT, STOPPED };

class Joint {
 public:
  Joint() : state_(), command_() {}
  ~Joint(){};

  JointState state_;
  JointCommand command_;
  double profile_velocity_ = 0.1;
  double profile_acceleration_ = 0.1;
  JointMode mode_ = JointMode::STOPPED;
};

class JointImitator {
 public:
  JointImitator();
  ~JointImitator() { circle_thread_.detach(); };

  void setMode(JointMode mode);
  void setRate(double rate);
  void setProfileVelocity(double profile_velocity);
  void setProfileAcceleration(double profile_acceleration);
  void setPosition(double position);
  void setVelocity(double velocity);
  void setEffort(double effort);
  void setPID(double p, double i, double d);

  JointMode getMode();
  double getRate();
  double getProfileVelocity();
  double getProfileAcceleration();
  double getPosition();
  double getVelocity();
  double getEffort();
  double getPGain();
  double getIGain();
  double getDGain();

 private:
  void circle();
  void update();
  void calculateInPositionMode(double dt);
  void calculateVelocity(double dt);
  double last_position_time = 0.0;
  std::thread circle_thread_;
  std::mutex mutex_;
  Joint joint_;
  double rate_;

  double Kp = 1.0;
  double Ki = 0.0;
  double Kd = 0.0;
};