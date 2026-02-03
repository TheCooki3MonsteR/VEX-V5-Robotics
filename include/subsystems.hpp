#pragma once
#include "api.h"

// Motors
extern pros::Motor IntakeMotor;
extern pros::Motor BigWheel;
extern pros::Motor SmallWheel;

// Pneumatics
extern pros::ADIDigitalOut MatchLoader;
extern pros::ADIDigitalOut Descorer;

// Optical
extern pros::Optical opticalSensor;

// Drive motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// IMU
extern pros::Imu imu;
