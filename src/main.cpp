#include "main.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include "subsystems.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <cstddef>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motor groups
pros::MotorGroup leftMotors({-8, -9, -10});
pros::MotorGroup rightMotors({1, 2, 3});

// Subsystem motors
pros::Motor IntakeMotor(11, pros::E_MOTOR_GEARSET_06);
pros::Motor BigWheel(-20, pros::E_MOTOR_GEARSET_18);
pros::Motor SmallWheel(12, pros::E_MOTOR_GEARSET_18);

// Pneumatics
pros::ADIDigitalOut MatchLoader('C');
pros::ADIDigitalOut Descorer('A');

// Sensors
pros::Optical opticalSensor(7);
pros::Imu imu(13);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(15, true);
// vertical tracking wheel
lemlib::TrackingWheel vertical(& verticalEnc, lemlib::Omniwheel::NEW_2, 0.8);

//CHANGE DISTANCES BASED ON YOUR ROBOT

// drivetrain settings
lemlib::Drivetrain drivetrain(& leftMotors, // left motor group
                              & rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325,
                              333,
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings linearController(7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             25, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(& vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            & imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, & throttleCurve, & steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    verticalEnc.reset_position(); 
    
	leftMotors.set_gearing(pros::E_MOTOR_GEARSET_06);
	rightMotors.set_gearing(pros::E_MOTOR_GEARSET_06);
	leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	IntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BigWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	SmallWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	IntakeMotor.set_current_limit(2500);
	BigWheel.set_current_limit(2500);
	SmallWheel.set_current_limit(2500);

	IntakeMotor.set_voltage_limit(12000);
	BigWheel.set_voltage_limit(12000);
	SmallWheel.set_voltage_limit(12000);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "n/IMU head: %f", imu.get_heading());
            pros::lcd::print(4, "IMU rot: %f", imu.get_rotation());
            pros::lcd::print(5, "IMU cal: %d", imu.is_calibrating());
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }	
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void R7W() {
    chassis.setPose(15, -48, 0);
    IntakeMotor.move(-127);
    Descorer.set_value(true);
    chassis.moveToPoint(22, -15, 500, {}, false);
    MatchLoader.set_value(true);

    //align with the long goal
    chassis.moveToPoint(50, -48, 2000);
    chassis.turnToHeading(180, 1000);

    //go into the match loader
    chassis.moveToPoint(50, -70, 1000);
    chassis.moveToPoint(50, 0, 200, {.forwards=false},false);
    chassis.moveToPoint(50, -70, 1000,{},false);

    //go to the long goal
    chassis.moveToPoint(50,-24,2000,{.forwards=false},false);
    IntakeMotor.move(-127);
    BigWheel.move(-127);
    SmallWheel.move(-127);

    pros::delay(2000);

    chassis.moveToPoint(60, -50, 2000);
    Descorer.set_value(false);
    chassis.moveToPoint(60, -7, 2000,{.forwards=false}, false);
}

void R4W(){
    chassis.setPose(15, -48, 0);
    IntakeMotor.move(-127);
    Descorer.set_value(true);
    chassis.moveToPoint(22, -15, 500, {}, false);
    MatchLoader.set_value(true);

    //align with the long goal
    chassis.moveToPoint(50, -48, 2000);
    chassis.turnToHeading(180, 1000);

    //go to the long goal
    chassis.moveToPoint(50,-24,2000,{.forwards=false},false);
    IntakeMotor.move(-127);
    BigWheel.move(-127);
    SmallWheel.move(-127);

    pros::delay(2000);

    chassis.moveToPoint(60, -50, 2000);
    Descorer.set_value(false);
    chassis.moveToPoint(60, -7, 2000,{.forwards=false}, false);
}

void autonomous() {
    R4W();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get left y and right y positions
        // get joystick values
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(forward, turn);

		// Intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		IntakeMotor.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		IntakeMotor.move(-127);
		}
		else {
		IntakeMotor.move(0);
		}

		// Big Wheel and Small Wheel and color sensor
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		BigWheel.move(127);
		SmallWheel.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		BigWheel.move(-127);
		SmallWheel.move(-127);
		}
		else {
		BigWheel.move(0);
		SmallWheel.move(0);
		}

		// Match Loader
		static bool match_last_up = false;
		static bool match_last_down = false;

		bool match_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		bool match_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
	
		if (match_up && !match_last_up) {
			MatchLoader.set_value(true);
		}
	
		if (match_down && !match_last_down) {
			MatchLoader.set_value(false);
		}

		match_last_up = match_up;
		match_last_down = match_down;

		// Descorer
		static bool desc_last_down = false;
		static bool desc_last_up = false;

		bool desc_down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		bool desc_up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

		if (desc_down && !desc_last_down) {
                Descorer.set_value(true);
		}
		if (desc_up && !desc_last_up) {
			Descorer.set_value(false);
		}

		desc_last_down = desc_down;
		desc_last_up = desc_up;

		// delay to save resources
        pros::delay(25);
    }
}