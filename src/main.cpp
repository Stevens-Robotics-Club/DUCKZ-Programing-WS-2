#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <algorithm>


pros::MotorGroup left_mg({LEFT_DRIVE_MOTOR_PORTS}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({RIGHT_DRIVE_MOTOR_PORTS}, pros::MotorGearset::blue);
pros::MotorGroup conveyor_mg({INTAKE_MOTOR_PORTS, CONVEYOR_MOTOR_PORTS},
                             pros::MotorGearset::blue);

pros::MotorGroup conveyor({CONVEYOR_MOTOR_PORTS}, pros::MotorGearset::blue);

pros::MotorGroup intake({INTAKE_MOTOR_PORTS}, pros::MotorGearset::blue);

// Pneumatics
pros::adi::Pneumatics conveyor_piston(CONVEYOR_PISTON_PORTS, false);
pros::adi::Pneumatics flap_piston(FLAP_PISTON_PORT, false);
pros::adi::Pneumatics loader_piston(LOADER_PISTON_PORT, false);

// =================== LEMLIB SETUP ===================

// drivetrain object
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 12.75,
                              lemlib::Omniwheel::NEW_275, 450,
                              8); // horizontalDrift=8 is recommended by the
                                  // docs for using traction wheels

// odometry sensors
pros::IMU imu(IMU_PORT);
pros::Rotation vertical_odom_encoder(VERTICAL_ODOM_SENSOR_PORT);
pros::Rotation horizontal_odom_encoder(HORIZONTAL_ODOM_SENSOR_PORT);

// tracking wheels
// NOTE: we consider the tracking center to be the center of rotation of the
// robot, i.e. the horizontal center of the bot, vertically in line with the
// traction wheels
lemlib::TrackingWheel vertical_tracking_wheel(
    &vertical_odom_encoder, lemlib::Omniwheel::NEW_2,
    -0.15); // BEHIND center by 0.1 inches, so negative according to the docs
lemlib::TrackingWheel horizontal_tracking_wheel(
    &horizontal_odom_encoder, lemlib::Omniwheel::NEW_2,
    -2); // LEFT of center by 2.25 inch, so negative according to docs

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr,
                            &horizontal_tracking_wheel, nullptr, &imu);

// PID controllers & constants
// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(14,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       40   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(3,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       13,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

lemlib::ExpoDriveCurve *throttle_curve =
    new lemlib::ExpoDriveCurve(8, 10, 1.028);
lemlib::ExpoDriveCurve *steer_curve =
    new lemlib::ExpoDriveCurve(0.5, 10, 1.027);

// MAIN CHASSIS OBJECT
lemlib::Chassis *Chassis =
    (new lemlib::Chassis(drivetrain, lateral_controller, angular_controller,
                         sensors, throttle_curve, steer_curve));


// print motor temps to the brain screen
static void print_motor_temps() {
  pros::screen::print(
      pros::text_format_e_t::E_TEXT_MEDIUM, 2, "RF: %i   RC: %i   RB: %i",
      (int)right_mg.get_temperature(0), (int)right_mg.get_temperature(1),
      (int)right_mg.get_temperature(2));
  pros::screen::print(
      pros::text_format_e_t::E_TEXT_MEDIUM, 3, "LF: %i   LC: %i   LB: %i",
      (int)left_mg.get_temperature(0), (int)left_mg.get_temperature(1),
      (int)left_mg.get_temperature(2));
  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 4,
                      "In: %i C: %i B: %i", (int)intake.get_temperature(),
                      (int)conveyor.get_temperature(),
                      (int)pros::battery::get_capacity());
}

// task function for printing odometry data to the brain
void showBrainData() {
  while (true) {
    lemlib::Pose pose = Chassis->getPose();
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 0,
                        "X: %.4f   Y: %.4f   Theta: %.4f", pose.x, pose.y,
                        pose.theta);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Gray Bot");

    print_motor_temps();

    // TODO: Remove after driver calibration is complete!

    pros::delay(20);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Welcome to the DUCKZ Programming Workshop!");
  Chassis->calibrate();
  Chassis->setPose(0, 0, 0);
  
  pros::Task brainTelemetry(showBrainData);
}

/**
 * Runs the autonomous code.
 */
void autonomous() {
  // You can set the inital position of the robot like this
  Chassis->setPose(0, 0, 90);

  // You can call moveToPoint to make it so that robot goes to a specific point
  Chassis->moveToPoint(20, 0, 999999, {.maxSpeed = 90}, false);
  Chassis->turnToHeading(0, 999999, {}, false);
  Chassis->moveToPose(-20, -20, 0, 999999, {.forwards = false}, false);

  // Note you can also make it so that forwards is false, otherwise the robot
  // will always try and go forward
  // Chassis->moveToPoint(int x, int y, int
  // timeout, {.forwards = false, .maxSpeed = 80}, false);

  // You can call moveToPose to make the robot move to a position and turn to an
  // angle
  // Chassis->moveToPose(int x, int y, int theta, int timeout, {.forwards =
  // false, .maxSpeed = 80}, false);

  // To make the robot turn without actually moving laterally call turnToheading
  // Chassis->turnToHeading(int theta, int timeout, {}, false);

  // You can chain all of these different commands together to create a full
  // auton route! The robot will go from point to point mapping the route that
  // you've given it
}

pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Runs the operator control code
 */
void opcontrol() {

  while (true) {

    /**
    Lemlib provides these helpers to help move the robot:

    Chassis->arcade(int throttle, int turn);
    Chassis->tank(int left, int right);
    */
    Chassis->tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                  master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // DO NOT REMOVE THIS DELAY
        // Without it your program cannot read input/issue motor move commands
        pros::delay(20);
  }
}
