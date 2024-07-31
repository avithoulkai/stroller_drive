#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Define Motor Control Pins
#define RIGHT_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 10

// Setup ROS
ros::NodeHandle nh;

// Setup Variables
float dtheta = 0, dx = 0, omega_left = 0, omega_right = 0;

// // Calibration Variables
// float right_lin_cal = 1.300;
// float left_lin_cal = 1.000;
// float right_ang_cal = 1.150;
// float left_ang_cal = 1.000;

float theta_scale = 1;
float linear_scale = 1;
float throttle_threshold = 5;

float wheel_base = 0.6604; // In meters
float wheel_radius = 0.2032;
float max_speed = 7;

float speed_scalar = 1;

// Servo objects for ESCs
Servo ESCright;
Servo ESCleft;

void controlMotor(Servo &ESC, float omega, const char* escName) {
  ESC.write(omega);
}

void controlCallback(const geometry_msgs::Twist& twist_msg) {
  // Scale dx & dtheta
  float desired_linear_velocity = twist_msg.linear.x * linear_scale;
  float desired_angular_velocity = twist_msg.angular.z * theta_scale;
  
  // Calculate omega_left & omega_right (angular velocities) and calibrate with scalers
  float V_left = desired_linear_velocity - (desired_angular_velocity * wheel_base / 2);
  float V_right = desired_linear_velocity + (desired_angular_velocity * wheel_base / 2);
  
  omega_left = V_left / wheel_radius;
  omega_right = V_right / wheel_radius;

  if (omega_left < 0.0) {
    omega_left = mapf(omega_left, -34.45, 0, 0, 85);
  } else if (omega_left > 0.0) {
    omega_left = mapf(omega_left, 0, 34.45, 95, 180);
  } else {
    omega_left = 90.0;
  }

  if (omega_right < 0.0) {
    omega_right = mapf(omega_right, -34.45, 0, 0, 85);
  } else if (omega_right > 0.0) {
    omega_right = mapf(omega_right, 0, 34.45, 95, 180);
  } else {
    omega_right = 90.0;
  }

  omega_left = constrain(omega_left, 0, 180);
  omega_right = constrain(omega_right, 0, 180);

  controlMotor(ESCright, omega_right, "right motor");
  controlMotor(ESCleft, omega_left, "Left ESC");
}

ros::Subscriber<geometry_msgs::Twist> control_sub("/cmd_vel", &controlCallback);

void setup() {
  Serial.begin(57600);

  // Attach ESCs
  ESCright.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  ESCleft.attach(LEFT_MOTOR_PIN, 1000, 2000);

  // Setup ROS
  nh.initNode();
  nh.subscribe(control_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}