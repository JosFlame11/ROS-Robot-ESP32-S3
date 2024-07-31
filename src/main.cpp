#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID.h>
#include <ESP32Servo.h>

///////// ROS headers ///////////
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

// Struct for Motor pins
struct Motor {
  int PWM;
  int In1;
  int In2;
  int encA;
  int encB;
  int MC;

  Motor(int pwm, int in1, int in2, int enca, int encb, int mc) : PWM(pwm), In1(in1), In2(in2), encA(enca), encB(encb), MC(mc) {}
};

// Struct for PID gains
struct PIDGains {
  double Kp;
  double Ki;
  double Kd;
  PIDGains(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}
};

// Declaration of Motors
Motor leftMotor(2, 13, 12, 9, 3, 0);
Motor rightMotor(1, 14, 21, 10, 11, 1);

// Motor specifications
const int resolution = 8;
const int freq = 3000;
const double CPR = 12.0;
const double gearRatio = 35.0;

ESP32Encoder leftEnc;
ESP32Encoder rightEnc;

// PID gains for both motors
PIDGains leftGains(2.683, 134.17, 0.0);
PIDGains rightGains(2.683, 134.17, 0.0);

// Variables for angular velocity
double left_vel_in_rad = 0;
double right_vel_in_rad = 0;
double leftSpeed = 5;
double rightSpeed = 5;
double leftPWM;
double rightPWM;

// PID objects for each motor
PID leftPID(&left_vel_in_rad, &leftPWM, &leftSpeed, leftGains.Kp, leftGains.Ki, leftGains.Kd);
PID rightPID(&right_vel_in_rad, &rightPWM, &rightSpeed, rightGains.Kp, rightGains.Ki, rightGains.Kd);

// Time variables for calculation
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
double dt = 0;

// Pulses variables for velocity calculation
long previousLeftPos = 0;
long currentLeftPos = 0;
long previousRightPos = 0;
long currentRightPos = 0;

// LED for debugging
const uint8_t l1 = 4;
const uint8_t l2 = 47;
const uint8_t l3 = 48;

// Sensors
const int DS_pin = 16;
const int OS1_pin = 18;
const int OS2_pin = 17;
int lastOS1state = 0;
int lastOS2state = 0;
float distanceValue = 0.0;

// Servos
const int servoShoulder_pin = 15;
const int servoElbow_pin = 7;
const int servoWrist_pin = 6;
const int servoGripper_pin = 5;

Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;

int pos[3] = {90, 90, 90}; // Change to static pose angles 
String gripperState[2] = {"OPEN", "CLOSE"}; // States to keep track of the gripper
String currentGripperState;

// Robot Constants
const double R = 33.5 / 1000;  // Wheel Radius
const double L = 81.5 / 1000.0; // Distance from center to wheel

// ROS node: Subscruber and publishers
ros::NodeHandle nh;
double Vx = 0.0;
double Vz = 0.0;

// Function for calculating rad/s
double calculateRadPerSec(long pulseDiff, double dt) {
  double revolution = static_cast<double>(pulseDiff) / (4 * CPR * gearRatio);
  return (revolution * (2 * PI) / dt);
}

// Function for setting the motor PWM
void setSpeed(double Lvel, double Rvel) {
  Lvel = constrain(Lvel, -255, 255);
  Rvel = constrain(Rvel, -255, 255);

  if (Lvel > 0) {
    digitalWrite(leftMotor.In1, HIGH);
    digitalWrite(leftMotor.In2, LOW);
  } else {
    digitalWrite(leftMotor.In1, LOW);
    digitalWrite(leftMotor.In2, HIGH);
  }

  if (Rvel > 0) {
    digitalWrite(rightMotor.In1, HIGH);
    digitalWrite(rightMotor.In2, LOW);
  } else {
    digitalWrite(rightMotor.In1, LOW);
    digitalWrite(rightMotor.In2, HIGH);
  }

  ledcWrite(leftMotor.MC, fabs(Lvel));
  ledcWrite(rightMotor.MC, fabs(Rvel));
}

// Function for reading the Distance Sensor
float distance(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum = sum + analogRead(DS_pin);
  }
  float lct = sum / n;
  float distance_cm = 17170.23 * pow(lct, -1.05); //solution of 5cm to 2380,and 30cm to 430
  return (distance_cm);
}

// Call Back Function to read the velocities
void velocityCallBack(const geometry_msgs::Twist& vel_msg) {
  Vx = vel_msg.linear.x;
  Vz = vel_msg.angular.z;
  leftSpeed = (Vx + L * Vz) / R;
  rightSpeed = (Vx - L * Vz) / R;
}

void servoCallback(const std_msgs::String& servo_msg){
  char cmd = servo_msg.data[0];
  switch (cmd){
    case 'u': // + shoulder
    pos[0] = constrain(pos[0] + 5, 0, 180); 
    shoulder.write(pos[0]);
    break;
    case 'j': // - shoulder
    pos[0] = constrain(pos[0] - 5, 0, 180); 
    shoulder.write(pos[0]);
    break;
    case 'i': // + elbow
    pos[1] = constrain(pos[1] + 5, 0, 180);
    elbow.write(pos[1]);
    break;
    case 'k': // - elbow
    pos[1] = constrain(pos[1] - 5, 0, 180);
    elbow.write(pos[1]);
    break;
    case 'o': // + wrist
    pos[2] = constrain(pos[2] + 5, 0, 180);
    wrist.write(pos[2]);
    break;
    case 'l': // - wrist
    pos[2] = constrain(pos[2] - 5, 0, 180);
    wrist.write(pos[2]);
    break;

    case 'n': // open gripper
    if (currentGripperState != gripperState[1]){
      gripper.writeMicroseconds(0);
      currentGripperState = gripperState[0];
    }
    break;
    case 'm': // close gripper
    if (currentGripperState != gripperState[0]){
      gripper.writeMicroseconds(1500);
      currentGripperState = gripperState[1];
    }
    break;
  }
}

// ROS SUBSCRIBERS
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velocityCallBack);
ros::Subscriber<std_msgs::String> servo_cmd("servo_cmd", servoCallback);

// ROS PUBLISHERS
std_msgs::Float32 left_rad_speed;
ros::Publisher leftSpeed_pub("leftSpeed", &left_rad_speed);
std_msgs::Float32 right_rad_speed;
ros::Publisher rightSpeed_pub("rightSpeed", &right_rad_speed);
sensor_msgs::Range distance_msg;
ros::Publisher DS_pub("Distance_Sensor", &distance_msg);

// Task to publish sensor data on core 0
void publishDataTask(void *parameter) {
  while (true) {
    // Publish sensor data
    left_rad_speed.data = left_vel_in_rad;
    right_rad_speed.data = right_vel_in_rad;
    leftSpeed_pub.publish(&left_rad_speed);
    rightSpeed_pub.publish(&right_rad_speed);

    // Calculate distance
    distanceValue = distance(20);
    distance_msg.range = distanceValue;
    distance_msg.header.stamp = nh.now();
    DS_pub.publish(&distance_msg);

    // Spin ROS
    nh.spinOnce();

    // Delay to control the publishing rate
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task to control motors on core 1
void motorControlTask(void *parameter) {
  while (true) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {
      dt = static_cast<double>(currentMillis - previousMillis) / 1000.0;
      if (dt == 0) {
        dt = 0.001; // Set a minimum dt to avoid division by zero
      }

      currentLeftPos = leftEnc.getCount();
      currentRightPos = rightEnc.getCount();

      long leftPulsesDiff = currentLeftPos - previousLeftPos;
      long rightPulsesDiff = currentRightPos - previousRightPos;

      previousLeftPos = currentLeftPos;
      previousRightPos = currentRightPos;

      // Calculate velocity of each motor
      left_vel_in_rad = calculateRadPerSec(leftPulsesDiff, dt);
      right_vel_in_rad = calculateRadPerSec(rightPulsesDiff, dt);

      leftPID.calculate();
      rightPID.calculate();

      setSpeed(leftPWM, rightPWM);
      previousMillis = currentMillis;
    }

    // Delay to control the task execution rate
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Serial.begin(115200);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(leftSpeed_pub);
  nh.advertise(rightSpeed_pub);
  nh.advertise(DS_pub);

  distance_msg.radiation_type = sensor_msgs::Range::INFRARED;
  distance_msg.header.frame_id = "/ir_ranger";
  distance_msg.field_of_view = 0.01;
  distance_msg.min_range = 0.05;  // For GP2D120XJ00F only. Adjust for other IR rangers
  distance_msg.max_range = 0.3;   // For GP2D120XJ00F only. Adjust for other IR rangers

  pinMode(leftMotor.PWM, OUTPUT);
  pinMode(leftMotor.In1, OUTPUT);
  pinMode(leftMotor.In2, OUTPUT);
  pinMode(rightMotor.PWM, OUTPUT);
  pinMode(rightMotor.In1, OUTPUT);
  pinMode(rightMotor.In2, OUTPUT);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(l3, OUTPUT);

  pinMode(OS1_pin, INPUT_PULLDOWN);
  pinMode(OS2_pin, INPUT_PULLDOWN);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  leftEnc.attachFullQuad(leftMotor.encA, leftMotor.encB);
  leftEnc.clearCount();
  rightEnc.attachFullQuad(rightMotor.encA, rightMotor.encB);
  rightEnc.clearCount();

  shoulder.attach(servoShoulder_pin);
  elbow.attach(servoElbow_pin);
  wrist.attach(servoWrist_pin);
  gripper.attach(servoGripper_pin);

  ledcSetup(leftMotor.MC, freq, resolution);
  ledcSetup(rightMotor.MC, freq, resolution);

  ledcAttachPin(leftMotor.PWM, leftMotor.MC);
  ledcAttachPin(rightMotor.PWM, rightMotor.MC);

  digitalWrite(l1, HIGH);
  shoulder.write(90);
  elbow.write(90);
  wrist.write(90);
  gripper.writeMicroseconds(0);
  currentGripperState = gripperState[0];

  // Create tasks and assign them to specific cores
  xTaskCreatePinnedToCore(publishDataTask, "Publish Data Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motor Control Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // Empty loop since tasks handle everything
}
