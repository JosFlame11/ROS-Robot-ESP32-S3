#include <Arduino.h>

#include <ESP32Encoder.h>
#include <PID.h>

// ------- ROS2 headers ------- //
#include "micro_ros_platformio.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>

// ------ Motor and sensor Definition ------ //
// Leds for debugging
#define L1 4
#define L2 47
#define L3 48

// Sensors
#define OS1_PIN 18
#define OS2_PIN 17
#define OS3_PIN 16

// Motors
struct Motor {
  int PWM;
  int In1;
  int In2;
  int encA;
  int encB;
  int MC;

  Motor(int pwm, int in1, int in2, int enca, int encb, int mc) : PWM(pwm), In1(in1), In2(in2), encA(enca), encB(encb), MC(mc) {}
};

Motor leftMotor(2, 13, 12, 9, 3, 5);
Motor rightMotor(1, 14, 21, 10, 11, 6);

// Motor specifications
const int resolution = 8;
const int frequency = 30000;
const float CPR = 12.0;
const float gear_ratio = 35.0;

// Wheel specifications
const float wheel_radius = 33.5/1000;
const float distance_wheel = 81.5/1000;

// Encoder declaration
ESP32Encoder leftEnc;
ESP32Encoder rightEnc;

// Variables for velocity control
// velocity of the encoders
float left_vel_in_rad = 0;
float right_vel_in_rad = 0;

// velocity expected
double leftSpeed = 0;
double rightSpeed = 0;

int leftPWM;
int rightPWM;

// Variables for velocity calculations
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
double dt = 0;

// Pulses variables for velocity calculation
long prev_left_pos = 0;
long curr_left_pos = 0;
long prev_right_pos = 0;
long curr_right_pos = 0;

// ------- Micro-Ros Initialization ------ //
rcl_publisher_t left_vel_pub;
rcl_publisher_t right_vel_pub;

rcl_publisher_t DS1_state_pub;
rcl_publisher_t DS2_state_pub;
rcl_publisher_t DS3_state_pub;

rcl_subscription_t left_vel_sub;
rcl_subscription_t right_vel_sub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Timer
const unsigned int send_msg_timer = RCL_MS_TO_NS(100);

// Error handler
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Publisher mesages
std_msgs__msg__Float32 left_vel_msgout;
std_msgs__msg__Float32 right_vel_msgout;
std_msgs__msg__Int8 ds1_state_msgout;
std_msgs__msg__Int8 ds2_state_msgout;
std_msgs__msg__Int8 ds3_state_msgout;

// Subscriber msgs
std_msgs__msg__Float32 left_vel_msgin;
std_msgs__msg__Float32 right_vel_msgin;

// ------ Function declaration ------ //
//Function for converting pulses into rad/s
float calculateRadPerSec(long pulseDiff, double dt){
  float revolution = static_cast<float>(pulseDiff) / (2 * CPR * gear_ratio);
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

// ------ Callback Functions ------ //
// changes left wheel velocity
void left_vel_callback(const void * msgin){
  const std_msgs__msg__Float32 * left_vel_msgin = (const std_msgs__msg__Float32 *)msgin;
  leftSpeed = left_vel_msgin->data;
}

// changes right wheel velocity
void right_vel_callback(const void * msgin){
  const std_msgs__msg__Float32 * right_vel_msgin = (const std_msgs__msg__Float32 *)msgin;
  rightSpeed = right_vel_msgin->data;
}

// timer callback that sends data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCL_UNUSED(last_call_time);
  if (timer != NULL){
    // Publish messages
    left_vel_msgout.data = left_vel_in_rad;
    right_vel_msgout.data = right_vel_in_rad;
    ds1_state_msgout.data = 0;
    ds2_state_msgout.data = 0;
    ds3_state_msgout.data = 0;

    RCSOFTCHECK(rcl_publish(&left_vel_pub, &left_vel_msgout, NULL));
    RCSOFTCHECK(rcl_publish(&right_vel_pub, &right_vel_msgout, NULL));
    RCSOFTCHECK(rcl_publish(&DS1_state_pub, &ds1_state_msgout, NULL));
    RCSOFTCHECK(rcl_publish(&DS2_state_pub, &ds2_state_msgout, NULL));
    RCSOFTCHECK(rcl_publish(&DS3_state_pub, &ds3_state_msgout, NULL));
  }

}

// Task for controlling the motors
void motorControlTask(void *parameter){
  while (true){
   currentMillis = millis();
   if((currentMillis - previousMillis) >= 10){
    dt = static_cast<double>(currentMillis - previousMillis) / 1000.0;

    if(dt == 0){
      dt = 0.001;
    }
    curr_left_pos = leftEnc.getCount();
    curr_right_pos = rightEnc.getCount();

    long left_pos_diff = curr_left_pos - prev_left_pos;
    long right_pos_diff = curr_right_pos - prev_right_pos;

    prev_left_pos = curr_left_pos;
    prev_right_pos = curr_right_pos;

    // calculate velocity for both motors
    left_vel_in_rad = calculateRadPerSec(left_pos_diff, dt);
    right_vel_in_rad = calculateRadPerSec(right_pos_diff, dt);

    // Fill here with code to control motors

    previousMillis = currentMillis;

   }
   vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Ros2 setup
  allocator = rcl_get_default_allocator();
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_esp32s3", "", &support));

  RCSOFTCHECK(rclc_publisher_init_default(
    &left_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
    "leftWheelSpeed"));
  RCSOFTCHECK(rclc_publisher_init_default(
    &right_vel_pub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rightWheelSpeed"));

  rclc_publisher_init_default(
    &DS1_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "DS1_state");
  
  rclc_publisher_init_default(
    &DS2_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "DS2_state");

  rclc_publisher_init_default(
    &DS3_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "DS3_state");

  rclc_subscription_init_default(
    &left_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "leftSpeed");

  rclc_subscription_init_default(
    &right_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rightSpeed");


  RCSOFTCHECK(rclc_timer_init_default(
    &timer,
    &support,
    send_msg_timer,
    timer_callback
  ));

  rclc_executor_init(&executor,  &support.context, 10, &allocator);
  rclc_executor_add_subscription(&executor, &left_vel_sub, &left_vel_msgin, &left_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &right_vel_sub, &right_vel_msgin, &right_vel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  // Sensors and motors setup

  pinMode(OS1_PIN, INPUT_PULLDOWN);
  pinMode(OS2_PIN, INPUT_PULLDOWN);
  pinMode(OS3_PIN, INPUT_PULLDOWN);

  pinMode(leftMotor.PWM, OUTPUT);
  pinMode(leftMotor.In1, OUTPUT);
  pinMode(leftMotor.In2, OUTPUT);
  pinMode(rightMotor.PWM, OUTPUT);
  pinMode(rightMotor.In1, OUTPUT);
  pinMode(rightMotor.In2, OUTPUT);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  leftEnc.attachHalfQuad(leftMotor.encA, leftMotor.encB);
  rightEnc.attachHalfQuad(rightMotor.encA, rightMotor.encB);
  leftEnc.clearCount();
  rightEnc.clearCount();

  ledcSetup(leftMotor.MC, frequency, resolution);
  ledcSetup(rightMotor.MC, frequency, resolution);

  ledcAttachPin(leftMotor.PWM, leftMotor.MC);
  ledcAttachPin(rightMotor.PWM, rightMotor.MC);

  digitalWrite(L1, HIGH);

  xTaskCreatePinnedToCore(motorControlTask,
   "Motor Control Task",
    10000,
    NULL,
    1,
    NULL,
    0);
}

void loop() {

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
