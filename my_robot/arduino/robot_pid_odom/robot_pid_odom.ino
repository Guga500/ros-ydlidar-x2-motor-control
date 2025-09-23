#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

// ===================== MOTOR PINS IBT-2 =====================
#define MOTOR_L_PWM 5   // ขา PWM
#define MOTOR_L_DIR 6  // ขา Direction
#define MOTOR_R_PWM 7
#define MOTOR_R_DIR 8

// ===================== ENCODER PINS =====================
#define ENCODER_L_A 2
#define ENCODER_L_B 9
#define ENCODER_R_A 3
#define ENCODER_R_B 10

volatile long encoderL_count = 0;
volatile long encoderR_count = 0;

// ===================== WHEEL PARAMETERS =====================
const float wheel_radius = 0.03; // meters
const float wheel_base = 0.08;   // meters
const int pulses_per_rev = 400;
const int  quad_multiplier  = 2;
const float ticks_per_rev   = pulses_per_rev * quad_multiplier;

// ===================== PID =====================
float kp = 1, ki = 1, kd = 0.1;
float integralL = 0, integralR = 0;
float lastErrorL = 0, lastErrorR = 0;

// ===================== VARIABLES =====================
float wheelSpeedL_cmd = 0, wheelSpeedR_cmd = 0;
float wheelSpeedL_actual = 0, wheelSpeedR_actual = 0;
unsigned long lastTime = 0;
const float dt = 0.05;

// ===================== ODOMETRY =====================
float x = 0, y = 0, theta = 0;

// ===================== ROS =====================
// [CHANGED] เพิ่มขนาดบัฟเฟอร์ rosserial เป็น 1024/1024
ros::NodeHandle_<ArduinoHardware, 5, 5, 1024, 1024> nh;

// ===================== Kill Switch =====================
bool motorsEnabled = false;

// ===================== cmd_vel =====================
geometry_msgs::Twist cmd_vel_msg;
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", [](const geometry_msgs::Twist& msg) {
  motorsEnabled = true; // เปิดมอเตอร์เมื่อ cmd_vel มา
  wheelSpeedL_cmd = msg.linear.x - msg.angular.z * wheel_base / 2;
  wheelSpeedR_cmd = msg.linear.x + msg.angular.z * wheel_base / 2;
});

// ===================== Odometry =====================
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("odom", &odom_msg);

// ===================== INTERRUPTS =====================
void encoderL_ISR() {
   if( digitalRead(ENCODER_L_A) == 0 ) {
    if ( digitalRead(ENCODER_L_B) == 0 ) {
      // A fell, B is low
      encoderL_count++; // moving reverse
    } else {
      // A rose, B is low
      encoderL_count--; // moving forward
    }
  }
}
void encoderR_ISR() {
  if( digitalRead(ENCODER_R_A) == 0 ) {
    if ( digitalRead(ENCODER_R_B) == 0 ) {
      // A fell, B is low
      encoderR_count++; // moving reverse
    } else {
      // A rose, B is low
      encoderR_count--; // moving forward
    }
  }
}

// ===================== PID =====================
int PID(float setpoint, float actual, float &integral, float &lastError) {
  float error = setpoint - actual;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;
  float output = kp * error + ki * integral + kd * derivative;
  if (output > 255) output = 255;
  if (output < -255) output = -255;
  return (int)output;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderR_ISR, CHANGE);

  // [ADDED] กำหนดบอดเรต rosserial = 57600 ให้ตรงกับฝั่ง PC + หน่วงกันชน handshake แรก
  nh.getHardware()->setBaud(57600);
  delay(2000);

  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_odom);

  lastTime = millis();
}

// ===================== LOOP =====================
void loop() {

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= dt * 1000) {
    lastTime = currentTime;

    // ===================== ENCODER =====================
    static long lastEncL = 0, lastEncR = 0;
    long deltaL = encoderL_count - lastEncL;
    long deltaR = encoderR_count - lastEncR;
    lastEncL = encoderL_count;
    lastEncR = encoderR_count;

    wheelSpeedL_actual = (1 * PI * wheel_radius * deltaL) / (pulses_per_rev * dt);
    wheelSpeedR_actual = (1 * PI * wheel_radius * deltaR) / (pulses_per_rev * dt);

    // ===================== PID =====================
    int pwmL = PID(wheelSpeedL_cmd, wheelSpeedL_actual, integralL, lastErrorL);
    int pwmR = PID(wheelSpeedR_cmd, wheelSpeedR_actual, integralR, lastErrorR);
    pwmL = 255;
    pwmR = 255;

    // ===================== MOTOR CONTROL IBT-2 =====================
    motorsEnabled = 1;
    if (motorsEnabled) 
    {
      if (wheelSpeedL_cmd < 0)
      {
        // Motor L
        analogWrite(MOTOR_L_PWM, min(abs(pwmL), 255));
        analogWrite(MOTOR_L_DIR, 0);
      }
      else if (wheelSpeedL_cmd > 0)
      {
        // Motor L
        analogWrite(MOTOR_L_PWM, 0);
        analogWrite(MOTOR_L_DIR, min(abs(pwmL), 255));
      }
      else
      {
        analogWrite(MOTOR_L_PWM, 0);
        analogWrite(MOTOR_L_DIR, 0);
      }

      if (wheelSpeedR_cmd > 0)
      {
        // Motor R
        analogWrite(MOTOR_R_PWM, 0);
        analogWrite(MOTOR_R_DIR, min(abs(pwmR), 255));
      }
      else if (wheelSpeedR_cmd < 0)
      {
        // Motor R
        analogWrite(MOTOR_R_PWM, min(abs(pwmR), 255));
        analogWrite(MOTOR_R_DIR, 0);
      }
      else
      {
        analogWrite(MOTOR_R_PWM, 0);
        analogWrite(MOTOR_R_DIR, 0);
      }
    }

    // ===================== ODOMETRY =====================
    float v = (wheelSpeedL_actual + wheelSpeedR_actual) / 2;
    float w = (wheelSpeedR_actual - wheelSpeedL_actual) / wheel_base;
    theta += w * dt;
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;

    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.z = sin(theta / 2);
    odom_msg.pose.pose.orientation.w = cos(theta / 2);

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;

    pub_odom.publish(&odom_msg);
    nh.spinOnce();
  }
}
