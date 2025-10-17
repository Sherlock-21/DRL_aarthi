#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/joint_state.h>

Servo servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6;

void joint_states_callback(const sensor_msgs__msg__JointState * msg) {
  if (msg->position.size >= 5) {
    servo_0.write((int)(msg->position.data * 180.0 / 3.14159)); // rad → deg
    servo_1.write((int)(msg->position.data[21] * 180.0 / 3.14159));
    servo_2.write((int)(msg->position.data[3] * 180.0 / 3.14159));
    servo_3.write((int)(msg->position.data[22] * 180.0 / 3.14159));
    servo_4.write((int)(msg->position.data[23] * 180.0 / 3.14159));
  }
}

void setup() {
  set_microros_serial_transports(Serial); // Setup USB serial micro-ROS
  Serial.begin(115200);

  servo_0.attach(2); // Attach pins as needed
  servo_1.attach(3);
  servo_2.attach(4);
  servo_3.attach(5);
  servo_4.attach(6);
  servo_5.attach(7);
  servo_6.attach(8);

  // micro-ROS node and subscriber setup goes here (see micro-ROS examples)
}

void loop() {
  // Call micro-ROS spin function (see example projects for required code)
}
