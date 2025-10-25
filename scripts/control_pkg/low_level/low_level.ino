#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <WiFi.h>

#define MAX_JOINTS 5
// WiFi credentials
const char* ssid = "Network";
const char* password = "12345678";
const char* agent_ip = "192.168.1.100";  // Your PC's IP address
const int agent_port = 8888;

// Servo objects
Servo servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6;

// micro-ROS objects
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
sensor_msgs__msg__JointState msg;

// Subscriber callback
void joint_states_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  
  if (msg->position.size >= 5) {
    servo_0.write((int)(msg->position.data[0])); // expects degrees 0-180
    servo_1.write((int)(msg->position.data[1]));
    servo_2.write((int)(msg->position.data[2]));
    servo_3.write((int)(msg->position.data[3]));
    servo_4.write((int)(msg->position.data[4]));
  }
  
  Serial.println("Joint command received");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Attach servos to pins
  servo_0.attach(13);
  servo_1.attach(12);
  servo_2.attach(14);
  servo_3.attach(27);
  servo_4.attach(25);
  

  // Set center positions
  servo_0.write(90);
  servo_1.write(90);
  servo_2.write(90);
  servo_3.write(90);
  servo_4.write(0);

  msg.name.capacity = MAX_JOINTS;
  msg.name.size = 0;
  msg.name.data = (rosidl_runtime_c__String*) malloc(MAX_JOINTS * sizeof(rosidl_runtime_c__String));
  
  msg.position.capacity = MAX_JOINTS;
  msg.position.size = 0;
  msg.position.data = (double*) malloc(MAX_JOINTS * sizeof(double));
  
  msg.velocity.capacity = MAX_JOINTS;
  msg.velocity.size = 0;
  msg.velocity.data = (double*) malloc(MAX_JOINTS * sizeof(double));
  
  msg.effort.capacity = MAX_JOINTS;
  msg.effort.size = 0;
  msg.effort.data = (double*) malloc(MAX_JOINTS * sizeof(double));
  
  // Configure micro-ROS WiFi transport
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // Initialize micro-ROS allocator
  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "esp32_joint_subscriber", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states");

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &joint_states_callback, ON_NEW_DATA);

  Serial.println("micro-ROS subscriber ready");
}

void loop() {
  // Spin executor to process callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
