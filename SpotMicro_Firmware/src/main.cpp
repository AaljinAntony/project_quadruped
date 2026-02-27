#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

// --- HARDWARE CONFIGURATION ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address for PCA9685
#define SERVO_FREQ 50 // Standard analog/digital servo frequency

// --- NETWORK CONFIGURATION ---
// Replace these with your actual Wi-Fi and Host machine details
char ssid[] = "YOUR_WIFI_SSID";
char psk[] = "YOUR_WIFI_PASSWORD";
IPAddress agent_ip(192, 168, 1, 100); // The IP address of the computer running ROS 2 (update this!)
size_t agent_port = 8888;

// --- CALIBRATION OFFSETS ---
// These are the values you will find using the joint_state_publisher_gui.
float joint_offsets[12] = {
  0.0, 0.0, 0.0, // Front Left (Shoulder, Leg, Foot)
  0.0, 0.0, 0.0, // Front Right (Shoulder, Leg, Foot)
  0.0, 0.0, 0.0, // Rear Left (Shoulder, Leg, Foot)
  0.0, 0.0, 0.0  // Rear Right (Shoulder, Leg, Foot)
};

// --- JOINT MAPPING ---
// Maps the exact ROS 2 joint string name to the physical PCA9685 pin (0-11)
struct JointMap {
  const char* name;
  int pca_pin;
  int offset_index;
};

JointMap joint_mapping[12] = {
  {"motor_front_left_shoulder", 0, 0},
  {"motor_front_left_leg", 1, 1},
  {"foot_motor_front_left", 2, 2},
  {"motor_front_right_shoulder", 3, 3},
  {"motor_front_right_leg", 4, 4},
  {"foot_motor_front_right", 5, 5},
  {"motor_rear_left_shoulder", 6, 6},
  {"motor_rear_left_leg", 7, 7},
  {"foot_motor_rear_left", 8, 8},
  {"motor_rear_right_shoulder", 9, 9},
  {"motor_rear_right_leg", 10, 10},
  {"foot_motor_rear_right", 11, 11}
};

// --- MICRO-ROS VARIABLES ---
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- MATH CONVERSION FUNCTION ---
int radToPwmTick(float radians) {
  // Convert Radians to Microseconds (1500us is center, +/- 1000us for 90 degrees)
  float pulse_us = 1500.0 + (radians * (1000.0 / 1.5708));
  
  // Constrain to safe physical limits to protect the DS3235 servos
  if (pulse_us < 500) pulse_us = 500;
  if (pulse_us > 2500) pulse_us = 2500;

  // Convert Microseconds to 12-bit PCA9685 Tick (0-4095)
  int tick = (int)(pulse_us / 4.8828);
  return tick;
}

// --- SUBSCRIBER CALLBACK ---
void joint_state_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  
  if (msg->position.size > 0 && msg->name.size > 0) {
    for (size_t i = 0; i < msg->name.size; i++) {
      // Find which physical pin this joint string corresponds to
      for (int j = 0; j < 12; j++) {
        if (strcmp(msg->name.data[i].data, joint_mapping[j].name) == 0) {
          // 1. Subtract the calibration offset to zero the physical leg
          float corrected_angle_rad = msg->position.data[i] - joint_offsets[joint_mapping[j].offset_index];
          
          // Print the received angle to the Arduino Serial Monitor
          Serial.printf("Received Front Left Angle: %.2f rad\n", msg->position.data[0]);
          
          // 2. Convert to PWM ticks
          int pwm_tick = radToPwmTick(corrected_angle_rad);
          
          // 3. Send to specific PCA9685 pin
          pwm.setPWM(joint_mapping[j].pca_pin, 0, pwm_tick);
          break; // Found the match, move to next incoming joint
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);

  // Initialize Micro-ROS via Wi-Fi
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create Support and Node
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_servo_bridge", "", &support);

  // Create Subscriber targeting CHAMP's Joint States
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states" 
  );

  // --- CRITICAL MEMORY ALLOCATION FOR INCOMING MESSAGES ---
  // Pre-allocate memory for dynamic arrays to prevent deserialization crashes!
  joint_msg.name.capacity = 12;
  joint_msg.name.data = (rosidl_runtime_c__String *) malloc(12 * sizeof(rosidl_runtime_c__String));
  joint_msg.name.size = 0;
  for(int i = 0; i < 12; i++){
      joint_msg.name.data[i].capacity = 50; 
      joint_msg.name.data[i].data = (char *) malloc(50 * sizeof(char));
      joint_msg.name.data[i].size = 0;
  }

  joint_msg.position.capacity = 12;
  joint_msg.position.data = (double*) malloc(12 * sizeof(double));
  joint_msg.position.size = 0;

  joint_msg.velocity.capacity = 12;
  joint_msg.velocity.data = (double*) malloc(12 * sizeof(double));
  joint_msg.velocity.size = 0;

  joint_msg.effort.capacity = 12;
  joint_msg.effort.data = (double*) malloc(12 * sizeof(double));
  joint_msg.effort.size = 0;

  // Create Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &joint_msg, &joint_state_callback, ON_NEW_DATA);
}

void loop() {
  // Keep the micro-ROS agent spinning to catch incoming CHAMP messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}