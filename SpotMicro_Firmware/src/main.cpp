#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <esp_wifi.h>
#include <ESP32Ping.h>

// --- HARDWARE CONFIGURATION ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50
#define VOLTAGE_PIN 34

// Verified Offsets from AGENT_MEMORY.md
int joint_offsets[12] = {
  363, 297, 296, // FL Foot, Leg, Shoulder
  371, 288, 299, // RL Foot, Leg, Shoulder
  236, 316, 315, // FR Foot, Leg, Shoulder
  245, 331, 326  // RR Foot, Leg, Shoulder
};

struct JointMap {
  const char* name;
  int pca_pin;
  int offset_index;
  float direction; // 1.0 for Left side, -1.0 for Right side (Mirrored)
};

// Mirroring Logic: Left side (FL, RL) is positive, Right side (FR, RR) is negative
// to ensure joint movements are synchronized in direction.
JointMap joint_mapping[12] = {
  {"motor_front_left_shoulder", 2, 2, 1.0}, {"motor_front_left_leg", 1, 1, 1.0}, {"foot_motor_front_left", 0, 0, 1.0},
  {"motor_rear_left_shoulder", 6, 5, 1.0},  {"motor_rear_left_leg", 5, 4, 1.0},  {"foot_motor_rear_left", 4, 3, 1.0},
  {"motor_front_right_shoulder", 10, 8, -1.0}, {"motor_front_right_leg", 9, 7, -1.0}, {"foot_motor_front_right", 8, 6, -1.0},
  {"motor_rear_right_shoulder", 14, 11, -1.0}, {"motor_rear_right_leg", 13, 10, -1.0}, {"foot_motor_rear_right", 12, 9, -1.0}
};

// --- NETWORK CONFIGURATION ---
char ssid[] = "SpotMicro";
char psk[] = "spotpassword";
IPAddress agent_ip(10, 42, 0, 1);
const int agent_port = 8888;

// --- MICRO-ROS GLOBALS ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Subscribers
rcl_subscription_t joint_sub;
rcl_subscription_t pose_sub;
rcl_subscription_t calib_sub;

// Publisher
rcl_publisher_t status_pub;

// Messages
sensor_msgs__msg__JointState joint_msg;
std_msgs__msg__Int32 pose_msg;
std_msgs__msg__Int32MultiArray calib_msg;
std_msgs__msg__String status_msg;

char status_buffer[128];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Failed status on line %d: %d\n",__LINE__,(int)temp_rc);}}

// --- UTILITIES ---
int radToPwmTick(float rad, float direction) {
  // 1.0 Rad approx 115 PWM ticks. Apply side direction.
  return 307 + (int)(rad * 115.0 * direction); 
}

float readVoltage() {
    int raw = analogRead(VOLTAGE_PIN);
    return (raw / 4095.0) * 3.3 * (12.0 / 2.0); 
}

void set_pose(int pose_id) {
    // 0=Neutral(307), 1=Stand(Calibrated), 2=Sit(Tucked)
    if (pose_id == 0) {
        for(int i=0; i<16; i++) pwm.setPWM(i, 0, 307);
    } else if (pose_id == 1) {
        for(int i=0; i<12; i++) pwm.setPWM(joint_mapping[i].pca_pin, 0, joint_offsets[joint_mapping[i].offset_index]);
    } else if (pose_id == 2) {
        for(int i=0; i<12; i++) {
            int base_idx = joint_mapping[i].offset_index;
            int type = base_idx % 3; // 2=Shoulder, 1=Leg, 0=Foot
            int pos = joint_offsets[base_idx];
            float dir = joint_mapping[i].direction;
            
            if (type == 1) pos += (int)(60 * dir); // Leg bent up
            if (type == 0) pos -= (int)(60 * dir); // Foot tucked inwards
            pwm.setPWM(joint_mapping[i].pca_pin, 0, pos);
        }
    }
}

// --- CALLBACKS ---
void joint_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  if (msg->position.size > 0) {
    for (size_t i = 0; i < msg->name.size; i++) {
        for (int j = 0; j < 12; j++) {
            if (strcmp(msg->name.data[i].data, joint_mapping[j].name) == 0) {
                float angle_rad = msg->position.data[i];
                float angle_deg = angle_rad * 57.2958; // 180/PI
                
                // Apply direction mirroring to radians
                int base_tick = radToPwmTick(angle_rad, joint_mapping[j].direction);
                // Apply calibration offset center
                int final_tick = base_tick + (joint_offsets[joint_mapping[j].offset_index] - 307);
                pwm.setPWM(joint_mapping[j].pca_pin, 0, final_tick);

                // Log every movement for high-level monitoring
                snprintf(status_buffer, sizeof(status_buffer), "MOVE: %s to %.1f deg", joint_mapping[j].name, angle_deg);
                status_msg.data.data = status_buffer;
                status_msg.data.size = strlen(status_buffer);
                rcl_publish(&status_pub, &status_msg, NULL);
                
                Serial.println(status_buffer);
                break;
            }
        }
    }
  }
}

void pose_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    set_pose(msg->data);
    snprintf(status_buffer, sizeof(status_buffer), "Pose: %d | Batt: %.2fV", msg->data, readVoltage());
    status_msg.data.data = status_buffer;
    status_msg.data.size = strlen(status_buffer);
    rcl_publish(&status_pub, &status_msg, NULL);
}

void calib_callback(const void * msgin) {
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size >= 2) {
        int pin = msg->data.data[0];
        int pwm_val = msg->data.data[1];
        if (pin >= 0 && pin < 16) {
            pwm.setPWM(pin, 0, pwm_val);
            snprintf(status_buffer, sizeof(status_buffer), "Raw PWM: Pin %d -> %d", pin, pwm_val);
            status_msg.data.data = status_buffer;
            status_msg.data.size = strlen(status_buffer);
            rcl_publish(&status_pub, &status_msg, NULL);
        }
    }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Default to Stand on power-up
  set_pose(1);

  WiFi.begin(ssid, psk);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) { delay(500); retry++; }
  
  if (WiFi.status() == WL_CONNECTED) {
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "spotmicro_unified", "", &support));

    RCCHECK(rclc_subscription_init_default(&joint_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
    RCCHECK(rclc_subscription_init_default(&pose_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "robot_pose"));
    RCCHECK(rclc_subscription_init_default(&calib_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "motor_calibration"));
    RCCHECK(rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_status"));

    // Memory Allocation
    joint_msg.name.capacity = 12;
    joint_msg.name.data = (rosidl_runtime_c__String*)malloc(12 * sizeof(rosidl_runtime_c__String));
    for(int i=0; i<12; i++) {
        joint_msg.name.data[i].capacity = 32;
        joint_msg.name.data[i].data = (char*)malloc(32);
    }
    joint_msg.position.capacity = 12;
    joint_msg.position.data = (double*)malloc(12 * sizeof(double));
    calib_msg.data.capacity = 4;
    calib_msg.data.data = (int32_t*)malloc(4 * sizeof(int32_t));
    status_msg.data.capacity = 128;
    status_msg.data.data = (char*)malloc(128);

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_sub, &joint_msg, &joint_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pose_sub, &pose_msg, &pose_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &calib_sub, &calib_msg, &calib_callback, ON_NEW_DATA));
    
    Serial.println("Unified Stance Control Ready.");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }
}
