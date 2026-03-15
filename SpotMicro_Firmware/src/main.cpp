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
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

// --- HARDWARE CONFIGURATION ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50
#define VOLTAGE_PIN 34 

// --- NETWORK CONFIGURATION ---
char ssid[] = "SpotMicro";
char psk[] = "spotpassword";
IPAddress agent_ip(10, 42, 0, 1);
size_t agent_port = 8888;

// --- JOINT MAPPING (SAME AS MAIN.CPP) ---
struct JointMap {
  const char* name;
  int pca_pin;
};

JointMap joint_mapping[12] = {
  {"FL Foot", 0}, {"FL Leg", 1}, {"FL Shoulder", 2},
  {"BL Leg", 4}, {"BL Shoulder", 5}, {"FR Foot", 6},
  {"FR Shoulder", 8}, {"BR Foot", 9}, {"BR Leg", 10},
  {"BR Shoulder", 12}, {"BL Foot", 13}, {"FR Leg", 14}
};

// --- MICRO-ROS VARIABLES ---
rcl_subscription_t calib_subscriber;
rcl_publisher_t status_publisher;
std_msgs__msg__Int32MultiArray calib_msg;
std_msgs__msg__String status_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

char status_buffer[200];

float readVoltage() {
    int raw = analogRead(VOLTAGE_PIN);
    float voltage = (raw / 4095.0) * 3.3 * (12.0 / 2.0); 
    return voltage;
}

void calib_callback(const void * msgin) {
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    
    if (msg->data.size >= 2) {
        int motor_idx = msg->data.data[0];
        int pwm_tick = msg->data.data[1];
        
        if (motor_idx >= 0 && motor_idx < 12) {
            pwm.setPWM(joint_mapping[motor_idx].pca_pin, 0, pwm_tick);
            
            float v = readVoltage();
            snprintf(status_buffer, sizeof(status_buffer), 
                     "[%s] PIN: %d, PWM: %d, Batt: %.2fV", 
                     joint_mapping[motor_idx].name, motor_idx, pwm_tick, v);
            
            status_msg.data.data = status_buffer;
            status_msg.data.size = strlen(status_buffer);
            rcl_publish(&status_publisher, &status_msg, NULL);
            
            Serial.println(status_buffer);
        }
    }
}

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "spotmicro_calibration_node", "", &support);

    rclc_subscription_init_default(&calib_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/motor_calibration");
    rclc_publisher_init_default(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/motor_status");

    calib_msg.data.capacity = 2;
    calib_msg.data.data = (int32_t*) malloc(2 * sizeof(int32_t));
    status_msg.data.capacity = 200;
    status_msg.data.data = (char*) malloc(200 * sizeof(char));

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &calib_subscriber, &calib_msg, &calib_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
