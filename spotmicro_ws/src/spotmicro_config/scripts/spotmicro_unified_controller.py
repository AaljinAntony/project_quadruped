#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
import math
import sys
import select
import termios
import tty
import threading
import os
from datetime import datetime

# --- CONFIGURATION (Mirrored from main.cpp for PWM Estimation) ---
PWM_CENTER = 307
TICKS_PER_DEG = 115.0 / 57.2958
JOINT_OFFSETS_DEG = [
    27.4, -7.5, -2.5,   # FL Foot, Leg, Shoulder
    33.9, -2.5, -2.0,   # RL Foot, Leg, Shoulder
    -38.9, -1.0, -0.5,  # FR Foot, Leg, Shoulder
    -41.4, 2.0, 6.5     # RR Foot, Leg, Shoulder
]

JOINT_MAPPING = [
    # Name, pca_pin, offset_idx, direction
    ("motor_front_left_shoulder", 2, 2, 1.0), ("motor_front_left_leg", 1, 1, 1.0), ("foot_motor_front_left", 0, 0, 1.0),
    ("motor_rear_left_shoulder", 6, 5, 1.0),  ("motor_rear_left_leg", 5, 4, 1.0),  ("foot_motor_rear_left", 4, 3, 1.0),
    ("motor_front_right_shoulder", 10, 8, -1.0), ("motor_front_right_leg", 9, 7, -1.0), ("foot_motor_front_right", 8, 6, -1.0),
    ("motor_rear_right_shoulder", 14, 11, -1.0), ("motor_rear_right_leg", 13, 10, -1.0), ("foot_motor_rear_right", 12, 9, -1.0)
]

# Physical Link Lengths (meters)
COXA = 0.080
FEMUR = 0.1112
TIBIA = 0.135
Z_STAND = 0.180
STEP_H = 0.06
MAX_REACH = FEMUR + TIBIA - 0.01

JOINT_NAMES = [m[0] for m in JOINT_MAPPING]

class ActionLogger:
    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        self.master_log = os.path.join(log_dir, "all_movements.log")
        self.current_action_file = None
        self.current_action_name = None
        
        # Try to ensure log dir exists (User might need to fix permissions)
        try:
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            if not os.path.exists(os.path.join(log_dir, "actions")):
                os.makedirs(os.path.join(log_dir, "actions"))
        except Exception as e:
            print(f"[LOGGER ERROR] Could not create log directories: {e}")

    def start_action(self, action_name):
        self.current_action_name = action_name.lower().replace(" ", "_")
        action_path = os.path.join(self.log_dir, "actions", f"{self.current_action_name}.log")
        try:
            self.current_action_file = open(action_path, "a")
        except:
            self.current_action_file = None

    def stop_action(self):
        if self.current_action_file:
            self.current_action_file.close()
            self.current_action_file = None
        self.current_action_name = None

    def log(self, action, joint_angles):
        """
        Logs angles and estimated PWM for all 12 motors.
        joint_angles: list of 12 rad values matching JOINT_NAMES order.
        """
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_entries = []
        
        # Calculate Estimated PWM for each joint
        # Logic matches main.cpp: final_angle_deg = offset + (relative_rad * 57.29 * direction)
        pwm_estimates = []
        for i, name in enumerate(JOINT_NAMES):
            rad_val = joint_angles[i]
            deg_rel = rad_val * 57.2958
            
            # Find hardware mapping params
            mapping = next(m for m in JOINT_MAPPING if m[0] == name)
            offset_deg = JOINT_OFFSETS_DEG[mapping[2]]
            final_deg = offset_deg + (deg_rel * mapping[3])
            pwm_tick = int(PWM_CENTER + (final_deg * TICKS_PER_DEG))
            pwm_estimates.append(pwm_tick)
            
            log_entries.append(f"{name}: {rad_val:.3f}rad ({pwm_tick}pwm)")

        msg = f"[{ts}] ACTION: {action} | " + " | ".join(log_entries) + "\n"
        
        # Write to Master Log
        try:
            with open(self.master_log, "a") as f:
                f.write(msg)
        except: pass
        
        # Write to Action Log
        if self.current_action_file:
            try:
                self.current_action_file.write(msg)
                self.current_action_file.flush()
            except: pass

class UnifiedController(Node):
    def __init__(self):
        super().__init__('unified_controller')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_pub = self.create_publisher(Int32, '/robot_pose', 10)
        
        # State Machine Flags
        self.state = "STAND"  # STAND, SIT, WALK_FWD, WALK_BKWD, TURN_L, TURN_R
        self.current_vel = 0.0
        self.target_max_vel = 0.10 # Updated to 10cm/s as requested
        self.active_key = None
        self.is_waiting_for_stand = False
        
        self.phase = 0.0
        self.dt = 0.02 # 50 Hz
        
        self.logger = ActionLogger()
        self.lock = threading.Lock()
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Advanced Gait & Logging Controller Active (10cm/s Max)")

    def solve_ik_2d(self, x, z):
        D = math.sqrt(x**2 + z**2)
        if D > MAX_REACH: D = MAX_REACH
        c3 = (D**2 - FEMUR**2 - TIBIA**2) / (2 * FEMUR * TIBIA)
        c3 = max(-1.0, min(1.0, c3))
        knee_ang = math.acos(c3)
        alpha = math.atan2(x, z)
        c_beta = (FEMUR**2 + D**2 - TIBIA**2) / (2 * FEMUR * D)
        c_beta = max(-1.0, min(1.0, c_beta))
        beta = math.acos(c_beta)
        hip_ang = alpha + beta
        return hip_ang, knee_ang

    def get_joint_array(self, phase, vel, state):
        H_stand, K_stand = self.solve_ik_2d(0.0, Z_STAND)
        stance_time = 0.75 
        
        # Direction Logic
        x_dir = 1.0 # Forward
        yaw_dir = 0.0
        
        if state == "WALK_BKWD": x_dir = -1.0
        elif state == "TURN_L": yaw_dir = 1.0; x_dir = 0.0
        elif state == "TURN_R": yaw_dir = -1.0; x_dir = 0.0
        
        stride_len = abs(vel) * stance_time 
        
        # Trot: Diagonal Phase Offset
        offsets = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
        angles = [0.0] * 12
        
        # Leg Mappings (matching main.cpp callback logic)
        legs = [
            ("FL", [0,1,2]), ("FR", [3,4,5]), 
            ("RL", [6,7,8]), ("RR", [9,10,11])
        ]
        
        for leg, indices in legs:
            l_phase = (phase + offsets[leg]) % 1.0
            x, z = 0.0, Z_STAND
            
            # Calculate individual leg stride based on global X and local Yaw (Pivot)
            # Left side legs (FL, RL) move forward during Turn Left
            # Right side legs (FR, RR) move backward during Turn Left
            l_stride = stride_len * x_dir
            if leg in ["FL", "RL"]: l_stride += (stride_len * yaw_dir)
            else: l_stride -= (stride_len * yaw_dir)
            
            if abs(vel) > 0.005:
                if l_phase < 0.75:
                    progress = l_phase / 0.75
                    x = (l_stride/2.0) - (l_stride * progress)
                else:
                    progress = (l_phase - 0.75) / 0.25
                    x = -(l_stride/2.0) + (l_stride * progress)
                    dynamic_step_h = max(0.01, (abs(vel) / 0.1) * STEP_H)
                    z = Z_STAND - dynamic_step_h * math.sin(math.pi * progress)
            
            # The firmware inverts X propulsion for biological mapping!
            ik_x = -x 
            H_ang, K_ang = self.solve_ik_2d(ik_x, z)
            
            angles[indices[0]] = 0.0 
            angles[indices[1]] = (H_ang - H_stand)
            angles[indices[2]] = -(K_ang - K_stand)
            
        return angles

    def control_loop(self):
        with self.lock:
            if self.state in ["STAND", "SIT"]:
                # If we were transitioning to stand, check if we arrived
                if self.is_waiting_for_stand and self.current_vel <= 0.001:
                    self.is_waiting_for_stand = False
                    self.get_logger().info("Serialization Complete: STAND.")
                return

            # Ramp Logic (One Action at a Time)
            if self.active_key:
                # Accelerate to 10cm/s (0.1m/s) over 1.0s
                self.current_vel += 0.1 * self.dt
                if self.current_vel > self.target_max_vel:
                    self.current_vel = self.target_max_vel
            else:
                # Decelerate to 0.0 over 1.0s
                self.current_vel -= 0.1 * self.dt
                if self.current_vel <= 0.001:
                    self.current_vel = 0.0
                    self.is_waiting_for_stand = False
                    prev_state = self.state
                    self.state = "STAND"
                    self.logger.stop_action()
                    self.get_logger().info(f"Action Finished: {prev_state}. Ready for next command.")
                    # Publish final stand pose
                    msg = Int32(); msg.data = 1; self.pose_pub.publish(msg)
                    return

            # Advance gait phase
            self.phase = (self.phase + self.dt * 1.5) % 1.0 # 1.5Hz cycle for 10cm/s
            
            joint_angles = self.get_joint_array(self.phase, self.current_vel, self.state)
            
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = JOINT_NAMES
            js.position = joint_angles
            self.joint_pub.publish(js)
            
            # Log Data
            self.logger.log(self.state, joint_angles)

    def trigger_action(self, key):
        with self.lock:
            # Action Serialization: Only allow if fully in STAND and not ramping down
            if self.state != "STAND" or self.is_waiting_for_stand:
                self.get_logger().warn(f"Action '{key}' rejected: Currently busy with {self.state}")
                return

            if key == 'w': self.state = "WALK_FWD"
            elif key == 's': self.state = "WALK_BKWD"
            elif key == 'a': self.state = "TURN_L"
            elif key == 'd': self.state = "TURN_R"
            elif key == 'x': 
                self.state = "SIT"
                msg = Int32(); msg.data = 2; self.pose_pub.publish(msg)
                self.logger.start_action("SIT")
                self.logger.log("SIT", self.get_joint_array(0, 0, "STAND")) # Log static pose
                self.logger.stop_action()
                return
            elif key == 'z':
                self.state = "STAND"
                msg = Int32(); msg.data = 1; self.pose_pub.publish(msg)
                self.logger.start_action("STAND")
                self.logger.log("STAND", self.get_joint_array(0, 0, "STAND"))
                self.logger.stop_action()
                return

            if self.state != "STAND":
                self.active_key = key
                self.current_vel = 0.0
                self.logger.start_action(self.state)
                self.get_logger().info(f"Executing: {self.state}")

    def release_action(self):
        with self.lock:
            if self.active_key:
                self.active_key = None
                self.is_waiting_for_stand = True
                self.get_logger().info(f"Releasing {self.state}... Ramping to STAND.")

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def input_thread(node):
    settings = termios.tcgetattr(sys.stdin)
    last_key = None
    last_key_time = 0.0
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in ['w', 's', 'a', 'd', 'z', 'x']:
                last_key_time = time.time()
                if last_key != key:
                    last_key = key
                    node.trigger_action(key)
                    
            if key == '\x03' or key == 'q': break
            
            # Debounce/Release logic
            if last_key and (time.time() - last_key_time > 0.4):
                if last_key in ['w', 's', 'a', 'd']:
                    node.release_action()
                last_key = None
                
    except Exception as e: print(e)
    finally: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    print("=============================================")
    print(" SPOTMICRO ADVANCED GAIT & LOGGER")
    print("=============================================")
    print(" [W/S] - Forward / Backward (10cm/s)")
    print(" [A/D] - Pivot Left / Right")
    print(" [Z]   - Stand")
    print(" [X]   - Sit")
    print(" [Q]   - Quit")
    print(" --- Action Serialization Active ---")
    print("=============================================")
    
    node = UnifiedController()
    t = threading.Thread(target=input_thread, args=(node,))
    t.daemon = True
    t.start()
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
