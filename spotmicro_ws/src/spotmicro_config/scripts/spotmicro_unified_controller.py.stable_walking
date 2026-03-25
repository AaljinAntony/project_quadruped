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

# Exact Physical Link Lengths measured by User (mm to meters)
COXA = 0.080     # 80 mm
FEMUR = 0.1112   # 111.2 mm
TIBIA = 0.135    # 135 mm
BODY_W = 0.108   # 108 mm
BODY_L = 0.320   # 320 mm

# IK Configuration
Z_STAND = 0.180  # Nominal walk ride-height 18cm
STEP_H = 0.06    # Step ground clearance 6cm
MAX_REACH = FEMUR + TIBIA - 0.01

# Names match the ESP32 main.cpp mapping array perfectly
JOINT_NAMES = [
    "motor_front_left_shoulder", "motor_front_left_leg", "foot_motor_front_left",
    "motor_front_right_shoulder", "motor_front_right_leg", "foot_motor_front_right",
    "motor_rear_left_shoulder", "motor_rear_left_leg", "foot_motor_rear_left",
    "motor_rear_right_shoulder", "motor_rear_right_leg", "foot_motor_rear_right"
]

class UnifiedController(Node):
    def __init__(self):
        super().__init__('unified_controller')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_pub = self.create_publisher(Int32, '/robot_pose', 10)
        
        # State Machine Flags
        self.state = "STAND"  # STAND, SIT, WALK
        self.target_vel = 0.0 # 0 to 0.05 m/s
        self.current_vel = 0.0
        
        self.w_key_held = False
        
        self.phase = 0.0
        self.dt = 0.02 # 50 Hz exactly matches servo refresh limits
        self.wait_time = 0.0
        
        self.lock = threading.Lock()
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Debug logger
        self.tick_counter = 0
        
    def solve_ik_2d(self, x, z):
        """2D planar IK mapping X (forward) and Z (downward) for a mammalian leg"""
        D = math.sqrt(x**2 + z**2)
        if D > MAX_REACH: D = MAX_REACH
        
        # Knee angle (theta 3) via law of cosines
        c3 = (D**2 - FEMUR**2 - TIBIA**2) / (2 * FEMUR * TIBIA)
        c3 = max(-1.0, min(1.0, c3))
        knee_ang = math.acos(c3)
        
        # Hip angle (theta 2)
        alpha = math.atan2(x, z)
        c_beta = (FEMUR**2 + D**2 - TIBIA**2) / (2 * FEMUR * D)
        c_beta = max(-1.0, min(1.0, c_beta))
        beta = math.acos(c_beta)
        hip_ang = alpha + beta
        
        return hip_ang, knee_ang
        
    def get_standing_angles(self):
        return self.solve_ik_2d(0.0, Z_STAND)
        
    def get_joint_array(self, phase, vel):
        H_stand, K_stand = self.get_standing_angles()
        
        # Calculate optimal physical stride length based on 0.75s stance time
        stance_time = 0.75 
        stride_len = vel * stance_time 
        
        # Trot: Diagonal Phase Offset
        offsets = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
        angles = [0.0] * 12
        
        mapping = [
            # Name, shoulderIdx, legIdx, footIdx, invert_math
            ("FL", 0, 1, 2, 1),
            ("FR", 3, 4, 5, 1),
            ("RL", 6, 7, 8, 1), # Math inversion is handled purely by main.cpp physical calibration!
            ("RR", 9, 10, 11, 1)
        ]
        
        for leg, sIdx, lIdx, fIdx, invR in mapping:
            l_phase = (phase + offsets[leg]) % 1.0
            x, z = 0.0, Z_STAND
            
            if vel > 0.01:
                # 75% stance, 25% swing
                if l_phase < 0.75:
                    # Stance Phase (propel body forward by moving ground backwards)
                    progress = l_phase / 0.75
                    x = (stride_len/2.0) - (stride_len * progress)
                    z = Z_STAND
                else:
                    # Swing Phase (smooth Bezier-like return forward clearing the ground)
                    progress = (l_phase - 0.75) / 0.25
                    x = -(stride_len/2.0) + (stride_len * progress)
                    
                    # Proportional step height based on velocity (starts low when barely moving)
                    dynamic_step_h = max(0.01, (vel / 0.05) * STEP_H)
                    z = Z_STAND - dynamic_step_h * math.sin(math.pi * progress)
            
            # The physical servo mounts caused the math propulsion vector to propel backward.
            # We invert the X calculation so the legs start in front and sweep backward relative to the math axis!
            ik_x = -x * invR
            H_ang, K_ang = self.solve_ik_2d(ik_x, z)
            
            # Since ESP32 0-offsets are biologically "Standing", we just publish the Delta
            angles[sIdx] = 0.0 
            angles[lIdx] = (H_ang - H_stand)
            # Physical analysis of ESP32 mapping dictates the raw math knee delta MUST be inverted!
            angles[fIdx] = -(K_ang - K_stand)
            
            # Debug log data set generation (1Hz)
            if self.tick_counter % 50 == 0 and leg == "FL":
                self.get_logger().info(f"DATASET FL | Phase: {l_phase:.2f} | X: {ik_x:.3f}m, Z: {z:.3f}m | H_delta: {angles[lIdx]:.2f}rad | K_delta: {angles[fIdx]:.2f}rad | V: {vel:.3f}m/s")
            
        return angles

    def control_loop(self):
        with self.lock:
            if self.wait_time > 0:
                self.wait_time -= self.dt
                return
            
            if self.state == "STAND": return
            elif self.state == "SIT": return
            elif self.state == "WALK":
                if self.w_key_held:
                    # RAMP exactly up to 5cm/s (0.05m/s) across 2.5 seconds
                    self.current_vel += 0.02 * self.dt
                    if self.current_vel > 0.05: self.current_vel = 0.05
                else:
                    # W key released -> Ramp down exactly geometrically opposite to ramp-up!
                    self.current_vel -= 0.02 * self.dt
                    if self.current_vel <= 0.001:
                        self.state = "STAND"
                        self.current_vel = 0.0
                        msg = Int32(); msg.data = 1
                        self.pose_pub.publish(msg)
                        self.get_logger().info("Gait zeroed perfectly. C++ 2.0s Smoothstep Catching remaining inertia.")
                        return
                
                # Advance 1 stride cycle per second
                self.phase = (self.phase + self.dt * 1.0) % 1.0
                
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = JOINT_NAMES
                js.position = self.get_joint_array(self.phase, self.current_vel)
                self.joint_pub.publish(js)
                self.tick_counter += 1
        
    def trigger_stand(self):
        with self.lock:
            if self.state != "STAND":
                self.state = "STAND"
                self.current_vel = 0.0
                msg = Int32(); msg.data = 1
                self.pose_pub.publish(msg)
                self.get_logger().info("Commanding: STAND")
            
    def trigger_sit(self):
        with self.lock:
            if self.state != "SIT":
                self.state = "SIT"
                self.current_vel = 0.0
                msg = Int32(); msg.data = 2
                self.pose_pub.publish(msg)
                self.get_logger().info("Commanding: SIT")

    def trigger_walk(self):
        with self.lock:
            if self.state == "SIT":
                self.state = "STAND"
                msg = Int32(); msg.data = 1
                self.pose_pub.publish(msg)
                self.get_logger().info("Commanding STAND... Waiting 3.0s before trot starts (2.0 C++ + 1.0 Strict)")
                self.wait_time = 3.0 
            elif self.state == "STAND":
                if self.wait_time <= 0:
                    self.state = "WALK"
                    self.w_key_held = True
                    self.current_vel = 0.0
                    self.get_logger().info("Executing TROT gait.")
            elif self.state == "WALK":
                 self.w_key_held = True
                 
    def release_walk(self):
        with self.lock:
            self.w_key_held = False

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def input_thread(node):
    settings = termios.tcgetattr(sys.stdin)
    w_was_down = False
    last_w_time = 0.0
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key == 'w':
                last_w_time = time.time()
                if not w_was_down:
                    w_was_down = True
                    node.trigger_walk()
                    
            if key == 'z': node.trigger_stand()
            elif key == 'x': node.trigger_sit()
            elif key == '\x03' or key == 'q': break
            
            # Terminal Hold/Release debounce (handles 500ms OS keyboard repeat gap)
            if w_was_down and (time.time() - last_w_time > 0.6):
                w_was_down = False
                node.release_walk()
                
    except Exception as e: print(e)
    finally: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    print("=============================================")
    print(" SPOTMICRO UNIFIED IK & TROT CONTROLLER")
    print("=============================================")
    print(f" Physical Core: Coxa={COXA*1000}mm, Fem={FEMUR*1000}mm, Tib={TIBIA*1000}mm")
    print(" [Z] - STOP & STAND")
    print(" [X] - STOP & SIT")
    print(" [W] (Hold) - RAMP FORWARD TROT (10 cm/s)")
    print(" [W] (Release) - 2.0s SMOOTH STAND")
    print(" [Q] / Ctrl+C - QUIT")
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
