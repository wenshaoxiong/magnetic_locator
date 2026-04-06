#!/usr/bin/env python3
print("🚀 [DEBUG] Smooth Helix Motion Started...")

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.entity_name = 'my_magnet' 
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            print("❌ Error: Gazebo service not found.")
            return
        
        # --- 恢复流畅的参数 ---
        self.timer_period = 0.05 # 20Hz 刷新，保证物理平滑
        self.radius = 0.12       
        self.z_base = 0.15       
        self.z_amp = 0.05        
        self.speed = 0.2         # 速度适中，画出完美的波浪
        
        self.timer = self.create_timer(self.timer_period, self.move_helix)
        self.start_time = time.time()

    def move_helix(self):
        t = time.time() - self.start_time
        angle = t * self.speed
        
        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose.position.x = self.radius * math.cos(angle)
        req.state.pose.position.y = self.radius * math.sin(angle)
        req.state.pose.position.z = self.z_base + self.z_amp * math.sin(angle * 2.5)
        req.state.pose.orientation.w = 1.0
        
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        
        self.client.call_async(req)

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()