import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
import math

class MagnetMover(Node):
    def __init__(self):
        super().__init__('magnet_mover')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('正在等待 Gazebo 服务启动...')
        
        # 轨迹参数：慢速、大范围
        self.r_max = 0.10   # 100mm
        self.r_min = 0.02   # 20mm
        self.z_max = 0.12   # 120mm
        self.z_min = 0.04   # 40mm
        self.t_max = 10 * math.pi 
        
        self.t = 0.0        
        self.dt = 0.02      # 提升更新频率到 50Hz，让运动更丝滑
        self.t_step = 0.005 # --- 关键：大幅减小步长，速度会变慢很多 ---
        
        self.timer = self.create_timer(self.dt, self.move_magnet)

    def move_magnet(self):
        if self.t > self.t_max:
            self.t = 0.0 
            return

        current_r = self.r_max - (self.r_max - self.r_min) * (self.t / self.t_max)
        x = current_r * math.cos(self.t)
        y = current_r * math.sin(self.t)
        z = self.z_min + (self.z_max - self.z_min) * (self.t / self.t_max)

        state = SetEntityState.Request()
        state.state.name = 'my_magnet'
        state.state.pose.position.x = x
        state.state.pose.position.y = y
        state.state.pose.position.z = z

        # 保持磁体竖直，消除姿态抖动
        state.state.pose.orientation.w = 1.0
        
        # 【关键修正】：强制将速度设为 0，防止 Gazebo 物理引擎给它叠加位移抖动
        state.state.twist.linear.x = 0.0
        state.state.twist.linear.y = 0.0
        state.state.twist.linear.z = 0.0
        state.state.twist.angular.x = 0.0
        state.state.twist.angular.y = 0.0
        state.state.twist.angular.z = 0.0
        
        self.client.call_async(state)
        self.t += self.t_step

def main():
    rclpy.init()
    node = MagnetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()