import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

class PoseSetter(Node):
    def __init__(self):
        super().__init__('pose_setter')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 Gazebo 服务...')

    def set_pos(self, x, y, z):
        req = SetEntityState.Request()
        req.state.name = 'my_magnet'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0 # 保持竖直
        
        # 停止运动，防止之前的脚本还在推它
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"✅ 磁铁已移动到: x={x}, y={y}, z={z}")

def main():
    rclpy.init()
    if len(sys.argv) < 4:
        print("用法: python3 set_pose.py <x> <y> <z>")
        return
    
    setter = PoseSetter()
    setter.set_pos(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.shutdown()

if __name__ == '__main__':
    main()
