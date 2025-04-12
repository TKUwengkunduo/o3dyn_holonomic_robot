import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BasicMotionDemoNode(Node):
    def __init__(self):
        super().__init__('basic_motion_demo_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Smooth motion demo (no stop, infinite rotation) started.')

        # ===== 可調參數區 =====
        self.max_speed = 0.6           # 前進 / 橫移最大速度
        self.rotation_speed = 1.2      # 最後旋轉的角速度
        self.motion_duration = 10.0     # 每段動作時間（秒）
        # =====================

        self.state = 0
        self.state_start_time = self.get_clock().now()

        time.sleep(5.0)
        self.timer = self.create_timer(0.001, self.update)

    def smooth_profile(self, t, total_time):
        """
        使用 S 曲線 (smoothstep) 產生優雅的速度比例。
        """
        if t < 0:
            return 0.0
        elif t > total_time:
            return 0.0

        s = t / total_time
        return 3 * s**2 - 2 * s**3

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        twist = Twist()

        if self.state == 0:
            # 前進 (x)
            if elapsed < self.motion_duration:
                scale = self.smooth_profile(elapsed, self.motion_duration)
                twist.linear.x = self.max_speed * scale
                self.publisher_.publish(twist)
            else:
                self.state += 1
                self.state_start_time = now

        elif self.state == 1:
            # 橫移 (y)
            if elapsed < self.motion_duration:
                scale = self.smooth_profile(elapsed, self.motion_duration)
                twist.linear.y = self.max_speed * scale
                self.publisher_.publish(twist)
            else:
                self.state += 1
                self.state_start_time = now

        elif self.state == 2:
            # 無限旋轉 (z)
            twist.angular.z = self.rotation_speed
            self.publisher_.publish(twist)
            # 保持此狀態直到節點被手動關閉

def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionDemoNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
