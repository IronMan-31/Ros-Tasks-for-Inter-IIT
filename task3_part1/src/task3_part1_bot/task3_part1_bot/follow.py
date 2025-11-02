import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        
        self.robo1_pose = None
        self.robo2_pose = None
        
        self.create_subscription(PoseStamped, '/aruco_pose', self.robo1_callback, 10)
        self.create_subscription(Odometry, '/odom', self.robo2_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.follow)

    def robo1_callback(self, msg):
        self.robo1_pose = msg.pose

    def robo2_callback(self, msg):
        self.robo2_pose = msg.pose.pose

    def follow(self):
        if self.robo1_pose is None or self.robo2_pose is None:
            return

        dx = self.robo1_pose.position.x 
        dy = self.robo1_pose.position.y 
        distance = math.sqrt(dx**2 + dy**2)

        # Orientation of robo2
        q = self.robo2_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        angle_to_target = math.atan2(dy, dx)

        angle_diff = -angle_to_target
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  

        self.get_logger().info(f'X - {dx}  , Y - {dy} ,  angle_diff - {angle_diff}')

        # Command velocities
        msg = Twist()
        if not(abs(dx)<0.8 and abs(dy)<0.8):
            msg.linear.x = 0.3
            msg.angular.z = 1.0 * angle_diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
