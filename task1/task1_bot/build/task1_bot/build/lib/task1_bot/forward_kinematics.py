from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty
import rclpy

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

move_map = {
    'w': (0.2, 0.0),
    's': (-0.2, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'x':(0.0,0.0)
}


class forward_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub=self.create_publisher(Twist,"cmd_vel",10)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)

        try:
            while True:
                key=get_key(settings)
                if key in move_map:
                    lin,ang=move_map[key]
                    msg=Twist()
                    msg.linear.x=lin
                    msg.angular.z=ang
                    self.pub.publish(msg)
                if key=='s':
                    break
        except Exception as e:
            self.get_logger.error(e)

def main(args=None):
    rclpy.init(args=args)
    nd=forward_node('for_node')
    nd.run()
    rclpy.shutdown()