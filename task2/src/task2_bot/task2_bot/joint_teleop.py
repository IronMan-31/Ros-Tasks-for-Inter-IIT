import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, termios, tty, time

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_position = [0.0, 0.0, 0.0, 0.0, -0.001, 0.0, 0.0]
        self.joint_names = [
            'Joint_1', 'Joint_2', 'Joint_3',
            'Joint_4', 'Joint_5', 'Joint_6',
            'pointer_extend_joint'
        ]

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'q':
                self.joint_position[0] += 0.1
            elif key == 'w':
                self.joint_position[0] -= 0.1
            elif key == 'e':
                self.joint_position[1] += 0.1
            elif key == 'r':
                self.joint_position[1] -= 0.1
            elif key=='t':
                self.joint_position[2]+=0.1
            elif key=='y':
                self.joint_position[2]-=0.1
            elif key=='u':
                self.joint_position[3]+=0.1
            elif key=='i':
                self.joint_position[3]-=0.1
            elif key=='o':
                self.joint_position[4]+=0.1
            elif key=='p':
                self.joint_position[4]-=0.1
            elif key=='a':
                self.joint_position[5]+=0.1
            elif key=='s':
                self.joint_position[5]-=0.1
            elif key == '\x03':  
                break

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.joint_position
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {self.joint_position}\n")
            
            rclpy.spin_once(self, timeout_sec=0)

def main():
    rclpy.init()
    node = ArmTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
