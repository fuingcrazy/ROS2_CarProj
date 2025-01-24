import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import threading
import time

class WheelControl(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("WheelControl node has been started")
        self.joint_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.initialize_joint_states()
        self.pub_rate = self.create_rate(10)
        self.thread_ = threading.Thread(target=self.thread_pub_)
        self.thread_.start()

    def initialize_joint_states(self):
        self.joint_states = JointState()
        self.joint_speeds = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ''
        self.joint_states.name = ['front_left_wheel_joint','front_left_roll_joint','front_right_wheel_joint','front_right_roll_joint','rear_left_wheel_joint','rear_right_wheel_joint'] #2 joints for front wheels,1 for rear
        self.joint_states.position = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_states.velocity = self.joint_speeds
        self.joint_states.effort = []

    def update_speed(self,speeds):
        self.joint_speeds = speeds

    def thread_pub_(self):
        last_time = time.time()
        while rclpy.ok():
            delta_time = time.time() - last_time
            last_time = time.time()
            self.joint_states.position[0] += delta_time * self.joint_states.velocity[0]
            self.joint_states.position[2] += delta_time * self.joint_states.velocity[2]
            self.joint_states.position[1] += delta_time * self.joint_states.velocity[1]
            self.joint_states.position[3] += delta_time * self.joint_states.velocity[3]
            self.joint_states.velocity = self.joint_speeds
            self.joint_states.header.stamp = self.get_clock().now().to_msg()

            self.joint_publisher_.publish(self.joint_states)
            self.pub_rate.sleep()
def main(args=None):
    rclpy.init(args=args)
    node = WheelControl("Wheel_control_node")
    node.update_speed([3.0,0.2,-3.0,-0.2,0.0,0.0])
    rclpy.spin(node)
    rclpy.shutdown()


