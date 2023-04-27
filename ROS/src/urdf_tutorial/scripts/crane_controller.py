#!usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from crane_interfaces.msg import MotionReference
from crane_interfaces.msg import Visualisation


# https://automaticaddison.com/how-to-write-a-ros2-publisher-and-subscriber-python-foxy/


class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')
        qos_profile = QoSProfile(depth=10)
        
        self.subscription = self.create_subscription(Visualisation, 'state_publisher', self.listener_callback, 10)
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        
        
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        degree = pi / 180.0
        # loop_rate = self.create_rate(30) #Hz
        
        # robot state
        self.theta_boom = 0.
        #t = 1
        omega_boom = 1 * degree    # rad/s
        
        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'base_link'
        odom_trans.child_frame_id = 'crane_boom'
        joint_state = JointState()
        
        self.t = 0
        self.lower=0.35
        self.upper=-0.6
        
    def timer_callback(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['base_to_crane_boom']
        self.joint_state.position = [self.theta_boom]
		
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)
		
    def listener_callback(self, input):
        self.theta_boom = input.boom_angle
        
        if(self.theta_boom < self.upper):
            self.theta_boom = self.upper
        elif(self.theta_boom > self.lower):
            self.theta_boom = self.lower 

#        try:
#            while rclpy.ok():
#                rclpy.spin_once(self)
#
#                # update joint_state
#                now = self.get_clock().now()
#                joint_state.header.stamp = now.to_msg()
#                joint_state.name = ['base_to_crane_boom']
#                joint_state.position = [theta_boom]
#               # update transform
#               odom_trans.header.stamp = now.to_msg()
#               odom_trans.transform.translation.x = -0.56876
#               odom_trans.transform.translation.y = 0.           
#               odom_trans.transform.translation.z = 1.752
#               odom_trans.transform.rotation = \
#                euler_to_quaternion(0, theta_boom, 0) # roll,pitch,yaw
#               # send the joint state and transform
#               self.joint_pub.publish(joint_state)
#               self.broadcaster.sendTransform(odom_trans)
#
#                # Create new robot state
#                t += 1/30
#                theta_boom = sin(t)/2 #omega_boom
#
#                # This will adjust as needed per iteration
#                loop_rate.sleep()
#
#        except KeyboardInterrupt:
#            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

