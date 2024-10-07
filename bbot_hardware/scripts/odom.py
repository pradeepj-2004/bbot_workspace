#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import math

class SkidSteeringOdomNode(Node):

    def __init__(self):
        super().__init__('skid_steering_odom_node')
        
        self.wheel_base = 0.28  # distance between the wheels
        self.wheel_radius = 0.07/2  # radius of the wheels
        self.ticks_per_revolution = 205 # encoder ticks per wheel revolution
        self.slip_factor = 1.0  # slip factor (adjust based on your robot)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
           
        self.left_degree = 0.0
        self.right_degree = 0.0
        self.prev_left_degree = 0
        self.prev_right_degree  = 0
  
        # Subscriptions
        self.wheel_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        #timer function to update
        self.timer_=self.create_timer(0.05,self.update_odometry)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = self.get_clock().now()
        self.i=self.get_clock().now()
        

    def joint_state_callback(self, msg):
        self.left_degree = msg.position[0]
        self.right_degree = msg.position[1]
        # print(self.left_degree ,self.right_degree)


   

    def update_odometry(self):
        if ((self.get_clock().now()-self.i).nanoseconds / 1e9 < 1):
            self.prev_left_degree  = self.left_degree   
            self.prev_right_degree = self.right_degree  
            self.prev_time = self.get_clock().now()
        
        else:

            current_time = self.get_clock().now()
            self.prev_time = current_time

            left_change=self.left_degree  - self.prev_left_degree 
            right_change=self.right_degree - self.prev_right_degree
            self.prev_left_degree  = self.left_degree   
            self.prev_right_degree = self.right_degree  

 

            #variable overflow handling cases
            # if(fl_ticks >2147483649):
            #     fl_ticks=-2*2147483647+fl_ticks                
            # elif(fl_ticks <-2147483649):
            #     fl_ticks=2*2147483647+fl_ticks

                
            # if(fr_ticks >2147483649):
            #     fr_ticks=-2*2147483647+fr_ticks
            # elif(fr_ticks <-2147483649):
            #     fr_ticks=2*2147483647+fr_ticks



            # Calculate wheel displacements
            left_distance  = self.wheel_radius*(left_change)
            right_distance = self.wheel_radius*(right_change)
            
            # Calculate change in position and orientation
            d = (left_distance + right_distance) / 2.0
            theta_delta = (right_distance-left_distance ) / (self.wheel_base)
    
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)
            self.theta += theta_delta

            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
    
            # Position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            # Orientation
            q = self.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            #speed  
            # odom_msg.twist.twist.linear.x=vel_robot
            # odom_msg.twist.twist.angular.z=omega_robot
            
            # Publish the odometry message
            self.odom_pub.publish(odom_msg)

            # Broadcast the transform over TF
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = SkidSteeringOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()