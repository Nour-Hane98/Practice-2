#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CollisionAvoider:
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('collision_avoider')
         
        # The minimun safe distance en meters 
        self.safety_threshold = 0.5

        # Number of LIDAR Sampling 
        self.visible_samples = 10   

        self.lidar = None
        self.current_cmd = Twist()
        
        # Subscribe to LIDAR data and velocity command topics 
        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd)
        
        # Publisher to control the robot's motion
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        
        # Setup the Timer 
        rospy.Timer(rospy.Duration(0.1), self.check_motion)
        
        rospy.loginfo("Start collision avoider")

    def callback_lidar(self, msg):
        self.lidar = msg

    def callback_cmd(self, msg):
        self.current_cmd = msg

    def check_motion(self, event):
        # We do nothing if we haven't received the lidar data yet 
        if self.lidar is None:
            return
        
        # Extract the Lidar ranges 
        ranges = self.lidar.ranges

        # Get the left and rght front lidar ranges 
        left_front = ranges[:self.visible_samples]
        right_front = ranges[-self.visible_samples:]

        # Combine them and filter the invalid range values
        front = [r for r in (left_front + right_front) if 0.0 < r < float('inf')]
        
        # Here we need to determinate if the obstacle is closer than the safety threshold
        danger = front and min(front) < self.safety_threshold

        if danger and self.current_cmd.linear.x > 0:
            
            # Here we need to stop the robot 
            stop = Twist()
            self.cmd_pub.publish(stop)

            rospy.logwarn(f"Obstacle detected at  {min(front):.2f} m. Forward motion Blocked")
        else:

            # if the path is safe 
            self.cmd_pub.publish(self.current_cmd)
            rospy.loginfo(f"Safe movement: forward {self.current_cmd.linear.x:.2f} m/s")

if __name__ == '__main__':
    try:
        CollisionAvoider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



