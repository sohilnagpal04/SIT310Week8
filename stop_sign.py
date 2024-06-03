#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)
        self.cmd_msg = Twist2DStamped()
        #Initialize ROS node
        self.tick_count = 0
        self.front_dis = 0

        self.robot_state = "LANE_FOLLOWING"

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/shravel/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/shravel/fsm_node/mode', FSMState, queue_size=1)
        self.pub = rospy.Publisher('/shravel/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/shravel/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/shravel/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/shravel/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def encoder_callback(self, msg):
        self.tick_count = msg.data

    def range_callback(self, msg):
        self.front_dis = msg.range
    
    def goal_distance(self, distance, linear_speed):
    # Calculate time needed to move the desired distance at the given speed
        init_tick = self.tick_count
        while abs(init_tick - self.tick_count) < ( distance * 100): 
         self.cmd_msg.header.stamp = rospy.Time.now()
         self.cmd_msg.v = linear_speed # striaght line velocity
         self.cmd_msg.omega = 0.0
         self.pub.publish(self.cmd_msg)
         rospy.loginfo("Forward!")
        self.stop_robot()

    def goal_angle(self, angle, angular_speed):
    # Calculate time needed to move the desired distance at the given speed

        init_tick = self.tick_count
        while abs(self.tick_count - init_tick) < ( angle * 25): 
         self.cmd_msg.header.stamp = rospy.Time.now()
         self.cmd_msg.v = 0.0 # striaght line velocity
         self.cmd_msg.omega = angular_speed
         self.pub.publish(self.cmd_msg)
         rospy.loginfo("rotate!")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        rospy.loginfo(detections[0])
        if detections[0].tag_id == 24:
         rospy.loginfo("STOP")
         rospy.sleep(3)
         self.set_state("NORMAL_JOYSTICK_CONTROL")
         rospy.sleep(2)         
         self.set_state("LANE_FOLLOWING") # Go back to lane following
         rospy.sleep(4)

        #############################

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
