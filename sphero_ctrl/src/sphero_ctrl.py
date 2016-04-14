#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32

class SpheroCtrl:

    def __init__(self):
        rospy.init_node('sphero_ctrl')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_turn_pub = rospy.Publisher('cmd_turn', Float32, queue_size=1)
 
        #self.sphero_target_sub = rospy.Subscriber("sphero_pos", Pose2D, self.sphero_pos_callback)
        #self.sphero_position_sub = rospy.Subscriber("sphero_target_pos", Pose2D, self.sphero_target_pos_callback)
        self.sphero_target_sub = rospy.Subscriber("tracked_pos", Pose2D, self.sphero_pos_callback)
        self.sphero_position_sub = rospy.Subscriber("target_pos", Pose2D, self.sphero_target_pos_callback)
        
        self.sphero_target_pos = [-1,-1]
        self.runnable = False
    
        self.K = 0.1

    def start(self):
        self.runnable = True

    def stop(self):
        cv = Twist()
        cv.linear.x = 0.0
        cv.linear.y = 0.0
        cv.linear.z = 0.0
        cv.angular.x = 0.0
        cv.angular.y = 0.0
        cv.angular.z = 0.0
        self.cmd_vel_pub.publish(cv)
        
        self.runnable = False

    def sphero_pos_callback(self, msg):
        if self.runnable == True:
            current_pos = [float(msg.x), float(msg.y)]
            print "current pos : " + str(current_pos)
            if self.sphero_target_pos[0] >= 0 and self.sphero_target_pos[1] >= 0:
                delta_x = self.sphero_target_pos[0] - current_pos[0]
                delta_y = self.sphero_target_pos[1] - current_pos[1]
                print "delta pos : " + str([delta_x, delta_y])
                
                cv = Twist()
                
                cv.linear.x = - max(self.K * delta_x , 10.0)
                cv.linear.y = max(self.K * delta_y, 10.0)
                cv.linear.z = 0.0
                cv.angular.x = 0.0
                cv.angular.y = 0.0
                cv.angular.z = 0.0
                print "vel : " + str([cv.linear.x, cv.linear.y])             
                self.cmd_vel_pub.publish(cv)
                

        

    def sphero_target_pos_callback(self, msg):
        self.sphero_target_pos = [float(msg.x), float(msg.y)]
        print "target pos : " + str(self.sphero_target_pos)



if __name__ == '__main__':

    ctrl = SpheroCtrl()
    ctrl.start()
    rospy.spin()
    ctrl.stop()

    
