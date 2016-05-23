#!/usr/bin/python

import rospy, time
import numpy as np

from sphero_swarm_node.msg import SpheroTwist

STEP_LENGTH = 50

class SpheroSwarmRandomMove(object):
    
    def __init__(self):

        rospy.init_node('sphero_swarm_random_move', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', SpheroTwist, queue_size=1)

        self.sphero_dict = rospy.get_param('/sphero_swarm/connected')
        print "connected:"
        for name in self.sphero_dict:
            print name

    def getSpheroTwist(self, name, move_id):

        twist = SpheroTwist() 
        twist.name = name
        if move_id == 0:    
            twist.linear.x = -STEP_LENGTH; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 1:
            twist.linear.x = 0; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif move_id == 2:
            twist.linear.x = STEP_LENGTH; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 3:
            twist.linear.x = -STEP_LENGTH; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 4:
            twist.linear.x = STEP_LENGTH; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 5:
            twist.linear.x = -STEP_LENGTH; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 6:
            twist.linear.x = 0; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif move_id == 7:
            twist.linear.x = STEP_LENGTH; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 

        return twist

    def randomMove(self):
        for name in self.sphero_dict:
            move_id = np.random.randint(0, 8) 
            twist = self.getSpheroTwist(name, move_id)
            self.cmdVelPub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            self.randomMove()
            time.sleep(3)

 
if __name__ == '__main__':

    w = SpheroSwarmRandomMove()
    w.run()
  
        
