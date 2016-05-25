#!/usr/bin/python
import rospy

import math
import sys
import tf
import PyKDL
import threading

from sphero_driver import sphero_driver
import dynamic_reconfigure.server

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero_swarm_node.msg import SpheroSwarmCollision, SpheroOdometry, SpheroImu, SpheroTwist, SpheroColor, SpheroDisableStabilization, SpheroTurn, SpheroBackLed, SpheroHeading, SpheroAngularVelocity
#from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sphero_node.cfg import ReconfigConfig

from sphero_swarm_node.srv import SpheroInfo, SpheroInfoRequest, SpheroInfoResponse

class SpheroAgent(object):

    def __init__(self, name, bt_addr, parent, lock):
        self.robot_name = name
        self.robot_bt_addr = bt_addr
        self.parent = parent
        self.robot = sphero_driver.Sphero(self.robot_name, self.robot_bt_addr, lock)
        self.robot.setDaemon(True)
    
        self.imu = SpheroImu()
        self.imu.name = name
        self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        
        self.last_cmd_vel_time = rospy.Time.now()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0
        
        self.is_connected = False


    def parse_collision(self, data):
        if self.is_connected:
            now = rospy.Time.now()
            collision = SpheroSwarmCollision()
            collision.header.stamp = now
            collision.name = self.robot_name
            collision.x = data["X"]
            collision.y = data["Y"]
            collision.z = data["Z"]
            collision.axis = int(data["Axis"])
            collision.x_magnitude = data["xMagnitude"]
            collision.y_magnitude = data["yMagnitude"]
            collision.speed = data["Speed"]
            collision.timestamp = data["Timestamp"]
            
            self.collision = collision
            self.parent.collision_pub.publish(self.collision)

    def parse_power_notify(self, data):
        if self.is_connected:
            self.power_state = data
            self.power_state_msg = self.parent.battery_state[data]


    def parse_data_strm(self, data):
        if self.is_connected:
            now = rospy.Time.now()
            imu = SpheroImu(header=rospy.Header(frame_id="imu_link"))
            imu.header.stamp = now
            imu.name = self.robot_name
            imu.orientation.x = data["QUATERNION_Q0"]
            imu.orientation.y = data["QUATERNION_Q1"]
            imu.orientation.z = data["QUATERNION_Q2"]
            imu.orientation.w = data["QUATERNION_Q3"]
            imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096.0*9.8
            imu.angular_velocity.x = data["GYRO_X_FILTERED"]*10*math.pi/180
            imu.angular_velocity.y = data["GYRO_Y_FILTERED"]*10*math.pi/180
            imu.angular_velocity.z = data["GYRO_Z_FILTERED"]*10*math.pi/180

            self.imu = imu
            self.parent.imu_pub.publish(self.imu)

            odom = SpheroOdometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
            odom.header.stamp = now
            odom.name = self.robot_name
            odom.pose.pose = Pose(Point(data["ODOM_X"]/100.0,data["ODOM_Y"]/100.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            odom.twist.twist = Twist(Vector3(data["VELOCITY_X"]/1000.0, 0, 0), Vector3(0, 0, data["GYRO_Z_FILTERED"]*10.0*math.pi/180.0))
            odom.pose.covariance =self.parent.ODOM_POSE_COVARIANCE                
            odom.twist.covariance =self.parent.ODOM_TWIST_COVARIANCE
            self.parent.odom_pub.publish(odom)                      

            #need to publish this trasform to show the roll, pitch, and yaw properly
            self.parent.transform_broadcaster.sendTransform((0.0, 0.0, 0.038 ),
                (data["QUATERNION_Q0"], data["QUATERNION_Q1"], data["QUATERNION_Q2"], data["QUATERNION_Q3"]),
                odom.header.stamp, "base_link", "base_footprint")


class SpheroSwarmNode(object):

    battery_state =  {1:"Battery Charging",
                      2:"Battery OK",
                      3:"Battery Low",
                      4:"Battery Critical"}


    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]


    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    def __init__(self, default_update_rate=50.0):
        rospy.init_node('sphero_swarm')
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)
        self.sphero_dict = {}
        self.sphero_connected = {}
        self._init_pubsub()
        self._init_params()
        
        self.last_diagnostics_time = rospy.Time.now()
        self.mutual_lock = threading.Lock()
        
    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', SpheroOdometry)
        self.imu_pub = rospy.Publisher('imu', SpheroImu)
        self.collision_pub = rospy.Publisher('collision', SpheroSwarmCollision)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', SpheroTwist, self.cmd_vel, queue_size = 20)
        self.cmd_turn_sub = rospy.Subscriber('cmd_turn', SpheroTurn, self.cmd_turn, queue_size = 20)
        self.color_sub = rospy.Subscriber('set_color', SpheroColor, self.set_color, queue_size = 20)
        self.back_led_sub = rospy.Subscriber('set_back_led', SpheroBackLed, self.set_back_led, queue_size = 20)
        self.stabilization_sub = rospy.Subscriber('disable_stabilization', SpheroDisableStabilization, self.set_disable_stabilization, queue_size = 20)
        self.heading_sub = rospy.Subscriber('set_heading', SpheroHeading, self.set_heading, queue_size = 20)
        self.angular_velocity_sub = rospy.Subscriber('set_angular_velocity', SpheroAngularVelocity, self.set_angular_velocity, queue_size = 20)
        self.reconfigure_srv = dynamic_reconfigure.server.Server(ReconfigConfig, self.reconfigure)
        self.transform_broadcaster = tf.TransformBroadcaster()

        self.add_sphero_srv = rospy.Service('add_sphero', SpheroInfo, self.add_sphero)
        self.remove_sphero_srv = rospy.Service('remove_sphero', SpheroInfo, self.remove_sphero)  

    def _init_params(self):
        
        self.connect_color_red = 255
        if rospy.has_param('/sphero_swarm/connect_red'):
            self.connect_color_red = rospy.get_param('/sphero_swarm/connect_red')
        rospy.set_param('/sphero_swarm/connect_red', self.connect_color_red)

        self.connect_color_blue = 255
        if rospy.has_param('/sphero_swarm/connect_blue'):
            self.connect_color_blue = rospy.get_param('/sphero_swarm/connect_blue')
        rospy.set_param('/sphero_swarm/connect_blue', self.connect_color_blue)
       
        self.connect_color_green = 255
        if rospy.has_param('/sphero_swarm/connect_green'):
            self.connect_color_green = rospy.get_param('/sphero_swarm/connect_green')
        rospy.set_param('/sphero_swarm/connect_green', self.connect_color_green)
        
        self.connect_back_led = 255
        if rospy.has_param('/sphero_swarm/connect_back_led'):
            self.connect_back_led = rospy.get_param('/sphero_swarm/connect_back_led',255)
        rospy.set_param('/sphero_swarm/connect_back_led', self.connect_back_led)

        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.diag_update_rate = rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))

        self.team_info = rospy.get_param('/sphero_swarm/team')
        #print str(self.team_info)

    def add_sphero(self, req):
        name = req.name
        bt_addr = req.bt_addr
        print "add sphero (" + str(name) + " " + str(bt_addr) + ")"
        try:
            sphero = SpheroAgent(name, bt_addr, self, self.mutual_lock)
            sphero.is_connected = sphero.robot.connect()
	    rospy.loginfo("Connect to Sphero with address: %s" % bt_addr)
            self.sphero_dict[name] = sphero
            self.team_info[name] = bt_addr
            self.sphero_connected[name] = bt_addr
	except:
            rospy.logerr("Failed to connect to %s." % name)
            return SpheroInfoResponse(0)

        rospy.set_param('/sphero_swarm/team', self.team_info)
        rospy.set_param('/sphero_swarm/connected', self.sphero_connected)
        return SpheroInfoResponse(1)

    def remove_sphero(self, req):
        name = req.name
        bt_addr = req.bt_addr
        print "remove sphero (" + str(name) + " " + str(bt_addr) + ")"
        sphero = self.sphero_dict[name]
        sphero.robot.disconnect()
        del self.sphero_dict[name]
        del self.team_info[name]
        del self.sphero_connected[name]

        rospy.set_param('/sphero_swarm/connected', self.sphero_connected)
        return SpheroInfoResponse(1)

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi);

    def start(self):
        for name in self.team_info:
	    try:
                bt_addr = self.team_info[name]
                print "add SPHERO (" + str(name) + " " + str(bt_addr) + ")"
                sphero = SpheroAgent(name, bt_addr, self, self.mutual_lock)
                #print "start connecting"
		sphero.is_connected = sphero.robot.connect()
		rospy.loginfo("Connect to Sphero with address: %s, (%s)" % (bt_addr, name))
                self.sphero_dict[name] = sphero
                self.sphero_connected[name] = bt_addr
	    except:
		rospy.logerr("Failed to connect to %s." % name)

        if len(self.sphero_dict)==0:
            sys.exit(1)
        
        for name in self.sphero_dict:
            sphero = self.sphero_dict[name]
            if sphero.is_connected == True: 
                #setup streaming    
		sphero.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
		sphero.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], sphero.parse_data_strm)
		#setup power notification
		sphero.robot.set_power_notify(True, False)
		sphero.robot.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'], sphero.parse_power_notify)
		#setup collision detection
		sphero.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
		sphero.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], sphero.parse_collision)
		#set the ball to connection color
		sphero.robot.set_rgb_led(self.connect_color_red,self.connect_color_green,self.connect_color_blue,0,False)
                #set the ball to connection back led
                sphero.robot.set_back_led(int(self.connect_back_led),False)
		#now start receiving packets
		sphero.robot.start()

        rospy.set_param('/sphero_swarm/connected', self.sphero_connected)

    def spin(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            for name in self.sphero_dict:
                sphero = self.sphero_dict.get(name)
                if  (now - sphero.last_cmd_vel_time) > self.cmd_vel_timeout:
                    if sphero.cmd_heading != 0 or sphero.cmd_speed != 0:
                        sphero.cmd_heading = 0
                        sphero.cmd_speed = 0
                        sphero.robot.roll(int(sphero.cmd_speed), int(sphero.cmd_heading), 1, False)
            if (now - self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.publish_diagnostics(now)
            r.sleep()
                    
    def stop(self):    
        print "stop sphero swarm"
  
        #tell the ball to stop moving before quiting
        for name in self.sphero_dict:
            sphero = self.sphero_dict[name]
            sphero.robot.roll(int(0), int(0), 1, False)
            sphero.robot.shutdown = True
        rospy.sleep(1.0)
        for name in self.sphero_dict:
            sphero = self.sphero_dict[name]
            sphero.is_connected = sphero.robot.disconnect()
            sphero.robot.join()
            del self.sphero_dict[name]
            del self.team_info[name]
            del self.sphero_connected[name]
        rospy.set_param('/sphero_swarm/team', self.team_info)
        rospy.set_param('/sphero_swarm/connected', self.sphero_connected)

    def publish_diagnostics(self, time):
        diag = DiagnosticArray()
        diag.header.stamp = time        
        
        for sphero_name in self.sphero_dict:
            sphero = self.sphero_dict[sphero_name]
            stat = DiagnosticStatus(name=sphero_name, level=DiagnosticStatus.OK, message=sphero.power_state_msg)
            #stat.message="Battery Status"
            if sphero.power_state == 3:
                stat.level=DiagnosticStatus.WARN
            if sphero.power_state == 4:
                stat.level=DiagnosticStatus.ERROR
            diag.status.append(stat)

        self.diag_pub.publish(diag)

    def cmd_vel(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            sphero.last_cmd_vel_time = rospy.Time.now()
            sphero.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            sphero.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
            sphero.robot.roll(int(sphero.cmd_speed), int(sphero.cmd_heading), 1, False)

    def cmd_turn(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            sphero.robot.roll(0, int(msg.turn), 0, False)
    
    def set_color(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            sphero.robot.set_rgb_led(int(msg.r*255),int(msg.g*255),int(msg.b*255),0,False)

    def set_back_led(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            sphero.robot.set_back_led(int(msg.back_led), False)

    def set_disable_stabilization(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            if not msg.disable:
                sphero.robot.set_stablization(1, False)
            else:
                sphero.robot.set_stablization(0, False)

    def set_heading(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            heading_deg = int(self.normalize_angle_positive(msg.heading)*180.0/math.pi)
            sphero.robot.set_heading(heading_deg, False)

    def set_angular_velocity(self, msg):
        sphero = self.sphero_dict.get(msg.name)
        if sphero == None:
            return
        if sphero.is_connected:
            rate = int((msg.angular_velocity*180/math.pi)/0.784)
            sphero.robot.set_rotation_rate(rate, False)

    def configure_collision_detect(self, msg):
        pass

    def reconfigure(self, config, level):
        for name in self.sphero_dict:
            sphero = self.sphero_dict[name] 
            if sphero.is_connected:
                sphero.robot.set_rgb_led(int(config['red']*255),int(config['green']*255),int(config['blue']*255),0,False)
        return config

        
if __name__ == '__main__':
    s = SpheroSwarmNode()
    s.start()
    s.spin()
    s.stop()

