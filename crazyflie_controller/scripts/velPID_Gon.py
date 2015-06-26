#!/usr/bin/env pythontf::Quaternion

import rospy
import math
import tf
import numpy as np
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Joy, Imu
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs


square =[[0.5,0.5,0.5,0.0],
              [0.5,-0.5,0.5,0.0],
              [-0.5,-0.5,0.5,0.0],
              [-0.5,0.5,0.5,0.0]] 




class PID:
    def __init__(self, kp, kd, ki, minOutput, maxOutput, name):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()
        self.pubOutput = rospy.Publisher('pid/output/' + name, Float32, queue_size=1)
        self.pubError = rospy.Publisher('pid/error/' + name, Float32, queue_size=1)
        self.pubP = rospy.Publisher('pid/p/' + name, Float32, queue_size=1)
        self.pubD = rospy.Publisher('pid/d/' + name, Float32, queue_size=1)
        self.pubI = rospy.Publisher('pid/i/' + name, Float32, queue_size=1)

    def reset(self):
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()

    def update(self, value, targetValue):
        time = rospy.get_time()
        dt = time - self.previousTime
        error = targetValue - value
        self.integral += error * dt
        p = self.kp * error
        d = self.kd * (error - self.previousError) / dt
        i = self.ki * self.integral
        output = p + d + i
        self.previousError = error
        self.previousTime = time
        self.pubOutput.publish(output)
        self.pubError.publish(error)
        self.pubP.publish(p)
        self.pubD.publish(d)
        self.pubI.publish(i)
        return max(min(output, self.maxOutput), self.minOutput)

class Controller():
    Manual = 0
    Automatic = 1
    TakeOff = 2
    Land = 3

    def __init__(self):
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("joy", Joy, self._joyChanged)
        rospy.Subscriber("cmd_vel_telop", Twist, self._cmdVelTelopChanged)
        self.cmd_vel_telop = Twist()
        #self.pidX = PID(20, 12, 0.0, -30, 30, "x")
        #self.pidY = PID(-20, -12, 0.0, -30, 30, "y")
        #self.pidZ = PID(15000, 3000.0, 1500.0, 10000, 60000, "z")
        #self.pidYaw = PID(50.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.pidX = PID(20, 12, 0.0, -20, 20, "x")
        self.pidY = PID(-20, -12, 0.0, -20, 20, "y")
        #50000  800
        self.pidZ = PID(15000, 3000.0,  1500.0, 10000, 60000, "z")
        self.pidYaw = PID(50.0, 0.0, 0.0, -100.0, 100.0, "yaw")
        self.state = Controller.Manual
        #Target Values
        self.pubtarX = rospy.Publisher('target_x', Float32, queue_size=1)
	self.pubtarY = rospy.Publisher('target_y', Float32, queue_size=1)
	self.pubtarZ = rospy.Publisher('target_z', Float32, queue_size=1)
        self.targetX = 0.0
        self.targetY = 0.0
	self.targetZ = 0.5
        self.des_angle = 0.0
        #self.power = 50000.0
        #Actual Values
	self.pubrealX = rospy.Publisher('real_x', Float32, queue_size=1)
	self.pubrealY = rospy.Publisher('real_y', Float32, queue_size=1)
	self.pubrealZ = rospy.Publisher('real_z', Float32, queue_size=1)
        self.lastJoy = Joy()

        #Path view
        self.pubPath = rospy.Publisher('cf_Uni_path', MarkerArray, queue_size=100)
        self.path = MarkerArray()
        #self.p = []

	#Square trajectory
	self.square_start = False
	self.square_pos = 0
	#self.square =[[0.5,0.5,0.5,0.0],
        #              [0.5,-0.5,0.5,90.0],
        #              [-0.5,-0.5,0.5,180.0],
        #              [-0.5,0.5,0.5,270.0]] 

	#landing flag
	self.land_flag = False
	self.power = 0.0

	#angular velocity
	self.pubAng_cf = rospy.Publisher('angular_rate/cf', Twist, queue_size=1)
	self.pubAng_mocap = rospy.Publisher('angular_rate/mocap', Twist, queue_size =1)
	self.velang = Twist()

	self.first_value = True
	self.quaternion_last = (0.0,0.0,0.0,0.0)
	#self.angX_last = 0.0
	#self.angY_last = 0.0
	#self.angZ_last = 0.0


    def _cmdVelTelopChanged(self, data):
        self.cmd_vel_telop = data
        if self.state == Controller.Manual:
            self.pubNav.publish(data)

    def pidReset(self):
        self.pidX.reset()
        self.pidZ.reset()
        self.pidZ.reset()
        self.pidYaw.reset()



    def square_go(self):
        if self.square_start == False:
	    self.square_pos = 0
            self.targetX = square[self.square_pos][0]
            self.targetY = square[self.square_pos][1]
            self.targetZ = square[self.square_pos][2]
            self.des_angle = square[self.square_pos][3]
            self.square_pos = self.square_pos + 1
            self.square_start = True
        else:
            self.targetX = square[self.square_pos][0]
            self.targetY = square[self.square_pos][1]
            self.targetZ = square[self.square_pos][2]
            self.des_angle = square[self.square_pos][3]
            self.square_pos = self.square_pos + 1
            if self.square_pos == 4:
                self.square_pos = 0



    def _joyChanged(self, data):
        if len(data.buttons) == len(self.lastJoy.buttons):
            delta = np.array(data.buttons) - np.array(self.lastJoy.buttons)
            print ("Buton ok")
            #Button 1
            if delta[0] == 1 and self.state != Controller.Automatic:
                print("Automatic!")
		self.land_flag = False
                #thrust = self.cmd_vel_telop.linear.z
                #print(thrust)
                self.pidReset()
                self.pidZ.integral = 40.0
                #self.targetZ = 1
                self.state = Controller.Automatic
            #Button 2
            if delta[1] == 1 and self.state != Controller.Manual:
                print("Manual!")
		self.land_flag = False
                self.pubNav.publish(self.cmd_vel_telop)
                self.state = Controller.Manual
            #Button 3
            if delta[2] == 1:
		self.land_flag = False
                self.state = Controller.TakeOff
                print("TakeOff!")
            #Button 4
            if delta[3] == 1:
                self.land_flag = True
                print("Landing!")
		self.square_start = False
                self.targetX = 0.0
                self.targetY = 0.0
                self.targetZ = 0.4
                self.des_angle = 0.0
		self.state = Controller.Automatic
		
            #Button 5
            if delta[4] == 1:
                
		self.square_go()
		#self.targetX = square[0][0]
                #self.targetY = square[0][1]
		#self.targetZ = square[0][2]
                #self.des_angle = square[0][3]
                #print(self.targetZ)
                #self.power += 100.0
                #print(self.power)
                self.state = Controller.Automatic
            #Button 6
            if delta[5] == 1:
		
		self.square_start = False
                self.targetX = 0.0
                self.targetY = 0.0
		self.targetZ = 0.5
                self.des_angle = 0.0
                #print(self.targetZ)
                #self.power -= 100.0
                #print(self.power)
                self.state = Controller.Automatic
        self.lastJoy = data


    def run(self):
        thrust = 0
        print("jello")
        while not rospy.is_shutdown():
	    if self.state == Controller.Manual:
		t = self.listener.getLatestCommonTime("/mocap", "/Nano_Mark_Gon3")
                if self.listener.canTransform("/mocap", "/Nano_Mark_Gon3", t):
		    #linear_m, angular_m = self.listener.lookupTwist("/Nano_Mark_Gon3","/mocap", t, rospy.Duration(0.01))
		    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark_Gon3", t)
		    euler = tf.transformations.euler_from_quaternion(quaternion)
		    #print(angular_m[0], angular_m[1], angular_m[2])

		    if self.first_value == True:
			self.quaternion_last = (
					quaternion[0],
					quaternion[1],
					quaternion[2],
					quaternion[3])
			time_last = rospy.Time.now()
			self.first_value = False	
		    else:
			
			time = rospy.Time.now()
			#Get differential quaternion
			quaternion_diff = (
				quaternion[0] - self.quaternion_last[0],
                                quaternion[1] - self.quaternion_last[1],
                                quaternion[2] - self.quaternion_last[2],
                                quaternion[3] - self.quaternion_last[3])
					
			#and the angular rate
			angular_rate = (2.0*(quaternion_diff)*quaternion.inverse())/(time-time_last)
		
			print(angular_rate)	
			#
			time_last = time
			quaternion_last = quaternion

		     	#Publish data MOCAP
			#ang_msg_m = self.velang
		   	#ang_msg_m.angular.x = angular_m[0]
		   	#ang_msg_m.angular.y = angular_m[1]
		    	#ang_msg_m.angular.z = angular_m[2]
		    	#self.pubAng_mocap.publish(ang_msg_m)

		    #Calculate angular rate in cf_body
		    #angX_cf = math.cos(euler[1])*angular_m[0] - math.cos(euler[0])*math.sin(euler[1])*angular_m[2]
		    #angY_cf = 1*angular_m[1] + math.sin(euler[0])*angular_m[2]
		    #angZ_cf = math.sin(euler[1])*angular_m[0] + math.cos(euler[0])*math.cos(euler[1])*angular_m[2]

		    #Publish data CF + low-pass filter
		    #ang_msg_cf = self.velang
		    #ang_msg_cf.angular.x = (angX_cf + self.angX_last)/2
		    #ang_msg_cf.angular.y = (angY_cf + self.angY_last)/2
		    #ang_msg_cf.angular.z = (angZ_cf + self.angZ_last)/2
		    #self.pubAng_cf.publish(ang_msg_cf)
			
		    #self.angX_last = ang_msg_cf.angular.x
		    #self.angY_last = ang_msg_cf.angular.y
		    #self.angZ_last = ang_msg_cf.angular.z
			
            if self.state == Controller.TakeOff:
                t = self.listener.getLatestCommonTime("/mocap", "/Nano_Mark_Gon3")
                if self.listener.canTransform("/mocap", "/Nano_Mark_Gon3", t):
                    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark_Gon3", t)
                    print(position[0],position[1],position[2])			
                    #if position[2] > 2.0 or thrust > 54000:
		    if thrust > 55000:
                        self.pidReset()
                        self.pidZ.integral = thrust / self.pidZ.ki
                        #self.targetZ = 0.5
                        self.state = Controller.Automatic
                        thrust = 0
                    else:
                        thrust += 500
                        #self.power = thrust
                        msg = self.cmd_vel_telop
                        msg.linear.z = thrust
                        self.pubNav.publish(msg)

	    if self.state == Controller.Land:
                t = self.listener.getLatestCommonTime("/mocap", "/Nano_Mark_Gon3")
                if self.listener.canTransform("/mocap", "/Nano_Mark_Gon3", t):
                    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark_Gon3", t)
		    if position[2] > 0.05:
			msg_land = self.cmd_vel_telop
			self.power -=100
			msg_land.linear.z = self.power
			self.pubNav.publish(msg_land)
		    else:
			msg_land = self.cmd_vel_telop
			msg_land.linear.z = 0
			self.pubNav.publish(msg_land)


            if self.state == Controller.Automatic:
                # transform target world coordinates into local coordinates
                t = self.listener.getLatestCommonTime("/mocap","/Nano_Mark_Gon3")
                print(t)
                if self.listener.canTransform("/mocap","/Nano_Mark_Gon3", t):
                    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark_Gon3",t)
                    #print(position[0],position[1],position[2])	
                    euler = tf.transformations.euler_from_quaternion(quaternion)
		    print(euler[2]*(180/math.pi))
                    msg = self.cmd_vel_telop
                    #print(self.power)

                    #Descompostion of the x and y contributions following the Z-Rotation
                    x_prim = self.pidX.update(0.0, self.targetX-position[0])
                    y_prim = self.pidY.update(0.0,self.targetY-position[1])
                    
                    msg.linear.x = x_prim*math.cos(euler[2]) - y_prim*math.sin(euler[2]) 
                    msg.linear.y = x_prim*math.sin(euler[2]) + y_prim*math.cos(euler[2])                    
                    

                    #---old stuff---  
                    #msg.linear.x = self.pidX.update(0.0, 0.0-position[0])
                    #msg.linear.y = self.pidY.update(0.0,-1.0-position[1])
                    #msg.linear.z = self.pidZ.update(position[2],1.0)
                                        
                    #z_prim = self.pidZ.update(position[2],self.targetZ)
                    #print(z_prim)
                    #if z_prim < self.power:
                    #    msg.linear.z = self.power
                    #else:
                    #    msg.linear.z = z_prim
                    #msg.linear.z = self.power
		    #print(self.power)

                    msg.linear.z = self.pidZ.update(0.0,self.targetZ-position[2]) #self.pidZ.update(position[2], self.targetZ)
                    msg.angular.z = self.pidYaw.update(0.0,self.des_angle*(math.pi/180) + euler[2])#*(180/math.pi))
                    #msg.angular.z = self.pidYaw.update(0.0,self.des_angle - euler[2])#*(180/math.pi))
                    print(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z)
                    #print(euler[2])
                    #print(msg.angular.z)
                    self.pubNav.publish(msg)

		    #Publish Real and Target position
                    self.pubtarX.publish(self.targetX)
                    self.pubtarY.publish(self.targetY)
                    self.pubtarZ.publish(self.targetZ)
                    self.pubrealX.publish(position[0])
                    self.pubrealY.publish(position[1])
                    self.pubrealZ.publish(position[2])

		    #change square point
		    if abs(self.targetX-position[0])<0.08 and \
		       abs(self.targetY-position[1])<0.08 and \
		       abs(self.targetZ-position[2])<0.08 and \
		       self.square_start == True:
		        self.square_go()

		    #Landing 
		    if abs(self.targetX-position[0])<0.1 and \
                       abs(self.targetY-position[1])<0.1 and \
                       abs(self.targetZ-position[2])<0.1 and \
                       self.land_flag == True:
                        self.state = Controller.Land
			self.power = msg.linear.z


		    


                    #Publish Path
                    #point = Marker()
		    #line = Marker()
		     
		    #point.header.frame_id = line.header.frame_id = 'mocap'

		    #POINTS
		    #point.action = point.ADD
		    #point.pose.orientation.w = 1.0
		    #point.id = 0
		    #point.type = point.POINTS
		    #point.scale.x = 0.01
		    #point.scale.y = 0.01
		    #point.color.g = 1.0
		    #point.color.a = 1.0

		    #LINE
		    #line.action = line.ADD
		    #line.pose.orientation.w = 1.0
		    #line.id = 1
		    #line.type = line.LINE_STRIP
		    #line.scale.x = 0.01
		    #line.color.g = 1.0
		    #line.color.a = 1.0

		    #p = Point()
		    #p.x = position[0]
		    #p.y = position[1]
		    #p.z = position[2]

		    #point.points.append(p)
		    # line.points.append(p)

		    #self.path.markers.append(p)
		
		    #id = 0	
		    #for m in self.path.markers:
		#	m.id = id
		#        id += 1

		    #self.pubPath.publish(self.path)

		    #self.pubPath.publish(point)
		    #self.pubPath.publish(line) 
		   
		    point = Marker()
		    point.header.frame_id = 'mocap'
		    point.type = point.SPHERE
		    #points.header.stamp = rospy.Time.now()
		    point.ns = 'cf_Uni_path'
		    point.action = point.ADD
		    #points.id = 0;
		    point.scale.x = 0.005
		    point.scale.y = 0.005
		    point.scale.z = 0.005
		    point.color.a = 1.0
                    point.color.r = 1.0
                    point.color.g = 1.0
                    point.color.b = 0.0
		    point.pose.orientation.w = 1.0
		    point.pose.position.x = position[0]
		    point.pose.position.y = position[1]
		    point.pose.position.z = position[2]
		
		    self.path.markers.append(point)

		    id = 0
   		    for m in self.path.markers:
       			m.id = id
       			id += 1

		    self.pubPath.publish(self.path)
		    #point = Point()
		    #point.x = position[0]
                    #point.y = position[1]
                    #point.z = position[2]
		    
		    #points.points.append(point)
	
		    

                    #self.p.append(pos2path)

		    #self.path.header.stamp = rospy.Time.now()
                    #self.path.header.frame_id = 'mocap'
                    #self.path.poses = self.p
                    #self.pubPath.publish(points)


            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    controller.run()
