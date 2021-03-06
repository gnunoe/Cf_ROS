#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy, Imu

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
        self.pidX = PID(20, 12.0, 0.2, -30, 30, "x")
        self.pidY = PID(-20, -12.0, -0.2, -30, 30, "y")
        #50000  800
        self.pidZ = PID(15000, 3000.0,  1500.0, 10000, 57000, "z")
        self.pidYaw = PID(50.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.state = Controller.Manual
        self.targetZ = 1
        self.targetX = 0.0
        self.targetY = -1.0
        self.des_angle = 90.0
        self.lastZ = 0.0
        self.power = 50000.0
	self.pubVz = rospy.Publisher('vel_z', Float32, queue_size=1)
        self.lastJoy = Joy()

    def _cmdVelTelopChanged(self, data):
        self.cmd_vel_telop = data
        if self.state == Controller.Manual:
            self.pubNav.publish(data)

    def pidReset(self):
        self.pidX.reset()
        self.pidZ.reset()
        self.pidZ.reset()
        self.pidYaw.reset()

    def _joyChanged(self, data):
        if len(data.buttons) == len(self.lastJoy.buttons):
            delta = np.array(data.buttons) - np.array(self.lastJoy.buttons)
            print ("Buton ok")
            #Button 1
            if delta[0] == 1 and self.state != Controller.Automatic:
                print("Automatic!")
                #thrust = self.cmd_vel_telop.linear.z
                #print(thrust)
                self.pidReset()
                self.pidZ.integral = self.power/self.pidZ.ki
                self.lastZ = 0.0
                #self.targetZ = 1
                self.state = Controller.Automatic
            #Button 2
            if delta[1] == 1 and self.state != Controller.Manual:
                print("Manual!")
                self.pubNav.publish(self.cmd_vel_telop)
                self.state = Controller.Manual
            #Button 3
            if delta[2] == 1:
                self.state = Controller.TakeOff
                print("TakeOff!")
            #Button 5
            if delta[4] == 1:
                self.targetY = 0.0
                self.des_angle = -90.0
                #print(self.targetZ)
                #self.power += 100.0
                print(self.power)
                #self.state = Controller.Automatic
            #Button 6
            if delta[5] == 1:
                self.targetY = -1.0
                self.des_angle = 90.0
                #print(self.targetZ)
                #self.power -= 100.0
                print(self.power)
                #self.state = Controller.Automatic
        self.lastJoy = data

    def run(self):
        thrust = 0
        print("jello")
        while not rospy.is_shutdown():
            if self.state == Controller.TakeOff:
                t = self.listener.getLatestCommonTime("/mocap", "/Nano_Mark3")
                if self.listener.canTransform("/mocap", "/Nano_Mark3", t):
                    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark3", t)
                    print(position[0],position[1],position[2])			
                    if position[2] > 0.2 or thrust > 54000:
                        self.pidReset()
                        #self.pidZ.integral = thrust / self.pidZ.ki
                        #self.targetZ = 0.5
                        self.state = Controller.Automatic
                        thrust = 0
                    else:
                        thrust += 100
                        self.power = thrust
                        msg = self.cmd_vel_telop
                        msg.linear.z = thrust
                        self.pubNav.publish(msg)

            if self.state == Controller.Automatic:
                # transform target world coordinates into local coordinates
                t = self.listener.getLatestCommonTime("/mocap","/Nano_Mark3")
                seconds = rospy.get_time()
                print(t)
                if self.listener.canTransform("/mocap","/Nano_Mark3", t):
                    position, quaternion = self.listener.lookupTransform("/mocap","/Nano_Mark3",t)
                    #print(position[0],position[1],position[2])	
                    euler = tf.transformations.euler_from_quaternion(quaternion)
		    #print(euler[2]*(180/math.pi))
                    msg = self.cmd_vel_telop
                    #print(self.power)

                    if self.lastZ == 0.0:
		    	self.lastZ = position[2]
			last_time = seconds;
    		    else:
                        dh = self.targetZ - position[2]
   			v_z = (position[2]-self.lastZ)/((seconds-last_time))
                        #print(v_z)
                    	self.pubVz.publish(v_z)
                        
                        #por encima de goal
                        if dh<0.0:
			    self.power -=50
			    #if v_z>0.0:
		 	    #    self.power -=50
			    #else:
				#self.power +=50
			#por debajo de goal
			if dh>0.0:
                            if self.power > 57000.0:
				self.power = 57000.0
			    else:
				self.power += 50
			    #if v_z<0.0:
			#	self.power +=50
			#    else:
			#	self.power -=50
                    
		    print(self.power)
		    msg.linear.z = self.power

                    #Descompostion of the x and y contributions following the Z-Rotation
                    x_prim = self.pidX.update(0.0, self.targetX-position[0])
                    y_prim = self.pidY.update(0.0,self.targetY-position[1])
                    
                    msg.linear.x = x_prim*math.cos(euler[2]) - y_prim*math.sin(euler[2]) 
                    msg.linear.y = x_prim*math.sin(euler[2]) + y_prim*math.cos(euler[2])                    
                    

                    #---old stuff---  
                    #msg.linear.x = self.pidX.update(0.0, 0.0-position[0])
                    #msg.linear.y = self.pidY.update(0.0,-1.0-position[1])
                    #msg.linear.z = self.pidZ.update(position[2],1.0)
                                        
                    #z_prim = self.pidZ.update(0.0,self.targetZ-position[2])
                    #print(z_prim)
                    #if z_prim < self.power:
                    #    msg.linear.z = self.power
                    #else:
                    #    msg.linear.z = z_prim
                    #msg.linear.z = z_prim

                    #msg.linear.z = self.pidZ.update(0.0,1.0-position[2]) #self.pidZ.update(position[2], self.targetZ)
                    #msg.angular.z = self.pidYaw.update(0.0,self.des_angle + euler[2]*(180/math.pi))
                    print(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z)
                    #print(euler[2])
                    #print(msg.angular.z)
                    self.pubNav.publish(msg)

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    controller.run()
