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
        self.pidX = PID(20, 12, 0.0, -30, 30, "x")
        self.pidY = PID(-20, -12, 0.0, -30, 30, "y")
        self.pidZ = PID(15000, 3000.0, 1500.0, 10000, 60000, "z")
        self.pidYaw = PID(-50.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.state = Controller.Manual
        self.targetZ = 0.5
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
            if delta[0] == 1 and self.state != Controller.Automatic:
                print("Automatic!")
                thrust = self.cmd_vel_telop.linear.z
                print(thrust)
                self.pidReset()
                self.pidZ.integral = thrust / self.pidZ.ki
                self.targetZ = 0.5
                self.state = Controller.Automatic
            if delta[1] == 1 and self.state != Controller.Manual:
                print("Manual!")
                self.pubNav.publish(self.cmd_vel_telop)
                self.state = Controller.Manual
            if delta[2] == 1:
                self.state = Controller.TakeOff
                print("TakeOff!")
            if delta[4] == 1:
                self.targetZ += 0.1
                print(self.targetZ)
            if delta[5] == 1:
                self.targetZ -= 0.1
                print(self.targetZ)
        self.lastJoy = data

    def run(self):
        thrust = 0
        while not rospy.is_shutdown():
            if self.state == Controller.TakeOff:
                t = self.listener.getLatestCommonTime("/world", "/vicon/crazyflie/crazyflie")
                if self.listener.canTransform("/world", "/vicon/crazyflie/crazyflie", t):
                    position, quaternion = self.listener.lookupTransform("/world", "/vicon/crazyflie/crazyflie", t)

                    if position[2] > 0.05 or thrust > 50000:
                        self.pidReset()
                        self.pidZ.integral = thrust / self.pidZ.ki
                        self.targetZ = 0.5
                        self.state = Controller.Automatic
                        thrust = 0
                    else:
                        thrust += 100
                        msg = self.cmd_vel_telop
                        msg.linear.z = thrust
                        self.pubNav.publish(msg)

            if self.state == Controller.Automatic:
                # transform target world coordinates into local coordinates
                t = self.listener.getLatestCommonTime("/vicon/crazyflie/crazyflie", "/world")
                if self.listener.canTransform("/vicon/crazyflie/crazyflie", "/world", t):
                    targetWorld = PoseStamped()
                    targetWorld.header.stamp = t
                    targetWorld.header.frame_id = "world"
                    targetWorld.pose.position.x = 0
                    targetWorld.pose.position.y = 0
                    targetWorld.pose.position.z = self.targetZ
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
                    targetWorld.pose.orientation.x = quaternion[0]
                    targetWorld.pose.orientation.y = quaternion[1]
                    targetWorld.pose.orientation.z = quaternion[2]
                    targetWorld.pose.orientation.w = quaternion[3]

                    targetDrone = self.listener.transformPose("/vicon/crazyflie/crazyflie", targetWorld)

                    quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    msg.linear.x = self.pidX.update(0.0, targetDrone.pose.position.x)
                    msg.linear.y = self.pidY.update(0.0, targetDrone.pose.position.y)
                    msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z) #self.pidZ.update(position[2], self.targetZ)
                    msg.angular.z = self.pidYaw.update(0.0, euler[2])
                    #print(euler[2])
                    #print(msg.angular.z)
                    self.pubNav.publish(msg)

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    controller.run()
