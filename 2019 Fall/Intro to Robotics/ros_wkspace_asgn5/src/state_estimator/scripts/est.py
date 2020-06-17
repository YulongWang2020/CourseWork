#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time
import copy

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####
        #print(sens)
        #print(self.x)
        vel_trans = sens.vel_trans
        vel_ang = sens.vel_ang
        num_of_land_mark = sens.readings
        t = 0.01
        # x(k+1) = x(k) + t * vel_trans * cos(theta)
        # y(k+1) = y(k) + t * vel_trans * sin(theta)
        # theta(k+1) = theta(k) + t * vel_ang
        #print(self.x[0],self.x[2],"--------------------------")
        next_est_x = self.x[0] + t * vel_trans  * math.cos(self.x[2])
        next_est_y = self.x[1] + t * vel_trans  * math.sin(self.x[2])
        next_est_ang = self.x[2] + t * vel_ang
        x_hat = [next_est_x,next_est_y,next_est_ang]
        x_hat = numpy.asarray(x_hat).astype(float)
        F = [[1, 0, -t*vel_trans*numpy.sin(self.x[2])], [0, 1, t*vel_trans*numpy.cos(self.x[2])], [0, 0, 1]]
        P_pre = numpy.dot(numpy.dot(F, self.P), numpy.transpose(F)) + self.V
        reading = sens.readings
        readings_check = []
        for i in range(len(reading)):
        # get sensor data
            if numpy.sqrt((next_est_x - reading[i].landmark.x)**2+(next_est_y - reading[i].landmark.y)**2) < 0.1:
                continue
            readings_check.append(reading[i])
        #print(self.x[0],self.x[2],"--------------------------")
        #### ----- YOUR CODE GOES HERE ----- ####
        #  nu = y(k+1) - h ( x^ (k+1) )
        #print(x_hat,"x_hat_xhat")
        # if len(sens.readings) == 0:
        #     self.x = x_hat
        #for i in range(len(sens.readings)):
        # # get sensor data
        # x_l.append(ss[i].landmark.x)
        # y_l.append(ss[i].landmark.y)
        # if numpy.sqrt((next_est_x - x_l[a])**2+(y_pre - y_l[a])**2) < 0.1:
        #     del sens.readings[i]
        #     del x_l[i]
        #     del y_l[i]
        #     a = a - 1
        # a = a + 1
        if len(readings_check) == 0:
            print("x = x _het 00000000000000000000000000000000000000000000000000000")
            self.x = x_hat
            self.P = P_pre
            return 
        else:
            dis = []
            ori = []
            xl = []
            yl = []
            H = []
            y = []
            y_hat = []
            W = []
            #print("working")
            for i in range(len(readings_check)):
                readings = readings_check[i]
                y.append(readings.range)
                y.append(readings.bearing) 
                dis.append(readings.range)
                ori.append(readings.bearing)
                xl.append(readings.landmark.x)
                yl.append(readings.landmark.y)
                H.append([(next_est_x - readings.landmark.x) / numpy.sqrt((next_est_x - readings.landmark.x)**2+(next_est_y - readings.landmark.y)**2),(next_est_y - readings.landmark.y) / numpy.sqrt((next_est_x - readings.landmark.x)**2+(next_est_y - readings.landmark.y)**2),0])
                H.append([(-next_est_y + readings.landmark.y) / ((next_est_x - readings.landmark.x)**2 + (next_est_y - readings.landmark.y)**2),(next_est_x - readings.landmark.x) / ((next_est_x - readings.landmark.x)**2 + (next_est_y - readings.landmark.y)**2),-1])
                y_hat.append(numpy.sqrt((next_est_x-readings.landmark.x)**2 + (next_est_y-readings.landmark.y)**2))
                y_hat.append(math.atan2(readings.landmark.y-next_est_y, readings.landmark.x-next_est_x) - next_est_ang)

            print("start updatting 1111111111111111111111111111111111111111111")
            H = numpy.asarray(H).astype(float)
            print(len(H))
            y_hat = numpy.asarray(y_hat)
            W = numpy.zeros((len(xl)*2,len(xl)*2))
            # print(W)
            # print(H,"HHHHHHHHHHHHHHHH")
            
            for i in range(len(xl)):
                W [i*2,i*2] = 0.1
                W [i*2+1,i*2+1] = 0.05
            W = W.astype(float)
            # print(W)

            # print(P_pre,"P_pre",P_pre.shape)
            nu = numpy.array(y).reshape(2*len(xl),1) - numpy.array(y_hat).reshape(2*len(xl),1)
            #print(nu,H.dot(x_hat))
            # y,numpy.array(y).reshape(2*len(xl),1) - H.dot(x_hat)
            # print(y,"yyyyyyy")
            # print(y_hat,"y_hat")
            #print(nu,"NUNUNUNUNUNU",nu.shape)

            for i in range(len(xl)):
                while nu[2*i+1] > numpy.pi:
                    nu[2*i+1] -= 2*numpy.pi
                while nu[2*i+1] < -numpy.pi:
                    nu[2*i+1] += 2*numpy.pi

            S = numpy.dot(numpy.dot(H, P_pre), numpy.transpose(H)) + W
            S = S.astype(float)
            # print(S,"sssssssssssssssssssssss")
            R = numpy.dot(numpy.dot(P_pre, numpy.transpose(H)), numpy.linalg.inv(S))
            R = R.astype(float)
            # print(R,"RRRRRRRRRRRRRRR")
            # print(x_hat,"x_hat-------------------")
            # print(numpy.dot(R, nu).shape)
            self.x = x_hat + numpy.dot(R, nu)
            self.P = P_pre - numpy.dot(numpy.dot(R, H), P_pre)
            # print(x_hat + numpy.dot(R, nu),"xxxxxxxxxxxxxxx")
            # print(P_pre - numpy.dot(numpy.dot(R, H), P_pre),"ppppppppppppppp")


    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
