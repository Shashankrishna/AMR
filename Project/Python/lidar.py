#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class lidar_converter: # Generate LiDAR Field from LiDAR sensor

    def __init__(self):
        # Create variables
        self.msg = Float64MultiArray()
        self.pX = 0
        self.pY = 0
        self.theta = 0

        # Create workspace
        x = np.linspace(0, 400, 1601)
        x = np.round(x, 2)
        y = np.linspace(0, 400, 1601)
        y = np.round(y, 2)

        # Create grids
        self.X, self.Y = np.meshgrid(x, y)

        # Create required subscribers and publishers
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size = 1)
        self.field_pub  = rospy.Publisher("/lidarfield", Float64MultiArray, queue_size = 10) # Custom publisher
        

    def laserCallback(self,data): # Get and generate laser data

        currentX = self.pX
        currentY = self.pY
        currentT = self.theta

        # Angles of LiDAR
        angles = np.linspace(0, 360, 720, endpoint=False)

        # Angles in global space
        lidarAngle = currentT + np.radians(angles)

        # Radii from LiDAR in cm
        r = 100*np.array(data.ranges)

        # Point Arrays
        LX = np.array([])
        LY = np.array([])

        # Sweep of data points
        for i in range(0, 720):
            # Calculate distances in global frame
            distX = currentX + r[i]*np.cos(lidarAngle[i])
            distY = currentY + r[i]*np.sin(lidarAngle[i])

            # Save data points inside workspace
            if (distX != float('inf')) and (distY != float('inf')):
                if (distX >= 0) and (distX <= 400) and (distY >= 0) and (distY <= 400):
                    LX = np.append(LX, round(distX))
                    LY = np.append(LY, round(distY))

        # Remove duplicate points if any
        points = np.array([LX, LY])
        points = np.unique(points, axis=1)
        LX = points[0]
        LY = points[1]

        # Constants for Point Fields
        A = 1;
        sig = np.array([70, 70])

        # Generate LiDAR Field
        mu = np.array([LX[0], LY[0]])
        lidarField = A*np.exp( -1* ( ((np.power(self.X-mu[0], 2)) /(2*sig[0])) + ((np.power(self.Y-mu[1], 2)) /(2*sig[1])) ) )

        for i in range(1,np.size(LX)):
            mu = np.array([LX[i], LY[i]])
            lidarField = lidarField + A*np.exp( -1* ( ((np.power(self.X-mu[0], 2)) /(2*sig[0])) + ((np.power(self.Y-mu[1], 2)) /(2*sig[1])) ) )

        k = lidarField.max()
        lidarField = lidarField/k

        # Prep field for transmission
        field = lidarField

        # Get the dimension for the ouput
        (H,W) = field.shape

        # Flatten the data
        listdata = field.flatten()

        # Publish the data
        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim[0].size = H
        self.msg.layout.dim[1].size = W
        self.msg.data = listdata
        self.field_pub.publish(self.msg)

    def odomCallback(self, data): # Get odometry data

        # Store position in cm
        self.pX = data.pose.pose.position.x*100
        self.pY = data.pose.pose.position.y*100

        # Calculate and store orientation
        quat_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        (roll, pitch, self.theta) = euler_from_quaternion(quat_list)
        
def main(): # Node Main
    lc = lidar_converter()
    rospy.init_node('lidarMap', anonymous=True)
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
