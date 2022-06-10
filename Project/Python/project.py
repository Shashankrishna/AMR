#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Joy, Range, LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import random
import numpy as np
import time

cmdVel = Twist()
cmdVel.linear.x, cmdVel.angular.z = 0,0

# Parameters
max_vel = 0.3
min_vel = 0.07
max_steering = np.pi/6

Kp = 100
Kp_angle = 1

pX = 0
pY = 0
theta = 0

update_lidar = 0

# Generate Empty LiDAR Field
lidarField = np.zeros([1601, 1601])

def LaserCallback(data): # Retrieve LiDAR data from lidar.py publisher
    global lidarField, update_lidar
    rows = data.layout.dim[0].size
    cols = data.layout.dim[1].size

    # Create matrix from data
    field = (np.asarray(data.data)).reshape(rows, cols)


    if (update_lidar == 0): # Get single pulse of LiDAR
        print("Got LiDAR")
        update_lidar = 1
        lidarField = field # Store data
    

def OdomCallback(data): # Get odometry data
    global pX, pY, theta
    
    # Store position in cm
    pX = data.pose.pose.position.x*100 
    pY = data.pose.pose.position.y*100

    # Calculate and store orientation
    quat_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, theta) = euler_from_quaternion(quat_list)


def main():
    # Setup publishers and subscribers    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.init_node('joy_node', anonymous=True)
    rate = rospy.Rate(20)
    rospy.Subscriber("/lidarfield", Float64MultiArray, LaserCallback) # Custom subscriber
    rospy.Subscriber("odom", Odometry, OdomCallback)

    # Wait to aquire LiDAR data
    i = 1
    while (update_lidar == 0):
        print(i)
        i = i+1
        rate.sleep()

    # Setup workspace (400 cm, 0.25 increments)
    x = np.linspace(0, 400, 1601) 
    x = np.round(x, 2)
    y = np.linspace(0, 400, 1601)
    y = np.round(y, 2)

    # Create grids for matrix math
    X, Y = np.meshgrid(x, y)

    # Define goal from camera data
    xg = 380
    yg = 380

    # Create Goal Field
    goalField = 0.5*(np.power((X-xg), 2) + np.power((Y-yg), 2))

    k = goalField.max()
    goalField = goalField/k

    # Create Camera Field, loaded from data file 
    # (generated by createCameraField.py)
    camField = np.loadtxt("camField.csv", delimiter=',')

    # Compute Gradient
    f = goalField + camField + lidarField
    np.savetxt("field.csv", f, delimiter=",") # Save combined field to data file

    gArr = np.gradient(f)

    FX = np.array(gArr[1]) # Gradient matrices 
    FY = np.array(gArr[0])
    FX = -1*FX
    FY = -1*FY

    # Storage arrays
    strX = np.array([0.0]) # Robot X pos
    strY = np.array([0.0]) # Robot Y pos
    strT = np.array([0.0]) # Robot Theta
    strV = np.array([0.0]) # Robot Vel.
    strG = np.array([0.0]) # Robot Angular Vel.
    
    # Control Loop
    while not rospy.is_shutdown():

        # Debug print statements
        print("----")
        print(pX)
        print(pY)
        print(np.degrees(theta))
               
        # Find closest point on grid
        tmpX = round(pX*4)/4
        tmpY = round(pY*4)/4

        # Find indexes of point on grid
        indx = np.where(x==tmpX)[0][0]
        indy = np.where(y==tmpY)[0][0]

        # Get gradient at point
        grad = np.array([FX[indy][indx], FY[indy][indx]])

        # Length of gradient
        g_len = np.sqrt(np.power(grad[0],2) + np.power(grad[1],2))

        # Normalized gradient
        norm_x = grad[0]/g_len
        norm_y = grad[1]/g_len
        
        # Velocity calculation
        vel = Kp*g_len

        if (np.abs(vel) > max_vel): # Vel Range check
            vel = np.sign(vel)*(max_vel)

        if (np.abs(vel) < min_vel): # Vel Range check
            vel = np.sign(vel)*(min_vel)

        # Compute angular velocity
        thetaTarget = np.arctan2(norm_y, norm_x)

        thetaGoal = thetaTarget - theta

        gamma = Kp_angle * (thetaGoal)

        gamma = np.arctan2(np.sin(gamma), np.cos(gamma))

        if (np.abs(gamma) > max_steering): # Angular Vel Range check
            gamma = np.sign(gamma)*(max_steering)

        # Publish velocity commands
        cmdVel.linear.x = vel
        cmdVel.angular.z = gamma

        # Record information
        strX = np.append(strX, pX)
        strY = np.append(strY, pY)
        strT = np.append(strT, theta)
        strG = np.append(strG, gamma)
        strV = np.append(strV, vel)

        # Exit condition
        if (np.sqrt( np.power(xg-pX, 2) + np.power(yg-pY, 2) ) <= 1):
            cmdVel.linear.x = 0
            cmdVel.angular.z = 0
            vel_pub.publish(cmdVel)
            rate.sleep()

            # Store recorded data
            output = np.array([strX, strY, strV, strT, strG])
            np.savetxt("data.csv", output.T, delimiter=",")
            break

        vel_pub.publish(cmdVel)
        rate.sleep()

if __name__ == "__main__":
    main()
