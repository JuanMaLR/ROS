#!/usr/bin/env python
#Indicate the way this program will be executed
 
#Import the required libraries
#For Ros-python functions
import rospy
 
#For math operations
import numpy as np
 
#For obtaining a Vector3 message
from geometry_msgs.msg import Vector3
 
#Declare a class
class InvKin: 
    #Declare the constructor of the class
    def __init__(self):
        #Instantiate the class attributes
        self.ang = Vector3()
        
        #Save in variables the measured values of the robotic arm (in cm)
        #Used the same convention as the one in the presentation
        #L0 is the base and L1 is the revolute joint from the base
        self.d1 = 10.5 #This is the distance of the offset from L0 to L2
        self.L2 = 14 #This is the value from the base (where the servos are) to the third joint
        self.L3 = 15.2 #This is the value from the thrid joint to the end effector (without the offset)
        self.a4 = 2.5 #This is the distance of the offset from L3 to the TCP (in x direction)
        self.d5 = 3.7 #This is the distance of the offset from L3 to the TCP (in z direction)
        
        #Instantiate the subscriber EEP (end effector position), that will be recieving a Vector3 message and will be calling
        #the callback function calcAng
        rospy.Subscriber('EEP', Vector3, calcAng)
        #Create the publisher object (the one that will be sending the information to the topic)
        #It publishes to robAng a Vector3 object with a queue_size of 10
        robAng = rospy.Publisher('robAngp', Vector3, queue_size=10)
        #We defined a rate of publication of 10Hz (that's why we use 10 for the queue size)
        rate = rospy.Rate(10) # 10hz
        
        #Create the loop for the program execution
        while not rospy.is_shutdown():
            #Execute the inverse kinematics calculation
            InvKinCalc()
            #Publish the inverse kinematics solution
            robAng.publish(self.ang)
            #Sleep the time that is necesary for synchronization
            rate.sleep()
     
    #Define function where all the math is computed
    def InvKinCalc():
        #First we calculate the angle of the base
        #Simple operation using arctan2 function from numpy (y/x coordinates)
        #Save the result in the x element of the empty Vector3 object
        self.ang.x = np.arctan2(p.y,p.x)

        #Corrections considering the offsets
        #For correcting x coordinate we only need to consider the offset a4 (which is the x direction distance from L3 to TCP)
        #We substract the value because the third joint is before the TCP; this centers it
        #We multiply a4 by the cosine of ang.x because we are in the x-axis and it's given in the presentation
        pointxc = p.x - self.a4*np.cos(self.ang.x)
        #Same as above, just here is sin because we are in the y-axis and it's given in the presentation
        pointyc = p.y - self.a4*np.sin(self.ang.x)
        #The last correction goes on z, but here we add one deviation and substract the other
        #d5 is added because the third joint is above the TCP, if we substracted it, then the TCP will try to go through the floor
        #d1 is substracted because TCP is above the offset.
        #This way, the TCP is able to reach the base plane of the robot. It's also given in the presentation
        pointzc = p.z + self.d5 - self.d1

        #Next, we take from the presentation the formulas for d and D and we compute them
        d = np.sqrt(pointxc**2 + pointyc**2)
        D = np.sqrt(d**2 + pointzc**2)

        #And finally we calculate the missing parameters
        #alpha, beta and gamma are calculated from the presentations formula
        ains = (D**2  + L2**2 - L3**2) / (2 * L2 * D)
        alpha = np.arccos(ain)
        bins = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
        beta  = np.arccos(bins)
        gamma = np.arctan2(pz,d)
   
    def calcAng(p):
        #Before moving the robot, we need to validate that the point provided can be reached by the arm
        #Thanks to the presentation notes this can be validated checking if cos(alpha) and cos(beta) are between -1 and 1
        if ((ains <= 1.0) and (ains >= -1.0) and (bins <= 1.0) and (bins >= -1.0)):

           #We convert the angles from radians into degrees and make proper adjustments
           #x is just the value in radians converted
           self.ang.x = np.degrees(ang.x)
           #y is just the sum of alpha and gamma (taken from the presentation)
           self.ang.y = np.degrees(alpha + gamma)
           #z is beta minus 180
           self.ang.z = np.degrees(beta) - 180
          
           #Lastly, we convert from degrees to arduino microseconds for more precision
           #This formula was developed by us after trying to move a servo using the microseconds function from arduino
           #We saw that the values in microseconds that a servo can move are between 1000 and 2000 (0º and 180º)
           #ang.x = (ang.x * 2000) / 180
           #However, in the case an angle of 0 is computed the angle value will be 0º and that is incorrect
           #So we need to add 1000, so that instead of 0us in 0º there will be 1000us
           #ang.x = 1000 + (ang.x * 2000) / 180
           #Besides that, we need to reduce the maximum angle value, so that in 180º the us will be 1000 instead 
           #of 2000, so that the sum returns 2000
           #To do that, we reduce the multiplication by half, so that with an 180º the us will be 2000
           #Perhaps we need to do some additional calibration
           self.ang.x = 1000 + (self.ang.x * 1000) / 180
           self.ang.y = 1000 + (self.ang.y * 1000) / 180
           self.ang.z = 1000 + (self.ang.z * 1000) / 180  
          
        else:
           #If the specified points are out of reach we decide to send the robot values to 0º
           self.ang.x = 1000.0
           self.ang.y = 1000.0
           self.ang.z = 1000.0

    
#Main definition
if __name__ == '__main__':
   #Initiate the ROS node invKin, use the argument anonymous=True to add an extra number at the end of the node name
   #and make it unique (just for precaution)
   rospy.init_node('invKin', anonymous=True)
   #Instantiate the class (this will keep the node running)
   InkKin()
