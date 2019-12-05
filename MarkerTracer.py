#!/usr/bin/env python
#Indicate the way this program will be executed

#Import the required libraries
#For Ros-python functions
import rospy

#For math operations
import numpy as np

#For obtaining a pose with a reference coordinate frame and timestamp
from geometry_msgs.msg import PoseStamped

#For obtaining a Vector3 message
from geometry_msgs.msg import Vector3
    
#Declare a class
class MarkerTracer:
    #Declare the constructor of the class
    def __init__(self):
        #Instantiate the attributes of the class
        self.ang = Vector3()
        self.msg = PoseStamped()

        #Instantiate the subscriber /visp_auto_tracker/object_position, 
        #that will be recieving a PoseStamped message and will be calling 
        #the callback function markPoint   
        rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, callMarkPoint)
        #Create the publisher object (the one that will be sending the information to the topic)
        #It publishes to EEP (end effector position) a Vector3 object with a queue_size of 10
        robAng = rospy.Publisher('EEP', Vector3, queue_size=10)
        #We defined a rate of publication of 10Hz (that's why we use 10 for the queue size)
        rate = rospy.Rate(10) # 10hz

        #Create the loop for the program execution
        while not rospy.is_shutdown():
            #Execute the marker tracker function
            MarkPosTrans()
            #Publish the marker tracker solution
            robAng.publish(self.ang)
            #Sleep the time that is necesary for synchronization
            rate.sleep()

    #Define function where all the math is computed
    def MarkPosTrans():
        #We create a matrix of 4 rows and 1 colums and fill it with "1"
        #This will keep the pose from the visp auto tracker as a vector
        #The extra 1 is necessary to do the proper representation of a vector
        vec = np.ones((4,1))
        
        #The values retrieved from the visp auto tracker must be converted before being manipulated
        #After careful analizys, the conversion to cm is as follows: 
        vec[0][0] = (msg.pose.position.x * 96.71) - 0.1
        vec[1][0] = (msg.pose.position.y * 269.71 * 3) / 10
        vec[2][0] = (msg.pose.position.z * 67) / 0.7446
        
        #After many measurements in cm we created the homogeneous transformation matrix of the camera w.r.t. the base
        htm = np.array(((-1,0,0,0),(0,0.9612,-0.2756,41),(0,0.2756,-0.9612,67),(0,0,0,1)))
        
        #Here we just compute the multiplication of the vector and the homogenous transformation matrix
        #to get position of the marker w.r.t. the camera
        mp = np.matmul(htm, vec)

        #All that is left is to assign the values to the output vector with some offset so the marker can still be seen 
        self.ang.x = mp[0][0]
        self.ang.y = mp[1][0] - 5
        
        #For Z, more complex corrections must be computed, first we measured the height of the camera to the floor, that is 67cm
        #Then, we preform some math to the ang.x and substract the pose.position.z in cm
        self.ang.z = (-0.0173 * self.ang.x) + 4.399 + 67 - vec[2][0]
        
        #In case the previous calculation outputs a value that may damage the TCP 
        #We just protect it from hitting the floor 
        #3.5 is a value that leaves the camera in a rest position??????
        if(self.ang.z < 3.5):
            self.ang.z = 3.5

#Callback function definition
def callMarkPoint(msg):  
    #Assign the value recieved from the callback to the attribute msg
    self.msg = msg

if __name__ == '__main__':
    #Initiate the ROS node VisCam, use the argument anonymous=True to add an extra number at the end of the node name
    #and make it unique (just for precaution)
    rospy.init_node('VisCam', anonymous=True)
    #Instantiate the class (this will keep the node running)
    try: 
        MarkerTracer()
    except:
        pass
