// Select library depending on arduino
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//Include libraries, one for servo, other for ROS and one last for Vector3
#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Vector3.h>

//Instantiate the node handler, so that we can create publishers and subscribers and manage serial coms. 
ros::NodeHandle  nh;

//Create 3 servo objects, one for each servo motor
Servo servo1, servo2, servo3;

//Callback function; recieves a Vector3 constant
void servo_cb(const geometry_msgs::Vector3& cmd_msg){

  //Use microseconds to control servos with more precision
  servo1.writeMicroseconds(cmd_msg.x); //set servo angle, should be from 0-180  
  servo2.writeMicroseconds(cmd_msg.y); //set servo angle, should be from 0-180  
  servo3.writeMicroseconds(cmd_msg.z); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led; used as debugging tool

  //Wait to update position one second
  delay(1000);
}

//Function to subscribe to the topic (RobAng) that publishes the angles in microseconds, has a Vector3 data type 
//and a callback function servo_cb
ros::Subscriber<geometry_msgs::Vector3> sub("robAngp", servo_cb);

//Arduino setup section
void setup(){
  pinMode(13, OUTPUT);

  //Initiate the node handler
  nh.initNode();

  //Instantiate the subscriber
  nh.subscribe(sub);
  
  //Attatch a pin to the servo objects
  servo1.attach(9); //attach it to pin 9    - Servo 1
  servo2.attach(10); //attach it to pin 10 - Servo 2
  servo3.attach(11); //attach it to pin 11 - Servo 3
}

//Arduino loop section
void loop(){
  //This function manages the callbacks (it excecutes them)
  nh.spinOnce();
  //Delay just for synchronization
  delay(1);
}
