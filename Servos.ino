
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

//Declare an array for saving callback information
float mov[3] = {650, 1000, 1000};

//Callback function; recieves a Vector3 constant
void servo_cb(const geometry_msgs::Vector3& cmd_msg){

  //Save the callback information
  if(mov[0] != cmd_msg.x)
    mov[0] = cmd_msg.x;
  else if(mov[1] != cmd_msg.y)
    mov[1] = cmd_msg.y;
  else if(mov[2] != cmd_msg.z)
    mov[2] = cmd_msg.z;

  //toggle led; used as debugging tool
  digitalWrite(13, HIGH-digitalRead(13));  
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
  servo1.attach(9); //attach it to pin 9   - Servo 1
  servo2.attach(10); //attach it to pin 10 - Servo 2
  servo3.attach(11); //attach it to pin 11 - Servo 3
}

//Arduino loop section
void loop(){
  //This function manages the callbacks (it excecutes them)
  nh.spinOnce();
    
  //Use microseconds to control servos with more precision
  servo1.writeMicroseconds(mov[0]); //Values from 1000 to 2000 aprox which map from 0 to 180' 
  servo2.writeMicroseconds(mov[1]); //Values from 1000 to 2000 aprox which map from 0 to 180'  
  servo3.writeMicroseconds(mov[2]); //Values from 1000 to 2000 aprox which map from 0 to 180'
  
  //Delay just for synchronization
  delay(1);
}
