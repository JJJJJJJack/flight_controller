#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include <time.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

//#define CHANNEL_MAP_A 1.599340478
//#define CHANNEL_MAP_B -1406.57873

#define JOY_BUTTON_MODE 1

using namespace std;

typedef enum FLIGHT_MODE
  {
   PASS_THROUGH = 0,
   AUTO,
  } FLIGHT_MODE;

FLIGHT_MODE flight_mode = PASS_THROUGH;
geometry_msgs::Twist cmd_vel;
sensor_msgs::Joy joystick_input, joystick_output;
bool joy_ready = false;

//void goal_callback(const geometry_msgs::PoseStamped& message){
//  goal << message.pose.position.x, message.pose.position.y, message.pose.position.z;
//}

void joystick_callback(const sensor_msgs::Joy& message){
  if(joy_ready == false){
    joy_ready = true;
    joystick_output = joystick_input;
  }
  joystick_input = message;
  if(joystick_input.buttons[JOY_BUTTON_MODE] == 1){
    flight_mode = PASS_THROUGH;
  }else{
    flight_mode = AUTO;
  }
}

//void cmd_vel_callback(const geometry_msgs::Twist& message){
//  cmd_vel = message;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bicopter_optitrack_controller");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(100);
  
  ros::Subscriber sub_joy       = n.subscribe("/joy", 1, joystick_callback);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("/joy_control", 5);

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    if(flight_mode == PASS_THROUGH){
      joystick_output = joystick_input;
    }

    if(joy_ready == true)
      pub_joy_output.publish(joystick_output);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
