#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include <time.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"

#include "BasicPID.h"

//#define CHANNEL_MAP_A 1.599340478
//#define CHANNEL_MAP_B -1406.57873

#define JOY_MODE_AXES 5

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

void joystick_callback(const sensor_msgs::Joy& message){
  if(joy_ready == false){
    joy_ready = true;
    joystick_output = message;
  }
  joystick_input = message;
  if(joystick_input.axes[JOY_MODE_AXES] == 1){
    flight_mode = PASS_THROUGH;
  }else{
    flight_mode = AUTO;
  }
}

void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
  
}

//void cmd_vel_callback(const geometry_msgs::Twist& message){
//  cmd_vel = message;
//}

int main(int argc, char **argv)
{
  float KP_x, KI_x, KD_x;
  float KP_y, KI_y, KD_y;
  float KP_z, KI_z, KD_z;
  float KP_yaw, KI_yaw, KD_yaw;

  ros::init(argc, argv, "bicopter_optitrack_controller");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(100);
  
  ros::Subscriber sub_joy       = n.subscribe("/joy", 1, joystick_callback);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("/joy_control", 5);

  // PID for x
  ros::param::get("/controller/KP_x", KP_x);
  ros::param::get("/controller/KI_x", KI_x);
  ros::param::get("/controller/KD_x", KD_x);
  // PID for y
  ros::param::get("/controller/KP_y", KP_y);
  ros::param::get("/controller/KI_y", KI_y);
  ros::param::get("/controller/KD_y", KD_y);
  // PID for z
  ros::param::get("/controller/KP_z", KP_z);
  ros::param::get("/controller/KI_z", KI_z);
  ros::param::get("/controller/KD_z", KD_z);
  // PID for yaw
  ros::param::get("/controller/KP_yaw", KP_yaw);
  ros::param::get("/controller/KI_yaw", KI_yaw);
  ros::param::get("/controller/KD_yaw", KD_yaw);

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  // PID controller init
  BasicPID controller_pose_x(KP_x, KI_x, KD_x);
  BasicPID controller_pose_y(KP_y, KI_y, KD_y);
  BasicPID controller_pose_z(KP_z, KI_z, KD_z);
  BasicPID controller_pose_yaw(KP_yaw, KI_yaw, KD_yaw);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    // Main logic
    if(flight_mode == PASS_THROUGH){
      // pass through the joystick command
      joystick_output = joystick_input;
    }else{
      // Implementing the position controller
      
    }

    if(joy_ready == true)
      pub_joy_output.publish(joystick_output);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
