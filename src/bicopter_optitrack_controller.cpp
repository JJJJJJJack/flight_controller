#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"

#include "BasicPID.h"

#define ros_freq 100

// Joystick button settings
#define JOY_CHANNEL_ARM 0
#define JOY_CHANNEL_FLIGHT_MODE 1
// Joy stick axes settings
#define JOY_CHANNEL_ROLL 0
#define JOY_CHANNEL_PITCH 1
#define JOY_CHANNEL_YAW 2
#define JOY_CHANNEL_THROTTLE 3
#define JOY_CONTROL_TYPE 5


using namespace std;

typedef enum FLIGHT_MODE
  {
   PASS_THROUGH = 0,
   AUTO_TAKEOFF,
   AUTO_LAND,
   AUTO_FLYING,
   AUTO_IDLE,
  } FLIGHT_MODE;

FLIGHT_MODE flight_mode = PASS_THROUGH, flight_mode_previous = PASS_THROUGH;
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseStamped pose_world, goal_world, origin_world, pose_local, goal_local;
sensor_msgs::Joy joystick_input, joystick_output;
bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = false;

geometry_msgs::PoseStamped frame_transform(geometry_msgs::PoseStamped world, geometry_msgs::PoseStamped origin){
  geometry_msgs::PoseStamped local = world;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(world.pose.position.x - origin.pose.position.x,
				  world.pose.position.y - origin.pose.position.y,
				  world.pose.position.z - origin.pose.position.z));
  
  return local;
}

void joystick_callback(const sensor_msgs::Joy& message){
  if(JOY_READY == false){
    JOY_READY = true;
    joystick_output = message;
  }
  joystick_input = message;
  if(joystick_input.axes[JOY_CONTROL_TYPE] == 0){
    flight_mode = PASS_THROUGH;
  }else if(flight_mode == PASS_THROUGH)
    if(TRANSFORM_READY == true){
      // Only set takeoff from pass through when pose feedback is ready.
      flight_mode = AUTO_TAKEOFF;
    }else{
      ROS_ERROR_THROTTLE(1, "No position feedback yet! Please double check!");
      ROS_ERROR_THROTTLE(1, "Fallback to PASS_THROUGH mode...");
    }
}

void goal_callback(const geometry_msgs::PoseStamped& message){
  if(flight_mode != AUTO_LAND){
    goal_world = message;
    goal_local = frame_transform(goal_world, origin_world);
  }
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(message.pose.position.x,
				  message.pose.position.y,
				  message.pose.position.z));
  tf::Quaternion q(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
}

void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
  //CONTROLLER_READY = true;
  pose_world = message;
  pose_local = frame_transform(pose_world, origin_world);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(message.pose.position.x,
				  message.pose.position.y,
				  message.pose.position.z));
  tf::Quaternion q(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad"));
  if(LOCALFRAME_READY == true){
    // Start publising the local frame
    transform.setOrigin(tf::Vector3(origin_world.pose.position.x,
				    origin_world.pose.position.y,
				    origin_world.pose.position.z));
    tf::Quaternion q1(origin_world.pose.orientation.x,
		      origin_world.pose.orientation.y,
		      origin_world.pose.orientation.z,
		      origin_world.pose.orientation.w);
    transform.setRotation(q1);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "origin"));
  }
}

float saturate(float input, float min, float max){
  if(input < min)
    return min;
  if(input > max)
    return max;
  return input;
}

int main(int argc, char **argv)
{
  float KP_x, KI_x, KD_x;
  float KP_y, KI_y, KD_y;
  float KP_z, KI_z, KD_z;
  float KP_yaw, KI_yaw, KD_yaw;
  float X_limit, Y_limit, Z_limit, YAW_limit;
  tfScalar roll, pitch, yaw;
  float TAKEOFF_HEIGHT, TAKEOFF_THROTTLE, TAKEOFF_THROTTLE_TIME, LAND_TIME;
  float ros_dt = 1.0/ros_freq;

  ros::init(argc, argv, "bicopter_optitrack_controller");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(ros_freq);
  
  ros::Subscriber sub_joy       = n.subscribe("/joy", 1, joystick_callback);
  ros::Subscriber sub_pose       = n.subscribe("/vrpn_client_node/bicopter/pose", 1, vrpn_pose_callback);
  ros::Subscriber sub_goal       = n.subscribe("/goal", 1, goal_callback);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("/joy_control", 5);

  // PID for x
  ros::param::get("/controller/KP_x", KP_x);
  ros::param::get("/controller/KI_x", KI_x);
  ros::param::get("/controller/KD_x", KD_x);
  ros::param::get("/controller/X_limit", X_limit);
  // PID for y
  ros::param::get("/controller/KP_y", KP_y);
  ros::param::get("/controller/KI_y", KI_y);
  ros::param::get("/controller/KD_y", KD_y);
  ros::param::get("/controller/Y_limit", Y_limit);
  // PID for z
  ros::param::get("/controller/KP_z", KP_z);
  ros::param::get("/controller/KI_z", KI_z);
  ros::param::get("/controller/KD_z", KD_z);
  ros::param::get("/controller/Z_limit", Z_limit);
  // PID for yaw
  ros::param::get("/controller/KP_yaw", KP_yaw);
  ros::param::get("/controller/KI_yaw", KI_yaw);
  ros::param::get("/controller/KD_yaw", KD_yaw);
  ros::param::get("/controller/YAW_limit", YAW_limit);

  // Some additional params
  ros::param::get("/controller/TAKEOFF_HEIGHT", TAKEOFF_HEIGHT);
  ros::param::get("/controller/TAKEOFF_THROTTLE", TAKEOFF_THROTTLE);
  ros::param::get("/controller/TAKEOFF_THROTTLE_TIME", TAKEOFF_THROTTLE_TIME);  
  ros::param::get("/controller/LAND_TIME", LAND_TIME);

  tf::TransformListener listener;
  
  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  // PID controller init
  BasicPID controller_pose_x(KP_x, KI_x, KD_x);
  BasicPID controller_pose_y(KP_y, KI_y, KD_y);
  BasicPID controller_pose_z(KP_z, KI_z, KD_z);
  BasicPID controller_pose_yaw(KP_yaw, KI_yaw, KD_yaw);
  controller_pose_x.limitIntegral(X_limit);
  controller_pose_y.limitIntegral(Y_limit);
  controller_pose_z.limitIntegral(Z_limit);
  controller_pose_yaw.limitIntegral(YAW_limit);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;
  float idletime = 0, land_height = 0;;

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    // Listen to tf transform in auto mode
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("quad", "goal", ros::Time(0), transform);
      TRANSFORM_READY = true;
    }catch(tf::TransformException &ex){
      // Warn in AUTO mode without feedback
      if(flight_mode != PASS_THROUGH){
	ROS_ERROR("Transform not ready! Check vrpn...");
	TRANSFORM_READY = false;
      }
    }
    
    // Main logic
    switch(flight_mode){
    case PASS_THROUGH:{
      // pass through the joystick command
      joystick_output = joystick_input;
      break;
    }
    case AUTO_TAKEOFF:{
      // Get the current Z and set it as ground Z (set only once)
      if(flight_mode_previous != AUTO_TAKEOFF){
	// Set local frame origin
	origin_world.pose = pose_world.pose;
	LOCALFRAME_READY = true;
      }
      // Only allow takeoff when joystick is set on arm
      if(joystick_input.buttons[JOY_CHANNEL_ARM] == 0){
	ROS_WARN_THROTTLE(1, "Warning: Waiting for arming command from joy...");
      }else{
	// Put default arming and flight mode in joystick
	joystick_output.buttons[JOY_CHANNEL_ARM] = 1;
	joystick_output.buttons[JOY_CHANNEL_FLIGHT_MODE] = 1;
	// Take off (gradually increase throttle to predefiend throttle command)
	if(joystick_output.axes[JOY_CHANNEL_THROTTLE] < TAKEOFF_THROTTLE)
	  joystick_output.axes[JOY_CHANNEL_THROTTLE] += ros_dt/TAKEOFF_THROTTLE_TIME*TAKEOFF_THROTTLE;      
	// If the current local Z is greater than TAKEOFF_HEIGHT, finish takeoff process
	if(pose_local.pose.position.z > TAKEOFF_HEIGHT)
	  flight_mode = AUTO_FLYING;
      }
      break;
    }
    case AUTO_LAND:{
      // Record the first land height for accurate land timing
      if(flight_mode_previous != AUTO_LAND)
	land_height = pose_local.pose.position.z;
      // Use current x,y as goal and gradually reduce height
      if(goal_local.pose.position.z >= 0)
	goal_local.pose.position.z -= ros_dt/LAND_TIME*land_height;
      // IDLE motor after land
      if(pose_local.pose.position.z <= TAKEOFF_HEIGHT / 4.0)
	flight_mode = AUTO_IDLE;
      // Intential fall-through
    }
    case AUTO_FLYING:{
      // Implementing main controller
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      controller_pose_x.update(saturate(transform.getOrigin().x(), -2, 2), 0, ros_dt);
      controller_pose_y.update(saturate(transform.getOrigin().y(), -2, 2), 0, ros_dt);
      controller_pose_z.update(saturate(transform.getOrigin().z(), -2, 2), 0, ros_dt);
      controller_pose_yaw.update(saturate(yaw, -M_PI, M_PI), 0, ros_dt);
      
      joystick_output.axes[JOY_CHANNEL_PITCH] = saturate(-controller_pose_x.output, -1, 1);
      joystick_output.axes[JOY_CHANNEL_ROLL] = saturate(controller_pose_y.output, -1, 1);
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(controller_pose_z.output + TAKEOFF_THROTTLE, 0, 1);
      joystick_output.axes[JOY_CHANNEL_YAW] = saturate(controller_pose_yaw.output, -1, 1);
      break;
    }
    case AUTO_IDLE:{
      if(flight_mode_previous != AUTO_IDLE){
	idletime = totaltime;
      }
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = 0;
      // Disarm after 3 sec in idle
      if(totaltime - idletime > 3.0)
	joystick_output.axes[JOY_CHANNEL_ARM] = 0;
      break;
    }
    }

    flight_mode_previous = flight_mode;

    if(JOY_READY == true)
      pub_joy_output.publish(joystick_output);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
