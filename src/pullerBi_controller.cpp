#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"

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
#define JOY_CONTROL_LAND 4
#define JOY_CONTROL_TYPE 5

// Joy stick nob land threshold, Land signal will only be triggered at value lower
#define JOY_LAND_NOB_THRESHOLD -0.9f
// Flip time window
#define FLIPTIME 0.8f
#define ASCENDTIME_FORWARD 0.4f
#define ASCENDTIME_BACK 0.3f
#define PI acos(-1)
using namespace std;

typedef enum FLIGHT_MODE
  {
   PASS_THROUGH = 0,
   AUTO_TAKEOFF,
   AUTO_LAND,
   AUTO_FLYING,
   AUTO_IDLE,
   AUTO_FLIP,
   DOCK} FLIGHT_MODE;

int dflag=0;
FLIGHT_MODE flight_mode = PASS_THROUGH, flight_mode_previous = PASS_THROUGH;
geometry_msgs::Twist cmd_vel;
geometry_msgs::TwistStamped ctrl_error;
string QUAD_NAME;
char TF_GOAL_NAME[100], TF_QUAD_NAME[100], TF_ORIGIN_NAME[100];
geometry_msgs::PoseStamped pose_world, goal_world, origin_world, pose_local, goal_local;
geometry_msgs::Vector3 goal_euler;
geometry_msgs::Vector3Stamped debug_data;
geometry_msgs::Vector3Stamped yaw_debug_data;
sensor_msgs::Joy joystick_input, joystick_output;
bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = false, GOAL_EULER_READY = false;
struct timeval vrpn_now, vrpn_last;
ros::Time last_vrpn_server_time;
bool first_vrpn = true, VRPN_UPDATE = false;
double dynamic_DT = 0;

geometry_msgs::PoseStamped frame_transform(geometry_msgs::PoseStamped world, geometry_msgs::PoseStamped origin){
  geometry_msgs::PoseStamped local = world;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(world.pose.position.x - origin.pose.position.x,
				  world.pose.position.y - origin.pose.position.y,
				  world.pose.position.z - origin.pose.position.z));
  local.pose.position.x = world.pose.position.x - origin.pose.position.x;
  local.pose.position.y = world.pose.position.y - origin.pose.position.y;
  local.pose.position.z = world.pose.position.z - origin.pose.position.z;
  return local;
}

void goalEuler_callback(const geometry_msgs::Vector3& message){
  GOAL_EULER_READY = true;
  goal_euler = message;
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
    if(TRANSFORM_READY == true && joystick_input.axes[JOY_CONTROL_LAND] > JOY_LAND_NOB_THRESHOLD){
      // Only set 
      flight_mode = AUTO_TAKEOFF;
    }else{
      ROS_ERROR_THROTTLE(1, "No position feedback or Landing mode! Please double check!");
      ROS_ERROR_THROTTLE(1, "Fallback to PASS_THROUGH mode...");
    }
}

void goal_callback(const geometry_msgs::PoseStamped& message){
  if(flight_mode != AUTO_LAND){
    goal_world = message;
    goal_local = goal_world;
  }
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(goal_local.pose.position.x,
				  -goal_local.pose.position.y,
				  goal_local.pose.position.z));
  tf::Quaternion q(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  tfScalar yaw_goal, pitch_goal, roll_goal;
  tf::Matrix3x3 mat_goal(q);
  mat_goal.getEulerYPR(yaw_goal, pitch_goal, roll_goal);
  tf::Quaternion q_goal;
  q_goal.setEulerZYX(-yaw_goal, 0, 0);
  transform.setRotation(q_goal);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", TF_GOAL_NAME));
}

void land_callback(const std_msgs::Int8& message){
  // Accept only land mode
  if(message.data == 1 && flight_mode != AUTO_IDLE && flight_mode != PASS_THROUGH)
    flight_mode = AUTO_LAND;
}

void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
  VRPN_UPDATE = true;
  // duration time test
  double vrpn_dt = 0.0f;
  if (first_vrpn)
  {
    first_vrpn = false;
    gettimeofday(&vrpn_last,NULL);
  }
  else
  {
    gettimeofday(&vrpn_now,NULL);
    vrpn_dt = vrpn_now.tv_sec - vrpn_last.tv_sec + 1e-6 * (vrpn_now.tv_usec - vrpn_last.tv_usec);
    //cout<<"vrpn_dt:  " <<vrpn_dt<<endl;
    vrpn_last = vrpn_now;
  }
  dynamic_DT = message.header.stamp.toSec() - last_vrpn_server_time.toSec();
  last_vrpn_server_time = message.header.stamp;

  //CONTROLLER_READY = true;
  pose_world = message;
  pose_local = frame_transform(pose_world, origin_world);

  static tf::TransformBroadcaster br;
  tf::Transform transform, transform_origin;
  transform.setOrigin(tf::Vector3(pose_world.pose.position.x,
				  pose_world.pose.position.y,
				  pose_world.pose.position.z));
  tf::Quaternion q0(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(q0);
  mat.getEulerYPR(yaw, pitch, roll);
  tf::Quaternion q;
  q.setEulerZYX(yaw, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", TF_QUAD_NAME));
  if(LOCALFRAME_READY == true){
    // Start publising the local frame
    transform_origin.setOrigin(tf::Vector3(origin_world.pose.position.x,
					   origin_world.pose.position.y,
					   origin_world.pose.position.z));
    tf::Quaternion q1(origin_world.pose.orientation.x,
		      origin_world.pose.orientation.y,
		      origin_world.pose.orientation.z,
		      origin_world.pose.orientation.w);
    tfScalar yaw_origin, pitch_origin, roll_origin;
    tf::Matrix3x3 mat_origin(q1);
    mat_origin.getEulerYPR(yaw_origin, pitch_origin, roll_origin);
    tf::Quaternion q_origin;
    q_origin.setEulerZYX(0, 0, yaw_origin);
    transform_origin.setRotation(q_origin);
    br.sendTransform(tf::StampedTransform(transform_origin, ros::Time::now(), "world", TF_ORIGIN_NAME));
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
  float EULER2JOY, YAWRATE2JOY;

  ros::init(argc, argv, "bicopter_optitrack_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  nh.param<std::string>("QUAD_NAME", QUAD_NAME, "jackBi");
  char DEVICE_NAME[100];
  strcpy(DEVICE_NAME, "/vrpn_client_node/");
  strcat(DEVICE_NAME, QUAD_NAME.c_str());
  strcat(DEVICE_NAME, "/pose");
  cout<<DEVICE_NAME<<endl;
  
  strcpy(TF_GOAL_NAME, QUAD_NAME.c_str());
  strcat(TF_GOAL_NAME, "goal");
  strcpy(TF_QUAD_NAME, QUAD_NAME.c_str());
  strcpy(TF_ORIGIN_NAME, QUAD_NAME.c_str());
  strcat(TF_ORIGIN_NAME, "origin");

	
  ros::Rate loop_rate(ros_freq);
  
  ros::Subscriber sub_joy       = n.subscribe("/joy", 1, joystick_callback);
  ros::Subscriber sub_pose       = n.subscribe(DEVICE_NAME, 1, vrpn_pose_callback);
  ros::Subscriber sub_goal       = n.subscribe("goal", 1, goal_callback);
  ros::Subscriber sub_land       = n.subscribe("land", 5, land_callback);
  ros::Subscriber sub_euler      = n.subscribe("goal_angle", 1, goalEuler_callback);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("joy_control", 5);
  ros::Publisher pub_ctrl_error = n.advertise<geometry_msgs::TwistStamped>("error", 5);
  ros::Publisher pub_debug_data = n.advertise<geometry_msgs::Vector3Stamped>("flight_control_debug", 1);
  ros::Publisher yaw_rt = n.advertise<geometry_msgs::Vector3Stamped>("yaw_rt",1);

  nh.param<float>("maxAngle", EULER2JOY, 55.0f);
  //n.getParam("flight_controller/maxAngle", EULER2JOY);
  EULER2JOY = EULER2JOY/180.0*M_PI;
  nh.param<float>("maxYawRate", YAWRATE2JOY, 300.0f);
  //n.getParam("flight_controller/maxYawRate", YAWRATE2JOY);
  YAWRATE2JOY = YAWRATE2JOY/180.0*M_PI;

  // PID for x
  n.getParam("controller/KP_x", KP_x);
  n.getParam("controller/KI_x", KI_x);
  n.getParam("controller/KD_x", KD_x);
  n.getParam("controller/X_limit", X_limit);
  // PID for y
  n.getParam("controller/KP_y", KP_y);
  n.getParam("controller/KI_y", KI_y);
  n.getParam("controller/KD_y", KD_y);
  n.getParam("controller/Y_limit", Y_limit);
  // PID for z
  n.getParam("controller/KP_z", KP_z);
  n.getParam("controller/KI_z", KI_z);
  n.getParam("controller/KD_z", KD_z);
  n.getParam("controller/Z_limit", Z_limit);
  // PID for yaw
  n.getParam("controller/KP_yaw", KP_yaw);
  n.getParam("controller/KI_yaw", KI_yaw);
  n.getParam("controller/KD_yaw", KD_yaw);
  n.getParam("controller/YAW_limit", YAW_limit);

  // Some additional params
  n.getParam("controller/TAKEOFF_HEIGHT", TAKEOFF_HEIGHT);
  n.getParam("controller/TAKEOFF_THROTTLE", TAKEOFF_THROTTLE);
  n.getParam("controller/TAKEOFF_THROTTLE_TIME", TAKEOFF_THROTTLE_TIME);  
  n.getParam("controller/LAND_TIME", LAND_TIME);

  tf::TransformListener listener;
  
  struct timeval tvstart, tvend;
  struct timeval flip_start, flip_end;
  double throttle_inflip = 0.0;

  gettimeofday(&tvstart,NULL);

  // PID controller init
  BasicPID controller_pose_x(KP_x, KI_x, KD_x, true, 30);
  BasicPID controller_pose_y(KP_y, KI_y, KD_y, true, 30);
  BasicPID controller_pose_z(KP_z, KI_z, KD_z, true, 30);
  BasicPID controller_pose_yaw(KP_yaw, KI_yaw, KD_yaw, true, 15);
  controller_pose_x.limitIntegral(X_limit);
  controller_pose_y.limitIntegral(Y_limit);
  controller_pose_z.limitIntegral(Z_limit);
  controller_pose_yaw.limitIntegral(YAW_limit);

  int count = 0;
  int flip_count = 0;
  int pre_flip_button = 1;
  srand((int)time(0));
  bool Start_timer = true, FIRSTFRAME_SKIPPED = false;
  bool Enable_Flip = false;
  float idletime = 0, land_height = 0;

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    // Listen to tf transform in auto mode
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(TF_QUAD_NAME, TF_GOAL_NAME, ros::Time(0), transform);
      TRANSFORM_READY = true;
      tfScalar roll_test, pitch_test, yaw_test;
      tf::Quaternion q_test;
      tf::Matrix3x3(transform.getRotation()).getRPY(roll_test, pitch_test, yaw_test);
      q_test = transform.getRotation();
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
      ROS_INFO_STREAM_THROTTLE(1, "TAKING OFF...");
      // Get the current Z and set it as ground Z (set only once)
      if(flight_mode_previous != AUTO_TAKEOFF){
	// Set local frame origin
	origin_world.pose = pose_world.pose;
	LOCALFRAME_READY = true;
	FIRSTFRAME_SKIPPED = false;
      }
      // Only allow takeoff when joystick is set on arm
      if(joystick_input.buttons[JOY_CHANNEL_ARM] == 0){
	ROS_WARN_THROTTLE(1, "Warning: Waiting for arming command from joy...");
      }else if(joystick_input.axes[JOY_CONTROL_LAND] <= JOY_LAND_NOB_THRESHOLD){
	ROS_WARN_THROTTLE(1, "Warning: Landing mode...");
      }else{
	// Put default arming and flight mode in joystick
	joystick_output.buttons[JOY_CHANNEL_ARM] = 1;
	// Take off (gradually increase throttle to predefiend throttle command)
	if(joystick_output.axes[JOY_CHANNEL_THROTTLE] < TAKEOFF_THROTTLE)
	  joystick_output.axes[JOY_CHANNEL_THROTTLE] += ros_dt/TAKEOFF_THROTTLE_TIME*TAKEOFF_THROTTLE;
	joystick_output.axes[JOY_CHANNEL_PITCH] = joystick_input.axes[JOY_CHANNEL_PITCH];
	joystick_output.axes[JOY_CHANNEL_ROLL]  = joystick_input.axes[JOY_CHANNEL_ROLL];
	joystick_output.axes[JOY_CHANNEL_YAW]   = joystick_input.axes[JOY_CHANNEL_YAW];
	// If the current local Z is greater than TAKEOFF_HEIGHT, finish takeoff process
	if(FIRSTFRAME_SKIPPED && pose_local.pose.position.z > TAKEOFF_HEIGHT){
	  flight_mode = AUTO_FLYING;
	}
	FIRSTFRAME_SKIPPED = true;
      }
      break;
    }
    case AUTO_LAND:{
      ROS_INFO_STREAM_THROTTLE(1, "LANDING...");
      
      // Use current x,y as goal and gradually reduce height
      if(goal_local.pose.position.z >= -0.5)
	goal_local.pose.position.z -= ros_dt/LAND_TIME*land_height;
      // Gradually decreases the throttle at low altitude
      if(pose_local.pose.position.z <= origin_world.pose.position.z + 0.05f){
        TAKEOFF_THROTTLE -= 0.001f;
      }
      // IDLE motor after land
      if(pose_local.pose.position.z <= TAKEOFF_HEIGHT+0.02f){
	flight_mode = AUTO_IDLE;
	idletime = totaltime;
      }
      // Intential fall-through
    }
    case AUTO_FLYING:{
      ROS_INFO_STREAM_THROTTLE(5, "FLYING!");
      // Implementing main controller
      double roll_sp, pitch_sp, yaw_sp, throttle_sp;
      tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);

      // Publish yaw debug data
      yaw_debug_data.header.stamp = ros::Time::now();
      yaw_debug_data.header.seq = count;
      yaw_debug_data.header.frame_id = "yaw_debug_data";
      yaw_debug_data.vector.x = 0;
      yaw_debug_data.vector.y = 0;
      yaw_debug_data.vector.z = yaw;

      // yaw correction
      if(flip_count % 2 == 1){
        yaw -= PI;
      }
      if(yaw < -PI)
        yaw += 2*PI;

      //Publish yaw debug data
      yaw_debug_data.vector.x = yaw;
      yaw_rt.publish(yaw_debug_data);
      // Only update the PID controller when there's new VRPN data. This is to eliminate the problem when VRPN is not syncing with local ROS.
      if(VRPN_UPDATE){
        controller_pose_x.update(saturate(transform.getOrigin().x(),  -2, 2), 0, dynamic_DT);
        controller_pose_y.update(saturate(-transform.getOrigin().y(), -2, 2), 0, dynamic_DT);
        controller_pose_z.update(saturate(transform.getOrigin().z(),  -1.2, 1.2), 0, dynamic_DT);
        controller_pose_yaw.update(saturate(-yaw, -M_PI, M_PI), 0, dynamic_DT);
        VRPN_UPDATE = false;
      }

      roll_sp     = flip_count % 2 == 0 ? saturate(controller_pose_y.output, -0.6, 0.6) + goal_euler.x/EULER2JOY : -saturate(controller_pose_y.output, -0.6, 0.6) - goal_euler.x/EULER2JOY;
      pitch_sp    = flip_count % 2 == 0 ? saturate(controller_pose_x.output, -0.6, 0.6) - goal_euler.y/EULER2JOY : -saturate(controller_pose_x.output, -0.6, 0.6) + goal_euler.y/EULER2JOY ;        
      yaw_sp      = joystick_input.axes[JOY_CHANNEL_YAW];//saturate(controller_pose_yaw.output, -0.6, 0.6) + goal_euler.z/YAWRATE2JOY;//
      throttle_sp = saturate(controller_pose_z.output, -0.05, 0.4) + TAKEOFF_THROTTLE;
      
      joystick_output = joystick_input;
      joystick_output.buttons[JOY_CHANNEL_FLIGHT_MODE]=dflag;
      joystick_output.axes[JOY_CHANNEL_PITCH]    = saturate(pitch_sp,   -1, 1);
      joystick_output.axes[JOY_CHANNEL_ROLL]     = saturate(roll_sp,    -1, 1);
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_sp, 0, 1);
      joystick_output.axes[JOY_CHANNEL_YAW]      = saturate(yaw_sp,     -1, 1);

      // Swtich to land mode when triggered by the nob
      if(joystick_input.axes[JOY_CONTROL_LAND] <= JOY_LAND_NOB_THRESHOLD){
        flight_mode = AUTO_LAND;
        // Record the first land height for accurate land timing
        land_height = pose_local.pose.position.z;
      }

      // Switch to flip mode when button changes (cannot flip in 1st autoflying loop)
      if(joystick_input.buttons[JOY_CHANNEL_FLIGHT_MODE]){
        flight_mode = DOCK;
      }
      

      // Publish error
      ctrl_error.header.stamp = ros::Time::now();
      ctrl_error.header.seq = count;
      ctrl_error.twist.linear.x = transform.getOrigin().x();
      ctrl_error.twist.linear.y = transform.getOrigin().y();
      ctrl_error.twist.linear.z = transform.getOrigin().z();
      ctrl_error.twist.angular.z = yaw;
      pub_ctrl_error.publish(ctrl_error);
      break;
      }
    
    case DOCK:{
      double roll_sp, pitch_sp, yaw_sp, throttle_sp;
      roll_sp     = joystick_input.axes[JOY_CHANNEL_ROLL]; //flip_count % 2 == 0 ? saturate(controller_pose_y.output, -0.6, 0.6) + goal_euler.x/EULER2JOY : -saturate(controller_pose_y.output, -0.6, 0.6) - goal_euler.x/EULER2JOY;
      pitch_sp    = joystick_input.axes[JOY_CHANNEL_PITCH]; //flip_count % 2 == 0 ? saturate(controller_pose_x.output, -0.6, 0.6) - goal_euler.y/EULER2JOY : -saturate(controller_pose_x.output, -0.6, 0.6) + goal_euler.y/EULER2JOY ;        
      yaw_sp      = joystick_input.axes[JOY_CHANNEL_YAW];//saturate(controller_pose_yaw.output, -0.6, 0.6) + goal_euler.z/YAWRATE2JOY;
      throttle_sp = saturate(controller_pose_z.output, -0.05, 0.4) + TAKEOFF_THROTTLE;
      
      joystick_output = joystick_input;
      joystick_output.axes[JOY_CHANNEL_PITCH]    = saturate(pitch_sp,   -1, 1);//manual controll 
      joystick_output.axes[JOY_CHANNEL_ROLL]     = saturate(roll_sp,    -1, 1);
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_sp, 0, 1);
      joystick_output.axes[JOY_CHANNEL_YAW]      = saturate(yaw_sp,     -1, 1);

      if(!joystick_input.buttons[JOY_CHANNEL_FLIGHT_MODE]){
      flight_mode = AUTO_FLYING;
      dflag=1;
    }
    break;
    }

    case AUTO_FLIP:{
      ROS_INFO_STREAM_THROTTLE(1, "FLIPING...");
      joystick_output = joystick_input;
      joystick_output.axes[JOY_CHANNEL_PITCH]    = 0;
      joystick_output.axes[JOY_CHANNEL_ROLL]     = 0;
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_inflip*1.05, 0, 1);
      joystick_output.axes[JOY_CHANNEL_YAW]      = 0;

      // Switch to flying mode when time is off
      gettimeofday(&flip_end,NULL);
      double fliptime = flip_end.tv_sec - flip_start.tv_sec + 1e-6 * (flip_end.tv_usec - flip_start.tv_usec);
      if((flip_count % 2 == 0 && fliptime <= ASCENDTIME_FORWARD) || (flip_count % 2 == 1 && fliptime <= ASCENDTIME_BACK)){
	joystick_output.buttons[JOY_CHANNEL_FLIGHT_MODE] = pre_flip_button;
      }

      if((flip_count % 2 == 0 && fliptime > ASCENDTIME_FORWARD) || (flip_count % 2 == 1 && fliptime > ASCENDTIME_BACK)){
        joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_inflip, 0, 1);
      }
      
      if((flip_count % 2 == 0 && fliptime > FLIPTIME + ASCENDTIME_FORWARD) || (flip_count % 2 == 1 && fliptime > FLIPTIME + ASCENDTIME_BACK)){
        flight_mode = AUTO_FLYING;
	pre_flip_button = joystick_input.buttons[JOY_CHANNEL_FLIGHT_MODE];
        flip_count++;
        // Make sure to reset the controller to avoid filtering the sudden change in error and derror
        controller_pose_x.controllerFirstRun = true;
        controller_pose_y.controllerFirstRun = true;
        controller_pose_z.controllerFirstRun = true;
        controller_pose_yaw.controllerFirstRun = true;

      }
      break;
  }
    case AUTO_IDLE:{
      ROS_INFO_STREAM_THROTTLE(5, "IDLE...");
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = 0;
      // Disarm after 3 sec in idle
      if(totaltime - idletime > 3.0)
	joystick_output.axes[JOY_CHANNEL_ARM] = 0;
      break;
    }
    }
    ros::param::set("/controller/Flip", flip_count);

    // Publish debug data
    debug_data.header.stamp = ros::Time::now();
    debug_data.header.seq = count;
    debug_data.header.frame_id = "debug_data";
    debug_data.vector.x = controller_pose_z.error;
    debug_data.vector.y = controller_pose_z.derror;
    if(JOY_READY)
      debug_data.vector.z = joystick_output.axes[JOY_CHANNEL_THROTTLE];
    pub_debug_data.publish(debug_data);
      

    flight_mode_previous = flight_mode;

    joystick_output.header.stamp = ros::Time::now();
    joystick_output.header.seq = count;
    joystick_output.header.frame_id = "joystick_output";
    if(JOY_READY == true)
      pub_joy_output.publish(joystick_output);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}

