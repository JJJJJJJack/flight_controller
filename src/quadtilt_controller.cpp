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
//////////////////////////////
#define JOY_CHANNEL_TILT 4
// Joy stick nob land threshold, Land signal will only be triggered at value lower
#define JOY_LAND_NOB_THRESHOLD -0.9f

using namespace std;

typedef enum FLIGHT_MODE
  {
   PASS_THROUGH = 0,
   AUTO_TAKEOFF,
   AUTO_LAND,
   AUTO_FLYING,
   AUTO_IDLE,
  } FLIGHT_MODE;


tfScalar debughuz = 0;//1118



FLIGHT_MODE flight_mode = PASS_THROUGH, flight_mode_previous = PASS_THROUGH;
geometry_msgs::Twist cmd_vel, ctrl_error;
string QUAD_NAME;
char TF_GOAL_NAME[100], TF_QUAD_NAME[100], TF_ORIGIN_NAME[100];
geometry_msgs::PoseStamped pose_world, goal_world, origin_world, pose_local, goal_local;
geometry_msgs::Vector3 goal_euler;
geometry_msgs::Vector3Stamped debug_data;
sensor_msgs::Joy joystick_input, joystick_output;
bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = false, GOAL_EULER_READY = false;
bool JOYSTICK_LAND = false;
float tilt_cmd = 0;

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
geometry_msgs::Twist keyboard_cmd_vel; // 存储键盘输入的 cmd_vel
bool KEYBOARD_READY = false; 
float KEYBOARD_TILT_SCALE = 0.02; // 键盘控制倾斜角系数
int keyboard_count = 0;
const int SCALE_CHANGE_THRESHOLD = 15;
const float KEYBOARD_TILT_SCALE_REDUCED = 0.01;

void keyboard_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    KEYBOARD_READY = true;
    keyboard_cmd_vel= *msg;

if (fabs(keyboard_cmd_vel.linear.x) > 0.01) { // 有效输入才计数
      keyboard_count++;
      
      if (keyboard_count >= SCALE_CHANGE_THRESHOLD) {
          KEYBOARD_TILT_SCALE = KEYBOARD_TILT_SCALE_REDUCED;
      }
    
      tilt_cmd += keyboard_cmd_vel.linear.x * KEYBOARD_TILT_SCALE;}
    
    
}//

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
  if(joystick_input.axes[JOY_CONTROL_LAND] <= JOY_LAND_NOB_THRESHOLD){
    JOYSTICK_LAND = true;
  }else{
    JOYSTICK_LAND = false;
  }
  joystick_input.axes[JOY_CONTROL_LAND] = 0;
  if(joystick_input.axes[JOY_CONTROL_TYPE] == 0){
    flight_mode = PASS_THROUGH;
  }else if(flight_mode == PASS_THROUGH)
    if(TRANSFORM_READY == true && message.axes[JOY_CONTROL_LAND] > JOY_LAND_NOB_THRESHOLD){
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
    goal_local = goal_world;
  }
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(goal_local.pose.position.x,
				  -goal_local.pose.position.y,
          goal_local.pose.position.z));
				  // goal_local.pose.position.z));//1105
  tf::Quaternion q(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  tfScalar yaw_goal, pitch_goal, roll_goal;
  tf::Matrix3x3 mat_goal(q);
  //mat_goal.getEulerYPR(yaw_goal, pitch_goal, roll_goal);

        double r_goal00, r_goal01, r_goal02;
        double r_goal10, r_goal11, r_goal12;
        double r_goal20, r_goal21, r_goal22;

        r_goal00 = mat_goal[0][0]; r_goal01=mat_goal[0][1];r_goal02=mat_goal[0][2];
        r_goal10 = mat_goal[1][0]; r_goal11=mat_goal[1][1];r_goal12=mat_goal[1][2];
        r_goal20 = mat_goal[2][0]; r_goal11=mat_goal[2][1];r_goal22=mat_goal[2][2]; 
          yaw_goal = atan2(-r_goal01, r_goal11);  // Z 轴旋转
          roll_goal = asin(r_goal21);        // X 轴旋转
          pitch_goal= atan2(-r_goal20, r_goal22); // Y 轴
  tf::Quaternion q_goal;
  q_goal.setRPY(0, 0, -yaw_goal);
  transform.setRotation(q_goal);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", TF_GOAL_NAME));
}

void land_callback(const std_msgs::Int8& message){
  // Accept only land mode
  if(message.data == 1 && flight_mode != AUTO_IDLE && flight_mode != PASS_THROUGH)
    flight_mode = AUTO_LAND;
}

void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
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
  //mat.getEulerYPR(yaw, pitch, roll);
       double rw00, rw01, rw02;
        double rw10, rw11, rw12;
        double rw20, rw21, rw22;  
        rw00 = mat[0][0]; rw01 = mat[0][1]; rw02 = mat[0][2];
        rw10 = mat[1][0]; rw11 = mat[1][1]; rw12 = mat[1][2];
        rw20 = mat[2][0]; rw21 = mat[2][1]; rw22 = mat[2][2]; 
          
          yaw = atan2(-rw01, rw11);  // Z 轴旋转
          roll = asin(rw21);        // X 轴旋转
          pitch = atan2(-rw20, rw22); // Y 轴旋转
  //ROS_INFO_THROTTLE(1.0, "Yaw: %.4f rad (%.2f deg)", yaw, yaw * 180.0 / M_PI);//1118
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
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
   // mat.getEulerYPR(yaw_origin, pitch_origin, roll_origin);
        double r_origin00, r_origin01, r_origin02;
        double r_origin10, r_origin11, r_origin12;
        double r_origin20, r_origin21, r_origin22;

        r_origin00 = mat_origin[0][0]; r_origin01 = mat_origin[0][1]; r_origin02 = mat_origin[0][2];
        r_origin10 = mat_origin[1][0]; r_origin11 = mat_origin[1][1]; r_origin12 = mat_origin[1][2];
        r_origin20 = mat_origin[2][0]; r_origin21 = mat_origin[2][1]; r_origin22 = mat_origin[2][2];
          
        yaw_origin = atan2(-r_origin01, r_origin11);  // Z 轴旋转
        roll_origin = asin(r_origin21);        // X 轴旋转
        pitch_origin = atan2(-r_origin20, r_origin22); // Y 轴旋转//1116
    tf::Quaternion q_origin;
    q_origin.setRPY(0, 0, yaw_origin);
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
  nh.param<std::string>("QUAD_NAME", QUAD_NAME, "quadTilt");
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
  ros::Subscriber sub_pose       = n.subscribe(DEVICE_NAME, 2, vrpn_pose_callback);
  ros::Subscriber sub_goal       = n.subscribe("goal", 1, goal_callback);
  ros::Subscriber sub_land       = n.subscribe("land", 5, land_callback);
  ros::Subscriber sub_euler      = n.subscribe("goal_angle", 1, goalEuler_callback);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("joy_control", 5);
  ros::Publisher pub_ctrl_error = n.advertise<geometry_msgs::Twist>("error", 5);
  ros::Publisher pub_debug_data = n.advertise<geometry_msgs::Vector3Stamped>("flight_control_debug", 1);



 //订阅 /cmd_vel 话题
 ros::Subscriber sub_keyboard = n.subscribe("/turtle1/cmd_vel", 1, keyboard_callback);;


  nh.param<float>("maxAngle", EULER2JOY, 55.0f);
  //ros::param::get("/flight_controller/maxAngle", EULER2JOY);
  EULER2JOY = EULER2JOY/180.0*M_PI;
  nh.param<float>("maxYawRate", YAWRATE2JOY, 300.0f);
  //ros::param::get("/flight_controller/maxYawRate", YAWRATE2JOY);
  YAWRATE2JOY = YAWRATE2JOY/180.0*M_PI;

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
  BasicPID controller_pose_x(KP_x, KI_x, KD_x, true, 30);
  BasicPID controller_pose_y(KP_y, KI_y, KD_y, true, 30);
  BasicPID controller_pose_z(KP_z, KI_z, KD_z, true, 30);
  BasicPID controller_pose_yaw(KP_yaw, KI_yaw, KD_yaw, true, 15);
  controller_pose_x.limitIntegral(X_limit);
  controller_pose_y.limitIntegral(Y_limit);
  controller_pose_z.limitIntegral(Z_limit);
  controller_pose_yaw.limitIntegral(YAW_limit);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true, FIRSTFRAME_SKIPPED = false;
  float idletime = 0, land_height = 0;;

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    // Listen to tf transform in auto mode
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(TF_QUAD_NAME, TF_GOAL_NAME, ros::Time(0), transform);
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
      /////////
      if (KEYBOARD_READY) {
        joystick_output.axes[JOY_CHANNEL_TILT] = saturate(tilt_cmd, 0, 1);
      }
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
      }else if(JOYSTICK_LAND){
	ROS_WARN_THROTTLE(1, "Warning: Landing mode...");
      }else{
	// Put default arming and flight mode in joystick
	joystick_output.buttons[JOY_CHANNEL_ARM] = 1;
	joystick_output.buttons[JOY_CHANNEL_FLIGHT_MODE] = 1;
	// Take off (gradually increase throttle to predefiend throttle command)
	if(joystick_output.axes[JOY_CHANNEL_THROTTLE] < TAKEOFF_THROTTLE)
	  joystick_output.axes[JOY_CHANNEL_THROTTLE] += ros_dt/TAKEOFF_THROTTLE_TIME*TAKEOFF_THROTTLE;
	joystick_output.axes[JOY_CHANNEL_PITCH] = joystick_input.axes[JOY_CHANNEL_PITCH];
 //
  joystick_output.axes[JOY_CHANNEL_TILT]  = 0;///
  ////////////////////////
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
      // IDLE motor after land
      if(pose_local.pose.position.z <= TAKEOFF_HEIGHT / 1.2){
	flight_mode = AUTO_IDLE;
	idletime = totaltime;
      }
      // Intential fall-through
    }
    case AUTO_FLYING:{
      ROS_INFO_STREAM_THROTTLE(5, "FLYING!");
      // Implementing main controller
      double roll_sp, pitch_sp, yaw_sp, throttle_sp,tilt;
      // tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
       tf::Matrix3x3 mat1(transform.getRotation());
        double r00, r01, r02;
        double r10, r11, r12;
        double r20, r21, r22;
        r00 = mat1[0][0]; r01 = mat1[0][1]; r02 = mat1[0][2];
        r10 = mat1[1][0]; r11 = mat1[1][1]; r12 = mat1[1][2];
        r20 = mat1[2][0]; r21 = mat1[2][1]; r22 = mat1[2][2];

          // 按 ZXY 顺序计算欧拉角
          yaw = atan2(-r01, r11);  // Z 轴旋转
          roll = asin(r21);        // X 轴旋转
          pitch = atan2(-r20, r22); // Y 轴旋转
     //  ROS_INFO_THROTTLE(0.5, "Flying Yaw: %.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);//1118
          controller_pose_x.update(saturate(transform.getOrigin().x(),  -2, 2), 0, ros_dt);
          controller_pose_y.update(saturate(-transform.getOrigin().y(), -2, 2), 0, ros_dt);
          controller_pose_z.update(saturate(transform.getOrigin().z(),  -2, 2), 0, ros_dt);
          controller_pose_yaw.update(saturate(-yaw, -M_PI, M_PI), 0, ros_dt);
     
          roll_sp     = saturate(controller_pose_y.output, -0.6, 0.6) + goal_euler.x/EULER2JOY;
          pitch_sp    = saturate(controller_pose_x.output, -0.6, 0.6) - goal_euler.y/EULER2JOY;
          yaw_sp      = saturate(controller_pose_yaw.output, -0.6, 0.6) + goal_euler.z/YAWRATE2JOY;
          throttle_sp = saturate(controller_pose_z.output, -0.6, 0.6) + TAKEOFF_THROTTLE;

          debughuz=yaw_sp;//1118
      // roll_sp     = saturate(controller_pose_y.output, -0.6, 0.6);
      // pitch_sp    = saturate(controller_pose_x.output, -0.6, 0.6) ;
      // yaw_sp      = saturate(controller_pose_yaw.output, -0.6, 0.6) ;
      // throttle_sp = saturate(controller_pose_z.output, -0.6, 0.6) + TAKEOFF_THROTTLE;


      joystick_output = joystick_input;
      ////////
      if (KEYBOARD_READY) {
        joystick_output.axes[JOY_CHANNEL_TILT]  = saturate(tilt_cmd, 0, 1);
      }  
     /*joystick_output.axes[JOY_CHANNEL_TILT] =saturate(tilt_cmd,   -1, 1);*/



      // float tilt_angle=tilt_cmd*M_PI/4;
      // float tilt_compensation=sin(tilt_angle)*0.5;
      // throttle_sp=saturate(controller_pose_z.output+tilt_compensation, -0.8, 0.8) + TAKEOFF_THROTTLE;
        throttle_sp=saturate(controller_pose_z.output, -0.8, 0.8) + TAKEOFF_THROTTLE;
      /////////
      joystick_output.axes[JOY_CHANNEL_PITCH]    = saturate(pitch_sp,   -1, 1);
      joystick_output.axes[JOY_CHANNEL_ROLL]     = saturate(roll_sp,    -1, 1);
      joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_sp, 0, 0.8);
      joystick_output.axes[JOY_CHANNEL_YAW]      = saturate(yaw_sp,     -1, 1);
      //joystick_output.axes[JOY_CHANNEL_YAW]   = joystick_input.axes[JOY_CHANNEL_YAW];
      // 
      // Swtich to land mode when triggered by the nob
      if(JOYSTICK_LAND){
	flight_mode = AUTO_LAND;
	// Record the first land height for accurate land timing
	land_height = pose_local.pose.position.z;
      }

      // Publish error
      ctrl_error.linear.x = transform.getOrigin().x();
      ctrl_error.linear.y = transform.getOrigin().y();
      ctrl_error.linear.z = transform.getOrigin().z();
      ctrl_error.angular.z = yaw;
      pub_ctrl_error.publish(ctrl_error);
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

    // Publish debug data
    debug_data.header.stamp = ros::Time::now();
    debug_data.header.seq = count;
    debug_data.header.frame_id = "debug_data";
    //debug_data.vector.x = controller_pose_z.error;
    debug_data.vector.x =debughuz;//1118
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