
#include "ros/ros.h"

#include <iostream>
#include <fstream>

#include <time.h>

#include <Eigen/Eigen>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"

#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "BasicPID.h"
#include "mav_planning_msgs/PolynomialTrajectory.h"

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

#define gravityG 9.8f

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
geometry_msgs::Twist cmd_vel, ctrl_error;
string QUAD_NAME;
char TF_GOAL_NAME[100], TF_QUAD_NAME[100], TF_ORIGIN_NAME[100];
geometry_msgs::PoseStamped pose_world, origin_world, pose_local;
sensor_msgs::Joy joystick_input, joystick_output;
bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = true, GOAL_EULER_READY = false;

// Global Variables
geometry_msgs::PoseStamped CurrVel;
sensor_msgs::Imu imuData;
mav_planning_msgs::PolynomialTrajectory poly_coeff;
Eigen::Matrix3d R_N_W_BODY;
double ROS_TIME = 0;
float trajectory_time_debug = 0;

// Global flags
bool POLY_UPDATE = false;
bool POLY_TIME_UPDATE = false;
bool POSE_UPDATE = false;

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


/*********   Callbacks   ***********/
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
        // Only set takeoff from pass through when pose feedback is ready.
        flight_mode = AUTO_TAKEOFF;
    }else{
      ROS_ERROR_THROTTLE(1, "No position feedback yet! Please double check!");
      ROS_ERROR_THROTTLE(1, "Fallback to PASS_THROUGH mode...");
    }
}

void Subscribe_CurrVel(const geometry_msgs::PoseStamped& CurrVel_msg)
{
  CurrVel = CurrVel_msg;
}

void Subscribe_PolyCoeff(const mav_planning_msgs::PolynomialTrajectory& poly_coeff_msg)
{
  if (poly_coeff_msg.segments.empty()){
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  }else{
    ROS_INFO("Trajectory sampler: received %lu waypoints",
             poly_coeff_msg.segments.size());
    POLY_UPDATE = true;
    poly_coeff = poly_coeff_msg;
  }
}

void Subscribe_Imu(const sensor_msgs::Imu& imuData_msg)
{
  imuData = imuData_msg;
}


/*********   Process polynomial   ***********/
Eigen::Vector3d getMinSnapPose(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d pose(0,0,0);
  for(int order = 0; order < poly_order; order++){
    pose(0) += poly_coeff.segments[coeff_segment_base].x[order]*pow(time_in_trajectory, order);
    pose(1) += poly_coeff.segments[coeff_segment_base].y[order]*pow(time_in_trajectory, order);
    pose(2) += poly_coeff.segments[coeff_segment_base].z[order]*pow(time_in_trajectory, order);
    // pose(0) += poly_coeff.data[coeff_segment_base + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
    // pose(1) += poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
    // pose(2) += poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
  }
  return pose;
}

Eigen::Vector3d getMinSnapVel(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d vel(0,0,0);
  for(int order = 0; order < poly_order-1; order++){
    vel(0) += (order + 1)*poly_coeff.segments[coeff_segment_base].x[order + 1]*pow(time_in_trajectory, order);
    vel(1) += (order + 1)*poly_coeff.segments[coeff_segment_base].y[order + 1]*pow(time_in_trajectory, order);
    vel(2) += (order + 1)*poly_coeff.segments[coeff_segment_base].z[order + 1]*pow(time_in_trajectory, order);
    // vel(0) += (poly_order - order - 1)*poly_coeff.data[coeff_segment_base + poly_order*0 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 2);
    // vel(1) += (poly_order - order - 1)*poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 2);
    // vel(2) += (poly_order - order - 1)*poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 2);
  }
  return vel;
}

Eigen::Vector3d getMinSnapAcc(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d acc(0,0,0);
  for(int order = 0; order < poly_order-2; order++){
    acc(0) += (order + 1)*(order + 2)*poly_coeff.segments[coeff_segment_base].x[order + 2]*pow(time_in_trajectory, order);
    acc(1) += (order + 1)*(order + 2)*poly_coeff.segments[coeff_segment_base].y[order + 2]*pow(time_in_trajectory, order);
    acc(2) += (order + 1)*(order + 2)*poly_coeff.segments[coeff_segment_base].z[order + 2]*pow(time_in_trajectory, order);
    // acc(0) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*0 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
    // acc(1) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
    // acc(2) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
  }
  return acc;
}

Eigen::Vector3d getMinSnapJerk(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d jerk(0,0,0);
  for(int order = 0; order < poly_order-3; order++){
    jerk(0) += (order + 1)*(order + 2)*(order + 3)*poly_coeff.segments[coeff_segment_base].x[order + 3]*pow(time_in_trajectory, order);
    jerk(1) += (order + 1)*(order + 2)*(order + 3)*poly_coeff.segments[coeff_segment_base].y[order + 3]*pow(time_in_trajectory, order);
    jerk(2) += (order + 1)*(order + 2)*(order + 3)*poly_coeff.segments[coeff_segment_base].z[order + 3]*pow(time_in_trajectory, order);
    // jerk(0) += (poly_order - order - 1)*(poly_order - order - 2)*(poly_order - order - 3)*poly_coeff.data[coeff_segment_base + poly_order*0 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 4);
    // jerk(1) += (poly_order - order - 1)*(poly_order - order - 2)*(poly_order - order - 3)*poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 4);
    // jerk(2) += (poly_order - order - 1)*(poly_order - order - 2)*(poly_order - order - 3)*poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 4);
  }
  return jerk;
}

double getMinSnapYawRate(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw_rate=0;
  if(!poly_coeff.segments[coeff_segment_base].yaw.empty()){
    for(int order = 0; order < poly_order-1; order++){
      minsnap_yaw_rate += (order + 1)*poly_coeff.segments[coeff_segment_base].yaw[order + 1]*pow(time_in_trajectory, order);
      // minsnap_yaw_rate += (poly_order - order - 1)*poly_coeff.data[coeff_segment_base + poly_order*3 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 2);
    }
  }
  return minsnap_yaw_rate;
}

double getMinSnapYaw(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw=0;
  if(!poly_coeff.segments[coeff_segment_base].yaw.empty()){
    for(int order = 0; order < poly_order; order++){
      minsnap_yaw += poly_coeff.segments[coeff_segment_base].yaw[order]*pow(time_in_trajectory, order);
      // minsnap_yaw += poly_coeff.data[coeff_segment_base + poly_order*3 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
    }
  }
  return minsnap_yaw;
}

Eigen::Matrix3d getR_N_W(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
  Eigen::Vector3d minsnap_acc;
  minsnap_acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
  // calculate frame N from acceration
  Eigen::Vector3d acc_W, z_N_W, x_Y_W, y_N_W, x_N_W, Theta_N;
  acc_W = -minsnap_acc;
  acc_W(2) += 9.8;
  z_N_W = acc_W/acc_W.norm();
  x_Y_W << cos(minsnap_yaw), sin(minsnap_yaw), 0;
  y_N_W = z_N_W.cross(x_Y_W);
  y_N_W.normalize();
  x_N_W = y_N_W.cross(z_N_W);
  Eigen::Matrix3d R_N_W;
  x_N_W.normalize();
  y_N_W.normalize();
  z_N_W.normalize();
  R_N_W.col(0) = x_N_W;
  R_N_W.col(1) = y_N_W;
  R_N_W.col(2) = z_N_W;
  // Get the other rotation matrix
  Eigen::Matrix3d R_N_W_neg;
  y_N_W *= -1;
  x_N_W = y_N_W.cross(z_N_W);
  x_N_W.normalize();
  y_N_W.normalize();
  z_N_W.normalize();
  R_N_W_neg.col(0) = x_N_W;
  R_N_W_neg.col(1) = y_N_W;
  R_N_W_neg.col(2) = z_N_W;
  // Calculate difference between R_N_W, R_W_N_neg, and R_N_W_body
  Eigen::Quaterniond Quat_R_N_W(R_N_W);
  Eigen::Quaterniond Quat_R_N_W_neg(R_N_W_neg);
  Eigen::Quaterniond Quat_body(R_N_W_BODY);
  Eigen::Quaterniond Quat_R_N_W_error = Quat_R_N_W * Quat_body.conjugate(); 
  Eigen::AngleAxisd angleAxisError(Quat_R_N_W_error);
  double error = angleAxisError.angle();
  Eigen::Quaterniond Quat_R_N_W_neg_error = Quat_R_N_W_neg * Quat_body.conjugate();
  Eigen::AngleAxisd angleAxisErrorNeg(Quat_R_N_W_neg_error);
  double error_neg = angleAxisErrorNeg.angle();
  if(error < error_neg){
    return R_N_W;
  }else{
    return R_N_W_neg;
  }
}

Eigen::Quaterniond getMinSnapQuat(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
  Eigen::Vector3d minsnap_acc;
  minsnap_acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
  // calculate frame N from acceration
  Eigen::Vector3d acc_W, z_N_W, x_Y_W, y_N_W, x_N_W, Theta_N;
  acc_W = -minsnap_acc;
  acc_W(2) += 9.8;
  z_N_W = acc_W/acc_W.norm();
  x_Y_W << cos(minsnap_yaw), sin(minsnap_yaw), 0;
  y_N_W = z_N_W.cross(x_Y_W);
  y_N_W.normalize();
  x_N_W = y_N_W.cross(z_N_W);
  Eigen::Matrix3d R_N_W;
  x_N_W.normalize();
  y_N_W.normalize();
  z_N_W.normalize();
  R_N_W.col(0) = x_N_W;
  R_N_W.col(1) = y_N_W;
  R_N_W.col(2) = z_N_W;
  Eigen::Quaterniond Quat_NWU(R_N_W);
  Eigen::AngleAxisd AA_NWU(Quat_NWU);
  Eigen::Vector3d NED_Axis = AA_NWU.axis();
  NED_Axis(1) = -NED_Axis(1);
  NED_Axis(2) = -NED_Axis(2);
  Eigen::AngleAxisd AA_NED(AA_NWU.angle(), NED_Axis);
  Eigen::Quaterniond Quat_NED(AA_NED);
  return Quat_NED;
}

Eigen::Vector3d getTheta_N(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d Theta_N;
  Eigen::Matrix3d R_N_W = getR_N_W(time_in_trajectory, poly_order, coeff_segment_base);
  Theta_N = R_N_W.eulerAngles(0, 1, 2);
  return Theta_N;
}

// FIXME Replace R_N_W by current rotation matrix
Eigen::Vector3d getOmega_N(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
    Eigen::Vector3d acc, jerk, omega_N, acc_antiG;
    Eigen::Matrix3d R_N_W;
    acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
    acc_antiG = -acc;
    acc_antiG(2) += 9.8;
    double f = acc_antiG.norm();
    R_N_W = getR_N_W(time_in_trajectory, poly_order, coeff_segment_base);
    jerk = getMinSnapJerk(time_in_trajectory, poly_order, coeff_segment_base);
    // omega_N = 1/f * Eigen::MatrixXd::Identity(3,3) * R_N_W.inverse() * jerk;
    omega_N(0) = 1/f * R_N_W.col(1).dot(jerk);
    omega_N(1) = -1/f * R_N_W.col(0).dot(jerk);
    omega_N(2) = 0;
    // double temp = omega_N(0);
    // omega_N(0) = omega_N(1);
    // omega_N(1) = -temp;
    // omega_N(2) = 0;
    return omega_N;
}

/*********   Utility functions   ***********/
float saturate(float input, float min, float max){
  if(input < min)
    return min;
  if(input > max)
    return max;
  return input;
}

  void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
  //CONTROLLER_READY = true;
  pose_world = message;
  pose_local = pose_world;

  static tf::TransformBroadcaster br;
  tf::Transform transform, transform_origin;
  transform.setOrigin(tf::Vector3(pose_world.pose.position.x,
				  -pose_world.pose.position.y,
				  -pose_world.pose.position.z));
  tf::Quaternion q0(message.pose.orientation.x,
		   message.pose.orientation.y,
		   message.pose.orientation.z,
		   message.pose.orientation.w);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(q0);
  mat.getEulerYPR(yaw, pitch, roll);
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", TF_QUAD_NAME));
  if(LOCALFRAME_READY == true){
    // Start publising the local frame
    transform_origin.setOrigin(tf::Vector3(origin_world.pose.position.x,
					   -origin_world.pose.position.y,
					   -origin_world.pose.position.z));
    tf::Quaternion q1(origin_world.pose.orientation.x,
		      origin_world.pose.orientation.y,
		      origin_world.pose.orientation.z,
		      origin_world.pose.orientation.w);
    tfScalar yaw_origin, pitch_origin, roll_origin;
    tf::Matrix3x3 mat_origin(q1);
    mat.getEulerYPR(yaw_origin, pitch_origin, roll_origin);
    tf::Quaternion q_origin;
    q_origin.setRPY(roll_origin, pitch_origin, yaw_origin);
    transform_origin.setRotation(q_origin);
    br.sendTransform(tf::StampedTransform(transform_origin, ros::Time::now(), "world", TF_ORIGIN_NAME));
  }
}

void land_callback(const std_msgs::Int8& message){
  // Accept only land mode
  if(message.data == 1 && flight_mode != AUTO_IDLE && flight_mode != PASS_THROUGH)
    flight_mode = AUTO_LAND;
}

void keyin_callback(const geometry_msgs::Twist& message){
  if(message.linear.x > 0.1){
    trajectory_time_debug += 0.01;
  }
  if(message.linear.x < -0.1){
    trajectory_time_debug -= 0.01;
  }
}

/*********   Main function   ***********/
int main(int argc, char **argv)
{
  // Local variables
  float KP_x, KI_x, KD_x;
  float KP_y, KI_y, KD_y;
  float KP_z, KI_z, KD_z;
  float KP_yaw, KI_yaw, KD_yaw;
  float KP_att;
  float X_limit, Y_limit, Z_limit, YAW_limit;
  float VEHICLE_WEIGHT;
  float land_height = 0;
  tfScalar roll, pitch, yaw;
  float ros_dt = 1.0/ros_freq;
  float EULER2JOY, YAWRATE2JOY;
  float TAKEOFF_HEIGHT, TAKEOFF_THROTTLE, TAKEOFF_THROTTLE_TIME, LAND_TIME;
  bool FIRSTFRAME_SKIPPED = false;
  // Trajectory param
  int segment_num = 0, poly_order = 0, current_seg_num = 0;
  double trajectory_start_time = 0, trajectory_end_time = 0, past_seg_time = 0;
  Eigen::Vector3d minsnap_pose, minsnap_vel, minsnap_acc, minsnap_jerk, minsnap_rate;
  double minsnap_yaw, minsnap_yaw_rate;
  Eigen::Quaterniond minsnap_quat;
  Eigen::Matrix3d minsnap_R_N_W;
  // Control variable
  geometry_msgs::TwistStamped cmd_vel;
  std_msgs::Float32 cmd_thrust;

  // Initialize ROS
  ros::init(argc, argv, "tracking_controller_SIL");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(ros_freq);

    

  //param name init
  nh.param<std::string>("QUAD_NAME", QUAD_NAME, "jackQuad");
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
  //param name init
  
  // Subscribers
  ros::Subscriber CurrVel_Sub     = n.subscribe("uav_kf", 5, Subscribe_CurrVel);
  ros::Subscriber IMU_Sub         = n.subscribe("imuData", 5, Subscribe_Imu);
  //ros::Subscriber PolyCoeff_Sub   = n.subscribe("trajectory_generator_node/poly_coeff", 1, Subscribe_PolyCoeff);
  //ros::Subscriber TimeAlloc_Sub   = n.subscribe("trajectory_generator_node/time_alloc", 1, Subscribe_TimeAlloc);
  ros::Subscriber PolyCoeff_Sub   = n.subscribe("trajectory", 1, Subscribe_PolyCoeff);
  ros::Subscriber sub_pose       = n.subscribe(DEVICE_NAME, 2, vrpn_pose_callback);
  ros::Subscriber sub_joy       = n.subscribe("/joy", 1, joystick_callback);
  ros::Subscriber sub_land       = n.subscribe("land", 5, land_callback);
  ros::Subscriber sub_keyin       = n.subscribe("/turtle1/cmd_vel", 5, keyin_callback);
  
 
  // Publishers
  ros::Publisher pub_control_rate   = n.advertise<geometry_msgs::TwistStamped>("/setpoint_attitude/cmd_vel", 1);
  ros::Publisher pub_control_thrust = n.advertise<std_msgs::Float32>("/thrust", 1);
  ros::Publisher pub_flight_mode    = n.advertise<std_msgs::Int16>("/flight_mode", 1);
  ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("joy_control", 5);
  // Debugging publishers
  ros::Publisher pub_path           = n.advertise<nav_msgs::Path>("path",1);
  ros::Publisher pub_minsnap_pose   = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_pose", 1);
  ros::Publisher pub_minsnap_vel    = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_vel", 1);
  ros::Publisher pub_minsnap_acc    = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_acc", 1);
  ros::Publisher pub_minsnap_jerk   = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_jerk", 1);
  ros::Publisher pub_minsnap_rate   = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_rate", 1);
  ros::Publisher pub_minsnap_goal_pose   = n.advertise<geometry_msgs::PoseStamped>("minsnap_goal_pose", 1);

  // Initialize parameters
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

  // PID for attitude
  ros::param::get("/controller/KP_att", KP_att);

  // Some additional params
  ros::param::get("/controller/VEHICLE_WEIGHT", VEHICLE_WEIGHT);
  ros::param::get("/controller/TAKEOFF_HEIGHT", TAKEOFF_HEIGHT);
  ros::param::get("/controller/TAKEOFF_THROTTLE", TAKEOFF_THROTTLE);
  ros::param::get("/controller/TAKEOFF_THROTTLE_TIME", TAKEOFF_THROTTLE_TIME);  
  ros::param::get("/controller/LAND_TIME", LAND_TIME);

  // Put controller gain into matrix
  Eigen::Matrix3d Kp_pose;
  Kp_pose << KP_x, 0, 0,
             0, KP_y, 0,
             0, 0, KP_z;
  // Kp_pose << 0, 0, 0,
  //            0, 0, 0,
  //            0, 0, 0;
  Eigen::Matrix3d Kd_pose;
  Kd_pose << KD_x, 0, 0,
             0, KD_y, 0,
             0, 0, KD_z;
  // Kd_pose << 0, 0, 0,
  //            0, 0, 0,
  //            0, 0, 0;
  Eigen::Matrix3d Ki_pose;
  Ki_pose << KI_x, 0, 0,
             0, KI_y, 0,
             0, 0, KI_z;
  Eigen::Vector3d pose_error_integral;
  pose_error_integral << 0, 0, 0;

  // Timers  
  double start_time = ros::Time::now().toSec();
  double current_time = start_time;
  int count = 0;

  // PID controller init
  /*BasicPID controller_pose_x(KP_x, KI_x, KD_x, true, 30);
  BasicPID controller_pose_y(KP_y, KI_y, KD_y, true, 30);
  BasicPID controller_pose_z(KP_z, KI_z, KD_z, true, 30);
  BasicPID controller_pose_yaw(KP_yaw, KI_yaw, KD_yaw, true, 15);
  controller_pose_x.limitIntegral(X_limit);
  controller_pose_y.limitIntegral(Y_limit);
  controller_pose_z.limitIntegral(Z_limit);
  controller_pose_yaw.limitIntegral(YAW_limit);*/
  float X_INIT, Y_INIT, Z_INIT;
  nh.param<std::float_t>("X_INIT", X_INIT, 0);
  nh.param<std::float_t>("Y_INIT", Y_INIT, 0);
  nh.param<std::float_t>("Z_INIT", Z_INIT, 0);
  minsnap_pose << X_INIT, Y_INIT, -Z_INIT;

  tf::TransformListener listener;
  // Main loop
  while (ros::ok())
  {
    ROS_TIME = ros::Time::now().toSec();
    // ROS_TIME = trajectory_time_debug;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(TF_QUAD_NAME, TF_ORIGIN_NAME, ros::Time(0), transform);
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
        ROS_INFO_STREAM_THROTTLE(1, "TAKING OFF...");
        //cout<<pose_local.pose.position.z<<endl<<endl;
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
          joystick_output.buttons[JOY_CHANNEL_FLIGHT_MODE] = 1;
          // Take off (gradually increase throttle to predefiend throttle command)
          if(joystick_output.axes[JOY_CHANNEL_THROTTLE] < TAKEOFF_THROTTLE)
            joystick_output.axes[JOY_CHANNEL_THROTTLE] += ros_dt/TAKEOFF_THROTTLE_TIME*TAKEOFF_THROTTLE;
          joystick_output.axes[JOY_CHANNEL_PITCH] = joystick_input.axes[JOY_CHANNEL_PITCH];
          joystick_output.axes[JOY_CHANNEL_ROLL]  = joystick_input.axes[JOY_CHANNEL_ROLL];
          joystick_output.axes[JOY_CHANNEL_YAW]   = joystick_input.axes[JOY_CHANNEL_YAW];
          // If the current local Z is greater than TAKEOFF_HEIGHT, finish takeoff process
          if(pose_local.pose.position.z > TAKEOFF_HEIGHT){
            ROS_INFO("Takeoff complete");
            flight_mode = AUTO_FLYING;
          }
          FIRSTFRAME_SKIPPED = true;
        }
        break;
      }
      case AUTO_LAND:{
        ROS_INFO_STREAM_THROTTLE(1, "LANDING...");
        
        // Use current x,y as goal and gradually reduce height
        if(minsnap_pose(2) <= 0.5)
          minsnap_pose(2) += ros_dt/LAND_TIME*land_height;
        // Gradually decreases the throttle at low altitude
        if(pose_local.pose.position.z <= origin_world.pose.position.z + 0.05f){
          VEHICLE_WEIGHT -= 0.001f;
        }
        // IDLE motor after land
        if(pose_local.pose.position.z <= TAKEOFF_HEIGHT+0.02f){
          flight_mode = AUTO_IDLE;
        }
        // Intential fall-through
      }

      case AUTO_FLYING:
      {
        current_time = ros::Time::now().toSec();
        
        // Processing polynomial
        // Check polynomial updates
        if(POLY_UPDATE){
        // If new poly, set new poly, set new time
          segment_num = poly_coeff.segments.size();
          poly_order = poly_coeff.segments[0].num_coeffs;
          // Reset total trajectory time
          trajectory_start_time = ROS_TIME;
          trajectory_end_time = trajectory_start_time;
          for(int i = 0; i < segment_num; i++){
            trajectory_end_time += poly_coeff.segments[i].segment_time.toSec();
          }
          past_seg_time = trajectory_start_time;
          current_seg_num = 0;
          POLY_UPDATE = false;
          POLY_TIME_UPDATE = false;
          ROS_INFO("Tracking trajectories");
          // Publish path to rviz, Just for debugging
          nav_msgs::Path path;
          path.header.stamp = ros::Time::now();
          path.header.frame_id = "world";
          int traj_time_num = 500;
          for(int i = 0; i < segment_num; i++){
            float traj_time = poly_coeff.segments[i].segment_time.toSec();
            for(int traj_time_unify = 0; traj_time_unify < traj_time_num; traj_time_unify++){
              float time_in_trajectory = (float)traj_time_unify/traj_time_num*traj_time;
              geometry_msgs::PoseStamped pose;
              pose.header.stamp = ros::Time::now();
              pose.header.frame_id = "world";
              // Convert from NED to NWU
              pose.pose.position.x = getMinSnapPose(time_in_trajectory, poly_order, i).x();
              pose.pose.position.y = -getMinSnapPose(time_in_trajectory, poly_order, i).y();
              pose.pose.position.z = -getMinSnapPose(time_in_trajectory, poly_order, i).z();
              pose.pose.orientation.x = getMinSnapQuat(time_in_trajectory, poly_order, i).x();
              pose.pose.orientation.y = getMinSnapQuat(time_in_trajectory, poly_order, i).y();
              pose.pose.orientation.z = getMinSnapQuat(time_in_trajectory, poly_order, i).z();
              pose.pose.orientation.w = getMinSnapQuat(time_in_trajectory, poly_order, i).w();
              path.poses.push_back(pose);
              ROS_INFO("Segment %d, time %f, pose %f %f %f, quat %f %f %f %f", i, time_in_trajectory, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
            }
          }
          pub_path.publish(path);
        }
        // If poly not finished, update goal_pose, goal_vel, goal_acc, goal_quat, goal_rate   
        if(ROS_TIME < trajectory_end_time){
          // Find which segment we are currently in
          double time_in_trajectory = ROS_TIME - past_seg_time;
          // If current time exceed the current segment
          if(time_in_trajectory > poly_coeff.segments[current_seg_num].segment_time.toSec()){
            // Accumulate segment time
            past_seg_time += poly_coeff.segments[current_seg_num].segment_time.toSec();
            time_in_trajectory -= poly_coeff.segments[current_seg_num].segment_time.toSec();
            current_seg_num += 1;
          }
          // Update feedforward terms
          int coeff_segment_base = current_seg_num;
          // Pose
          minsnap_pose = getMinSnapPose(time_in_trajectory, poly_order, coeff_segment_base);
          // Vel
          minsnap_vel = getMinSnapVel(time_in_trajectory, poly_order, coeff_segment_base);
          // Acc
          minsnap_acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
          // R_N_W
          minsnap_R_N_W = getR_N_W(time_in_trajectory, poly_order, coeff_segment_base);
          // Quat
          minsnap_quat = getMinSnapQuat(time_in_trajectory, poly_order, coeff_segment_base);
          // Jerk
          minsnap_jerk = getMinSnapJerk(time_in_trajectory, poly_order, coeff_segment_base);
          // Yaw
          minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
          minsnap_yaw_rate = getMinSnapYawRate(time_in_trajectory, poly_order, coeff_segment_base);
          // Rate
          minsnap_rate = getOmega_N(time_in_trajectory, poly_order, coeff_segment_base);

          geometry_msgs::Vector3Stamped pubMinPose;
          pubMinPose.header.stamp = ros::Time::now();
          pubMinPose.header.frame_id = "world";
          pubMinPose.vector.x = minsnap_pose(0);
          pubMinPose.vector.y = minsnap_pose(1);
          pubMinPose.vector.z = minsnap_pose(2);
          pub_minsnap_pose.publish(pubMinPose);
          geometry_msgs::Vector3Stamped pubMinVel;
          pubMinVel.header.stamp = ros::Time::now();
          pubMinVel.header.frame_id = "world";
          pubMinVel.vector.x = minsnap_vel(0);
          pubMinVel.vector.y = minsnap_vel(1);
          pubMinVel.vector.z = minsnap_vel(2);
          pub_minsnap_vel.publish(pubMinVel);
          geometry_msgs::Vector3Stamped pubMinAcc;
          pubMinAcc.header.stamp = ros::Time::now();
          pubMinAcc.header.frame_id = "world";
          pubMinAcc.vector.x = minsnap_acc(0);
          pubMinAcc.vector.y = minsnap_acc(1);
          pubMinAcc.vector.z = minsnap_acc(2);
          pub_minsnap_acc.publish(pubMinAcc);
          geometry_msgs::Vector3Stamped pubMinJerk;
          pubMinJerk.header.stamp = ros::Time::now();
          pubMinJerk.header.frame_id = "world";
          pubMinJerk.vector.x = minsnap_jerk(0);
          pubMinJerk.vector.y = minsnap_jerk(1);
          pubMinJerk.vector.z = minsnap_jerk(2);
          pub_minsnap_jerk.publish(pubMinJerk);
          geometry_msgs::Vector3Stamped pubMinRate;
          pubMinRate.header.stamp = ros::Time::now();
          pubMinRate.header.frame_id = "world";
          pubMinRate.vector.x = minsnap_rate(0);
          pubMinRate.vector.y = minsnap_rate(1);
          pubMinRate.vector.z = minsnap_rate(2);
          pub_minsnap_rate.publish(pubMinRate);
          geometry_msgs::PoseStamped pubGoalPose;
          pubGoalPose.header.stamp = ros::Time::now();
          pubGoalPose.header.frame_id = "world";
          pubGoalPose.pose.position.x = minsnap_pose(0);
          pubGoalPose.pose.position.y = -minsnap_pose(1);
          pubGoalPose.pose.position.z = -minsnap_pose(2);
          pubGoalPose.pose.orientation.x = minsnap_quat.x();
          pubGoalPose.pose.orientation.y = minsnap_quat.y();
          pubGoalPose.pose.orientation.z = minsnap_quat.z();
          pubGoalPose.pose.orientation.w = minsnap_quat.w();
          pub_minsnap_goal_pose.publish(pubGoalPose);
        }else{
          // If poly finished, fix goal_pose, set others to 0
          minsnap_vel << 0,0,0;
          minsnap_acc << 0,0,0;
          minsnap_jerk << 0,0,0;
          minsnap_rate << 0,0,0;
          minsnap_yaw_rate = 0;
        }
        

        // Position controller
        // Get position error in world frame
        Eigen::Vector3d current_pose(pose_world.pose.position.x, -pose_world.pose.position.y, -pose_world.pose.position.z);
        Eigen::Vector3d pose_error = minsnap_pose - current_pose;
        // cout<<minsnap_pose<<endl<<endl;
        // cout<<current_pose<<endl<<endl;
        // cout<<pose_error<<endl<<endl<<endl;
        // EKF velocity
        pose_error_integral += pose_error/ros_freq;
        Eigen::Vector3d current_vel(CurrVel.pose.orientation.x, CurrVel.pose.orientation.y, CurrVel.pose.orientation.z);
        Eigen::Vector3d vel_error = minsnap_vel - current_vel;
        Eigen::Vector3d F_des = -Kp_pose * pose_error - Kd_pose * vel_error - Ki_pose * pose_error_integral + VEHICLE_WEIGHT*Eigen::Vector3d(0,0,gravityG) - VEHICLE_WEIGHT * minsnap_acc;
        // cout<<"F_des: "<<VEHICLE_WEIGHT<<endl<<F_des<<endl;
        // Formulate attitude feedback
        //adding NEU to NED transform
        Eigen::Quaterniond Quat_Feedback_NWU(
          pose_world.pose.orientation.w,
          pose_world.pose.orientation.x,
          pose_world.pose.orientation.y,
          pose_world.pose.orientation.z
        );
        Eigen::AngleAxisd AA_Feedback_NWU(Quat_Feedback_NWU);
        Eigen::Vector3d NED_Axis = AA_Feedback_NWU.axis();
        NED_Axis(1) = -NED_Axis(1);
        NED_Axis(2) = -NED_Axis(2);
        Eigen::AngleAxisd AA_Feedback_NED(AA_Feedback_NWU.angle(), NED_Axis);
        Eigen::Quaterniond Quat_Feedback_NED(AA_Feedback_NED);
        
        // 将四元数转换为旋转矩阵
        Eigen::Matrix3d R_N_W = Quat_Feedback_NED.toRotationMatrix();
        R_N_W_BODY = R_N_W;
        // Use feedback RNW
        float T_des = saturate(F_des.dot(R_N_W.col(2)), 0, VEHICLE_WEIGHT*9.8f*10.0f);
        
        // cout<<T_des<<endl<<endl;
        // Calculate quaternion and force to compensate position error
        Eigen::Vector3d Z_B_des = F_des/F_des.norm();
        Eigen::Vector3d X_C_des(cos(minsnap_yaw), sin(minsnap_yaw), 0);
        //cout<<"Minsnap Yaw"<<X_C_des<<endl<<endl;;
        Eigen::Vector3d Y_B_des = Z_B_des.cross(X_C_des);
        Y_B_des.normalize();
        Eigen::Vector3d X_B_des = Y_B_des.cross(Z_B_des);
        X_B_des.normalize();
        Eigen::Matrix3d R_des;
        R_des.col(0) = X_B_des;
        R_des.col(1) = Y_B_des;
        R_des.col(2) = Z_B_des;
        Y_B_des = -Y_B_des;
        X_B_des = Y_B_des.cross(Z_B_des);
        X_B_des.normalize();
        Y_B_des.normalize();
        Z_B_des.normalize();
        Eigen::Matrix3d R_des_neg;
        R_des_neg.col(0) = X_B_des;
        R_des_neg.col(1) = Y_B_des;
        R_des_neg.col(2) = Z_B_des;
        Eigen::Quaterniond Quat_des(R_des);
        Eigen::Quaterniond Quat_des_neg(R_des_neg);
        Eigen::AngleAxisd angleAxisError(Quat_des * Quat_Feedback_NED.conjugate());
        double angleError = fabs(angleAxisError.angle());
        Eigen::AngleAxisd angleAxisErrorNeg(Quat_des_neg * Quat_Feedback_NED.conjugate());
        double angleErrorNeg = fabs(angleAxisErrorNeg.angle());
        Eigen::Quaterniond Quat_Contr;
        if(angleError < angleErrorNeg){
          Quat_Contr = Quat_des;
        }else{
          Quat_Contr = Quat_des_neg;
        }

        // 转换为旋转矩阵
        Eigen::Matrix3d rot_matrix = Quat_Contr.toRotationMatrix();

        // 提取欧拉角（Z-Y-X顺序，即Yaw-Pitch-Roll）
        Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(2, 1, 0);

        // Attitude controller
        // Get desired attitude = goal_quat + Quat_Contr
        Eigen::Quaterniond Quat_error = Quat_Contr * Quat_Feedback_NED.conjugate();
        Eigen::AngleAxisd AA_error(Quat_error);
        // Calculate ang axis as cont_rate
        double rate_Angle = 2.0 * acos(Quat_error.w());
        Eigen::Vector3d rate_Axis = (rate_Angle > 1e-6) ? Quat_error.vec().normalized() : Eigen::Vector3d::UnitX();
        //cout<< rate_Angle<<endl;
        //cout<<rate_Axis<<endl;
        // Attitude controller
        // Eigen::Vector3d omega_Contr = KP_att * rate_Angle * rate_Axis;
        Eigen::Vector3d omega_Contr = KP_att * R_N_W_BODY.inverse() * AA_error.angle() * AA_error.axis();
        // Get desired rate = goal_rate + cont_rate
        Eigen::Vector3d omega_Des = minsnap_rate + omega_Contr;// + minsnap_rate + 
        // Publish rate/Send through SBUS
        cmd_vel.header.seq += 1;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.twist.angular.x = omega_Des(0);
        cmd_vel.twist.angular.y = omega_Des(1);
        cmd_vel.twist.angular.z = omega_Des(2);
        if(omega_Des(0) > 1e2 || omega_Des(1) > 1e2 || omega_Des(2) > 1e2)
          cout<<"Warning! Omega explode"<<endl;
        if(!isnan(omega_Des(0)) && !isnan(omega_Des(1)) & !isnan(omega_Des(2)))
          pub_control_rate.publish(cmd_vel);
        // Publish thrust
        // VEHICLE_WEIGHT * 9.8 = K * TAKEOFF_THROTTLE^2
        // K = VEHICLE_WEIGHT * 9.8 / TAKEOFF_THROTTLE^2
        // Output_force = K * Throttle_command^2
        // Throttle_command = sqrt(T_des/K)
        float throttle_K = VEHICLE_WEIGHT * 9.8f / pow(TAKEOFF_THROTTLE, 2);
        cmd_thrust.data = sqrt(T_des/throttle_K);
        if(!isnan(T_des))
          pub_control_thrust.publish(cmd_thrust);
        // Publish flight mode  
        joystick_output = joystick_input;
        double roll_sp, pitch_sp, yaw_sp, throttle_sp;
        roll_sp = omega_Des(0)*57.3/2000.0f;
        pitch_sp = -omega_Des(1)*57.3/2000.0f;
        yaw_sp = omega_Des(2)*57.3/2000.0f;
        throttle_sp = cmd_thrust.data;
        
        joystick_output.axes[JOY_CHANNEL_ROLL]     = saturate(roll_sp,    -1, 1);
        joystick_output.axes[JOY_CHANNEL_PITCH]    = saturate(pitch_sp,   -1, 1);
        joystick_output.axes[JOY_CHANNEL_YAW]      = saturate(yaw_sp,     -1, 1);
        joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_sp, 0, 1); 
        
        // Swtich to land mode when triggered by the nob
        if(joystick_input.axes[JOY_CONTROL_LAND] <= JOY_LAND_NOB_THRESHOLD){
          flight_mode = AUTO_LAND;
          // Record the first land height for accurate land timing
          land_height = pose_local.pose.position.z;
        }
        break;// Do not run anything without vehicle pose
      }//end of case autoflying
    }//end of case

    if(JOY_READY == true){
      joystick_output.header.seq = count;
      joystick_output.header.stamp = ros::Time::now();
      pub_joy_output.publish(joystick_output);
    }


    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }//end of ros ok
  
  return 0;
}
