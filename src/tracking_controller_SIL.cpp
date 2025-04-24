/**************************
Tracking controller in SIL
For testing minsnap polynomial tracking
Author: Jack He
Date created: 03-20-2025
Topics:
  Subscribe
    Vehicle IMU         "/imuData"  sensor_msgs/Imu
    Vehicle Position    "/pose"     geometry_msgs/PoseStamped
    Vehicle Velocity    "/vel"      geometry_msgs/PoseStamped
    Trajectory coeff    "/trajectory"  mav_planning_msgs/PolynomialTrajectory
  Publish
    Control rate        "/setpoint_attitude/cmd_vel"  geometry_msgs/TwistStamped
    Control thrust      "/thrust"       std_msgs/Float32
    Flight mode         "/flight_mode"  std_msgs/Int16
**************************/

/**************************
 * TODO list
 * EKF velocity
 * Update rate based on IMU
 * 
 * ***********************/

#include "ros/ros.h"

#include <iostream>
#include <fstream>

#include <time.h>

#include <Eigen/Eigen>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/Imu.h"

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

//FLIGHT_MODE flight_mode = PASS_THROUGH, flight_mode_previous = PASS_THROUGH;
geometry_msgs::Twist cmd_vel, ctrl_error;

bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = false, GOAL_EULER_READY = false;

// Global Variables
geometry_msgs::PoseStamped CurrPose, CurrVel;
sensor_msgs::Imu imuData;
mav_planning_msgs::PolynomialTrajectory poly_coeff;

// Global flags
bool POLY_UPDATE = false;
bool POLY_TIME_UPDATE = false;
bool POSE_UPDATE = false;


/*********   Callbacks   ***********/
void Subscribe_CurrPose(const geometry_msgs::PoseStamped& CurrPose_msg)
{
  POSE_UPDATE = true;
  CurrPose = CurrPose_msg;
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
  return R_N_W;
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
    omega_N = 1/f * Eigen::MatrixXd::Identity(3,3) * R_N_W.inverse() * jerk;
    double temp = omega_N(0);
    omega_N(0) = omega_N(1);
    omega_N(1) = -temp;
    omega_N(2) = 0;
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
  tfScalar roll, pitch, yaw;
  float ros_dt = 1.0/ros_freq;
  float EULER2JOY, YAWRATE2JOY;
  // Trajectory param
  int segment_num = 0, poly_order = 0, current_seg_num = 0;
  double trajectory_start_time = 0, trajectory_end_time = 0, past_seg_time = 0;
  Eigen::Vector3d minsnap_pose, minsnap_vel, minsnap_acc, minsnap_jerk, minsnap_rate;
  double minsnap_yaw, minsnap_yaw_rate;
  Eigen::Quaterniond minsnap_quat;
  Eigen::Matrix3d minsnap_R_N_W;
  // Debugging variables
  geometry_msgs::Vector3 pubMinPose, pubMinVel, pubMinAcc, pubMinRate;
  // Control variable
  geometry_msgs::TwistStamped cmd_vel;
  std_msgs::Float32 cmd_thrust;
  std_msgs::Int16 flight_mode;

  // Initialize ROS
  ros::init(argc, argv, "tracking_controller_SIL");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(ros_freq);
  
  // Subscribers
  ros::Subscriber CurrPose_Sub    = n.subscribe("pose", 5, Subscribe_CurrPose);
  ros::Subscriber CurrVel_Sub     = n.subscribe("uav_kf", 5, Subscribe_CurrVel);
  ros::Subscriber IMU_Sub         = n.subscribe("imuData", 5, Subscribe_Imu);
  //ros::Subscriber PolyCoeff_Sub   = n.subscribe("trajectory_generator_node/poly_coeff", 1, Subscribe_PolyCoeff);
  //ros::Subscriber TimeAlloc_Sub   = n.subscribe("trajectory_generator_node/time_alloc", 1, Subscribe_TimeAlloc);
  ros::Subscriber PolyCoeff_Sub   = n.subscribe("trajectory", 1, Subscribe_PolyCoeff);
  

  // Publishers
  ros::Publisher pub_control_rate   = n.advertise<geometry_msgs::TwistStamped>("/setpoint_attitude/cmd_vel", 1);
  ros::Publisher pub_control_thrust = n.advertise<std_msgs::Float32>("/thrust", 1);
  ros::Publisher pub_flight_mode    = n.advertise<std_msgs::Int16>("/flight_mode", 1);

  // Debugging publishers
  ros::Publisher pub_minsnap_pose   = n.advertise<geometry_msgs::Vector3>("minsnap_pose", 1);
  ros::Publisher pub_minsnap_vel    = n.advertise<geometry_msgs::Vector3>("minsnap_vel", 1);
  ros::Publisher pub_minsnap_acc    = n.advertise<geometry_msgs::Vector3>("minsnap_acc", 1);
  ros::Publisher pub_minsnap_rate   = n.advertise<geometry_msgs::Vector3>("minsnap_rate", 1);

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

  // Put controller gain into matrix
  Eigen::Matrix3d Kp_pose;
  Kp_pose << KP_x, 0, 0,
             0, KP_y, 0,
             0, 0, KP_z;
  Eigen::Matrix3d Kd_pose;
  Kd_pose << KD_x, 0, 0,
             0, KD_y, 0,
             0, 0, KD_z;

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

  // Main loop
  while (ros::ok())
  {
    if(POSE_UPDATE){
      current_time = ros::Time::now().toSec();

      // Processing polynomial
      // Check polynomial updates
      if(POLY_UPDATE){
      // If new poly, set new poly, set new time
        segment_num = poly_coeff.segments.size();
        poly_order = poly_coeff.segments[0].num_coeffs;
        // Reset total trajectory time
        trajectory_start_time = ros::Time::now().toSec();
        trajectory_end_time = trajectory_start_time;
        for(int i = 0; i < segment_num; i++){
          trajectory_end_time += poly_coeff.segments[i].segment_time.toSec();
        }
        past_seg_time = trajectory_start_time;
        current_seg_num = 0;
        POLY_UPDATE = false;
        POLY_TIME_UPDATE = false;
        ROS_INFO("Tracking trajectories");
      }
      // If poly not finished, update goal_pose, goal_vel, goal_acc, goal_quat, goal_rate   
      if(ros::Time::now().toSec() < trajectory_end_time){
        // Find which segment we are currently in
        double time_in_trajectory = ros::Time::now().toSec() - past_seg_time;
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
        minsnap_quat = Eigen::Quaterniond(minsnap_R_N_W);
        // Jerk
        minsnap_jerk = getMinSnapJerk(time_in_trajectory, poly_order, coeff_segment_base);
        // Yaw
        minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
        minsnap_yaw_rate = getMinSnapYawRate(time_in_trajectory, poly_order, coeff_segment_base);
        // Rate
        minsnap_rate = getOmega_N(time_in_trajectory, poly_order, coeff_segment_base);
        // Tracking mode
        flight_mode.data = 2;
        // Debugging
        pubMinPose.x = minsnap_pose(0);
        pubMinPose.y = minsnap_pose(1);
        pubMinPose.z = minsnap_pose(2);
        pub_minsnap_pose.publish(pubMinPose);
        pubMinVel.x = minsnap_vel(0);
        pubMinVel.y = minsnap_vel(1);
        pubMinVel.z = minsnap_vel(2);
        pub_minsnap_vel.publish(pubMinVel);
        pubMinAcc.x = minsnap_acc(0);
        pubMinAcc.y = minsnap_acc(1);
        pubMinAcc.z = minsnap_acc(2);
        pub_minsnap_acc.publish(pubMinAcc);
        pubMinRate.x = minsnap_rate(0);
        pubMinRate.y = minsnap_rate(1);
        pubMinRate.z = minsnap_rate(2);
        pub_minsnap_rate.publish(pubMinRate);
      }else{
        // If poly finished, fix goal_pose, set others to 0
        minsnap_vel << 0,0,0;
        minsnap_acc << 0,0,0;
        minsnap_jerk << 0,0,0;
        minsnap_rate << 0,0,0;
        minsnap_yaw_rate = 0;
        flight_mode.data = 2;
      }
      

      // Position controller
      // Get position error in world frame
      Eigen::Vector3d current_pose(CurrPose.pose.position.x, CurrPose.pose.position.y, CurrPose.pose.position.z);
      Eigen::Vector3d pose_error = minsnap_pose - current_pose;
      cout<<minsnap_pose<<endl<<endl;
      cout<<current_pose<<endl<<endl;
      cout<<pose_error<<endl<<endl<<endl;
      // EKF velocity
      Eigen::Vector3d current_vel(CurrVel.pose.orientation.x, CurrVel.pose.orientation.y, CurrVel.pose.orientation.z);
      Eigen::Vector3d vel_error = minsnap_vel - current_vel;
      Eigen::Vector3d F_des = -Kp_pose * pose_error - Kd_pose * vel_error + VEHICLE_WEIGHT*Eigen::Vector3d(0,0,gravityG) - VEHICLE_WEIGHT * minsnap_acc;
      // Formulate attitude feedback
      Eigen::Quaterniond Quat_Feedback(
        imuData.orientation.w,  // w
        imuData.orientation.x,  // x
        imuData.orientation.y,  // y
        imuData.orientation.z   // z
      );
      // 将四元数转换为旋转矩阵
      Eigen::Matrix3d R_N_W = Quat_Feedback.toRotationMatrix();
      // Use feedback RNW
      float T_des = F_des.dot(R_N_W.col(2));
      
      // Calculate quaternion and force to compensate position error
      Eigen::Vector3d Z_B_des = F_des/F_des.norm();
      Eigen::Vector3d X_C_des(cos(minsnap_yaw), sin(minsnap_yaw), 0);
      cout<<"Minsnap Yaw"<<X_C_des<<endl<<endl;;
      Eigen::Vector3d Y_B_des = Z_B_des.cross(X_C_des);
      Y_B_des.normalize();
      Eigen::Vector3d X_B_des = Y_B_des.cross(Z_B_des);
      X_B_des.normalize();
      Eigen::Matrix3d R_des;
      R_des.col(0) = X_B_des;
      R_des.col(1) = Y_B_des;
      R_des.col(2) = Z_B_des;
      cout<<"R_des:"<<endl<<R_des<<endl;
      // Convert to quaternion
      Eigen::Quaterniond Quat_Contr(R_des);

      // 转换为旋转矩阵
      Eigen::Matrix3d rot_matrix = Quat_Contr.toRotationMatrix();

      // 提取欧拉角（Z-Y-X顺序，即Yaw-Pitch-Roll）
      Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(2, 1, 0);

      // Attitude controller
      // Get desired attitude = goal_quat + Quat_Contr
      Eigen::Quaterniond Quat_error = Quat_Contr * Quat_Feedback.conjugate();
      // Calculate ang axis as cont_rate
      double rate_Angle = 2.0 * acos(Quat_error.w());
      Eigen::Vector3d rate_Axis = (rate_Angle > 1e-6) ? Quat_error.vec().normalized() : Eigen::Vector3d::UnitX();
      cout<< rate_Angle<<endl;
      cout<<rate_Axis<<endl;
      // Attitude controller
      Eigen::Vector3d omega_Contr = KP_att * rate_Angle * rate_Axis;
      // Get desired rate = goal_rate + cont_rate
      Eigen::Vector3d omega_Des = minsnap_rate + omega_Contr;// + minsnap_rate + 
      // Publish rate/Send through SBUS
      cmd_vel.header.seq += 1;
      cmd_vel.header.stamp = ros::Time::now();
      cmd_vel.twist.angular.x = omega_Des(0);
      cmd_vel.twist.angular.y = omega_Des(1);
      cmd_vel.twist.angular.z = 0;//omega_Des(2);
      if(omega_Des(0) > 1e2 || omega_Des(1) > 1e2 || omega_Des(2) > 1e2)
        cout<<"Warning! Omega explode"<<endl;
      if(!isnan(omega_Des(0)) && !isnan(omega_Des(1)) & !isnan(omega_Des(2)))
        pub_control_rate.publish(cmd_vel);
      // Publish thrust
      cmd_thrust.data = sqrt(T_des/4.0*2.0)/10.0;
      if(!isnan(T_des))
        pub_control_thrust.publish(cmd_thrust);
      // Publish flight mode
      pub_flight_mode.publish(flight_mode);
    
    }// Do not run anything without vehicle pose

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
