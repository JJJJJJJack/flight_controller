#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

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

#define JOY_CHANNEL_AUX1 4
#define JOY_CHANNEL_AUX2 5
#define JOY_CHANNEL_AUX3 6

// Joy stick nob land threshold
#define JOY_LAND_NOB_THRESHOLD -0.9f
#define gravityG 9.8f

#define MOTOR_P1 4.409f
#define MOTOR_P2 6.888f
#define MOTOR_P3 -0.4763f

using namespace std;

//========================================================================
struct TrajectoryPoint {
    double time;
    double x, z;
    double vx, vz;
    double theta, omega;
    double delta;
    double T_cmd;
    double delta_d;
};

std::vector<TrajectoryPoint> csv_trajectory;
bool CSV_TRAJECTORY_LOADED = false;
bool TRAJECTORY_READY = false;
bool TRAJECTORY_STARTED = false;
double trajectory_start_time = 0;
std::string csv_path;
// ========================================================================

typedef enum FLIGHT_MODE {
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
bool JOY_READY = false, CONTROLLER_READY = false, TRANSFORM_READY = false, LOCALFRAME_READY = false;

// Global Variables
geometry_msgs::PoseStamped CurrVel;
sensor_msgs::Imu imuData;
Eigen::Matrix3d R_N_W_BODY;
double ROS_TIME = 0;

// ================================================
// 键盘输入检测函数
bool kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }

    return false;
}

char getch()
{
    char c;
    system("stty raw");
    c = getchar();
    system("stty cooked");
    return c;
}
// ================================================

void loadCSVTrajectory(const std::string& filename) {
    csv_trajectory.clear(); 
    
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Cannot open CSV file: %s", filename.c_str());
        return;
    }
    
    std::string line;
    
    if (!std::getline(file, line)) {
        ROS_ERROR("Empty CSV file");
        return;
    }
    
    ROS_INFO("CSV Header: %s", line.c_str());
    
    int point_count = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        TrajectoryPoint point;
        
        try {
            std::getline(ss, cell, ','); point.time = std::stod(cell);
            std::getline(ss, cell, ','); point.x = std::stod(cell);
            std::getline(ss, cell, ','); point.z = std::stod(cell);
            std::getline(ss, cell, ','); point.vx = std::stod(cell);
            std::getline(ss, cell, ','); point.vz = std::stod(cell);
            std::getline(ss, cell, ','); point.theta = std::stod(cell);
            std::getline(ss, cell, ','); point.omega = std::stod(cell);
            std::getline(ss, cell, ','); point.delta = std::stod(cell);
            std::getline(ss, cell, ','); point.T_cmd = std::stod(cell);
            std::getline(ss, cell, ','); point.delta_d = std::stod(cell);
            
            csv_trajectory.push_back(point);
            point_count++;
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse CSV line: %s, error: %s", line.c_str(), e.what());
        }
    }
    
    CSV_TRAJECTORY_LOADED = (point_count > 0);
    if (CSV_TRAJECTORY_LOADED) {
        ROS_INFO("Successfully loaded CSV trajectory, total %d points, time range: %.3f ~ %.3f seconds", 
                point_count, csv_trajectory.front().time, csv_trajectory.back().time);
        ROS_INFO("Trajectory duration: %.3f s, control frequency: %.1f Hz", 
                csv_trajectory.back().time, (point_count-1)/csv_trajectory.back().time);
    } else {
        ROS_ERROR("CSV trajectory loading failed, no valid data points");
    }
    file.close();
}

TrajectoryPoint getCSVTrajectoryPoint(double time) {
    if (csv_trajectory.empty() || !CSV_TRAJECTORY_LOADED) {
        ROS_ERROR_THROTTLE(1, "CSV trajectory not loaded!");
        return TrajectoryPoint();
    }
    
    if (time <= csv_trajectory.front().time) {
        return csv_trajectory.front();
    }
    if (time >= csv_trajectory.back().time) {
        return csv_trajectory.back();
    }
    
    size_t left = 0, right = csv_trajectory.size() - 1;
    while (left <= right) {
        size_t mid = left + (right - left) / 2;
        
        if (csv_trajectory[mid].time <= time && 
            (mid == csv_trajectory.size()-1 || csv_trajectory[mid+1].time > time)) {
            
            if (mid == csv_trajectory.size()-1) {
                return csv_trajectory[mid];
            }
            
            double t0 = csv_trajectory[mid].time;
            double t1 = csv_trajectory[mid+1].time;
            double alpha = (time - t0) / (t1 - t0);
            
            TrajectoryPoint result;
            result.time = time;
            result.x = (1-alpha)*csv_trajectory[mid].x + alpha*csv_trajectory[mid+1].x;
            result.z = (1-alpha)*csv_trajectory[mid].z + alpha*csv_trajectory[mid+1].z;
            result.vx = (1-alpha)*csv_trajectory[mid].vx + alpha*csv_trajectory[mid+1].vx;
            result.vz = (1-alpha)*csv_trajectory[mid].vz + alpha*csv_trajectory[mid+1].vz;
            result.theta = (1-alpha)*csv_trajectory[mid].theta + alpha*csv_trajectory[mid+1].theta;
            result.omega = (1-alpha)*csv_trajectory[mid].omega + alpha*csv_trajectory[mid+1].omega;
            result.delta = (1-alpha)*csv_trajectory[mid].delta + alpha*csv_trajectory[mid+1].delta;
            result.T_cmd = (1-alpha)*csv_trajectory[mid].T_cmd + alpha*csv_trajectory[mid+1].T_cmd;
            result.delta_d = (1-alpha)*csv_trajectory[mid].delta_d + alpha*csv_trajectory[mid+1].delta_d;
            
            return result;
        } else if (csv_trajectory[mid].time < time) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return csv_trajectory.back();
}

void resampleTrajectoryForControlFreq(double control_freq) {
    if (!CSV_TRAJECTORY_LOADED || csv_trajectory.empty()) return;
    
    std::vector<TrajectoryPoint> resampled;
    double total_time = csv_trajectory.back().time;
    int new_size = total_time * control_freq + 1;
    
    ROS_INFO("Resampling trajectory from %zu points to %d points for %.1f Hz control", 
             csv_trajectory.size(), new_size, control_freq);
    
    for (int i = 0; i < new_size; i++) {
        double t = i / control_freq;
        resampled.push_back(getCSVTrajectoryPoint(t));
    }
    
    csv_trajectory = resampled;
    ROS_INFO("Resampling completed, new time range: %.3f ~ %.3f seconds", 
             csv_trajectory.front().time, csv_trajectory.back().time);
}

// ==================================================================
// 四元数转换函数 
Eigen::Quaterniond QuatNWUtoQuatNED(Eigen::Quaterniond Quat_NWU)
{
    Eigen::AngleAxisd AA_NWU(Quat_NWU);
    Eigen::Vector3d NWU_Axis = AA_NWU.axis();
    NWU_Axis(1) = -NWU_Axis(1);  
    NWU_Axis(2) = -NWU_Axis(2);  
    Eigen::AngleAxisd AA_NED(AA_NWU.angle(), NWU_Axis);
    Eigen::Quaterniond Quat_NED(AA_NED);
    return Quat_NED;
}

Eigen::Quaterniond QuatNEDtoQuatNWU(Eigen::Quaterniond Quat_NED)
{
    Eigen::AngleAxisd AA_NED(Quat_NED);
    Eigen::Vector3d NED_Axis = AA_NED.axis();
    NED_Axis(1) = -NED_Axis(1);  
    NED_Axis(2) = -NED_Axis(2);  
    Eigen::AngleAxisd AA_NWU(AA_NED.angle(), NED_Axis);
    Eigen::Quaterniond Quat_NWU(AA_NWU);
    return Quat_NWU;
}
// ==================================================================

// 坐标变换函数
geometry_msgs::PoseStamped frame_transform(geometry_msgs::PoseStamped world, geometry_msgs::PoseStamped origin){
    geometry_msgs::PoseStamped local = world;
    local.pose.position.x = world.pose.position.x - origin.pose.position.x;
    local.pose.position.y = world.pose.position.y - origin.pose.position.y;
    local.pose.position.z = world.pose.position.z - origin.pose.position.z;
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
    } else if(flight_mode == PASS_THROUGH) {
        if(TRANSFORM_READY == true && joystick_input.axes[JOY_CONTROL_LAND] > JOY_LAND_NOB_THRESHOLD){
            // Only set takeoff from pass through when pose feedback is ready.
            flight_mode = AUTO_TAKEOFF;
        } else {
            ROS_ERROR_THROTTLE(1, "No position feedback yet! Please double check!");
            ROS_ERROR_THROTTLE(1, "Fallback to PASS_THROUGH mode...");
        }
    }
}

void Subscribe_CurrVel(const geometry_msgs::PoseStamped& CurrVel_msg) {
    CurrVel = CurrVel_msg;
}

void Subscribe_Imu(const sensor_msgs::Imu& imuData_msg) {
    imuData = imuData_msg;
}

void vrpn_pose_callback(const geometry_msgs::PoseStamped& message){
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
        transform_origin.setOrigin(tf::Vector3(origin_world.pose.position.x,
                                              -origin_world.pose.position.y,
                                              -origin_world.pose.position.z));
        tf::Quaternion q1(origin_world.pose.orientation.x,
                         origin_world.pose.orientation.y,
                         origin_world.pose.orientation.z,
                         origin_world.pose.orientation.w);
        tfScalar yaw_origin, pitch_origin, roll_origin;
        tf::Matrix3x3 mat_origin(q1);
        mat_origin.getEulerYPR(yaw_origin, pitch_origin, roll_origin);
        tf::Quaternion q_origin;
        q_origin.setRPY(roll_origin, pitch_origin, yaw_origin);
        transform_origin.setRotation(q_origin);
        br.sendTransform(tf::StampedTransform(transform_origin, ros::Time::now(), "world", TF_ORIGIN_NAME));
    }
}

void land_callback(const std_msgs::Int8& message){
    if(message.data == 1 && flight_mode != AUTO_IDLE && flight_mode != PASS_THROUGH)
        flight_mode = AUTO_LAND;
}

float saturate(float input, float min, float max){
    if(input < min) return min;
    if(input > max) return max;
    return input;
}

float motorCmdFromThrottle(float throttle){
    float motorCmd = (-MOTOR_P2 + sqrt(MOTOR_P2*MOTOR_P2 - 4*MOTOR_P1*(MOTOR_P3 - throttle)))/(MOTOR_P1*2);
    return motorCmd;
}

// 旋转矩阵
Eigen::Matrix3d getNearestRotationMatrix(Eigen::Vector3d z_N_W, double yaw) {
    Eigen::Matrix3d R_N_W;
    Eigen::Vector3d x_Y_W, y_N_W, x_N_W;
    x_Y_W << cos(yaw), sin(yaw), 0;
    y_N_W = z_N_W.cross(x_Y_W);
    y_N_W.normalize();
    x_N_W = y_N_W.cross(z_N_W);
    x_N_W.normalize();
    R_N_W.col(0) = x_N_W;
    R_N_W.col(1) = y_N_W;
    R_N_W.col(2) = z_N_W;
    
    Eigen::Matrix3d R_N_W_neg;
    y_N_W *= -1;
    x_N_W = y_N_W.cross(z_N_W);
    x_N_W.normalize();
    y_N_W.normalize();
    z_N_W.normalize();
    R_N_W_neg.col(0) = x_N_W;
    R_N_W_neg.col(1) = y_N_W;
    R_N_W_neg.col(2) = z_N_W;
    
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
    } else {
        return R_N_W_neg;
    }
}

// 主函数
int main(int argc, char **argv) {
    float KP_x, KI_x, KD_x;
    float KP_y, KI_y, KD_y;
    float KP_z, KI_z, KD_z;
    float KP_yaw, KI_yaw, KD_yaw;
    float KP_att;
    float X_limit, Y_limit, Z_limit, YAW_limit;
    float VEHICLE_WEIGHT;
    float TAKEOFF_HEIGHT, TAKEOFF_THROTTLE, TAKEOFF_THROTTLE_TIME, LAND_TIME;
    bool FIRSTFRAME_SKIPPED = false;
    float ros_dt = 1.0/ros_freq;
    float land_height = 0;
    Eigen::Vector3d minsnap_pose, minsnap_vel, minsnap_acc, minsnap_jerk, minsnap_rate;
    double minsnap_yaw, minsnap_yaw_rate;
    Eigen::Quaterniond minsnap_quat;
    
    geometry_msgs::TwistStamped cmd_vel;
    std_msgs::Float32 cmd_thrust;
    
    ros::init(argc, argv, "bicopter_through_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(ros_freq);
    
    nh.param<std::string>("QUAD_NAME", QUAD_NAME, "bicopter");
    char DEVICE_NAME[100];
    strcpy(DEVICE_NAME, "/vrpn_client_node/");
    strcat(DEVICE_NAME, QUAD_NAME.c_str());
    strcat(DEVICE_NAME, "/pose");
    
    strcpy(TF_GOAL_NAME, QUAD_NAME.c_str());
    strcat(TF_GOAL_NAME, "goal");
    strcpy(TF_QUAD_NAME, QUAD_NAME.c_str());
    strcpy(TF_ORIGIN_NAME, "world");
    strcat(TF_ORIGIN_NAME, "origin");
    
    // ================================================
    nh.param<std::string>("csv_path", csv_path, "/home/xianghe/catkin_ws/src/flight_controller/trajectories/drone_trajectory.csv");
    
    std::ifstream test_file(csv_path.c_str());
    if (!test_file.good()) {
        ROS_ERROR("CSV file does not exist: %s", csv_path.c_str());  
    } else {
        ROS_INFO("CSV file exists and ready to load: %s", csv_path.c_str());  
        test_file.close();
        
        loadCSVTrajectory(csv_path);
        if (CSV_TRAJECTORY_LOADED) {
            resampleTrajectoryForControlFreq(100.0); 
            ROS_INFO("CSV trajectory loaded and resampled successfully");
            TRAJECTORY_READY = true;
        } else {
            ROS_ERROR("CSV trajectory loading failed");
        }
    }
    // ================================================
    
    // 订阅者
    ros::Subscriber CurrVel_Sub = n.subscribe("uav_kf", 5, Subscribe_CurrVel);
    ros::Subscriber IMU_Sub = n.subscribe("imuData", 5, Subscribe_Imu);
    ros::Subscriber sub_pose = n.subscribe(DEVICE_NAME, 2, vrpn_pose_callback);
    ros::Subscriber sub_joy = n.subscribe("/joy", 1, joystick_callback);
    ros::Subscriber sub_land = n.subscribe("land", 5, land_callback);
    
    // 发布者
    ros::Publisher pub_control_rate = n.advertise<geometry_msgs::TwistStamped>("/setpoint_attitude/cmd_vel", 1);
    ros::Publisher pub_control_thrust = n.advertise<std_msgs::Float32>("/thrust", 1);
    ros::Publisher pub_flight_mode = n.advertise<std_msgs::Int16>("/flight_mode", 1);
    ros::Publisher pub_joy_output = n.advertise<sensor_msgs::Joy>("joy_control", 5);
    
    // 调试发布者
    ros::Publisher pub_path = n.advertise<nav_msgs::Path>("path",1);
    ros::Publisher pub_minsnap_pose = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_pose", 1);
    ros::Publisher pub_minsnap_vel = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_vel", 1);
    ros::Publisher pub_minsnap_acc = n.advertise<geometry_msgs::Vector3Stamped>("minsnap_acc", 1);
    
    // 读取控制参数
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
    
    // PID控制矩阵
    Eigen::Matrix3d Kp_pose;
    Kp_pose << KP_x, 0, 0,
               0, KP_y, 0,
               0, 0, KP_z;
    
    Eigen::Matrix3d Kd_pose;
    Kd_pose << KD_x, 0, 0,
               0, KD_y, 0,
               0, 0, KD_z;
    
    Eigen::Matrix3d Ki_pose;
    Ki_pose << KI_x, 0, 0,
               0, KI_y, 0,
               0, 0, KI_z;
    
    Eigen::Vector3d pose_error_integral;
    pose_error_integral << 0, 0, 0;
    
    tf::TransformListener listener;
    
    int count = 0;

    float X_INIT, Y_INIT, Z_INIT;
    nh.param<std::float_t>("X_INIT", X_INIT, 0);
    nh.param<std::float_t>("Y_INIT", Y_INIT, 0);
    nh.param<std::float_t>("Z_INIT", Z_INIT, 0);
    minsnap_pose << X_INIT, Y_INIT, -Z_INIT;
    
    // 显示启动提示
    ROS_INFO("================================================");
    ROS_INFO("Bicopter Trajectory Tracking Controller Started");
    ROS_INFO("================================================");
    ROS_INFO("Operation:");
    ROS_INFO("1. Switch to AUTO mode (JOY_CONTROL_TYPE axis)");
    ROS_INFO("2. Press SPACE key to start trajectory tracking");
    ROS_INFO("================================================");
    
    // 主循环
    while (ros::ok()) {
        ROS_TIME = ros::Time::now().toSec();
        
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("world", TF_QUAD_NAME, ros::Time(0), transform);
            TRANSFORM_READY = true;
        } catch(tf::TransformException &ex) {
            if(flight_mode != PASS_THROUGH) {
                ROS_ERROR_THROTTLE(1, "Transform not ready! Check vrpn...");
                TRANSFORM_READY = false;
            }
        }
   
        if (kbhit()) {
            char c = getch();
            if (c == ' ') {  
                if (flight_mode == AUTO_FLYING && !TRAJECTORY_STARTED) {
                    trajectory_start_time = ROS_TIME;
                    TRAJECTORY_STARTED = true;
                    ROS_INFO("================================================");
                    ROS_INFO("SPACE key pressed! Trajectory tracking STARTED!");
                    ROS_INFO("================================================");
                    nav_msgs::Path path;
                    path.header.stamp = ros::Time::now();
                    path.header.frame_id = "world";
                    int traj_time_num = 1000;
                    float traj_time = 2;
                    for(int traj_time_unify = 0; traj_time_unify < traj_time_num; traj_time_unify++){
                        float time_in_trajectory = (float)traj_time_unify/traj_time_num*traj_time;
                        geometry_msgs::PoseStamped pose;
                        pose.header.stamp = ros::Time::now();
                        pose.header.frame_id = "world";
                        TrajectoryPoint desired_point_ned = getCSVTrajectoryPoint(time_in_trajectory);
                        // NED to NWU for visualization
                        pose.pose.position.x = desired_point_ned.x;
                        pose.pose.position.y = 0;
                        pose.pose.position.z = -desired_point_ned.z;  
                        tf::Quaternion desired_q;
                        desired_q.setRPY(0, desired_point_ned.theta, 0);
                        pose.pose.orientation.x = desired_q.x();
                        pose.pose.orientation.y = desired_q.y();
                        pose.pose.orientation.z = desired_q.z();
                        pose.pose.orientation.w = desired_q.w();
                        path.poses.push_back(pose);
                        if(traj_time_unify % 50 == 0) {
                            ROS_INFO("time %f, pose %f %f %f, quat %f %f %f %f", 
                                time_in_trajectory, 
                                pose.pose.position.x, 
                                pose.pose.position.y, 
                                pose.pose.position.z, 
                                pose.pose.orientation.x, 
                                pose.pose.orientation.y, 
                                pose.pose.orientation.z, 
                                pose.pose.orientation.w);
                        }
                        // ROS_INFO("time %f, pose %f %f %f, quat %f %f %f %f", time_in_trajectory, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
                    }
                    pub_path.publish(path);
                }
            }
        }
        
        // 飞行模式处理
        switch(flight_mode) {
            case PASS_THROUGH: {
                joystick_output = joystick_input;
                if(JOY_READY){
                    joystick_output.axes[JOY_CHANNEL_AUX1] = 0;
                    joystick_output.axes[JOY_CHANNEL_AUX2] = 0;
                    joystick_output.axes[JOY_CHANNEL_AUX3] = 0;
                }
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
                if(joystick_output.axes[JOY_CHANNEL_THROTTLE] < TAKEOFF_THROTTLE){
                    joystick_output.axes[JOY_CHANNEL_THROTTLE] += ros_dt/TAKEOFF_THROTTLE_TIME*TAKEOFF_THROTTLE;
                }
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
            case AUTO_FLYING: {
                if (!TRAJECTORY_READY || !CSV_TRAJECTORY_LOADED) {
                    ROS_ERROR_THROTTLE(1, "Trajectory not loaded! Cannot track.");
                    break;
                }

                TrajectoryPoint desired_point_ned;
                double current_trajectory_time;
                
                if (!TRAJECTORY_STARTED) {
                    ROS_INFO_THROTTLE(2, "Waiting for SPACE key to start trajectory tracking...");
                    desired_point_ned.x = X_INIT;
                    desired_point_ned.z = -Z_INIT; 
                    desired_point_ned.vx = 0;
                    desired_point_ned.vz = 0;
                    desired_point_ned.theta = 0;
                } else {
                    current_trajectory_time = ROS_TIME - trajectory_start_time;
                    desired_point_ned = getCSVTrajectoryPoint(current_trajectory_time);
                }
                
                if (TRAJECTORY_STARTED && current_trajectory_time >= csv_trajectory.back().time) {
                    ROS_INFO("Trajectory tracking completed! Staying in AUTO_FLYING mode.");
                    TRAJECTORY_STARTED = false; 
                    break;
                }
                
                static double prev_time = current_trajectory_time;
                static double prev_vx = desired_point_ned.vx;
                static double prev_vz = desired_point_ned.vz;
                
                double ax_desired = 0;
                double az_desired = 0;
                
                if (TRAJECTORY_STARTED && current_trajectory_time > prev_time && prev_time > 0) {
                    double dt = current_trajectory_time - prev_time;
                    if (dt > 1e-6) { 
                        ax_desired = (desired_point_ned.vx - prev_vx) / dt;
                        az_desired = (desired_point_ned.vz - prev_vz) / dt;
                    }
                }
                
                prev_time = current_trajectory_time;
                prev_vx = desired_point_ned.vx;
                prev_vz = desired_point_ned.vz;
                
                minsnap_pose << desired_point_ned.x, 0, desired_point_ned.z;
                minsnap_vel << desired_point_ned.vx, 0, desired_point_ned.vz;
                minsnap_acc << ax_desired, 0, az_desired; 
                minsnap_yaw = 0;  
                
                // 发布调试信息
                geometry_msgs::Vector3Stamped pubMinPose;
                pubMinPose.header.stamp = ros::Time::now();
                pubMinPose.header.frame_id = "world";
                pubMinPose.vector.x = desired_point_ned.x;                    
                pubMinPose.vector.y = 0;                                      
                pubMinPose.vector.z = -desired_point_ned.z;                   
                pub_minsnap_pose.publish(pubMinPose);
                
                geometry_msgs::Vector3Stamped pubMinVel;
                pubMinVel.header.stamp = ros::Time::now();
                pubMinVel.header.frame_id = "world";
                pubMinVel.vector.x = desired_point_ned.vx;                   
                pubMinVel.vector.y = 0;                                       
                pubMinVel.vector.z = -desired_point_ned.vz;                  
                pub_minsnap_vel.publish(pubMinVel);
                
                geometry_msgs::Vector3Stamped pubMinAcc;
                pubMinAcc.header.stamp = ros::Time::now();
                pubMinAcc.header.frame_id = "world";
                pubMinAcc.vector.x = ax_desired;                             
                pubMinAcc.vector.y = 0;                                       
                pubMinAcc.vector.z = -az_desired;                            
                pub_minsnap_acc.publish(pubMinAcc);
                
                ROS_INFO_THROTTLE(0.5, "Tracking trajectory: t=%.2f/%.2f, x=%.2f, z=%.2f (NED)", 
                                 current_trajectory_time, csv_trajectory.back().time,
                                 desired_point_ned.x, desired_point_ned.z);
                
                // ========== 位置控制器=========
                // NWU转NED
                Eigen::Vector3d current_pose_ned(
                    pose_world.pose.position.x,      
                    -pose_world.pose.position.y,    
                    -pose_world.pose.position.z      
                );
                
                Eigen::Vector3d pose_error = minsnap_pose - current_pose_ned;
                pose_error_integral += pose_error / ros_freq;
                // cout<<"pose_error: "<<pose_error<<endl<<endl;
                // cout<<"minsnap_pose: "<<minsnap_pose<<endl<<endl;
                // cout<<"current_pose_ned: "<<current_pose_ned<<endl<<endl;
                // cout<<"pose_error_integral: "<<pose_error_integral<<endl<<endl;
                
                Eigen::Vector3d current_vel_ned(
                    CurrVel.pose.orientation.x, 
                    CurrVel.pose.orientation.y,     
                    CurrVel.pose.orientation.z      
                );
                
                Eigen::Vector3d vel_error = minsnap_vel - current_vel_ned;
                
                // PID控制计算期望力
                // 增加积分限幅
                pose_error_integral = pose_error_integral.cwiseMin(Eigen::Vector3d(X_limit, Y_limit, Z_limit));
                pose_error_integral = pose_error_integral.cwiseMax(-Eigen::Vector3d(X_limit, Y_limit, Z_limit));
                Eigen::Vector3d F_des = -Kp_pose * pose_error - Kd_pose * vel_error - Ki_pose * pose_error_integral 
                                      + VEHICLE_WEIGHT * Eigen::Vector3d(0, 0, gravityG) - VEHICLE_WEIGHT * minsnap_acc;
                ROS_INFO_THROTTLE(2, "F_des NED: [%.2f, %.2f, %.2f]", F_des(0), F_des(1), F_des(2));
                
                // ========== 姿态计算 ==========
                // 当前姿态（NWU坐标系）
                Eigen::Quaterniond Quat_Feedback_NWU(
                    pose_world.pose.orientation.w,
                    pose_world.pose.orientation.x,
                    pose_world.pose.orientation.y,
                    pose_world.pose.orientation.z
                );
                
                // 转换为NED四元数
                Eigen::Quaterniond Quat_Feedback_NED = QuatNWUtoQuatNED(Quat_Feedback_NWU);
                Eigen::Matrix3d R_N_W = Quat_Feedback_NED.toRotationMatrix();
                R_N_W_BODY = R_N_W;

                // 推力计算
                float T_des = saturate(F_des.dot(R_N_W.col(2)), 0, VEHICLE_WEIGHT * 9.8f * 10.0f);
                // cout<<"T_des: "<<T_des<<endl<<endl;
                
                // 姿态控制
                Eigen::Vector3d Z_B_des = F_des / F_des.norm();
                Eigen::Matrix3d R_des = getNearestRotationMatrix(Z_B_des, minsnap_yaw);
                
                Eigen::Quaterniond Quat_Contr(R_des);
                Eigen::Quaterniond Quat_error = Quat_Contr * Quat_Feedback_NED.conjugate();
                Eigen::AngleAxisd AA_error(Quat_error);
                
                // 角速度控制
                Eigen::Vector3d omega_Contr = KP_att * R_N_W_BODY.inverse() * AA_error.angle() * AA_error.axis();
                Eigen::Vector3d omega_Des = minsnap_rate + omega_Contr;
                
                // 发布控制指令
                cmd_vel.header.seq += 1;
                cmd_vel.header.stamp = ros::Time::now();
                cmd_vel.twist.angular.x = omega_Des(0);
                cmd_vel.twist.angular.y = omega_Des(1);
                cmd_vel.twist.angular.z = omega_Des(2);
                
                if(!isnan(omega_Des(0)) && !isnan(omega_Des(1)) && !isnan(omega_Des(2))) {
                    pub_control_rate.publish(cmd_vel);
                }
                
                // 推力控制
                float throttle_K = VEHICLE_WEIGHT * 9.8f / pow(TAKEOFF_THROTTLE, 2); 
                // cout<<"TAKEOFF_THROTTLE: "<<TAKEOFF_THROTTLE<<endl<<endl;
                cmd_thrust.data = sqrt(T_des / throttle_K);
                if(!isnan(T_des)) {
                    pub_control_thrust.publish(cmd_thrust);
                }
                
                // 转换为手柄信号，注意俯仰角推杆是正（对应低头）
                joystick_output = joystick_input;
                double roll_sp = omega_Des(0) * 57.3 / 2000.0f;
                double pitch_sp = -omega_Des(1) * 57.3 / 2000.0f;
                double yaw_sp = omega_Des(2) * 57.3 / 2000.0f;
                double throttle_sp = cmd_thrust.data;
                
                joystick_output.axes[JOY_CHANNEL_ROLL] = saturate(roll_sp, -1, 1);
                joystick_output.axes[JOY_CHANNEL_PITCH] = saturate(pitch_sp, -1, 1);
                joystick_output.axes[JOY_CHANNEL_YAW] = saturate(yaw_sp, -1, 1);
                joystick_output.axes[JOY_CHANNEL_THROTTLE] = saturate(throttle_sp, 0, 1);
                // AUX1 AUX2 AUX3 为期望角度在机体系下的投影
                joystick_output.axes[JOY_CHANNEL_AUX1] = saturate(roll_sp, -1, 1);
                joystick_output.axes[JOY_CHANNEL_AUX2] = saturate(pitch_sp, -1, 1);
                joystick_output.axes[JOY_CHANNEL_AUX3] = saturate(yaw_sp, -1, 1);
                break;
            }
            
            default:
                joystick_output = joystick_input;
                if(JOY_READY){
                    joystick_output.axes[JOY_CHANNEL_AUX1] = 0;
                    joystick_output.axes[JOY_CHANNEL_AUX2] = 0;
                    joystick_output.axes[JOY_CHANNEL_AUX3] = 0;
                }
                break;
        }
        
        flight_mode_previous = flight_mode;
        
        if(JOY_READY == true) {
            joystick_output.header.seq = count;
            joystick_output.header.stamp = ros::Time::now();
            pub_joy_output.publish(joystick_output);
        }
        
        std_msgs::Int16 flight_mode_msg;
        flight_mode_msg.data = flight_mode;
        pub_flight_mode.publish(flight_mode_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
}