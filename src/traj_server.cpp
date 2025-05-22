#include "mpc.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "time.h"
#include <nav_msgs/Path.h>
#include <boost/algorithm/clamp.hpp>
#include "trajectory_info.h"
#include "bezier_curve.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
int N = 0;

const double t_step = 0.03;
const double save_distance = 1.2;

ros::Publisher vel_cmd_pub, ref_vel_cmd_pub, ref_point_pub, ref_path_pub;
ros::Publisher recorded_path_pub;
ros::Publisher global_path_pub;
nav_msgs::Path recorded_path;

nav_msgs::Odometry odom_;
geometry_msgs::PoseStamped current_pose_;

geometry_msgs::Twist cmd, ref_cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
double traj_duration_;

Eigen::Vector3d odom_pos_, odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC mpc_controller;
double roll, pitch, yaw;
geometry_msgs::PoseStamped pose_cur;
tf::Quaternion quat;
std_msgs::UInt8 is_adjust_pose;
std_msgs::UInt8 dir;

enum DIRECTION {
    POSITIVE = 0, NEGATIVE = 1
};

std_msgs::UInt8 stop_command;

////time record
clock_t start_clock, end_clock;
double duration;

trajectory_utils::TrajectoryInfo trajectory_info;

void globalPathCallback(nav_msgs::PathConstPtr msg) {

    if (msg->poses.empty()) {
        ROS_ERROR("Received an empty path");
        receive_traj_ = false;
        return;
    }

    mpc_controller.generateReferenceTrajectory(current_pose_, msg->poses);

    receive_traj_ = true;
}

void poseCallback(geometry_msgs::PoseStampedConstPtr msg) {
    pose_cur = *msg;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(std_msgs::UInt8ConstPtr msg) {
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::UInt8ConstPtr &msg) {
    dir = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_ = *msg;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (dir.data == NEGATIVE) {
        if (yaw > 0) {
            yaw -= PI;
        } else if (yaw < 0) {
            yaw += PI;
        }
    }

    recorded_path.header.frame_id = "map";
    geometry_msgs::PoseStamped pose;
    pose.pose = msg->pose.pose;
    recorded_path.poses.push_back(pose);
    recorded_path_pub.publish(recorded_path);

    current_pose_.pose = msg->pose.pose;

}

geometry_msgs::Twist last_cmd;

void cmdCallback(const ros::TimerEvent &e) {

    if (!receive_traj_)
        return;

    if (!mpc_controller.calculateVelocity(current_pose_, cmd)) {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        receive_traj_ = false;
    }

    vel_cmd_pub.publish(cmd);
    last_cmd = cmd;
}

void clickPointCallback(const geometry_msgs::PointStamped &msg) {
    ROS_INFO("Received clicked point: (%.2f, %.2f)", msg.point.x, msg.point.y);
    geometry_msgs::PoseStamped current_pose, target_pose;
    current_pose.pose = odom_.pose.pose;
    target_pose.pose.position = msg.point;
    target_pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(
                    std::atan2(msg.point.y - odom_.pose.pose.position.y,
                               msg.point.x - odom_.pose.pose.position.x));

    trajectory_utils::TrajectoryPoint traj_point;
    trajectory_info.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(odom_pos_(0), odom_pos_(1)), traj_point);

    nav_msgs::Path pub_path;
    pub_path.header.frame_id = "odom";
    pub_path.poses =  kappaConstrainedBezierCurve({current_pose, target_pose},
                                                  traj_point.path_point().kappa());
    global_path_pub.publish(pub_path);

//    globalPathCallback(&pub_path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node("~");

    ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
    ros::Subscriber global_path_sub = node.subscribe("/global_path", 10, globalPathCallback);
    ros::Subscriber clicked_point_sub = node.subscribe("/clicked_point", 10, clickPointCallback);

    ros::Publisher pub_map = node.advertise<nav_msgs::OccupancyGrid>("/map",10);
    nav_msgs::OccupancyGrid msg;// 创建一个OccupancyGrid类型的消息
    msg.header.frame_id = "map";
    msg.info.resolution = 1.0;
    msg.info.width = 30;
    msg.info.height = 30;
    msg.data.resize(msg.info.width*msg.info.height);

    mpc_controller.init(1.8, 1.0, 19, 0.3);
    vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    ref_vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/ref_cmd_vel", 50);
    ref_point_pub = node.advertise<geometry_msgs::PoseStamped>("/ref_point", 50);
    ref_path_pub = node.advertise<nav_msgs::Path>("/ref_path", 50);
    recorded_path_pub = node.advertise<nav_msgs::Path>("/recorded_path", 50);
    global_path_pub = node.advertise<nav_msgs::Path>("/global_path", 2);
    stop_command.data = 0;
    dir.data = POSITIVE;


    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.05), cmdCallback);

    node.param("/traj_server/horizon", N, 0);
    ROS_INFO("horizon: %d", N);

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    pub_map.publish(msg);

    ros::spin();

    return 0;
}