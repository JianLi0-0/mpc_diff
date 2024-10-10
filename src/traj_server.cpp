#include "uniform_bspline.h"
#include "MPC.hpp"
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
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
#define N 15

const double t_step = 0.03;
const double save_distance = 1.2;

ros::Publisher vel_cmd_pub, ref_vel_cmd_pub, ref_point_pub, ref_path_pub;
ros::Publisher recorded_path_pub;
ros::Publisher global_path_pub;
nav_msgs::Path recorded_path;

geometry_msgs::Twist cmd, ref_cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;

Eigen::Vector3d odom_pos_, odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
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

    Eigen::MatrixXd pos_pts(3, msg->poses.size());

//    for (size_t i = 0; i < msg->poses.size(); ++i) {
//        pos_pts(0, i) = msg->poses[i].pose.position.x;
//        pos_pts(1, i) = msg->poses[i].pose.position.y;
//        pos_pts(2, i) = msg->poses[i].pose.position.z;
//    }
//
//    UniformBspline pos_traj(pos_pts, 3, 0.07);

    trajectory_utils::TrajectoryPoint traj_point;
    bool flag = trajectory_info.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(odom_pos_(0), odom_pos_(1)), traj_point);

    std::cout << traj_point.DebugString() << std::endl;

    std::vector<trajectory_utils::PathPoint> path_data;
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        trajectory_utils::PathPoint p;
        p.set_x(msg->poses[i].pose.position.x);
        p.set_y(msg->poses[i].pose.position.y);
        path_data.push_back(p);
    }

    trajectory_info.setPathData(path_data);

    trajectory_info.calSpeedData(
            0.0, traj_point.v(), traj_point.a(),
            trajectory_info.getPathDataPtr()->Length()-save_distance, 2.0);

    trajectory_info.combinePathAndSpeedProfile();

//    {
//
//        std::vector<double> t_vec, s_vec, v_vec, a_vec;
//        double time_span = trajectory_info.getSpeedDataPtr()->get_duration();
//        for (double t=0.0;t<=time_span;t+=0.01) {
//            t_vec.push_back(t);
//        }
//        std::vector<double> x_traj_vec, y_traj_vec, s_traj_vec, v_traj_vec, a_traj_vec, kappa_traj_vec, theta_traj_vec;
//        x_traj_vec.reserve(t_vec.size());
//        y_traj_vec.reserve(t_vec.size());
//        s_traj_vec.reserve(t_vec.size());
//        v_traj_vec.reserve(t_vec.size());
//        a_traj_vec.reserve(t_vec.size());
//        theta_traj_vec.reserve(t_vec.size());
//        kappa_traj_vec.reserve(t_vec.size());
//        auto discretized_trajectory = trajectory_info.getTrajectoryPtr();
//        for (auto t : t_vec) {
//            auto traj_point = discretized_trajectory->Evaluate(t);
//            x_traj_vec.push_back(traj_point.path_point().x());
//            y_traj_vec.push_back(traj_point.path_point().y());
//            s_traj_vec.push_back(traj_point.path_point().s());
//            kappa_traj_vec.push_back(traj_point.path_point().kappa());
//            v_traj_vec.push_back(traj_point.v());
//            a_traj_vec.push_back(traj_point.a());
//            theta_traj_vec.push_back(traj_point.path_point().theta());
//        }
//
//        plt::plot(t_vec, s_traj_vec);
//        plt::plot(t_vec, v_traj_vec);
//        plt::plot(t_vec, a_traj_vec);
//        plt::plot(t_vec, kappa_traj_vec);
//        plt::plot(t_vec, theta_traj_vec);
//
//        plt::show();
//    }

    traj_duration_ = trajectory_info.getSpeedDataPtr()->get_duration();
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

std::vector<double> t_vec, x_ref_vec, y_ref_vec, theta_ref_vec, s_ref_vec, v_ref_vec, w_ref_vec, a_ref_vec, kappa_ref_vec;
bool is_data_updated = false;

void MPC_calculate(double &remain_s) {
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r, pos_r_1, pos_final, v_r_1, v_r_2, X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r, x_r_1, x_r_2;
    double v_linear_1, w;
    double t_k, t_k_1;

    auto discretized_trajectory = trajectory_info.getTrajectoryPtr();
    trajectory_utils::TrajectoryPoint traj_point;
    trajectory_info.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(odom_pos_(0), odom_pos_(1)), traj_point);
    double t_cur = traj_point.relative_time();
    std::cout << "t_cur: " << t_cur << ",  traj_duration_: " << traj_duration_ << std::endl;

//    pos_final = traj_[0].evaluateDeBoor(traj_duration_);
    auto end_point = discretized_trajectory->Evaluate(traj_duration_);
    pos_final << end_point.path_point().x(), end_point.path_point().y(), 0.0;

    remain_s = end_point.path_point().s() - traj_point.path_point().s();

    {
        is_data_updated = true;
        t_vec.clear();
        x_ref_vec.clear();
        y_ref_vec.clear();
        theta_ref_vec.clear();
        s_ref_vec.clear();
        v_ref_vec.clear();
        w_ref_vec.clear();
        a_ref_vec.clear();
        kappa_ref_vec.clear();
    }

    bool is_orientation_adjust = false;
    double orientation_adjust=0;

    for (int i = 0; i < N; i++) {

        t_k = t_cur + i * t_step;
        t_k_1 = t_cur + (i + 1) * t_step;

        t_vec.push_back(t_k);

        auto pos_r_raw = discretized_trajectory->Evaluate(t_k);
        auto pos_r_1_raw = discretized_trajectory->Evaluate(t_k_1);
        pos_r << pos_r_raw.path_point().x(), pos_r_raw.path_point().y(), 0.0;

        x_r(0) = pos_r(0);
        x_r(1) = pos_r(1);

        x_ref_vec.push_back(pos_r(0));
        y_ref_vec.push_back(pos_r(1));
        s_ref_vec.push_back(pos_r_raw.path_point().s());

        v_linear_1 = pos_r_raw.v();
        v_ref_vec.push_back(v_linear_1);

        x_r(2) = pos_r_raw.path_point().theta();
        theta_ref_vec.push_back(x_r(2));

        double yaw1 = pos_r_raw.path_point().theta();
        double yaw2 = pos_r_1_raw.path_point().theta();

        if (is_orientation_adjust) {
            x_r(2) += orientation_adjust;
        }

        if (abs(yaw2 - yaw1) > PI) {
            is_orientation_adjust = true;
            if ((yaw2 - yaw1) < 0) {
                orientation_adjust = 2 * PI;
            } else {
                orientation_adjust = -2 * PI;
            }
        }

        w = pos_r_raw.v() * pos_r_raw.path_point().kappa();
        w_ref_vec.push_back(w);
        kappa_ref_vec.push_back(pos_r_raw.path_point().kappa());

        u_r(0) = v_linear_1;
        u_r(1) = w;

        X_r.push_back(x_r);
        U_r.push_back(u_r);
    }

    X_k(0) = odom_pos_(0);
    X_k(1) = odom_pos_(1);
    if (yaw / X_r[0](2) < 0 && abs(yaw) > (PI * 5 / 6)) {
        if (yaw < 0) {
            X_k(2) = yaw + 2 * PI;
        } else {
            X_k(2) = yaw - 2 * PI;
        }
    } else {
        X_k(2) = yaw;
    }

    u_k = mpc_controller.MPC_Solve_qp(X_k, X_r, U_r, N);

    if (dir.data == NEGATIVE) {
        cmd.linear.x = -u_k.col(0)(0);
    } else {
        cmd.linear.x = u_k.col(0)(0);
    }

    cmd.angular.z = u_k.col(0)(1);
    static int conut1 = 0;
    conut1 += 1;

    cout << "current vel : : " << u_k.col(0)(0) << "m/s" << endl;

//    vel_cmd_pub.publish(cmd);
    ref_cmd.linear.x = v_ref_vec[0];
    ref_cmd.angular.z = w_ref_vec[0];
    ref_cmd.angular.y = kappa_ref_vec[0];
    ref_vel_cmd_pub.publish(ref_cmd);
    geometry_msgs::PoseStamped ref_point;
    ref_point.header.frame_id = "map";
    ref_point.pose.position.x = x_ref_vec[0];
    ref_point.pose.position.y = y_ref_vec[0];
    ref_point.pose.orientation = tf::createQuaternionMsgFromYaw(theta_ref_vec[0]);
    ref_point_pub.publish(ref_point);
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = "map";
    for (int i = 0; i < N; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x_ref_vec[i];
        pose.pose.position.y = y_ref_vec[i];
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_ref_vec[i]);
        ref_path.poses.push_back(pose);
    }
    ref_path_pub.publish(ref_path);
}

void stopCallback(std_msgs::UInt8ConstPtr msg) {
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
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

}

geometry_msgs::Twist last_cmd;

void cmdCallback(const ros::TimerEvent &e) {

    if (!receive_traj_)
        return;

    double remain_s = 0.0;
    MPC_calculate(remain_s);

    if (remain_s < 0.06) {
        ROS_INFO("remain_s < 0.1");
        trajectory_info.reset();
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        receive_traj_ = false;
    }

    vel_cmd_pub.publish(cmd);
    last_cmd = cmd;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node("~");

    ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
    ros::Subscriber global_path_sub = node.subscribe("/global_path", 10, globalPathCallback);

    mpc_controller.MPC_init(node);
    vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    ref_vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/ref_cmd_vel", 50);
    ref_point_pub = node.advertise<geometry_msgs::PoseStamped>("/ref_point", 50);
    ref_path_pub = node.advertise<nav_msgs::Path>("/ref_path", 50);
    recorded_path_pub = node.advertise<nav_msgs::Path>("/recorded_path", 50);
    stop_command.data = 0;
    dir.data = POSITIVE;

//    plt::figure_size(640, 640);


    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.03), cmdCallback);

//  nh.param("traj_server/time_forward", time_forward_, -1.0);
//  last_yaw_ = 0.0;
//  last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    ros::spin();

    return 0;
}