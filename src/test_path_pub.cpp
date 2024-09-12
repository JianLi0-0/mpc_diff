#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher global_path_pub;
nav_msgs::Odometry odom;
geometry_msgs::Pose odom_pose;
nav_msgs::Path global_path;
bool is_received_odom = false;

void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom = *msg;
    is_received_odom = true;

    odom_pose = odom.pose.pose;
//    cout << "odom: " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl;

//    double min_dist = 1e6;
//    int min_idx = 0;
//    for (int i = 0; i < global_path.poses.size(); i++) {
//        double dx = global_path.poses[i].pose.position.x - odom.pose.pose.position.x;
//        double dy = global_path.poses[i].pose.position.y - odom.pose.pose.position.y;
//        double dist = sqrt(dx * dx + dy * dy);
//        if (dist < min_dist) {
//            min_dist = dist;
//            min_idx = i;
//        }
//    }
//
//    cout << "min_idx: " << min_idx << endl;
//
//    global_path.poses.erase(global_path.poses.begin(), global_path.poses.begin() + min_idx);
//
//    global_path_pub.publish(global_path);
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& pose) {
    // 提取机器人位姿的平移和旋转部分
    Eigen::Translation3d translation(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
    Eigen::Quaterniond rotation(robot_pose.orientation.w, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z);

    // 创建变换矩阵
    Eigen::Affine3d transform = translation * rotation;

    // 提取目标位姿的平移部分
    Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);

    // 将目标位姿的平移部分变换到世界坐标系
    Eigen::Vector3d transformed_point = transform * point;

    // 创建变换后的位姿
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = transformed_point.x();
    transformed_pose.position.y = transformed_point.y();
    transformed_pose.position.z = transformed_point.z();
    transformed_pose.orientation = pose.orientation; // 保持原来的方向

    return transformed_pose;
}

nav_msgs::Path transformPath(const nav_msgs::Path& path, const geometry_msgs::Pose& robot_pose, const std::string& target_frame) {
    nav_msgs::Path transformedPath;
    transformedPath.header = path.header;
    transformedPath.header.frame_id = target_frame;

    for (const auto& pose : path.poses) {
        geometry_msgs::PoseStamped transformedPose = pose;
        transformedPose.header = transformedPath.header;
        transformedPose.pose = transformPose(robot_pose, pose.pose);
        transformedPath.poses.push_back(transformedPose);
    }

    return transformedPath;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_path_pub");
    ros::NodeHandle nh("test_node");

    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 2);
    ros::Subscriber odom_sub = nh.subscribe("/state_estimation", 10, odometryCallback);

    ros::Duration(0.5).sleep();

    std::vector<Eigen::Vector3d> points_list;

    global_path.header.frame_id = "sensor";
    global_path.header.stamp = ros::Time::now();

    double l1 = 2.0;
    double r1 = 2.0;
    double l2 = 4.0;
    for (double l = 0.0; l < l1; l += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.header = global_path.header;
        pose.pose.position.x = l;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.8;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
    }
    for (double rad = -M_PI / 2; rad < 0; rad += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.header = global_path.header;
        pose.pose.position.x = l1 + r1 * cos(rad);
        pose.pose.position.y = r1 + r1 * sin(rad);
        pose.pose.position.z = 0.8;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
    }
    for (double l = 0.0; l < l2; l += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.header = global_path.header;
        pose.pose.position.x = l1 + r1;
        pose.pose.position.y = r1 + l;
        pose.pose.position.z = 0.8;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
    }

//    cout << global_path << endl;
    ros::Rate rate(1);
    while (ros::ok() && !is_received_odom) {
        ros::spinOnce();
        rate.sleep();
    }

    global_path_pub.publish(transformPath(global_path, odom_pose, "odom"));

    ros::Time start = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        if ((ros::Time::now() - start).toSec() > 2.7) {
            start = ros::Time::now();
            global_path_pub.publish(transformPath(global_path, odom_pose, "odom"));
        }
        rate.sleep();
    }

    return 0;
}