#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

using namespace std;

ros::Publisher global_path_pub;
nav_msgs::Odometry odom;
nav_msgs::Path global_path;

void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom = *msg;
    cout << "odom: " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl;

    double min_dist = 1e6;
    int min_idx = 0;
    for (int i = 0; i < global_path.poses.size(); i++) {
        double dx = global_path.poses[i].pose.position.x - odom.pose.pose.position.x;
        double dy = global_path.poses[i].pose.position.y - odom.pose.pose.position.y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }

    cout << "min_idx: " << min_idx << endl;

    global_path.poses.erase(global_path.poses.begin(), global_path.poses.begin() + min_idx);

    global_path_pub.publish(global_path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_path_pub");
    ros::NodeHandle nh("test_node");

    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 2);
//    ros::Subscriber odom_sub = nh.subscribe("/state_estimation", 10, odometryCallback);

    ros::Duration(0.5).sleep();

    std::vector<Eigen::Vector3d> points_list;

    global_path.header.frame_id = "map";
    global_path.header.stamp = ros::Time::now();

    double l1 = 2.0;
    double r1 = 1.0;
    double l2 = 2.0;
    for (double l = 0.0; l < l1; l += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = l;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.8;
        global_path.poses.push_back(pose);
    }
    for (double rad = -M_PI / 2; rad < M_PI / 2; rad += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = l1 + r1 * cos(rad);
        pose.pose.position.y = r1 + r1 * sin(rad);
        pose.pose.position.z = 0.8;
        global_path.poses.push_back(pose);
    }
    for (double l = 0.0; l < l2; l += 0.1) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = l1 - l;
        pose.pose.position.y = r1 + r1;
        pose.pose.position.z = 0.8;
        global_path.poses.push_back(pose);
    }

    cout << global_path << endl;

    global_path_pub.publish(global_path);

    ros::Rate rate(10);
    while (ros::ok()) {
//        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}