#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;

ros::Publisher global_path_pub;
nav_msgs::Odometry odom;
geometry_msgs::Pose odom_pose;
nav_msgs::Path global_path;
bool is_received_odom = false;

double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return std::sqrt(dx*dx+dy*dy);
}
// 计算二项式系数的函数
unsigned long binomialCoefficient(unsigned int n, unsigned int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    if (k > n - k) k = n - k;
    unsigned long c = 1;
    for (unsigned int i = 0; i < k; ++i) {
        c *= (n - i);
        c /= (i + 1);
    }
    return c;
}

// 计算贝塞尔曲线点的函数
std::vector<geometry_msgs::PoseStamped> bezierCurve(
        const std::vector<geometry_msgs::PoseStamped>& points, const double& interval) {
    unsigned int numPoints = 100;
    auto len = distance(points.front(), points.back());
    numPoints = static_cast<unsigned int>(len / interval);
    std::vector<geometry_msgs::PoseStamped> curve(numPoints);
    unsigned int n = points.size() - 1;

    // 预计算二项式系数
    std::vector<unsigned long> binomials(n + 1);
    for (unsigned int i = 0; i <= n; ++i) {
        binomials[i] = binomialCoefficient(n, i);
    }

    // 计算贝塞尔曲线
    for (unsigned int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        float one_minus_t = 1.0 - t;
        float x = 0.0;
        float y = 0.0;
        for (unsigned int j = 0; j <= n; ++j) {
            float coefficient = binomials[j] * std::pow(one_minus_t, n - j) * std::pow(t, j);
            x += points[j].pose.position.x * coefficient;
            y += points[j].pose.position.y * coefficient;
        }
        curve[i].pose.position.x = x;
        curve[i].pose.position.y = y;
        curve[i].pose.orientation.w = 1;
        curve[i].header.frame_id = points[0].header.frame_id;
    }

    return curve;
}

std::vector<geometry_msgs::PoseStamped> cubicBezierCurve(
        const std::vector<geometry_msgs::PoseStamped>& points) {
    auto len = distance(points.front(), points.back());
    geometry_msgs::PoseStamped p1, p2, p3, p4;
    p1 = points.front();
    p4 = points.back();

    // 起点向前延伸0.35倍的距离
    p2 = p1;
    double cof_start = 0.4;
    p2.pose.position.x += cof_start * len * std::cos(tf::getYaw(p1.pose.orientation));
    p2.pose.position.y += cof_start * len * std::sin(tf::getYaw(p1.pose.orientation));

    // 终点向后延伸0.4倍的距离
    p3 = p4;
    double cof_end = 0.4;
    p3.pose.position.x += cof_end * (p1.pose.position.x - p4.pose.position.x);
    p3.pose.position.y += cof_end * (p1.pose.position.y - p4.pose.position.y);

    return bezierCurve({p1,p2,p3,p4}, 0.05);
}


void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom = *msg;
    is_received_odom = true;
    odom_pose = odom.pose.pose;
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

void clickPointCallback(const geometry_msgs::PointStamped &msg) {
    ROS_INFO("Received clicked point: (%.2f, %.2f)", msg.point.x, msg.point.y);
    geometry_msgs::PoseStamped current_pose, target_pose;
    current_pose.pose = odom.pose.pose;
    target_pose.pose.position = msg.point;
    target_pose.pose.orientation.w = 1.0;
    nav_msgs::Path pub_path;
    pub_path.header.frame_id = "odom";
    pub_path.poses = cubicBezierCurve({current_pose, target_pose});
    global_path_pub.publish(pub_path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_path_pub");
    ros::NodeHandle nh("test_node");

    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 2);
    ros::Subscriber odom_sub = nh.subscribe("/state_estimation", 10, odometryCallback);
    ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 10, clickPointCallback);
    ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map",10);

    nav_msgs::OccupancyGrid msg;// 创建一个OccupancyGrid类型的消息
    msg.header.frame_id = "map";
    msg.info.resolution = 1.0;
    msg.info.width = 30;
    msg.info.height = 30;
    msg.data.resize(msg.info.width*msg.info.height);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        msg.header.stamp = ros::Time::now();
        msg.info.origin.position.x = odom.pose.pose.position.x - msg.info.width/2.0;
        msg.info.origin.position.y = odom.pose.pose.position.y - msg.info.height/2.0;

        pub_map.publish(msg);

        rate.sleep();
    }

    return 0;
}