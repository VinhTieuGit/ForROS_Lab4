#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Globals
geometry_msgs::Pose current_pose;
geometry_msgs::Point goal_point;
bool pose_received = false;
bool goal_received = false;

// Parameters
const float Kv = 1.5;
const float Kw = 6.0;

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    current_pose = *msg;
    pose_received = true;
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    goal_point = *msg;
    goal_received = true;
}

// Normalize angle to [-π, π]
float normalizeAngle(float angle) {
    return atan2(sin(angle), cos(angle));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_point_sphero");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/bolt/bolt_pose", 10, poseCallback);
    ros::Subscriber goal_sub = nh.subscribe("/bolt/bolt_goal", 10, goalCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/bolt/cmd_vel", 10);

    ros::Rate rate(5); // 5 Hz loop

    while (ros::ok()) {
        ros::spinOnce();

        if (!pose_received || !goal_received) {
            rate.sleep();
            continue;
        }

        float dx = goal_point.x - current_pose.position.x;
        float dy = goal_point.y - current_pose.position.y;
        float distance = sqrt(dx * dx + dy * dy);

        // Get yaw from orientation.z (assuming planar orientation)
        float theta = current_pose.orientation.z;

        float angle_to_goal = atan2(dy, dx);
        float angle_error = normalizeAngle(angle_to_goal - theta);

        // Compute speed commands
        float linear_speed = Kv * distance;
        float angular_speed = Kw * angle_error;

        geometry_msgs::Twist cmd;
        cmd.linear.x = std::min(std::max(linear_speed, 0.0f), 25.0f);  // Clamp
        cmd.angular.z = static_cast<int>(angular_speed * 180.0 / M_PI);  // Convert to deg

        twist_pub.publish(cmd);

        ROS_INFO("Target: (%.2f, %.2f) | Current: (%.2f, %.2f) | Heading: %.2f°, Speed: %.2f",
                 goal_point.x, goal_point.y,
                 current_pose.position.x, current_pose.position.y,
                 cmd.angular.z, cmd.linear.x);

        rate.sleep();
    }

    return 0;
}
