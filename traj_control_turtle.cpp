#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

// Global variables for current pose and goal
turtlesim::Pose current_pose;
bool pose_received = false;

// Target position
const float x_goal = 1.0;
const float y_goal = 3.0;

// Gain parameters (can be adjusted)
const float Kv = 1.5;
const float Kw = 6.0;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
    pose_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_control_turtle");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        ros::spinOnce();

        if (!pose_received)
        {
            loop_rate.sleep();
            continue;
        }

        float dx = x_goal - current_pose.x;
        float dy = y_goal - current_pose.y;
        float distance = sqrt(dx * dx + dy * dy);
        float angle_to_goal = atan2(dy, dx);
        float angle_error = angle_to_goal - current_pose.theta;

        // Normalize angle_error to [-pi, pi]
        angle_error = atan2(sin(angle_error), cos(angle_error));

        geometry_msgs::Twist cmd_vel;

        // Stop condition
        if (distance < 0.05)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }
        else
        {
            cmd_vel.linear.x = Kv * distance;
            cmd_vel.angular.z = Kw * angle_error;
        }

        vel_pub.publish(cmd_vel);
        loop_rate.sleep();
    }

    return 0;
}
