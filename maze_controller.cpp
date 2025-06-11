
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <iostream>
#include <cstdlib>

using std::placeholders::_1;

class MazeController : public rclcpp::Node
{
public:
    MazeController(float duration, float r, float g, float b)
        : Node("twist_bolt_teleop_cpp"),
          duration_(duration), color_r_(r), color_g_(g), color_b_(b), color_a_(1.0)
    {
        pub_action_ = this->create_publisher<geometry_msgs::msg::Twist>("/bolt/cmd_vel", 10);
        pub_duration_ = this->create_publisher<std_msgs::msg::Float32>("/bolt/cmd_duration", 10);
        pub_color_ = this->create_publisher<std_msgs::msg::ColorRGBA>("/bolt/cmd_color", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&MazeController::publish_commands, this));

        RCLCPP_INFO(this->get_logger(), "Maze controller started");
    }

private:
    void publish_commands()
    {
        if (step_ >= 16)
        {
            RCLCPP_INFO(this->get_logger(), "Finished all steps.");
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x = moves_[step_][0];
        twist.angular.z = moves_[step_][1];

        std_msgs::msg::Float32 duration_msg;
        duration_msg.data = duration_;

        std_msgs::msg::ColorRGBA color_msg;
        color_msg.r = color_r_;
        color_msg.g = color_g_;
        color_msg.b = color_b_;
        color_msg.a = color_a_;

        pub_action_->publish(twist);
        pub_duration_->publish(duration_msg);
        pub_color_->publish(color_msg);

        RCLCPP_INFO(this->get_logger(), "Step %d: Sent speed %.2f, angular %.2f", step_, twist.linear.x, twist.angular.z);
        step_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_action_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_duration_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pub_color_;
    rclcpp::TimerBase::SharedPtr timer_;

    float duration_;
    float color_r_, color_g_, color_b_, color_a_;
    int step_ = 0;

    int moves_[16][2] = {
        {20, 0}, {0, -90}, {50, 0}, {0, 90}, {30, 0}, {40, 0},
        {0, 90}, {20, 0}, {50, 0}, {20, 0}, {0, -90}, {40, 0},
        {0, -90}, {40, 0}, {0, 90}, {20, 0}
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 5)
    {
        std::cout << "Usage: twist_bolt_teleop_cpp duration(2~8) color-r(0~255) color-g(0~255) color-b(0~255)" << std::endl;
        return 1;
    }

    float duration = std::stof(argv[1]);
    float r = std::stof(argv[2]);
    float g = std::stof(argv[3]);
    float b = std::stof(argv[4]);

    auto node = std::make_shared<MazeController>(duration, r, g, b);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
