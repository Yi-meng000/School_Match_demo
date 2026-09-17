#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/chassis_cmd.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"
#include <chrono>
using namespace std::chrono_literals;

class CommMuxNode : public rclcpp::Node
{
public:
    CommMuxNode(const std::string & node_name) : Node(node_name)
    {
        sub_cmd_controller_ = this->create_subscription<robot_interfaces::msg::ControllerCmd>(
            "cmd_controller", 10, 
            std::bind(&CommMuxNode::CmdChassisCallback, this, std::placeholders::_1));
        sub_cmd_track_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_track", 10,
            std::bind(&CommMuxNode::CmdTrackCallback, this, std::placeholders::_1));
        pub_cmd_chassis_ = this->create_publisher<robot_interfaces::msg::ChassisCmd>("cmd_chassis", 10);
        pub_timer_ = this->create_wall_timer(10ms,std::bind(&CommMuxNode::PubTimerCallback, this));
    }

    ~CommMuxNode()
    {
        RCLCPP_INFO(this->get_logger(), "CommMuxNode is shutting down.");
    }

private:
    rclcpp::Subscription<robot_interfaces::msg::ControllerCmd>::SharedPtr sub_cmd_controller_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_track_;
    rclcpp::Publisher<robot_interfaces::msg::ChassisCmd>::SharedPtr pub_cmd_chassis_;
    rclcpp::TimerBase::SharedPtr pub_timer_; // 定时器，用于定期发布底盘控制命令

    bool trajectory = false; // 轨迹标志，初始值为 false
    bool enable = false; // 使能标志，初始值为 false
    bool protect = false; // 保护标志，初始值为 false

    struct chassis_velocity
    {
        float vx;
        float vy;
        float vw;
    };
    
    chassis_velocity controller_vel; // 控制器输出速度
    chassis_velocity track_vel; // 寻迹输出速度

    void CmdChassisCallback(const robot_interfaces::msg::ControllerCmd::SharedPtr msg)
    {
        trajectory = msg->trajectory;
        enable = msg->enable;
        protect = msg->protect;
        controller_vel.vx = msg->vx;
        controller_vel.vy = msg->vy;
        controller_vel.vw = msg->vw;
    }

    void CmdTrackCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        track_vel.vx = msg->linear.x;
        track_vel.vy = msg->linear.y;
        track_vel.vw = msg->angular.z;
    }

    void PubTimerCallback()
    {
        // 定时发布底盘控制命令
        auto chassis_cmd_msg = robot_interfaces::msg::ChassisCmd();
        chassis_cmd_msg.enable = enable;
        chassis_cmd_msg.protect = protect;
        if(trajectory) {
            chassis_cmd_msg.vx = track_vel.vx;
            chassis_cmd_msg.vy = track_vel.vy;
            chassis_cmd_msg.vw = track_vel.vw;
        } else {
            chassis_cmd_msg.vx = controller_vel.vx;
            chassis_cmd_msg.vy = controller_vel.vy;
            chassis_cmd_msg.vw = controller_vel.vw;
        }

        pub_cmd_chassis_->publish(chassis_cmd_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommMuxNode>("comm_mux_node");
    RCLCPP_INFO(node->get_logger(), "comm_mux_node has started!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
