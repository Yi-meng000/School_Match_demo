#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/chassis_cmd.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"
#include "robot_interfaces/msg/tracing_input.hpp"

class CommMuxNode : public rclcpp::Node
{
public:
    CommMuxNode(const std::string & node_name) : Node(node_name)
    {
        sub_cmd_controller_ = this->create_subscription<robot_interfaces::msg::ControllerCmd>(
            "cmd_controller", 10, 
            std::bind(&CommMuxNode::CmdChassisCallback, this, std::placeholders::_1));
        pub_cmd_chassis_ = this->create_publisher<robot_interfaces::msg::ChassisCmd>("cmd_chassis", 10);
        pub_tracing_input_ = this->create_publisher<robot_interfaces::msg::TracingInput>("tracing_input", 10);
    }

    ~CommMuxNode()
    {
        RCLCPP_INFO(this->get_logger(), "CommMuxNode is shutting down.");
    }

private:
    rclcpp::Subscription<robot_interfaces::msg::ControllerCmd>::SharedPtr sub_cmd_controller_;
    // todo 订阅寻迹节点的cmd_track
    rclcpp::Publisher<robot_interfaces::msg::ChassisCmd>::SharedPtr pub_cmd_chassis_;
    rclcpp::Publisher<robot_interfaces::msg::TracingInput>::SharedPtr pub_tracing_input_;

    void CmdChassisCallback(const robot_interfaces::msg::ControllerCmd::SharedPtr msg)
    {
        auto tracing_input_msg = robot_interfaces::msg::TracingInput();
        auto chassis_cmd_msg = robot_interfaces::msg::ChassisCmd();


        // todo 目前没有自动和手动的切换，加入锁点后应补充上仲裁
        chassis_cmd_msg.enable = msg->enable;
        chassis_cmd_msg.protect = msg->protect;
        chassis_cmd_msg.vx = msg->vx;
        chassis_cmd_msg.vy = msg->vy;
        chassis_cmd_msg.vw = msg->vw;
        
        pub_cmd_chassis_->publish(chassis_cmd_msg);
        pub_tracing_input_->publish(tracing_input_msg);
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
