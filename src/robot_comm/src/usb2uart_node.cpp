#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/chassis_cmd.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>
#include <cstring>
#include <cstdint>
#include <string>


#pragma pack(push, 1) // 把当前的对齐规则“压入栈中备份”，并将当前对齐规则强制改为“1字节对齐”

struct ChassisCmdFrame {
    uint8_t  header;
    uint8_t  enable;
    uint8_t  protect;  
    float    vel_x;   
    float    vel_y;    
    float    vel_w;  
    uint8_t  tail;        
};

#pragma pack(pop)     // 从栈中“弹出并恢复”原来的默认对齐规则


class Usb2UartTransNode : public rclcpp::Node
{
public:
    explicit Usb2UartTransNode(const std::string & node_name): Node(node_name)
    {
        initSerial("/dev/tty_stm32h7", 115200);

        sub_cmd_chassis_ = this->create_subscription<robot_interfaces::msg::ChassisCmd>(
            "cmd_chassis", 10, 
            std::bind(&Usb2UartTransNode::CmdChassisCallback, this, std::placeholders::_1));
    }

    ~Usb2UartTransNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "stm32发送串口已关闭");
        }
    }

private:
    rclcpp::Subscription<robot_interfaces::msg::ChassisCmd>::SharedPtr sub_cmd_chassis_; // 订阅cmd_chassis的智能指针
    int serial_fd_{-1}; // 串口文件描述符 (-1 表示未打开)

    bool initSerial(const std::string &port_name, int baud_rate)
    {
        // 以读写模式、非控制终端模式打开串口
        serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s: %s", port_name.c_str(), strerror(errno));
            return false;
        }

        // 清除 O_NDELAY 恢复正常读写状态
        fcntl(serial_fd_, F_SETFL, 0);

        struct termios options;
        memset(&options, 0, sizeof(options));

        // 获取当前串口配置
        if (tcgetattr(serial_fd_, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
            close(serial_fd_); 
            serial_fd_ = -1;
            return false;
        }

        // 设置波特率
        speed_t speed = B115200;
        if (baud_rate == 9600) speed = B9600;
        else if (baud_rate == 460800) speed = B460800;
        else if (baud_rate == 921600) speed = B921600;

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // 配置 8N1 (8 数据位, 无校验, 1 停止位)
        options.c_cflag |= (CLOCAL | CREAD); // 启用接收并设为本地连接
        options.c_cflag &= ~PARENB;          // 无奇偶校验
        options.c_cflag &= ~CSTOPB;          // 1 个停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8 个数据位
        options.c_cflag &= ~CRTSCTS;         // 无硬件流控

        // 配置为 Raw 原始数据传输模式（禁止 Linux 内核转义 0x0A, 0x0D 等字符）
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
        options.c_oflag &= ~OPOST;

        // 刷新输入缓冲区并立即应用设置
        tcflush(serial_fd_, TCIFLUSH);
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
            close(serial_fd_); 
            serial_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "串口 %s 初始化成功，波特率: %d", port_name.c_str(), baud_rate);
        return true;
    }

    void CmdChassisCallback(const robot_interfaces::msg::ChassisCmd::SharedPtr msg)
    {
        if (serial_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "串口未就绪！");
            return;
        }

        ChassisCmdFrame frame;
        frame.header = 0xA5;
        frame.enable = msg->enable;
        frame.protect = msg->protect;
        frame.vel_x = msg->vx;
        frame.vel_y = msg->vy;
        frame.vel_w = msg->vw;
        frame.tail = 0x5A;
        ssize_t bytes_sent = write(serial_fd_, &frame, sizeof(frame));
        if(bytes_sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "向串口发送数据失败: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), 
            "串口发送 cmd_chassis: enable=%d, protect=%d, vel_x=%.2f, vel_y=%.2f, vel_w=%.2f", 
            frame.enable, frame.protect, frame.vel_x, frame.vel_y, frame.vel_w);
        }
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Usb2UartTransNode>("usb2uart_node");
    RCLCPP_INFO(node->get_logger(), "usb2uart_node has started!");  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

