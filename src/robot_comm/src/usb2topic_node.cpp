#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/chassis_cmd.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>
#include <string>
#include <cstring>
#include <cstdint>
#include <vector>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;

constexpr size_t kFrameSize = 25; // 蓝牙帧长度
// std::string wifi_connect = "AT+CWJAP=iQOO Neo9S Pro,134795Frt\r\n";
        // std::string wifi_connect = "AT+CWJAP=iQOO 12,arcstar123\r\n";
        // write(serial_fd_, wifi_connect.c_str(), wifi_connect.length());
class BluetoothReceiveNode : public rclcpp::Node
{
public:
    explicit BluetoothReceiveNode(const std::string & node_name): Node(node_name)
    {
        pub_cmd_controller_ = this->create_publisher<robot_interfaces::msg::ControllerCmd>("cmd_controller", 10);
        initSerial("/dev/tty_bluetooth", 115200); 
        read_timer_ = this->create_wall_timer(10ms,
            std::bind(&BluetoothReceiveNode::TransTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Publisher for cmd_controller has been created.");
    }

    ~BluetoothReceiveNode() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "蓝牙接受串口已关闭");
            serial_fd_ = -1;
        }
    }


private:
    rclcpp::Publisher<robot_interfaces::msg::ControllerCmd>::SharedPtr pub_cmd_controller_; // 发布cmd_controller的智能指针
    int serial_fd_{-1}; // 串口文件描述符 (-1 表示未打开)
    std::vector<uint8_t> rx_buf_; // 接收缓冲区
    rclcpp::TimerBase::SharedPtr read_timer_; // 定时器，用于定期读取串口数据

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
            close(serial_fd_); 
            serial_fd_ = -1;            
            RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
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
        options.c_cc[VMIN]  = 0;
        options.c_cc[VTIME] = 0;
        
        // 刷新输入缓冲区并立即应用设置
        tcflush(serial_fd_, TCIFLUSH);
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            close(serial_fd_); 
            serial_fd_ = -1;            
            RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "串口 %s 初始化成功，波特率: %d", port_name.c_str(), baud_rate);
        return true;
    }

    void TransTimerCallback(){
        if(serial_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "串口未就绪！");
            return;
        }
        uint8_t buffer[256];
        for(;;)
        {
            ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                rx_buf_.insert(rx_buf_.end(), buffer, buffer + bytes_read);
            }else if (bytes_read < 0) {
                RCLCPP_ERROR(this->get_logger(), "读取串口数据失败: %s", strerror(errno));
                break;
            } else {
                break; // 没有更多数据可读
            }
        }
        SpliceFrame();
    }

    void SpliceFrame(){
        while(rx_buf_.size() >= kFrameSize) {
            if (rx_buf_[0] != 0xA5)  
            { 
                rx_buf_.erase(rx_buf_.begin()); 
                continue; 
            }
            if (rx_buf_[kFrameSize - 1] != 0x5A) 
            { 
                rx_buf_.erase(rx_buf_.begin()); 
                continue; 
            }
            auto frame_msg = robot_interfaces::msg::ControllerCmd();
            frame_msg.enable = rx_buf_[1];
            frame_msg.protect = rx_buf_[2];
            frame_msg.lockpoint = rx_buf_[3];
            frame_msg.trajectory = rx_buf_[4];
            memcpy(&frame_msg.goal_x, &rx_buf_[5], sizeof(uint16_t));
            memcpy(&frame_msg.goal_y, &rx_buf_[7], sizeof(uint16_t));
            memcpy(&frame_msg.goal_yaw, &rx_buf_[9], sizeof(uint16_t));
            memcpy(&frame_msg.vx, &rx_buf_[11], sizeof(float));
            memcpy(&frame_msg.vy, &rx_buf_[15], sizeof(float));
            memcpy(&frame_msg.vw, &rx_buf_[19], sizeof(float));
            pub_cmd_controller_->publish(frame_msg);
            rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + kFrameSize); 
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BluetoothReceiveNode>("bluetooth_receive_node");
    RCLCPP_INFO(node->get_logger(), "bluetooth_receive_node has started!");  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

