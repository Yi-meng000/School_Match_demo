#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_comm/chassis_feedback_protocol.hpp"
#include "robot_interfaces/msg/chassis_cmd.hpp"
#include "robot_interfaces/msg/chassis_feedback.hpp"

namespace
{

constexpr float kMaxSpeed = 1.75F;
constexpr float kMaxOmega = 5.23598776F;

#pragma pack(push, 1)
struct ChassisCmdFrame
{
  std::uint8_t header;
  std::uint8_t enable;
  std::uint8_t protect;
  float vel_x;
  float vel_y;
  float vel_w;
  std::uint8_t tail;
};
#pragma pack(pop)

static_assert(sizeof(ChassisCmdFrame) == 16U, "unexpected chassis command frame size");

}  // namespace

class Usb2UartTransNode : public rclcpp::Node
{
public:
  explicit Usb2UartTransNode(const std::string & node_name)
  : Node(node_name)
  {
    port_name_ =
      declare_parameter<std::string>("port_name", "/dev/tty_stm32h7");
    const int baud_rate = declare_parameter<int>("baud_rate", 115200);
    const int read_period_ms =
      declare_parameter<int>("read_period_ms", 2);
    feedback_frame_id_ =
      declare_parameter<std::string>("feedback_frame_id", "base_link_hf");
    const std::string feedback_topic =
      declare_parameter<std::string>(
      "feedback_topic", "chassis_feedback");

    if (read_period_ms <= 0) {
      throw std::invalid_argument("read_period_ms must be positive");
    }

    feedback_publisher_ =
      create_publisher<robot_interfaces::msg::ChassisFeedback>(
      feedback_topic, rclcpp::SensorDataQoS());
    command_subscription_ =
      create_subscription<robot_interfaces::msg::ChassisCmd>(
      "cmd_chassis", 10,
      std::bind(
        &Usb2UartTransNode::commandCallback, this,
        std::placeholders::_1));

    initSerial(port_name_, baud_rate);
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(read_period_ms),
      std::bind(&Usb2UartTransNode::readSerial, this));

    RCLCPP_INFO(
      get_logger(),
      "STM32 serial bridge: port=%s baud=%d feedback=%s read_period=%d ms",
      port_name_.c_str(), baud_rate, feedback_topic.c_str(),
      read_period_ms);
  }

  ~Usb2UartTransNode() override
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      RCLCPP_INFO(get_logger(), "STM32 serial port closed");
    }
  }

private:
  bool initSerial(const std::string & port_name, int baud_rate)
  {
    serial_fd_ = open(
      port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "cannot open serial port %s: %s",
        port_name.c_str(), std::strerror(errno));
      return false;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(), "cannot read serial attributes: %s",
        std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    speed_t speed = B115200;
    switch (baud_rate) {
      case 9600:
        speed = B9600;
        break;
      case 115200:
        speed = B115200;
        break;
      case 460800:
        speed = B460800;
        break;
      case 921600:
        speed = B921600;
        break;
      default:
        RCLCPP_ERROR(get_logger(), "unsupported baud rate: %d", baud_rate);
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcflush(serial_fd_, TCIFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(), "cannot set serial attributes: %s",
        std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "serial port %s opened at %d baud",
      port_name.c_str(), baud_rate);
    return true;
  }

  void commandCallback(
    const robot_interfaces::msg::ChassisCmd::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "serial port is not ready");
      return;
    }

    ChassisCmdFrame frame{};
    frame.header = 0xA5U;
    frame.enable = msg->enable;
    frame.protect = msg->protect;
    frame.vel_x = msg->vx / kMaxSpeed * 128.0F;
    frame.vel_y = msg->vy / kMaxSpeed * 128.0F;
    frame.vel_w = msg->vw / kMaxOmega * 128.0F;
    frame.tail = 0x5AU;

    if (!writeFrame(
        reinterpret_cast<const std::uint8_t *>(&frame), sizeof(frame)))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "failed to send a complete cmd_chassis frame");
    }
  }

  bool writeFrame(const std::uint8_t * data, std::size_t size)
  {
    std::size_t sent = 0U;
    while (sent < size) {
      const ssize_t result = write(serial_fd_, data + sent, size - sent);
      if (result > 0) {
        sent += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return false;
      }
      if (result < 0) {
        RCLCPP_ERROR(
          get_logger(), "serial write failed: %s",
          std::strerror(errno));
      }
      return false;
    }
    return true;
  }

  void readSerial()
  {
    if (serial_fd_ < 0) {
      return;
    }

    std::array<std::uint8_t, 256U> bytes{};
    while (true) {
      const ssize_t count = read(serial_fd_, bytes.data(), bytes.size());
      if (count > 0) {
        const auto samples = feedback_parser_.append(
          bytes.data(), static_cast<std::size_t>(count));
        for (const auto & sample : samples) {
          publishFeedback(sample);
        }
        continue;
      }
      if (count == 0) {
        break;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "serial read failed: %s", std::strerror(errno));
      break;
    }
  }

  void publishFeedback(
    const robot_comm::chassis_feedback::VelocitySample & sample)
  {
    robot_interfaces::msg::ChassisFeedback feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = feedback_frame_id_;
    feedback.vx = sample.vx;
    feedback.vy = sample.vy;
    feedback.vw = sample.vw;
    feedback_publisher_->publish(feedback);
  }

  std::string port_name_;
  std::string feedback_frame_id_;
  int serial_fd_{-1};
  robot_comm::chassis_feedback::StreamParser feedback_parser_;
  rclcpp::Subscription<robot_interfaces::msg::ChassisCmd>::SharedPtr
    command_subscription_;
  rclcpp::Publisher<robot_interfaces::msg::ChassisFeedback>::SharedPtr
    feedback_publisher_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Usb2UartTransNode>("usb2uart_node");
  RCLCPP_INFO(node->get_logger(), "usb2uart_node has started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
