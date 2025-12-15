/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include <xv_11_laser_driver/xv11_laser.h>
#include <boost/asio.hpp>

class NeatoLaserPublisher : public rclcpp::Node
{
public:
  NeatoLaserPublisher() : Node("neato_laser_publisher")
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::string>("frame_id", "neato_laser");
    this->declare_parameter<int>("firmware_version", 2);

    this->get_parameter("port", port_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("firmware_version", firmware_version_);

    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt16>("rpms", 10);

    try {
      laser_ = std::make_shared<xv_11_laser_driver::XV11Laser>(port_, baud_rate_, firmware_version_, io_);
    } catch (boost::system::system_error &ex) {
      RCLCPP_ERROR(this->get_logger(), "Error instantiating laser object: %s", ex.what());
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NeatoLaserPublisher::publish_scan, this)
    );
  }

private:
  void publish_scan()
  {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = frame_id_;
    scan->header.stamp = this->now();

    laser_->poll(scan);

    std_msgs::msg::UInt16 rpms_msg;
    rpms_msg.data = laser_->rpms;

    laser_pub_->publish(*scan);
    motor_pub_->publish(rpms_msg);
  }

  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  int firmware_version_;

  boost::asio::io_service io_;
  std::shared_ptr<xv_11_laser_driver::XV11Laser> laser_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeatoLaserPublisher>());
  rclcpp::shutdown();
  return 0;
}
