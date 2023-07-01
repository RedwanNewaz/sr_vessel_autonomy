//
// Created by redwan on 6/30/23.
//
#include "ThreadedSocketClient.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include <fmt/core.h>
#include <memory>

using namespace std::chrono_literals;
std::queue<std::string> payload;

namespace sea_robotics
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    class VesselAutonomy : public rclcpp::Node
    {
    public:
        VesselAutonomy(const std::string &nodeName):Node(nodeName)
        {
            this->declare_parameter("IP", "127.0.0.1");
            this->declare_parameter("PORT", 8080);

            auto ip = this->get_parameter("IP").get_parameter_value().get<std::string>();
            auto port = this->get_parameter("PORT").get_parameter_value().get<int>();
            srClient_ = std::make_unique<ThreadedSocketClient>(ip, port);

            cmd_str_msg_ = "$PSEAC,T,,{},{},*12<CR><LF>";
            cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(
                    &VesselAutonomy::cmd_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(get_logger(), "%s initialized", nodeName.c_str());


        }

        ~VesselAutonomy()
        {
            srClient_->Disconnect();
        }
        /**
         * Thruster mode: Control thrusters. Use fields 3 and 4 to
         * set the commanded thrust percent and thrust angle or difference.
         * e.g.,  $PSEAC,T,,20,-10,*12<CR><LF>
         * @param msg geometry_msgs/msg/twist
         */
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg)
        {
            auto radiansToDegrees = [](double radians) {
                return radians * (180.0 / M_PI);
            };
            double thrust_percent = 100.0 * std::min(msg->linear.x, 1.0);
            double thrust_angle = radiansToDegrees(msg->angular.z);
            std::string new_cmd = fmt::format(cmd_str_msg_, thrust_percent, thrust_angle);
            payload.push(new_cmd);
        }
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        std::string cmd_str_msg_;
        std::unique_ptr<ThreadedSocketClient> srClient_;

    };


}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto vesselAutonomy = std::make_shared<sea_robotics::VesselAutonomy> ("vesselAutonomy");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(vesselAutonomy);
    executor.spin();
    rclcpp::shutdown();


    return 0;
}

