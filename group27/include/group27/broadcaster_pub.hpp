#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
using namespace std::chrono_literals;

/**
 * @class BroadcasterPub
 * @brief A ROS2 node that broadcasts transforms.
 *
 * The BroadcasterPub class declares and gets parameters for aruco markers.
 * It also creates a buffer of transforms and a TransformBroadcaster.
 */
class BroadcasterPub : public rclcpp::Node
{
public:
    BroadcasterPub(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster started");

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();
        
        aruco_subscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10, std::bind(&BroadcasterPub::arucoCallback, this, std::placeholders::_1));   
    }


private:
    /**
     * @brief  Boolean to decide whether to execute the broadcaster or not
     * 
     */
    bool param_broadcast_;
    /**
     * @brief  Buffer that stores several seconds of transforms for easy lookup by the listener.
     * 
     */

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /**
     * @brief  broadcaster object to broadcast the transform
     * 
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /**
     * @brief utils object to use the utility functions
     * 
     */

    std::shared_ptr<Utils> utils_ptr_;
    /**
     * @brief wall timer object for the broadcaster
     * 
     */
    rclcpp::TimerBase::SharedPtr broadcast_timer_;

    /**
     * @brief Aruco marker position data received from the subscriber
     * 
     */
    ros2_aruco_interfaces::msg::ArucoMarkers aruco_pos_data;
    /**
     * @brief subscriber object for the aruco pose  
     * 
     */
   
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    /**
     * @brief callback function for the aruco pose subscriber
     * 
     * 
     */
    void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void broadcast_timer_cb_();
    /**
     * @brief Timer to broadcast the transform
     *
     */
    void static_broadcast_timer_cb_();
};
