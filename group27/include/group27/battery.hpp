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
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>

using namespace std::chrono_literals;

/**
 * @brief Class for the battery node
 *
 */
class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode(std::string node_name) : Node(node_name)
    {
        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        utils_ptr_ = std::make_shared<Utils>();

        // load a buffer of transforms
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // initialize the transform listener
        transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // flag to bradcast
        broadcast_flag = false;
        // flag to print battery statements onlt once
        print_once = true;

        // timer to publish the transform
        listener_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&BatteryNode::listen_timer_cb_, this));

        values = {};
        // initialize the subscriber clock
        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", rclcpp::SensorDataQoS(), std::bind(&BatteryNode::clock_cb, this, std::placeholders::_1));
        // initialize the subscriber for the battery
        battery_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image",
                                                                                                    rclcpp::SensorDataQoS(), std::bind(&BatteryNode::battery_cb, this, std::placeholders::_1));
        // timer for the broadcaster
        broadcast_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&BatteryNode::broadcast_timer_cb_, this));
        // initialize the subscriber for the end of maze
        end_maze_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("end_of_maze", rclcpp::SensorDataQoS(), std::bind(&BatteryNode::end_maze_cb, this, std::placeholders::_1));
    }

private:
    bool param_listen_;
    bool param_broadcast_;
    bool broadcast_flag;
    bool print_once;

    /**
     * @brief variable to set clock
     *
     */
    rosgraph_msgs::msg::Clock clock_time;
    /**
     * @brief variable to get battery data
     *
     */
    mage_msgs::msg::AdvancedLogicalCameraImage battery_data;
    /**
     * @brief variable to set end point reached
     *
     */
    std_msgs::msg::Int32 end_point_reached;

    /**
     * @brief variable to store battery values
     *
     */
    std::vector<int> values;
    /**
     * @brief variable to store battery poses
     *
     */
    std::vector<std::vector<double>> floating_battery_pose;

    /**
     * @brief variable to get battery pose
     *
     */
    geometry_msgs::msg::Pose battery_pose_out;

    /**
     * @brief   transform listener variable
     *
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    /**
     * @brief Buffer that stores several seconds of transforms for easy lookup by the listener.
     *
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief Static broadcaster object
     *
     */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    /**
     * @brief Broadcaster object
     *
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /**
     * @brief Utils object to access utility functions
     *
     */
    std::shared_ptr<Utils> utils_ptr_;

    /**
     * @brief subscriber to camera
     *
     */

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscriber_;

    /**
     * @brief subscriber to get clock
     *
     */
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    /**
     * @brief subscriber to get end maze value
     *
     */
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr end_maze_subscriber_;

    /*!< Boolean parameter to whether or not start the broadcaster */

    /**
     * @brief Wall timer object for the broadcaster
     *
     */
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    /**
     * @brief Wall timer object for the listener
     *
     */
    rclcpp::TimerBase::SharedPtr listener_timer_;
    /**
     * @brief Wall timer object for the static broadcaster
     *
     */
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;
    /**
     * @brief Wall timer object for the listener
     *
     */
    rclcpp::TimerBase::SharedPtr listen_timer_;

    /**
     * @brief subscriber callback to the clock
     *
     * @param msg
     */
    void clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    /**
     * @brief subscriber callback to the camera
     *
     * @param msg
     */
    void battery_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief stores all the unique batteries
     *
     * @param value
     * @param b_x
     * @param b_y
     * @param b_z
     * @param o_x
     * @param o_y
     * @param o_z
     * @param o_w
     */
    void addUniqueValue(int value, double b_x, double b_y, double b_z, double o_x, double o_y, double o_z, double o_w);
    /**
     * @brief prints final values after ending the maze
     *
     * @param msg
     */
    void end_maze_cb(const std_msgs::msg::Int32::SharedPtr msg);
    /**
     * @brief timer to call transform
     *
     */
    void listen_timer_cb_();

    /**
     * @brief listenes to the transform
     *
     * @param source_frame
     * @param target_frame
     */
    void listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void broadcast_timer_cb_();
};
