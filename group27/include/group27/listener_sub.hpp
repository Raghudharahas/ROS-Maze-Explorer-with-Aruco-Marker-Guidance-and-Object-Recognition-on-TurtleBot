#pragma once

#include <cmath>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
// needed for the listener
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <std_msgs/msg/int32.hpp>
using namespace std::chrono_literals;

/**
 * @class ListenerSub
 * @brief A ROS2 node that listens to transforms and parameters.
 *
 * The ListenerSub class declares and gets parameters for aruco markers.
 * It also creates a buffer of transforms and a TransformListener.
 */
class ListenerSub : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Listener Sub object
     * 
     * 
     */
    ListenerSub(std::string node_name) : Node(node_name)
    {
        
        RCLCPP_INFO(this->get_logger(), "Listener started");
        
        /**
         * @brief  Buffer that stores several seconds of transforms for easy lookup by the listener.
         * 
         */
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());

        transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        /**
         * @brief  Declare and get parameters for aruco markers.
         * @params aruco_marker_0, aruco_marker_1, aruco_marker_2
         * 
         */
        this->declare_parameter("aruco_marker_0", "right_90");
        this->declare_parameter("aruco_marker_1", "left_90");
        this->declare_parameter("aruco_marker_2", "end");
        aruco_marker_0 = this->get_parameter("aruco_marker_0").as_string();
        aruco_marker_1 = this->get_parameter("aruco_marker_1").as_string();
        aruco_marker_2 = this->get_parameter("aruco_marker_2").as_string();

        // listen_timer_ = this->create_wall_timer(1s, std::bind(&ListenerSub::listen_timer_cb_, this));
        /**
         * @brief  Subscribe to the odom topic.
         * 
         */
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ListenerSub::get_odom, this, std::placeholders::_1));   

        /**
         * @brief  Subscribe to the aruco_markers topic.
         * 
         */
        aruco_subscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10, std::bind(&ListenerSub::arucoCallback, this, std::placeholders::_1));
        /**
         * @brief  Create a wall timer object.
         * 
         */
        listen_timer2_ = this->create_wall_timer(100ms, std::bind(&ListenerSub::listen_timer2_cb_, this));
        /**
         * @brief  Create a Velocity publisher object.
         * 
         */
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        /**
         * @brief  Create a end_of_maze publisher object.
         * 
         */
        end_of_maze_= this->create_publisher<std_msgs::msg::Int32>("end_of_maze", 10);
        end_of_maze(0);
        
    }

private:
     /**
      * @brief Boolean parameter to whether or not start the listener
      * 
      */
    bool param_listen_;
    /**
     * @brief Buffer that stores several seconds of transforms for easy lookup by the listener.
     * 
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /**
     * @brief Transform listener object
     * 
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    /**
     * @brief  A shared pointer to the timerbase object.
     * 
     */
    rclcpp::TimerBase::SharedPtr listen_timer_;
    rclcpp::TimerBase::SharedPtr listen_timer2_;
    /**
     * @brief  publisher object for the velocity.
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_; 
    /**
     * @brief end of maze 
     * 
     * 
     */
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr end_of_maze_;
    /**
     * @brief odometry data received from the odom topic.
     * 
     */
    nav_msgs::msg::Odometry odom_data;
    /**
     * @brief  pose data to be published.
     * 
     */
    geometry_msgs::msg::Pose pose_out;
    /**
     * @brief Aruco marker position data received from the subscription.
     * 
     */
    ros2_aruco_interfaces::msg::ArucoMarkers aruco_pos_data;
    /**
     * @brief subscription object to subscribe to the aruco_markers topic.
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    /**
     * @brief callback function for the aruco_markers topic.
     * 
     * @param msg 
     */
    void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    /**
     * @brief subscription object to subscribe to odometry data
     * 
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    /**
     * @brief callback function for odometry data subscription.
     * 
     * @param msg 
     */
    void get_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    /**
     * @brief callback function for the timer.
     * 
     */
    void listen_timer2_cb_();
    /**
     * @brief Parameter for the aruco marker 0.
     * 
     */
    std::string aruco_marker_0;
    /**
     * @brief Parameter for the aruco marker 1.
     * 
     */
    std::string aruco_marker_1;
    /**
     * @brief Parameter for the aruco marker 2.
     * 
     */
    std::string aruco_marker_2;
    /**
     * @brief function to move the robot forward.
     * 
     */
    void move_forward();
    /**
     * @brief function to turn the robot left.
     * 
     */
    void turn_left();
    /**
     * @brief function to turn the robot right.
     * 
     */
    void turn_right();
    /**
     * @brief function to stop the robot.
     * 
     */
    void stop();
    void end_of_maze(int value);
    
    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to listen to the transform
     *
     */
    void listen_timer_cb_();
};
