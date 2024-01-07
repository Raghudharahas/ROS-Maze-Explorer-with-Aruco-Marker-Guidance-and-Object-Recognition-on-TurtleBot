#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <listener_sub.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <utils.hpp>
#include<thread>
#include <chrono>
#include <tf2/exceptions.h>
// allows to use, 50ms, etc
using namespace std::chrono_literals;

// Method to listen to the transform between two frames
void ListenerSub::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;

    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }
    //Extract the position and orientation from the transform
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;
}
 // callback for the lsiten timer
void ListenerSub::listen_timer_cb_()
{
    // listen_transform("camera_rgb_optical_frame", "aruco_markers");
    listen_transform("odom", "aruco_markers");
}
// callback for receiving odom data
void ListenerSub::get_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_data = *msg;
    listen_timer_cb_();
}
// callback for receiving aruco data
void ListenerSub::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    aruco_pos_data = *msg;
}
// callback for second listen timer
void ListenerSub::listen_timer2_cb_()
{
    //calculate distance between robot and aruco
    double distance = sqrt(pow(pose_out.position.x - odom_data.pose.pose.position.x, 2) + pow(pose_out.position.y - odom_data.pose.pose.position.y, 2));
    //log the distance
    // if the distance is greater than 1.0, move forward
    if (distance > 1.0)
    {
        move_forward();
        // RCLCPP_INFO(this->get_logger(), "Moving forward");
    }
    else
    {
        // otherwise stop and check the aruco marker id to decide the next action
        stop();
        // the following checks the marker id and the corespoinding action to be taken whethe to moce left or right or stop
        if (aruco_pos_data.marker_ids[0] == 0)
        {
            if (aruco_marker_0 == "right_90")
            {   
                RCLCPP_INFO(this->get_logger(), "Turning right");
                turn_right();
            }
            else if (aruco_marker_0 == "left_90")
            {
                RCLCPP_INFO(this->get_logger(), "Turning left");
                turn_left();
            }
            else if (aruco_marker_0 == "end")
            {
                // RCLCPP_INFO(this->get_logger(), "Stopping");
                stop();
                end_of_maze(1);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Invalid aruco_marker_0");
            }
        }
        else if (aruco_pos_data.marker_ids[0] == 1)
        {
            if (aruco_marker_1 == "right_90")
            {
                RCLCPP_INFO(this->get_logger(), "Turning right");
                turn_right();
            }
            else if (aruco_marker_1 == "left_90")
            {
                RCLCPP_INFO(this->get_logger(), "Turning left");
                turn_left();
            }
            else if (aruco_marker_1 == "end")
            {
                // RCLCPP_INFO(this->get_logger(), "Stopping");
                stop();
                end_of_maze(1);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Invalid aruco_marker_1");
            }
        }
        else if (aruco_pos_data.marker_ids[0] == 2)
        {
            if (aruco_marker_2 == "right_90")
            {
                RCLCPP_INFO(this->get_logger(), "Turning right");
                turn_right();
            }
            else if (aruco_marker_2 == "left_90")
            {
                RCLCPP_INFO(this->get_logger(), "Turning left");
                turn_left();
            }
            else if (aruco_marker_2 == "end")
            {
                // RCLCPP_INFO(this->get_logger(), "Stopping");
                stop();
                end_of_maze(1);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Invalid aruco_marker_2");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid aruco_marker");
        }
    }
}
//method to move the robot forward
void ListenerSub::move_forward()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.4;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;
    vel_publisher_->publish(vel_msg);
    
}
//method to turn the robot left
void ListenerSub::turn_left()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.1;
    vel_publisher_->publish(vel_msg);
    std::this_thread::sleep_for(std::chrono::seconds(8)); 
}
//method to turn the robot right
void ListenerSub::turn_right()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = -0.1;
    vel_publisher_->publish(vel_msg);
    std::this_thread::sleep_for(std::chrono::seconds(8)); 

}
//method to publish a message to stop the robot
void ListenerSub::stop()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;
    vel_publisher_->publish(vel_msg);
}
void ListenerSub::end_of_maze(int value)
{
    std_msgs::msg::Int32 endpoint;
    endpoint.data = value;
    end_of_maze_->publish(endpoint);
}
// main function
int main(int argc, char **argv)
{
    // initialize the node
    rclcpp::init(argc, argv);
    // create a node
    auto node = std::make_shared<ListenerSub>("listener_sub");
    // spin the node
    rclcpp::spin(node);
    // shutdown the node
    rclcpp::shutdown();
    
}