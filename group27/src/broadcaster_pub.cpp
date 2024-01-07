#include <broadcaster_pub.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

//callback function for the broadcast timer
void BroadcasterPub::broadcast_timer_cb_()
{
    // create a transformStamped message
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    // get the current time
    double broadcast_time = aruco_pos_data.header.stamp.sec;
    //converr the time to rclcpp::Time
    rclcpp::Time broadcast_time_rclcpp(broadcast_time,aruco_pos_data.header.stamp.nanosec);
    dynamic_transform_stamped.header.stamp = broadcast_time_rclcpp;

    // Set the frame_id and child_frame_id
    dynamic_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
    dynamic_transform_stamped.child_frame_id = "aruco_markers";
    //set the translation of the transform from the aruco marker data
    dynamic_transform_stamped.transform.translation.x =aruco_pos_data.poses[0].position.x;
    dynamic_transform_stamped.transform.translation.y = aruco_pos_data.poses[0].position.y;
    dynamic_transform_stamped.transform.translation.z = aruco_pos_data.poses[0].position.z;
    //set the rotation of the transform from the aruco marker data
    // geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, M_PI / 2, M_PI / 3);
    dynamic_transform_stamped.transform.rotation.x = aruco_pos_data.poses[0].orientation.x;
    dynamic_transform_stamped.transform.rotation.y = aruco_pos_data.poses[0].orientation.y;
    dynamic_transform_stamped.transform.rotation.z = aruco_pos_data.poses[0].orientation.z;
    dynamic_transform_stamped.transform.rotation.w = aruco_pos_data.poses[0].orientation.w;
    // Send the transform
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);
}
//call back function for the aruco marker data 
void BroadcasterPub::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  //store the received aruco marker data
   aruco_pos_data = *msg;
    //call the broadcast timer callback function
   broadcast_timer_cb_();
  // RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
}

//main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BroadcasterPub>("broadcaster_pub");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
