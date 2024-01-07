#include <battery.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
// needed for the listener
#include <tf2/exceptions.h>
#include <vector>
#include <algorithm>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

// subscriber callback to the clock
void BatteryNode::clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  clock_time = *msg;
  double broadcast_time = clock_time.clock.sec;
  rclcpp::Time battery_time(broadcast_time, clock_time.clock.nanosec);
}

// stores unique battery vale
void BatteryNode::addUniqueValue(int value, double b_x, double b_y, double b_z, double o_x, double o_y, double o_z, double o_w)
{

  if (std::find(values.begin(), values.end(), value) == values.end())
  {

    values.push_back(value);
    std::vector<double> b_pose{b_x, b_y, b_z, o_x, o_y, o_z, o_w};
    floating_battery_pose.push_back(b_pose);
  }
}
// function to end the maze and print batteries detected
void BatteryNode::end_maze_cb(const std_msgs::msg::Int32::SharedPtr msg)
{
  end_point_reached = *msg;
  if (end_point_reached.data == 1 && print_once)
  {
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");

    // prints final stored battery values
    for (int i = 0; i < values.size(); i++)
    {
      if (values[i] == int(mage_msgs::msg::Part::BLUE))
      {
        RCLCPP_INFO(this->get_logger(), "Blue battery detected at xyz=[%f,%f,%f] rpy=[%f,%f,%f]", floating_battery_pose[i][0], floating_battery_pose[i][1], floating_battery_pose[i][2], floating_battery_pose[i][3], floating_battery_pose[i][4], floating_battery_pose[i][5], floating_battery_pose[i][6]);
      }
      else if (values[i] == int(mage_msgs::msg::Part::GREEN))
      {
        RCLCPP_INFO(this->get_logger(), "Green battery detected at xyz=[%f,%f,%f] rpy=[%f,%f,%f]", floating_battery_pose[i][0], floating_battery_pose[i][1], floating_battery_pose[i][2], floating_battery_pose[i][3], floating_battery_pose[i][4], floating_battery_pose[i][5], floating_battery_pose[i][6]);
      }
      else if (values[i] == int(mage_msgs::msg::Part::RED))
      {
        RCLCPP_INFO(this->get_logger(), "Red battery detected at xyz=[%f,%f,%f] rpy=[%f,%f,%f]", floating_battery_pose[i][0], floating_battery_pose[i][1], floating_battery_pose[i][2], floating_battery_pose[i][3], floating_battery_pose[i][4], floating_battery_pose[i][5], floating_battery_pose[i][6]);
      }
      else if (values[i] == int(mage_msgs::msg::Part::ORANGE))
      {
        RCLCPP_INFO(this->get_logger(), "Orange battery detected at xyz=[%f,%f,%f] rpy=[%f,%f,%f]", floating_battery_pose[i][0], floating_battery_pose[i][1], floating_battery_pose[i][2], floating_battery_pose[i][3], floating_battery_pose[i][4], floating_battery_pose[i][5], floating_battery_pose[i][6]);
      }
      else if (values[i] == int(mage_msgs::msg::Part::PURPLE))
      {
        RCLCPP_INFO(this->get_logger(), "Purple battery detected at xyz=[%f,%f,%f] rpy=[%f,%f,%f]", floating_battery_pose[i][0], floating_battery_pose[i][1], floating_battery_pose[i][2], floating_battery_pose[i][3], floating_battery_pose[i][4], floating_battery_pose[i][5], floating_battery_pose[i][6]);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "No battery detected");
      }
      geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    }
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");

    // shutdown node after printing
    rclcpp::shutdown();
  }
  print_once = false;
}

// timer callback for the broadcaster
void BatteryNode::broadcast_timer_cb_()
{

  // if broadcaster flag is true, then broadcast
  if (broadcast_flag == true)
  {

    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    double broadcast_time = clock_time.clock.sec;
    rclcpp::Time battery_time(broadcast_time, clock_time.clock.nanosec);

    geometry_msgs::msg::TransformStamped t_stamped_broadcast;
    dynamic_transform_stamped.header.stamp = battery_time;
    dynamic_transform_stamped.header.frame_id = "logical_camera_link";
    dynamic_transform_stamped.child_frame_id = "battery_frame";

    dynamic_transform_stamped.transform.translation.x = battery_data.part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = battery_data.part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = battery_data.part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = battery_data.part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = battery_data.part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = battery_data.part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = battery_data.part_poses[0].pose.orientation.w;
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);
  }
}

// callback function to subscribe logical camera
void BatteryNode::battery_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

  battery_data = *msg;
  // if there are no parts, dont broadcast
  if (battery_data.part_poses.size() == 0)
  {
    broadcast_flag = false;
  }
  else
  {
    broadcast_flag = true;
  }
}

// function that listens to the transform
void BatteryNode::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
  if (broadcast_flag == true)
  {
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose battery_pose_out;
    try
    {
      t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 100ms);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
      return;
    }

    battery_pose_out.position.x = t_stamped.transform.translation.x;
    battery_pose_out.position.y = t_stamped.transform.translation.y;
    battery_pose_out.position.z = t_stamped.transform.translation.z;
    battery_pose_out.orientation = t_stamped.transform.rotation;
    // store the unique battery detected
    addUniqueValue(battery_data.part_poses[0].part.color, battery_pose_out.position.x, battery_pose_out.position.y, battery_pose_out.position.z, battery_pose_out.orientation.x, battery_pose_out.orientation.y, battery_pose_out.orientation.z, battery_pose_out.orientation.w);
  }
}

// timer callback to listen to the transform between two frames
void BatteryNode::listen_timer_cb_()
{
  // calls listen_transform  function to listen between given frames
  listen_transform("odom", "battery_frame");
}

// main function to start the node
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryNode>("battery_data");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
