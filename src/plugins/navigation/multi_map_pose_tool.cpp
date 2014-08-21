// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

#include "multi_map_pose_tool.h"

namespace rviz
{

MultiMapPoseTool::MultiMapPoseTool()
{
  shortcut_key_ = 'f';

  nh_ = new ros::NodeHandle("");
  this->current_robot_ns = "robot"; // Initial control namespace
  this->current_map = "First_Floor"; // Initial map name 
  this->seqCount = 0;

  pub_topic_property_ = new StringProperty( "Topic", "multi_map_pose",
                                        "The topic on which to publish pose estimates",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );

}

void MultiMapPoseTool::onInitialize()
{
  PoseTool::onInitialize();

  setName( "Multi-Map Pose Flag" );
  updateTopic();
}

void MultiMapPoseTool::updateTopic()
{
  pub_ = nh_->advertise<multi_map_navigation::MultiMapNavigationGoal>( pub_topic_property_->getStdString(), 1 );
  sub_current_namespace = nh_->subscribe<std_msgs::String>(MUX_CONTROL_TOPIC, 2, &MultiMapPoseTool::ns_callback, this);
}

void MultiMapPoseTool::ns_callback(const std_msgs::String::ConstPtr& msg)
{
  if (this->current_robot_ns.compare(msg->data) != 0) {
    
    sub_current_map_name.shutdown();
    this->current_robot_ns = msg->data;

    sub_current_map_name = nh_->subscribe<std_msgs::String>(this->current_robot_ns + MAP_NAME_TOPIC, 5, &MultiMapPoseTool::current_map_callback, this);
  }
}

void MultiMapPoseTool::current_map_callback(const std_msgs::String::ConstPtr& msg)
{
  this->current_map = msg->data;
}

void MultiMapPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = this->current_robot_ns + "/map";

  multi_map_navigation::MultiMapNavigationGoal multiMapPos;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.header.seq = this->seqCount;

  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat, pose.pose.orientation);
  
  // pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  // pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  // pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

  multiMapPos.target_pose = pose;
  multiMapPos.goal_map = this->current_map;

  this->seqCount++;
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());

  pub_.publish(multiMapPos);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MultiMapPoseTool, rviz::Tool )