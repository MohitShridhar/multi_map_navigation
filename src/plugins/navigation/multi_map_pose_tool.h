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

#ifndef Q_MOC_RUN
# include <QObject>

# include <ros/ros.h>

# include "rviz/default_plugin/tools/pose_tool.h"
# include <std_msgs/String.h>
# include <std_msgs/Bool.h>
# include <multi_map_navigation/MultiMapNavigationGoal.h>

#endif

#define MUX_CONTROL_TOPIC "/namespace_mux/control"
#define MAP_NAME_TOPIC "/current_map_name"

namespace rviz
{
  class Arrow;
  class DisplayContext;
  class StringProperty;

  class MultiMapPoseTool: public PoseTool
  {
    Q_OBJECT
    public:
      MultiMapPoseTool();
      virtual ~MultiMapPoseTool() {}
      virtual void onInitialize();

    protected:
      virtual void onPoseSet(double x, double y, double theta);

    private Q_SLOTS:
      void updateTopic();

    private:
      ros::NodeHandle* nh_;
      ros::Publisher pub_;
      ros::Subscriber sub_current_map_name, sub_current_namespace;

      StringProperty* pub_topic_property_;
      std::string current_map, current_robot_ns;
      uint seqCount;

      void ns_callback(const std_msgs::String::ConstPtr& msg);
      void current_map_callback(const std_msgs::String::ConstPtr& msg);
  };

}