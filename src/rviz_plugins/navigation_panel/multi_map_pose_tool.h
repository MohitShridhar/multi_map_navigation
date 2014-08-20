#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <ros/ros.h>

# include "rviz/default_plugin/tools/pose_tool.h"
# include <std_msgs/String.h>
# include <std_msgs/Bool.h>
# include <multi_map_navigation/MultiMapNavigationGoal.h>

#endif

#define MUX_CONTROL_TOPIC "/rviz_mux/control"
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