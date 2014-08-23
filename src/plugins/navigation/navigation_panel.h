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
#include <ros/ros.h>

#include <rviz/panel.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <multi_map_navigation/MultiMapNavigationGoal.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <multi_map_navigation/MultiMapNavigationAction.h>
#include <multi_map_navigation/SetMap.h>
#include <actionlib/client/simple_action_client.h>
#include <map_store/ListMaps.h>
#include <map_store/MapListEntry.h>

#endif


class QLineEdit;
class QComboBox;
class QHBoxLayout;
class QVBoxLayout;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QLabel;


#define MUX_CONTROL_TOPIC "/namespace_mux/control"
#define MULTI_MAP_POS_EST_TOPIC "/multi_map_pose"
#define MARKER_PUB_TOPIC "/flagged_locations"
#define MARKER_LABEL_PUB_TOPIC "/flagged_location_labels"
#define MAP_NAME_TOPIC "/current_map_name"
#define MOVE_GOAL_SERVER_NAME "/multi_map_navigation/move"
#define CANCEL_GOAL_TOPIC_NAME "/move_base/cancel"

namespace rviz
{

  class Navigator: public rviz::Panel
  {

  typedef actionlib::SimpleActionClient<multi_map_navigation::MultiMapNavigationAction> Client;

  Q_OBJECT
  public:
    Navigator( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  public Q_SLOTS:
    void setNamespace();


  protected Q_SLOTS:
    void updateVizState();
    void deleteFlaggedLoc();
    void setGoal();
    void cancelGoal();
    void setMap();


  protected:
    QComboBox *ns_combo_box_, *goal_combo_box_, *map_combo_box_;
    QHBoxLayout *mux_controller_layout, *add_delete_layout, *goto_cancel_layout;
    QVBoxLayout *loc_tree_layout, *goal_combo_layout, *map_combo_layout;
    QTreeWidget *flagged_locations_tree;
    QLabel *map_title_;

    QPushButton *btn_add_, *btn_delete_, *btn_goto_, *btn_cancel_, *btn_set_map_;

    ros::NodeHandle* rosNode;
    ros::Publisher mux_control_pub, marker_arr_pub, marker_label_arr_pub, cancel_goal_pub;
    ros::Subscriber multi_map_pose_est_sub, current_map_name_sub;

  private:
    std::vector<std::string> active_bots; 
    std::string outgoing_ns, current_map_name;
    std_msgs::String currNamespace;
    bool hasMapInitialized;

    visualization_msgs::MarkerArray marker_arr, marker_label_arr;
    
    multi_map_navigation::MultiMapNavigationGoal flaggedLoc;
    std::vector<map_store::MapListEntry> map_list;
    Client* goalClient;

    ros::ServiceClient list_map_client, set_map_client;


    void initialize();
    void setupActionlibClient();

    void setupNode();
    void setupMuxControl();
    void setupMainGuiPanel();
    void setupLocTree();
    void setupDeleteBtn();
    void setupGoalCombo();
    void setupMapCombo();
    void setupGotoCancel();
    void setupMapServices();

    void establishConnections();
    void pose_cb(const multi_map_navigation::MultiMapNavigationGoal::ConstPtr& msg);
    void map_name_cb(const std_msgs::String::ConstPtr& msg);

    void appendItem(float pose_x, float pose_y, float pose_yaw, std::string map_name);
    void createDataChild(QTreeWidgetItem *parent, std::string name, float value);
    void createStringChild(QTreeWidgetItem *parent, std::string name, std::string value_string);
    void addSubChildren(QTreeWidgetItem *item, rviz::Config flag) const;
    void loadFlagProps(rviz::Config flag, QTreeWidgetItem *item);
    void initializeMap(std::string init_map_name);

    void publish_markers();
    void delete_all_markers();

    void loadGoalComboBox();
    void loadMapComboBox();
    
  };

} // end namespace rviz_plugin_tutorials
