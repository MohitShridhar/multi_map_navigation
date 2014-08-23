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
#include <math.h>
#include <iostream>

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
#include <map_store/DeleteMap.h>
#include <map_store/PublishMap.h>
#include <map_store/RenameMap.h>
#include <map_store/SaveMap.h>
#include <map_store/SetOrigin.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "map_server.cpp"

#endif


class QLineEdit;
class QComboBox;
class QHBoxLayout;
class QVBoxLayout;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QLabel;
class QDoubleSpinBox;
class QFileDialog;

#define NAMESPACE_TOPIC "/namespace_mux/control"

namespace rviz
{

  class MapManager: public rviz::Panel
  {

  Q_OBJECT
  public:
    MapManager( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  public Q_SLOTS:


  protected Q_SLOTS:
    void deleteMap();
    void renameMap();
    void setPrimaryMap();
    void setSecondaryMap();
    void setSecondaryMapOrigin();
    void autoAlignMaps();
    void addNewMap();

  protected:
    QVBoxLayout *db_status_layout, *map_tree_layout, *combo_box_layout, *pos_editor_layout, *auto_align_layout;
    QHBoxLayout *btn_layout;
    QLabel *db_status_display_;
    QTreeWidget *map_tree;
    QPushButton *btn_add, *btn_delete, *btn_set_origin, *btn_set_primary, *btn_set_secondary, *btn_auto_align;
    QComboBox *primary_map_combo, *secondary_map_combo;
    
    QDoubleSpinBox *x_man_editor, *y_man_editor, *theta_man_editor;
    QDoubleSpinBox *x_auto_pri_editor, *y_auto_pri_editor, *theta_auto_pri_editor;
    QDoubleSpinBox *x_auto_sec_editor, *y_auto_sec_editor, *theta_auto_sec_editor;

  private:
    void setupStatusDisplay();
    void setupMainGuiPanel();
    void setupMapTree();
    void setupAddDeleteBtn();
    void setupBoxCombos();
    void setupPosEditor();
    void setupAutoAligner();
    void setupConnections();
    QDoubleSpinBox* addSpinboxEditor(QHBoxLayout *parent_layout, QDoubleSpinBox *spinbox, std::string label);
    void addMapTreeItem(std::string map_name, std::string map_id);

    void clearMapTree();
    void initialize();
    void setupMapServices();
    void reloadMapListAndCombo();

    std::string getMapId(std::string map_name);
    void setMap(std::string id, ros::ServiceClient client);
    void loadMapListAndCombo();
    void loadOrigin();
    void loadCombos();
    void secondary_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void createStringChild(QTreeWidgetItem *parent, std::string name, std::string value_string);
    void createNewOffsetMap();
    void renameUntiledMaps();

    bool get_map_cb(nav_msgs::GetMap::Request &request, nav_msgs::GetMap::Response &response);
    void pri_comm_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void sec_comm_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    float getYawFromQuat(float x, float y, float z, float w);

    ros::ServiceClient pri_list_map_client, pri_set_map_client, pri_delete_map_client, pri_rename_map_client;
    ros::ServiceClient sec_set_map_client, sec_delete_map_client, sec_save_map_client, sec_set_origin_client;
    ros::ServiceServer sec_set_map_server;

    ros::NodeHandle *rosNode;
    ros::Subscriber secondary_map_sub, pri_comm_pose_sub, sec_comm_pose_sub;
    ros::Publisher save_map_pub;

    std::string primaryMapNs, secondaryMapNs, last_secondary_map_id, last_secondary_map_name;
    std::vector<std::string> alignerNamespaces;
    std::vector<map_store::MapListEntry> map_list;
    MapServer *map_server;

    nav_msgs::OccupancyGrid secondary_map, new_offset_map;

  };

}
