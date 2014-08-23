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

#include <stdio.h>

#include <iostream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>
#include <QTreeWidget>
#include <QPushButton>

#include "navigation_panel.h"

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

namespace rviz
{

  Navigator::Navigator( QWidget* parent )
    : rviz::Panel( parent )
  {
  	setupNode();
  	
    setupMuxControl();
    setupLocTree();

    setupDeleteBtn();
    setupMapCombo();
    setupGoalCombo();
    setupGotoCancel();

  	setupMainGuiPanel();

  	establishConnections();
    initialize();
  }

  void Navigator::establishConnections()
  {
    flagged_locations_tree->blockSignals(true);  // safety: make sure 'data change' signals don't interfere with automatically appended locations

  	connect(ns_combo_box_, SIGNAL(activated(int)), this, SLOT(setNamespace(void)) );
    connect(flagged_locations_tree, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(updateVizState()) );
    connect(btn_delete_, SIGNAL(clicked(void)), this, SLOT(deleteFlaggedLoc()) );
    connect(btn_goto_, SIGNAL(clicked(void)), this, SLOT(setGoal()));
    connect(btn_cancel_, SIGNAL(clicked(void)), this, SLOT(cancelGoal()));
    connect(btn_set_map_, SIGNAL(clicked(void)), this, SLOT(setMap()));

    multi_map_pose_est_sub = this->rosNode->subscribe<multi_map_navigation::MultiMapNavigationGoal>(MULTI_MAP_POS_EST_TOPIC, 2, &Navigator::pose_cb, this);
    current_map_name_sub = this->rosNode->subscribe<std_msgs::String>(MAP_NAME_TOPIC, 2, &Navigator::map_name_cb, this);

    marker_arr_pub = this->rosNode->advertise<visualization_msgs::MarkerArray>(MARKER_PUB_TOPIC, 10);
    marker_label_arr_pub = this->rosNode->advertise<visualization_msgs::MarkerArray>(MARKER_LABEL_PUB_TOPIC, 10);
  }

  void Navigator::setupMapServices()
  {
    list_map_client = this->rosNode->serviceClient<map_store::ListMaps>("/" + currNamespace.data + "/multi_map_navigation/list_maps"); // Note: this list shouldn't change during the GUI runtime
    set_map_client = this->rosNode->serviceClient<multi_map_navigation::SetMap>("/" + currNamespace.data + "/multi_map_navigation/set_map");    
  }

  void Navigator::initialize()
  {
    setNamespace();
    setupMapServices();
    loadMapComboBox();

    hasMapInitialized = false;
  }

  void Navigator::setupActionlibClient()
  {
    goalClient = new Client("/" + this->currNamespace.data + MOVE_GOAL_SERVER_NAME, true);

    while(!goalClient->waitForServer(ros::Duration(2.0))) {
      ROS_INFO("Waiting for Goal Manager");
    }
  }

  void Navigator::updateVizState()
  {
    publish_markers();
    loadGoalComboBox();
  }

  void Navigator::setupDeleteBtn()
  {
    add_delete_layout = new QHBoxLayout;

    btn_delete_ = new QPushButton("Delete", this);
    add_delete_layout->addWidget(btn_delete_);
  }

  void Navigator::loadMapComboBox()
  {
    QStringList maps;
    map_combo_box_->clear();

    map_store::ListMaps srv;

    if (list_map_client.call(srv)) {
      map_list = srv.response.map_list;

      for (std::vector<map_store::MapListEntry>::iterator it = map_list.begin(); it != map_list.end(); ++it) {
        maps << QString::fromStdString((*it).name);
      }

    } else {
      ROS_ERROR("Could not retrieve a list of available maps from the database.");
    }

    map_combo_box_->addItems(maps);

  }

  void Navigator::setMap()
  {
    multi_map_navigation::SetMap targetMap;

    targetMap.request.name = map_combo_box_->currentText().toStdString();

    if (set_map_client.call(targetMap)) {
      ROS_INFO("Map set to : %s", targetMap.request.name.c_str());
    } else {
      ROS_ERROR("Could set the desired map");
    }

  }

  void Navigator::pose_cb(const multi_map_navigation::MultiMapNavigationGoal::ConstPtr& msg)
  {
    flaggedLoc = *msg;

    double roll, pitch, yaw;
    tf::Quaternion quat = tf::Quaternion(flaggedLoc.target_pose.pose.orientation.x, flaggedLoc.target_pose.pose.orientation.y, flaggedLoc.target_pose.pose.orientation.z, flaggedLoc.target_pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    appendItem(flaggedLoc.target_pose.pose.position.x, flaggedLoc.target_pose.pose.position.y, yaw, flaggedLoc.goal_map);
  }

  void Navigator::map_name_cb(const std_msgs::String::ConstPtr& msg)
  {
    if ((msg->data).compare(this->current_map_name) != 0) {
      this->current_map_name = msg->data;
      publish_markers();
    }

    map_title_->setText(QString::fromStdString("Set Map: (" + current_map_name + ")"));

    if (!hasMapInitialized) {
      initializeMap(msg->data);
    }
  }

  void Navigator::initializeMap(std::string init_map_name)
  {
    multi_map_navigation::SetMap initMap;

    initMap.request.name = init_map_name;

    if (set_map_client.call(initMap)) {
      ROS_INFO("Initializing map to %s", init_map_name.c_str());
    } else {
      ROS_INFO("Could not initialize map to %s", init_map_name.c_str());
    }

    hasMapInitialized = true;
  }

  void Navigator::appendItem(float pose_x, float pose_y, float pose_yaw, std::string map_name)
  {
    flagged_locations_tree->blockSignals(true);  //mutex lock

    QTreeWidgetItem *location = new QTreeWidgetItem(flagged_locations_tree);

    std::stringstream ss;
    ss << flagged_locations_tree->indexOfTopLevelItem(location) + 1;
    std::string index = ss.str();

    location->setText(0, QString::fromStdString("Flag " + index));
    location->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);

    createDataChild(location, "Pose X", pose_x);
    createDataChild(location, "Pose Y", pose_y);
    createDataChild(location, "Pose Yaw", pose_yaw);
    createStringChild(location, "Map", map_name);
    
    loadGoalComboBox();
    publish_markers(); 

    flagged_locations_tree->blockSignals(false);  

  }

  void Navigator::deleteFlaggedLoc()
  {
    flagged_locations_tree->blockSignals(true); 
    delete_all_markers();

    QTreeWidgetItem *selectedItem = flagged_locations_tree->currentItem();

    if (!selectedItem) { // nothing selected
      return;
    }

    // Check that the item actually exists in the tree:
    int index = flagged_locations_tree->indexOfTopLevelItem(selectedItem);
    if (index >= 0 && index < flagged_locations_tree->topLevelItemCount()){
      delete selectedItem;
    }

    loadGoalComboBox();
    publish_markers();

    flagged_locations_tree->blockSignals(false);  
  }

  void Navigator::setGoal()
  {
    int indexOfSelectedGoal = goal_combo_box_->currentIndex();
    QTreeWidgetItem *goalLocation = flagged_locations_tree->topLevelItem(indexOfSelectedGoal);

    multi_map_navigation::MultiMapNavigationGoal goal;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = this->currNamespace.data + "/map";

    goal.target_pose.pose.position.x = goalLocation->child(0)->child(0)->data(0, Qt::DisplayRole).toFloat();
    goal.target_pose.pose.position.y = goalLocation->child(1)->child(0)->data(0, Qt::DisplayRole).toFloat();

    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, goalLocation->child(2)->child(0)->data(0, Qt::DisplayRole).toFloat());

    goal.target_pose.pose.orientation.x = quat.x();
    goal.target_pose.pose.orientation.y = quat.y();
    goal.target_pose.pose.orientation.z = quat.z();
    goal.target_pose.pose.orientation.w = quat.w();

    goal.goal_map = goalLocation->child(3)->child(0)->data(0, Qt::DisplayRole).toString().toStdString();

    ROS_INFO("Sending goal");
    
    std_msgs::Bool cancel;
    cancel.data = false; 
    cancel_goal_pub.publish(cancel);

    goalClient->sendGoal(goal);
    // goalClient.waitForResult();
  }

  void Navigator::cancelGoal()
  {
    std_msgs::Bool cancel;
    cancel.data = true;
    cancel_goal_pub.publish(cancel);
  }

  void Navigator::delete_all_markers()
  {
    int numFlaggedLocs = flagged_locations_tree->topLevelItemCount();

    for (int i=0; i<numFlaggedLocs; i++) {
      marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
      marker_label_arr.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    marker_arr_pub.publish(marker_arr);
    marker_label_arr_pub.publish(marker_label_arr);
  }

  void Navigator::publish_markers()
  {
    int numFlaggedLocs = flagged_locations_tree->topLevelItemCount();

    marker_arr.markers.clear();
    marker_arr.markers.resize(numFlaggedLocs);

    marker_label_arr.markers.clear();
    marker_label_arr.markers.resize(numFlaggedLocs);

    for (int i=0; i<numFlaggedLocs; i++) {
      QTreeWidgetItem *item = flagged_locations_tree->topLevelItem(i);

      float pose_x = item->child(0)->child(0)->data(0, Qt::DisplayRole).toFloat();
      float pose_y = item->child(1)->child(0)->data(0, Qt::DisplayRole).toFloat();
      float pose_yaw = item->child(2)->child(0)->data(0, Qt::DisplayRole).toFloat();
      std::string map_name = item->child(3)->child(0)->data(0, Qt::DisplayRole).toString().toStdString();
      std::string loc_name = item->text(0).toStdString();

      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, pose_yaw);

      // Create cube markers to indicate flagged locations:
      marker_arr.markers[i].header.frame_id = currNamespace.data + "/map";
      marker_arr.markers[i].header.stamp = ros::Time();
      marker_arr.markers[i].ns = "";
      marker_arr.markers[i].id = i;
      marker_arr.markers[i].type = visualization_msgs::Marker::CUBE;
      
      if (current_map_name.compare(map_name) == 0) {
        marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
      } else {
        marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
      }

      marker_arr.markers[i].pose.position.x = pose_x;
      marker_arr.markers[i].pose.position.y = pose_y;
      marker_arr.markers[i].pose.position.z = 0.15;
      marker_arr.markers[i].pose.orientation.x = quat.x();
      marker_arr.markers[i].pose.orientation.y = quat.y();
      marker_arr.markers[i].pose.orientation.z = quat.z();
      marker_arr.markers[i].pose.orientation.w = quat.w();
      
      marker_arr.markers[i].scale.x = 0.3;
      marker_arr.markers[i].scale.y = 0.3;
      marker_arr.markers[i].scale.z = 0.3;

      marker_arr.markers[i].color.a = 0.7;
      marker_arr.markers[i].color.r = 0.4;
      marker_arr.markers[i].color.g = 0.0;
      marker_arr.markers[i].color.b = 0.8;

      // Create labels (optional view) for the cube markers
      marker_label_arr.markers[i].header.frame_id = currNamespace.data + "/map";
      marker_label_arr.markers[i].header.stamp = ros::Time();
      marker_label_arr.markers[i].ns = "";
      marker_label_arr.markers[i].id = i;
      marker_label_arr.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_label_arr.markers[i].text = loc_name;

      if (current_map_name.compare(map_name) == 0) {
        marker_label_arr.markers[i].action = visualization_msgs::Marker::ADD;
      } else {
        marker_label_arr.markers[i].action = visualization_msgs::Marker::DELETE;
      }
      
      marker_label_arr.markers[i].pose.position.x = pose_x + 0.7;
      marker_label_arr.markers[i].pose.position.y = pose_y;
      marker_label_arr.markers[i].pose.position.z = 0.5;

      marker_label_arr.markers[i].scale.x = 0.4;
      marker_label_arr.markers[i].scale.y = 0.4;
      marker_label_arr.markers[i].scale.z = 0.4;

      marker_label_arr.markers[i].color.a = 1.0;
      marker_label_arr.markers[i].color.r = 1.0;
      marker_label_arr.markers[i].color.g = 1.0;
      marker_label_arr.markers[i].color.b = 1.0;

    }


    marker_arr_pub.publish(marker_arr);
    marker_label_arr_pub.publish(marker_label_arr);

  }

  void Navigator::createDataChild(QTreeWidgetItem *parent, std::string name, float value)
  {
    QTreeWidgetItem *name_row = new QTreeWidgetItem(parent);
    name_row->setText(0, QString::fromStdString(name));

    QTreeWidgetItem *value_row = new QTreeWidgetItem(name_row);
    value_row->setData(0, Qt::DisplayRole, QVariant(value));
    value_row->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);
  }

  void Navigator::createStringChild(QTreeWidgetItem *parent, std::string name, std::string value_string)
  {
    QTreeWidgetItem *name_row = new QTreeWidgetItem(parent);
    name_row->setText(0, QString::fromStdString(name));

    QTreeWidgetItem *value_row = new QTreeWidgetItem(name_row);
    value_row->setText(0, QString::fromStdString(value_string));
    value_row->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);
  }


  void Navigator::setupGoalCombo()
  {
    goal_combo_layout = new QVBoxLayout;
    goal_combo_layout->addWidget(new QLabel("Set Goal:"));

    goal_combo_box_ = new QComboBox;

    goal_combo_layout->addWidget(goal_combo_box_);

  }

  void Navigator::loadGoalComboBox()
  {
    goal_combo_box_->clear();

    QStringList flaggedLocations;

    int numFlaggedLocs = flagged_locations_tree->topLevelItemCount();

    for (int i=0; i<numFlaggedLocs; i++) {
      QTreeWidgetItem *location = flagged_locations_tree->topLevelItem(i);
      flaggedLocations << location->text(0);
    }

    goal_combo_box_->addItems(flaggedLocations);
  }

  void Navigator::setupMapCombo()
  {
    map_combo_layout = new QVBoxLayout;
    map_title_ = new QLabel("Set Map:");
    map_combo_layout->addWidget(map_title_);

    map_combo_box_ = new QComboBox;
    
    QHBoxLayout *button_combo_layout = new QHBoxLayout;
    button_combo_layout->addWidget(map_combo_box_);

    btn_set_map_ = new QPushButton("Set", this);
    btn_set_map_->setMaximumWidth(60);
    button_combo_layout->addWidget(btn_set_map_);

    map_combo_layout->addLayout(button_combo_layout);
  }

  void Navigator::setupGotoCancel()
  {
    goto_cancel_layout = new QHBoxLayout;

    btn_goto_ = new QPushButton("Go", this);
    goto_cancel_layout->addWidget(btn_goto_);

    btn_cancel_ = new QPushButton("Cancel", this);
    goto_cancel_layout->addWidget(btn_cancel_);

  }

  void Navigator::setupLocTree()
  {
    loc_tree_layout = new QVBoxLayout;
    loc_tree_layout->addWidget(new QLabel("Flagged Locations:"));    

    flagged_locations_tree = new QTreeWidget();
    flagged_locations_tree->setColumnCount(1);

    flagged_locations_tree->setHeaderLabel("Coordinates");

    loc_tree_layout->addWidget(flagged_locations_tree);
    loc_tree_layout->setAlignment(Qt::AlignTop);
  }

  void Navigator::setNamespace()
  {
  	currNamespace.data = ns_combo_box_->currentText().toStdString();
  	mux_control_pub.publish(currNamespace);

    // Restart current map subscriber
    current_map_name_sub.shutdown();
    current_map_name_sub = this->rosNode->subscribe<std_msgs::String>("/" + currNamespace.data + MAP_NAME_TOPIC, 2, &Navigator::map_name_cb, this);

    cancel_goal_pub.shutdown();
    cancel_goal_pub = this->rosNode->advertise<std_msgs::Bool>("/" + currNamespace.data + "/cancel_all_goals", 2);

    set_map_client.shutdown();
    set_map_client = this->rosNode->serviceClient<multi_map_navigation::SetMap>("/" + currNamespace.data + "/multi_map_navigation/set_map");    

    setupActionlibClient();
  }

  void Navigator::setupMainGuiPanel()
  {
  	QVBoxLayout* layout = new QVBoxLayout;
  	layout->addLayout(mux_controller_layout);
    layout->addLayout(map_combo_layout);
    layout->addLayout(loc_tree_layout);
    layout->addLayout(add_delete_layout);
    layout->addLayout(goal_combo_layout);
    layout->addLayout(goto_cancel_layout);
  	layout->setAlignment(Qt::AlignTop);
  	setLayout(layout);
  }

  void Navigator::setupNode()
  {
  	rosNode = new ros::NodeHandle("");
  	mux_control_pub = rosNode->advertise<std_msgs::String>(MUX_CONTROL_TOPIC, 2);

  	rosNode->getParam("/namespace_mux/active_bots", active_bots);
  	rosNode->getParam("/namespace_mux/outgoing_ns", outgoing_ns);

  	if (active_bots.empty()) {
  		ROS_ERROR("Navigator: No bots active. Make sure that all the bots have been initialized\n");
  		std::exit(EXIT_FAILURE);
  	}
  }

  void Navigator::setupMuxControl()
  {
  	mux_controller_layout = new QHBoxLayout;

  	mux_controller_layout->addWidget(new QLabel("Active Robot:"));
  	ns_combo_box_ = new QComboBox;
  	mux_controller_layout->addWidget(ns_combo_box_);

  	for (std::vector<std::string>::iterator it = active_bots.begin(); it != active_bots.end(); ++it) {
  		ns_combo_box_->addItem(QString::fromStdString(*it));
  	}
  }

  void Navigator::addSubChildren(QTreeWidgetItem *item, rviz::Config flag) const
  {
    // Add all the individual properties of an element

    for (int i=0; i<4; i++) {
      flag.mapSetValue(item->child(i)->text(0), item->child(i)->child(0)->text(0));
    }
  }

  void Navigator::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );

    config.mapSetValue("Class", getClassId());
    rviz::Config flags_config = config.mapMakeChild("Flagged Locations");

    int numFlaggedLocs = flagged_locations_tree->topLevelItemCount();

    for (int i=0; i<numFlaggedLocs; i++) {
      QTreeWidgetItem *item = flagged_locations_tree->topLevelItem(i);
      // rviz::Config flag = flags_config.mapMakeChild(item->text(0));
      rviz::Config flag = flags_config.listAppendNew();
      flag.mapSetValue("Name", item->text(0));

      addSubChildren(item, flag);
    }

  }

  void Navigator::loadFlagProps(rviz::Config flag, QTreeWidgetItem *item)
  {
    float pose_x;
    flag.mapGetFloat("Pose X", &pose_x);
    createDataChild(item, "Pose X", pose_x);

    float pose_y;
    flag.mapGetFloat("Pose Y", &pose_y);
    createDataChild(item, "Pose Y", pose_y);

    float pose_yaw;
    flag.mapGetFloat("Pose Yaw", &pose_yaw);
    createDataChild(item, "Pose Yaw", pose_yaw);

    QString map_name;
    flag.mapGetString("Map", &map_name);
    createStringChild(item, "Map", map_name.toStdString());

  }

  void Navigator::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );

    rviz::Config flags_config = config.mapGetChild("Flagged Locations");

    int numFlaggedLocs = flags_config.listLength();

    for (int i=0; i<numFlaggedLocs; i++) {
      rviz::Config flag = flags_config.listChildAt(i);
      QTreeWidgetItem *item = new QTreeWidgetItem(flagged_locations_tree);

      QString locName = "Flag " + QString::number(i + 1);
      flag.mapGetString("Name", &locName);
      item->setText(0, locName);
      item->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);

      loadFlagProps(flag, item);
    }

    loadGoalComboBox();
    publish_markers();
    flagged_locations_tree->blockSignals(false);  
  }

} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::Navigator,rviz::Panel)
