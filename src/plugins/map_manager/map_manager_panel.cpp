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
#include <QDoubleSpinBox>
#include <QFileDialog>

#include "map_manager_panel.h"

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

namespace rviz
{

  MapManager::MapManager( QWidget* parent )
    : rviz::Panel( parent )
  {
  	setupStatusDisplay();
  	setupMapTree();
  	setupAddDeleteBtn();
  	setupBoxCombos();
  	setupAutoAligner();
  	setupPosEditor();

  	setupMainGuiPanel();
  	setupConnections();

  	initialize();
  }

  void MapManager::initialize()
  {
  	map_tree->blockSignals(true);

  	setupMapServices();
  	loadMapListAndCombo();
  	map_server = NULL;

  	map_tree->blockSignals(false);
  }


  void MapManager::setupConnections()
  {
  	this->rosNode = new ros::NodeHandle("");

  	this->rosNode->getParam("/aligner_namespaces", alignerNamespaces);
  	primaryMapNs = alignerNamespaces.at(0);
  	secondaryMapNs = alignerNamespaces.at(1);

  	pri_comm_pose_sub = this->rosNode->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/map_aligner/comm_pose_pri", 2, &MapManager::pri_comm_pose_cb, this);
  	sec_comm_pose_sub = this->rosNode->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/map_aligner/comm_pose_sec", 2, &MapManager::sec_comm_pose_cb, this);

  	secondary_map_sub = this->rosNode->subscribe<nav_msgs::OccupancyGrid>("/" + secondaryMapNs + "/map_store_map", 1, &MapManager::secondary_map_cb, this);
  	// save_map_pub = this->rosNode->advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  	connect(btn_delete, SIGNAL(clicked(void)), this, SLOT(deleteMap(void)));
    connect(map_tree, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(renameMap()) );
    connect(btn_set_primary, SIGNAL(clicked(void)), this, SLOT(setPrimaryMap(void)));
    connect(btn_set_secondary, SIGNAL(clicked(void)), this, SLOT(setSecondaryMap(void)));
    connect(btn_set_origin, SIGNAL(clicked(void)), this, SLOT(setSecondaryMapOrigin(void)));
    connect(btn_auto_align, SIGNAL(clicked(void)), this, SLOT(autoAlignMaps(void)));
    connect(btn_add, SIGNAL(clicked(void)), this, SLOT(addNewMap(void)));
  }

  void MapManager::setupMapServices()
  {
  	pri_list_map_client = this->rosNode->serviceClient<map_store::ListMaps>("/" + primaryMapNs + "/list_maps");
  	pri_delete_map_client = this->rosNode->serviceClient<map_store::DeleteMap>("/" + primaryMapNs + "/delete_map");
  	pri_set_map_client = this->rosNode->serviceClient<map_store::PublishMap>("/" + primaryMapNs + "/publish_map");
  	pri_rename_map_client = this->rosNode->serviceClient<map_store::RenameMap>("/" + primaryMapNs + "/rename_map");

  	sec_set_map_client = this->rosNode->serviceClient<map_store::PublishMap>("/" + secondaryMapNs + "/publish_map");
  	sec_delete_map_client = this->rosNode->serviceClient<map_store::DeleteMap>("/" + secondaryMapNs + "/delete_map");
  	sec_set_origin_client = this->rosNode->serviceClient<map_store::SetOrigin>("/" + secondaryMapNs + "/set_origin");
  	sec_save_map_client = this->rosNode->serviceClient<map_store::SaveMap>("/save_map");

  	// sec_set_map_server = this->rosNode->advertiseService("dynamic_map", &MapManager::get_map_cb, this);
  }

  void MapManager::pri_comm_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
  	x_auto_pri_editor->setValue(msg->pose.pose.position.x);
  	y_auto_pri_editor->setValue(msg->pose.pose.position.y);

  	float yaw = getYawFromQuat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

	theta_auto_pri_editor->setValue(yaw);
  }

  void MapManager::sec_comm_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
  	x_auto_sec_editor->setValue(msg->pose.pose.position.x);
  	y_auto_sec_editor->setValue(msg->pose.pose.position.y);

  	float yaw = getYawFromQuat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

	theta_auto_sec_editor->setValue(yaw);
  }

  float MapManager::getYawFromQuat(float x, float y, float z, float w)
  {
  	double roll, pitch, yaw;
  	tf::Quaternion quat = tf::Quaternion(x, y, z, w);
  	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  	return yaw;
  }

  void MapManager::renameMap()
  {
  	map_tree->blockSignals(true);

  	QTreeWidgetItem *selectedItem = map_tree->currentItem();

  	if (!selectedItem) {
  		return;
  	}

  	int index = map_tree->indexOfTopLevelItem(selectedItem);
  	if (index >= 0 && index < map_tree->topLevelItemCount()) {

  		map_store::RenameMap renameReq;
  		renameReq.request.map_id = selectedItem->child(0)->child(0)->text(0).toStdString();
  		renameReq.request.new_name = selectedItem->text(0).toStdString();

  		if (pri_rename_map_client.call(renameReq)) {
  			ROS_INFO("Renaming map to %s", renameReq.request.new_name.c_str());
  		} else {
  			ROS_ERROR("Could not rename map to %s", renameReq.request.new_name.c_str());
  			return;
  		}
  	}

  	loadCombos();
  	map_tree->blockSignals(false);
  }

  void MapManager::deleteMap()
  {
  	map_tree->blockSignals(true);

  	QTreeWidgetItem *selectedItem = map_tree->currentItem();

  	if (!selectedItem) {
  		return;
  	}

  	// Double check that the item actually exists
  	int index = map_tree->indexOfTopLevelItem(selectedItem);
  	if (index >=0 && index < map_tree->topLevelItemCount()) {

  		map_store::DeleteMap deleteReq;
  		deleteReq.request.map_id = selectedItem->child(0)->child(0)->text(0).toStdString();

  		if (pri_delete_map_client.call(deleteReq)) {
  			ROS_INFO("Deleting %s from map database", selectedItem->text(0).toStdString().c_str());
  		} else {
  			ROS_ERROR("Deleting %s from map database - FAILED", selectedItem->text(0).toStdString().c_str());
  			return;
  		}

  		delete selectedItem;
  	}

  	loadCombos();
  	map_tree->blockSignals(false);
  }

  void MapManager::loadCombos()
  {
   	QStringList maps;
  	primary_map_combo->clear();
  	secondary_map_combo->clear();

  	map_store::ListMaps list_srv;

  	if (pri_list_map_client.call(list_srv)) {
  		map_list = list_srv.response.map_list;

	      for (std::vector<map_store::MapListEntry>::iterator it = map_list.begin(); it != map_list.end(); ++it) {
	        maps << QString::fromStdString(it->name);   
	      }

  	} else {
  		ROS_ERROR("Could not retrieve a list of available maps from the database.");
  	}

  	primary_map_combo->addItems(maps);
  	secondary_map_combo->addItems(maps); 	
  }

  void MapManager::secondary_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
  	secondary_map = *msg;
  	loadOrigin();
  }

  void MapManager::loadMapListAndCombo()
  {
  	QStringList maps;
  	primary_map_combo->clear();
  	secondary_map_combo->clear();

  	renameUntiledMaps();

  	map_store::ListMaps list_srv;

  	if (pri_list_map_client.call(list_srv)) {
  		map_list = list_srv.response.map_list;

	      for (std::vector<map_store::MapListEntry>::iterator it = map_list.begin(); it != map_list.end(); ++it) {
	        maps << QString::fromStdString(it->name);   
	        addMapTreeItem(it->name, it->map_id);
	      }

  	} else {
  		ROS_ERROR("Could not retrieve a list of available maps from the database.");
  	}

  	primary_map_combo->addItems(maps);
  	secondary_map_combo->addItems(maps);
  }

  void MapManager::renameUntiledMaps()
  {
  	map_store::ListMaps list_srv;

  	if (pri_list_map_client.call(list_srv)) {
  		map_list = list_srv.response.map_list;

	    for (std::vector<map_store::MapListEntry>::iterator it = map_list.begin(); it != map_list.end(); ++it) {
	    	
	    	if (it->name.compare("") == 0) {
	        	map_store::RenameMap renameSrv;
	        	renameSrv.request.new_name = "New Map";
	        	renameSrv.request.map_id = it->map_id;

	        	if (!pri_rename_map_client.call(renameSrv)) {
	        		ROS_ERROR("Renaming untitled maps failed");
	        		return;
	        	}

	    	}
	    }
  	}

  }

  void MapManager::addMapTreeItem(std::string map_name, std::string map_id)
  {
  	QTreeWidgetItem *map_item = new QTreeWidgetItem(map_tree);
  	
  	map_item->setText(0, QString::fromStdString(map_name));
	map_item->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);

	createStringChild(map_item, "Map ID", map_id);
  }

  void MapManager::createStringChild(QTreeWidgetItem *parent, std::string name, std::string value_string)
  {
    QTreeWidgetItem *name_row = new QTreeWidgetItem(parent);
    name_row->setText(0, QString::fromStdString(name));

    QTreeWidgetItem *value_row = new QTreeWidgetItem(name_row);
    value_row->setText(0, QString::fromStdString(value_string));
  }

  std::string MapManager::getMapId(std::string map_name)
  {
  	int numMaps = map_tree->topLevelItemCount();

  	for (int i=0; i<numMaps; i++) {
  		QTreeWidgetItem *map = map_tree->topLevelItem(i);

  		if (map->text(0).toStdString().compare(map_name) == 0) {
  			return map->child(0)->child(0)->text(0).toStdString(); // access the child of the tree which contains MapID
  		}
  	}

  	return ""; // empty string to indicate no corresponding map ID was found
  }

  void MapManager::setPrimaryMap()
  {
  	std::string map_id = getMapId(primary_map_combo->currentText().toStdString());
  	setMap(map_id, pri_set_map_client);
  }

  void MapManager::setSecondaryMap()
  {
  	std::string map_id = getMapId(secondary_map_combo->currentText().toStdString());
  	setMap(map_id, sec_set_map_client);
  }

  void MapManager::setSecondaryMapOrigin()
  {	
  	tf::Quaternion quat;
  	quat.setRPY(0.0, 0.0, theta_man_editor->value());

  	map_store::SetOrigin setOriginSrv;
  	
  	setOriginSrv.request.map_id = getMapId(secondary_map_combo->currentText().toStdString());
  	setOriginSrv.request.pos_x = x_man_editor->value();
  	setOriginSrv.request.pos_y = y_man_editor->value();
  	setOriginSrv.request.rot_x = quat.x();
  	setOriginSrv.request.rot_y = quat.y();
  	setOriginSrv.request.rot_z = quat.z();
  	setOriginSrv.request.rot_w = quat.w();

  	if (!sec_set_origin_client.call(setOriginSrv)) {
  		ROS_ERROR("SERVICE CALL FAILED: Could not set the specified origin");
  		return;
  	}

  	setMap(setOriginSrv.request.map_id, sec_set_map_client);
  }

  void MapManager::autoAlignMaps()
  {
  	float initial_x = secondary_map.info.origin.position.x;
  	float initial_y = secondary_map.info.origin.position.y;

  	// pre-compute theta, i.e. the angular difference between the common poses
  	float delta_theta = theta_auto_pri_editor->value() - theta_auto_sec_editor->value();
  	float theta = delta_theta + getYawFromQuat(secondary_map.info.origin.orientation.x, secondary_map.info.origin.orientation.y, secondary_map.info.origin.orientation.z, secondary_map.info.origin.orientation.w);

  	tf::Quaternion quat;
  	quat.setRPY(0.0, 0.0, theta);

  	// secondary map - common pose after rotation
  	float prev_sec_pose_x = x_auto_sec_editor->value() - initial_x;
  	float prev_sec_pose_y = y_auto_sec_editor->value() - initial_y;
  	float rotated_sec_pose_x = initial_x + (prev_sec_pose_x * cosf(delta_theta) - prev_sec_pose_y * sinf(delta_theta));
  	float rotated_sec_pose_y = initial_y + (prev_sec_pose_x * sinf(delta_theta) + prev_sec_pose_y * cosf(delta_theta));

  	float delta_x = x_auto_pri_editor->value() - rotated_sec_pose_x;
  	float delta_y = y_auto_pri_editor->value() - rotated_sec_pose_y;

  	map_store::SetOrigin setOriginSrv;
  	setOriginSrv.request.map_id = getMapId(secondary_map_combo->currentText().toStdString());
  	setOriginSrv.request.pos_x = initial_x + delta_x;
  	setOriginSrv.request.pos_y = initial_y + delta_y;
  	setOriginSrv.request.rot_x = quat.x();
  	setOriginSrv.request.rot_y = quat.y();
  	setOriginSrv.request.rot_z = quat.z();
  	setOriginSrv.request.rot_w = quat.w();

  	if (!sec_set_origin_client.call(setOriginSrv)) {
  		ROS_ERROR("SERVICE CALL FAILED: Could not set the desired origin - theta offset stage failed");
  		return;
  	}

  	setMap(setOriginSrv.request.map_id, sec_set_map_client); 
  }


  // Pre-condition: Secondary map metadata has already been published
  void MapManager::loadOrigin()
  {
  	x_man_editor->setValue(secondary_map.info.origin.position.x);
  	y_man_editor->setValue(secondary_map.info.origin.position.y);

  	float yaw = getYawFromQuat(secondary_map.info.origin.orientation.x, secondary_map.info.origin.orientation.y, secondary_map.info.origin.orientation.z, secondary_map.info.origin.orientation.w);
  	theta_man_editor->setValue(yaw);
  }

  void MapManager::setMap(std::string id, ros::ServiceClient client)
  {
  	if (id.empty()) {
  		ROS_ERROR("The Map ID of the specified map could not be found");
  		return;
  	}

  	map_store::PublishMap targetMap;
  	targetMap.request.map_id = id;

  	if (!client.call(targetMap)) {
  		ROS_ERROR("Could not set the desired map for alignment");
  		return;
  	}
  }

  void MapManager::setupStatusDisplay()
  {
  	db_status_layout = new QVBoxLayout;

  	db_status_display_ = new QLabel("<b>MongoDB Static-Map Database</b>");
  	db_status_layout->addWidget(db_status_display_);
  }

  void MapManager::setupMapTree()
  {
  	map_tree_layout = new QVBoxLayout;

  	map_tree = new QTreeWidget();
  	map_tree->setColumnCount(1);
  	map_tree->setHeaderLabel("");

  	map_tree_layout->addWidget(map_tree);
  	map_tree_layout->setAlignment(Qt::AlignTop);

  }

  void MapManager::clearMapTree()
  {
  	int numMaps = map_tree->topLevelItemCount();

  	for (int i=0; i<numMaps; i++) {
  		QTreeWidgetItem *item = map_tree->topLevelItem(i);
  		delete item;
  	}

  }

  void MapManager::setupAddDeleteBtn()
  {
  	btn_layout = new QHBoxLayout;

  	btn_add = new QPushButton("Add", this);  	
	btn_layout->addWidget(btn_add);

  	btn_delete = new QPushButton("Delete", this);
  	btn_layout->addWidget(btn_delete);
  }

  void MapManager::setupBoxCombos()
  {
  	combo_box_layout = new QVBoxLayout;

  	combo_box_layout->addWidget(new QLabel("Reference Map - Fixed"));
  	QHBoxLayout *primary_btn_layout = new QHBoxLayout;
  	primary_map_combo = new QComboBox;
  	btn_set_primary = new QPushButton("Set", this);
  	btn_set_primary->setMaximumWidth(60);

  	primary_btn_layout->addWidget(primary_map_combo);
  	primary_btn_layout->addWidget(btn_set_primary);

  	combo_box_layout->addLayout(primary_btn_layout);

  	combo_box_layout->addWidget(new QLabel("Secondary Map - To Offset"));  	
  	QHBoxLayout *secondary_btn_layout = new QHBoxLayout;  	
  	secondary_map_combo = new QComboBox;
  	btn_set_secondary = new QPushButton("Set", this);
  	btn_set_secondary->setMaximumWidth(60);

  	secondary_btn_layout->addWidget(secondary_map_combo);
  	secondary_btn_layout->addWidget(btn_set_secondary);

  	combo_box_layout->addLayout(secondary_btn_layout);
  }

  void MapManager::setupPosEditor()
  {
  	pos_editor_layout = new QVBoxLayout;

  	QLabel *pos_editor_label = new QLabel("<b>MANUAL ALIGNER</b>");
  	pos_editor_label->setAlignment(Qt::AlignCenter);
  	pos_editor_layout->addWidget(pos_editor_label);

  	QHBoxLayout *spinboxes_layout = new QHBoxLayout;
  	x_man_editor = addSpinboxEditor(spinboxes_layout, x_man_editor, "X");
  	y_man_editor = addSpinboxEditor(spinboxes_layout, y_man_editor, "Y");
  	theta_man_editor = addSpinboxEditor(spinboxes_layout, theta_man_editor, "Theta");

  	btn_set_origin = new QPushButton("Set Origin", this);

  	pos_editor_layout->addLayout(spinboxes_layout);
  	pos_editor_layout->addWidget(btn_set_origin);
  }

  QDoubleSpinBox* MapManager::addSpinboxEditor(QHBoxLayout *parent_layout, QDoubleSpinBox *spinbox, std::string label)
  {
  	QVBoxLayout *spinbox_layout = new QVBoxLayout;
  	spinbox = new QDoubleSpinBox;
  	
  	spinbox->setSingleStep(0.1);
  	spinbox->setRange(-1000, 1000);
  	spinbox_layout->addWidget(spinbox);
  	spinbox_layout->addWidget(new QLabel(QString::fromStdString(label)));

  	parent_layout->addLayout(spinbox_layout);

  	return spinbox;
  }

  void MapManager::addNewMap()
  {
    // Find YAML File:
    QString filename = QFileDialog::getOpenFileName( 
        this, 
        tr("Open YAML Map file"), 
        QDir::currentPath(), 
        tr("YAML files (*.yaml);;All files (*.*)") );
    
    if(filename.isNull())
    {
    	ROS_ERROR("Could not load the selected YAML File");
      	return;
    }

    ROS_DEBUG("Loading YAML File: %s ....", filename.toStdString().c_str());

    // Check if a map_server is already running. If so, shut it down
    if (map_server) {	
    	delete map_server;
    }

  	// Start map server with specified YAML file  
    try {
    	map_server = new MapServer(filename.toStdString(), 0.0); // resolution arg = 0.0, as per latest USAGE specifications (map_server pkg)
    } catch (std::runtime_error& e) {
    	ROS_ERROR("map_server exception: %s", e.what());
    	return;
    }

    ros::Duration(1.3).sleep(); // wait for map_store to update the list with the new map      

    reloadMapListAndCombo();
  }

  void MapManager::reloadMapListAndCombo()
  {
    // refresh map tree:
  	map_tree->blockSignals(true);

    if (map_tree->topLevelItemCount() > 0) {
    	map_tree->clear();
    }

    loadMapListAndCombo(); 

    map_tree->blockSignals(false);
  }

  void MapManager::setupAutoAligner()
  {
  	auto_align_layout = new QVBoxLayout;

  	QLabel *auto_align_label = new QLabel("<b>AUTO ALIGNER</b>");
  	auto_align_label->setAlignment(Qt::AlignCenter);
  	auto_align_layout->addWidget(auto_align_label);

  	// Primary map common pose:
  	auto_align_layout->addWidget(new QLabel("Reference Map - Common Pose"));
  	// auto_align_layout->addWidget(new QLabel("topic: /map_aligner/comm_pose_pri"));

  	QHBoxLayout *auto_pri_spinbox_layout = new QHBoxLayout;
  	x_auto_pri_editor = addSpinboxEditor(auto_pri_spinbox_layout, x_auto_pri_editor, "X");
  	y_auto_pri_editor = addSpinboxEditor(auto_pri_spinbox_layout, y_auto_pri_editor, "Y");
  	theta_auto_pri_editor = addSpinboxEditor(auto_pri_spinbox_layout, theta_auto_pri_editor, "Theta");
  	auto_align_layout->addLayout(auto_pri_spinbox_layout);

  	// Secondary map common pose:
  	auto_align_layout->addWidget(new QLabel("Secondary Map - Common Pose"));
  	// auto_align_layout->addWidget(new QLabel("topic: /map_aligner/comm_pose_sec"));

  	QHBoxLayout *auto_sec_spinbox_layout = new QHBoxLayout;
  	x_auto_sec_editor = addSpinboxEditor(auto_sec_spinbox_layout, x_auto_sec_editor, "X");
  	y_auto_sec_editor = addSpinboxEditor(auto_sec_spinbox_layout, y_auto_sec_editor, "Y");
  	theta_auto_sec_editor = addSpinboxEditor(auto_sec_spinbox_layout, theta_auto_sec_editor, "Theta");
  	auto_align_layout->addLayout(auto_sec_spinbox_layout);

  	btn_auto_align = new QPushButton("Auto Align", this);
  	auto_align_layout->addWidget(btn_auto_align);
  }

  void MapManager::setupMainGuiPanel()
  {
  	QVBoxLayout* layout = new QVBoxLayout;
  	layout->addLayout(db_status_layout);
  	layout->addLayout(map_tree_layout);
  	layout->addLayout(btn_layout);
  	layout->addLayout(combo_box_layout);
  	layout->addLayout(pos_editor_layout);
  	layout->addLayout(auto_align_layout);

  	layout->setAlignment(Qt::AlignTop);
  	setLayout(layout);
  }


  void MapManager::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void MapManager::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::MapManager,rviz::Panel )
