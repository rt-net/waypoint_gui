// Copyright 2021 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "waypoint_gui.h"

namespace rviz_plugin
{
WaypointGui::WaypointGui(QWidget* parent)
    :rviz::Panel(parent)
{
    // pub
    waypoints_pub     = nh_.advertise<std_msgs::Int32MultiArray>("waypoints_route", 1);
    start_journey_pub = nh_.advertise<std_msgs::Empty>("start_journey", 1);
    success_sub       = nh_.subscribe("waypoints_success", 1, &rviz_plugin::WaypointGui::callbackSuccess, this);
    save_waypoint_pub = nh_.advertise<std_msgs::Empty>("path_ready", 1);
    nh_.getParam("/waypoint_sum", waypoint_sum);
    flag.resize(waypoint_sum);

    // Qt
    sca = new QScrollArea(this);
    inner = new QWidget();
    btn_ok = new QPushButton("Start");
    btn_save = new QPushButton("Save");
    line = new QLineEdit();
    btn_ok->setFont(QFont("Arial", 12, QFont::Bold));
    btn_save->setFont(QFont("Arial", 12, QFont::Bold));
    line->setFixedSize( 150, 50);
    line->setFont(QFont("Arial", 12, QFont::Bold)); 
    line->setText("");
    btn_ok->setFixedSize(btnSize);
    btn_save->setFixedSize(btnSize);
    layout = new QGridLayout();
    layout2 = new QGridLayout();
    for(int i = 0; i < waypoint_sum; i++)
    {
      waypoints_btn[i] = new QPushButton(QString::number(i+1));
      waypoints_btn[i]->setFixedSize(btnSize_);
      waypoints_btn[i]->setFont(QFont("Arial", 12, QFont::Bold));
      layout2->addWidget(waypoints_btn[i], 1, i);
    }
    qDebug("waypoint_sum: %d", waypoint_sum);
    layout2->addWidget(line, 1, waypoint_sum);
    inner->setLayout(layout2);
    sca->setWidget(inner);
    sca->setWidgetResizable(true);
    layout->addWidget(sca, 1, 0);
    layout->addWidget(btn_ok, 2, 0, 1, 1);
    layout->addWidget(btn_save, 3, 0, 1, 1);
    this->setLayout(layout);

    connect( btn_ok, SIGNAL( clicked() ), this, SLOT( clickedStart() ));
    connect( btn_save, SIGNAL( clicked() ), this, SLOT( clickedSave() ));
    for(int k=0; k< waypoint_sum; k++) 
    {
        signalmapper[k] = new QSignalMapper();
        connect( waypoints_btn[k], SIGNAL( clicked() ), signalmapper[k], SLOT( map()) );
        signalmapper[k]->setMapping(waypoints_btn[k],k);
        connect( signalmapper[k],  SIGNAL( mapped(int)), this, SLOT( btnClicked(int) ));
    }
}

void WaypointGui::btnClicked(int value)
{
  flag[value] = !flag[value];
  qstr_.clear();

  if(flag[value])
  {
     waypoints_btn[value]->setStyleSheet(
    "background-color:skyblue; color:black; border-radius:8px;" //文字の背景色
    "border-color:blue; border-style:solid; border-width:4px;" //文字の囲い
     );
    qDebug("add waypoints array");
    vec.push_back(value);
  }
  else{
    waypoints_btn[value]->setStyleSheet(
    "default" 
    );
    qDebug("remove waypoints array");
    std::remove(vec.begin(), vec.end(), value);
    vec.pop_back();
  }

  // lineに選択されたwaypoint番号表示
  for(const auto& num : vec)
  {
      QString qstr = QString::number(num+1); // qstringに変換
      qstr += "->";
      qstr_ += qstr;
  }
  line->setText(qstr_);
}

void WaypointGui::clickedStart()
{
  for(auto num : vec)
  {
    waypoints_array.data.push_back(num);
    qDebug("waypoints num: %d " , num);
  }
  waypoints_pub.publish(waypoints_array);
  qDebug("Published waypoints array");
  sleep(0.5);
  start_journey_pub.publish(empty);
  qDebug("Published Start journey message");
}

void WaypointGui::clickedSave()
{
  save_waypoint_pub.publish(empty);
  qDebug("Saved waypoints");
}

void WaypointGui::callbackSuccess(const std_msgs::Empty::ConstPtr& msg)
{
    waypoints_array.data.clear();
    vec.clear();
    line->setText("");
    qDebug("delete waypoints array");
    for(int i =0 ; i < waypoint_sum ;i++)
    {
      flag[i] = false;
      waypoints_btn[i]->setStyleSheet(
      "default"
    );
    }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::WaypointGui, rviz::Panel)