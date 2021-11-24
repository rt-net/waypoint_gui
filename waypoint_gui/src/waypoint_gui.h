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

#ifndef Q_MOC_RUN
    #include <ros/ros.h>
    #include <rviz/panel.h>
    #include "std_msgs/String.h"
    #include <std_msgs/Bool.h>
    #include <vector>
    #include <utility>
    #include <stdio.h>
    #include "std_msgs/Int32MultiArray.h"
    #include "std_msgs/Empty.h"
    #include <unistd.h>

    // qt headers here
    #include <QtCore>
    #include <QPushButton>
    #include <QString>
    #include <QSize>
    #include <QLabel>
    #include <QGridLayout>
    #include <QScrollArea>
    #include <QTextEdit>
    #include <QPalette>
    #include <QLineEdit>
    #include <QFont>
    #include <QString>
    #include <QImage>
#endif

namespace rviz_plugin
{
class WaypointGui: public rviz::Panel {
Q_OBJECT 
public:
    WaypointGui(QWidget* Parent = 0);
    int waypoint_sum = 0;

private:
  ros::NodeHandle nh_;
  ros::Publisher waypoints_pub, start_journey_pub, save_waypoint_pub;
  ros::Subscriber success_sub;
  std_msgs::Int32MultiArray waypoints_array;
  std_msgs::Empty empty;
  std::vector<bool> flag;
  std::vector<int> vec;

  // Qt options
  QScrollArea *sca;
  QWidget* inner;
  QGridLayout *layout;
  QGridLayout *layout2;
  QLabel *mlabel;
  // QImage *image;
  QPushButton *btn_ok, *btn_save;
  QPushButton *waypoints_btn[20];
  QSignalMapper* signalmapper[20];
  QLineEdit *line;
  QString qstr_;
  const QSize btnSize = QSize(650, 50);
  const QSize btnSize_ = QSize(50, 50);

protected Q_SLOTS:
void btnClicked(int);
void btnClicked_(int);
void clickedStart();
void clickedSave();
void callbackSuccess(const std_msgs::Empty::ConstPtr&);

};
} // end namespace rviz_plugin
