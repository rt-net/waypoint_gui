#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2021 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose


class MarkerPublisher():
    def __init__(self):
      self.server = InteractiveMarkerServer("waypoints_marker")
      self.clicked_sub = rospy.Subscriber("clicked_point", PointStamped, self.callbackClicked)
      self.num_pub = rospy.Publisher("waypointnum_array", MarkerArray, queue_size = 1)

      self.clicked_points = PointStamped()
      self.markernum_array = MarkerArray()
      self.waypoints_array = PoseArray()
      self.clicked_prev = PointStamped()
      self.clicked_points = PointStamped()
      self.clicked_sum = 0

    def processFeedback(self, feedback):
      p = feedback.pose
      # print(feedback.marker_name + " is now at " + str(p.position.x) + ", " + str(p.position.y) + ", " + str(p.position.z))
      self.markernum_array.markers[int(feedback.marker_name)].pose.position.x = p.position.x 
      self.markernum_array.markers[int(feedback.marker_name)].pose.position.y = p.position.y
      self.markernum_array.markers[int(feedback.marker_name)].pose.position.z = p.position.z
      self.num_pub.publish(self.markernum_array)
      return

    def callbackClicked(self, msg):
      self.clicked_points = msg
      self.clicked_sum += 1
      return

    def setIntmarker(self, i):
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = "map"
      int_marker.name = str(i)
      int_marker.description = str(i)
      int_marker.pose.position.x = self.clicked_points.point.x
      int_marker.pose.position.y = self.clicked_points.point.y
      int_marker.pose.position.z = self.clicked_points.point.z
      int_marker.pose.orientation.x = 0.0
      int_marker.pose.orientation.y = 0.0
      int_marker.pose.orientation.z = 0.0
      int_marker.pose.orientation.w = 1.0
      return int_marker

    def setArrow(self, i):
      marker = Marker()
      marker.type = marker.ARROW
      marker.header.frame_id = "map"
      marker.action = marker.ADD
      marker.scale.x = 1.2
      marker.scale.y = 0.5
      marker.scale.z = 0.5
      marker.color.r = 0.0
      marker.color.g = 0.5
      marker.color.b = 0.5
      marker.color.a = 0.5
      marker.pose.position.x = self.clicked_points.point.x
      marker.pose.position.y = self.clicked_points.point.y
      marker.pose.position.z = self.clicked_points.point.z
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 1.0
      return marker

    def setControler(self, i, marker):
      rotate_control = InteractiveMarkerControl()
      rotate_control.name = str(i)
      rotate_control.orientation.w = 1
      rotate_control.orientation.x = 0
      rotate_control.orientation.y = 1
      rotate_control.orientation.z = 0
      rotate_control.always_visible = True
      rotate_control.markers.append(marker)
      rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
      return rotate_control

    def setNum(self, i):
      marker_num = Marker()
      marker_num.header.frame_id = "map"
      marker_num.header.stamp = rospy.Time.now()
      marker_num.ns = "marker_num"
      marker_num.id = i
      marker_num.action = Marker.ADD
      marker_num.pose.position.x = self.clicked_points.point.x
      marker_num.pose.position.y = self.clicked_points.point.y
      marker_num.pose.position.z = self.clicked_points.point.z
      marker_num.pose.orientation.x= 0.0
      marker_num.pose.orientation.y= 1.0
      marker_num.pose.orientation.z= 1.0
      marker_num.pose.orientation.w= 1.0
      marker_num.color.r = 0.0
      marker_num.color.g = 0.0
      marker_num.color.b = 0.0
      marker_num.color.a = 1.0
      marker_num.scale.x = 1
      marker_num.scale.y = 1
      marker_num.scale.z = 1
      marker_num.lifetime = rospy.Duration()
      marker_num.type = Marker.TEXT_VIEW_FACING
      marker_num.text = str(i+1)
      self.markernum_array.markers.append(marker_num)
      self.num_pub.publish(self.markernum_array)
      return

    def setMarkerarray(self, i):
      msg = Pose()
      msg.position.x = self.clicked_points.point.x
      msg.position.y = self.clicked_points.point.y
      msg.position.z = self.clicked_points.point.z
      msg.orientation.x = 0.0
      msg.orientation.y = 0.0
      msg.orientation.z = 0.0
      msg.orientation.w = 1.0
      self.waypoints_array.poses.append(msg)
      return

    def pubMarker(self):
      if(self.clicked_prev == self.clicked_points):
         return
      num = self.clicked_sum - 1
      # interactive markerの追加
      int_marker = self.setIntmarker(num)
      # markerの追加
      marker = self.setArrow(num)
      # controlerの追加
      rotate_control = self.setControler(num, marker)
      int_marker.controls.append( rotate_control )
      self.server.insert( int_marker, self.processFeedback)
      self.setNum(num)
      self.server.applyChanges()
      # pubするmarkerarrayをセット
      self.setMarkerarray(num)
      self.clicked_prev = self.clicked_points
      return


def main():
    rospy.init_node('marker_publiser')
    marker_pub = MarkerPublisher()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        marker_pub.pubMarker()
        r.sleep()

if __name__ == '__main__':
    main()