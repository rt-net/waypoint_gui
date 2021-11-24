#!/usr/bin/env python
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

#  * This program is based on
#  * https://github.com/danielsnider/follow_waypoints/blob/master/src/follow_waypoints/follow_waypoints.py
#  * which is released under the Unlicense.

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray, Marker
from std_msgs.msg import Empty, Int32MultiArray 
from geometry_msgs.msg import PointStamped
from tf import TransformListener
import tf
import math
import rospkg
import time
import csv

# change Pose to the correct frame 
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()

#Path for saving and retreiving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('waypoint_gui')+"/saved_waypoints/waypoints.csv"
waypoints = []

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

    def execute(self, userdata):
        global waypoints
        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                #This is the loop which exist when the robot is near a certain GOAL point.
                distance = 10
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                    distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))
        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Subscribe to pose message to get new waypoints
        self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)
        self.marker_publisher  = rospy.Publisher("waypointnum_array", MarkerArray, queue_size=1)
        self.route_sub = rospy.Subscriber('waypoints_route', Int32MultiArray, self.callbackRoute)
        self.waypoints_sub = rospy.Subscriber('/waypoints_marker/feedback', InteractiveMarkerFeedback, self.callbackPose)
        self.clicked_sub = rospy.Subscriber("clicked_point", PointStamped, self.callbackClicked)
        self.waypose_array = PoseArray()
        self.route_array   = Int32MultiArray()
        self.saved_waypoints = []

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
            with open(output_file_path, 'w') as file:
                for current_pose in self.saved_waypoints :
                    file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
	        rospy.loginfo('poses written to '+ output_file_path)
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        self.start_journey_bool = False

        # Start thread to listen start_jorney 
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv
        def wait_for_start_journey():
            """thread worker function"""
            data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
            rospy.loginfo('Recieved path READY start_journey')
            if(len(self.route_array.data)>=1):
                try:
                    for num in range(len(self.route_array.data)):
                        current_pose = PoseWithCovarianceStamped() 
                        current_pose.pose.pose.position.x     =  self.waypose_array.poses[self.route_array.data[num]].pose.position.x
                        current_pose.pose.pose.position.y     =  self.waypose_array.poses[self.route_array.data[num]].pose.position.y
                        current_pose.pose.pose.position.z     =  self.waypose_array.poses[self.route_array.data[num]].pose.position.z
                        current_pose.pose.pose.orientation.x  =  self.waypose_array.poses[self.route_array.data[num]].pose.orientation.x
                        current_pose.pose.pose.orientation.y  =  self.waypose_array.poses[self.route_array.data[num]].pose.orientation.y
                        current_pose.pose.pose.orientation.z  =  self.waypose_array.poses[self.route_array.data[num]].pose.orientation.z
                        current_pose.pose.pose.orientation.w  =  self.waypose_array.poses[self.route_array.data[num]].pose.orientation.w
                        waypoints.append(current_pose)
                        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
                        rospy.loginfo('move to waypoints')
                except IndexError:
                    print("out of list")
                    return

            else:
                markerarray = MarkerArray()
                with open(output_file_path, 'r') as file:
                    reader = csv.reader(file, delimiter = ',')
                    for row in reader:
                        clicked = PointStamped()
                        clicked.point.x = float(row[0])
                        clicked.point.y = float(row[1])
                        clicked.point.z = float(row[2])
                        current_pose = PoseWithCovarianceStamped() 
                        current_pose.pose.pose.position.x     =    float(row[0])
                        current_pose.pose.pose.position.y     =    float(row[1])
                        current_pose.pose.pose.position.z     =    float(row[2])
                        current_pose.pose.pose.orientation.x = float(row[3])
                        current_pose.pose.pose.orientation.y = float(row[4])
                        current_pose.pose.pose.orientation.z = float(row[5])
                        current_pose.pose.pose.orientation.w = float(row[6])
                        marker = Marker()
                        marker.type = marker.ARROW
                        marker.header.frame_id = "map"
                        marker.action = marker.ADD
                        marker.id = current_pose.pose.pose.position.x 
                        marker.scale.x = 1.2
                        marker.scale.y = 0.5
                        marker.scale.z = 0.5
                        marker.color.r = 0.0
                        marker.color.g = 0.5
                        marker.color.b = 0.5
                        marker.color.a = 0.5
                        marker.pose.position.x = current_pose.pose.pose.position.x 
                        marker.pose.position.y = current_pose.pose.pose.position.y
                        marker.pose.position.z = current_pose.pose.pose.position.z
                        marker.pose.orientation.x = current_pose.pose.pose.orientation.x
                        marker.pose.orientation.y = current_pose.pose.pose.orientation.y
                        marker.pose.orientation.z = current_pose.pose.pose.orientation.z
                        marker.pose.orientation.w = current_pose.pose.pose.orientation.w
                        markerarray.markers.append(marker)
                        self.marker_publisher.publish(markerarray)
                        waypoints.append(current_pose)
                        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            self.saved_waypoints  = waypoints
            self.start_journey_bool = True
            
        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()

        topic = self.addpose_topic
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            # waypoints.append(changePose(pose, "map"))
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

    def callbackRoute(self, msg):
        self.route_array = msg
        rospy.loginfo('Recieved waypoints array')
    
    def callbackPose(self, msg):
        if(len(self.waypose_array.poses) < 1):
            return
        self.waypose_array.poses[int(msg.marker_name)].header = msg.header
        self.waypose_array.poses[int(msg.marker_name)].pose = msg.pose
    
    def callbackClicked(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.waypose_array.poses.append(pose)

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.success_pub = rospy.Publisher('waypoints_success', Empty, queue_size=1)
        self.empty = Empty()
        

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        self.success_pub.publish(self.empty)
        return 'success'

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
