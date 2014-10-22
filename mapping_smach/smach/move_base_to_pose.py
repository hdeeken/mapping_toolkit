#!/usr/bin/env python

import roslib; roslib.load_manifest('muffin_smach_common')
import rospy
import smach
import smach_ros
import message_filters
import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class move_base_to_pose(smach.State):
    # move to target pose
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['goal_pose'])


    def execute(self, userdata):
        rospy.loginfo("MoveBase called.")
        try:
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server(rospy.Duration.from_sec(10.0))

            goal = MoveBaseGoal()
            goal.target_pose = userdata.goal_pose
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(80.0))
            if client.get_state() == 3: 
                return 'succeeded'
            else: 
                return 'failed'
        except actionlib.ActionException, e: 
            rospy.logwarn("MoveBase action failed: %s" %e)
            return 'failed'

