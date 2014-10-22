#!/usr/bin/env python

import roslib; roslib.load_manifest('mapping_smach')
import rospy
import smach
import smach_ros

from smach import Concurrence, Sequence
from util import *
from move_base_to_pose import *

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from collections import deque


class wait_for_command(WaitForMsgState):
    def __init__(self): 
        rospy.loginfo("Waiting for commands. You can use:")
        rospy.loginfo("unknown, collect, execute, exit")
        WaitForMsgState.__init__(self, '/command', String, self._msg_cb, output_keys=['command'])

    def _msg_cb(self, msg, ud):
        rospy.loginfo("Got Command %s.", msg.data)
        ud.command = msg

class process_command(smach.State):
    def __init__(self): 
        rospy.loginfo("Waiting to process commands.")
        smach.State.__init__(self, outcomes=['unknown', 'collect', 'execute', 'exit'], input_keys=['command'])

    def execute(self, userdata):
        if userdata.command.data == "exit":
          rospy.loginfo('I will exit.')
          return 'exit'
        elif userdata.command.data == "collect":
          rospy.loginfo('I will gather poses.')
          return 'collect'
        elif userdata.command.data == "execute":
          rospy.loginfo('I will execute the poses.')
          return 'execute'
        else:
          rospy.loginfo('Thats no command, I will idle.')
          return 'unknown'

class wait_for_pose(WaitForMsgState):
    def __init__(self): 
        rospy.loginfo("Waiting for pose.")
        WaitForMsgState.__init__(self, 'mapping_pose', PoseStamped, self._msg_cb, output_keys=['pose'])

    def _msg_cb(self, msg, ud):
        ud.pose = msg

class add_pose_to_queue(smach.State):
    def __init__(self): 
        rospy.loginfo("Waiting to add poses.")
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['pose'])

    def execute(self, userdata):
        global unvisited_poses
        unvisited_poses.append(userdata.pose)
        rospy.loginfo('Added Pose #%d', len(unvisited_poses))
        return 'succeeded'

class get_pose_from_queue(smach.State):
    def __init__(self): 
        rospy.loginfo("Waiting to deliver poses.")
        self.queue = deque([])
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['pose'], output_keys=['poses'])

    def execute(self, userdata):
        self.queue.append(userdata.pose)
        userdata.poses=self.queue
        rospy.loginfo('Added Pose #%d', len(self.queue))
        return 'succeeded'

#### pose collection ####
class collection_cyle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.5)
        return 'active'

def collection_child_term_cb(outcome_map):
    if outcome_map['COLLECTION_CYCLE'] == 'active':
        return True
    elif outcome_map['COLLECTION_FUNCTION'] == 'succeeded':
        return True
    elif outcome_map['COLLECTION_CONTROL'] == 'invalid':
        return True
    else:
        return False

def collection_out_cb(outcome_map):
    if outcome_map['COLLECTION_CYCLE'] == 'active':
        return 'collecting_active'
    elif outcome_map['COLLECTION_FUNCTION'] == 'succeeded':
        return 'collecting_active'
    else:
        return 'collecting_inactive'

def collection_control(ud, msg):
    if msg.data == 'exit_collection' or msg.data == 'execute' or msg.data == 'exit':
      return False
    else: 
      return True

### execution ####
class execution_cyle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.5)
        return 'active'

def execution_child_term_cb(outcome_map):
    if outcome_map['EXECUTION_CYCLE'] == 'active':
        return True
    elif outcome_map['EXECUTION_FUNCTION'] == 'succeeded' or outcome_map['EXECUTION_FUNCTION'] == 'failed':
        return True
    elif outcome_map['EXECUTION_CONTROL'] == 'invalid':
        return True
    else:
        return False

def execution_out_cb(outcome_map):
    if outcome_map['EXECUTION_CYCLE'] == 'active':
        return 'execution_active'
    elif outcome_map['EXECUTION_FUNCTION'] == 'succeeded':
        return 'execution_active'
    elif outcome_map['EXECUTION_CONTROL'] == 'collect':
        rospy.loginfo("DIRECTLY TO COLLECT")
        return 'execution_active'
    else:
        return 'execution_inactive'

def execution_control(ud, msg):
    if msg.data == 'exit_execution' or msg.data == 'collect' or msg.data == 'exit':
      return False
    else: 
      return True

class get_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], output_keys=['pose'])
    def execute(self, userdata):
        global unvisited_poses, current_pose
        rospy.loginfo('Got %d poses to execute', len(unvisited_poses))
        if len(unvisited_poses) > 0:
          current_pose = unvisited_poses.popleft()
          userdata.pose = current_pose
          rospy.loginfo('Execute pose')
          rospy.loginfo('Got %d poses left', len(unvisited_poses))
          return 'succeeded'
        else:
          rospy.loginfo('Nothing to execute, please collect new poses')
          return 'failed'

class publish_poses(smach.State):

 def __init__(self):
    smach.State.__init__(self, outcomes=['succeeded'])
 def execute(self, userdata):
    global unvisited_poses, visited_poses, current_pose
    global unvisited_poses_publisher,visited_poses_publisher, current_pose_publisher

    unvisited_poses_marker = MarkerArray()
    count = 0
    for pose in unvisited_poses:
      marker = Marker()
      marker.header = pose.header
      marker.type = marker.ARROW
      marker.action = marker.ADD
      marker.id = count
      marker.ns = 'unvisited'
      marker.scale.x = 0.2
      marker.scale.y = 0.1
      marker.scale.z = 0.1
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 1.0
      marker.pose = pose.pose
      unvisited_poses_marker.markers.append(marker)
      count += 1
    unvisited_poses_publisher.publish(unvisited_poses_marker)
    
    visited_poses_marker = MarkerArray()
    count = 0
    for pose in visited_poses:
      marker = Marker()
      marker.header = pose.header
      marker.type = marker.ARROW
      marker.action = marker.ADD
      marker.id = count
      marker.ns = 'visited'
      marker.scale.x = 0.2
      marker.scale.y = 0.1
      marker.scale.z = 0.1
      marker.color.r = 0.5
      marker.color.g = 1.0
      marker.color.b = 0.0
      marker.color.a = 1.0
      marker.pose = pose.pose
      visited_poses_marker.markers.append(marker) 
      count += 1    
    visited_poses_publisher.publish(visited_poses_marker)
   
    if current_pose != 0:
      current_pose_marker = Marker()
      current_pose_marker.header = current_pose.header
      current_pose_marker.type = marker.ARROW
      current_pose_marker.action = marker.ADD
      current_pose_marker.id = 0
      current_pose_marker.ns = 'current'
      current_pose_marker.scale.x = 0.3
      current_pose_marker.scale.y = 0.1
      current_pose_marker.scale.z = 0.1
      current_pose_marker.color.r = 1.0
      current_pose_marker.color.g = 0.0
      current_pose_marker.color.b = 1.0
      current_pose_marker.color.a = 1.0
      current_pose_marker.pose = current_pose.pose
      current_pose_publisher.publish(current_pose_marker)

    return 'succeeded'

class mark_pose_visted(smach.State):

 def __init__(self):
    smach.State.__init__(self, outcomes=['succeeded'])
 def execute(self, userdata):
    global visited_poses, current_pose
    visited_poses.append(current_pose)
    current_pose = 0
    return 'succeeded'

### smach execution ###

def myhook():
  print "shutdown time!"


def main():
    rospy.init_node("preemption_example")

    global unvisited_poses_publisher, visited_poses_publisher, current_pose_publisher
    global unvisited_poses, visited_poses, current_pose
    unvisited_poses = deque([])
    visited_poses = deque([])
    current_pose = 0

    unvisited_poses_publisher = rospy.Publisher('mapping_smach/unvisited_poses', MarkerArray)
    visited_poses_publisher = rospy.Publisher('mapping_smach/visited_poses', MarkerArray)
    current_pose_publisher = rospy.Publisher('mapping_smach/current_pose', Marker)

    collect_and_add = smach.Sequence(outcomes = ['succeeded', 'aborted', 'preempted'], connector_outcome = 'succeeded')
    with collect_and_add:
        smach.Sequence.add('WAIT_FOR_POSE', wait_for_pose(),
                     transitions={'succeeded':'ADD_POSE', 'aborted':'aborted', 'preempted':'preempted'},
                     remapping={'pose':'pose'})
        smach.Sequence.add('ADD_POSE', add_pose_to_queue(),
                     transitions={'succeeded':'PUBLISH_MARKER'},
                     remapping={'poses':'poses'})
        smach.Sequence.add('PUBLISH_MARKER', publish_poses(),
                     transitions={'succeeded':'succeeded'})

    collection_concurrence = smach.Concurrence(outcomes=['collecting_active', 'collecting_inactive'],
                                        default_outcome='collecting_active',
                                        child_termination_cb=collection_child_term_cb,
                                        outcome_cb=collection_out_cb)

    with collection_concurrence:
        smach.Concurrence.add('COLLECTION_CYCLE', collection_cyle())
        smach.Concurrence.add('COLLECTION_FUNCTION', collect_and_add)
        smach.Concurrence.add('COLLECTION_CONTROL', smach_ros.MonitorState("/command", String, collection_control)) 

    extract_and_move = smach.Sequence(outcomes = ['succeeded', 'failed'], connector_outcome = 'succeeded')
    with extract_and_move:
        smach.Sequence.add('GET_POSE', get_pose(), transitions={'succeeded':'PUBLISH_MARKER2','failed':'failed'}, remapping={'pose':'goal_pose'})
        smach.Sequence.add('PUBLISH_MARKER2', publish_poses(), transitions={'succeeded':'MOVE_TO_POSE'})
        smach.Sequence.add('MOVE_TO_POSE', move_base_to_pose(), transitions={'succeeded':'MARK_POSE_VISTED','failed':'failed'})
        smach.Sequence.add('MARK_POSE_VISTED', mark_pose_visted(), transitions={'succeeded':'PUBLISH_MARKER'})
        smach.Sequence.add('PUBLISH_MARKER', publish_poses(), transitions={'succeeded':'succeeded'})

    execution_concurrence = smach.Concurrence(outcomes=['execution_active', 'execution_inactive'],
                                        default_outcome='execution_active',
                                        child_termination_cb=execution_child_term_cb,
                                        outcome_cb=execution_out_cb)

    with execution_concurrence:
        smach.Concurrence.add('EXECUTION_CYCLE', execution_cyle())
        smach.Concurrence.add('EXECUTION_FUNCTION', extract_and_move)
        smach.Concurrence.add('EXECUTION_CONTROL', smach_ros.MonitorState("/command", String, execution_control))

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('WAIT_FOR_COMMAND', wait_for_command(), transitions={'succeeded':'PROCESS_COMMAND', 'aborted':'WAIT_FOR_COMMAND', 'preempted':'WAIT_FOR_COMMAND'})
        smach.StateMachine.add('PROCESS_COMMAND', process_command(), transitions={'unknown':'WAIT_FOR_COMMAND', 'collect':'COLLECTION_CONCURRENCE', 'execute':'EXECUTION_CONCURRENCE', 'exit':'DONE'})      
        smach.StateMachine.add('COLLECTION_CONCURRENCE', collection_concurrence, transitions={'collecting_active':'COLLECTION_CONCURRENCE', 'collecting_inactive':'WAIT_FOR_COMMAND'}) 
        smach.StateMachine.add('EXECUTION_CONCURRENCE', execution_concurrence, transitions={'execution_active':'EXECUTION_CONCURRENCE', 'execution_inactive':'WAIT_FOR_COMMAND'})

    while not rospy.is_shutdown():
      #sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
      #sis.start()
      sm.execute()
      rospy.spin()
      #sis.stop()

if __name__ == '__main__':
    main()
