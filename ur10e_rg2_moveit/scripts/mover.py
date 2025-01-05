#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""

# def add_marker(name, pose):
#     marker = moveit_commander.PlanningSceneInterface()
#     marker_pose = geometry_msgs.msg.PoseStamped()
#     marker_pose.header.frame_id = "base_link"  # Replace with your frame
#     marker_pose.pose = pose
#     marker.add_box(name, marker_pose, size=(1, 1, 1))

# def publish_pose_as_axes(pose, topic_name="visualization_pose"):
#     pose_pub = rospy.Publisher(topic_name, geometry_msgs.msg.PoseStamped, queue_size=10)

#     pose_stamped = geometry_msgs.msg.PoseStamped()
#     pose_stamped.header.frame_id = "world"  # Replace with your robot's base frame
#     pose_stamped.header.stamp = rospy.Time.now()
#     pose_stamped.pose = pose

#     rospy.sleep(1)  # Ensure publisher is ready
#     pose_pub.publish(pose_stamped)
#     rospy.loginfo(f"Published pose to topic: {topic_name}")
#     # rospy.loginfo(f"Published pose : {pose_stamped}")


def plan_pick_and_place(req):
    

    scene = moveit_commander.PlanningSceneInterface()
    rospy.logwarn(scene.get_known_object_names())

    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("LIN")
    move_group.set_goal_tolerance(0.01) 
    move_group.set_planning_time(10)   
    current_robot_joint_configuration = req.joints_input.joints

    # publish_pose_as_axes(req.pick_pose, "pre_grasp_pose_topic")
    req.pick_pose.orientation.x = 0.5
    req.pick_pose.orientation.y = -0.5
    req.pick_pose.orientation.z = -0.5
    req.pick_pose.orientation.w = 0.5

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    # pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    # if not pick_up_pose.joint_trajectory.points:
    #     return response

    # previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # # Place - move gripper to desired placement position
    # place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    # if not place_pose.joint_trajectory.points:
    #     return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    # response.trajectories.append(pick_up_pose)
    # response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response


def collision_callback(msg):
    rospy.loginfo("Collision Object Position: x=%f, y=%f, z=%f", msg.primitive_poses[0].position.x, msg.primitive_poses[0].position.y, msg.primitive_poses[0].position.z)
    rospy.loginfo("Collision Object Dimensions: width=%f, height=%f, depth=%f", msg.primitives[0].dimensions[0], msg.primitives[0].dimensions[1], msg.primitives[0].dimensions[2])

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()
    
    # # Create a box in the planning scene with the received pose and dimensions
    box_name = "table"
    size_x = msg.primitives[0].dimensions[0]
    size_y = msg.primitives[0].dimensions[1]
    size_z = msg.primitives[0].dimensions[2]
    # x = msg.primitive_poses[0].position.x
    # y = msg.primitive_poses[0].position.y
    # z = msg.primitive_poses[0].position.z

    box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = msg.id
    #rospy.loginfo("Msg ID: %s", msg.id)
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position = msg.primitive_poses[0].position 
    box_pose.pose.orientation = msg.primitive_poses[0].orientation 
     
    # Add a box to planning scene
    scene.add_box(box_name, box_pose, size=(size_x,size_y,size_z))
    rospy.sleep(2.0)  # Wait for the update to be applied
    scene.attach_box(link="base_link", name=box_name)
    rospy.sleep(2.0)  # Wait for the update to be applied

    # known_objects = scene.get_known_object_names()
    # rospy.logwarn(scene.get_known_object_names())
    # if "table" not in known_objects:
    #     rospy.logwarn("Collision object 'table' is not in the planning scene!")
    # else:
    #     rospy.loginfo("Collision object 'table' is present in the planning scene.")
    # is_known = box_name in scene.get_known_object_names()

    # rospy.loginfo("All scene: %s", scene.get_known_object_names())
    # rospy.loginfo("Box is in the scene? : %s",is_known)

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10e_rg2_moveit_server')

    # Subscribe to the CollisionObject topic
    rospy.Subscriber("/collision_object", moveit_msgs.msg.CollisionObject, collision_callback)
    rospy.sleep(5.0)  # Wait for the update to be applied

    # box_name = "table"
    # size_x = 5
    # size_y = 5
    # size_z = 0.64
    # # x = msg.primitive_poses[0].position.x
    # # y = msg.primitive_poses[0].position.y
    # # z = msg.primitive_poses[0].position.z

    # box_pose = geometry_msgs.msg.PoseStamped()
    # # box_pose.header.frame_id = msg.id
    # #rospy.loginfo("Msg ID: %s", msg.id)
    # box_pose.header.frame_id = "base_link"
    # box_pose.pose.position = geometry_msgs.msg.Point(0,0,-0.4)
    # box_pose.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,0) 
     
    # # Add a box to planning scene
    # scene.add_box(box_name, box_pose, size=(size_x,size_y,size_z))

    # rospy.sleep(2.0)  # Allow time for the scene to update

    s = rospy.Service('ur10e_rg2_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
