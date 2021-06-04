bin/env python

from __future__ import division, print_function
#
# import argparse
# from pathlib2 import Path
#
# import cv_bridge
import franka_msgs.msg
import geometry_msgs.msg
import numpy as np
import rospy
import sensor_msgs.msg

from yolov5_ros.msg import yolo_object

from vgn.utils import ros_utils
from vgn.utils.transform import Rotation, Transform
from vgn.utils.panda_control import PandaCommander



class PandaGraspController(object):
    def __init__(self):
        self.robot_error = False

        self.base_frame_id = rospy.get_param("~base_frame_id")
        self.tool0_frame_id = rospy.get_param("~tool0_frame_id")
        self.T_tool0_tcp = Transform.from_dict(rospy.get_param("~T_tool0_tcp"))  # TODO
        self.T_tcp_tool0 = self.T_tool0_tcp.inverse()
        self.finger_depth = rospy.get_param("~finger_depth")
        # self.size = 6.0 * self.finger_depth
        # self.scan_joints = rospy.get_param("~scan_joints")

        self.setup_panda_control()
        self.tf_tree = ros_utils.TransformTree()
        # self.define_workspace()
        # self.create_planning_scene()
        # self.tsdf_server = TSDFServer()
        # self.plan_grasps = self.select_grasp_planner(args.model)
        # self.logger = Logger(args.logdir, args.description)

        rospy.loginfo("Ready to take action")

    def setup_panda_control(self):
        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            franka_msgs.msg.FrankaState,
            self.robot_state_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/joint_states", sensor_msgs.msg.JointState, self.joints_cb, queue_size=1
        )
        self.pc = PandaCommander()
        self.pc.move_group.set_end_effector_link(self.tool0_frame_id)


    def robot_state_cb(self, msg):
        detected_error = False
        if np.any(msg.cartesian_collision):
            detected_error = True
        for s in franka_msgs.msg.Errors.__slots__:
            if getattr(msg.current_errors, s):
                detected_error = True
        if not self.robot_error and detected_error:
            self.robot_error = True
            rospy.logwarn("Detected robot error")

    def joints_cb(self, msg):
        self.gripper_width = msg.position[7] + msg.position[8]

    def recover_robot(self):
        self.pc.recover()
        self.robot_error = False
        rospy.loginfo("Recovered from robot error")

    def get_pos(self):
        # self.detected_objects_topic = "/detected_objects_in_image"
        pose = rospy.wait_for_message("/detected_objects_in_image", yolo_object)
        cls, pos = pose.Class, np.array([pose.x/1000, pose.y/1000, pose.z/1000, 1])
        return cls, pos

    def robot_begin(self):
        T_base_grasp = Transform.from_list([1, 0, 0, 0, 0.45, 0, 0.6])
        self.pc.goto_pose(T_base_grasp * self.T_tcp_tool0)

    def run(self):

        self.pc.move_gripper(0.08)
        # self.pc.home()
        #  x y z w
        self.robot_begin()
        rospy.sleep(0.1)
        rospy.loginfo("Grasp execution")
        T_base_grasp = Transform.from_list([1, 0, 0, 0, 0.45, 0, 0.6])
        self.pc.goto_pose(T_base_grasp * self.T_tcp_tool0)
        0.46602357 - 0.06991329
        0.21936246
 
        self.robot_begin()

    def execute_grasp(self, grasp):
        T_base_grasp = grasp

        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, -0.05])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, -0.05])
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp
        T_base_retreat = T_base_grasp * T_grasp_retreat

        self.pc.goto_pose(T_base_pregrasp * self.T_tcp_tool0, velocity_scaling=0.2)
        self.approach_grasp(T_base_grasp)

        if self.robot_error:
            return False
        rospy.loginfo("Grasp begin")
        self.pc.grasp(width=0.0, force=20.0)
        rospy.loginfo("Grasp finish")
        if self.robot_error:
            return False

        self.pc.goto_pose(T_base_retreat * self.T_tcp_tool0)

        # lift hand
        T_retreat_lift_base = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_lift = T_retreat_lift_base * T_base_retreat
        self.pc.goto_pose(T_base_lift * self.T_tcp_tool0)

        if self.gripper_width > 0.004:
            return True
        else:
            return False

    def approach_grasp(self, T_base_grasp):
        self.pc.goto_pose(T_base_grasp * self.T_tcp_tool0)

    def drop_pet(self):
        self.pc.goto_joints(
            [0.678, 0.097, 0.237, -1.63, -0.031, 1.756, 0.931], 0.2, 0.2
        )
        self.pc.move_gripper(0.08)

    def drop_metal(self):
        self.pc.goto_joints(
            [-0.678, 0.097, 0.237, -1.63, -0.031, 1.756, 0.931], 0.2, 0.2
        )
        self.pc.move_gripper(0.08)


def main():

    rospy.init_node("panda_grasp")
    print('panda is starting')
    panda_grasp = PandaGraspController()
    panda_grasp.run()


if __name__ == "__main__":
    main()
