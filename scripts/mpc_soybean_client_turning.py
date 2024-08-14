#!/usr/bin/env python
import numpy as np
import actionlib
import rospy

import common.utils as utils
from amiga_path_planning.msg import full_path


from mpc_amiga.msg import plan_dispatchAction, plan_dispatchGoal

class MPCControllerClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('execute_mpc_controller', plan_dispatchAction)
        rospy.Subscriber("/turning_path", full_path, self.planning_cb)
        self.rate = rospy.Rate(10) # 10hz

        self.turning = False

        self.dl = 0.1

        self.global_cx = None
        self.global_cy = None
        self.global_cyaw = None
        self.global_ck = None
        self.global_sp = None

        self.send_goal = True

    def planning_cb(self, plan):
        self.turning = True
        self.send_goal = True
        self.global_cx = plan.global_cx
        self.global_cy = plan.global_cy
        self.global_cyaw = plan.global_cyaw
        self.global_ck = plan.global_ck
        self.global_sp = plan.global_sp

    def read_plan_from_file(self):
        self.global_cx, self.global_cy, self.global_cyaw, self.global_ck = utils.get_course_from_file(True, False, self.dl)
        self.global_sp = []

    def feedback_cb(self, feedback):
        print(f"Current waypoint: {feedback.current_waypoint}")

    def run(self):
        self.client.wait_for_server()

        while not rospy.is_shutdown():

            if self.turning:
                self.turning = False
                description = "turning"

                goal = plan_dispatchGoal()
                goal.global_cx = self.global_cx
                goal.global_cy = self.global_cy
                goal.global_cyaw = self.global_cyaw
                goal.global_ck = self.global_ck
                goal.global_sp = self.global_sp
                goal.description = description

                self.client.send_goal(goal, feedback_cb=self.feedback_cb)

                self.client.wait_for_result()
                result = self.client.get_result()

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mpc_controller_client')
    controller = MPCControllerClient()
    controller.run()