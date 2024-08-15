#!/usr/bin/env python
import numpy as np
import actionlib
import rospy

import common.utils as utils

from mpc_amiga.msg import plan_dispatchAction, plan_dispatchGoal

def feedback_cb(feedback):
    print(f"Current waypoint: {feedback.current_waypoint}")


def controller_client():
    client = actionlib.SimpleActionClient('execute_mpc_controller', plan_dispatchAction)
    client.wait_for_server()


    dl = 0.1
    global_cx, global_cy, global_cyaw, global_ck = utils.get_course_from_file(True, False, dl)
    goal = plan_dispatchGoal()

    goal.global_cx = global_cx
    goal.global_cy = global_cy
    goal.global_cyaw = global_cyaw
    goal.global_ck = global_ck
    goal.description = "between_rows"

    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Controller executing {goal.description} execution status is {result.success}")


if __name__ == '__main__':
    rospy.init_node('mpc_controller_client_straight')
    controller_client()