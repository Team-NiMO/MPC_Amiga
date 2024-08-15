#!/usr/bin/env python
import numpy as np
import actionlib
import rospy

import common.utils as utils
from std_msgs.msg import Bool

from mpc_amiga.msg import plan_dispatchAction, plan_dispatchGoal
from mpc_amiga.srv import file_plan_request


class MPCControllerClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('execute_mpc_controller', plan_dispatchAction)
        self.planning_srv = rospy.Service('file_reader', file_plan_request, self.handle_srv_planning)
        self.row_id = 0


    def feedback_cb(self, feedback):
        print(f"Current waypoint: {feedback.current_waypoint}")

    def handle_srv_planning(self, req):
        self.client.wait_for_server()

        response = Bool()

        if req.read_plan.data == True:
            global_cx, global_cy, global_cyaw, global_ck = utils.get_course_from_file_iter(self.row_id, 0.1)
            goal = plan_dispatchGoal()

            goal.global_cx = global_cx
            goal.global_cy = global_cy
            goal.global_cyaw = global_cyaw
            goal.global_ck = global_ck                

            goal.description = "between_rows"    

            self.client.send_goal(goal, feedback_cb=self.feedback_cb)     
            self.client.wait_for_result()   
            result = self.client.get_result()
            rospy.loginfo(f"Controller executing {goal.description} execution status is {result.success}")            
            if result:
                self.row_id+=1

            if global_cx:
                response.data = True
            else:
                response.data = False

        return response


if __name__ == '__main__':
    rospy.init_node('mpc_controller_client_straight')
    controller = MPCControllerClient()
    rospy.spin()