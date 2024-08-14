#!/usr/bin/env python
import numpy as np
import actionlib
import rospy 
import acado
import sys
import math

from mpc_amiga.msg import plan_dispatchAction, plan_dispatchResult, plan_dispatchFeedback

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf

import common.global_defs as defs
import common.utils as utils
import common.robot_motion_skid_steer as bot_model
from common.path import A_star_path
from common.utils_viz import markerVisualization

class mpc_controller_server():
    def __init__(self):

        self.server = actionlib.SimpleActionServer('execute_mpc_controller', plan_dispatchAction, self.execute, False)
        self.server.start()

        self.robot_state = bot_model.kinematics(0, 0, 0, 0)
        self.global_path = A_star_path()
        self.dt = 0.1

        self.yaw_prev_ = 0.
        self.vel_up = 0
        self.vel_down = defs.TARGET_SPEED
        self.w_up = 0        

        self.count_init = 0
        self.can_delete_file = True
        self.nav_glob_finished = False
        self.viz_utils = markerVisualization()      

        self.odomSubs = rospy.Subscriber("/odometry/filtered", Odometry, self.callbackFilteredOdom)
        self.controlPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)      
        
        self.turning = False

        self.rate = rospy.Rate(10) # 10hz

    def callbackFilteredOdom(self, odom_msg):
        current_time = rospy.Time.now()
        #robotPoseEstimate = PoseStamped()
        #robotPoseEstimate.pose.position = odom_msg.pose.pose.position
        # robotPoseEstimate.pose.orientation = odom_msg.pose.pose.orientation
        x_meas = odom_msg.pose.pose.position.x
        y_meas = odom_msg.pose.pose.position.y
        quat_pose = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        euler_meas = tf.transformations.euler_from_quaternion(quat_pose) #RPY

        v_meas = odom_msg.twist.twist.linear.x
        if abs(v_meas)<1e-4:
            v_meas = 0
        w_meas = odom_msg.twist.twist.angular.z
        
        #yaw_inRange = euler_meas[2]%(2*math.pi)
        yaw_inRange = utils.wrapTopm2Pi(euler_meas[2], self.yaw_prev_)
        #if yaw_inRange < 0:
        #    yaw_inRange = 2*math.pi + yaw_inRange
        #elif yaw_inRange > 2*math.pi:
        #    yaw_inRange = 2*math.pi - yaw_inRange
        
        # print("Here")
        # print(yaw_inRange*180/math.pi)
        #print(yaw_prev_*180/math.pi)
        self.robot_state.set_meas(x_meas, y_meas, yaw_inRange, v_meas, w_meas)
        self.yaw_prev_ = yaw_inRange        


    #here I need to create the msg to send - chech in case of a carlike
    def make_twist_msg(self, accel, acc_omega, goalData, warn_w, yaw_meas):
        global vel_up
        global vel_down
        global w_up
        dt_in = 0.1
        cmd = Twist()
        if not goalData[0]:
            print("accel", accel)
            # cmd_vel_ = vel_up + dt_in*defs.TARGET_SPEED/defs.T_RAMP_UP
            cmd_vel_ = self.vel_up + dt_in*accel
            self.vel_up = cmd_vel_

            cmd_w_ = self.w_up + dt_in*acc_omega
            self.w_up = cmd_w_ 

            # if cmd_vel_ < defs.MAX_TARGET_SPEED and cmd_vel_ >defs.MIN_TARGET_SPEED: #if cmd_vel_ < defs.TARGET_SPEED:
            #     cmd.linear.x = cmd_vel_
            # elif cmd_vel_ > defs.MAX_TARGET_SPEED:    
            #     cmd.linear.x = defs.MAX_TARGET_SPEED #cmd.linear.x = defs.TARGET_SPEED
            # elif cmd_vel_ < defs.MIN_TARGET_SPEED:
            #     cmd.linear.x = defs.MIN_TARGET_SPEED
            cmd.linear.x = cmd_vel_
            if not warn_w:
                cmd.angular.z =  self.w_up# + acc_omega*dt_in
            else:
                self.w_up = 0
                cmd.angular.z =  0# + acc_omega*dt_in
        else:
            cmd_w_ = self.w_up + dt_in*acc_omega
            self.w_up = cmd_w_ 
            dToGoal = goalData[1]
            #cmd_vel_ = vel_down - dt_in*vel_down/defs.T_RAMP_DOWN
            #cmd_vel_ = vel_down - dt_in*defs.TARGET_SPEED/defs.T_RAMP_DOWN
            cmd_vel_ = self.vel_down - dt_in*self.vel_down*self.vel_down/(5*dToGoal)
            print(dToGoal)
            if dToGoal < defs.DIST_TO_GOAL_STOP: #was .1
                cmd.linear.x = 0
                cmd.angular.z = 0
                print("Goal Reached")
                self.vel_down = defs.MIN_TARGET_SPEED
                # self.can_delete_file, self.nav_glob_finished = utils.delete_pruning_points_from_file(self.can_delete_file, self.nav_glob_finished)
                rospy.set_param('nav_stat', True)
                
            else:
                cmd.linear.x = cmd_vel_
                self.vel_down = cmd_vel_
                cmd.angular.z = 0.3*self.w_up

            #cmd.angular.z = w_up
        print("output_vel", cmd.linear.x)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        
        return cmd


    def iterative_linear_mpc_control(self, xref, dref, oa, ow):
        """
        MPC contorl with updating operational point iteraitvely
        """
        x0 = [self.robot_state.x, self.robot_state.y, self.robot_state.v, self.robot_state.yaw, self.robot_state.w]  
        if oa is None or ow is None:
            oa = [0.0] * defs.T
            ow = [0.0] * defs.T

        for i in range(defs.MAX_ITER):
            xbar = self.robot_state.predict_motion(oa, ow, defs.T)
            #print(xref.shape)
            poa, podw = oa[:], ow[:]
            oa, ow, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(ow - podw))  # calc u change value
            if du <= defs.DU_TH:
                break
        else:
            print("Iterative is max iter")

        #robot_state.refreshState()
        return oa, ow, ox, oy, oyaw, ov    

    # MPC using ACADO
    def linear_mpc_control(self, xref, xbar, x0, dref):
        # see acado.c for parameter details
        _x0=np.zeros((1, defs.NX))  
        X=np.zeros((defs.T+1, defs.NX))
        U=np.zeros((defs.T, defs.NU))    
        Y=np.zeros((defs.T, defs.NY))    
        yN=np.zeros((1, defs.NYN))    
        _x0[0,:]=np.transpose(x0)  # initial state    
        for t in range(defs.T):
            Y[t,:] = np.transpose(xref[:,t])  # reference state
            X[t,:] = np.transpose(xbar[:,t])  # predicted state
        X[-1,:] = X[-2,:]    
        yN[0,:]=Y[-1,:defs.NYN]         # reference terminal state
        #print(Y.shape)
        X, U = acado.mpc(0, 1, _x0, X,U,Y,yN, np.transpose(np.tile(defs.Q,defs.T)), defs.Qf, 0)    
        ox_mpc = utils.get_nparray_from_matrix(X[:,0])
        oy_mpc = utils.get_nparray_from_matrix(X[:,1])
        ov_mpc = utils.get_nparray_from_matrix(X[:,2])
        oyaw_mpc = utils.get_nparray_from_matrix(X[:,3])
        oa_mpc = utils.get_nparray_from_matrix(U[:,0])
        ow_mpc = utils.get_nparray_from_matrix(U[:,1])
        return oa_mpc, ow_mpc, ox_mpc, oy_mpc, oyaw_mpc, ov_mpc      
    
    def execute(self, goal):
        feedback = plan_dispatchFeedback()
        result = plan_dispatchResult()
        result.success = False
        

        init_route = 1
        target_ind_move = 0
        init_accel = 0.1
        dl = 0.1

        args = rospy.myargv(argv=sys.argv)
        #print(args)
        if len(args)!=2:
            print("ERROR:Provide fresh_start argument ")
            sys.exit(1)
        is_fresh_start = args[1] 
        

        global_cx = list(goal.global_cx)
        global_cy = list(goal.global_cy)
        global_cyaw = list(goal.global_cyaw)
        global_ck = list(goal.global_ck)
        if goal.description=="between_rows":
            self.turning = False
            global_sp = utils.calc_speed_profile_1(global_cx, global_cy, global_cyaw, global_ck)
        else:
            self.turning = True
            global_sp = np.array(goal.global_sp)*defs.MAX_TARGET_SPEED
            global_sp = global_sp.tolist()
        # global_sp = utils.calc_speed_profile_2(global_cx, global_cy, global_cyaw, defs.TARGET_SPEED)

        #get the stopping points
        # ppx, ppy = utils.get_pruning_points(is_fresh_start) #if is a fresh_start use the original_file, otherwise use the cropped one
        ppx = [global_cx[-1]]
        ppy = [global_cy[-1]]

        cx, cy, cyaw, ck, sp = None, None, None, None, None

        #this is used to visualize the path on rviz
        current_path = self.viz_utils.create_path(global_cx, global_cy)    

        self.robot_state.get_current_meas_state()      
        #print(is_fresh_start)
        if is_fresh_start == "1":
            #print("Here1")
            target_ind = 0
            warn_w = False
            flag_start_stragiht = False
        else:
            #print("Here0")
            target_ind = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)    

        ow, oa = None, None
        global_cyaw = utils.smooth_yaw(global_cyaw)
        prune_done = 1 #comes from the parameter server, 1 when the robot is good to go
        prune_done_ = 1
        index_pruning = 0
        
        while not rospy.is_shutdown():

            if self.server.is_preempt_requested():
                rospy.loginfo('Goal preempted')
                self.server.set_preempted()
                return   
                     
            prune_done = rospy.get_param("/pruning_status")    
            self.viz_utils.pathPub.publish(current_path)
            self.viz_utils.publish_marker(ppx, ppy)
            if not prune_done or init_route:
                self.can_delete_file = True
                self.robot_state.get_current_meas_state()
                print("Target_ind", target_ind)

                if index_pruning < len(ppx) and not self.turning:
                    print("turning", self.turning)
                    if not init_route:
                        target_ind = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)

                    cx, cy, cyaw, ck, sp = utils.crop_global_plan(global_cx, global_cy, global_cyaw, global_ck, global_sp, ppx[index_pruning], ppy[index_pruning], target_ind)
                    goal = [cx[-defs.OFFSET_TO_GOAL], cy[-defs.OFFSET_TO_GOAL]]
                    offset_stop = defs.OFFSET_TO_GOAL
                else:
                    target_ind_ = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)
                    cx = global_cx[target_ind_:]
                    cy = global_cy[target_ind_:]
                    cyaw = global_cyaw[target_ind_:]
                    ck = global_ck[target_ind_:]
                    sp = global_sp[target_ind_:]
                    goal = [cx[-1], cy[-1]]
                    #print(goal)
                    offset_stop = 0
                target_ind, _ = utils.calc_nearest_index(self.robot_state, cx, cy, cyaw, 0)                

                # initial yaw compensation
                if self.robot_state.yaw - cyaw[target_ind] >= math.pi:
                    self.robot_state.yaw -= math.pi * 2.0
                elif self.robot_state.yaw - cyaw[target_ind] <= -math.pi:
                    self.robot_state.yaw += math.pi * 2.0                        
                
                prune_done_ = prune_done

            if prune_done:
                diff_prune = prune_done_ - prune_done
                if diff_prune != 0 or init_route:
                    current_time_ = rospy.Time.now().to_sec()
                    rospy.set_param('nav_stat', False)
                    index_pruning+=1
                    target_ind_move = target_ind
                    if not init_route:
                        flag_start_stragiht = True
                    init_route = 0                

                prune_done_ = prune_done
                self.robot_state.get_current_meas_state()
                xref, target_ind_move, dref = utils.calc_ref_trajectory_v1(
                    self.robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, self.dt, target_ind_move)
                current_local_waypoint = [cx[target_ind_move], cy[target_ind_move]]
                feedback.current_waypoint = current_local_waypoint
                self.server.publish_feedback(feedback)
                self.viz_utils.pubish_single_marker(current_local_waypoint[0], current_local_waypoint[1])


                if self.robot_state.yaw - cyaw[target_ind_move] >= math.pi:
                    self.robot_state.yaw -= math.pi * 2.0
                elif self.robot_state.yaw - cyaw[target_ind_move] <= -math.pi:
                    self.robot_state.yaw += math.pi * 2.0          

                oa, ow, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
                    xref, dref, oa, ow)                

                if ow is not None:
                    wi, ai = ow[0], oa[0]              
                # warm-up solver
                if True: #target_ind < 10:
                    if abs(self.robot_state.v) < 0.05:
                        if sp[target_ind_move]<0:
                            ai = -0.1
                        else:
                            #print(robot_state.v)
                            ai = init_accel
                            wi = 0.01                   
                init_accel = oa[0]

                goalData = utils.check_goal(self.robot_state.get_current_pos_meas(), goal, target_ind_move, len(cx)-offset_stop)             
                if is_fresh_start == "0" or flag_start_stragiht:    
                    latest_time = rospy.Time.now().to_sec()
                    if (abs(latest_time - current_time_) < 2.0):
                        w_i = 0.0 
                        warn_w = True
                        ow = [0.0] * defs.T
                        # print("Here")                     
                    else:
                        flag_start_stragiht = False
                        warn_w = False    

                print('Yaw:', self.robot_state.yaw)
                print('Goal yaw', cyaw[target_ind_move])
                print('Goal sp', sp[target_ind_move])
                cmd_command = self.make_twist_msg(ai, wi, goalData, warn_w, self.robot_state.yaw)

                if self.nav_glob_finished:
                    result.success = True
                    self.server.set_succeeded(result)
                    print("Global navigation finished!!")
                self.controlPub.publish(cmd_command)
            self.rate.sleep()

        rospy.logwarn("Exiting without goal completion")
        print("DEBUG",len(global_cx))
        result = plan_dispatchResult()
        result.success = False
        self.server.set_aborted(result)



if __name__ == '__main__':
  rospy.init_node('mpc_controller_server')
  server = mpc_controller_server()
  rospy.spin()    