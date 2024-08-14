#!/usr/bin/env python
import numpy as np
import actionlib
import rospy 
import acado
import sys
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf

from common.utils import cubic_spline_planner
import common.global_defs as defs
import common.utils as utils
import common.robot_motion_skid_steer as bot_model
from common.path import A_star_path
from common.utils_viz import markerVisualization



class mpc_controller():
    def __init__(self):
        rospy.init_node('mpc_controller')


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
        self.pathSubs = rospy.Subscriber("/visualization_path", Path, self.callbackPathPlanning)  
        
        self.controlPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)        
        
        self.rate = rospy.Rate(10) # 10hz

    def callbackPathPlanning(self, path_msg):
        x_path = []
        y_path = []
        yaw_path = []
        complete_path = path_msg.poses
        for pose in complete_path:
            x_path.append(pose.pose.position.x)
            y_path.append(pose.pose.position.y)
            quat_path = pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([quat_path.x, quat_path.y, quat_path.z, quat_path.w])
            yaw_path.append(euler[2])#RPY
        if len(x_path) > 0:
            self.global_path.update_path(x_path, y_path, yaw_path)
        else:
            print("Received an empty path!")

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
                self.can_delete_file, self.nav_glob_finished = utils.delete_pruning_points_from_file(self.can_delete_file, self.nav_glob_finished)
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
    
    def execute(self):
      

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

        # global_cx = list(goal.global_cx)
        # global_cy = list(goal.global_cy)
        # global_cyaw = list(goal.global_cyaw)
        # global_ck = list(goal.global_ck)
        # global_sp = utils.calc_speed_profile_1(global_cx, global_cy, global_cyaw, global_ck)
        # # global_sp = utils.calc_speed_profile_2(global_cx, global_cy, global_cyaw, defs.TARGET_SPEED)

        # #get the stopping points
        # ppx, ppy = utils.get_pruning_points(is_fresh_start) #if is a fresh_start use the original_file, otherwise use the cropped one

        

        # #this is used to visualize the path on rviz
        # current_path = self.viz_utils.create_path(global_cx, global_cy)    

        # self.robot_state.get_current_meas_state()      

        #print(is_fresh_start)
        # if is_fresh_start == "1":
        #     #print("Here1")
        #     target_ind = 0
        #     warn_w = False
        #     flag_start_stragiht = False
        # else:
        #     #print("Here0")
        #     target_ind = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)    

        # global_cyaw = utils.smooth_yaw(global_cyaw)

        cx, cy, cyaw, ck, sp = None, None, None, None, None
        ow, oa = None, None
        target_ind = 0
        index_pruning = 0
        
        while not rospy.is_shutdown():
            if self.global_path.new_path_acq:
                rospy.set_param('nav_stat', False)    

                prune_done = 1 #comes from the parameter server, 1 when the robot is good to go
                prune_done_ = 1      

                global_cx_, global_cy_, global_cyaw_ = self.global_path.x_path, self.global_path.y_path, self.global_path.yaw_path
                global_cx, global_cy, global_cyaw,  global_ck, _ = cubic_spline_planner.calc_spline_course(global_cx_, global_cy_, 0.1)
                current_path = self.viz_utils.create_path(global_cx, global_cy)
                global_cyaw = utils.smooth_yaw(global_cyaw)
                ppx = [global_cx[-1], global_cx[-1]]
                ppy = [global_cy[-1], global_cy[-1]]
                global_sp = utils.calc_speed_profile_1(global_cx, global_cy, global_cyaw, global_ck)
                init_route = True

                if len(ppx) > 0:
                    self.viz_utils.pubish_single_marker(ppx[0], ppy[0], True)
                self.viz_utils.pathPub.publish(current_path)

                if len(global_cx)>0:
                    if init_route:
                        can_delete_file = True
                        self.robot_state.get_current_meas_state()
                        print("Target_ind", target_ind)
                        if index_pruning < len(ppx):
                            if not init_route:
                                #target_ind = target_ind_move - defs.OFFSET_TO_GOAL
                                target_ind = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)

                            cx, cy, cyaw, ck, sp = utils.crop_global_plan(global_cx, global_cy, global_cyaw, global_ck, global_sp, ppx[index_pruning], ppy[index_pruning], target_ind)
                            #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cx_test.mat', {'cx':cx})
                            #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cy_test.mat', {'cy':cy})
                            #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cyaw_test.mat', {'cyaw':cyaw})
                            if is_global_nav:
                                goal = [cx[-defs.OFFSET_TO_GOAL], cy[-defs.OFFSET_TO_GOAL]]
                            else:
                                goal = [ppx[0], ppy[0]]

                            #print(target_ind)
                            offset_stop = defs.OFFSET_TO_GOAL
                        else:
                            target_ind_ = utils.calc_nearest_index_pruning(self.robot_state.x, self.robot_state.y, global_cx, global_cy, 0)
                            cx = global_cx[target_ind_:]
                            cy = global_cy[target_ind_:]
                            cyaw = global_cyaw[target_ind_:]
                            ck = global_ck[target_ind_:]
                            sp = global_sp[target_ind_:]
                            if is_global_nav or len(ppx)==0 :
                                goal = [cx[-1], cy[-1]]
                            else:
                                goal = [ppx[0], ppy[0]]
                            #print(goal)
                            offset_stop = 0
                        #print(goal)

                        target_ind, _ = utils.calc_nearest_index(self.robot_state, cx, cy, cyaw, 0)

                        # initial yaw compensation
                        if self.robot_state.yaw - cyaw[target_ind] >= math.pi:
                            self.robot_state.yaw -= math.pi * 2.0
                        elif self.robot_state.yaw - cyaw[target_ind] <= -math.pi:
                            self.robot_state.yaw += math.pi * 2.0                        
                        
                        #latest_read = robot_state.yaw
                        #init_route = 0           


                    if not init_route:
                        flag_start_stragiht = True
                    else:
                        current_time_ = rospy.Time.now().to_sec()
                        rospy.set_param('nav_stat', False)
                        index_pruning+=1
                        target_ind_move = target_ind
                            
                        init_route = 0

                    self.robot_state.get_current_meas_state()
                    #   robot_state.yaw = -1*robot_state.yaw
                    #xref, target_ind_move, dref = utils.calc_ref_trajectory(
                    #    robot_state, cx, cy, cyaw, ck, sp, dl, dt, target_ind_move)            
                    xref, target_ind_move, dref = utils.calc_ref_trajectory_v1(
                        self.robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, self.dt, target_ind_move)
                    self.viz_utils.pubish_single_marker(cx[target_ind_move], cy[target_ind_move])
                    #x_ref_all = np.append(x_ref_all, xref,axis = 1)
                    #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/x_ref_all.mat', {'x_ref_all':x_ref_all})
                    #print(target_ind_move)
                    

                    if self.robot_state.yaw - cyaw[target_ind_move] >= math.pi:
                        self.robot_state.yaw -= math.pi * 2.0
                    elif self.robot_state.yaw - cyaw[target_ind_move] <= -math.pi:
                        self.robot_state.yaw += math.pi * 2.0
                    #publish_marker(Marker.POINTS, xref)
                    #print(xref[3,:])
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
                    #print(goal)
                    #apply the control signals
                    #dt_cmd = rospy.Time.now().to_sec() - current_time.to_sec()

                    goalData = utils.check_goal(self.robot_state.get_current_pos_meas(), goal, target_ind_move, len(cx)-offset_stop)
                    done_navigation = goalData[0]
                    #print(dt_cmd)
                    #warn_w = False
                    #if is_fresh_start == "0" or diff_prune != 0:
                    if is_fresh_start == "0" or flag_start_stragiht:    
                        latest_time = rospy.Time.now().to_sec()
                        if (abs(latest_time - current_time_) < 2.0):
                            w_i = 0.0 
                            warn_w = True
                            ow = [0.0] * defs.T
                            # print("Here")
                            #yaw_prev_ = latest_yaw#((-yaw_prev_ + math.pi) % (2*math.pi) - math.pi)*-1
                            #if robot_state.yaw - cyaw[target_ind] >= math.pi:
                            #    robot_state.yaw -= math.pi * 2.0
                            #elif robot_state.yaw - cyaw[target_ind] <= -math.pi:
                            #    robot_state.yaw += math.pi * 2.0                        

                        else:
                            flag_start_stragiht = False
                            warn_w = False
                    print('Yaw:', self.robot_state.yaw)
                    print('Goal yaw', cyaw[target_ind_move])
                    print(warn_w)
                    cmd_command, global_nav_finished = self.make_twist_msg(ai, wi, goalData, warn_w, self.robot_state.yaw, is_global_nav)
                    is_global_nav = global_nav_finished #global nav finished is true is still in global nav, false if it finished
                    # if nav_glob_finished:
                    #     print("Global navigation finished - Exiting ...")
                    #     break
                    self.controlPub.publish(cmd_command)

            self.rate.sleep()


if __name__ == '__main__':
  
  controller = mpc_controller()
  controller.execute()
  rospy.spin()    