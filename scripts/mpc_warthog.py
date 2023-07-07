#!/usr/bin/env python
import rospy
import numpy as np
import acado
import math

import scipy.io as sio

import common.global_defs as defs
import common.utils as utils
import common.robot_motion_skid_steer as bot_model

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from visualization_msgs.msg import Marker
import tf


first_seen = False
isInitState = True

#initial robot state
robot_state = bot_model.kinematics(0, 0, 0, 0)
last_time = rospy.Time()
dt = 0.1
yaw_prev_ = 0
vel_up = 0
vel_down = defs.TARGET_SPEED
w_up = 0
count_init = 0

local_path_pub = rospy.Publisher("/localPath", Marker, queue_size=10)
mark = Marker()
def publish_marker(marker_type, pose, scale=[0.5,0.5,0.5], color=[1.,0.,0.]):
    
    mark.header.stamp = rospy.Time.now()
    mark.header.frame_id = "odom"
    mark.type = marker_type
    #mark.pose.position.x = pose[0]
    #mark.pose.position.y = pose[1]
    #mark.pose.position.z = 0
    mark.pose.orientation.x = 0
    mark.pose.orientation.y = 0
    mark.pose.orientation.z = 0
    mark.pose.orientation.w = 1

    #mark.action = mark.ADD
    #mark.lifetime = rospy.Duration(0)

    mark.scale.x = scale[0]
    mark.scale.y = scale[1]
    mark.scale.z = scale[2]
    mark.color.a = 1
    mark.color.r = color[0]
    mark.color.g = color[1]
    mark.color.b = color[2]

    for column in pose.T:
        point = Point()
        point.x = column[0]
        point.y = column[1]
        point.z = 1.0
        mark.points.append(point)

    local_path_pub.publish(mark)

# def publish_local_path(path, color=[1.,0.,0.]):
#     my_path = Path()
#     my_path.header.frame_id = 'odom'
#     # my_path.color.r = color[0]
#     # my_path.color.g = color[1]
#     # my_path.color.b = color[2]
#     for column in path.T:
#         pose = PoseStamped()
#         pose.pose.position.x = column[0]
#         pose.pose.position.y = column[1]
#         my_path.poses.append(pose)
#     local_path_pub.publish(my_path)


def iterative_linear_mpc_control(xref, dref, oa, ow):
    """
    MPC contorl with updating operational point iteraitvely
    """
    x0 = [robot_state.x, robot_state.y, robot_state.v, robot_state.yaw, robot_state.w]  
    if oa is None or ow is None:
        oa = [0.0] * defs.T
        ow = [0.0] * defs.T

    for i in range(defs.MAX_ITER):
        xbar = robot_state.predict_motion(oa, ow, defs.T)
        #print(xref.shape)
        poa, podw = oa[:], ow[:]
        oa, ow, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(ow - podw))  # calc u change value
        if du <= defs.DU_TH:
            break
    else:
        print("Iterative is max iter")

    #robot_state.refreshState()
    return oa, ow, ox, oy, oyaw, ov

# MPC using ACADO
def linear_mpc_control(xref, xbar, x0, dref):
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
    
def callbackFilteredOdom(odom_msg):
    #global last_time
    global yaw_prev_
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
    yaw_inRange = utils.wrapTopm2Pi(euler_meas[2], yaw_prev_)
    # print("Here")
    # print(yaw_inRange*180/math.pi)
    #print(yaw_prev_*180/math.pi)
    robot_state.set_meas(x_meas, y_meas, yaw_inRange, v_meas, w_meas)
    yaw_prev_ = yaw_inRange

    # if (last_time.to_nsec()==0):
    #     print("Here")
    #     robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
    #     last_time = current_time
    # elif (last_time.to_nsec() > 0):
    #     dt = current_time.to_sec() - last_time.to_sec() 
    #     if (~robot_state.IsFresh and dt>=0.1):
    #         print(dt)
    #         robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
    #         last_time = current_time         

#here I need to create the msg to send - chech in case of a carlike
def make_twist_msg(accel, acc_omega, dt_in_, goalData, w_meas):
    global vel_up
    global vel_down
    global w_up
    dt_in = 0.1
    cmd = Twist()
    if not goalData[0]:
        cmd_vel_ = vel_up + dt_in*defs.TARGET_SPEED/defs.T_RAMP_UP
        vel_up = cmd_vel_

        cmd_w_ = w_up + dt_in*acc_omega
        w_up = cmd_w_ 
        print(cmd_w_)
        if cmd_vel_ < defs.TARGET_SPEED:
            cmd.linear.x = cmd_vel_
        else:    
            cmd.linear.x = defs.TARGET_SPEED

        #if cmd_w_ < omega:
        #    cmd.angular.z = cmd_w_
        #else:
        #    cmd.angular.z = omega
        #print(acc_omega)
        cmd.angular.z =  w_up + acc_omega*dt_in
    else:
        dToGoal = goalData[1]
        cmd_vel_ = vel_down - dt_in*vel_down/defs.T_RAMP_DOWN
        #cmd_vel_ = vel_down - dt_in*defs.TARGET_SPEED*defs.TARGET_SPEED/dToGoal
        print(dToGoal)
        if dToGoal < 0.1:
            cmd.linear.x = 0
        else:
            cmd.linear.x = cmd_vel_
            vel_down = cmd_vel_

        cmd.angular.z = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    
    return cmd


def mpc_node():
    #global count_init
    rospy.init_node('mpc_jackal', anonymous=True)

    odomSubs = rospy.Subscriber("/odometry/filtered", Odometry, callbackFilteredOdom)
    controlPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pathPub = rospy.Publisher("/aPath", Path, queue_size=10)

    x_ref_all = np.zeros((4, 1), dtype = float) #debugging
    init_accel = 1
    #generate the paths
    dl =0.1
    #cx, cy, cyaw, ck = utils.get_straight_course(dl)
    #cx, cy, cyaw, ck = utils.get_straight_course2(dl)
    #cx, cy, cyaw, ck = utils.get_straight_course3(dl)
    #cx, cy, cyaw, ck = utils.get_forward_course(dl)
    #cx, cy, cyaw, ck = utils.get_switch_back_course(dl)
    cx, cy, cyaw, ck = utils.get_vineyard_course(dl)
    #cx, cy, cyaw, ck = utils.get_course_from_file(dl)

    sp = utils.calc_speed_profile(cx, cy, cyaw, defs.TARGET_SPEED)
    #sio.savemat('speed_profile.mat', {'speed_profile':sp})
    rate = rospy.Rate(10) # 10hz
    goal = [cx[-1], cy[-1]]

        
    my_path = Path()
    my_path.header.frame_id = 'odom'
    for x,y in zip(cx, cy):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y        
        my_path.poses.append(pose)
    

    # initial yaw compensation
    if robot_state.yaw - cyaw[0] >= math.pi:
        robot_state.yaw -= math.pi * 2.0
    elif robot_state.yaw - cyaw[0] <= -math.pi:
        robot_state.yaw += math.pi * 2.0

    target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, 0)
    
    ow, oa = None, None
    cyaw = utils.smooth_yaw(cyaw)
    #sio.savemat('cyaw_smoothed.mat', {'cyaw_smoothed':cyaw})
    
    while not rospy.is_shutdown():
        pathPub.publish(my_path)

        current_time = rospy.Time.now()

        robot_state.get_current_meas_state()

        #xref, target_ind, dref = utils.calc_ref_trajectory(
        #    robot_state, cx, cy, cyaw, ck, sp, dl, dt, target_ind)
        xref, target_ind, dref = utils.calc_ref_trajectory_v1(
            robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, dt, target_ind)
        #x_ref_all = np.append(x_ref_all, xref,axis = 1)
        #sio.savemat('x_ref_all.mat', {'x_ref_all':x_ref_all})

        
        publish_marker(Marker.POINTS, xref)
        #print(xref[3,:])
        oa, ow, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, dref, oa, ow)

        if ow is not None:
            wi, ai = ow[0], oa[0]
            #ai = oa[0]
            #wi = 0

        #if count_init < 50:
        #    wi = wi_/100
        #    count_init = count_init + 1
        #else:
        #    wi = wi_
                  
        # warm-up solver
        if True: #target_ind < 10:
            if abs(robot_state.v) < 0.05:
                if sp[target_ind]<0:
                    ai = -0.1
                else:
                    #print(robot_state.v)
                    ai = init_accel
                    wi = 0.1

        init_accel = oa[0]
        #apply the control signals
        dt_cmd = rospy.Time.now().to_sec() - current_time.to_sec()
        goalData = utils.check_goal(robot_state.get_current_pos_meas(), goal, target_ind, len(cx))
        #print(dt_cmd)
        cmd_command = make_twist_msg(ai, wi, dt_cmd, goalData, robot_state.w)
        controlPub.publish(cmd_command)

        
        rate.sleep()


if __name__ == '__main__':
    mpc_node()
    
