#!/usr/bin/env python
import numpy as np
import casadi as ca     
import time


def MPC(self_state, goal_state):
    opti = ca.Opti()
    ## parameters for optimization
    T = 0.10
    N = 10  # MPC horizon
    v_max = 1.5
    # omega_max = 0.523 | 5.0
    omega_max = 0.30
    L = 608.54e-3 #wheel base
    Q = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 0.1]])
    R = np.array([[0.4, 0.0], [0.0, 0.2]])
    goal = goal_state[:,:3]
    opt_x0 = opti.parameter(3) #vector3-parameter
    opt_controls = opti.variable(N, 2) #matrix10*2-variable
    v = opt_controls[:, 0]
    omega = opt_controls[:, 1]

    ## state variables
    opt_states = opti.variable(N+1, 3) #matrix11*3:x,y,psi
    x = opt_states[:, 0] #
    y = opt_states[:, 1]
    psi = opt_states[:, 2]

    ## create function for F(x)

    f = lambda x_, u_: ca.vertcat(*[u_[0]*ca.cos(x_[2]), u_[0]*ca.sin(x_[2]), u_[0]*ca.tan(u_[1])/L]) # f---caculate speed and output to column vector format

    ## set constraint
    opti.subject_to(opt_states[0, :] == opt_x0.T)

    # Admissable Control constraints
    opti.subject_to(opti.bounded(-v_max, v, v_max)) #declare double inequalities fot linear-speed
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max)) #declare double inequalities for angular-speed

    # System Model constraints---between front and rear points
    for i in range(N):
        x_next = opt_states[i, :] + T*f(opt_states[i, :], opt_controls[i, :]).T
        opti.subject_to(opt_states[i+1, :]==x_next)

    #### cost function
    obj = 0 
    for i in range(N):
        obj = obj + 1.2*ca.mtimes([(opt_states[i, :] - goal[[i]]), Q, (opt_states[i, :]- goal[[i]]).T]) + 0.5*ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T]) 
    # obj = obj + 2*ca.mtimes([(opt_states[0, :] - goal[[0]]), Q, (opt_states[0, :]- goal[[0]]).T])

    opti.minimize(obj)
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-6, 'ipopt.acceptable_obj_change_tol':1e-6}
    opti.solver('ipopt',opts_setting)
    opti.set_value(opt_x0, self_state[:,:3])

    try:
        sol = opti.solve()
        u_res = sol.value(opt_controls)
        state_res = sol.value(opt_states)
    except:
        state_res = np.repeat(self_state[:3],N+1,axis=0)
        u_res = np.zeros([N,2])

    return state_res, u_res
