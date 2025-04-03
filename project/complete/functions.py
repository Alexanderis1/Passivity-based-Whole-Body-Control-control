import numpy as np
# Alessandro's functions
def compute_robot_dynamics(M, C, tau_g, tau):

    # M => Inertia Matrix (n x n)
    # C => Coriolis Matrix + C terms (n x n)
    # tau_g => Applied Gravity Vector (n)
    # tau => Applied Torques Vector (n)
    

    q_ddot = np.linalg.inv(M) @ (tau - C @ tau - tau_g)
    return q_ddot

    # q_ddot => Generalized Acceleration (n)



def compute_robot_dynamics_with_external_forces(M, C, tau_g, S, tau_j, tau_int, L_all, w_all):

    # M => Inertia Matrix (n x n)
    # C => Coriolis Matrix + C terms (n x n)
    # tau_g => Applied Gravity Vector (n)
    # S => Selection Matrix (n x n_act)
    # tau_j => Vector of Joints Applied Forces (n_act)
    # tau_int => Vector of Internal Forces (n_act)
    # L_all => Matrix Jac[Link] <=> External Force (n x m)
    # w_all => Vector of External Forces (m) 


    tau = S @ (tau_j + tau_int) + L_all.T @ w_all
    q_ddot = np.linalg.inv(M) @ (tau - C @ tau - tau_g)
    return q_ddot

    # q_ddot => Generalized Acceleration (n)



def compute_task_space_dynamics(J, J_dot, M, C, q_dot, q_ddot):

    # J => Jacobian wrt task (m x n)
    # J_dot => d[J] (m x n)
    # M => Inertia Matrix (n x n)
    # C => Coriolis Matrix + C terms (n x n)
    # q_dot => Generalized Velocity (n)
    # q_ddot => Generalized Acceleration (n)

    x_dot = J @ q_dot
    x_ddot = J @ q_ddot + J_dot @ q_dot
    Q_k = J @ np.linalg.inv(M) @ C - J_dot
    return x_dot, x_ddot, Q_k

    # x_dot => Velocity in the task space (m)
    # x_ddot => d[x_dot] (m)
    # Q_k => Coriolis Matrix in the task space (m x n)



def compute_task_space_matrices(J, M):

    # J => Jacobian wrt task (m x n)
    # M => Inertia Matrix (n x n)


    M_k = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)
    T_k = M_k @ J @ np.linalg.inv(M)
    return M_k, T_k

    # M_k => Inertia Matrix in the task space (m x m)
    # T_k => Weighted pseudoinverse of the Jacobian (m x n)

# Daniele's functions

def compute_task_errors(x_dot, x_ddot, x_dot_ref, x_ddot_ref):

    # x_dot => Velocity in the task space (m)
    # x_ddot => d[x_dot] (m)
    # x_dot_ref => Desired x_dot (m)
    # x_ddot_ref => Desired x_ddot (m)


    x_dot_error = x_dot - x_dot_ref
    x_ddot_error = x_ddot - x_ddot_ref
    return x_dot_error, x_ddot_error

    # x_dot_error => Difference from the reference velocity (m)
    # x_ddot_error => Difference from the reference acceleration (m)


def compute_desired_force(tasks,pos_error,vel_error,ff,gravity_torque,current,Jdot,M) :

    "compute the desirede force f_desired as the stack of the fk forces"
    # task = ['lfoot', 'rfoot', 'com', 'torso', 'base', 'joints']  as in the origina file 
    #same for pos_error, vel_error, ff  

    f_des = np.array([]).reshape(0, 1)


    
    for task in tasks :
        
        # Mk= compute_Mk(task)                                  #  I assume the other code have some function like that , when will merge code 
        # Tk= compute_Tk(task)                                # we eventually change the input of that function according to yours 
        
        
        Mk,Tk=compute_task_space_matrices(Jdot[task], M)    #according to Alessandro function  

        Dk= compute_Dk(task)
        Ck= compute_Ck(task)
        Qk= compute_Qk(task)
        Kk= compute_Kk(task)
        f=Tk@gravity_torque + Mk@Qk@current['joint']['vel'] + Mk @ ff[task] + (Ck +Dk) @ vel_error[task] +Kk@ pos_error[task]
        f_des = np.vstack((f_des, f.reshape(-1, 1)))
    return f_des





def joint_torque_limit(robot):
        dim=robot.getNumDofs()
        limit=np.zeros(dim-6)
        for i in range(dim) :
          if i > 5 :
           #joint=robot.getDof(i).getName() 
           dof=robot.getDof(i)
           effort_limit = dof.getForceUpperLimit() 
           #print(joint,effort_limit)
           limit[i-6]=effort_limit
        return limit
          



def create_U(support,foot_id,n_act,Jdot):
    S=np.hstack(np.zeros((n_act,6)),np.eye(n_act))     #S= 24 x 30 
    if support == 'ds':
        LEE=np.hstack(Jdot['lfoot'].T,Jdot['lfoot'].T)      
    elif foot_id == 'lfoot' :
        LEE=Jdot['lfoot'].T
    else :
       LEE=Jdot['rfoot'].T
    return np.hstack(S,LEE.T)
    
