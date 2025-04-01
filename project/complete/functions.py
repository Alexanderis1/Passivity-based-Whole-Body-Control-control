import numpy as np

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


