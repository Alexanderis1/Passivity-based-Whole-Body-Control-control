# General robot dynamic and task space equations

import numpy as np

# Robot DOF
n = 3  

# Task Space Dim
m = 2  

# Inertia Matrix
M = np.diag([10.0, 8.0, 5.0])

# Coriolis Matrix + C terms
C = np.array([[0.5, 0.2, 0.1],
              [0.2, 0.4, 0.2],
              [0.1, 0.2, 0.3]])

# Applied Gravity Vector
tau_g = np.array([2.0, 1.5, 1.0])

# Vector of Joints Applied Forces
tau_j = np.array([12.0, 7.0, 4.0])

# Vector of Internal Forces
tau_int = np.array([1.0, 0.5, 0.3])

# Selection Matrix
S = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

# Matrix Jac[Link] <=> External Force
L_all = np.array([[0.2, 0.1],
                  [0.1, 0.3],
                  [0.4, 0.2]])

# Vector of External Forces
w_all = np.array([3.0, 2.0])

# Generalized Velocity
q_dot = np.array([0.4, -0.3, 0.2])

# Jacobian wrt task
J = np.array([[0.8, 0.3, 0.2],
              [0.2, 0.6, 0.7]])

# d[J]
J_dot = np.array([[0.01, 0.02, 0.01],
                  [0.02, 0.01, 0.02]])

# Generalized Acceleration
q_ddot = np.linalg.inv(M) @ (S @ (tau_j + tau_int) + L_all.T @ w_all - C @ q_dot - tau_g)

# Velocity in the task space
x_dot = J @ q_dot
# Acceleration in the task space
x_ddot = J @ q_ddot + J_dot @ q_dot

# Coriolis Matrix in the task space
Q_k = J @ np.linalg.inv(M) @ C - J_dot

# Inertia Matrix in the task space
M_k = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)

#Weighted pseudoinverse of the Jacobian
T_k = M_k @ J @ np.linalg.inv(M)

print("q_ddot:", q_ddot)
print("x_dot:", x_dot)
print("x_ddot:", x_ddot)
print("Q_k:", Q_k)
print("M_k:", M_k)
print("T_k:", T_k)