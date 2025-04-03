import numpy as np
from utils import *
# 1 Step

def compute_task_space_matrices(J, M):

    # J => Jacobian wrt task (m x n)
    # M => Inertia Matrix (n x n)


    M_k = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)
    T_k = M_k @ J @ np.linalg.inv(M)
    return M_k, T_k




# 2 Step

def compute_damping_stiffness(hrp4, J_k, alpha=10, beta=50.0):
    """
    Compute task-space damping (D_k) and stiffness (K_k) matrices using:
    D_k = alpha * M_k
    K_k = beta * M_k
    
    :param hrp4: The humanoid robot model.
    :param J_k: Task Jacobian matrix (n_k x 31).
    :param alpha: Scaling factor for damping.
    :param beta: Scaling factor for stiffness.
    :return: D_k (n_k x n_k), K_k (n_k x n_k)
    """
    # Get the full-body inertia matrix (31x31)
    M = hrp4.getMassMatrix()

    # Compute task-space inertia matrix M_k = (J_k * M^{-1} * J_k.T)^{-1}
    M_inv = np.linalg.inv(M)
    M_k = np.linalg.inv(J_k @ M_inv @ J_k.T)  # (n_k x n_k)
    # M_k = np.identity
    # Compute D_k and K_k
    D_k = alpha * M_k  # n_k x n_k
    K_k = beta * M_k   # n_k x n_k

    return D_k, K_k



def create_W(tasks,weight,J,M):
    
    for i,task in enumerate(tasks):
       # print(task)
        if i == 0:
            dim_tasks=J[task].shape[0]
            PSI=weight[task]*np.eye(dim_tasks)
            M_,Tk=compute_task_space_matrices(J[task], M) 
           # print(i)

        else :
            dim_task=J[task].shape[0]
            PSI=block_diag(PSI, weight[task]*np.eye(dim_task))
     
            Mk,Tk=compute_task_space_matrices(J[task], M) 
            M_=block_diag(M_,Mk)
            
  
    #print(np.diag(PSI))
    
    Gamma=np.linalg.inv(M_)
    #print(np.diag(W))
    return Gamma@PSI



# 3 Step


def compute_desired_force(tasks,pos_error,vel_error,ff,gravity_torque,current,J,Jdot,M,pos_gain,vel_gain,robot) :

    "compute the desirede force f_desired as the stack of the fk forces"
    # task = ['lfoot', 'rfoot', 'com', 'torso', 'base', 'joints']  as in the origina file 
    #same for pos_error, vel_error, ff  

    f_des = np.array([]).reshape(0, 1)

    C=current['Coriolis_matrix']['C']
    inv_M=np.linalg.inv(M)


    
    for task in tasks :
        
        
        # Mk= compute_Mk(task)                                  #  I assume the other code have some function like that , when will merge code 
        # Tk= compute_Tk(task)                                # we eventually change the input of that function according to yours 
        
           
            Mk,Tk=compute_task_space_matrices(J[task], M)    #according to Alessandro function  

    
            dim= len(Mk)
            Qk=-Jdot[task] + J[task]@inv_M@ C
            Ck = Mk@ Qk @ Tk.T
    #     (dim x dim)=    (dimx dim) x (dim x 30) x (30 x dim)

    
            Dk,Kk=compute_damping_stiffness(robot, J[task], pos_gain[task], vel_gain[task])    #according to  Elisa function

  
            f=       Tk@gravity_torque +      Mk@Qk@current['joint']['vel'] +          Mk @ ff[task]            + (Ck +Dk) @ vel_error[task] +Kk@ pos_error[task]
#dimension:  dim  =    (dim x 30) * (30 x 1)    + (dimxdim) * (dim x 30) * (30 x 1) +  (dim x dim) x (dim x 1)   +(dim x dim) * (dim x 1)    + (dim x dim) + (dim x 30)
 

            f_des = np.vstack((f_des, f.reshape(-1, 1)))   # dimension = coluomn vector of  (Sum(dim) for each task  = 6 +6 + 3 +3 + 3 +10 = 31


    return f_des # 31 x 1




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
          



def create_U(n_act,J,contact):
    contact_l = contact == 'lfoot'  or contact == 'ds'
    contact_r = contact == 'rfoot' or contact == 'ds'
    S=np.hstack((np.zeros((n_act,6)),np.eye(n_act)))    #S= 24 x 30 
    S=S.T   #30 x 24
    LEE=np.hstack((contact_l*J['lfoot'].T,contact_r*J['rfoot'].T))     #30x12
                                                                    
    U=np.hstack((S,LEE))  # 30 x 36
  
    return U


def TU(n_act,tasks,J,M,contact):
    T=T_matrix(tasks,J,M)   # 31 x 30
    U=create_U(n_act,J,contact)    # 30 x 36
    return T@U  # 31 x 36






def T_matrix(tasks,J,M) :
    T = np.array([]).reshape(0, 30)
    for task in tasks :
        Mk,Tk=compute_task_space_matrices(J[task], M)       # Tk = dim_task x 30
        
        T = np.vstack((T, Tk))
    #print(T.shape)   #31 x 30
    return T

    




class QP_MPTC:
    def __init__(self, n_actuated,dim_contact,task_dim,n_ineq_constraints=0):
        "num contact could be 6 for single support or 12 for double support"
        self.n_vars = n_actuated+dim_contact     ## in the originsl qp of scianca n_vars = 72 { 30 for qddot + 30 for tau + 12 for foot }
                                ## now we have n_vars = 36 {24 tau actuated joint + 12 for foot}    so we have to change according 
        
        self.task_dim = task_dim   ### in our code this is equal to 31 

        self.n_ineq_constraints = n_ineq_constraints  # 16 

        self.opti = ca.Opti('conic')  
  # probrably is better to use different Opti 
        self.x = self.opti.variable(self.n_vars)            # self.x = (36 x 1)


                     

    
        self.Tu = self.opti.parameter(self.task_dim,self.n_vars)     #   ( 31 x 36)
        self.W = self.opti.parameter(self.task_dim, self.task_dim)   # (31 x 31)
        self.f_des=self.opti.parameter(self.task_dim)           # (31 x 1)


        
        
        objective = 0.5 * self.x.T @ self.Tu.T @ self.W @self.Tu @ self.x - self.f_des.T @ self.W @ self.Tu @ self.x
                          ##  1x36   36 x 31     31x31    31x36    36x1      1x31         31x 31    31x 36     36x 1 
                         
        self.opti.minimize(objective)


        self.limit=self.opti.parameter(n_actuated)   # number of actuated jointy
        self.opti.subject_to(self.x[:n_actuated] <= self.limit )                 ###################################################     
        self.opti.subject_to(self.x[:n_actuated] >= -self.limit)
       
        if self.n_ineq_constraints > 0:
             self.A_ineq_ = self.opti.parameter(self.n_ineq_constraints, self.n_vars)  # 16 x  36
             self.b_ineq_ = self.opti.parameter(self.n_ineq_constraints)
             self.opti.subject_to(self.A_ineq_ @ self.x<= self.b_ineq_)
                                   #16x36         36x1        16x1
        self.opti.subject_to(self.x[29] >= 0)    #fz>0
        self.opti.subject_to(self.x[35] >= 0)   #fz >0

        p_opts = {'expand': True}
        s_opts = {'max_iter': 1000, 'verbose': False}
        self.opti.solver('osqp', p_opts, s_opts) 
        # p_opts = {'expand': True}
        # s_opts = {'max_iter': 1000, 'print_level': None}
        # self.opti.solver('ipopt', p_opts, s_opts)

  

    def set_values(self, Tu, W,f_des,limit=None, A_ineq=None, b_ineq=None):
        self.opti.set_value(self.Tu, Tu)
        self.opti.set_value(self.f_des, f_des)
        self.opti.set_value(self.W, W)

        self.opti.set_value(self.limit,limit)
        if self.n_ineq_constraints > 0 and A_ineq is not None and b_ineq is not None:
            self.opti.set_value(self.A_ineq_, A_ineq)
            self.opti.set_value(self.b_ineq_, b_ineq)
     




    def solve(self):
        try:
            solution = self.opti.solve()
            x_sol = solution.value(self.x)
        except RuntimeError as e:
            print("QP Solver failed:", e)
            x_sol = np.zeros(self.n_vars)
        return x_sol
    








