import casadi as ca
from scipy.spatial.transform import Rotation as R
import numpy as np
# solves a constrained QP with casadi
class QPSolver:
    def __init__(self, n_actuated,num_contact,task_dim,n_ineq_constraints=0):
        "num contact could be 6 for single support or 12 for double support"
        self.n_vars = n_actuated+num_contact     ## in the originsl qp of scianca n_vars = 72 { 30 for qddot + 30 for tau + 12 for foot }
                                ## now we have n_vars = 36 {24 tau actuated joint + 12 for foot}    so we have to change according 
        self.task_dim = task_dim   ### in our code this is equal to 31 
        self.n_ineq_constraints = n_ineq_constraints

        self.opti = ca.Opti('conic')  # probrably is better to use different Opti 
        self.x = self.opti.variable(self.n_vars)            


                     

    
        self.Tu = self.opti.parameter(self.task_dim,self.n_vars)
        self.W = self.opti.parameter(self.task_dim, self.task_dim)
        self.f_des=self.opti.parameter(self.task_dim)


        
        
        objective = 0.5 * self.x.T @ self.Tu @ self.W @self.Tu @ self.x - self.f_des.T @ self.W @ self.Tu @ self.x
        self.opti.minimize(objective)


        self.limit=self.opti.parameter(n_actuated)   # number of actuated jointy
        self.opti.subject_to(self.x[:n_actuated] <= self.limit )
        self.opti.subject_to(self.x[:n_actuated] >= -self.limit)
        if self.n_ineq_constraints > 0:
            if num_contact ==  12 :
             m = num_contact // 2
             self.A_ineq_ = self.opti.parameter(self.n_ineq_constraints, m)
             self.b_ineq_ = self.opti.parameter(self.n_ineq_constraints)
             self.opti.subject_to(self.A_ineq_ @ self.x[-m:] <= self.b_ineq_)
             self.opti.subject_to(self.A_ineq_ @ self.x[-num_contact :-m] <= self.b_ineq_)
            else :
                 self.A_ineq_ = self.opti.parameter(self.n_ineq_constraints, m)
                 self.b_ineq_ = self.opti.parameter(self.n_ineq_constraints)
                 self.opti.subject_to(self.A_ineq_ @ self.x[-num_contact:] <= self.b_ineq_)

        else:
            self.A_ineq_ = None
            self.b_ineq_ = None

        # solver options
        p_opts = {'expand': True}
        s_opts = {'max_iter': 1000, 'verbose': False}
        self.opti.solver('osqp', p_opts, s_opts)

    def set_values(self, Tu, W,f_des,limit=None, A_eq=None, b_eq=None, A_ineq=None, b_ineq=None):
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
    




