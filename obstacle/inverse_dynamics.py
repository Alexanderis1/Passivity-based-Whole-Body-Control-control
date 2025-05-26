import dartpy as dart
import numpy as np
from utils import *
from functions import *

class InverseDynamics:
    def __init__(self, robot, redundant_dofs,foot_size=0.1, µ=0.5):
        self.robot = robot
        self.dofs = self.robot.getNumDofs()
        self.d = foot_size / 2.
        self.µ = µ

        # define sizes for QP solver
        self.num_contacts = 2
        self.num_contact_dims = self.num_contacts * 6
        self.n_vars = 2 * self.dofs + self.num_contact_dims

        self.n_eq_constraints = self.dofs
        self.n_actuated=self.dofs - 6                 ###############################

        self.n_ineq_constraints = 8 * self.num_contacts

        # initialize QP solver
        #self.qp_solver = QPSolver(self.n_vars, self.n_eq_constraints, self.n_ineq_constraints)
        
        self.qp_solver2= QP_MPTC(self.n_actuated,self.num_contact_dims,31,self.n_ineq_constraints)    #################



        # selection matrix for redundant dofs
        self.joint_selection = np.zeros((self.dofs, self.dofs))
        self.redundant=np.zeros((len(redundant_dofs),self.dofs))   ################################
        j=0
        for i in range(self.dofs):
            joint_name = self.robot.getDof(i).getName()
            if joint_name in redundant_dofs:
                self.joint_selection[i, i] = 1
                self.redundant[j,i]=1          ###############################       we need self.redundant cause we want J['joint'] to be full rank
                j+=1                           ############################
       # print(self.redundant)               

    def get_joint_torques(self, desired, current, contact):
        contact_l = contact == 'lfoot'  or contact == 'ds'
        contact_r = contact == 'rfoot' or contact == 'ds'

        # robot parameters
        lsole = self.robot.getBodyNode('l_sole')
        rsole = self.robot.getBodyNode('r_sole')
        torso = self.robot.getBodyNode('torso')
        base  = self.robot.getBodyNode('body')

        # weights and gainszmp_z_mid_param
        tasks = ['lfoot', 'rfoot', 'com', 'torso', 'base', 'joints']
        weights   = {'lfoot':  2., 'rfoot':  2., 'com':  1., 'torso': 1., 'base': 1., 'joints': 1.}
        pos_gains = {'lfoot': 15., 'rfoot': 15., 'com':  1., 'torso': 10., 'base': 10., 'joints': 1. }
        vel_gains = {'lfoot': 10., 'rfoot': 10., 'com': 10., 'torso': 5., 'base': 5., 'joints': 0.1}
        # jacobians
        J = {'lfoot' : self.robot.getJacobian(lsole,        inCoordinatesOf=dart.dynamics.Frame.World()),
             'rfoot' : self.robot.getJacobian(rsole,        inCoordinatesOf=dart.dynamics.Frame.World()),
             'com'   : self.robot.getCOMLinearJacobian(     inCoordinatesOf=dart.dynamics.Frame.World()),
             'torso' : self.robot.getAngularJacobian(torso, inCoordinatesOf=dart.dynamics.Frame.World()),
             'base'  : self.robot.getAngularJacobian(base,  inCoordinatesOf=dart.dynamics.Frame.World()),
             'joints': self.redundant}

        # jacobians derivatives
        Jdot = {'lfoot' : self.robot.getJacobianClassicDeriv(lsole, inCoordinatesOf=dart.dynamics.Frame.World()),   
                'rfoot' : self.robot.getJacobianClassicDeriv(rsole, inCoordinatesOf=dart.dynamics.Frame.World()),
                'com'   : self.robot.getCOMLinearJacobianDeriv(     inCoordinatesOf=dart.dynamics.Frame.World()),
                'torso' : self.robot.getAngularJacobianDeriv(torso, inCoordinatesOf=dart.dynamics.Frame.World()),
                'base'  : self.robot.getAngularJacobianDeriv(base,  inCoordinatesOf=dart.dynamics.Frame.World()),
                'joints': np.zeros((len(self.redundant),self.dofs))}   #########

        # feedforward terms
        ff = {'lfoot' : desired['lfoot']['acc'],
              'rfoot' : desired['rfoot']['acc'],
              'com'   : desired['com']['acc'],
              'torso' : desired['torso']['acc'],
              'base'  : desired['base']['acc'],
              'joints': self.redundant@desired['joint']['acc']}   ##########

        # error vectors
        pos_error = {'lfoot' : pose_difference(desired['lfoot']['pos'] , current['lfoot']['pos'] ),
                     'rfoot' : pose_difference(desired['rfoot']['pos'], current['rfoot']['pos']),
                     'com'   : desired['com']['pos'] - current['com']['pos'],
                     'torso' : rotation_vector_difference(desired['torso']['pos'], current['torso']['pos']),
                     'base'  : rotation_vector_difference(desired['base']['pos'] , current['base']['pos'] ),
                     'joints': self.redundant@(desired['joint']['pos'] - current['joint']['pos'])}    ##############

        # velocity error vectors
        vel_error = {'lfoot' : desired['lfoot']['vel'] - current['lfoot']['vel'],
                     'rfoot' : desired['rfoot']['vel'] - current['rfoot']['vel'],
                     'com'   : desired['com']['vel']   - current['com']['vel'],
                     'torso' : desired['torso']['vel'] - current['torso']['vel'],
                     'base'  : desired['base']['vel']  - current['base']['vel'],
                     'joints': self.redundant@(desired['joint']['vel'] - current['joint']['vel'])}  ##########


        A_ineq2=np.zeros((self.n_ineq_constraints,36))
        b_ineq = np.zeros(self.n_ineq_constraints)
        A = np.array([[ 1, 0, 0, 0, 0, -self.d],
                      [-1, 0, 0, 0, 0, -self.d],
                      [0,  1, 0, 0, 0, -self.d],
                      [0, -1, 0, 0, 0, -self.d],
                      [0, 0, 0,  1, 0, -self.µ],
                      [0, 0, 0, -1, 0, -self.µ],
                      [0, 0, 0, 0,  1, -self.µ],
                      [0, 0, 0, 0, -1, -self.µ]])
       # A_ineq[0:self.n_ineq_constraints, f_c_indices] = block_diag(A, A)

        A_ineq2[0:self.n_ineq_constraints,24:36]=block_diag(A,A)                   ##############
    
    

        #######################################################
        C_term=self.robot.getCoriolisForces()
        M=self.robot.getMassMatrix()   ## or getAugMassMatrix ()
        gravity_torque= self.robot.getGravityForces()
        f_des=compute_desired_force(tasks,pos_error,vel_error,ff,gravity_torque,current,J,Jdot,M,pos_gains,vel_gains,self.robot)  ##############
       # print(f_des.shape)
        Tu=TU(self.n_actuated,tasks,J,M,contact)   ###################
       # print(Tu.shape,'TU')
        W=create_W(tasks,weights,J,M)    ############
        


        #print(W)
        limit=joint_torque_limit(self.robot)      ###################

        # solve the QP, compute torques and return them
      #  self.qp_solver.set_values(H, F, A_eq, b_eq, A_ineq, b_ineq)
        self.qp_solver2.set_values(Tu,W,f_des,limit, A_ineq2, b_ineq)
       
        solution = self.qp_solver2.solve()
       #
        #tau = solution[tau_indices]
        
       


        

        solution=self.qp_solver2.solve()
        tau=solution[:24]
       
        return tau