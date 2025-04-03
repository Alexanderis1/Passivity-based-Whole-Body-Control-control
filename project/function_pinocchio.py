import sys
import numpy as np
import importlib

pinocchio_env_path='/opt/openrobots/lib/python3.12/site-packages'     ## USE YOUR PATH 

sys.path.insert(0, pinocchio_env_path)
pinocchio = importlib.import_module("pinocchio")

print(f"Pinocchio version: {pinocchio.__version__}")   #if this print 3.4.0 then is ok
fcl= importlib.import_module("hppfcl")

print(f"Pinocchio path: {pinocchio.__file__}")
visualize=importlib.import_module("pinocchio.visualize")

MeshcatVisualizer=visualize.MeshcatVisualizer
def euler_to_quaternion1(roll, pitch, yaw):
    """
    convert euler angle in quaternion
    """

    c_roll = np.cos(roll / 2)
    s_roll = np.sin(roll / 2)
    c_pitch = np.cos(pitch / 2)
    s_pitch = np.sin(pitch / 2)
    c_yaw = np.cos(yaw / 2)
    s_yaw = np.sin(yaw / 2)


    x = s_roll * c_pitch * c_yaw - c_roll * s_pitch * s_yaw
    y = c_roll * s_pitch * c_yaw + s_roll * c_pitch * s_yaw
    z = c_roll * c_pitch * s_yaw - s_roll * s_pitch * c_yaw
    w = c_roll * c_pitch * c_yaw + s_roll * s_pitch * s_yaw

    
    return np.array([x, y, z, w])




def convert_q_Dart_into_q_Pinocchio(q_dart,q_pinocchio):
    "this function try to convert the q value in Dart in q value in Pinocchio"
    "takes as input :"
    "q_dart which are the value in Dart  usually are computed with self.hrp4.GetPositions() or in the setting phase "
    "q_pinocchi0 are just q= np.zeros(model1.nq)  vector of zero with the same dimension of the pinocchio variable "
    " Dart has 30 variable ,   pinocchio 31 , cause use the quaternion for the floating base , while dart use XYZ  euler angle"
    "also the position and orientation positions are exchange"
    

    assert len(q_dart)==len(q_pinocchio)-1 , "ERROR"
    q_pinocchio[0:3]=q_dart[3:6]
    quater=euler_to_quaternion1(q_dart[0], q_dart[1], q_dart[2])
    q_pinocchio[3:7]=quater
    q_pinocchio[7:]=q_dart[6:]

    return q_pinocchio




def permutation_matrix(n):
    "if multiply this matrix for a vector then we swtiched position of first 3 element with the element number 4 5 6 "
    "Note :   P=P'=P^-1   (very useful propriety :) "
    if n < 6:
        raise ValueError("must have 6 element or more")
    
    P = np.eye(n) 
    
   
    for i in range(3):
        P[i, i], P[i, i+3] = 0, 1
        P[i+3, i+3], P[i+3, i] = 0, 1
    
    return P

def velocity_pin_dart(v_d,v_p):
  assert len(v_p) == len(v_d), 'ERROR, velocity must be same dimension'
  n=len(v_p)
  P=permutation_matrix(n)
  return P@v_p



def convert_matrix(M):
    "the inertia matrix M and corilis matrix C of pinocchio are related with formula M~= PMP  so this function transform "
    
    n=len(M)
    P=permutation_matrix(n)
    return P@M@P





# import pinocchio 
# from pinocchio.visualize import MeshcatVisualizer
import os
import sys


class Coriolis_with_pionocchio :
    def __init__(self,visual=1):
        "I was not able to find a function whitch allow us to compute the corilis matrix with dart (we could only compute the coriolis term)"
        "So i create this class in order to compute tha C matrix using PINOCCHIO library"
        "FOR USING THIS CLASS YOU NEED TO IMPORT PINOCCHIO  ( IT MIGHT CAUSE SOME ERROR WITH COMPATIBILITY WITH DART AND NUMPY BUT DEPEND BY YOUR COMPUTER "
        "if not work try with importlib)"
        self.visual=visual
        ##  this part is needed in order to create a pinocchio model
        current_dir = os.path.dirname(os.path.abspath(__file__)) #changhe with yours  path

        mesh_dir = (os.path.join(current_dir, "meshes"))
        urdf_model_path = (os.path.join(current_dir, "urdf", "hrp4.urdf"))

        self.model, self.collision_model, self.visual_model = pinocchio.buildModelsFromUrdf(
        urdf_model_path, package_dirs=mesh_dir, root_joint=pinocchio.JointModelFreeFlyer()
        )
        print(self.model)


        if self.visual==1 :    #only if you want to visualize with pinocchio vilualizer (instead of dart)
         try:
          self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
          self.viz.initViewer(open=True)
          #self.viz.loadViewerModel()
         except ImportError as err:
           print(
             "Error while initializing the viewer. "
             "It seems you should install Python meshcat"
             )
           print(err)
           sys.exit(0)
         self.viz.loadViewerModel()
        
        
        default_mass = 1e-8
        default_inertia_matrix = 1e-10 * np.identity(3)  
        for i in range(len(self.model.inertias)):       #same as in dart 
         if self.model.inertias[i].mass == 0.0:  
            self.model.inertias[i] = pinocchio.Inertia(default_mass, np.zeros(3), default_inertia_matrix)
        self.data = self.model.createData()

      #inizialize this class inside the _init_ of hpr4
      #using :
      #self.pino = Coriolis_with_pionocchio() 

    def compute_C(self,robot):
       "call this matrix when you need to compute the Coriolis matrix "
       self.robot=robot
       
       q_dart=self.robot.getPositions()
       vel_dart=self.robot.getVelocities()
       acc_dart=self.robot.getAccelerations()
       q_pin = pinocchio.neutral(self.model)
       q_pin=convert_q_Dart_into_q_Pinocchio(q_dart,q_pin)
       
       if self.visual == 1 :
        self.viz.display(q_pin)
        

       n=self.robot.getNumDofs()
       P=permutation_matrix(n)
       v_pin = pinocchio.utils.zero(self.model.nv)
       v_pin= P@vel_dart          
       a_pin=P@acc_dart
       pinocchio.forwardKinematics(self.model, self.data, q_pin, v_pin,a_pin)
       pinocchio.centerOfMass(self.model, self.data, q_pin, v_pin, a_pin)
       pinocchio.crba(self.model, self.data, q_pin)  
      # M_pin=self.data.M
      # M_dart = self.hrp4.getMassMatrix()
      # assert np.amax(P@M_pin@P- M_dart ) <= 0.0000001
       pinocchio.computeCoriolisMatrix(self.model, self.data, q_pin, v_pin)  

       C_pin=self.data.C  #coriolis Matrix
       C_dart=P@C_pin@P
      
      
       assert np.amax( C_dart@vel_dart - self.robot.getCoriolisForces()) <= 1e-8 ,  "the coriolis term is different, there must be some error in computation"   # so we are sure that P@C_pin@P = C_dart 
      #######################################

       
       return C_dart
       



        

