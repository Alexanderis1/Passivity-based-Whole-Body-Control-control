# 1 Step




# 2 Step




# 3 Step

import numpy as np
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
    
