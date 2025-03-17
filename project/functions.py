# 1 Step




# 2 Step




# 3 Step


def compute_desired_force(tasks,pos_error,vel_error,ff,gravity_torque,current) :

    "compute the desirede force f_desired as the stack of the fk forces"
    # task = ['lfoot', 'rfoot', 'com', 'torso', 'base', 'joints']  as in the origina file 
    #same for pos_error, vel_error, ff  


    #

    num_task = len(tasks)
    f_des=np.zeros(num_task)
    for i,task in enumerate(tasks) :
        Mk= compute_Mk(task)                                  #  I assume the other code have some function like that , when will merge code 
        Tk= compute_Tk(task)                                  # we eventually change the input of that function according to yours 
        Dk= compute_Dk(task)
        Ck= compute_Ck(task)
        Qk= compute_Qk(task)
        Kk= compute_Kk(task)
        f_des[i] = Tk@gravity_torque + Mk@Qk@current['joint']['vel'] + Mk @ ff[task] + (Ck +Dk) @ vel_error[task] +Kk@ pos_error[task]
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
          