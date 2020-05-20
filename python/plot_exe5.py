import matplotlib.pyplot as plt	
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pylab

params = {'legend.fontsize': 20,
          'legend.linewidth': 2}
plt.rcParams.update(params)













#############################X plot##############################
source = open("saved_data/trajectory_x.txt", "r")
X_x = source.readlines()
source.close()

source = open("saved_data/des_trajectory_x.txt", "r")
des_X_x = source.readlines()
source.close()

plt.figure(3)
plt.subplot(2, 2, 1)
	
ploted_X_x, = plt.plot(0.001*np.arange(0,np.size(X_x),1), X_x, "b", linewidth=2.0)
ploted_Des_X_x, = plt.plot(0.001*np.arange(0,np.size(des_X_x),1), des_X_x, "r--", linewidth=2.0)
plt.legend([ploted_X_x, ploted_Des_X_x], ['$X_x$', '$X^*_x$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_x), -0.6, 0.85])
plt.grid(True)




source = open("saved_data/trajectory_y.txt", "r")
X_y = source.readlines()
source.close()

source = open("saved_data/des_trajectory_y.txt", "r")
des_X_y = source.readlines()
source.close()

plt.figure(3)
plt.subplot(2, 2, 2)
	
ploted_X_y, = plt.plot(0.001*np.arange(0,np.size(X_y),1), X_y, "b", linewidth=2.0)
ploted_Des_X_y, = plt.plot(0.001*np.arange(0,np.size(des_X_y),1), des_X_y, "r--", linewidth=2.0)
plt.legend([ploted_X_y, ploted_Des_X_y], ['$X_y$', '$X^*_y$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_y), 0.15, 0.75])
plt.grid(True)




source = open("saved_data/trajectory_z.txt", "r")
X_z = source.readlines()
source.close()

source = open("saved_data/des_trajectory_z.txt", "r")
des_X_z = source.readlines()
source.close()

plt.figure(3)
plt.subplot(2, 2, 3)

print len(np.arange(0,np.size(X_z),1))
print np.size(X_z)
	
ploted_X_z, = plt.plot(0.001*np.arange(0,np.size(X_z),1), X_z,  "b", linewidth=2.0)
ploted_Des_X_z, = plt.plot(0.001*np.arange(0,np.size(des_X_z),1), des_X_z, "r--", linewidth=2.0)
plt.legend([ploted_X_z, ploted_Des_X_z], ['$X_z$', '$X^*_z$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_z), 0.15, 0.85])
plt.grid(True)





source = open("saved_data/X_err.txt", "r")
X_err = source.readlines()
source.close()


plt.figure(3)
plt.subplot(2, 2, 4)
	
ploted_X_err, = plt.plot(0.001*np.arange(0,np.size(X_err),1), X_err, "b", linewidth=2.0)

plt.legend([ploted_X_err], ['$X_{err}$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_err), 0.0, 0.005])
plt.grid(True)
#############################X plot##############################
















#############################Velocities plot##############################
source = open("saved_data/V_7_x.txt", "r")
V_7_x = source.readlines()
source.close()

source = open("saved_data/V_7_des_x.txt", "r")
V_7_des_x = source.readlines()
source.close()

plt.figure(4)
plt.subplot(2, 2, 2)
	
ploted_V_7_x, = plt.plot(0.001*np.arange(0,np.size(V_7_x),1), V_7_x, "b", linewidth=1.0)
ploted_V_7_des_x, = plt.plot(0.001*np.arange(0,np.size(V_7_des_x),1), V_7_des_x, "r--", linewidth=1.0)
plt.legend([ploted_V_7_x, ploted_V_7_des_x], ['$\dot{X_x}$', '$\dot{X^*}_x$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_x), -1.0, 2])
plt.grid(True)





source = open("saved_data/V_7_y.txt", "r")
V_7_y = source.readlines()
source.close()

source = open("saved_data/V_7_des_y.txt", "r")
V_7_des_y = source.readlines()
source.close()

plt.figure(4)
plt.subplot(2, 2, 3)
	
ploted_V_7_y, = plt.plot(0.001*np.arange(0,np.size(V_7_y),1), V_7_y, "b", linewidth=1.0)
ploted_V_7_des_y, = plt.plot(0.001*np.arange(0,np.size(V_7_des_y),1), V_7_des_y, "r--", linewidth=1.0)
plt.legend([ploted_V_7_y, ploted_V_7_des_y], ['$\dot{X_y}$', '$\dot{X^*}_y$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_y), -1.0, 2])
plt.grid(True)





source = open("saved_data/V_7_z.txt", "r")
V_7_z = source.readlines()
source.close()

source = open("saved_data/V_7_des_z.txt", "r")
V_7_des_z = source.readlines()
source.close()

plt.figure(4)
plt.subplot(2, 2, 4)
	
ploted_V_7_z, = plt.plot(0.001*np.arange(0,np.size(V_7_z),1), V_7_z, "b", linewidth=1.0)
ploted_V_7_des_z, = plt.plot(0.001*np.arange(0,np.size(V_7_des_z),1), V_7_des_z, "r--", linewidth=1.0)
plt.legend([ploted_V_7_z, ploted_V_7_des_z], ['$\dot{X_z}$', '$\dot{X^*}_z$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_z), -1.0, 2])
plt.grid(True)
#############################Velocities plot##############################









#############################q plot##############################
source = open("saved_data/q_0.txt", "r")
q_0 = source.readlines()
source.close()

source = open("saved_data/q_0_max.txt", "r")
q_0_max = source.readlines()
source.close()

source = open("saved_data/q_0_min.txt", "r")
q_0_min = source.readlines()
source.close()

plt.figure(5)
	
ploted_q_0, = plt.plot(0.001*np.arange(0,np.size(q_0),1), q_0, linewidth=1.0)
ploted_q_0_max, = plt.plot(0.001*np.arange(0,np.size(q_0_max),1), q_0_max, "k--", linewidth=1.0)
ploted_q_0_min, = plt.plot(0.001*np.arange(0,np.size(q_0_min),1), q_0_min, "k-.", linewidth=1.0)
plt.legend([ploted_q_0, ploted_q_0_max, ploted_q_0_min], ['$q_0$', '$q_{max}$', '$q_{min}$'])
	



source = open("saved_data/q_1.txt", "r")
q_1 = source.readlines()
source.close()
plt.figure(5)
ploted_q_1, = plt.plot(0.001*np.arange(0,np.size(q_1),1), q_1, linewidth=1.0)
plt.legend([ploted_q_1], ['$q_1$'])
	







source = open("saved_data/q_2.txt", "r")
q_2 = source.readlines()
source.close()
plt.figure(5)
ploted_q_2, = plt.plot(0.001*np.arange(0,np.size(q_2),1), q_2, linewidth=1.0)
plt.legend([ploted_q_2], ['$q_2$'])
	





source = open("saved_data/q_3.txt", "r")
q_3 = source.readlines()
source.close()
plt.figure(5)
ploted_q_3, = plt.plot(0.001*np.arange(0,np.size(q_3),1), q_3, linewidth=1.0)
plt.legend([ploted_q_3], ['$q_3$'])





source = open("saved_data/q_4.txt", "r")
q_4 = source.readlines()
source.close()
plt.figure(5)
ploted_q_4, = plt.plot(0.001*np.arange(0,np.size(q_4),1), q_4, linewidth=1.0)
plt.legend([ploted_q_4], ['$q_4$'])
	






source = open("saved_data/q_5.txt", "r")
q_5 = source.readlines()
source.close()
plt.figure(5)	
ploted_q_5, = plt.plot(0.001*np.arange(0,np.size(q_5),1), q_5, linewidth=1.0)
plt.legend([ploted_q_5], ['$q_5$'])






source = open("saved_data/q_6.txt", "r")
q_6 = source.readlines()
source.close()
plt.figure(5)
ploted_q_6, = plt.plot(0.001*np.arange(0,np.size(q_6),1), q_6, linewidth=1.0)
plt.legend([ploted_q_6, ploted_q_5, ploted_q_4, ploted_q_3, ploted_q_2, ploted_q_1, ploted_q_0, ploted_q_0_max, ploted_q_0_min], ['$q_6$', '$q_5$', '$q_4$', '$q_3$', '$q_2$', '$q_1$', '$q_0$', '$q_{max}$', '$q_{min}$' ])


plt.ylabel("$(rad)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_0), -5, 3.5])
plt.grid(True)
#############################q plot##############################


















#############################q_dot plot##############################
source = open("saved_data/q_dot_0.txt", "r")
q_dot_0 = source.readlines()
source.close()

source = open("saved_data/q_dot_0_max.txt", "r")
q_dot_0_max = source.readlines()
source.close()

source = open("saved_data/q_dot_0_min.txt", "r")
q_dot_0_min = source.readlines()
source.close()

plt.figure(6)
	
ploted_q_dot_0, = plt.plot(0.001*np.arange(0,np.size(q_dot_0),1), q_dot_0, linewidth=1.0)
ploted_q_dot_0_max, = plt.plot(0.001*np.arange(0,np.size(q_dot_0_max),1), q_dot_0_max, "k--", linewidth=1.0)
ploted_q_dot_0_min, = plt.plot(0.001*np.arange(0,np.size(q_dot_0_min),1), q_dot_0_min, "k-.", linewidth=1.0)

	



source = open("saved_data/q_dot_1.txt", "r")
q_dot_1 = source.readlines()
source.close()
plt.figure(6)
ploted_q_dot_1, = plt.plot(0.001*np.arange(0,np.size(q_dot_1),1), q_dot_1, linewidth=1.0)

	







source = open("saved_data/q_dot_2.txt", "r")
q_dot_2 = source.readlines()
source.close()
plt.figure(6)
ploted_q_dot_2, = plt.plot(0.001*np.arange(0,np.size(q_dot_2),1), q_dot_2, linewidth=1.0)

	





source = open("saved_data/q_dot_3.txt", "r")
q_dot_3 = source.readlines()
source.close()
plt.figure(6)
ploted_q_dot_3, = plt.plot(0.001*np.arange(0,np.size(q_dot_3),1), q_dot_3, linewidth=1.0)






source = open("saved_data/q_dot_4.txt", "r")
q_dot_4 = source.readlines()
source.close()
plt.figure(6)
ploted_q_dot_4, = plt.plot(0.001*np.arange(0,np.size(q_dot_4),1), q_dot_4, linewidth=1.0)

	






source = open("saved_data/q_dot_5.txt", "r")
q_dot_5 = source.readlines()
source.close()
plt.figure(6)	
ploted_q_dot_5, = plt.plot(0.001*np.arange(0,np.size(q_dot_5),1), q_dot_5, linewidth=1.0)







source = open("saved_data/q_dot_6.txt", "r")
q_dot_6 = source.readlines()
source.close()
plt.figure(6)
ploted_q_dot_6, = plt.plot(0.001*np.arange(0,np.size(q_dot_6),1), q_dot_6, linewidth=1.0)
plt.legend([ploted_q_dot_6, ploted_q_dot_5, ploted_q_dot_4, ploted_q_dot_3, ploted_q_dot_2, ploted_q_dot_1, ploted_q_dot_0, ploted_q_dot_0_max, ploted_q_dot_0_min], ['$\dot{q}_6$', '$\dot{q}_5$', '$\dot{q}_4$', '$\dot{q}_3$', '$\dot{q}_2$', '$\dot{q}_1$', '$\dot{q}_0$', '$\dot{q}_{max}$', '$\dot{q}_{min}$' ])


plt.ylabel("$(rad/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dot_0), -4.0, 2.5])
plt.grid(True)
#############################q_dot plot##############################












#############################tau plot##############################
source = open("saved_data/tau_0.txt", "r")
tau_0 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_0, = plt.plot(0.001*np.arange(0,np.size(tau_0),1), tau_0, linewidth=2.0)

	



source = open("saved_data/tau_1.txt", "r")
tau_1 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_1, = plt.plot(0.001*np.arange(0,np.size(tau_1),1), tau_1, linewidth=2.0)

	



source = open("saved_data/tau_2.txt", "r")
tau_2 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_2, = plt.plot(0.001*np.arange(0,np.size(tau_2),1), tau_2, linewidth=2.0)

	





source = open("saved_data/tau_3.txt", "r")
tau_3 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_3, = plt.plot(0.001*np.arange(0,np.size(tau_3),1), tau_3, linewidth=2.0)






source = open("saved_data/tau_4.txt", "r")
tau_4 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_4, = plt.plot(0.001*np.arange(0,np.size(tau_4),1), tau_4, linewidth=2.0)

	






source = open("saved_data/tau_5.txt", "r")
tau_5 = source.readlines()
source.close()
plt.figure(7)	
ploted_tau_5, = plt.plot(0.001*np.arange(0,np.size(tau_5),1), tau_5, linewidth=2.0)







source = open("saved_data/tau_6.txt", "r")
tau_6 = source.readlines()
source.close()
plt.figure(7)
ploted_tau_6, = plt.plot(0.001*np.arange(0,np.size(tau_6),1), tau_6, linewidth=2.0)
plt.legend([ploted_tau_6, ploted_tau_5, ploted_tau_4, ploted_tau_3, ploted_tau_2, ploted_tau_1, ploted_tau_0], [r'$\tau_6$', r'$\tau_5$', r'$\tau_4$', r'$\tau_3$', r'$\tau_2$', r'$\tau_1$', r'$\tau_0$' ])


plt.ylabel("$(N.m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(tau_0), -200, 200])
plt.grid(True)
#############################tau plot##############################









############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################
source = open("saved_data/Real_Ec_7_x.txt", "r")
Real_Ec_7_x = source.readlines()
source.close()

source = open("saved_data/Ec_max_7_x.txt", "r")
Ec_max_7_x = source.readlines()
source.close()

source = open("saved_data/Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x.txt", "r")
Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x = source.readlines()
source.close()

source = open("saved_data/Acc_7_x.txt", "r")
Acc_7_x = source.readlines()
source.close()

#source = open("saved_data/Acc_7_x_nxt_step_limit.txt", "r")
#Acc_7_x_nxt_step_limit = source.readlines()
#source.close()

plt.figure(1)

ploted_Real_Ec_7_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x),1), Real_Ec_7_x, "b", linewidth=2.0)
ploted_Ec_max_7_x, = plt.plot(0.001*np.arange(0,np.size(Ec_max_7_x),1), Ec_max_7_x, "r--", linewidth=2.0)
ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x),1), Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, "b--", linewidth=2.0)
ploted_Acc_7_x, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x),1), Acc_7_x, "k", linewidth=2.0)
#ploted_Acc_7_x_nxt_step_limit, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x_nxt_step_limit),1), Acc_7_x_nxt_step_limit, "c", linewidth=2.0)
#plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x, ploted_Acc_7_x_nxt_step_limit], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$', '$\ddot{X}_{x|k+1}^{lim} (m/s^2)$'])
#plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$'])
plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x], ['$Real E_{c} (J)$', '$E_{cmax} (J)$'])	

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ec_7_x), -2, 2])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################










############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################
source = open("saved_data/Ec_7_x_nxt_step.txt", "r")
Ec_7_x_nxt_step = source.readlines()
source.close()

source = open("saved_data/Ec_max_7_x.txt", "r")
Ec_max_7_x = source.readlines()
source.close()

source = open("saved_data/Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step.txt", "r")
Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step = source.readlines()
source.close()

source = open("saved_data/Acc_7_x_nxt_step.txt", "r")
Acc_7_x_nxt_step = source.readlines()
source.close()

source = open("saved_data/Acc_7_x_nxt_step_limit.txt", "r")
Acc_7_x_nxt_step_limit = source.readlines()
source.close()


plt.figure(2)

ploted_Ec_7_x_nxt_step, = plt.plot(0.001*np.arange(0,np.size(Ec_7_x_nxt_step),1), Ec_7_x_nxt_step, "b", linewidth=2.0)
ploted_Ec_max_7_x, = plt.plot(0.001*np.arange(0,np.size(Ec_max_7_x),1), Ec_max_7_x, "r--", linewidth=2.0)
ploted_Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step, = plt.plot(0.001*np.arange(0,np.size(Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step),1), Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step, "b--", linewidth=2.0)
ploted_Acc_7_x_nxt_step, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x_nxt_step),1), Acc_7_x_nxt_step, "k", linewidth=2.0)
ploted_Acc_7_x_nxt_step_limit, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x_nxt_step_limit),1), Acc_7_x_nxt_step_limit, "c", linewidth=2.0)
plt.legend([ploted_Ec_7_x_nxt_step, ploted_Ec_max_7_x, ploted_Ec_7_x_rcnsrcted_wth_small_Ep_7_x_nxt_step, ploted_Acc_7_x_nxt_step, ploted_Acc_7_x_nxt_step_limit], ['$E_{c|k+1} (J)$', '$E_{cmax} (J)$', '$E_{c|sum Ep|k+1} (J)$', '$\ddot{X}_{x|k+1} (m/s^2)$', '$\ddot{X}_{x|k+1}^{lim} (m/s^2)$'])

	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Ec_7_x_nxt_step), -0.35, 0.3])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################















############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################
source = open("saved_data/Force_sensor_x.txt", "r")
Force_sensor_x = source.readlines()
source.close()



source = open("saved_data/Force_sensor_y.txt", "r")
Force_sensor_y = source.readlines()
source.close()


source = open("saved_data/Force_sensor_z.txt", "r")
Force_sensor_z = source.readlines()
source.close()

plt.figure(8)


ploted_Force_sensor_x, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_x),1), Force_sensor_x, "b", linewidth=2.0)
ploted_Force_sensor_y, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_y),1), Force_sensor_y, "r", linewidth=2.0)
ploted_Force_sensor_z, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_z),1), Force_sensor_z, "g", linewidth=2.0)
plt.legend([ploted_Force_sensor_x, ploted_Force_sensor_y, ploted_Force_sensor_z], ['$F_x (N)$', '$F_y (N)$', '$F_z (N)$'])

	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Force_sensor_x), 0, 600])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################








############################plot force_x_nxt_step,  force_x_nxt_step_limit_max, force_x_nxt_step_limit_min and Force_sensor_x##############################
source = open("saved_data/force_x_nxt_step.txt", "r")
force_x_nxt_step = source.readlines()
source.close()


source = open("saved_data/force_x_nxt_step_limit_max.txt", "r")
force_x_nxt_step_limit_max = source.readlines()
source.close()


source = open("saved_data/force_x_nxt_step_limit_min.txt", "r")
force_x_nxt_step_limit_min = source.readlines()
source.close()

source = open("saved_data/Force_sensor_x.txt", "r")
Force_sensor_x = source.readlines()
source.close()

plt.figure(9)

ploted_force_x_nxt_step, = plt.plot(0.001*np.arange(0,np.size(force_x_nxt_step),1), force_x_nxt_step, "b", linewidth=2.0)
ploted_force_x_nxt_step_limit_max, = plt.plot(0.001*np.arange(0,np.size(force_x_nxt_step_limit_max),1), force_x_nxt_step_limit_max, "r", linewidth=2.0)
ploted_force_x_nxt_step_limit_min, = plt.plot(0.001*np.arange(0,np.size(force_x_nxt_step_limit_min),1), force_x_nxt_step_limit_min, "r--", linewidth=2.0)
ploted_Force_sensor_x, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_x),1), Force_sensor_x, "g", linewidth=2.0)
plt.legend([ploted_force_x_nxt_step, ploted_force_x_nxt_step_limit_max, ploted_force_x_nxt_step_limit_min, ploted_Force_sensor_x], ['$F_{x|k+1} (N)$', '$F_{x|k+1}^{max} (N)$', '$F_{x|k+1}^{min} (N)$', '$F_{x|sensor} (N)$'])

	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(force_x_nxt_step), 0, 600])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot force_x_nxt_step,  force_x_nxt_step_limit_max, force_x_nxt_step_limit_min and Force_sensor_x##############################







############################plot Ep_x_nxt_step,  Ep_x_nxt_step_limit_max, Ep_x_nxt_step_limit_min and Real_Ep_x##############################
source = open("saved_data/Ep_x_nxt_step.txt", "r")
Ep_x_nxt_step = source.readlines()
source.close()


source = open("saved_data/Ep_x_nxt_step_limit_max.txt", "r")
Ep_x_nxt_step_limit_max = source.readlines()
source.close()


source = open("saved_data/Ep_x_nxt_step_limit_min.txt", "r")
Ep_x_nxt_step_limit_min = source.readlines()
source.close()

source = open("saved_data/Real_Ep_x.txt", "r")
Real_Ep_x = source.readlines()
source.close()

plt.figure(10)

ploted_Ep_x_nxt_step,           = plt.plot(0.001*np.arange(0,np.size(Ep_x_nxt_step),1), Ep_x_nxt_step, "b", linewidth=2.0)
ploted_Ep_x_nxt_step_limit_max, = plt.plot(0.001*np.arange(0,np.size(Ep_x_nxt_step_limit_max),1), Ep_x_nxt_step_limit_max, "r", linewidth=2.0)
ploted_Ep_x_nxt_step_limit_min, = plt.plot(0.001*np.arange(0,np.size(Ep_x_nxt_step_limit_min),1), Ep_x_nxt_step_limit_min, "r--", linewidth=2.0)
ploted_Real_Ep_x,               = plt.plot(0.001*np.arange(0,np.size(Real_Ep_x),1), Real_Ep_x, "g", linewidth=2.0)
plt.legend([ploted_Ep_x_nxt_step, ploted_Ep_x_nxt_step_limit_max, ploted_Ep_x_nxt_step_limit_min, ploted_Real_Ep_x], ['$Ep_{x|k+1} (J)$', '$Ep_{x|k+1}^{max} (J)$', '$Ep_{x|k+1}^{min} (J)$', '$Ep_{x|sensor} (J)$'])

	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Ep_x_nxt_step), 0, 0.01])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot force_x_nxt_step,  force_x_nxt_step_limit_max, force_x_nxt_step_limit_min and Force_sensor_x##############################













############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################
source = open("saved_data/Real_Ec_7_x.txt", "r")
Real_Ec_7_x = source.readlines()
source.close()

source = open("saved_data/Ec_max_7_x.txt", "r")
Ec_max_7_x = source.readlines()
source.close()



#source = open("saved_data/Acc_7_x_nxt_step_limit.txt", "r")
#Acc_7_x_nxt_step_limit = source.readlines()
#source.close()

plt.figure(11)

ploted_Real_Ec_7_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x),1), Real_Ec_7_x, "b", linewidth=2.0)
ploted_Ec_max_7_x, = plt.plot(0.001*np.arange(0,np.size(Ec_max_7_x),1), Ec_max_7_x, "r--", linewidth=2.0)

#ploted_Acc_7_x_nxt_step_limit, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x_nxt_step_limit),1), Acc_7_x_nxt_step_limit, "c", linewidth=2.0)
#plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x, ploted_Acc_7_x_nxt_step_limit], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$', '$\ddot{X}_{x|k+1}^{lim} (m/s^2)$'])
#plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$'])
plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x], ['$Real E_{c} (J)$', '$E_{cmax} (J)$'])	

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ec_7_x), -0.05, 0.25])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################







plt.show()
