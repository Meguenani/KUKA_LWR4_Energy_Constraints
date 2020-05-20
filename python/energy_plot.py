import matplotlib.pyplot as plt	
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pylab

params = {'legend.fontsize': 20,
          'legend.linewidth': 2}
plt.rcParams.update(params)




#############################tau plot##############################
source = open("saved_data/tau_0.txt", "r")
tau_0 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_0, = plt.plot(0.001*np.arange(0,np.size(tau_0),1), tau_0, "b", linewidth=2.0)

	



source = open("saved_data/tau_1.txt", "r")
tau_1 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_1, = plt.plot(0.001*np.arange(0,np.size(tau_1),1), tau_1, "g", linewidth=2.0)

	



source = open("saved_data/tau_2.txt", "r")
tau_2 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_2, = plt.plot(0.001*np.arange(0,np.size(tau_2),1), tau_2, "r", linewidth=2.0)

	





source = open("saved_data/tau_3.txt", "r")
tau_3 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_3, = plt.plot(0.001*np.arange(0,np.size(tau_3),1), tau_3, "c", linewidth=2.0)






source = open("saved_data/tau_4.txt", "r")
tau_4 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_4, = plt.plot(0.001*np.arange(0,np.size(tau_4),1), tau_4, "m", linewidth=2.0)

	






source = open("saved_data/tau_5.txt", "r")
tau_5 = source.readlines()
source.close()
plt.figure('tau')	
ploted_tau_5, = plt.plot(0.001*np.arange(0,np.size(tau_5),1), tau_5, "y", linewidth=2.0)







source = open("saved_data/tau_6.txt", "r")
tau_6 = source.readlines()
source.close()
plt.figure('tau')
ploted_tau_6, = plt.plot(0.001*np.arange(0,np.size(tau_6),1), tau_6, "k", linewidth=2.0)
plt.legend([ploted_tau_6, ploted_tau_5, ploted_tau_4, ploted_tau_3, ploted_tau_2, ploted_tau_1, ploted_tau_0], [r'$\tau_6$', r'$\tau_5$', r'$\tau_4$', r'$\tau_3$', r'$\tau_2$', r'$\tau_1$', r'$\tau_0$' ])


plt.ylabel("$(N.m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(tau_0), -50, 10])
plt.grid(True)
#############################tau plot##############################






##############################tau sensor plot##############################
#source = open("saved_data/tau_sensor_0.txt", "r")
#tau_sensor_0 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_0, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_0),1), tau_sensor_0, "b--", linewidth=2.0)

#	



#source = open("saved_data/tau_sensor_1.txt", "r")
#tau_sensor_1 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_1, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_1),1), tau_sensor_1, "g--", linewidth=2.0)

#	



#source = open("saved_data/tau_sensor_2.txt", "r")
#tau_sensor_2 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_2, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_2),1), tau_sensor_2, "r--", linewidth=2.0)

#	





#source = open("saved_data/tau_sensor_3.txt", "r")
#tau_sensor_3 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_3, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_3),1), tau_sensor_3, "c--", linewidth=2.0)






#source = open("saved_data/tau_sensor_4.txt", "r")
#tau_sensor_4 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_4, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_4),1), tau_sensor_4, "m--", linewidth=2.0)

#	






#source = open("saved_data/tau_sensor_5.txt", "r")
#tau_sensor_5 = source.readlines()
#source.close()
#plt.figure('tau_sensor')	
#ploted_tau_sensor_5, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_5),1), tau_sensor_5, "y--", linewidth=2.0)







#source = open("saved_data/tau_sensor_6.txt", "r")
#tau_sensor_6 = source.readlines()
#source.close()
#plt.figure('tau_sensor')
#ploted_tau_sensor_6, = plt.plot(0.001*np.arange(0,np.size(tau_sensor_6),1), tau_sensor_6, "k--", linewidth=2.0)
#plt.legend([ploted_tau_sensor_6, ploted_tau_sensor_5, ploted_tau_sensor_4, ploted_tau_sensor_3, ploted_tau_sensor_2, ploted_tau_sensor_1, ploted_tau_sensor_0], [r'$\tau_{sensor_{6}}$', r'$\tau_{sensor_{5}}$', r'$\tau_{sensor_{4}}$', r'$\tau_{sensor_{3}}$', r'$\tau_{sensor_{2}}$', r'$\tau_{sensor_{1}}$', r'$\tau_{sensor_{0}}$' ])


#plt.ylabel("$(N.m)$", fontsize=20)
#plt.xlabel("$time (s)$", fontsize=20)
#plt.axis([0,0.001*np.size(tau_sensor_0), -50, 10])
#plt.grid(True)
##############################tau sensor plot##############################







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

plt.figure('q')
	
ploted_q_0, = plt.plot(0.001*np.arange(0,np.size(q_0),1), q_0, linewidth=2.0)
ploted_q_0_max, = plt.plot(0.001*np.arange(0,np.size(q_0_max),1), q_0_max, "k--", linewidth=2.0)
ploted_q_0_min, = plt.plot(0.001*np.arange(0,np.size(q_0_min),1), q_0_min, "k-.", linewidth=2.0)
plt.legend([ploted_q_0, ploted_q_0_max, ploted_q_0_min], ['$q_0$', '$q_{max}$', '$q_{min}$'])
	



source = open("saved_data/q_1.txt", "r")
q_1 = source.readlines()
source.close()
plt.figure('q')
ploted_q_1, = plt.plot(0.001*np.arange(0,np.size(q_1),1), q_1, linewidth=2.0)
plt.legend([ploted_q_1], ['$q_1$'])
	







source = open("saved_data/q_2.txt", "r")
q_2 = source.readlines()
source.close()
plt.figure('q')
ploted_q_2, = plt.plot(0.001*np.arange(0,np.size(q_2),1), q_2, linewidth=2.0)
plt.legend([ploted_q_2], ['$q_2$'])
	





source = open("saved_data/q_3.txt", "r")
q_3 = source.readlines()
source.close()
plt.figure('q')
ploted_q_3, = plt.plot(0.001*np.arange(0,np.size(q_3),1), q_3, linewidth=2.0)
plt.legend([ploted_q_3], ['$q_3$'])





source = open("saved_data/q_4.txt", "r")
q_4 = source.readlines()
source.close()
plt.figure('q')
ploted_q_4, = plt.plot(0.001*np.arange(0,np.size(q_4),1), q_4, linewidth=2.0)
plt.legend([ploted_q_4], ['$q_4$'])
	






source = open("saved_data/q_5.txt", "r")
q_5 = source.readlines()
source.close()
plt.figure('q')	
ploted_q_5, = plt.plot(0.001*np.arange(0,np.size(q_5),1), q_5, linewidth=2.0)
plt.legend([ploted_q_5], ['$q_5$'])






source = open("saved_data/q_6.txt", "r")
q_6 = source.readlines()
source.close()
plt.figure('q')
ploted_q_6, = plt.plot(0.001*np.arange(0,np.size(q_6),1), q_6, linewidth=2.0)
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

plt.figure('qdot')
	
ploted_q_dot_0, = plt.plot(0.001*np.arange(0,np.size(q_dot_0),1), q_dot_0, linewidth=2.0)
ploted_q_dot_0_max, = plt.plot(0.001*np.arange(0,np.size(q_dot_0_max),1), q_dot_0_max, "k--", linewidth=2.0)
ploted_q_dot_0_min, = plt.plot(0.001*np.arange(0,np.size(q_dot_0_min),1), q_dot_0_min, "k-.", linewidth=2.0)

	



source = open("saved_data/q_dot_1.txt", "r")
q_dot_1 = source.readlines()
source.close()
plt.figure('qdot')
ploted_q_dot_1, = plt.plot(0.001*np.arange(0,np.size(q_dot_1),1), q_dot_1, linewidth=2.0)

	







source = open("saved_data/q_dot_2.txt", "r")
q_dot_2 = source.readlines()
source.close()
plt.figure('qdot')
ploted_q_dot_2, = plt.plot(0.001*np.arange(0,np.size(q_dot_2),1), q_dot_2, linewidth=2.0)

	





source = open("saved_data/q_dot_3.txt", "r")
q_dot_3 = source.readlines()
source.close()
plt.figure('qdot')
ploted_q_dot_3, = plt.plot(0.001*np.arange(0,np.size(q_dot_3),1), q_dot_3, linewidth=2.0)






source = open("saved_data/q_dot_4.txt", "r")
q_dot_4 = source.readlines()
source.close()
plt.figure('qdot')
ploted_q_dot_4, = plt.plot(0.001*np.arange(0,np.size(q_dot_4),1), q_dot_4, linewidth=2.0)

	






source = open("saved_data/q_dot_5.txt", "r")
q_dot_5 = source.readlines()
source.close()
plt.figure('qdot')	
ploted_q_dot_5, = plt.plot(0.001*np.arange(0,np.size(q_dot_5),1), q_dot_5, linewidth=2.0)







source = open("saved_data/q_dot_6.txt", "r")
q_dot_6 = source.readlines()
source.close()
plt.figure('qdot')
ploted_q_dot_6, = plt.plot(0.001*np.arange(0,np.size(q_dot_6),1), q_dot_6, linewidth=2.0)
plt.legend([ploted_q_dot_6, ploted_q_dot_5, ploted_q_dot_4, ploted_q_dot_3, ploted_q_dot_2, ploted_q_dot_1, ploted_q_dot_0, ploted_q_dot_0_max, ploted_q_dot_0_min], ['$\dot{q}_6$', '$\dot{q}_5$', '$\dot{q}_4$', '$\dot{q}_3$', '$\dot{q}_2$', '$\dot{q}_1$', '$\dot{q}_0$', '$\dot{q}_{max}$', '$\dot{q}_{min}$' ])


plt.ylabel("$(rad/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dot_0), -4.0, 2.5])
plt.grid(True)
#############################q_dot plot##############################












############################Velocities plot##############################

source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()

source = open("saved_data/E_7_max.txt", "r")
E_7_max = source.readlines()
source.close()

plt.figure('Ec_Xdot')
plt.subplot(2, 2, 1)

ploted_E_7_C_ob, = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)
ploted_E_7_max, = plt.plot(0.001*np.arange(0,np.size(E_7_max),1), E_7_max, "r--", linewidth=2.0)
plt.legend([ploted_E_7_C_ob, ploted_E_7_max], ['$E_{ceff}$', '$E_{ceffmax}$'])
	
plt.ylabel("$(Joules)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(E_7_C_ob), -2.5, 2.5])
plt.grid(True)




source = open("saved_data/V_7_x.txt", "r")
V_7_x = source.readlines()
source.close()

source = open("saved_data/V_7_des_x.txt", "r")
V_7_des_x = source.readlines()
source.close()

plt.figure('Ec_Xdot')
plt.subplot(2, 2, 2)
	
ploted_V_7_x, = plt.plot(0.001*np.arange(0,np.size(V_7_x),1), V_7_x, "b", linewidth=2.0)
ploted_V_7_des_x, = plt.plot(0.001*np.arange(0,np.size(V_7_des_x),1), V_7_des_x, "r--", linewidth=2.0)
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

plt.figure('Ec_Xdot')
plt.subplot(2, 2, 3)
	
ploted_V_7_y, = plt.plot(0.001*np.arange(0,np.size(V_7_y),1), V_7_y, "b", linewidth=2.0)
ploted_V_7_des_y, = plt.plot(0.001*np.arange(0,np.size(V_7_des_y),1), V_7_des_y, "r--", linewidth=2.0)
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

plt.figure('Ec_Xdot')
plt.subplot(2, 2, 4)
	
ploted_V_7_z, = plt.plot(0.001*np.arange(0,np.size(V_7_z),1), V_7_z, "b", linewidth=2.0)
ploted_V_7_des_z, = plt.plot(0.001*np.arange(0,np.size(V_7_des_z),1), V_7_des_z, "r--", linewidth=2.0)
plt.legend([ploted_V_7_z, ploted_V_7_des_z], ['$\dot{X_z}$', '$\dot{X^*}_z$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_z), -1.0, 2])
plt.grid(True)
############################Velocities plot##############################



###########################Distance plot222222222##############################
source = open("saved_data/d_07_ob.txt", "r")
Dist_07_nrst_ob = source.readlines()
source.close()

source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()

source = open("saved_data/E_7_max.txt", "r")
E_7_max = source.readlines()
source.close()


plt.figure('Dist_Ec_Ecmax_plot')

ploted_E_7_C_ob, = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)
ploted_E_7_max, = plt.plot(0.001*np.arange(0,np.size(E_7_max),1), E_7_max, "r--", linewidth=2.0)
ploted_Dist_07_nrst_ob, = plt.plot(0.001*np.arange(0,np.size(Dist_07_nrst_ob),1), Dist_07_nrst_ob, "g", linewidth=2.0)
plt.legend([ploted_E_7_C_ob, ploted_E_7_max, ploted_Dist_07_nrst_ob], ['$E_{eff} (Joule)$', '$E_{effmax} (Joule)$', "$d_{eff-obst} (m)$"])

#	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Dist_07_nrst_ob), -0.04, 0.8])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################Distance plot222222222##############################



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

plt.figure('Force Sensor')


ploted_Force_sensor_x, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_x),1), Force_sensor_x, "b", linewidth=2.0)
ploted_Force_sensor_y, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_y),1), Force_sensor_y, "r", linewidth=2.0)
ploted_Force_sensor_z, = plt.plot(0.001*np.arange(0,np.size(Force_sensor_z),1), Force_sensor_z, "g", linewidth=2.0)
plt.legend([ploted_Force_sensor_x, ploted_Force_sensor_y, ploted_Force_sensor_z], ['$F_x (N)$', '$F_y (N)$', '$F_z (N)$'])

	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Force_sensor_x), 0, 600])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################









###########################Ec reconstructed with Ep & Ep profil##############################

source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()

source = open("saved_data/Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst.txt", "r")
Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst = source.readlines()
source.close()

plt.figure('Ec reconstructed with Ep direction obstacle')


ploted_E_7_C_ob,                                             = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)
ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst),1), Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst, "r--", linewidth=2.0)

plt.legend([ploted_E_7_C_ob, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst], ['$E_{c_{|k}} (Joule)$', '$\sum\limits_{n=1}^{k-1} E_{p_{|n}} (Joule)$'])

#	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(E_7_C_ob), -2.5, 2.5])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
###########################Ec reconstructed with Ep & Ep profil##############################









###########################Ec reconstructed with Ep##############################

source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()

source = open("saved_data/Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real.txt", "r")
Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real = source.readlines()
source.close()

plt.figure('Ec reconstructed with Ep direction Xerr')


ploted_E_7_C_ob,                                             = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)
ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real),1), Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real, "r--", linewidth=2.0)

plt.legend([ploted_E_7_C_ob, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real], ['$E_{eff} (Joule)$', '$sum E_{p} (Joule)$'])

#	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(E_7_C_ob), -0.8, 0.8])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
###########################Ec reconstructed with Ep##############################




###########################Ep profil selon Xerr##############################
source = open("saved_data/Real_Ep_7_X_err_real.txt", "r")
Real_Ep_7_X_err_real = source.readlines()
source.close()

source = open("saved_data/Real_Ep_7_X_err_real_obst.txt", "r")
Real_Ep_7_X_err_real_obst = source.readlines()
source.close()


source = open("saved_data/Acc_7_x.txt", "r")
Acc_7_x = source.readlines()
source.close()



plt.figure('Ep profil selon Xerr & obst1')


ploted_Real_Ep_7_X_err_real, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real),1), Real_Ep_7_X_err_real, "b", linewidth=2.0)
ploted_Real_Ep_7_X_err_real_obst, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_obst),1), Real_Ep_7_X_err_real_obst, "r--", linewidth=2.0)
ploted_Acc_7_x, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x),1), Acc_7_x, "m", linewidth=2.0)

plt.legend([ploted_Real_Ep_7_X_err_real, ploted_Real_Ep_7_X_err_real_obst, ploted_Acc_7_x], ['$E_{p}profile Xerr (Joule)$', '$E_{p}profile obst1 (Joule)$', '$Real \ddot{X}_x (m/s^2)$'])

#	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real), -0.1, 0.1])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
###########################Ep profil selon Xerr##############################






###########################Ep profil selon Xerr##############################
source = open("saved_data/Real_Ep_7_X_err_real_x.txt", "r")
Real_Ep_7_X_err_real_x = source.readlines()
source.close()

plt.figure('Ep profil XYZ REAL')
plt.subplot(3, 1, 1)

ploted_Real_Ep_7_X_err_real_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_x),1), Real_Ep_7_X_err_real_x, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_x], ['$E_{p_{profile}}^{x} (Joule)$'])

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_x), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Real_Ep_7_X_err_real_y.txt", "r")
Real_Ep_7_X_err_real_y = source.readlines()
source.close()

plt.figure('Ep profil XYZ REAL')
plt.subplot(3, 1, 2)

ploted_Real_Ep_7_X_err_real_y, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_y),1), Real_Ep_7_X_err_real_y, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_y], ['$E_{p_{profile}}^{y} (Joule)$'])


plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_y), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Real_Ep_7_X_err_real_z.txt", "r")
Real_Ep_7_X_err_real_z = source.readlines()
source.close()

plt.figure('Ep profil XYZ REAL')
plt.subplot(3, 1, 3)

ploted_Real_Ep_7_X_err_real_z, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_z),1), Real_Ep_7_X_err_real_z, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_z], ['$E_{p_{profile}}^{z} (Joule)$'])

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_z), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
###########################Ep profil selon Xerr##############################







#############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################
#source = open("saved_data/Real_Ec_7_x.txt", "r")
#Real_Ec_7_x = source.readlines()
#source.close()

#source = open("saved_data/Ec_max_7_x.txt", "r")
#Ec_max_7_x = source.readlines()
#source.close()

#source = open("saved_data/Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x.txt", "r")
#Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x = source.readlines()
#source.close()

#source = open("saved_data/Acc_7_x.txt", "r")
#Acc_7_x = source.readlines()
#source.close()

##source = open("saved_data/Acc_7_x_nxt_step_limit.txt", "r")
##Acc_7_x_nxt_step_limit = source.readlines()
##source.close()

#plt.figure(1)

#ploted_Real_Ec_7_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x),1), Real_Ec_7_x, "b", linewidth=2.0)
#ploted_Ec_max_7_x, = plt.plot(0.001*np.arange(0,np.size(Ec_max_7_x),1), Ec_max_7_x, "r--", linewidth=2.0)
#ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x),1), Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, "b--", linewidth=2.0)
#ploted_Acc_7_x, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x),1), Acc_7_x, "k", linewidth=2.0)
##ploted_Acc_7_x_nxt_step_limit, = plt.plot(0.001*np.arange(0,np.size(Acc_7_x_nxt_step_limit),1), Acc_7_x_nxt_step_limit, "c", linewidth=2.0)
##plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x, ploted_Acc_7_x_nxt_step_limit], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$', '$\ddot{X}_{x|k+1}^{lim} (m/s^2)$'])
#plt.legend([ploted_Real_Ec_7_x, ploted_Ec_max_7_x, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_x, ploted_Acc_7_x], ['$Real E_{c} (J)$', '$E_{cmax} (J)$', '$Real E_{c|sum Ep} (J)$', '$Real \ddot{X}_x (m/s^2)$'])
#	
#plt.xlabel("$time (s)$", fontsize=20)
#plt.axis([0,0.001*np.size(Real_Ec_7_x), -2, 2])
#plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
#plt.grid(True)
############################plot real Ec, Real Ec_rcnstrctd_with_small_Ep, Real current Acc_x##############################














##FINAL PLOTS
###########################q_0 plot_comp FINAL DEBUG##############################
source_1 = open("saved_data/q_0.txt", "r")
q_0 = source_1.readlines()
source_1.close()

source_1 = open("saved_data/q_0_max.txt", "r")
q_0_max = source_1.readlines()
source_1.close()

source_1 = open("saved_data/q_0_min.txt", "r")
q_0_min = source_1.readlines()
source_1.close()

#plt.figure(' q FINAL !!!')
plt.figure('comp q, q_ddot, q_dddot FINAL3 !!!')
plt.figure(figsize=(10,10))
ax1 = plt.subplot2grid((5,1), (0,0), colspan=1)
#ax1.set_title('-a-')

#plt.subplot(4, 1, 1)


ploted_q_0,                    = plt.plot(0.001*np.arange(0,np.size(q_0),1),                    q_0,                    "b",   linewidth=2.0)
ploted_q_0_max,                = plt.plot(0.001*np.arange(0,np.size(q_0_max),1),                q_0_max,                "r--",   linewidth=2.0)


plt.legend([ploted_q_0, ploted_q_0_max], ['$q_{0}$', '$q_{M_{0}}$'])

#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(rad)$", fontsize=20)
plt.axis([0,0.001*np.size(q_0), 0.1, 3.2])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
###########################q_0 plot_comp FINAL DEBUG##############################

#############################compatibility q_dot-q_dddot0 plot##############################
source_1 = open("saved_data/q_dot_bounds_max_comp_Acc_Posi_vel_cmd_0.txt", "r")
q_dot_bounds_max_comp_Acc_Posi_vel_cmd_0 = source_1.readlines()
source_1.close()


source_1 = open("saved_data/q_dot_0.txt", "r")
q_dot_0 = source_1.readlines()
source_1.close()

#plt.figure('comp q, q_ddot, q_dddot FINAL2 !!!')
ax2 = plt.subplot2grid((5,1), (1,0), colspan=1)
#ax2.set_title('-b-')
#plt.subplot(4, 1, 2)

ploted_q_dot_0,                                 = plt.plot(0.001*np.arange(0,np.size(q_dot_0),1),                                       q_dot_0,                                       "b",  linewidth=2.0)
ploted_q_dot_0_max,                             = plt.plot(0.001*np.arange(0,np.size(q_dot_0_max),1),                                   q_dot_0_max,                                   "r--",  linewidth=2.0)

#ploted_q_dot_bounds_max_comp_Acc_Posi_vel_cmd_0 = plt.plot(0.001*np.arange(0,np.size(q_dot_bounds_max_comp_Acc_Posi_vel_cmd_0),1),      q_dot_bounds_max_comp_Acc_Posi_vel_cmd_0,      "m",  linewidth=2.0)

plt.legend([ploted_q_dot_0, ploted_q_dot_0_max], ['$\dot{q}_0$', '$\dot{q}_{M_{0}}$'])


#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(rad/s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dot_0), -0.4, 3])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
#############################compatibility q_dot-q_dddot0 plot##############################

############################q_dot_dot plot_comp##############################
source_1 = open("saved_data/q_dotdot_0.txt", "r")
q_dotdot_0 = source_1.readlines()
source_1.close()

source_1 = open("saved_data/q_dotdot_0_max_comp.txt", "r")
q_dotdot_0_max_comp = source_1.readlines()
source_1.close()





#plt.figure('comp q, q_ddot, q_dddot FINAL2 !!!')
ax3 = plt.subplot2grid((5,1), (2,0), colspan=1)
#ax3.set_title('-c-')
#plt.subplot(4, 1, 3)



ploted_q_dotdot_0,                      = plt.plot(0.001*np.arange(0,np.size(q_dotdot_0),1),                      q_dotdot_0,                   "b",   linewidth=2.0)
ploted_q_dotdot_0_max_comp,             = plt.plot(0.001*np.arange(0,np.size(q_dotdot_0_max_comp),1),             q_dotdot_0_max_comp,          "m--",   linewidth=2.0)




plt.legend([ploted_q_dotdot_0, ploted_q_dotdot_0_max_comp], ['$\ddot{q}_{0}$', r'$f_{\alpha}$'])


#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(rad/s^2)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdot_0), -5, 14])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
############################q_dot_dot plot_comp##############################

##########################q_dot_dot_dot  pour la comp posi_jerk plot##############################
source_1 = open("saved_data/q_dotdotdot_gurobi_0.txt", "r")
q_dotdotdot_gurobi_0 = source_1.readlines()
source_1.close()

source_1 = open("saved_data/q_dotdotdot_0_max.txt", "r")
q_dotdotdot_0_max = source_1.readlines()
source_1.close()

source_1 = open("saved_data/q_dotdotdot_0_min.txt", "r")
q_dotdotdot_0_min = source_1.readlines()
source_1.close()

#source_1 = open("saved_data/q_dotdot_bounds_deriv_max_comp_0.txt", "r")
#q_dotdot_bounds_deriv_max_comp_0 = source_1.readlines()
#source_1.close()



#plt.figure('comp q, q_ddot, q_dddot FINAL2 !!!')
ax4 = plt.subplot2grid((5,1), (3,0), colspan=1)
#ax4.set_title('-d-')
#plt.subplot(4, 1, 4)

ploted_q_dotdotdot_gurobi_0,   = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_gurobi_0),1),   q_dotdotdot_gurobi_0,   "b",   linewidth=2.0)
ploted_q_dotdotdot_0_max,      = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_max),1),      q_dotdotdot_0_max,      "r--", linewidth=2.0)
ploted_q_dotdotdot_0_min,      = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_min),1),      q_dotdotdot_0_min,      "r:",  linewidth=2.0)
#ploted_q_dotdot_bounds_deriv_max_comp_0, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_bounds_deriv_max_comp_0),1),   q_dotdot_bounds_deriv_max_comp_0,          "m--",   linewidth=2.0)

#plt.legend([ploted_q_dotdotdot_gurobi_0, ploted_q_dotdotdot_0_max, ploted_q_dotdotdot_0_min, ploted_q_dotdot_bounds_deriv_max_comp_0], ['$\dddot{q}_{0}$', '$\dddot{q_{M_{0}}}$', '$\dddot{q_{m_{0}}}$', '$\dddot{q}_{max.0.constr}$'])
plt.legend([ploted_q_dotdotdot_gurobi_0, ploted_q_dotdotdot_0_max, ploted_q_dotdotdot_0_min], ['$\dddot{q_{0}}$', '$\dddot{q_{M_{0}}}$', '$\dddot{q_{m_{0}}}$'])


plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(rad/s^3)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdotdot_gurobi_0), -35, 35])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
##########################q_dot_dot_dot  pour la comp posi_jerk plot##############################


#############################n for all##############################
#n pour comp vel jerk
source = open("saved_data/n_pos_jerk_vel_0.txt", "r")
n_pos_jerk_vel_0 = source.readlines()
source.close()


#plt.figure('n for all FINAL !!!')
#plt.figure('comp q, q_ddot, q_dddot FINAL2 !!!')
ax5 = plt.subplot2grid((5,1), (4,0), rowspan=1)
#ax5.set_title('-e-')

#n pour comp vel jerk
ploted_n_pos_jerk_vel_0,                          = plt.plot(0.001*np.arange(0,np.size(n_pos_jerk_vel_0),1),                          n_pos_jerk_vel_0,                          "k",   linewidth=2.0)


#n for all
plt.legend([ploted_n_pos_jerk_vel_0], ['$n_{1}$'])


plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(n_pos_jerk_vel_0), -200, 620])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
#############################n for all##############################











#qddot

#############################q_dotdot plot##############################
source = open("saved_data/q_dotdot_0.txt", "r")
q_dotdot_0 = source.readlines()
source.close()



plt.figure('qdotdot')
	
ploted_q_dotdot_0, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_0),1), q_dotdot_0, "b", linewidth=2.0)


	



source = open("saved_data/q_dotdot_1.txt", "r")
q_dotdot_1 = source.readlines()
source.close()
plt.figure('qdotdot')
ploted_q_dotdot_1, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_1),1), q_dotdot_1, "g", linewidth=2.0)

	







source = open("saved_data/q_dotdot_2.txt", "r")
q_dotdot_2 = source.readlines()
source.close()
plt.figure('qdotdot')
ploted_q_dotdot_2, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_2),1), q_dotdot_2, "r", linewidth=2.0)

	





source = open("saved_data/q_dotdot_3.txt", "r")
q_dotdot_3 = source.readlines()
source.close()
plt.figure('qdotdot')
ploted_q_dotdot_3, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_3),1), q_dotdot_3, "c", linewidth=2.0)






source = open("saved_data/q_dotdot_4.txt", "r")
q_dotdot_4 = source.readlines()
source.close()
plt.figure('qdotdot')
ploted_q_dotdot_4, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_4),1), q_dotdot_4, "m", linewidth=2.0)

	






source = open("saved_data/q_dotdot_5.txt", "r")
q_dotdot_5 = source.readlines()
source.close()
plt.figure('qdotdot')	
ploted_q_dotdot_5, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_5),1), q_dotdot_5, "y", linewidth=2.0)







source = open("saved_data/q_dotdot_6.txt", "r")
q_dotdot_6 = source.readlines()
source.close()
plt.figure('qdotdot')
ploted_q_dotdot_6, = plt.plot(0.001*np.arange(0,np.size(q_dotdot_6),1), q_dotdot_6, "k", linewidth=2.0)
plt.legend([ploted_q_dotdot_6, ploted_q_dotdot_5, ploted_q_dotdot_4, ploted_q_dotdot_3, ploted_q_dotdot_2, ploted_q_dotdot_1, ploted_q_dotdot_0], ['$\ddot{q}_6$', '$\ddot{q}_5$', '$\ddot{q}_4$', '$\ddot{q}_3$', '$\ddot{q}_2$', '$\ddot{q}_1$', '$\ddot{q}_0$'])


plt.ylabel("$(rad/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdot_0), -25, 25])
plt.grid(True)
#############################q_dotdot plot##############################



















##JERK
#############################q_dotdotdot plot##############################
source = open("saved_data/q_dotdotdot_0.txt", "r")
q_dotdotdot_0 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_0, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0),1), q_dotdotdot_0, "b", linewidth=2.0)



source = open("saved_data/q_dotdotdot_0_min.txt", "r")
q_dotdotdot_0_min = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_0_min, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_min),1), q_dotdotdot_0_min, "k--", linewidth=2.0)



source = open("saved_data/q_dotdotdot_0_max.txt", "r")
q_dotdotdot_0_max = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_0_max, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_max),1), q_dotdotdot_0_max, "k-.", linewidth=2.0)




source = open("saved_data/q_dotdotdot_1.txt", "r")
q_dotdotdot_1 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_1, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_1),1), q_dotdotdot_1, "g", linewidth=2.0)

	







source = open("saved_data/q_dotdotdot_2.txt", "r")
q_dotdotdot_2 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_2, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_2),1), q_dotdotdot_2, "r", linewidth=2.0)

	





source = open("saved_data/q_dotdotdot_3.txt", "r")
q_dotdotdot_3 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_3, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_3),1), q_dotdotdot_3, "c", linewidth=2.0)






source = open("saved_data/q_dotdotdot_4.txt", "r")
q_dotdotdot_4 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_4, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_4),1), q_dotdotdot_4, "m", linewidth=2.0)

	






source = open("saved_data/q_dotdotdot_5.txt", "r")
q_dotdotdot_5 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')	
ploted_q_dotdotdot_5, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_5),1), q_dotdotdot_5, "y", linewidth=2.0)







source = open("saved_data/q_dotdotdot_6.txt", "r")
q_dotdotdot_6 = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_6, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_6),1), q_dotdotdot_6, "k", linewidth=2.0)
plt.legend([ploted_q_dotdotdot_6, ploted_q_dotdotdot_5, ploted_q_dotdotdot_4, ploted_q_dotdotdot_3, ploted_q_dotdotdot_2, ploted_q_dotdotdot_1, ploted_q_dotdotdot_0, ploted_q_dotdotdot_0_min, ploted_q_dotdotdot_0_max], ['$\dddot{q}_6$', '$\dddot{q}_5$', '$\dddot{q}_4$', '$\dddot{q}_3$', '$\dddot{q}_2$', '$\dddot{q}_1$', '$\dddot{q}_0$', '$\dddot{q}_{0_{min}}$', '$\dddot{q}_{0_{max}}$'])


plt.ylabel("$(rad/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdotdot_0), -450, 450])
plt.grid(True)
#############################q_dotdotdot plot##############################




#############################X plot##############################
source = open("saved_data/trajectory_x.txt", "r")
X_x = source.readlines()
source.close()

source = open("saved_data/des_trajectory_x.txt", "r")
des_X_x = source.readlines()
source.close()

plt.figure(13)
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

plt.figure(13)
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

plt.figure(13)
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


plt.figure(13)
plt.subplot(2, 2, 4)
	
ploted_X_err, = plt.plot(0.001*np.arange(0,np.size(X_err),1), X_err, "b", linewidth=2.0)

plt.legend([ploted_X_err], ['$X_{err}$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_err), 0.0, 0.005])
plt.grid(True)
#############################X plot##############################















###########################Ep profil selon XYZ FUTUR##############################
source = open("saved_data/Ep_x_before_after.txt", "r")
Ep_x_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_x_before_after_max.txt", "r")
Ep_x_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_x_before_after_min.txt", "r")
Ep_x_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTURE')
plt.subplot(3, 1, 1)

ploted_Ep_x_before_after, = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after),1), Ep_x_before_after, "b", linewidth=2.0)
ploted_Ep_x_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after_max),1), Ep_x_before_after_max, "r--", linewidth=2.0)
ploted_Ep_x_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after_min),1), Ep_x_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_x_before_after, ploted_Ep_x_before_after_max, ploted_Ep_x_before_after_min], ['$E_{p_{profileF}}^{x} (Joule)$', '$E_{p_{max}}^{x} (Joule)$', '$E_{p_{min}}^{x} (Joule)$'])

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_x), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)



source = open("saved_data/Ep_y_before_after.txt", "r")
Ep_y_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_y_before_after_max.txt", "r")
Ep_y_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_y_before_after_min.txt", "r")
Ep_y_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTURE')
plt.subplot(3, 1, 2)

ploted_Ep_y_before_after, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after),1), Ep_y_before_after, "b", linewidth=2.0)
ploted_Ep_y_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after_max),1), Ep_y_before_after_max, "r--", linewidth=2.0)
ploted_Ep_y_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after_min),1), Ep_y_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_y_before_after, ploted_Ep_y_before_after_max, ploted_Ep_y_before_after_min], ['$E_{p_{profileF}}^{y} (Joule)$', '$E_{p_{max}}^{y} (Joule)$', '$E_{p_{min}}^{y} (Joule)$'])

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_y), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Ep_z_before_after.txt", "r")
Ep_z_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_z_before_after_max.txt", "r")
Ep_z_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_z_before_after_min.txt", "r")
Ep_z_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTURE')
plt.subplot(3, 1, 3)

ploted_Ep_z_before_after, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after),1), Ep_z_before_after, "b", linewidth=2.0)
ploted_Ep_z_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after_max),1), Ep_z_before_after_max, "r--", linewidth=2.0)
ploted_Ep_z_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after_min),1), Ep_z_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_z_before_after, ploted_Ep_z_before_after_max, ploted_Ep_z_before_after_min], ['$E_{p_{profileF}}^{z} (Joule)$', '$E_{p_{max}}^{z} (Joule)$', '$E_{p_{min}}^{z} (Joule)$'])

plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_z), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)

###########################Ep profil selon XYZ FUTUR##############################



###########################OPTIMIZED F_eq##############################
source_1 = open("saved_data/disp_force_sensor_x.txt", "r")
disp_force_sensor_x = source_1.readlines()
source_1.close()

source_1 = open("saved_data/disp_force_sensor_y.txt", "r")
disp_force_sensor_y = source_1.readlines()
source_1.close()

source_1 = open("saved_data/disp_force_sensor_z.txt", "r")
disp_force_sensor_z = source_1.readlines()
source_1.close()


plt.figure('disp_force_sensor !!!')


ploted_disp_force_sensor_x,                    = plt.plot(0.001*np.arange(0,np.size(disp_force_sensor_x),1),                    disp_force_sensor_x,                    "b",   linewidth=2.0)
ploted_disp_force_sensor_y,                    = plt.plot(0.001*np.arange(0,np.size(disp_force_sensor_y),1),                    disp_force_sensor_y,                    "g",   linewidth=2.0)
ploted_disp_force_sensor_z,                    = plt.plot(0.001*np.arange(0,np.size(disp_force_sensor_z),1),                    disp_force_sensor_z,                    "m",   linewidth=2.0)

plt.legend([ploted_disp_force_sensor_x, ploted_disp_force_sensor_y, ploted_disp_force_sensor_z], ['$d_x (m)$', '$d_y (m)$', '$d_z (m)$'])

#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(N)$", fontsize=20)
plt.axis([0,0.001*np.size(disp_force_sensor_x), -0.1, 0.1])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
###########################OPTIMIZED F_eq##############################












###########################OPTIMIZED F_eq##############################
source_1 = open("saved_data/optimized_F_eq.txt", "r")
optimized_F_eq = source_1.readlines()
source_1.close()

source_1 = open("saved_data/status_K_max.txt", "r")
status_X_ddot_max = source_1.readlines()
source_1.close()

plt.figure('OPTIMIZED F_eq !!!')


ploted_optimized_F_eq,                    = plt.plot(0.001*np.arange(0,np.size(optimized_F_eq),1),                    optimized_F_eq,                    "b",   linewidth=2.0)
ploted_status_X_ddot_max,                 = plt.plot(0.001*np.arange(0,np.size(status_X_ddot_max),1),                 status_X_ddot_max,                 "r--", linewidth=2.0)


plt.legend([ploted_optimized_F_eq, ploted_status_X_ddot_max], ['$K$', '$STATUS$'])

#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(N)$", fontsize=20)
plt.axis([0,0.001*np.size(optimized_F_eq), -5000, 5000])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
###########################OPTIMIZED F_eq##############################






###########################Ep profil selon XYZ FUTUR##############################
source = open("saved_data/Ep_x_before_after.txt", "r")
Ep_x_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_x_reconstructed.txt", "r")
Ep_x_reconstructed = source.readlines()
source.close()

source = open("saved_data/Ep_x_before_after_max.txt", "r")
Ep_x_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_x_before_after_min.txt", "r")
Ep_x_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTUR WITH RECNSTRUTED')
plt.subplot(3, 1, 1)

ploted_Ep_x_before_after,     = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after),1), Ep_x_before_after, "b", linewidth=2.0)
ploted_Ep_x_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_x_reconstructed),1), Ep_x_reconstructed, "m", linewidth=2.0)
ploted_Ep_x_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after_max),1), Ep_x_before_after_max, "r--", linewidth=2.0)
ploted_Ep_x_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_x_before_after_min),1), Ep_x_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_x_before_after, ploted_Ep_x_reconstructed, ploted_Ep_x_before_after_max, ploted_Ep_x_before_after_min], ['$E_{p_{profileXerr}}^{x}$', '$E_{p_{profileXerrrecn}}^{x}$', '$E_{p_{max}}^{x}$', '$E_{p_{min}}^{x}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_x), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)



source = open("saved_data/Ep_y_before_after.txt", "r")
Ep_y_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_y_reconstructed.txt", "r")
Ep_y_reconstructed = source.readlines()
source.close()

source = open("saved_data/Ep_y_before_after_max.txt", "r")
Ep_y_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_y_before_after_min.txt", "r")
Ep_y_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTUR WITH RECNSTRUTED')
plt.subplot(3, 1, 2)

ploted_Ep_y_before_after, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after),1), Ep_y_before_after, "b", linewidth=2.0)
ploted_Ep_y_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_y_reconstructed),1), Ep_y_reconstructed, "m", linewidth=2.0)
ploted_Ep_y_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after_max),1), Ep_y_before_after_max, "r--", linewidth=2.0)
ploted_Ep_y_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_y_before_after_min),1), Ep_y_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_y_before_after, ploted_Ep_y_reconstructed, ploted_Ep_y_before_after_max, ploted_Ep_y_before_after_min], ['$E_{p_{profileXerr}}^{y}$', '$E_{p_{profileXerrrecn}}^{y}$', '$E_{p_{max}}^{y}$', '$E_{p_{min}}^{y}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_y), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Ep_z_before_after.txt", "r")
Ep_z_before_after = source.readlines()
source.close()

source = open("saved_data/Ep_z_reconstructed.txt", "r")
Ep_z_reconstructed = source.readlines()
source.close()

source = open("saved_data/Ep_z_before_after_max.txt", "r")
Ep_z_before_after_max = source.readlines()
source.close()

source = open("saved_data/Ep_z_before_after_min.txt", "r")
Ep_z_before_after_min = source.readlines()
source.close()

plt.figure('Ep profil XYZ FUTUR WITH RECNSTRUTED')
plt.subplot(3, 1, 3)

ploted_Ep_z_before_after, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after),1), Ep_z_before_after, "b", linewidth=2.0)
ploted_Ep_z_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_z_reconstructed),1), Ep_z_reconstructed, "m", linewidth=2.0)
ploted_Ep_z_before_after_max, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after_max),1), Ep_z_before_after_max, "r--", linewidth=2.0)
ploted_Ep_z_before_after_min, = plt.plot(0.001*np.arange(0,np.size(Ep_z_before_after_min),1), Ep_z_before_after_min, "r-.", linewidth=2.0)
plt.legend([ploted_Ep_z_before_after, ploted_Ep_z_reconstructed, ploted_Ep_z_before_after_max, ploted_Ep_z_before_after_min], ['$E_{p_{profileXerr}}^{z}$', '$E_{p_{profileXerrrecn}}^{z}$', '$E_{p_{max}}^{z}$', '$E_{p_{min}}^{z}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_z), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)

###########################Ep profil selon XYZ FUTUR##############################












###########################Ep_7_rebuilt_Xerr##############################
source_1 = open("saved_data/Ep_7_rebuilt_Xerr.txt", "r")
Ep_7_rebuilt_Xerr= source_1.readlines()
source_1.close()

source_1 = open("saved_data/Ep_max_Xerr.txt", "r")
Ep_max_Xerr = source_1.readlines()
source_1.close()



plt.figure('Ep_7_rebuilt_Xerr !!!')


ploted_Ep_7_rebuilt_Xerr,              = plt.plot(0.001*np.arange(0,np.size(Ep_7_rebuilt_Xerr),1),                    Ep_7_rebuilt_Xerr,                    "b",   linewidth=2.0)
ploted_Ep_max_Xerr,                    = plt.plot(0.001*np.arange(0,np.size(Ep_max_Xerr),1),                    Ep_max_Xerr,                    "r--",   linewidth=2.0)


plt.legend([ploted_Ep_7_rebuilt_Xerr], ['$E_{p}^{err}$'])

#plt.xlabel("$time (s)$", fontsize=20)
plt.ylabel("$(J)$", fontsize=20)
plt.axis([0,0.001*np.size(optimized_F_eq), 0.0, 0.1])
plt.ticklabel_format(useOffset=False)
plt.grid(True)
###########################Ep_7_rebuilt_Xerr##############################















plt.show()