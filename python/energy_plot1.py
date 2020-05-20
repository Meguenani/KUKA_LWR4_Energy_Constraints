import matplotlib.pyplot as plt	
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pylab

params = {'legend.fontsize': 20,
          'legend.linewidth': 2}
plt.rcParams.update(params)








############################Velocities plot##############################
source_1 = open("saved_data/optimized_F_eq.txt", "r")
optimized_F_eq = source_1.readlines()
source_1.close()


plt.figure('K_Xdot')
plt.subplot(2, 2, 1)

ploted_optimized_F_eq,                    = plt.plot(0.001*np.arange(0,np.size(optimized_F_eq),1),                    optimized_F_eq,                    "b",   linewidth=2.0)

plt.legend([ploted_optimized_F_eq], ['$K$'])
	
plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(optimized_F_eq), -220, -410])
plt.grid(True)




source = open("saved_data/V_7_x.txt", "r")
V_7_x = source.readlines()
source.close()

source = open("saved_data/V_7_des_x.txt", "r")
V_7_des_x = source.readlines()
source.close()

plt.figure('K_Xdot')
plt.subplot(2, 2, 2)
	
ploted_V_7_x, = plt.plot(0.001*np.arange(0,np.size(V_7_x),1), V_7_x, "b", linewidth=2.0)
ploted_V_7_des_x, = plt.plot(0.001*np.arange(0,np.size(V_7_des_x),1), V_7_des_x, "r--", linewidth=2.0)
plt.legend([ploted_V_7_x, ploted_V_7_des_x], ['$\dot{X}_{x}$', '$\dot{X}_{x}^{*}$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_x), -1.3, 1.3])
plt.grid(True)





source = open("saved_data/V_7_y.txt", "r")
V_7_y = source.readlines()
source.close()

source = open("saved_data/V_7_des_y.txt", "r")
V_7_des_y = source.readlines()
source.close()

plt.figure('K_Xdot')
plt.subplot(2, 2, 3)
	
ploted_V_7_y, = plt.plot(0.001*np.arange(0,np.size(V_7_y),1), V_7_y, "b", linewidth=2.0)
ploted_V_7_des_y, = plt.plot(0.001*np.arange(0,np.size(V_7_des_y),1), V_7_des_y, "r--", linewidth=2.0)
plt.legend([ploted_V_7_y, ploted_V_7_des_y], ['$\dot{X}_{y}$', '$\dot{X}_{y}^{*}$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_y), -0.5, 0.3])
plt.grid(True)





source = open("saved_data/V_7_z.txt", "r")
V_7_z = source.readlines()
source.close()

source = open("saved_data/V_7_des_z.txt", "r")
V_7_des_z = source.readlines()
source.close()

plt.figure('K_Xdot')
plt.subplot(2, 2, 4)
	
ploted_V_7_z, = plt.plot(0.001*np.arange(0,np.size(V_7_z),1), V_7_z, "b", linewidth=2.0)
ploted_V_7_des_z, = plt.plot(0.001*np.arange(0,np.size(V_7_des_z),1), V_7_des_z, "r--", linewidth=2.0)
plt.legend([ploted_V_7_z, ploted_V_7_des_z], ['$\dot{X}_{z}$', '$\dot{X}_{z}^{*}$'])
	
plt.ylabel("$(m/s)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(V_7_z), -1.1, 1.1])
plt.grid(True)
############################Velocities plot##############################









#############################X plot##############################
source = open("saved_data/trajectory_x.txt", "r")
X_x = source.readlines()
source.close()

source = open("saved_data/des_trajectory_x.txt", "r")
des_X_x = source.readlines()
source.close()

plt.figure('Xerr_X')
plt.subplot(2, 2, 2)
	
ploted_X_x, = plt.plot(0.001*np.arange(0,np.size(X_x),1), X_x, "b", linewidth=2.0)
ploted_Des_X_x, = plt.plot(0.001*np.arange(0,np.size(des_X_x),1), des_X_x, "r--", linewidth=2.0)
plt.legend([ploted_X_x, ploted_Des_X_x], ['$X_x$', '$X_{x}^{*}$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_x), -0.4, 0.85])
plt.grid(True)




source = open("saved_data/trajectory_y.txt", "r")
X_y = source.readlines()
source.close()

source = open("saved_data/des_trajectory_y.txt", "r")
des_X_y = source.readlines()
source.close()

plt.figure('Xerr_X')
plt.subplot(2, 2, 3)
	
ploted_X_y, = plt.plot(0.001*np.arange(0,np.size(X_y),1), X_y, "b", linewidth=2.0)
ploted_Des_X_y, = plt.plot(0.001*np.arange(0,np.size(des_X_y),1), des_X_y, "r--", linewidth=2.0)
plt.legend([ploted_X_y, ploted_Des_X_y], ['$X_{y}$', '$X_{y}^{*}$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_y), 0.25, 0.68])
plt.grid(True)




source = open("saved_data/trajectory_z.txt", "r")
X_z = source.readlines()
source.close()

source = open("saved_data/des_trajectory_z.txt", "r")
des_X_z = source.readlines()
source.close()

plt.figure('Xerr_X')
plt.subplot(2, 2, 4)

print len(np.arange(0,np.size(X_z),1))
print np.size(X_z)
	
ploted_X_z, = plt.plot(0.001*np.arange(0,np.size(X_z),1), X_z,  "b", linewidth=2.0)
ploted_Des_X_z, = plt.plot(0.001*np.arange(0,np.size(des_X_z),1), des_X_z, "r--", linewidth=2.0)
plt.legend([ploted_X_z, ploted_Des_X_z], ['$X_{z}$', '$X_{z}^{*}$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_z), 0.15, 0.85])
plt.grid(True)





source = open("saved_data/X_err.txt", "r")
X_err = source.readlines()
source.close()


plt.figure('Xerr_X')
plt.subplot(2, 2, 1)
	
ploted_X_err, = plt.plot(0.001*np.arange(0,np.size(X_err),1), X_err, "b", linewidth=2.0)

plt.legend([ploted_X_err], ['$||X_{err}||$'])
	
plt.ylabel("$(m)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(X_err), 0.000, 0.003])
plt.grid(True)
#############################X plot##############################



#############################Energy plot plot##############################
source = open("saved_data/Real_Ep_7_X_err_real_x.txt", "r")
Real_Ep_7_X_err_real_x = source.readlines()
source.close()

plt.figure('Energy profile')
plt.subplot(2, 2, 2)
	
ploted_Real_Ep_7_X_err_real_x, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_x),1), Real_Ep_7_X_err_real_x, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_x], ['$E_{p_{profile}}^{x}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_x), -0.01, 0.015])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Real_Ep_7_X_err_real_y.txt", "r")
Real_Ep_7_X_err_real_y = source.readlines()
source.close()

plt.figure('Energy profile')
plt.subplot(2, 2, 3)
	
ploted_Real_Ep_7_X_err_real_y, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_y),1), Real_Ep_7_X_err_real_y, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_y], ['$E_{p_{profile}}^{y}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_y), -0.005, 0.0045])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)





source = open("saved_data/Real_Ep_7_X_err_real_z.txt", "r")
Real_Ep_7_X_err_real_z = source.readlines()
source.close()


plt.figure('Energy profile')
plt.subplot(2, 2, 4)

ploted_Real_Ep_7_X_err_real_z, = plt.plot(0.001*np.arange(0,np.size(Real_Ep_7_X_err_real_z),1), Real_Ep_7_X_err_real_z, "b", linewidth=2.0)
plt.legend([ploted_Real_Ep_7_X_err_real_z], ['$E_{p_{profile}}^{z}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Real_Ep_7_X_err_real_z), -0.015, 0.02])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()

source = open("saved_data/Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst.txt", "r")
Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst = source.readlines()
source.close()

plt.figure('Energy profile')
plt.subplot(2, 2, 1)
	
ploted_E_7_C_ob,                                             = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)
ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst, = plt.plot(0.001*np.arange(0,np.size(Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst),1), Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst, "r--", linewidth=2.0)

plt.legend([ploted_E_7_C_ob, ploted_Real_Ec_7_x_rcnsrcted_wth_small_Ep_7_X_err_real_obst], ['$E_{c_{|k}}^{EE,O}$', '$\sum_{n=1}^{k-1} E_{p_{|n}}^{EE,O}$'])
	
plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(E_7_C_ob), -2.5, 2.5])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
#############################Energy plot plot##############################




















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
plt.axis([0,0.001*np.size(q_0), -5, 3.0])
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
plt.axis([0,0.001*np.size(q_dot_0), -4.5, 3.0])
plt.grid(True)
#############################q_dot plot##############################





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


plt.ylabel("$(rad/s^2)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdot_0), -35, 25])
plt.grid(True)
#############################q_dotdot plot##############################




#torques

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
plt.axis([0,0.001*np.size(tau_0), -150, 50])
plt.grid(True)
#############################tau plot##############################






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
ploted_q_dotdotdot_0_min, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_min),1), q_dotdotdot_0_min, "k-.", linewidth=2.0)



source = open("saved_data/q_dotdotdot_0_max.txt", "r")
q_dotdotdot_0_max = source.readlines()
source.close()
plt.figure('qdotdotdot !!!!')
ploted_q_dotdotdot_0_max, = plt.plot(0.001*np.arange(0,np.size(q_dotdotdot_0_max),1), q_dotdotdot_0_max, "k--", linewidth=2.0)




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


plt.ylabel("$(rad/s^3)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(q_dotdotdot_0), -1150, 750])
plt.grid(True)
#############################q_dotdotdot plot##############################





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
plt.legend([ploted_E_7_C_ob, ploted_E_7_max, ploted_Dist_07_nrst_ob], ['$S_c   (J)$', '$E_{c_{limit}} (J)$', "$d   (m)$"])
#plt.legend([ploted_E_7_C_ob, ploted_E_7_max, ploted_Dist_07_nrst_ob], ['$E_{c_{|k}}^{EE,O} (J)$', '$E_{c_{limit}} (J)$', "$d (m)$"])
#	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Dist_07_nrst_ob), -0.04, 2.6])
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
plt.legend([ploted_Force_sensor_x, ploted_Force_sensor_y, ploted_Force_sensor_z], ['$||F_x||$', '$||F_y||$', '$||F_z||$'])

plt.ylabel("$(N)$", fontsize=20)	
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Force_sensor_x), 0, 1900])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################plot nxt_step Ec, nxt_step Ec_rcnstrctd_with_small_Ep, nxt_step Acc_x##############################







###############################Ep contact (n a de signification que durant le contact) X(tau_final) m_eq ||Xerr||############################################
source = open("saved_data/Ep_x_reconstructed.txt", "r")
Ep_x_reconstructed = source.readlines()
source.close()

plt.figure('Ep contact (n a de signification que durant le contact) X(tau_final) m_eq ||Xerr||')
plt.subplot(3, 1, 1)

ploted_Ep_x_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_x_reconstructed),1), Ep_x_reconstructed, "b", linewidth=2.0)
plt.legend([ploted_Ep_x_reconstructed], ['$E_{p}^{x}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Ep_x_reconstructed), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Ep_y_reconstructed.txt", "r")
Ep_y_reconstructed = source.readlines()
source.close()

plt.figure('Ep contact (n a de signification que durant le contact) X(tau_final) m_eq ||Xerr||')
plt.subplot(3, 1, 2)

ploted_Ep_y_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_y_reconstructed),1), Ep_y_reconstructed, "b", linewidth=2.0)
plt.legend([ploted_Ep_y_reconstructed], ['$E_{p}^{y}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Ep_y_reconstructed), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)




source = open("saved_data/Ep_z_reconstructed.txt", "r")
Ep_z_reconstructed = source.readlines()
source.close()

plt.figure('Ep contact (n a de signification que durant le contact) X(tau_final) m_eq ||Xerr||')
plt.subplot(3, 1, 3)

ploted_Ep_z_reconstructed,    = plt.plot(0.001*np.arange(0,np.size(Ep_z_reconstructed),1), Ep_z_reconstructed, "b", linewidth=2.0)
plt.legend([ploted_Ep_z_reconstructed], ['$E_{p}^{z}$'])

plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Ep_z_reconstructed), -0.03, 0.03])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
###############################Ep contact (n a de signification que durant le contact) X(tau_final) m_eq ||Xerr||############################################









###########################Ec_obst plot222222222##############################
source = open("saved_data/E_7_C_ob.txt", "r")
E_7_C_ob = source.readlines()
source.close()


plt.figure('Ec_obst')

ploted_E_7_C_ob, = plt.plot(0.001*np.arange(0,np.size(E_7_C_ob),1), E_7_C_ob, "b", linewidth=2.0)


plt.legend([ploted_E_7_C_ob], ['$S_c$'])
#plt.legend([ploted_E_7_C_ob, ploted_E_7_max, ploted_Dist_07_nrst_ob], ['$E_{c_{|k}}^{EE,O} (J)$', '$E_{c_{limit}} (J)$', "$d (m)$"])
#	
plt.ylabel("$(J)$", fontsize=20)
plt.xlabel("$time (s)$", fontsize=20)
plt.axis([0,0.001*np.size(Dist_07_nrst_ob), -0.04, 2.6])
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.grid(True)
############################Ec_obst plot222222222##############################






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

