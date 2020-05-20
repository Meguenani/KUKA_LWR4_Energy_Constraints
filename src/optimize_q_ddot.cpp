#include <Eigen/Lgsm>
#include <Eigen/Geometry> 
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/LU> 
#include <iostream>
#include "/home/anis/libs/gurobi563/linux64/include/gurobi_c++.h"
#include "save_data_in_txt.h"


#include <Eigen/Lgsm>
#include <Eigen/Geometry> 
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/LU> 
#include <iostream>
#include "/home/anis/libs/gurobi563/linux64/include/gurobi_c++.h"
#include "save_data_in_txt.h"

using std::cout;
using std::endl;
using namespace std;

Eigen::VectorXd optimize_q_ddot(Eigen::Matrix<double, 7, 7> M_inv, Eigen::VectorXd b, Eigen::Matrix<double, 7, 1> tau_min, Eigen::Matrix<double, 7, 1> tau_max){
Eigen::VectorXd q_dotdot_bounds_optimized(14); //Les 7 premiers sont les mins et les 7 suivants sont les max.
int i, status_q_ddot_opt_min, status_q_ddot_opt_max;


/*
std::cout << " " <<std::endl;
std::cout << " " <<std::endl;
std::cout << " :::::::::::::::::::::::::::::::::::::::::::::: " <<std::endl;
std::cout << " M_inv : " <<std::endl;
std::cout <<   M_inv <<std::endl;
std::cout << "  " <<std::endl;
std::cout << " b : " <<std::endl;
std::cout <<   b <<std::endl;
std::cout << " " <<std::endl;
std::cout << " tau_min : " <<std::endl;
std::cout <<   tau_min <<std::endl;
std::cout << " " <<std::endl;
std::cout << " tau_max : " <<std::endl;
std::cout <<   tau_max <<std::endl;
std::cout << " " <<std::endl;
std::cout << " " <<std::endl;
std::cout << " :::::::::::::::::::::::::::::::::::::::::::::: " <<std::endl;
*/

  GRBEnv env_q_ddot         = GRBEnv();

//--------------------------------q_ddoti--------------------------//
  GRBModel model_q_ddot   = GRBModel(env_q_ddot);
  GRBVar* vars            = model_q_ddot.addVars(7, GRB_CONTINUOUS);

  vars[0]                 = model_q_ddot.addVar(tau_min(0, 0), tau_max(0, 0), 0.0, GRB_CONTINUOUS, "tau_0");
  vars[1]                 = model_q_ddot.addVar(tau_min(1, 0), tau_max(1, 0), 0.0, GRB_CONTINUOUS, "tau_1");
  vars[2]                 = model_q_ddot.addVar(tau_min(2, 0), tau_max(2, 0), 0.0, GRB_CONTINUOUS, "tau_2");
  vars[3]                 = model_q_ddot.addVar(tau_min(3, 0), tau_max(3, 0), 0.0, GRB_CONTINUOUS, "tau_3");
  vars[4]                 = model_q_ddot.addVar(tau_min(4, 0), tau_max(4, 0), 0.0, GRB_CONTINUOUS, "tau_4");
  vars[5]                 = model_q_ddot.addVar(tau_min(5, 0), tau_max(5, 0), 0.0, GRB_CONTINUOUS, "tau_5");
  vars[6]                 = model_q_ddot.addVar(tau_min(6, 0), tau_max(6, 0), 0.0, GRB_CONTINUOUS, "tau_6");

  model_q_ddot.update();

  for(i=0;i<7;i++){
  GRBLinExpr obj_lin_q_ddot_min  = M_inv(i, 0)*vars[0] + M_inv(i, 1)*vars[1] + M_inv(i, 2)*vars[2] + M_inv(i, 3)*vars[3] + M_inv(i, 4)*vars[4] + M_inv(i, 5)*vars[5] + M_inv(i, 6)*vars[6] - (M_inv(i, 0)*b[0] + M_inv(i, 1)*b[1] + M_inv(i, 2)*b[2] + M_inv(i, 3)*b[3] + M_inv(i, 4)*b[4] + M_inv(i, 5)*b[5] + M_inv(i, 6)*b[6]);
  model_q_ddot.setObjective(obj_lin_q_ddot_min, GRB_MINIMIZE);
  model_q_ddot.getEnv().set(GRB_IntParam_OutputFlag  , 0);
  model_q_ddot.optimize();
  q_dotdot_bounds_optimized[i]   = M_inv(i, 0)*(vars[0].get(GRB_DoubleAttr_X)-b[0]) + M_inv(i, 1)*(vars[1].get(GRB_DoubleAttr_X)-b[1]) + M_inv(i, 2)*(vars[2].get(GRB_DoubleAttr_X)-b[2]) + M_inv(i, 3)*(vars[3].get(GRB_DoubleAttr_X)-b[3]) + M_inv(i, 4)*(vars[4].get(GRB_DoubleAttr_X)-b[4]) + M_inv(i, 5)*(vars[5].get(GRB_DoubleAttr_X)-b[5]) + M_inv(i, 6)*(vars[6].get(GRB_DoubleAttr_X)-b[6]);
  status_q_ddot_opt_min          = model_q_ddot.get(GRB_IntAttr_Status);
  save_status_q_ddot_opt_min(status_q_ddot_opt_min);

  //Reset model
  model_q_ddot.reset();

  GRBLinExpr obj_lin_q_ddot_max  = M_inv(i, 0)*vars[0] + M_inv(i, 1)*vars[1] + M_inv(i, 2)*vars[2] + M_inv(i, 3)*vars[3] + M_inv(i, 4)*vars[4] + M_inv(i, 5)*vars[5] + M_inv(i, 6)*vars[6] - (M_inv(i, 0)*b[0] + M_inv(i, 1)*b[1] + M_inv(i, 2)*b[2] + M_inv(i, 3)*b[3] + M_inv(i, 4)*b[4] + M_inv(i, 5)*b[5] + M_inv(i, 6)*b[6]);
  model_q_ddot.setObjective(obj_lin_q_ddot_max, GRB_MAXIMIZE);
  model_q_ddot.getEnv().set(GRB_IntParam_OutputFlag  , 0);
  model_q_ddot.optimize();
  q_dotdot_bounds_optimized[i+7] = M_inv(i, 0)*(vars[0].get(GRB_DoubleAttr_X)-b[0]) + M_inv(i, 1)*(vars[1].get(GRB_DoubleAttr_X)-b[1]) + M_inv(i, 2)*(vars[2].get(GRB_DoubleAttr_X)-b[2]) + M_inv(i, 3)*(vars[3].get(GRB_DoubleAttr_X)-b[3]) + M_inv(i, 4)*(vars[4].get(GRB_DoubleAttr_X)-b[4]) + M_inv(i, 5)*(vars[5].get(GRB_DoubleAttr_X)-b[5]) + M_inv(i, 6)*(vars[6].get(GRB_DoubleAttr_X)-b[6]);
  status_q_ddot_opt_max = model_q_ddot.get(GRB_IntAttr_Status);
  save_status_q_ddot_opt_max(status_q_ddot_opt_max);

  model_q_ddot.reset();
  }
//--------------------------------q_ddoti--------------------------//


return q_dotdot_bounds_optimized;
}














/*
Eigen::VectorXd optimize_q_ddot(Eigen::Matrix<double, 7, 7> M, Eigen::VectorXd b, Eigen::Matrix<double, 7, 1> tau_min, Eigen::Matrix<double, 7, 1> tau_max){
Eigen::VectorXd q_dotdot_bounds_optimized(14); //Les 7 premiers sont les mins et les 7 suivants sont les max.
int i, status_q_ddot_opt_min, status_q_ddot_opt_max;

  GRBEnv env_q_ddot     = GRBEnv();
  GRBModel model_q_ddot = GRBModel(env_q_ddot);
  GRBVar* vars   = model_q_ddot.addVars(14, GRB_CONTINUOUS);

  vars[0] = model_q_ddot.addVar(tau_min(0, 0), tau_max(0, 0), 0.0, GRB_CONTINUOUS, "tau");
  vars[1] = model_q_ddot.addVar(tau_min(1, 0), tau_max(1, 0), 0.0, GRB_CONTINUOUS, "tau_1");
  vars[2] = model_q_ddot.addVar(tau_min(2, 0), tau_max(2, 0), 0.0, GRB_CONTINUOUS, "tau_2");
  vars[3] = model_q_ddot.addVar(tau_min(3, 0), tau_max(3, 0), 0.0, GRB_CONTINUOUS, "tau_3");
  vars[4] = model_q_ddot.addVar(tau_min(4, 0), tau_max(4, 0), 0.0, GRB_CONTINUOUS, "tau_4");
  vars[5] = model_q_ddot.addVar(tau_min(5, 0), tau_max(5, 0), 0.0, GRB_CONTINUOUS, "tau_5");
  vars[6] = model_q_ddot.addVar(tau_min(6, 0), tau_max(6, 0), 0.0, GRB_CONTINUOUS, "tau_6");

  vars[7]  = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd");
  vars[8]  = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_1");
  vars[9]  = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_2");
  vars[10] = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_3");
  vars[11] = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_4");
  vars[12] = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_5");
  vars[13] = model_q_ddot.addVar(-1e6, 1e6, 0.0, GRB_CONTINUOUS, "q_dd_6");
  model_q_ddot.update();

  for(i=0;i<7;i++){
  GRBLinExpr obj_lin_q_ddot_i_min = vars[i+7];
  model_q_ddot.setObjective(obj_lin_q_ddot_i_min, GRB_MINIMIZE);  //On minimise q_ddto_i
  model_q_ddot.addConstr(M(i, 0) * vars[7] + M(i, 1) * vars[8] + M(i, 2) * vars[9] + M(i, 3) * vars[10] + M(i, 4) * vars[11] + M(i, 5) * vars[12] + M(i, 6) * vars[13]  + b(i, 0) == vars[i], "ci");
  model_q_ddot.getEnv().set(GRB_IntParam_OutputFlag  , 0);
  model_q_ddot.optimize();
  q_dotdot_bounds_optimized[i] = vars[i+7].get(GRB_DoubleAttr_X);
  status_q_ddot_opt_min = model_q_ddot.get(GRB_IntAttr_Status);
  save_status_q_ddot_opt_min(status_q_ddot_opt_min);


//Reset model
  model_q_ddot.reset();


  GRBLinExpr obj_lin_q_ddot_i_max = vars[i+7];
  model_q_ddot.setObjective(obj_lin_q_ddot_i_max, GRB_MAXIMIZE);  //On maximise q_ddto_i
  model_q_ddot.getEnv().set(GRB_IntParam_OutputFlag  , 0);
  model_q_ddot.optimize();
  q_dotdot_bounds_optimized[i+7] = vars[i+7].get(GRB_DoubleAttr_X);
  status_q_ddot_opt_max = model_q_ddot.get(GRB_IntAttr_Status);
  save_status_q_ddot_opt_max(status_q_ddot_opt_max);

//Reset model and revove constr
  model_q_ddot.reset();
  model_q_ddot.remove(model_q_ddot.getConstrs()[0]);
  }

return q_dotdot_bounds_optimized;
}
*/





