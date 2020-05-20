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
/*
double X_ddot_max;
double X_ddot_min;
int init = 0;


double optimize_X_ddot(Eigen::Matrix<double, 7, 1> tau_min, Eigen::Matrix<double, 7, 1> tau_max, double m_eq_7_j, Eigen::Matrix<double, 1, 7> J_70_C_proj, double Eta_proj_C){
double X_ddot_optimized;
double K_optimized; //Les 7 premiers sont les mins et les 7 suivants sont les max.
Eigen::VectorXd tau_braking_optimized(7);
int i, status_K_max;

if(init == 0){
X_ddot_max =  50000000;
X_ddot_min = -50000000;
init = 1;
}


try {
std::cout<<" AAAAAAA3 " << std::endl;
  GRBEnv env_X_ddot         = GRBEnv();
std::cout<<" AAAAAAA31 " << std::endl;
//--------------------------------q_ddoti--------------------------//
  GRBModel model_X_ddot   = GRBModel(env_X_ddot);
std::cout<<" AAAAAAA32 " << std::endl;
  GRBVar* vars            = model_X_ddot.addVars(8, GRB_CONTINUOUS);
std::cout<<" AAAAAAA33 " << std::endl;
  vars[0]                 = model_X_ddot.addVar(X_ddot_min, X_ddot_max, 0.0, GRB_CONTINUOUS, "X_ddot");
  vars[1]                 = model_X_ddot.addVar(tau_min[0], tau_max[0], 0.0, GRB_CONTINUOUS, "tau_0_braking");
  vars[2]                 = model_X_ddot.addVar(tau_min[1], tau_max[1], 0.0, GRB_CONTINUOUS, "tau_1_braking");
  vars[3]                 = model_X_ddot.addVar(tau_min[2], tau_max[2], 0.0, GRB_CONTINUOUS, "tau_2_braking");
  vars[4]                 = model_X_ddot.addVar(tau_min[3], tau_max[3], 0.0, GRB_CONTINUOUS, "tau_3_braking");
  vars[5]                 = model_X_ddot.addVar(tau_min[4], tau_max[4], 0.0, GRB_CONTINUOUS, "tau_4_braking");
  vars[6]                 = model_X_ddot.addVar(tau_min[5], tau_max[5], 0.0, GRB_CONTINUOUS, "tau_5_braking");
  vars[7]                 = model_X_ddot.addVar(tau_min[6], tau_max[6], 0.0, GRB_CONTINUOUS, "tau_6_braking");
std::cout<<" AAAAAAA34 " << std::endl;
  model_X_ddot.update();
std::cout<<" AAAAAAA35 " << std::endl;
  GRBQuadExpr obj_Quad = 0;
  obj_Quad = pow(m_eq_7_j, 2)*(vars[0]*vars[0]);
std::cout<<" AAAAAAA36 " << std::endl;
  GRBLinExpr obj_lin = 0; 
  obj_lin = (2*m_eq_7_j*Eta_proj_C)*vars[0];           
 std::cout<<" AAAAAAA37 " << std::endl;
  double constante = pow(Eta_proj_C, 2);
  std::cout<<" AAAAAAA38 " << std::endl;
  model_X_ddot.setObjective(obj_Quad+obj_lin+constante, GRB_MINIMIZE);
std::cout<<" AAAAAAA4 " << std::endl;
  model_X_ddot.addConstr((J_70_C_proj[0]*m_eq_7_j)*vars[0] + (J_70_C_proj[0]*Eta_proj_C) == vars[1], "c10");
  model_X_ddot.addConstr((J_70_C_proj[1]*m_eq_7_j)*vars[0] + (J_70_C_proj[1]*Eta_proj_C) == vars[2], "c11");
  model_X_ddot.addConstr((J_70_C_proj[2]*m_eq_7_j)*vars[0] + (J_70_C_proj[2]*Eta_proj_C) == vars[3], "c12");
  model_X_ddot.addConstr((J_70_C_proj[3]*m_eq_7_j)*vars[0] + (J_70_C_proj[3]*Eta_proj_C) == vars[4], "c13");
  model_X_ddot.addConstr((J_70_C_proj[4]*m_eq_7_j)*vars[0] + (J_70_C_proj[4]*Eta_proj_C) == vars[5], "c14");
  model_X_ddot.addConstr((J_70_C_proj[5]*m_eq_7_j)*vars[0] + (J_70_C_proj[5]*Eta_proj_C) == vars[6], "c15");
  model_X_ddot.addConstr((J_70_C_proj[6]*m_eq_7_j)*vars[0] + (J_70_C_proj[6]*Eta_proj_C) == vars[7], "c16");
std::cout<<" AAAAAAA5 " << std::endl;
  //model_X_ddot.getEnv().set(GRB_IntParam_OutputFlag  , 0);
std::cout<<" AAAAAAA6 " << std::endl;
std::cout<<"pow(m_eq_7_j, 2) : "<<pow(m_eq_7_j, 2)<<std::endl;
std::cout<<"Eta_proj_C : "<<Eta_proj_C<<std::endl;
  model_X_ddot.optimize();
std::cout<<" AAAAAAA7 " << std::endl;
  X_ddot_optimized      = vars[0].get(GRB_DoubleAttr_X);
  K_optimized           = (m_eq_7_j*X_ddot_optimized) + Eta_proj_C;
std::cout<<" AAAAAAA8 " << std::endl;
  tau_braking_optimized << vars[1].get(GRB_DoubleAttr_X), vars[2].get(GRB_DoubleAttr_X), vars[3].get(GRB_DoubleAttr_X), vars[4].get(GRB_DoubleAttr_X), vars[5].get(GRB_DoubleAttr_X), vars[6].get(GRB_DoubleAttr_X), vars[7].get(GRB_DoubleAttr_X);
std::cout<<" AAAAAAA9 " << std::endl;
  status_K_max  = model_X_ddot.get(GRB_IntAttr_Status);
  save_status_K_max(status_K_max);

std::cout<<" "<<std::endl;
std::cout<<" "<<std::endl;
std::cout<<" "<<std::endl;
std::cout<<"K_optimized : "<<K_optimized<<std::endl;
std::cout<<"X_ddot_optimized : "<<X_ddot_optimized<<std::endl;
std::cout<<"m_eq_7_j : "<<m_eq_7_j<<std::endl;
std::cout<<"Eta_proj_C : "<<Eta_proj_C<<std::endl;


  //Reset model
    model_X_ddot.reset();

//--------------------------------q_ddoti--------------------------//


return K_optimized;
}


    catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
    } 


    catch(...) {
    cout << "Exception during optimization" << endl;
    }

}
*/























double K_max;
double K_min;
int    init = 0;

double optimize_K(Eigen::Matrix<double, 7, 1> tau_min, Eigen::Matrix<double, 7, 1> tau_max, double m_eq_7_j, Eigen::Matrix<double, 1, 7> J_70_C_proj, double Eta_proj_C){
double K_optimized;
Eigen::VectorXd tau_braking_optimized(7);
int i, status_K_max;

if(init == 0){
K_max =  50000000;
K_min = -50000000;
init = 1;
}


try {

  GRBEnv env_K         = GRBEnv();

//--------------------------------q_ddoti--------------------------//
  GRBModel model_K   = GRBModel(env_K);

  GRBVar* vars            = model_K.addVars(8, GRB_CONTINUOUS);

  vars[0]                 = model_K.addVar(K_min, K_max, 0.0, GRB_CONTINUOUS, "K");
  vars[1]                 = model_K.addVar(tau_min[0], tau_max[0], 0.0, GRB_CONTINUOUS, "tau_0_braking");
  vars[2]                 = model_K.addVar(tau_min[1], tau_max[1], 0.0, GRB_CONTINUOUS, "tau_1_braking");
  vars[3]                 = model_K.addVar(tau_min[2], tau_max[2], 0.0, GRB_CONTINUOUS, "tau_2_braking");
  vars[4]                 = model_K.addVar(tau_min[3], tau_max[3], 0.0, GRB_CONTINUOUS, "tau_3_braking");
  vars[5]                 = model_K.addVar(tau_min[4], tau_max[4], 0.0, GRB_CONTINUOUS, "tau_4_braking");
  vars[6]                 = model_K.addVar(tau_min[5], tau_max[5], 0.0, GRB_CONTINUOUS, "tau_5_braking");
  vars[7]                 = model_K.addVar(tau_min[6], tau_max[6], 0.0, GRB_CONTINUOUS, "tau_6_braking");

  model_K.update();

  GRBQuadExpr obj_Quad = 0;
  obj_Quad = (vars[0]*vars[0]);          

  model_K.setObjective(obj_Quad, GRB_MAXIMIZE);

  model_K.addConstr(J_70_C_proj[0]*vars[0]  == vars[1], "c10");
  model_K.addConstr(J_70_C_proj[1]*vars[0]  == vars[2], "c11");
  model_K.addConstr(J_70_C_proj[2]*vars[0]  == vars[3], "c12");
  model_K.addConstr(J_70_C_proj[3]*vars[0]  == vars[4], "c13");
  model_K.addConstr(J_70_C_proj[4]*vars[0]  == vars[5], "c14");
  model_K.addConstr(J_70_C_proj[5]*vars[0]  == vars[6], "c15");
  model_K.addConstr(J_70_C_proj[6]*vars[0]  == vars[7], "c16");

  model_K.getEnv().set(GRB_IntParam_OutputFlag  , 0);

  model_K.optimize();

  K_optimized      = vars[0].get(GRB_DoubleAttr_X);

  tau_braking_optimized << vars[1].get(GRB_DoubleAttr_X), vars[2].get(GRB_DoubleAttr_X), vars[3].get(GRB_DoubleAttr_X), vars[4].get(GRB_DoubleAttr_X), vars[5].get(GRB_DoubleAttr_X), vars[6].get(GRB_DoubleAttr_X), vars[7].get(GRB_DoubleAttr_X);

  status_K_max  = model_K.get(GRB_IntAttr_Status);
  save_status_K_max(status_K_max);

//std::cout<<" "<<std::endl;
//std::cout<<"tau_braking_optimized : "<<std::endl;
//std::cout<< tau_braking_optimized <<std::endl;

  //Reset model
    model_K.reset();

//--------------------------------q_ddoti--------------------------//


return K_optimized;
}


    catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
    } 


    catch(...) {
    cout << "Exception during optimization" << endl;
    }

}









