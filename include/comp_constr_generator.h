#include <iostream>
#include <Eigen/Lgsm>
#include <Eigen/Dense>    
#include <cmath> 

Eigen::VectorXd compute_poly_q_ddot_constr_coefficients(double init_q_dot_u, double target_q_dot_u, double init_q_ddot_u, double target_q_ddot_u, double init_q_dddot_u, double target_q_dddot_u, double init_q_ddddot_u, double target_q_ddddot_u, double T_for_braking_posi_constr_u);
double compute_needed_time_steps_for_braking_posi_constr(double init_q_dot_u, double target_q_dot_u, double q_dotdot_bounds_max_u, double q_dddot_bounds_max_u);
Eigen::VectorXd compute_nxt_step_des_qddot_cnstr_frm_brking_posi_cnstr(double init_q_dot_u, double target_q_dot_u, Eigen::VectorXd a_i_qddot_cnstr_frm_brking_posi_cnstr, double T_for_braking_posi_constr_u, double t_2_for_braking_posi_constr_u);


//double compute_normalization_coeff_s_dot(Eigen::VectorXd a_i_1);
//double compute_normalization_coeff_s_dot_dot(Eigen::VectorXd a_i_1);
//double compute_trig_time_posi(Eigen::VectorXd a_i_1, double T, double current_posi, double nxt_step_des, double init_posi, double target_posi);
//double compute_trig_time_vel(Eigen::VectorXd a_i_1_d, double T, double V, double nxt_step_des_V, double init_posi, double target_posi);

//double compute_trig_s(double prev_posi, double nxt_step_des, double init_posi, double target_posi);

//double compute_trig_s_dot(double T, double prev_V, double nxt_step_des_V, double init_V, double target_V, double init_posi, double target_posi);


//double compute_angle_time(double init_rot_angle, double target_rot_angle, double v_angle_max, double acc_angle_max);
//Eigen::VectorXd compute_nxt_step_des_x(double init_posi_x, double target_posi_x, Eigen::VectorXd a_i_x, double Posi_x_curr ,double T_x, double t_2_x);
//Eigen::VectorXd compute_nxt_step_des_y(double init_posi_y, double target_posi_y, Eigen::VectorXd a_i_y, double Posi_y_curr ,double T_y, double t_2_y);
//Eigen::VectorXd compute_nxt_step_des_z(double init_posi_z, double target_posi_z, Eigen::VectorXd a_i_z, double Posi_z_curr ,double T_z, double t_2_z);
//Eigen::VectorXd compute_nxt_step_des_alpha(double init_orient_alpha, double target_orient_alpha, Eigen::VectorXd a_i_alpha, double T_alpha, double t_1_alpha, double t_2_alpha);
//Eigen::VectorXd compute_nxt_step_des_beta(double init_orient_beta, double target_orient_beta, Eigen::VectorXd a_i_beta, double T_beta, double t_1_beta, double t_2_beta);
//Eigen::VectorXd compute_nxt_step_des_gamma(double init_orient_gamma, double target_orient_gamma, Eigen::VectorXd a_i_gamma, double T_gamma, double t_1_gamma, double t_2_gamma);
//Eigen::VectorXd compute_nxt_step_des_angle(double init_posi_angle, double target_posi_angle, Eigen::VectorXd a_i_angle, double Posi_angle_curr, double nxt_step_des_angle_prev, double T_angle, double t_1_angle, double t_2_angle);
//double compute_quat_cosHalfTheta(Eigen::Displacementd::Rotation3D q_I, Eigen::Displacementd::Rotation3D q_F);
//Eigen::Displacementd::Rotation3D compute_nxt_step_des_quat(Eigen::Displacementd::Rotation3D q_I, Eigen::Displacementd::Rotation3D q_F, Eigen::VectorXd a_i_angle, double cosHalfTheta, double T_angle, double t_1_angle, double t_2_angle);


