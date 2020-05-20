#include <Eigen/Lgsm>
#include <Eigen/Geometry> 
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/LU> 
#include <iostream>
#include "Maximum.h"
#include "Minimum.h"



//Nombre d'itération final pour chaque liaison 
Eigen::VectorXi int_root_neg_jerk(Eigen::Matrix<int, 4, 7> rounded_Quartic_roots_neg_jerk_poly, Eigen::VectorXd q_bounds_min, Eigen::VectorXd q_bounds_max, Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd q_dddot_bounds_min, Eigen::VectorXd q_dddot_bounds_max, double dt){
int i;
int g;
int min_min_q_ddot_index;
int n; //Nombre d'itératio (4 valeurs), il faut en choisir une qui minimise val_min_de_q_ddot, (q_ddot(t) < val_min_de_q_ddot)
Eigen::VectorXi n_minimizing(7); //Le vecteur retourné est constitué de int
Eigen::VectorXd min_q_ddot(4);
int min_min_q_ddot; //index de la valeur min de max_q_ddot
for(g=0; g<7; g++){
for(i=0; i<=3; i++){
n = rounded_Quartic_roots_neg_jerk_poly(i,g);

if(n <=2){
n=2;
}

min_q_ddot[i] = (2*(q_bounds_max[g] - q[g])/((pow(n, 2) - n)*pow(dt, 2))) - ((2*q_dot[g])/((n-1)*dt)) - ((((pow(n,2)/3)-n+(2/3))*q_dddot_bounds_min[g]*dt)/(n-1)); 
if(g==0){
//std::cout << "n neg: "<< n <<std::endl;
//std::cout << "min_q_ddot[i] : "<< min_q_ddot[i]  <<std::endl;
}
}



min_min_q_ddot_index = min_in_vector_index_4(min_q_ddot); //Donne l'index de la valeur de "rounded_Quartic_roots_neg_jerk_poly" qui minimise "min_q_ddot"
if(g==0){
//std::cout << "min_min_q_ddot_index : "<< min_min_q_ddot_index  <<std::endl;
}
if(rounded_Quartic_roots_neg_jerk_poly(min_min_q_ddot_index, g)==0 || rounded_Quartic_roots_neg_jerk_poly(min_min_q_ddot_index, g)==1 || rounded_Quartic_roots_neg_jerk_poly(min_min_q_ddot_index, g)==2){rounded_Quartic_roots_neg_jerk_poly(min_min_q_ddot_index, g)=3;}
n_minimizing[g]=rounded_Quartic_roots_neg_jerk_poly(min_min_q_ddot_index, g); //The value of n that minimizez the "val_min_de_q_ddot"
if(g==0){
//std::cout << "n_minimizing[0] : "<< n_minimizing[g]  <<std::endl;
}
}

return n_minimizing;
}






Eigen::VectorXi int_root_pos_jerk(Eigen::Matrix<int, 4, 7> rounded_Quartic_roots_pos_jerk_poly, Eigen::VectorXd q_bounds_min, Eigen::VectorXd q_bounds_max, Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd q_dddot_bounds_min, Eigen::VectorXd q_dddot_bounds_max, double dt){
int i;
int g;
int max_max_q_ddot_index;
int n; //Nombre d'itération (4 valeurs), il faut en choisir une qui maximize val_min_de_q_ddot, (q_ddot(t) > val_max_de_q_ddot)
Eigen::VectorXi n_maximizing(7);
Eigen::VectorXd max_q_ddot(4);
int max_max_q_ddot; //index de la vameur max de max_q_ddot

for(g=0; g<7; g++){
for(i=0; i<=3; i++){
n = rounded_Quartic_roots_pos_jerk_poly(i,g);

if(n <= 2){
n=2;
}

max_q_ddot[i] = (2*(q_bounds_min[g] - q[g])/((pow(n, 2) - n)*pow(dt, 2))) - ((2*q_dot[g])/((n-1)*dt)) - ((((pow(n,2)/3)-n+(2/3))*q_dddot_bounds_max[g]*dt)/(n-1)); 
if(g==0){
//std::cout << "n pos: "<< n <<std::endl;
//std::cout << "max_q_ddot[i] : "<< max_q_ddot[i]  <<std::endl;
}
}

max_max_q_ddot_index = max_in_vector_index_4(max_q_ddot);
if(g==0){
//std::cout << "max_max_q_ddot_index : "<< max_max_q_ddot_index  <<std::endl;
}
if(rounded_Quartic_roots_pos_jerk_poly(max_max_q_ddot_index, g)==0 || rounded_Quartic_roots_pos_jerk_poly(max_max_q_ddot_index, g)==1 || rounded_Quartic_roots_pos_jerk_poly(max_max_q_ddot_index, g)==2){rounded_Quartic_roots_pos_jerk_poly(max_max_q_ddot_index, g)=3;}
n_maximizing[g]=rounded_Quartic_roots_pos_jerk_poly(max_max_q_ddot_index, g); //The value of n that maximizes the "val_max_de_q_ddot"
if(g==0){
//std::cout << "n_maximizing[0] : "<< n_maximizing[g]  <<std::endl;
}
}

return n_maximizing;
}
