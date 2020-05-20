

double compute_distance(Eigen::Displacementd point_1 , Eigen::Displacementd point_2);
double dist_to_nrst_ob(double distance_1, double distance_2, double distance_3, double distance_4);
char *id_nrst_ob(double distance_1, double distance_2, double distance_3, double distance_4);
Eigen::Vector3d compute_vect_dist_nrst_obst(char* nrst_ob_id, Eigen::Displacementd point_1, Eigen::Displacementd point_2, Eigen::Displacementd point_3, Eigen::Displacementd point_4, 
					                      Eigen::Displacementd point_5, Eigen::Displacementd point_6, Eigen::Displacementd point_7, Eigen::Displacementd point_8);

Eigen::Vector3d compute_vect_dist_nrst_obst_considered(Eigen::Displacementd point_1, Eigen::Displacementd point_2);
double compute_posi_Err(Eigen::Displacementd H_7, double circle_traj_x, double circle_traj_y, double circle_traj_z);

