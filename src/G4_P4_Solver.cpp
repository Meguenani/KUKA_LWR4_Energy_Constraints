#include "G4ios.hh"
#include "globals.hh"
#include "G4AnalyticalPolSolver.hh"
#include "geomdefs.hh"
#include "Absolute.h"
#include <iostream>
#include <Eigen/Lgsm>
#include <Eigen/Dense>    
#include <cmath> 

#include  <complex>
#include "G4_P4_ApproxEqual.h"
#include "round_up.h"  //Sert à arrondir les double en int (supérieur)

// #include "ApproxEqual.hh"

// const G4double kApproxEqualTolerance = 1E-2;

// Return true if the double check is approximately equal to target
//
// Process:
//
// Return true is check if less than kApproxEqualTolerance from target



Eigen::Matrix<int, 8, 7> G4_P4_Solver(Eigen::Matrix<double, 10, 7> p_){
G4double r[3][5]; //All roots r_[1][k_] stock les parties réelles et r_[2][k_] les parties imaginaires. k_ = nombre de racines (de 0 à 4)
Eigen::VectorXd real_roots(4);
Eigen::Matrix<int, 8, 7> rounded_roots; //Les racines arrondies sont des ints. 8 racines pour chaque liaison. 4 pour jerk_neg et 4 pour jerk_pos. CHaque colonne c'est pour une liaison. les 4 première lignes c'est pour neg_jerk, les 4 suivantes sont pour pos_jerk
real_roots << 0, 0, 0, 0;

/************************************Quartic Equation solver************************************/
  G4int i_1, k_, n_1, iRoot, iMax = 10000;
  G4int iCheck = iMax/10;
  G4double r_[3][5];
  G4double poly[5];
  G4double a_1, b_1, c_1, d_1, tmp, range = 10*mm;
  G4AnalyticalPolSolver solver;
  enum Eroot {k_2, k_3, k_4};
  Eroot useCase = k_4;

  G4cout.precision(20);

/*
  a_1    = 14.511252641677856;
  b_1    = 14.7648024559021;
  c_1    = 14.82865571975708;
  d_1    = 14.437621831893921;

std::cout<<"TES SOLVER !!"<<std::endl;


  p_[0] =  1.;
  p_[1] = -a_1 - b_1 - c_1 - d_1;
  p_[2] =  (a_1+b_1)*(c_1+d_1) + a_1*b_1 + c_1*d_1;
  p_[3] = -(a_1+b_1)*c_1*d_1 - a_1*b_1*(c_1+d_1);
  p_[4] =  a_1*b_1*c_1*d_1;
*/

//P[0] x^4 + P[1] x^3 + P[2] x^2 + P[3] * x + P[4]
/*
  p_[0] = -15;
  p_[1] =  5;
  p_[2] =  2;
  p_[3] =  4;
  p_[4] =  2;
*/
/*
  p_[0] = -(q_dddot_bounds_min * pow(dt, 6))/3;
  p_[1] =  (q_dddot_bounds_min * pow(dt, 6));
  p_[2] =  (2*q_dot*pow(dt, 4)) - (q_dddot_bounds_min * pow(dt, 6))/3;
  p_[3] =  (4*pow(dt, 2)*(q_bounds_max-q));
  p_[4] =  -2*pow(dt, 2)*(q_bounds_max-q);
*/

  // iRoot = solver.BiquadRoots(p_,r_);

for(int y=0;y<7;y++){
//Neg_jerk 
  poly[0] = p_(0,y);     //Chaque colonne correspond à une liaison
  poly[1] = p_(1,y);     //Chaque colonne correspond à une liaison
  poly[2] = p_(2,y);     //Chaque colonne correspond à une liaison
  poly[3] = p_(3,y);     //Chaque colonne correspond à une liaison
  poly[4] = p_(4,y);     //Chaque colonne correspond à une liaison

  iRoot = solver.QuarticRoots(poly,r_);

  for( k_ = 1; k_ <= 4; k_++ )
  { 
/*
      G4cout<<"k_    = "<<k_<<G4endl;
      G4cout<<"a_1    = "<<a_1<<G4endl;
      G4cout<<"b_1    = "<<b_1<<G4endl;
      G4cout<<"c_1    = "<<c_1<<G4endl;
      G4cout<<"d_1    = "<<d_1<<G4endl;
*/
      //k_ est le nopmbre de racines
      //G4cout<<"root neg_jerk = "<< r_[1][k_] << " " << r_[2][k_] <<" i_1" << G4endl<<G4endl; //Affiche les racines complètes


      //if((double) r_[2][k_] == 0 && (double) r_[1][k_] >= 0){ 
      if((double) r_[2][k_] == 0){   
      real_roots[k_-1] = (double)r_[1][k_];
      //rounded_roots(k_-1, y) = round_up(real_roots[k_-1]);   // On remplit colonne par colonne. 
      //rounded_roots(k_-1, y) = absolute(round_up(real_roots[k_-1]));   // On remplit colonne par colonne. 
      rounded_roots(k_-1, y) = round_up(absolute(real_roots[k_-1]));   // On remplit colonne par colonne. 
      }    
  }


//Pos_jerk
  poly[0] = p_(5,y);     //Chaque colonne correspond à une liaison
  poly[1] = p_(6,y);     //Chaque colonne correspond à une liaison
  poly[2] = p_(7,y);     //Chaque colonne correspond à une liaison
  poly[3] = p_(8,y);     //Chaque colonne correspond à une liaison
  poly[4] = p_(9,y);     //Chaque colonne correspond à une liaison
  iRoot = solver.QuarticRoots(poly,r_);

  for( k_ = 1; k_ <= 4; k_++ )
  { 
/*
      G4cout<<"k_    = "<<k_<<G4endl;
      G4cout<<"a_1    = "<<a_1<<G4endl;
      G4cout<<"b_1    = "<<b_1<<G4endl;
      G4cout<<"c_1    = "<<c_1<<G4endl;
      G4cout<<"d_1    = "<<d_1<<G4endl;
*/
      //k_ est le nopmbre de racines
      //G4cout<<"root pos_jerk = "<< r_[1][k_] << " " << r_[2][k_] <<" i_1" << G4endl<<G4endl;


      //if((double) r_[2][k_] == 0 && (double) r_[1][k_] >= 0){  
      if((double) r_[2][k_] == 0){  
      real_roots[k_-1] = (double)r_[1][k_];
      //rounded_roots(k_+3, y) = round_up(real_roots[k_-1]);   // On remplit colonne par colonne. 
      //rounded_roots(k_+3, y) = absolute(round_up(real_roots[k_-1]));   // On remplit colonne par colonne. 
      rounded_roots(k_+3, y) = round_up(absolute(real_roots[k_-1]));   // On remplit colonne par colonne. 
      }    
  }

}



/*
  for ( n_1 = 2; n_1 <= 4; n_1++ )
  {
    // Various test cases 

    if( n_1 == 4 )   // roots: 1,2,3,4 
    { 
      p_[0] =  1.; 
      p_[1] = -10.; 
      p_[2] =  35.; 
      p_[3] = -50.; 
      p_[4] =  24.; 

      p_[0] =  1.; 
      p_[1] =  0.; 
      p_[2] =  4.; 
      p_[3] =  0.; 
      p_[4] =  4.; // roots: +-i_1 sqrt(2) 
    }
    if( n_1 == 3 )  // roots:  1,2,3 
    { 
      p_[0] =  1.; 
      p_[1] = -6.; 
      p_[2] = 11.; 
      p_[3] = -6.; 
    }
    if(n_1==2)  // roots : 1 +- i_1 
    { 
      p_[0] =  1.; 
      p_[1] = -2.; 
      p_[2] =  2.;
    }

    if( n_1 == 2 )
    {
      // G4cout<<"Test QuadRoots(p_,r_):"<<G4endl;    
      i_1 = solver.QuadRoots(p_,r_);
    }
    else if( n_1 == 3 )
    {
      // G4cout<<"Test CUBICROOTS(p_,r_):"<<G4endl;    
      i_1 = solver.CubicRoots(p_,r_);
    }
    else if( n_1 == 4 )
    {
      // G4cout<<"Test BIQUADROOTS(p_,r_):"<<G4endl;    
      i_1 = solver.BiquadRoots(p_,r_);
    }

    for( k_ = 1; k_ <= n_1; k_++ )
    { 
      // G4cout << r_[1][k_] << " " << r_[2][k_] <<" i_1" << G4endl;
    }
  }

  G4cout << G4endl << G4endl;
*/
/************************************Quartic Equation solver************************************/

  return rounded_roots;
}


