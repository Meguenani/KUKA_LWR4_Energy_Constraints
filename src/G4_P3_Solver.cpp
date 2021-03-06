#include "G4ios.hh"
#include "globals.hh"
#include "G4AnalyticalPolSolver.hh"
#include "geomdefs.hh"

#include <iostream>
#include <Eigen/Lgsm>
#include <Eigen/Dense>    
#include <cmath> 
#include "Absolute.h"
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



Eigen::Matrix<int, 6, 7> G4_P3_Solver(Eigen::Matrix<double, 10, 7> p_){

G4double r[3][5]; //All roots r_[1][k_] stock les parties réelles et r_[2][k_] les parties imaginaires. k_ = nombre de racines (de 0 à 4)
Eigen::VectorXd real_roots(3);

Eigen::Matrix<int, 6, 7> rounded_roots; //Les racines arrondies sont des ints. 8 racines pour chaque liaison. 4 pour jerk_neg et 4 pour jerk_pos. CHaque colonne c'est pour une liaison. les 4 première lignes c'est pour neg_jerk, les 4 suivantes sont pour pos_jerk
real_roots << 0, 0, 0;
G4double p[5];

G4int k_;


for(int y=0;y<7;y++){
//Neg_jerk 

//Chargement
  p[0] = p_(0,y);     //Chaque colonne correspond à une liaison
  p[1] = p_(1,y);     //Chaque colonne correspond à une liaison
  p[2] = p_(2,y);     //Chaque colonne correspond à une liaison
  p[3] = p_(3,y);     //Chaque colonne correspond à une liaison
  p[4] = p_(4,y);     //Chaque colonne correspond à une liaison

//Résolution
G4double x,t,b,c,d;
G4int k;
if( p[0] != 1. )
{
for(k = 1; k < 4; k++ ) { p[k] = p[k]/p[0]; }
p[0] = 1.;
}
x = p[1]/3.0;
t = x*p[1];
b = 0.5*( x*( t/1.5 - p[2] ) + p[3] );
t = ( t - p[2] )/3.0;
c = t*t*t;
d = b*b - c;
if( d >= 0. )
{
d = std::pow( (std::sqrt(d) + std::fabs(b) ), 1.0/3.0 );
if( d != 0. )
{
if( b > 0. ) { b = -d; }
else { b = d; }
c = t/b;
}
d = std::sqrt(0.75)*(b - c);
r[2][2] = d;
b = b + c;
c = -0.5*b-x;
r[1][2] = c;
if( ( b > 0. && x <= 0. ) || ( b < 0. && x > 0. ) )
{
r[1][1] = c;
r[2][1] = -d;
r[1][3] = b - x;
r[2][3] = 0;
}
else
{
r[1][1] = b - x;
r[2][1] = 0.;
r[1][3] = c;
r[2][3] = -d;
}
} // end of 2 equal or complex roots
else
{
if( b == 0. ) { d = std::atan(1.0)/1.5; }
else { d = std::atan( std::sqrt(-d)/std::fabs(b) )/3.0; }
if( b < 0. ) { b = std::sqrt(t)*2.0; }
else { b = -2.0*std::sqrt(t); }
c = std::cos(d)*b;
t = -std::sqrt(0.75)*std::sin(d)*b - 0.5*c;
d = -t - c - x;
c = c - x;
t = t - x;
if( std::fabs(c) > std::fabs(t) ) { r[1][3] = c; }
else
{
r[1][3] = t;
t = c;
}
if( std::fabs(d) > std::fabs(t) ) { r[1][2] = d; }
else
{
r[1][2] = t;
t = d;
}
r[1][1] = t;
for(k = 1; k < 4; k++ ) { r[2][k] = 0.; }
}

//Extraction des racines
 for( k_ = 1; k_ <= 3; k_++ )
  { 
/*
      G4cout<<"k_    = "<<k_<<G4endl;
      G4cout<<"a_1    = "<<a_1<<G4endl;
      G4cout<<"b_1    = "<<b_1<<G4endl;
      G4cout<<"c_1    = "<<c_1<<G4endl;
      G4cout<<"d_1    = "<<d_1<<G4endl;
*/
      //k_ est le nopmbre de racines
      //G4cout<<"root neg_jerk = "<< r[1][k_] << " " << r[2][k_] <<" i_1" << G4endl<<G4endl; //Affiche les racines complètes


      if((double) r[2][k_] == 0){  
      //if((double) r[2][k_] == 0 && (double) r[1][k_] >= 0){  
      real_roots[k_-1] = (double)r[1][k_];
      rounded_roots(k_-1, y) = absolute(round_up(real_roots[k_-1]));   // On remplit colonne par colonne. 
      //rounded_roots(k_-1, y) = absolute(real_roots[k_-1]);   // On remplit colonne par colonne. 
      }    
  }



















//Pos_jerk
//Chargement
  p[0] = p_(5,y);     //Chaque colonne correspond à une liaison
  p[1] = p_(6,y);     //Chaque colonne correspond à une liaison
  p[2] = p_(7,y);     //Chaque colonne correspond à une liaison
  p[3] = p_(8,y);     //Chaque colonne correspond à une liaison
  p[4] = p_(9,y);     //Chaque colonne correspond à une liaison




//Résolution
if( p[0] != 1. )
{
for(k = 1; k < 4; k++ ) { p[k] = p[k]/p[0]; }
p[0] = 1.;
}
x = p[1]/3.0;
t = x*p[1];
b = 0.5*( x*( t/1.5 - p[2] ) + p[3] );
t = ( t - p[2] )/3.0;
c = t*t*t;
d = b*b - c;
if( d >= 0. )
{
d = std::pow( (std::sqrt(d) + std::fabs(b) ), 1.0/3.0 );
if( d != 0. )
{
if( b > 0. ) { b = -d; }
else { b = d; }
c = t/b;
}
d = std::sqrt(0.75)*(b - c);
r[2][2] = d;
b = b + c;
c = -0.5*b-x;
r[1][2] = c;
if( ( b > 0. && x <= 0. ) || ( b < 0. && x > 0. ) )
{
r[1][1] = c;
r[2][1] = -d;
r[1][3] = b - x;
r[2][3] = 0;
}
else
{
r[1][1] = b - x;
r[2][1] = 0.;
r[1][3] = c;
r[2][3] = -d;
}
} // end of 2 equal or complex roots
else
{
if( b == 0. ) { d = std::atan(1.0)/1.5; }
else { d = std::atan( std::sqrt(-d)/std::fabs(b) )/3.0; }
if( b < 0. ) { b = std::sqrt(t)*2.0; }
else { b = -2.0*std::sqrt(t); }
c = std::cos(d)*b;
t = -std::sqrt(0.75)*std::sin(d)*b - 0.5*c;
d = -t - c - x;
c = c - x;
t = t - x;
if( std::fabs(c) > std::fabs(t) ) { r[1][3] = c; }
else
{
r[1][3] = t;
t = c;
}
if( std::fabs(d) > std::fabs(t) ) { r[1][2] = d; }
else
{
r[1][2] = t;
t = d;
}
r[1][1] = t;
for(k = 1; k < 4; k++ ) { r[2][k] = 0.; }
}


//Extraction des racines
  for( k_ = 1; k_ <= 3; k_++ )
  { 
/*
      G4cout<<"k_    = "<<k_<<G4endl;
      G4cout<<"a_1    = "<<a_1<<G4endl;
      G4cout<<"b_1    = "<<b_1<<G4endl;
      G4cout<<"c_1    = "<<c_1<<G4endl;
      G4cout<<"d_1    = "<<d_1<<G4endl;
*/
      //k_ est le nopmbre de racines
      //G4cout<<"root pos_jerk = "<< r[1][k_] << " " << r[2][k_] <<" i_1" << G4endl<<G4endl;


      if((double) r[2][k_] == 0){  
      //if((double) r[2][k_] == 0 && (double) r[1][k_] >= 0){  
      real_roots[k_-1] = (double)r[1][k_];
        rounded_roots(k_+2, y) = absolute(round_up(real_roots[k_-1]));   // On remplit colonne par colonne. 
      //rounded_roots(k_+2, y) = absolute(real_roots[k_-1]);   // On remplit colonne par colonne. 
      }    
  }
}
  return rounded_roots;
}


