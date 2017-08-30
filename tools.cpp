#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

// check the validity of the following inputs:
//  * the estimation vector size should not be zero
//  * the estimation vector size should equal ground truth vector size
if(estimations.size() != ground_truth.size() || estimations.size() == 0)
{
  cout << "Invalid estimation or ground_truth data" << endl;
  return rmse;
 }

//accumulate squared residuals
for(unsigned int i=0; i < estimations.size(); ++i){

  VectorXd residual = estimations[i] - ground_truth[i];

  //coefficient-wise multiplication
  residual = residual.array()*residual.array();
  rmse += residual;
}

//calculate the mean
rmse = rmse/estimations.size();

//calculate the squared root
rmse = rmse.array().sqrt();

//return the result
return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
//recover state parameters
float px = x_state(0);
float py = x_state(1);
float vx = x_state(2);
float vy = x_state(3);

//TODO: YOUR CODE HERE
  float pxy2 = px * px + py * py;
//check division by zero
if (pxy2 < 0.00001)
    {
        pxy2 = 0.00001;
    }
//compute the Jacobian matrix
float pxy2sqrt = sqrt(pxy2);

float H11 = px / pxy2sqrt;
float H12 = py / pxy2sqrt;
float H21 = -py / pxy2;
float H22 = px / pxy2;
float H31 = py * (vx * py - vy * px)/ (pxy2 * pxy2sqrt);
float H32 = px * (vy * px - vx * py)/ (pxy2 * pxy2sqrt);
float H33 = px / pxy2sqrt;
float H34 = py / pxy2sqrt;

Hj << H11, H12, 0, 0,
      H21, H22, 0, 0,
      H31, H32, H33, H34;
  return Hj;
}
