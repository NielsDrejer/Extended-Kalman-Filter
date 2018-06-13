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
    
    // TODO: YOUR CODE HERE
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    
    if(estimations.size() != 0 && estimations.size() == ground_truth.size())
    {
        for( unsigned int i=0; i < estimations.size(); ++i ) {
            VectorXd residual = estimations[i] - ground_truth[i];
            
            residual = residual.array() * residual.array();

            rmse += residual;
        }
    
        rmse = rmse / estimations.size();
    
        rmse = rmse.array().sqrt();
    }
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);

    if( x_state.size() != 4 ) {
    	return Hj;
    }

    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    double c1 = px*px + py*py;

    /* Make sure we don't divide by anything close to zero */
    if( fabs(c1) < 0.0001 ) {
      return Hj;
    }

    double c2 = sqrt(c1);
    double c3 = (c1*c2);

    //compute the Jacobian matrix
    Hj(0,0) = (px / c2);
    Hj(0,1) = (py / c2);
    Hj(0,2) = 0;
    Hj(0,3) = 0;
        
    Hj(1,0) = - (py / c1);
    Hj(1,1) = (px / c1);
    Hj(1,2) = 0;
    Hj(1,3) = 0;
        
    Hj(2,0) = (py*(vx*py - vy*px))/c3;
    Hj(2,1) = (px*(vy*px - vx*py))/c3;
    Hj(2,2) = Hj(0,0);
    Hj(2,3) = Hj(0,1);
    
    return Hj;
}
