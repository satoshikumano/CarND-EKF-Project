#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
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
	float normSqrt = pow(px, 2) + pow(py, 2);
	float norm = sqrt(normSqrt);

	//check division by zero
	if (normSqrt == 0) {
	    cout << "Can not compute Jacobian." << endl;
	    return Hj;
	}
	//compute the Jacobian matrix
	Hj(0,0) = px / norm;
	Hj(0,1) = py / norm;
	Hj(0,2) = 0;
	Hj(0,3) = 0;
	Hj(1,0) = - (py / normSqrt);
	Hj(1,1) = px / normSqrt;
	Hj(1,2) = 0;
	Hj(1,3) = 0;
	Hj(2,0) = py * (vx*py - vy*px) / pow(norm, 3);
	Hj(2,1) = px * (vy*px - vx*py) / pow(norm, 3);
	Hj(2,2) = px / norm;
	Hj(2,3) = py / norm;

	return Hj;
}
