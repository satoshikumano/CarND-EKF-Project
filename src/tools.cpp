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
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	if (estimations.size() == 0) {
	    cout << "Empty vector." << endl;
	    return rmse;
	}
	if (estimations.size() != ground_truth.size()) {
	    cout << "Size is different."  << endl;
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i) {
        VectorXd res = estimations[i] - ground_truth[i];
        rmse(0) += pow(res(0),2);
        rmse(1) += pow(res(1),2);
        rmse(2) += pow(res(2),2);
        rmse(3) += pow(res(3),2);
	}

	//calculate the mean
	// ... your code here
	rmse = rmse / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse(0) = sqrt(rmse(0));
	rmse(1) = sqrt(rmse(1));
	rmse(2) = sqrt(rmse(2));
	rmse(3) = sqrt(rmse(3));

  cout << "RMSE: " << rmse << endl;
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
