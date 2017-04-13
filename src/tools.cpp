#include <iostream>
#include "tools.h"
using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
	rmse << 0,0,0,0;



	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if (estimations.size() < 1)
	{
	    cout << "Invalid est vec" << endl;
	    return rmse;
	}

	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
    if (estimations.size() != ground_truth.size())
	{
	    cout << "Size mismatch" << endl;
	    return rmse;
	}
	//cout << "Size:" << estimations.size() << endl;



	//accumulate squared residuals
	VectorXd acc_err(4);
	acc_err<<0,0,0,0;
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd err  = (estimations[i]-ground_truth[i]);
        //cout<<" err:"<< err<<endl;
         VectorXd se = err.array()*err.array();
        //cout<<" mse:"<< se<<endl;
	    acc_err = acc_err+se;

	}

	//calculate the mean
	// ... your code here
	//cout<<"Acc mse:"<< acc_err<<endl;
	acc_err /= estimations.size();
	//float mse = se/estimations.size();

	//calculate the squared root
	// ... your code here
    rmse = acc_err.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
//recover state parameters
float px = x_state(0);
float py = x_state(1);
float vx = x_state(2);
float vy = x_state(3);

  Hj<<1,1,0,0,
      1,1,0,0,
      1,1,1,1;
  float px_py = px*px + py*py;
  // cout << "den:" << px_py << endl;
//check division by zero
if(fabs(px_py) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

//compute the Jacobian matrix
Hj(0,0) = px/pow(px_py,0.5);
Hj(0,1) = py/pow(px_py,0.5);
Hj(1,0) = -1.0*py/px_py;
Hj(1,1) = px/px_py;
Hj(2,0) = py*(vx*py - vy*px)/pow(px_py,1.5);
Hj(2,1) = px*(vy*px - vx*py)/pow(px_py,1.5);
Hj(2,2) = px/pow(px_py,0.5);
Hj(2,3) = py/pow(px_py,0.5);
return Hj;
}
