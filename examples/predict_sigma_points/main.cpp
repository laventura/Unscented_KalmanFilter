#include <iostream>
#include <Dense>
#include <vector>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main() {

	//Create a UKF instance
	UKF ukf;

/*******************************************************************************
* Programming assignment calls
*******************************************************************************/

    MatrixXd Xsig_pred = MatrixXd(15, 5);
    ukf.SigmaPointPrediction(&Xsig_pred);

	return 0;
}