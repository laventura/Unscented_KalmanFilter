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
    
    VectorXd z_out = VectorXd(3);
    MatrixXd S_out = MatrixXd(3, 3);
    ukf.PredictRadarMeasurement(&z_out, &S_out);

	return 0;
}