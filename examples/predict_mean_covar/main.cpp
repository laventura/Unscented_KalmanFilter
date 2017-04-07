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
    
    VectorXd x_pred = VectorXd(5);
    MatrixXd P_pred = MatrixXd(5, 5);
    ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

	return 0;
}

/** Expected result 

 x =
5.93637

1.49035

2.20528

0.536853

0.353577

expected result p:
 P =
0.00543425 -0.0024053 0.00341576 -0.00348196 -0.00299378

-0.0024053 0.010845 0.0014923 0.00980182 0.00791091

0.00341576 0.0014923 0.00580129 0.000778632 0.000792973

-0.00348196 0.00980182 0.000778632 0.0119238 0.0112491

**/