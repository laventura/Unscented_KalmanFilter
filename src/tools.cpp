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
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0) 
  {
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i) 
  {
		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse    += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

float Tools::EvaluateNIS(const std::vector<double> &nis_values, 
                  MeasurementPackage::SensorType sensorType)
{
  const float nis_radar95   = 7.815;
  const float nis_lidar95   = 5.991;

  float limit95 = 0.0;

  if (sensorType == MeasurementPackage::RADAR) {
    limit95 = nis_radar95;
  } else if (sensorType == MeasurementPackage::LASER) {
    limit95 = nis_lidar95;
  }

  int above95 = 0;
  for (int k = 0; k < nis_values.size(); k++) {
    if (nis_values[k] > limit95) {
      above95++;
    }
  }

  return (above95 / (float)nis_values.size());
}
