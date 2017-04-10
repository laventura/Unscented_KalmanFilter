#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
    Determine NIS performance
  **/
  float EvaluateNIS(const std::vector<double> &nis_values, 
                  MeasurementPackage::SensorType sensorType);
};

#endif /* TOOLS_H_ */
