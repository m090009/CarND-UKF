#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

void Tools::Init(){
  dataFile.open("../data/data.txt", ios::out);
  if( !dataFile.is_open() )
  {
    cout << "Error opening data.json file" << endl;
    exit(1);
  }
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	// Init Variables 
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Check Estimation size
  // Estimation Vector check
  if (estimations.size() == 0) {
    cout << "CalculateRMSE: size Zero" << endl; 
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE: size mismatch" << endl; 
    return rmse;
  }
//   if (estimations.size() > 0 && estimations.size() == ground_truth.size()) {
  // Sum all squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    // ... your code here
    
    VectorXd residual = estimations[i] - ground_truth[i];
    // Squared Residual
    VectorXd residual_squared = pow(residual.array(), 2);
    // Adding it to the total rmse
    rmse += residual_squared;
  }   

  //calculate the mean 1/n
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt(); 




  //return the result
  return rmse;
}