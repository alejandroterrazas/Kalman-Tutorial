/** 
 * Write a function 'filter()' that implements a multi-
 *   dimensional Kalman Filter for the example given
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd S;    //used in creation of Kalman Gain??  Why named S?
MatrixXd K;   //Kalman Gain Matrix
MatrixXd Y;  //measurment matrix
MatrixXd C;  //C matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix
MatrixXd B; //B matrix 

void update_prediction(VectorXd &x);
void initial_process_cov_matrix();
void update_predicted_process_cov_matrix();
void calculate_Kalman_gain();
void import_new_observation(int speed, int velocity);
void update_current_state();
void update_process_cov_matrix();

int main() {
  /**
   * Code used as example to work with Eigen matrices
  */

  float dt = 1.;

  // design the KF with 1D motion
  x = VectorXd(2);
  x << 4000, 280;
  P = MatrixXd(2, 2);

  u = VectorXd(1);
  u << 2;

  F = MatrixXd(2, 2);
  F << 1, dt, 0, 1;

//  H = MatrixXd(1, 2);
//  H << 1, 0;
  H = MatrixXd::Identity(2,2);

  R = MatrixXd(2, 2);
  R << 625, 0, 0, 36;

  S = MatrixXd(2,2);
  S << 0, 0, 0, 0;

  K = MatrixXd(2,2);

  C = MatrixXd::Identity(2, 2);

  B = MatrixXd(2,1);
  B << .5*pow(dt,2), dt;

  Y = MatrixXd(2,1);

  I = MatrixXd::Identity(2, 2);

  Q = MatrixXd(2, 2);
  Q << 0, 0, 0, 0;
  
  // create a list of measurements
  VectorXd pos_measurements(4);
  pos_measurements << 4260, 4550, 4860, 5110;

  VectorXd vel_measurements(4);
  vel_measurements << 282, 285, 286, 290;
    
  initial_process_cov_matrix();
  // call Kalman filter algorithm for each measurement (1Hz)

  for (unsigned int n = 0; n < 4; ++n) {
    cout << "********** N: " << n << endl;
    update_prediction(x);
    update_predicted_process_cov_matrix();
    calculate_Kalman_gain();
    import_new_observation(pos_measurements[n], vel_measurements[n]);
    update_current_state();
    update_process_cov_matrix();
  }

  return 0;
}

void initial_process_cov_matrix() {
  cout << "Initial_process_cov_matrix" << endl;
  P << 400, 0, 0, 25;
  cout << P << endl;
}

void update_prediction(VectorXd &x) {
  cout << "update_prediction" << endl;
  x = (F * x) + (B * u);
  cout << x << endl;
}

void update_predicted_process_cov_matrix() {
  cout << "update_predicted_process_cov_matrix" << endl;
  P = (F * P) * F.transpose() + Q;
  P(0,1) = 0;
  P(1,0) = 0;

  cout << "P after update: " << P << endl;

}

void calculate_Kalman_gain() {
    cout << "calculate_Kalman_gain" << endl;
    S = R + (H * (P * H.transpose()));
    K = (P * H.transpose()) * S.inverse();
    cout << "Kalman gain matrix: " << K << endl;
}

void import_new_observation(int speed, int velocity) {
    MatrixXd measurement = MatrixXd(2, 1);
    measurement << speed, velocity;
    cout << "measurement: "<<  measurement  << endl;
    Y  = C * measurement;
    cout << "Y: " << Y << endl;
}

void update_current_state() {
  cout << "x: " << x << endl;
  cout << "H: " << H << endl;
  cout << "Y: " << Y << endl;

  MatrixXd delta = Y - (H * x);
  cout << "delta: " << delta << endl;

  x = x + (K * delta);
  cout << "adjusted x: " << x << endl;
}


void update_process_cov_matrix() {

    MatrixXd I = MatrixXd::Identity(2, 2);
  
    P = P * (I - (K * H)) * (I - (K * H)).transpose() + (K * R) * K.transpose();

   cout << "Updated Process Covariance Matrix: " << P << endl;

}
