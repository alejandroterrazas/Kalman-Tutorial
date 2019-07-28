#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "Eigen/Dense"
#include "measurement_package.h"
#include <fstream>
#include <cstdio>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::istringstream;

#define PI 3.141592653589
#define THRESHOLD 0.0001

// using the following for processing json files
using json = nlohmann::json;


//creates measurment package for LIDAR measurement
MeasurementPackage processLidarRecord(istringstream& iss) {
     //std::cout << "LIDAR measurment" << std::endl;
     MeasurementPackage meas_package; 
     meas_package.sensor_type_ = MeasurementPackage::LASER;
     meas_package.raw_measurements_ = VectorXd(2);
     iss >> meas_package.raw_measurements_(0);
     iss >> meas_package.raw_measurements_(1);
     iss >>  meas_package.timestamp_;

     return meas_package;
}

//creates measurment package for RADAR measurement
MeasurementPackage processRadarRecord(istringstream& iss) {
     //std::cout << "RADAR measurement" << std::endl;
     MeasurementPackage meas_package;
     meas_package.sensor_type_ = MeasurementPackage::RADAR;
     meas_package.raw_measurements_ = VectorXd(3);
     iss >> meas_package.raw_measurements_(0);
     iss >> meas_package.raw_measurements_(1);
     iss >> meas_package.raw_measurements_(2);
     iss >> meas_package.timestamp_;
  
     return meas_package;
}

// processIncomingMeasurement receives a record and screens it.
// called each time a new measurement is recieved from queue

MeasurementPackage processIncomingMeasurement(string record) {
   auto found_null = record.find("null");
   auto b1 = record.find_first_of("[");
   auto b2 = record.find_first_of("]");

   if (found_null == string::npos && b1 != string::npos && b2 != string::npos) {

      auto j = json::parse(record.substr(b1, b2 - b1 + 1));
      string event = j[0].get<string>();

      if (event  == "telemetry") {
          
          MeasurementPackage mp;

          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];
          istringstream iss(sensor_measurement);
          
          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;
         
          //determine is measurement is LIDAR ("L") or RADAR ("R")
          if (sensor_type.compare("L") == 0)
              mp = processLidarRecord(iss);
          else if (sensor_type.compare("R") == 0)
              mp = processRadarRecord(iss);

          //add ground truth values to the measurement pack
          mp.ground_truth_ = VectorXd(4);
          iss >> mp.ground_truth_(0);
          iss >> mp.ground_truth_(1);
          iss >> mp.ground_truth_(2);
          iss >> mp.ground_truth_(3);
          return mp;

       }  //end if telemetry

   } else  //not a telemetry record; throw exception
       throw "Invalid Record";

} //end function

void updateQ(MatrixXd &Q_, double dt, float n_ax, float n_ay) {

    double dt2 = dt*dt;
    double dt3 = dt*dt*dt;
    double dt4 = dt*dt*dt*dt;
   
    Q_ << dt4/(n_ax*4), 0, dt3/(2*n_ax), 0,
               0, dt4/(4*n_ay), 0, dt3*2*n_ay,
               dt3/(2*n_ax), 0, dt2*n_ax, 0,
               0, dt3/(2*n_ay), 0, dt2*n_ay;
}

void updateF(MatrixXd &F_, double dt) {
  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
}

void updateJacobian(MatrixXd &Hj_, const VectorXd& state) {
  float px = state(0);
  float py = state(1);
  float vx = state(2);
  float vy = state(3);

  if (fabs(px) < THRESHOLD and fabs(py) < THRESHOLD)
     px = THRESHOLD, py = THRESHOLD;
     //py = THRESHOLD;

  double c1 = px*px + py*py;
  double c2 = sqrt(c1);
  double c3 = c1*c2;

  Hj_ << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
          py*(vx*px - vy*py)/c3, px*(vy*px - vx*py)/c3, px/c2,  py/c2;
   
}

VectorXd cartesian_2_polar(VectorXd cartesian) {
  float px = cartesian(0);
  float py = cartesian(1);
  float vx = cartesian(2);
  float vy = cartesian(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float rho_dot = 0;
  if (rho > THRESHOLD)  
     rho_dot = (px*vx  + py*vy) / rho;
  
  VectorXd polar = VectorXd(3);
  polar << rho, phi, rho_dot;

  return polar;

}

int main() {

  uWS::Hub h;
 
  //measurement covariance matrices - laser and radar
  MatrixXd R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << 0.0225, 0,
              0, 0.0225;

  MatrixXd R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //measurement ?? laser and radar 
  // radar is the Jacobian and is updated each iteration
  MatrixXd H_lidar_ = MatrixXd(2, 4);
  H_lidar_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  //set with compute Jacobian in tools
  MatrixXd Hj_ = MatrixXd(3,4);

  MatrixXd Q_ = MatrixXd(4,4);
  MatrixXd F_ = MatrixXd(4,4); 

  MatrixXd P_ = MatrixXd(4,4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  //vectors to hold current_state and predicted_state
  VectorXd current_state(4);
  VectorXd predicted_state(4);
    
  bool is_initialized = false; 
  bool write_results = true;
 // h calls a lambda function  
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message, 2 signifies a websocket event
    
  h.onMessage([&current_state, &predicted_state, &is_initialized, &write_results, &Q_, &F_, &P_, &Hj_, &H_lidar_, &R_radar_, &R_lidar_]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {

    float noise_ax = 9.0, noise_ay = 9.0;
    long long current_time, previous_time;
    //define matrices and vectors to be used inside the lambda function
    MatrixXd S, K;
    VectorXd Y; 
    //I is used for both RADAR and LIDAR     
    MatrixXd I = MatrixXd::Identity(4, 4);
    
    std::ofstream fp;
 
    double error_x, error_y, error_vx, error_vy;
  
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

         try {
             MeasurementPackage m_pack = processIncomingMeasurement(string(data));
           
            // check if state has been initialized; if yes, process else, initialize
            if (is_initialized) {
               current_time = m_pack.timestamp_;
               //Times are in tenths of microseconds; divide by 1000000 for secs
               float dt = (current_time - previous_time)/1000000.;
               //if dt is too small (e.g., RADAR and LIDAR arrive near in time, supply a small dt
               //to avoid divide by zero 
               if (dt < THRESHOLD)
                    dt = THRESHOLD;

               //std::cout << "dt: " << dt << std::endl;
               updateQ(Q_, dt, noise_ax, noise_ay);
               updateF(F_, dt);

               //make the prediction here; always the same for LIDAR and RADAR      
               predicted_state = F_ * current_state;
               P_ = F_ * P_ * F_.transpose() + Q_;
             
               if (m_pack.sensor_type_ == MeasurementPackage::RADAR) {

                  //calculate K; requires Hjacobian, and S
                  
                  updateJacobian(Hj_, predicted_state);
                  S = R_radar_ + (Hj_ * (P_ * Hj_.transpose()));
                  K = (P_ * Hj_.transpose()) * S.inverse();

                  VectorXd h_pred  = cartesian_2_polar(predicted_state);

                  VectorXd z(3);
                  z[0] = m_pack.raw_measurements_(0);
                  z[1] = m_pack.raw_measurements_(1);
                  z[2] = m_pack.raw_measurements_(2);

                  //compute the error signal
                  Y = z - h_pred;

                  //ensure that the error phi is between -PI and PI
                  while (fabs(Y[1]) >  PI)
                     Y[1] < 0 ? Y[1] += 2*PI : Y[1] -= 2*PI;        

                  //update the current state with Kalman Gain and error measurement
                  current_state = predicted_state + (K * Y);

                  //update the process covariance matrix
                  P_ = (I - K * Hj_) * P_;
 
               } else if (m_pack.sensor_type_ == MeasurementPackage::LASER) { 
                 
                  S = R_lidar_ + (H_lidar_ * (P_ * H_lidar_.transpose()));
                  K = (P_ * H_lidar_.transpose()) * S.inverse();

                  //LIDAR is  in cartesian coords. Use them directly.
                  //Note: LIDAR only gives position x, y.  No velocity is provided.                
                  VectorXd z(2);
                  z[0] = m_pack.raw_measurements_(0);
                  z[1] = m_pack.raw_measurements_(1);
        
                  //Compute the error signal                     
                  Y = z - (H_lidar_ * predicted_state); 
                  
                  //Update the current state with Kalman weighted
                  current_state = predicted_state + (K * Y);
                
                  //Update the Process Covariance Matrix
                  P_ = (I - K * H_lidar_) * P_;
               }

             } else if (!is_initialized) {  //not initialized yet
               
               //remove results file (ekf.cvs) prior to writing
               if (write_results)
                  int status = remove("./ekf.csv");

               if (m_pack.sensor_type_ == MeasurementPackage::RADAR) {
                   //for RADAR, convert rho and phi to cartesian & initialize current_state  
                   float rho = m_pack.raw_measurements_(0);
                   float  phi = m_pack.raw_measurements_(1);
                   current_state[0] = rho * cos(phi);
                   current_state[1] = rho * sin(phi);
               } else if (m_pack.sensor_type_ == MeasurementPackage::LASER) {
                  //for LIDAR, no conversion need.  Add directly to current_state
                   current_state[0] = m_pack.raw_measurements_(0);
                   current_state[1] = m_pack.raw_measurements_(1);
               }

               //At initialization velocity is not known.  Set to zero
               current_state[2] =  0.;
               current_state[3] =  0.;

               //initialize current time
               current_time = m_pack.timestamp_;
               is_initialized = true;
             }
          
             //check the current state is not exacly zero
             if (fabs(current_state[0]) < THRESHOLD and fabs(current_state[1]) < THRESHOLD)
                  current_state[0] = THRESHOLD, current_state[1] = THRESHOLD;
            
             previous_time = current_time;
         
             //ground_truth.push_back(gt_values);
             error_x = sqrt(pow((current_state[0] - m_pack.ground_truth_(0)),2));
             error_y = sqrt(pow((current_state[1] - m_pack.ground_truth_(1)),2));
             error_vx = sqrt(pow((current_state[2] - m_pack.ground_truth_(2)),2));
             error_vy = sqrt(pow((current_state[3] - m_pack.ground_truth_(3)), 2));
 
             json msgJson;
             msgJson["estimate_x"] = current_state[0]; 
             msgJson["estimate_y"] = current_state[1]; 
             msgJson["rmse_x"] =  error_x; 
             msgJson["rmse_y"] = error_y; 
             msgJson["rmse_vx"] = error_vx; 
             msgJson["rmse_vy"] = error_vy;
 
            // uncomment to pause the program on each iteratoin
            //system("read");

            // set write_output == true to write a file for analysis
            if (write_results) { 
               fp.open("./ekf.csv", std::ios::app);
               fp << current_state[0] << "," << current_state[1] << ",";
               fp << error_x << "," << error_y << "," << error_vx << "," << error_vy << ",";
               fp << std::endl;
               fp.close();
            }
                   
            auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
        catch( const char* e ) {
            string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    
   }  // end if length  websocket message if

  }); // end h.onMessage


  //
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}  //end main


