

# Extended Kalman Filter Project

The goal of this project is to apply the kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

There are three parts to this tutorial.  The first part is written is the jupyter notebook named Kalman-Tutorial.ipynb.  This notebook is based on Dr. Michal van Biezen's multipart Youtube tutorial https://www.youtube.com/user/ilectureonline.  The notebook was meant to mirror his calculations of the basic linear Kalman Filter.  The same model is developed in C++ as `Kalman-Tutorial.cpp`.  The cpp file can be compiled directly in GNU g++ Kalman-Tutoria.cpp -o Kalman-Tutorial and then run with ./Kalman-Tutorial.

The second part os the tutorial builds an Extended Kalman Filter (EKF) that fuses LIDAR and RADAR data supplied in the Udacity Self-driving Car Term 1 project.  The file is named obj_pose-laser-radar-synthetic-input.txt.  Three notebooks are provided for LIDAR, RADAR and Fusion.  These are: a) LIDAR-Only.ipynb, b) RADAR-Only.ipynb and c) RADAR-LIDAR-Combined.ipynb.  

Finally, the third part of this tutorial is a C++ version of the LIDAR and RADAR fusion that works with the Udacity Simulator (see immediately below).  The code is stored in one cpp program main.cpp.  The third part requires compilation using make.  The code was written in C++11 with Make 3.81 as required by Udacity.  A jupyter notebook PlotExtendedKalmanErrors.ipynb is provided that can be used to see the errors over iterations.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `



