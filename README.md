# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This is my submission for Term2 Project 1 of Udacity's Self Driving Car Nanodegree.

In this project I utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

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

## Code Style

I have tried to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric Points

Here I address the project Rubric points.

1. Compiling:

Code can be compiled using cmake and make as per instructions in the above section "Basic Build Instructions".

2. Accuracy:

Here are the RMSE values observed for Dataset 1.

(0.0973178	0.0854597	0.451267	0.439935)

These are below the values specified in the Rubric (0.11, 0.11, 0.52, 0.52).

3. Follows the Correct Algorithm:

The code follows the algorithm discussed in the class lessons.

4. Code Efficiency:

Code has minimal changes and computations necessary to perform the prediction and update using Kalman filter.
There are no unnecessary computations however code is not optimized for performance either.

5. Additional information:

I have added the following pre-processor macros to control the code flow.

  1. ./src/kalman_filter.cpp:52:#define DISABLE_RADAR 0
  
This can be used to disable the accounting of RADAR measurements to see how system performs with LASER data alone.

  2. ./src/kalman_filter.cpp:32:#define DISABLE_LASER 0
  
This can be used to disable the accounting of LASER measurements to see how system performs with RADAR data alone.


Here is the table of RMSE values observed with different combinations.

| Sensor        | Dataset1 RMSE           | Dataset2 RMSE           |
| ------------- |:-----------------------:| -----------------------:|
| LASER         | 0.147	0.115	0.638	0.534 | 0.116	0.128	0.587	0.699 |
| RADAR         | 0.230	0.346	0.583	0.804 | 0.274	0.387	0.722	0.926 |
| LASAR+RADAR   | 0.097	0.085	0.451	0.439 | 0.072	0.096	0.414	0.527 |

  3. ./src/tools.h:11:#define VERBOSE_PRINTS 1
  
This can be used to enable/disable the verbose prints.

Here is a sample output on the console when prints are enabled.

--------
P = 

0.00706985 0.00226997  0.0184169 0.00695398

0.00226997 0.00511771 0.00826626  0.0114158

0.0184169 0.00826626   0.120551  0.0371732

0.00695398  0.0114158  0.0371732  0.0888411

Estimation:   -7.23246	10.8959	5.19534	0.0613838	

Ground truth: -7.23983	10.9063	5.19994	0.00179692	

RMSE:         0.0973178	0.0854597	0.451267	0.439935

--------

Thank you for the review.
