## Udacity Self-Driving Car Engineer Nanodegree, Term 2, Project 1: Extended Kalman Filters

---

The goal of this project is to implement an extended Kalman Filter in C++, which fusions Lidar and Radar Measurements to track a vehicle. Noisy Radar and Lidar measurements are provided, as well as a Simulator App which can visualize the measurements and the results of the Extended Kalman Filter calculations. Seed code for the project is provided here:

https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

[//]: # (Image References)
[image1]: ./writeup-images/Accuracy1.png
[image2]: ./writeup-images/Accuracy1-total.png
[image3]: ./writeup-images/Accuracy2.png

## Notes about Installation

I am using a Mac for development, so I tried to use the install-mac.sh script for installing the required software.

The install-mac.sh script however did not install the uWebSockets correctly. I executed the relevant parts of the script manually on the command line instead, which in fact turns out to be all lines except the first one. I also had to  modify the line in the script that calls cmake, as the original line did not set the openssl root and libraries directories correctly. The modified install-mac.sh script is included in my submitted files.

Furthermore I had to modify the generated CMakeLists.txt and instead use the following line to enable make to find the linker:

link_directories(/usr/local/Cellar/libuv/1.20.3/lib)

The original line referred to the folder /usr/local/Cellar/libuv/1*/lib, and that didn't work.

After that everything worked as expected. I used the Sublime text editor and make on the command line to build.

## Notes about Implementation

I used the provided file structure and basically filled in the missing bits. My changes can thus be summarized as:

1. Completed tools.cpp, including the 2 functions that calculate root mean squared error (RMSE) and the Jacobian matrix.

2. Completed FusionEKF.cpp, including initializing the Kalman Filter, preparing the Q and F matrices for the prediction step, and calling the radar and lidar update functions.

3. Completed kalman_filter.cpp, including the Predict(), Update(), and UpdateEKF() functions. I added a new function here with the common parts of the Update and UpdateEKF functions.

In most cases I used the code I had developed in the lesson exercises, with a couple of changes. I used 'double' rather than 'float' for my variables to increase precision a little bit, and I added a few extra checks for values close to zero, to increase the robustness of the code. The most important thing was to normalize the theta angle in the KalmanFilter::UpdateEKF function to be within the range -pi to pi (kalman_filter.cpp line 70-77). Without the normalization the kalman filter output goes wild around the middle of the path.

## Rubric Points

---

#### 1. Compiling  

The code compiles and links when calling 'make all' on the command line.

#### 2. Accuracy

Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

The resulting RMSE values do depend quite a lot about the starting values for the state variables. I used 2 different starting points for vx and vy, namely 0,0, or the actual values taken from the first measurement in the file obj_pose-laser-radar-synthetic-input.txt. The state initialization code can be found in FusionEKF.cpp line 101/102.

As expected the RMSE values was higher when using 0,0 as starting point:

![alt text][image1]

This is easy to understand as 0,0 only is accurate if the observed vehicle is standing still at the first measurement. When using the first observed velocity values instead I got:

![alt text][image3]

Note I did not change the initialization of my state covariance matrix P here, presumably I should as the uncertainty of the velocity is small when I used the correct values!

In any case I think using 0,0 is actually better because normally I would have no clue about whether the observed vehicle is moving or not prior to the first measurement. This point is illustrated when using Dataset 2, where the vehicle moves in the opposite direction causing the first velocity measurement from Dataset 1 to give worse RMSE results than 0,0.

In both cases the RMSE values were within the requirements for passing.

#### 3. Follows the correct Algorithm

My code follows the algorithms from the lessons. The code in kalman_filter.cpp should be easy to verify as I used the same variable names as in the lesson material.

It follows from the structure of the ProcessMeasurement() function in FusionEKF.cpp that the first measurement is used to initialize the filter. The code part in line 59-125 is executed only at the first measurement and takes care of all the initialization that is not already done in the constructor (line 16-46).

For all further measurements the handling is done in line 140-189 of FusionEKF.cpp. The processing includes calculating the time delta, updating the state transition matrix F, calculating the process covariance matrix Q, and then calling Predict and Update from the kalman filter. The Update call distinguishes between radar and lidar measurements and calls UpdateEKF() and Update() respectively (line 176-189).

#### 4. Code Efficiency

I extracted the common parts of the Update() and UpdateEKF() functions in kalman_filter.cpp into a function called UpdateCommon(), to avoid redundant code.

Other than that I actually went for code clarity more than compactness. I presume you can argue that the calculation of the Q matrix in FusionEKF.cpp line 148-160 is an example of avoiding repeated calculation of the same values.

### Conclusion

I miss information about how to choose the values for initial state covariance P. I did not find anything in the lesson material so I simply used the values provided in the example code in the lesson. This however is not really explained.

I believe all rubric points have been fulfilled.
