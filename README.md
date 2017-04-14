# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

- Atul Acharya

---

## Results

The Unscented Kalman Filter (UKF) is an extension of the regular Extended Kalman Filter (EKF). The UKF allows for non-linear models (unlike the EKF, which assumes a _constant velocity_ model). UKF allows for:

- constant turn rate and velocity (CTRV)
- constant turn rate and acceleration (CTRA)
- constant steering angle and veloticy (CSAV)
- constant curvature and acceleration (CCA)

This project assumes the **CTRV** motion model on given datasets. To deal with non-linear models, UKF works via unscented transformations. In the **Predict phase**, it begins by generating Sigma points, augments them, and then predicts the mean state vector and process covariance matrices. In the **Update phase**, the sigma points are transformed into measurement space, and then the updates are applied based on sensor (Radar/Lidar) measurements to get the new values for state vector and process covariance matrix.

Results of the UKF project are shown below.

[//]: # (Image References)
[image1]: ./images/results_vis.png
[image2]: ./images/nis_vis.png

![UKF prediction][image1]

The UKF parameters are also shown, along with the resulting RMSE values for each dataset. The UKF parameters are chosen to optimize the RMSE within the required ranges.

- on dataset1, the RMSE values of [px, py, vx, vy] values are within the required range of [0.09, 0.09, 0.65, 0.65]

- on dataset2, the RMSE values of [px, py, vx, vy] values are within the required range of [0.20, 0.20, 0.55, 0.55]

The chart below shows the NIS visualization.

![NIS results][image2]


---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/4d0420af-0527-4c9f-a5cd-56ee0fe4f09e)
for instructions and the project rubric.
