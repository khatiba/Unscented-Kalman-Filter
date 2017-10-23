[//]: # (Image References)
[image1]: ./images/sim-screen.png
[image2]: ./images/nis-stats.png


# Unscented Kalman Filter
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

![UKF Results][image1]

Shown in the image above, the resulting RMSE values are within the acceptable ranges of `0.09`, `0.10` for **px** and **py** and `0.40`, `0.30` for **vx** and **vy**.

| State | RMSE   |
| ----- | ----   |
| PX    | 0.0718 |
| PY    | 0.0835 |
| VX    | 0.3399 |
| VY    | 0.2433 |


![NIS Results][image2]

The noise parameters were tuned by using intuition, trial and error and consistency checking. The image above shows Normalized Innovation Squared (NIS) values for lidar and radar. For good consistency, no more than 5% of NIS values should be above the red line and no more than 5% of NIS values should be below the yellow line.

The final noise parameters used: 

`longitudinal acceleration noise = 1.8 m/s^2`

`yaw acceleration noise = 0.39 rad/s^2`


#### Installation
This project requires the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

**Using Docker**

Docker is the easiest, simply cd to the directory where you clone this repo and run: 

``docker run -t -d --rm -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest``

**Without Docker**

Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/).

#### Running the Program

1. Clone this repo.
2. Make a build directory: 
        
    `mkdir build && cd build`

3. Compile:
    
    `cmake .. && make` 

   * On windows, you may need to run:
    
        `cmake .. -G "Unix Makefiles" && make`

4. Run it:

    `./ExtendedKF `

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

`["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)`


OUTPUT: values provided by the c++ program to the simulator

```
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

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


## Generating Additional Data
If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
