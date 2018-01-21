
### Sefl-Driving Car Nanodegree Program. Term 2
<img style="float: left;" src="https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg">

## Project 6: Extended Kalman Filter

### Overview
---
The goal of this project is to create a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

The project includes following steps:
* Completion of the [project template](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) C++ sorce code in order to build an Extended Kalman Filter (EKF) by applying the general processing flow as described in the class lessons
* Testing of the EKF code with input data generated by the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* Visualizion of the EKF output in a Jupyter notebook adopted from the provided in the [Sensor Fusion utility repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities)


### Project directory content:

* [README.md](README.md) - This file.
* [src/](src/) folder:
   - project C++ souce code files
   - [CMakeLists.txt](src/CMakeLists.txt) - `cmake` input file to generate the project `Makefile`. It was moved to the `src` folder and adopted to generate an `Eclipse` project - thanks to [Eclipse IDE profile](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/tree/master/ide_profiles/Eclipse) repo for provided instructions. 
* [data/](data/) folder:
    - [ekf-visualization.ipynb](data/ekf-visualization.ipynb) - an IPython notebook for the output data visualization and analysis.
    - [ekf_pos_output.txt](data/ekf_pos_output.txt) - output data file generated by the project code
    - [ekf_position.png](data/ekf_position.png) - the output image generated by the notebook 

### Project Environment


The project code was built and executed on a Linux VMware guest. The simulator was running on a Windows host with a port forwarding setup as instructed [here](https://discussions.udacity.com/t/running-simulator-on-windows-and-code-on-ubuntu/255869)  

### Project Build instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile the code: `cmake ../src/ && make` (Please note that `CMakeList.txt` file is located in folder `src/`)
3. Run the code: `./ExtendedKF`

### Project results

The source file [main.cpp](src/main.cpp) was modified to store the output data records in a text file [ekf_pos_output.txt](data/ekf_pos_output.txt) in the following format:
`est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy RMSE(0) RMSE(1) RMSE(2) RMSE(3)`

After processing the output file in the [ekf-visualization.ipynb](data/ekf-visualization.ipynb) notebook the object position estimates look as following:
![ekf_position.png](data/ekf_position.png)

The below table shows the position and velocity acccuracy of the last 10 estimates:
![ekf_rmse.png](data/ekf_rmse.png)

### Issues

When executed on Linux VM, an error `"Segmentation fault (core dumped)"` is reported after `h.onDisconnection()` call. The issue is still unsolved. I blame it on the environment and appreciate any inputs/hints that might help to resolve it.
