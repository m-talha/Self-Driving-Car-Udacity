# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Overview
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

---

## Dependencies

Keep in mind that the minimum project dependency versions are:

**cmake: 3.5**
- All OSes: [click here for installation instructions](https://cmake.org/install/)

**make: 4.1 (Linux and Mac), 3.81 (Windows)**
- Linux: make is installed by default on most Linux distros
- Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
- Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

**gcc/g++: 5.4**
- Linux: gcc / g++ is installed by default on most Linux distros
- Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
- Windows: recommend using [MinGW](http://www.mingw.org/)

If missing, the script in the installation process below will install them automatically.

---

## Installation

Start by cloning this project.

There are 3 components to install to run the EKF:
1. Unity simulator - For interfacing and graphical output. The Term 2 Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). 
2. Installation of uWebSocketIO - handles communication between the C++ EKF and Unity simulator.
3. Installation of EKF - the Extended Kalman Filter in this repo.

### uWebSocketIO Installation

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

**Linux Installation**

From the project repository directory run the script: `install-ubuntu.sh`

**Mac Installation**

From the project repository directory run the script: `install-mac.sh`

**Windows Installation**
##### **Bash on Windows**

This project has been developed using the Linux Bash Shell on Windows 10 with Ubuntu 18.04 LTS and assume you have this [installed](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/). 

Navigate to the project folder in bash and run: `sudo install-linux.sh`

This step may fail for number of reasons as listed below:

- `ln: failed to create symbolic link '/usr/lib/libuWS.so': File exists`, run the following command: `sudo rm /usr/lib/libuWS.so` and rerun the above command.
- `install-ubuntu.sh` has only `rw` but no `x` permission. Run `chmod a+x install-ubuntu.sh` to give execution permission
- Cannot find the package `libuv1-dev`
   - To install the package run `sudo apt-get install libuv1.dev`
   - If you still cannot install the package run the following to get the package and install it:
      ```
      sudo add-apt-repository ppa:acooks/libwebsockets6
      sudo apt-get update
      sudo apt-get install libuv1.dev
      ```
- May complain about the version of cmake you have. You need a version greater than 3.0. [Here is a link](https://askubuntu.com/questions/355565/how-to-install-latest-cmake-version-in-linux-ubuntu-from-command-line) which describes how to get version 3.8. Look at Teocci's response in this link
- Installing cmake requires g++ compiler. Install a g++ version 4.9 or greater. Here are the steps:
```
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.9
```

##### **Docker**

Alternatively, a Docker image containing all the project dependencies can be used.

First install [Docker Toolbox for Windows](https://docs.docker.com/toolbox/toolbox_install_windows/) (Windows 10 Home) or [Docker Desktop on Windows](https://docs.docker.com/docker-for-windows/install/) (Windows 10 Pro, Enterprise, and Education).

Next, launch the Docker Quickstart Terminal. The default Linux virtual environment should load up. You can test that Docker is setup correctly by running `docker version` and `docker ps`.

You can enter a Docker image that has all the project dependencies by running:

`docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest`

Once inside Docker you can clone over the GitHub project repositories and run the project from there. 

##### **Port forwarding is required when running code on VM and simulator on host**

For security reasons, the VM does not automatically open port forwarding, so you need to manually [enable port 4567](https://www.howtogeek.com/122641/how-to-forward-ports-to-a-virtual-machine-and-use-it-as-a-server/). This is needed for the C++ program to successfully connect to the host simulator.

**Port Forwarding Instructions**

1. First open up Oracle VM VirtualBox
2. Click on the default session and select settings.
3. Click on Network, and then Advanced.
4. Click on Port Forwarding
5. Click on the green plus, adds new port forwarding rule.
6. Add a rule that assigns `4567` as both the host port and guest Port

### EKF Installation

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
1. mkdir build
2. cd build
3. sudo cmake ..
4. make
5. ./ExtendedKF
```

Launch the simulator selecting the `EKF` project and press `Start`. If you see this message, it is working: `Listening to port 4567 Connected!!!`

---

## uWebSockeIO Communication

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
