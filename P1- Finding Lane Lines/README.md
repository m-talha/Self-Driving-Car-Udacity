# **Finding Lane Lines on the Road** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="examples/laneLines_thirdPass.jpg" width="480" alt="Combined Image" />

Overview
---

When we drive, we use our eyes to decide where to go.  The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle.  Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm.

Project Structure
---

- **[P1.pynb](https://github.com/m-talha/Self-Driving-Car-Engineer-Udacity-P1-Lane-Finding/blob/master/P1.ipynb):** Jupyter notebook containing the pipeline for finding the lanes

- **[writeup.md](https://github.com/m-talha/Self-Driving-Car-Engineer-Udacity-P1-Lane-Finding/blob/master/writeup.md):** Writeup of the pipeline implementation describing the steps taken to extract edges and convert them to line segments using Hough transform

- **[test_videos_output](https://github.com/m-talha/Self-Driving-Car-Engineer-Udacity-P1-Lane-Finding/tree/master/test_videos_output):** Directory containing the processed test videos

- **[test_images_output](https://github.com/m-talha/Self-Driving-Car-Engineer-Udacity-P1-Lane-Finding/tree/master/test_images_output):** Directory containing the processed test images

Running the code
---

The IPython notebook can be run using a Jupyter Notebook app such as Anaconda. The project depends on the NumPy, OpenCV, Matplotlib & MoviePy libraries.
