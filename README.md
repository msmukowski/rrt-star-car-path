# **University project of RRT * algorithm for car path planning**

[![image](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/HeZhang1994/gif-creator/blob/master/LICENSE)
[![image](https://img.shields.io/badge/python-3.8-blue.svg)]()
[![image](https://img.shields.io/badge/status-stable-brightgreen.svg)]()
[![image](https://img.shields.io/badge/version-1.0.0-informational)]()

> **Goal**: The algorithm must find the path on the map taking into account the kinematics of the car.

### Important
- Preconfigured Rviz is required for proper operation.
- The application was developed and tested on **Linux**, *Ubuntu 19.04 and higher*.

## Functions

- Runs the **RRT\* algorithm**, which searches the area on a two-dimensional bitmap, distinguishing between allowed and forbidden fields.

- Creates **real-time visualizations** of the search tree on a bitmap.
<p><center><img src="images/rrtstar.png" width="50%"></center></p>

- It allows you to run **simulations** of the vehicle driving along the found path in both **RViz** and **Gazebo**.
<p><center><img src="images/car.png" width="50%"></center></p>


## Usage

- **To run clean RRT\* (without simulation):**
```
    roslaunch rrt_star_car_path rrt_star.launch
```
 - **To run rrt\* + car simulation:**
```
    roslaunch rrt_star_car_path rrt_star_car.launch
```


## Results
- At the moment, the application finishes its work after finding and displaying a path or showing a driving simulation.