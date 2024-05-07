# ROS2 Point Cloud Clustering and Segmentation

We will start with RTAB mapping, a powerful technique for creating accurate 3D maps using RGB-D cameras. Through hands-on projects, you will learn how to use this technique to generate high-quality point clouds from your own data.
Next, we will dive into the Kitti Dataset and explore how to use 3D lidars for object detection. We will teach you how to use advanced techniques for detecting  objects in real-time, such as lidar-based segmentation and clustering.
We will also cover ROS2, an essential tool for visualizing and processing point cloud data. With ROS2, you will learn how to use rviz and PCL to create stunning visualizations and analyze your point cloud data with ease.
In addition, we will explore cylindrical and planar segmentation, two important techniques for extracting meaningful information from your point cloud data. Through a series of hands-on exercises, you will learn how to use these techniques to accurately identify and classify objects in your point clouds.

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About-this-Repository">About This Repository</a></li>
    <li><a href="#Using-this-Repository">Using this Repository</a></li>
    <li><a href="#Course-Workflow">Course Workflow</a></li>
    <li><a href="#Features">Features</a></li>
    <li><a href="#Pre-Course-Requirements">Pre-Course Requirments</a></li>
    <li><a href="#Link-to-the-Course">Link to the Course</a></li>
  </ol>
</details>

## About this Repository
This is repository for the course **ROS2 Point Clouds For Autonomous Self Driving Car using PCL** availble at Udemy . Complete source code is open sourced.

![alt text](image_resources/main_cover.png)

## Create you workspace

Create your workspace at `$HOME` or any desired location.

```bash
mkdir  ~/ros2_ws/src/
```
## Using this Repository

* Move into your workspace/src folder

```bash
.g cd ~/ros2_ws/src/
```

* Clone the repository in your workspace

<!-- TODO: Add instructions for devcontainer -->
<!-- TODO: Add instructions to recursively init submodules -->

```bash
git clone git@github.com:wambitz/ros2-humble-pcl-devcontainer.git
```

* Perform make and build through `colcon`

 ```bash
 cd ~/ros2_ws/
  colcon build
 ```

* Source your Workspace in any terminal you open to Run files from this workspace

```bash
source /path/to/ros2_ws/install/setup.bash
```

- Make sure kitti data is in *~/ros2_ws/data*
- Build Kitti Data Processing and run it

```bash
colcon build && ros2 launch  point_cloud_processing process_kitti.launch.py
```

- Run Rviz with config file
```
ros2 launch  point_cloud_processing bring_rviz.launch.py
```

## Course Workflow
- Creating Custom Point clouds using pcl in cpp
- Create 3D point clouds using depth cameras
- Setup ROS2 and Kitti Dataset for processing

## Features

**Creating Point Clouds**

![alt text](./image_resources/custom.gif)

**Building Point Cloud Maps with RTAB-Map**

![alt text](./image_resources/rtabmap.gif)

**Processing Kitti Dataset**

![alt text](./image_resources/kitti.gif)

## Pre-Course Requirements

**Software Based**
* Ubuntu 22.04 (LTS)
* ROS2 - Humble


## Link to the Course
Below is a discounted coupon for people who want to take the course in which more explanation to this code has been added

**[[Udemy course]](https://www.udemy.com/course/ros2-point-clouds-for-autonomous-self-driving-car-using-pcl/?couponCode=MAY_LEARN)**



