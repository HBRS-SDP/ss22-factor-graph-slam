# Factor Graph - SLAM

# Factor Graph

<center>

![Forney Factor Graph with representation of message passing](./Images%20and%20plots/pictures/readme_pictures/FFG.png)

<caption><b>Forney Factor Graph with representation of message passing</b></caption>

</center>

<br>
Factor graphs are one of the probabilistic graphical inference models, well suited for modelling and inferencing complex **estimation/optimization** problems like Simultaneous Localization and Mapping (SLAM) or Structure from Motion (SFM). Formally, Factor graphs are bi-partite graphs with two nodes,
 
*   **Variable nodes**,  represents unknown quantities in the problem
*   **Factor nodes**, defines the relationship between variables

SLAM problem solves two parts, **Localisation** (knowing where you are in the environmnet) and **Mapping** (How the environment looks). Factor graphs are one of the ways to solve the SLAM problem by modelling the unknown locations as nodes and their relationship between varibales or with environemnt objects (landmarks) as factors. Consider the simple SLAM problem below,

<center>

![SLAM](./Images%20and%20plots/pictures/readme_pictures/slam.png)
<caption><b>Simple SLAM example</b></caption>

</center>

x1, x2 and x3 are nodes which are unknown locations 
l1 and l2 are the location of landmarks (which will be extracted from the environment using sensors)
f1, f2, f3 are factors which defines the relationship between nodes and landmarks

Factor graphs GTSAM library 

# Required libraries

- [GTSAM](https://github.com/borglab/gtsam)
- [aruco detect](http://wiki.ros.org/aruco_detect)

# Building GTSAM libraries

```
git clone https://github.com/borglab/gtsam.git
````
Then, in the root folder of the library, execute the following commands
```
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```
Pre-requisite libraries: **Boost, CMake**

If not, install it with the following commands
```
$ sudo apt-get install libboost-all-dev
$ sudo apt-get install cmake
```

# Building aruco detect library

```
sudo apt-get install ros-noetic-aruco-detect
```
