# pjpo-curvature

This is the project page of the paper "Path Planning for Autonomous Driving with Curvature-considered Quadratic Optimization", to appear in IV 2023. 

## About

The proposed planner generates the curvature-concerned paths in structure scenarios.

## Prerequisites
1. Build HSL **(recommended for better performance)**
    ```shell
    git clone https://github.com/coin-or-tools/ThirdParty-HSL.git --depth=1

    # Obtain a tarball with HSL source code from http://www.hsl.rl.ac.uk/ipopt/ and unpack this tarball
    gunzip coinhsl-x.y.z.tar.gz
    tar xf coinhsl-x.y.z.tar

    # Rename the directory `coinhsl-x.y.z` to `coinhsl`, or set a symbolic link:
    ln -s coinhsl-x.y.z coinhsl

    ./configure
    make
    sudo make install
    ```
2. Build [OSQP-eigen](https://github.com/robotology/osqp-eigen)

## Build
Clone repository to any catkin workspace and compile workspace

 ```shell
   cd ~/catkin_ws/src
   git clone https://github.com/badaolaziang/pjpo-curvature
   cd .. && catkin_make
   source devel/setup.bash
   ```

## Run a demo
1. launch the code:

```shell
  roslaunch pjpo_curvature combine.launch
```
2. run the planner:

    set a random 2D goal in the rviz, and get the path(s).

3. change the parameters of src/pjpo_curvature/launch/combine.launch and src/pjpo_curvature/script/reference_publisher.py to change the starting pose and the reference path.

## Acknowledgement
We would like to express sincere thanks to the authors of the following tools and packages:

Guide line and framework:[Cartesian_planner](https://github.com/libai1943/PJPOCurvature)

QP solver: [OSQP](https://github.com/osqp/osqp)&[OSQP-eigen](https://github.com/robotology/osqp-eigen)

