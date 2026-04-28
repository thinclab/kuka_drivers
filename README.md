# ROS2 KUKA Drivers Fork

This fork is identical to the original `kuka_drivers` repository, except that it has some minor changes to work with the `kuka_kontrol` package.

The original KUKA Drivers GitHub Repository and Documentation can be found at the links below.<br>
[KUKA Drivers GitHub](https://github.com/kroshu/kuka_drivers)<br>
[KUKA Drivers Documentation](https://github.com/kroshu/kuka_drivers/wiki)<br>

#### Table of Contents
[Installation](#Installation)<br>

## Installation
It is necessary to install the corresponding fork of the `kuka_robot_descriptions` repository alongside this forked driver package. The original and forked KUKA Robot Descriptions GitHub Repositories can be found at the links below.<br>
[Original KUKA Robot Descriptions GitHub](https://github.com/kroshu/kuka_robot_descriptions)<br>
[Forked KUKA Robot Descriptions GitHub](https://github.com/thinclab/kuka_robot_descriptions/tree/jazzy)<br>

It is recommended to have a separate workspace for this driver and the robot descriptions repository to simplify the build process; use the commands below to create the `/kuka_ws` and to clone the forks into the `/src` directory.

    mkdir -p ~/kuka_ws/src
    cd ~/kuka_ws/src
    git clone -b jazzy https://github.com/thinclab/kuka_drivers.git
    git clone -b jazzy https://github.com/thinclab/kuka_robot_descriptions.git

After you have cloned the fork, go to `~/kuka_ws` and resolve dependencies.

    cd ~/kuka_ws
    rosdep install --from-paths src --ignore-src -r -y

Then, build the package in the workspace using the command below.

    MAKEFLAGS=`getconf _NPROCESSORS_ONLN` colcon build --continue-on-error --parallel-workers 4 --symlink-install --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
