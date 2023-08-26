# asl-tb3-utils
ASL Turtlebot 3 ROS2 utility packages

## Installation Guide
1. Setup Ubuntu 22.04 following the guides for [MacOS]() or [Windows]() (TODO: empty link).
2. Install ROS2 **Humble** on Ubuntu 22.04 following this
   [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
3. Install Gazebo **Garden** following this [guide](https://gazebosim.org/docs/garden/install_ubuntu).
4. Set up a ROS2 workspace by running the following scripts in a Ubuntu 22.04 terminal
   (ignore the comment lines starting with `#`)
    ```sh
    # create the turtlebot workspace
    mkdir -p ~/tb_ws/src

    # clone the sources
    cd ~/tb_ws/src
    git clone -b humble https://github.com/gazebosim/ros_gz.git
    git clone https://github.com/StanfordASL/asl-tb3-utils.git

    # install dependencies
    source /opt/ros/humble/setup.bash  # use setup.zsh if using zsh
    rosdep update && rosdep install --from-paths ~/tb_ws/src -r -i -y

    # build the code (might take a few minutes)
    export GZ_VERSION=garden
    cd ~/tb_ws && colcon build --symlink-install

    # include the setup script (replace bash with zsh if using zsh)
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source \$HOME/tb_ws/install/local_setup.bash" >> ~/.bashrc
    ```

## Development Guide
TODO

