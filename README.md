# asl-tb3-utils
ASL Turtlebot 3 ROS2 utility packages

## Installation Guide
1. Install ROS2 **Humble** on Ubuntu 22.04 following this
   [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
   - These, and the following, installation instructions require opening a terminal, and copy-pasting commands into that terminal. In Ubuntu, use CTRL+ALT+T to open the default terminal. Remember to copy all commands exactly and in order, or you may encounter errors!
   - On the **Install ROS Packages** step, choose the ROS-Base Install (`sudo apt install ros-humble-ros-base`).
   - Make sure to also install development tools (`sudo apt install ros-dev-tools`).
2. Install Gazebo **Garden** (ignore the comment lines starting with `#`).
   ```sh
   # Install dependencies
   sudo apt-get update
   sudo apt-get install lsb-release wget gnupg

   # Setup Gazebo keys and sources
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

   # Install Gazebo
   sudo apt-get update
   sudo apt-get install gz-garden
   ```
3. Initialize `rosdep` (can skip this step if done earlier)
    ```sh
    sudo rosdep init
    ```
4. Set up a ROS2 workspace by running the following scripts (ignore the comment lines starting with `#`)
    ```sh
    # install apt depedencies
    sudo apt install git

    # create the turtlebot workspace
    mkdir -p ~/tb_ws/src

    # clone the sources
    cd ~/tb_ws/src
    git clone -b humble https://github.com/gazebosim/ros_gz.git
    git clone https://github.com/StanfordASL/asl-tb3-driver.git
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
    echo "alias update_tb_ws=\$HOME/tb_ws/src/asl-tb3-utils/scripts/update.sh" >> ~/.bashrc
    source ~/.bashrc
    ```
5. Try starting ROS and Gazebo with a simulated TurtleBot to verify that everything is installed correctly. Use this command:
   ```sh
   ros2 launch asl_tb3_sim root.launch.py
   ```
   To close the simulator and ROS session after testing your installation enter CTRL+C in the terminal session where you entered the command above.

## Environment Variables

* `DISABLE_GUI` -- Set to 1 to disable Gazebo GUI. This can speed up simulation if running inside
    VM or limited compute hardware.
* `INSIDE_VM` -- Set to 1 to use software OpenGL. OpenGL implementation in VMWare and WSL does
    not work very well with Gazebo. Set this env variable if you see flickering or
    blank screen in Gazebo.

## Development Guide

### Pull Latest and Re-Build Local Workspace
Run `update_tb_ws` in your terminal.

### Make Contribution to This Repository
1. Create a new branch.

2. Commit and push changes to the new branch.

3. Create a pull request and ask someone to review it.

4. Merge after an approval review.

## F.A.Q.
1. When I run Gazebo a bunch of errors print that say ``[ruby $(which gz) sim-2] [Err] [SDFFeatures.cc:843] The geometry element of collision [left_hand] couldn't be created``, should I be worried?

    These are a known issue with Gazebo, and are safe to ignore.

2. If Gazebo cannot find model uri, e.g. the terminal throws the following error
    ```
    ... Msg: Unable to find uri[model://asl_tb3_sim/models/turtlebot3_world]
    ```

    You need to rebuild `tb_ws` and make sure to include the environment variable `GZ_VERSION=garden`.
    To do a clean rebuild, run the following command in a terminal.

    ```sh
    cd ~/tb_ws && rm -r build install log
    GZ_VERSION=garden colcon build --symlink-install
    ```
