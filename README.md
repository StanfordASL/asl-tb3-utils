# asl-tb3-utils
ASL Turtlebot 3 ROS2 utility packages

## Installation Guide
1. Setup Ubuntu 22.04 following the guides for [MacOS]() or [Windows]() (TODO: empty link).
2. Install ROS2 **Humble** on Ubuntu 22.04 following this
   [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
   - These, and the following, installation instructions require opening a terminal, and copy-pasting commands into that terminal. In Ubuntu, use CTRL+ALT+T to open the default terminal. Remember to copy all commands exactly and in order, or you may encounter errors! 
   - On the **Install ROS Packages** step, choose the Desktop Install (`sudo apt install ros-humble-desktop`).
3. Install Gazebo **Garden** (ignore the comment lines starting with `#`).
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
4. Initialize `rosdep` (can skip this step if done earlier)
    ```sh
    sudo rosdep init
    ```
5. Set up a ROS2 workspace by running the following scripts (ignore the comment lines starting with `#`)
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
    source ~/.bashrc
    ```
6. Try starting ROS and Gazebo with a simulated TurtleBot to verify that everything is installed correctly. Use this command:
   ```sh
   ros2 launch asl_tb3_sim root.launch.py 
   ```
   To close the simulator and ROS session after testing your installation enter CTRL+C in the terminal session where you entered the command above.

## Development Guide
TODO

## F.A.Q.
- When I run Gazebo a bunch of errors print that say ``[ruby $(which gz) sim-2] [Err] [SDFFeatures.cc:843] The geometry element of collision [left_hand] couldn't be created``, should I be worried?
   - These are a known issue with Gazebo, and are safe to ignore.

