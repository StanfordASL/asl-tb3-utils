# Installing Ubuntu 22.04 with Windows Subsystem for Linux (WSL)

## Requirements
- A computer with Windows 10/11
- About 20 Gb of free disk space. 

## Installation Guide
1. Perform windows updates to ensure that your OS is compatible with the latest versions of WSL.
2. Install WSL and Ubuntu 22.04 through the Microsoft store by following this [guide](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overview).
   - Feel free to skip installing Octave in the "Install and use a GUI package" section of the tutorial.
3. **If the computer you are using has a dedicated GPU, then do this step otherwise it can be ignored.** While in the WSL Ubuntu terminal enter the following command to disable GPU accelerated rendering in Gazebo (the robot simulator used in this class).
```sh
# Force CPU rendering in Gazebo
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

# Enable changes
source ~/.bashrc
```

