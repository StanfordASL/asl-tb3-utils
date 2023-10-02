# Some Automated Scripts

## Dependencies
Install `curl` with
```bash
sudo apt install curl -y
```

## Automatic Installation

1. If your Ubuntu 22.04 has less than 8GB RAM, you need to increase swap size by running
the following command

    ```bash
    bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/increaseswap.bash)
    ```

2. Run the following script to set up ROS2 and simulation
    ```bash
    bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install.bash)
    ```

3. Restart terminal or run `source ~/.bashrc`.

4. (Optional) If you are in VM or WSL environment
    ```bash
    $HOME/tb_ws/src/asl-tb3-utils/scripts/add_vm_flags.bash
    ```


**Note**: if you use `zsh`, then do the above steps and replace all `bash` with `zsh`.

## Automatic Pull Update and Re-build
```bash
sh <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/update.sh)
```

