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

2. Run one of the following script
    - For native Ubuntu 22.04
        ```bash
        bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install.bash)
        ```
    - For VM or WSL Ubuntu 22.04
        ```bash
        bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install_vmware.bash)
        ```

3. Restart terminal or run `source ~/.bashrc`.

**Note**: if you use `zsh`, then do the above steps and replace `bash` with `zsh`

## Automatic Pull Update and Re-build
```bash
sh <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/update.sh)
```

