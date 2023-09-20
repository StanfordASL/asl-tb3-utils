# Some Automated Scripts

## Dependencies
Install `curl` with
```bash
sudo apt install curl -y
```

## Automatic Installation

1. Run the following script
    ```bash
    bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install.bash)
    ```
2. Restart terminal or run `source ~/.bashrc`.

**Note**: if you use `zsh`, then do the above steps and replace `bash` with `zsh`

## Automatic Pull Update and Re-build
```bash
sh <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/update.sh)
```

