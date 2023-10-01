#!/usr/bin/env zsh

echo "export DISABLE_GUI=1" >> ~/.zshrc

zsh <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install.zsh)

