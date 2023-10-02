#!/usr/bin/env bash

echo "export DISABLE_GUI=1" >> ~/.bashrc
echo "export INSIDE_VM=1" >> ~/.bashrc

bash <(curl -s https://raw.githubusercontent.com/StanfordASL/asl-tb3-utils/main/scripts/install.bash)

