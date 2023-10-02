#!/usr/bin/env bash

echo "Expanding Swap to 16 Gb"

sudo swapoff /swapfile
sudo rm /swapfile
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

echo "Done Expanding swap to 16G"
