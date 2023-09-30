#!/usr/bin/env sh

echo "Install apt dependencies"
sudo apt install git python3-dev python3-venv cmake build-essentials vim tmux htop

echo "Pulling config repo"
git clone https://github.com/alvinsunyixiao/configs.git $HOME/configs

echo "Symlink dot files"
mkdir -p ~/.colcon
ln -s $HOME/configs/vim/.vimrc ~
ln -s $HOME/configs/tmux/.tmux.conf ~
ln -s $HOME/configs/colcon/defaults.yaml ~/.colcon

echo "Configure Vim"
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
vim +'PlugInstall --sync' +qa

