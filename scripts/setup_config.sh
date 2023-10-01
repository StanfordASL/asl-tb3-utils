#!/usr/bin/env sh

echo "Install apt dependencies"
sudo apt install -qq git python3-dev python3-venv cmake build-essential vim tmux htop gh -y

echo "Symlink dot files"
mkdir -p ~/.colcon
git clone https://github.com/alvinsunyixiao/configs.git $HOME/configs
ln -s $HOME/configs/vim/.vimrc ~
ln -s $HOME/configs/tmux/.tmux.conf ~
ln -s $HOME/configs/colcon/defaults.yaml ~/.colcon

echo "Configure Vim"
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
vim -E +'PlugInstall --sync' +qa

echo "Rebuild workspace"
update_tb_ws

