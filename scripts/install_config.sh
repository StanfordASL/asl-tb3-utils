#!/usr/bin/env sh

echo "Install apt dependencies"
sudo apt install -qq git python3-dev python3-venv cmake build-essential vim tmux htop gh -y

echo "Symlink dot files"
if [ -d "$HOME/configs" ]; then
    mv $HOME/configs $HOME/configs.bak
    echo "Found existing ~/configs, moved to ~/configs.bak"
fi
for f in ".vimrc" ".tmux.conf" ".colcon/defaults.yaml"
do
    if [ -f $HOME/$f ]; then
        mv $HOME/$f $HOME/$f.bak
        echo "Found existing ~/$f, moved to ~/$f.bak"
    fi
done
mkdir -p ~/.colcon
git clone https://github.com/alvinsunyixiao/configs.git $HOME/configs
ln -sf $HOME/configs/vim/.vimrc ~
ln -sf $HOME/configs/tmux/.tmux.conf ~
ln -sf $HOME/configs/colcon/defaults.yaml ~/.colcon

echo "Configure Vim"
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
vim -E +'PlugInstall --sync' +qa

echo "Rebuild workspace"
$HOME/tb_ws/src/asl-tb3-utils/scripts/update.sh
