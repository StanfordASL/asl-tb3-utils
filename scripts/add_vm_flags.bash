#!/usr/bin/env bash

if [ -z "${DISABLE_GUI}" ]; then
    echo "export DISABLE_GUI=1" >> ~/.bashrc
fi
if [ -z "${INSIDE_VM}" ]; then
    echo "export INSIDE_VM=1" >> ~/.bashrc
fi
