#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

### Install numba on X86 ubuntu for KCF tracking
sudo apt-get install llvm
sudo apt install python3-pip
pip3 install llvmlite
pip3 install numba
# For numba on ARM ubuntu, refer to https://blog.csdn.net/benchuspx/article/details/109152810
