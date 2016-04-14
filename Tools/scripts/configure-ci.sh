#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ex

ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"

RPI_ROOT="master"
RPI_TARBALL="$RPI_ROOT.tar.gz"

mkdir -p $HOME/opt
pushd $HOME/opt

# PX4 toolchain
compiler=$ARM_ROOT
if [ ! -d "$HOME/opt/$compiler" ]; then
  wget http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL
  tar -xf $ARM_TARBALL
fi

# RPi/BBB toolchain
compiler="tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64"
if [ ! -d "$HOME/opt/$compiler" ]; then
  wget http://firmware.ardupilot.org/Tools/Travis/NavIO/$RPI_TARBALL
  tar -xf $RPI_TARBALL $compiler
fi

popd

mkdir -p $HOME/bin

# symlink to compiler versions
ln -s /usr/bin/gcc-4.9 ~/bin/gcc
ln -s /usr/bin/g++-4.9 ~/bin/g++
ln -s /usr/bin/clang-3.7 ~/bin/clang
ln -s /usr/bin/clang++-3.7 ~/bin/clang++
ln -s /usr/bin/llvm-ar-3.7 ~/bin/llvm-ar

mkdir -p $HOME/ccache

# configure ccache
ln -s /usr/bin/ccache ~/ccache/g++
ln -s /usr/bin/ccache ~/ccache/gcc
ln -s /usr/bin/ccache ~/ccache/arm-none-eabi-g++
ln -s /usr/bin/ccache ~/ccache/arm-none-eabi-gcc
ln -s /usr/bin/ccache ~/ccache/arm-linux-gnueabihf-g++
ln -s /usr/bin/ccache ~/ccache/arm-linux-gnueabihf-gcc

exportline="export PATH=$HOME/ccache"
exportline="${exportline}:$HOME/bin"
exportline="${exportline}:$HOME/opt/gcc-arm-none-eabi-4_9-2015q3/bin"
exportline="${exportline}:$HOME/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin"
exportline="${exportline}:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then
    echo nothing to do;
else
    echo $exportline >> ~/.profile;
fi

. ~/.profile

pip install --user argparse empy pyserial pexpect mavproxy