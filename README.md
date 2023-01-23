# Learnig cpp
Repository for c++ related experiments like:
- coding examples
- build system

## Setup Windows

### Installation instructions:
[Install cmake and MinGW](https://perso.uclouvain.be/allan.barrea/opencv/building_tools.html)

### Check installation
Open Powersehll and enter:

    g++ --version
    gdb --version
    cmake --version

## Setup Windows Subsystem for Linux 2

Install a Linux distridution. E.g. [Ubuntu via windows AppStore](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#3-download-ubuntu)

Checkout the repo to the home directory e.g. \\wsl.localhost\OSD\home\<Me>\

> **HINT** for OSD6: If connection is not possible use

    wsl --shutdown

Show Linux distributions in Windows Subsystem for Linux:

     wsl --list --verbose

Install development environment:

    sudo apt-get update
    sudo apt-get install build-essential gcc g++ gdb
    sudo apt-get install cmake ansible

Configure CMake:

<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>P

    Cmake: Configure