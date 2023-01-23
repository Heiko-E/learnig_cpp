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

## Setup WSL2

> **HINT** for OSD6: If connection is not possible use

    wsl --shutdown

Install development environment:

    sudo apt-get update
    sudo apt-get install build-essential gcc g++ gdb
    sudo apt-get install cmake ansible

Configure CMake:

<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>P

    Cmake: Configure