Robot Modelling Library (RML)
===========

This is a lightweight C++ library using [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for modelling robots defined by a [URDF](http://wiki.ros.org/urdf) (Universal Robot Descriptor File).

## Features
- URDF parsing
- Forward Kinematics
- Inverse Kinematics with [Ipopt](https://coin-or.github.io/Ipopt/) interface [ifopt](https://github.com/ethz-adrl/ifopt)
- Automatic differentiation support with [autodiff](https://github.com/autodiff/autodiff)
- Plotting with [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### TODO: 
- Geometric Jacobians
- Dynamics
- ODE solver
- URDF visualizer
- Documentation/Examples

## Install

### 1. Install Dependencies
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [TinyXML2](https://github.com/leethomason/tinyxml2)
- [autodiff](https://github.com/autodiff/autodiff)
- [ifopt](https://github.com/ethz-adrl/ifopt) with MA57 solver from https://www.hsl.rl.ac.uk/ipopt/
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### 2. Build with cmake
  ```bash
  git clone https://github.com/Tom0Brien/RML.git && cd RML
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  ```
