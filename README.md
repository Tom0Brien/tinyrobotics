Robot Modelling Library (RML)
===========

This is a lightweight C++ library for modelling robots defined using a [URDF](http://wiki.ros.org/urdf) (Universal Robot Descriptor File).



## Features
- URDF parsing to an [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) based templated class.
- Forward Kinematics
- Geometric Jacobians
- Automatic differentiation support with [autodiff](https://github.com/autodiff/autodiff)

### To be implemented

- [ ] Inverse Kinematics with IPOPT interface [ifopt](https://github.com/ethz-adrl/ifopt)
- [ ] Dynamics:
   - Full model dynamics
   - Holonomic reduction
   - Non-Holonomic reduction
   - Constrained model dynamics
- [ ] ODE solver with event detection for simulation of hybrid systems
- [ ] URDF visualizer
- [ ] Plotting

## Install

### 1. Install Dependencies
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [Boost](https://www.boost.org/)
- [autodiff](https://github.com/autodiff/autodiff)
- [ifopt](https://github.com/ethz-adrl/ifopt)
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### 2. Build with cmake
  ```bash
  git clone https://github.com/Tom0Brien/RML.git && cd RML
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  ```
