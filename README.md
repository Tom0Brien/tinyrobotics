EigenRobotics
===========

This is a lightweight C++ library using [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for robots defined by a [URDF](http://wiki.ros.org/urdf) (Unified Robot Description Format).

## Features
- URDF parsing
- C++ templated
- Forward Kinematics
- Inverse Kinematics (with [Ipopt](https://coin-or.github.io/Ipopt/) interface [ifopt](https://github.com/ethz-adrl/ifopt))
- Dynamics
- Geometric Jacobians
- Centre of mass position 
- Automatic differentiation with [autodiff](https://github.com/autodiff/autodiff)
- ODE solver

### In Development:
- Analytical Jacobians
- Analytical Derivatives
- ODE solver higher order routines
- Documentation & Examples

## Install

### 1. Install Dependencies
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [TinyXML2](https://github.com/leethomason/tinyxml2)
- [autodiff](https://github.com/autodiff/autodiff)
- [ifopt](https://github.com/ethz-adrl/ifopt)

### 2. Build with cmake
  ```bash
  git clone https://github.com/Tom0Brien/RML.git && cd RML
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  ```

## Example
The code below demonstrates how to load in a URDF model and compute the forward kinematics to a link.

```c++
// Create a robot model with 4 joints
std::string path_to_urdf = "../data/urdfs/simple.urdf";
const int n_joints       = 4;
auto model               = RML::model_from_urdf<double, n_joints>(path_to_urdf);

// Create a configuration vector of zeros
auto q0 = model.home_configuration();

// Compute the forward kinematics to the left foot at the home configuration
auto H0 = RML::forward_kinematics(model, q0, "left_foot");
```
