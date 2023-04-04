tinyrobotics
===========

**tinyrobotics** is a lightweight C++ library which provides core robotics algorithms and tools. 

The goal of **tinyrobotics** is to be as simple as possible but still be incredibly fast and versatile.

## Features
<h2><a href="https://tom0brien.github.io/tinyrobotics/structtr_1_1model_1_1Model.html#details">Model</a></h2>

A tinyrobotics [model](https://tom0brien.github.io/tinyrobotics/structtr_1_1model_1_1Model.html#details) consists of a collection of links connected via joints. The following functions can be used to build a model, all of which are implemented in [Parser](https://tom0brien.github.io/tinyrobotics/Parser_8hpp.html)

| Function      | Description                                                                              |
| ------------- | ---------------------------------------------------------------------------------------- |
| `import_urdf` | Generate a tinyrobotics model from a [URDF](http://wiki.ros.org/urdf) robot description. |

<h2><a href="https://tom0brien.github.io/tinyrobotics/Kinematics_8hpp.html">Kinematics</a></h2>

| Function                 | Description                                                       |
| ------------------------ | ----------------------------------------------------------------- |
| `forward_kinematics`     | Compute transforms between links.                                 |
| `forward_kinematics_com` | Compute transforms between link and another links centre of mass. |
| `translation`            | Compute translation between links.                                |
| `rotation`               | Compute rotation between links.                                   |
| `geometric_jacobian`     | Compute geometric jacobian to a link.                             |
| `geometric_jacobian_com` | Compute geometric jacobian to a links centre of mass.             |
| `centre_of_mass`         | Compute centre of mass of model.                                  |

<h2><a href="https://tom0brien.github.io/tinyrobotics/Dynamics_8hpp.html">Dynamics</a></h2>

| Function           | Description                                                                    |
| ------------------ | ------------------------------------------------------------------------------ |
| `forward_dynamics` | Compute joint accelerations given joint positions and velocities and torques.  |
| `inverse_dynamics` | Compute required joint torques for given motion.                               |
| `mass_matrix`      | Compute mass matrix given joint positions.                                     |
| `total_energy`     | Compute total energy (kinetic + potential) given joint positions and velocity. |

## Install

### 1. Install Dependencies
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [TinyXML2](https://github.com/leethomason/tinyxml2) 
- [autodiff](https://github.com/autodiff/autodiff) 
- [ifopt](https://github.com/ethz-adrl/ifopt) 

### 2. Build with cmake
  ```bash
  git clone https://github.com/Tom0Brien/tinyrobotics.git && cd RML
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  ```

## Examples
The code below demonstrates how to load in a URDF model and compute the forward kinematics between two links.

```c++
// Create a tinyrobotics model with 4 joints defined in example.urdf
auto model = import_urdf<double, 4>("example.urdf");

// Create a configuration vector of all zeros
auto q = model.home_configuration();

// Get the target link index
auto target_idx = model.get_link("target_link").idx;

// Compute the forward kinematics to the target link from the base frame at the home configuration
auto H = forward_kinematics(model, q, target_idx);
```
