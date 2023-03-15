tinyrobotics
===========

**tinyrobotics** is a lightweight C++ library which provides core robotics algorithms and tools. 

The goal of **tinyrobotics** is to be as simple as possible. The library leverages [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) and modern C++ to be incredibly fast and versatile.

## Features
### Models
A tinyrobotics [model](./include/Model.hpp) consists of a collection of [links](./include/Link.hpp) connected via [joints](./include/Joint.hpp). The following functions can be used to build a model, all of which are implemented in [Parser](./include/Parser.hpp).
| Function | Description |
| --- | --- |
| `model_from_urdf` | Generate a tinyrobotics model from a [URDF](http://wiki.ros.org/urdf) robot description. |

### Kinematics
Kinematics algorithms are implemented in [Kinematics](./include/Kinematics.hpp).
| Function | Description |
| --- | --- |
| `forward_kinematics` | Compute transforms between links. |
| `forward_kinematics_com` | Compute transforms between link and another links centre of mass. |
| `translation` | Compute translation between links. |
| `rotation` | Compute rotation between links. |
| `geometric_jacobian` | Compute geometric jacobian to a link. |
| `geometric_jacobian_com` | Compute geometric jacobian to a links centre of mass. |
| `centre_of_mass` | Compute centre of mass of model. |

Inverse Kinematics is implemented in [InverseKinematics](./include/InverseKinematics.hpp).
| Function | Description |
| --- | --- |
| `inverse_kinematics` | Solve joint positions to achieve desired pose of model. |

### Dynamics
Dynamics algorithms are implemented in [Dynamics](./include/Dynamics.hpp).
| Function | Description |
| --- | --- |
| `forward_dynamics` | Compute joint accelerations given joint positions and velocities and torques. |
| `inverse_dynamics` | Compute required joint torques for given motion. |
| `mass_matrix` | Compute mass matrix given joint positions. |
| `total_energy` | Compute total energy (kinetic + potential) given joint positions and velocity. |

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
// Create a robot model with 4 joints
auto model = model_from_urdf<double, 4>("example.urdf");

// Create a configuration vector of all zeros
auto q = model.home_configuration();

// Compute the forward kinematics to the target frame for the source frame at the home configuration
auto H = forward_kinematics(model, q, "source_frame", "target_frame");
```
