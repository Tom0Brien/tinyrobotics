![image](https://user-images.githubusercontent.com/41043317/231810128-a1c5257d-8b27-4a82-838d-a1c588b8e913.png)
---

**tinyrobotics** is a lightweight C++ library which provides core robotics algorithms for kinematics and dynamics.

The goal of **tinyrobotics** is to be as simple as possible while still being incredibly fast and versatile.

## Features
The core algorithms of tinyrobotics are listed below, for detailed documentation on all the available functions, see <a href="https://tom0brien.github.io/tinyrobotics">documentation</a>.
<h2><a href="https://tom0brien.github.io/tinyrobotics/structtr_1_1model_1_1Model.html#details">Model</a></h2>

| Function      | Description                                                                              |
| ------------- | ---------------------------------------------------------------------------------------- |
| `import_urdf` | Generate a tinyrobotics model from a [URDF](http://wiki.ros.org/urdf) robot description. |

<h2><a href="https://tom0brien.github.io/tinyrobotics/Kinematics_8hpp.html">Kinematics</a></h2>

| Function                 | Description                                                               |
| ------------------------ | -----------------------------------------------------------------         |
| `forward_kinematics`     | Compute homogeneous transform between links.                              |
| `inverse_kinematics`     | Solve joint positions for desired pose between links.                     |
| `jacobian`     | Compute geometric jacobian to a link from base.                           |
| `center_of_mass`         | Compute center of mass of model.                                          |

<h2><a href="https://tom0brien.github.io/tinyrobotics/Dynamics_8hpp.html">Dynamics</a></h2>

| Function           | Description                                                                     |
| ------------------ | ------------------------------------------------------------------------------  |
| `forward_dynamics` | Compute joint accelerations given joint positions, velocities and torques.      |
| `inverse_dynamics` | Compute joint torques given joint positions, velocities and accelerations.      |
| `mass_matrix`      | Compute mass matrix given joint positions.                                      |
| `kinetic_energy`   | Compute kinetic energy given joint positions and velocity.                      |
| `potential_energy` | Compute potential energy given joint positions and velocity.                    |
| `total_energy`     | Compute total energy (kinetic + potential) given joint positions and velocities.|

## Install

### 1. Install Dependencies
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [TinyXML2](https://github.com/leethomason/tinyxml2)
- [NLopt](https://github.com/stevengj/nlopt)

```bash
sudo apt install -y libeigen3-dev catch2 libtinyxml2-dev
```

### 2. Build with cmake
  ```bash
  git clone https://github.com/Tom0Brien/tinyrobotics.git && cd tinyrobotics
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in the include folder to /usr/local/include*
  ```

## Examples
Numerous examples are provided in the `examples` folder. 

The code below demonstrates how to load in a URDF model and compute the forward kinematics.
```c++
// Create a tinyrobotics model with 4 joints using URDF file defined in 4_link.urdf
auto model = import_urdf<double, 4>("4_link.urdf");

// Create a home configuration vector (all zeros).
auto q = model.home_configuration();

// Compute the forward kinematics to the link 2 from the base frame at the home configuration.
auto H = forward_kinematics(model, q, 2);
```
