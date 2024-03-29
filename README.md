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

### 1. Install dependencies
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Catch2](https://github.com/catchorg/Catch2)
- [TinyXML2](https://github.com/leethomason/tinyxml2)
- [NLopt](https://github.com/stevengj/nlopt)

```bash
sudo apt install -y libeigen3-dev catch2 libtinyxml2-dev
```

### 2. Build and install with cmake
  ```bash
  git clone https://github.com/Tom0Brien/tinyrobotics.git && cd tinyrobotics
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in the include folder to /usr/local/include*
  ```

## Example
The code below demonstrates how to generate a model via a URDF and use kinematics and dynamics functions.
Numerous other examples are provided in the `examples` folder. 

```c++
// Parse URDF
const int n_joints      = 5;
auto model              = import_urdf<double, n_joints>("5_link.urdf");

// Create test configuration, velocity, acceleration and torque vectors
auto q = model.random_configuration();
auto dq = Eigen::Matrix<double, n_joints, 1>::Zero();
auto ddq = Eigen::Matrix<double, n_joints, 1>::Zero();
auto tau = Eigen::Matrix<double, n_joints, 1>::Zero();

// Forward Kinematics
std::string source_link = "base";
std::string target_link = "end_effector";
auto H = forward_kinematics(model, q, target_link, source_link);

// Center of Mass
auto com = center_of_mass(model, q);

// Inverse Kinematics
InverseKinematicsOptions<double, n_joints> options;
options.max_iterations = 1000;
options.tolerance      = 1e-4;
options.method         = InverseKinematicsMethod::LEVENBERG_MARQUARDT;
auto q_solution             = inverse_kinematics(model, target_link, source_link, H, q, options);

// Jacobian
auto J = jacobian(model, q, target_link);

// Forward Dynamics
auto acceleration = forward_dynamics(model, q, dq, tau);

// Inverse Dynamics
auto torque = inverse_dynamics(model, q, dq, ddq);

// Mass Matrix
auto M = mass_matrix(model, q);

// Kinetic Energy
auto T = kinetic_energy(model, q, dq);

// Potential Energy
auto V = potential_energy(model, q);

// Total Energy
auto E = total_energy(model, q, dq);
```
