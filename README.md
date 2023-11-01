# cpp-interface-proxqp![](https://img.shields.io/badge/status-experimental-critical)
![](https://img.shields.io/badge/Tests-developer%20workflow-orange)![](https://img.shields.io/badge/Ubuntu%2022.04%20LTS%20(x64)-Unknown-yellow)![](https://img.shields.io/badge/MacOS%2013.1%20(ARM64)%20-passing-passing)

Interface between DQ_Robotics and [proxqp](https://github.com/Simple-Robotics/proxsuite)

## 1. Recommended installation process for proxqp

Check the official PROXSuite [instructions](https://github.com/Simple-Robotics/proxsuite/blob/main/doc/5-installation.md) 

Summary (homebrew for MacOS)

```shell
brew install proxsuite
```

Summary (building from sources - Ubuntu):

```shell
cd ~/Downloads
git clone https://github.com/Simple-Robotics/proxsuite.git --recursive
cd proxsuite
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF
make
sudo make install
```

## 2. Build the cpp-interface-proxqp from sources (Tested in MacOS ARM64 and Ubuntu 20.04LTS 64Bits)


### MacOS

```shell
cd ~/Downloads
git clone https://github.com/juanjqo/cpp-interface-proxqp.git
cd cpp-interface-proxqp
mkdir build
cd build
cmake ..
make -j16
sudo make install
```

### Ubuntu
```shell
  cd ~/Downloads
  git clone https://github.com/juanjqo/cpp-interface-proxqp.git
  cd cpp-interface-proxqp
  chmod +x debian/rules
  fakeroot debian/rules clean
  fakeroot debian/rules build
  fakeroot debian/rules binary
  cd ..
  sudo apt install ./*.deb
```



### Example of use:

```CPP
#include <memory>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

using namespace Eigen;
using namespace DQ_robotics;

auto robot = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
auto proxqp_solver = std::make_shared<DQ_PROXQPSolver>();

DQ_ClassicQPController controller_proxqp(robot, proxqp_solver);
controller_proxqp.set_gain(0.5);
controller_proxqp.set_damping(0.1);
controller_proxqp.set_control_objective(DQ_robotics::Translation);
controller_proxqp.set_stability_threshold(0.001);

DQ xdesired = 1 + E_*0.5*DQ(0, 0.2, 0.3, 0.3);
VectorXd q = VectorXd::Zero(7);
auto u_proxqp = controller_proxqp.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
std::cout<<"u_proxqp:    "<<u_proxqp.transpose()<<std::endl;
```

### CMakeLists.txt:

```cmake
add_executable(my_example my_example.cpp)
target_link_libraries(my_example
                      Threads::Threads
                      dqrobotics)
```
