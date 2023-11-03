# cpp-interface-proxqp![Static Badge](https://img.shields.io/badge/status-experimental--beta-critical)

![](https://img.shields.io/badge/Tests-developer%20workflow-orange)![Static Badge](https://img.shields.io/badge/C%2B%2B-14-blue)![](https://img.shields.io/badge/Ubuntu%2022.04%20LTS%20(x64)-passing-passing)![](https://img.shields.io/badge/MacOS%2013.1%20(ARM64)%20-passing-passing)![Visitors](https://api.visitorbadge.io/api/visitors?path=https%3A%2F%2Fgithub.com%2Fjuanjqo%2Fcpp-interface-proxqp&label=visitors&countColor=%23ff8a65&style=flat)

Interface between DQ_Robotics and [proxqp](https://github.com/Simple-Robotics/proxsuite)

## Prerequisites

- DQ Robotics for C++ (master branch)

  ```shell
  sudo add-apt-repository ppa:dqrobotics-dev/development
  sudo apt-get update
  sudo apt-get install libdqrobotics
  ```

## 1. Recommended installation process for proxqp

Check the official PROXSuite [instructions](https://github.com/Simple-Robotics/proxsuite/blob/main/doc/5-installation.md) 

Summary (MacOS using homebrew)

```shell
brew install proxsuite
```

Summary (Ubuntu: building from sources with the vectorization support disabled):

```shell
cd ~/Downloads
git clone https://github.com/Simple-Robotics/proxsuite.git --recursive
cd proxsuite
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF
make
sudo make install
```

## 2. Build the cpp-interface-proxqp from sources (Tested on MacOS ARM64 and Ubuntu {20.04LTS-64Bits, 22.04LTS-64Bits})


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
  sudo dpkg -i ./*.deb
```



### Example of use:

#### Using the DQ_ClassicQPController class

```CPP
#include <memory>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

using namespace Eigen;
using namespace DQ_robotics;

int main(){
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
}
```

```shell
u_proxqp:       0.107115    0.147883    0.107115    0.029608    0.107115   -0.261227 2.66176e-17
```

#### Using the DQ_PROXQPSolver interface

```cpp
#include <memory>
#include <dqrobotics/solvers/DQ_PROXQPSolver.h>

using namespace Eigen;
using namespace DQ_robotics;

int main(){
    auto proxqp_solver = std::make_shared<DQ_PROXQPSolver>();

    MatrixXd H = MatrixXd::Zero(3,3);
    H << 1,-1, 1,
        -1, 2,-2,
        1,-2, 4;

    VectorXd f = VectorXd::Zero(3);
    f << 2,-3, 1;

    VectorXd lb = VectorXd::Zero(3);
    VectorXd ub = VectorXd::Ones(3);

    MatrixXd Aeq = MatrixXd::Ones(1,3);
    VectorXd beq = VectorXd::Ones(1);
    beq(0) = 0.5;

    MatrixXd A(6,3);
    VectorXd b(6);
    MatrixXd I = MatrixXd::Identity(3,3);
    A << I, -I;
    b << ub, -lb;

    auto u = proxqp_solver->solve_quadratic_program(H, f, A, b, Aeq, beq);
    std::cout<<"u: "<<u.transpose()<<std::endl;
}
```

```shell
u: -9.84188e-17          0.5  -2.6844e-17
```

### CMakeLists.txt:

```cmake
add_executable(my_example my_example.cpp)
target_link_libraries(my_example
                      Threads::Threads
                      dqrobotics)
```
