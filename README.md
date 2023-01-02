# cpp-interface-osqp

Interface between DQ_Robotics and [OSQP](https://github.com/osqp/osqp)

## 1. Recommended installation process for OSQP

Check the official OSQP [instructions](https://osqp.org/docs/get_started/sources.html) 

Summary (Ubuntu):

```shell
cd ~/Downloads
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
cmake --build .
cmake --build . --target install
```


## 2. Build the cpp-interface-osqp from sources (Tested only in MacOS ARM64)

```shell
cd ~/Downloads
git clone https://github.com/juanjqo/cpp-interface-osqp.git
cd cpp-interface-osqp
mkdir build
cd build
cmake ..
make -j16
sudo make install
```

Example of use:

```CPP
#include <memory>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_OSQPSolver.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

using namespace Eigen;
using namespace DQ_robotics;

auto robot = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
auto osqp_solver   = std::make_shared<DQ_OSQPSolver>(DQ_OSQPSolver());

DQ_ClassicQPController controller_osqp(robot, osqp_solver);
controller_osqp.set_gain(0.5);
controller_osqp.set_damping(0.1);
controller_osqp.set_control_objective(DQ_robotics::Translation);
controller_osqp.set_stability_threshold(0.001);

DQ xdesired = 1 + E_*0.5*DQ(0, 0.2, 0.3, 0.3);
VectorXd q = VectorXd::Zero(7);
auto u_osqp = controller_osqp.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
std::cout<<"u_osqp:    "<<u_osqp.transpose()<<std::endl;
```
