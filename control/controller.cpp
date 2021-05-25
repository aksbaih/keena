// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "GenericController.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/model/panda/panda.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = JOINT_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {
    GenericController controller(robot_file);
    Matrix3d desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    Vector3d xd;
    xd << 0, -0.37, 0.763;
    controller.gotoPosition(xd, desired_rotation, false, 0.1, 0.1, "move");
    xd << 0, -0.37, 0.553;
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.1, "lower");
    xd << 0, -0.37, 0.763;
    controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.1, "grab");
    cout << "Controller finished" << endl;
    return 0;
}