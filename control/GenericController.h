#ifndef GENERICCONTROLLER
#define GENERICCONTROLLER

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include <float.h>
#include <iostream>
#include <string>
#include <signal.h>

using namespace std;
using namespace Eigen;

class GenericController {

public:
    /* loads the given robot and prepares it for control through a redis client
     */
    GenericController(const string robot_file);

    /* moves the end effector to a desired position and rotation (in world coordinates) after
     * achieving the desired grip. It stops once the goal is within the provided tolerances
     */
    void gotoPosition(  const Vector3d desired_position,
                        const Matrix3d desired_rotation,
                        const bool desired_grip,            // true means closed grip
                        const double positionalTolerance,   // norm position
                        const double rotationalTolerance,   // norm rotation
                        const string taskName = "");


private:
    // redis and redis flags
	RedisClient redis_client;
	const string JOINT_ANGLES_KEY = "sai2::keena::sensors::q";
	const string JOINT_VELOCITIES_KEY = "sai2::keena::sensors::dq";
	const string JOINT_TORQUES_COMMANDED_KEY = "sai2::keena::actuators::fgc";
    const string CONTROLLER_RUNING_KEY = "sai2::keena::controller_running";
    const string SIMULATION_LOOP_DONE_KEY = "sai2::keena::simulation::done";
    const string CONTROLLER_LOOP_DONE_KEY = "sai2::keena::controller::done";
    // robot
	Sai2Model::Sai2Model *robot;
	VectorXd initial_q;
	int dof;
	double maxVelocity;
	// end effector constants
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.2193);  // 0.0539 + 0.1654 height tip of middle finger
	const Vector3d base_position = Vector3d(0.0, 0.1, 0.1); // from world.urdf
	const Vector3d base_rpy = Vector3d(0, 0, 0);    // from world.urdf
	// grip constants
	const Vector2d openGrip = Vector2d(0, 0);
	const Vector2d closedGrip = Vector2d(-0.04, 0.04);
    // equilibrium constants
    const double gripEquilibriumVelocity = 0.15;
    const double gripEquilibriumDuration = 0.5;
    const double positionalEquilibriumVelocity = 0.01;
    const double positionalEquilibriumAngularVelocity = 0.01;
    const double positionalEquilibriumDuration = 0.5;
};

#endif
