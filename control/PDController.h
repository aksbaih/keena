
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>

using namespace std;
using namespace Eigen;

class PDController {

public:
    /* initializes a PD controller for the given robot with a velocity
     * saturation at maxVelocity
     */
    PDController(   const string robot_file,
                    double maxVelocity = 1.0);

    /* runs PD control to move the EE to the desired position and orientation
     * doesn't account for obstacles or joint limits // TODO: make it account for obstacles and joint limits
     */
    void gotoPosition(  const Vector3d position,
                        const Matrix3d orientation);


private:
    // redis and redis flags
	Redis::RedisClient redis_client;
	const string JOINT_ANGLES_KEY = "sai2::keena::sensors::q";
	const string JOINT_VELOCITIES_KEY = "sai2::keena::sensors::dq";
	const string JOINT_TORQUES_COMMANDED_KEY = "sai2::keena::actuators::fgc";

    // robot
	Sai2Model::Sai2Model robot;
	VectorXd initial_q;
	int dof;
	double maxVelocity;

	// end effector constants
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);


}