
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
                    double maxVelocity = 4.0);

    /* runs PD control to move the EE to the desired position and orientation
     * doesn't account for obstacles or joint limits // TODO: make it account for obstacles and joint limits
     * it stops when the end effector is within targetTolerance from the target
     * and stays there for timeWithinTolerance seconds.
     */
    void gotoPosition(  const Vector3d position,
                        const Matrix3d orientation,
                        double targetTolerance,
                        double timeWithinTolerance,
                        string taskName = "");


private:
    // redis and redis flags
	RedisClient redis_client;
	const string JOINT_ANGLES_KEY = "sai2::keena::sensors::q";
	const string JOINT_VELOCITIES_KEY = "sai2::keena::sensors::dq";
	const string JOINT_TORQUES_COMMANDED_KEY = "sai2::keena::actuators::fgc";
    const string CONTROLLER_RUNING_KEY = "sai2::keena::controller_running";

    // robot
	Sai2Model::Sai2Model *robot;
	VectorXd initial_q;
	int dof;
	double maxVelocity;

	// end effector constants
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);

};
