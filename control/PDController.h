
#ifndef PDCONTROLLER
#define PDCONTROLLER

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
     * this operation maintains any previous call regarding the grip of the fingers.
     */
    void gotoPosition(  const Vector3d position,
                        const Vector3d rpy,
                        double targetTolerance,
                        double timeWithinTolerance,
                        const string taskName = "");

    /* holdGrip holds all joints in place while moving the fingers to a closed position
     * applying a gripGain positional gain on the fingers. It returns after timeWithinTolerance
     * seconds of the fingers staying in the same position within a tolerance of equilibriumTolerance.
     */
    void holdGrip(  const double gripGain,
                    const double equilibriumTolerance = 0.0001,
                    const double timeWithinTolerance = 0.2,
                    const string taskName = "");

    /* same as holdGrip except that it holds it in the open position */
    void releaseGrip(   const double gripGain,
                        const double equilibriumTolerance = 0.0001,
                        const double timeWithinTolerance = 0.2,
                        const string taskName = "");


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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.2193);  // 0.0539 + 0.1654 height tip of middle finger

	// gains
    double kp = 200.0;
    double kv = 40.0;
    double kpj = 0.5 * kp;
    double kvj = 0.5 * kv;

	// grip variables
	const Vector2d openGrip = Vector2d(0.04, -0.04);
	const Vector2d closedGrip = Vector2d(0, 0);
	Vector2d desiredFingerPosition = openGrip;
	double gripGain = 0;
	const double gripKv = kvj;
    void enforceGrip(   const double equilibriumTolerance,
                        const double timeWithinTolerance);

    // utils
    Matrix3d rpyToMatrix(const Vector3d rpy);

};

#endif
