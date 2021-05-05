

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;


PDController::PDController( const string robot_file,
                            double vmax = 1.0) {
	// set attributes
	maxVelocity = vmax;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robot
	robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	initial_q = robot->_q;
	dof = robot->dof();
	robot->updateModel();
}


void PDController::gotoPosition(const Vector3d position,
                                const Matrix3d orientation) {
	// initialize the task
	VectorXd command_torques = VectorXd::Zero(dof);
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// task loop
    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        // read robot state from redis
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->updateModel();
        robot->Jv(Jv, link_name, pos_in_link);
        robot->nullspaceMatrix(N, Jv);
        robot->taskInertiaMatrix(Lambda, Jv);
        VectorXd g(dof); robot->gravityVector(g);

        double kp = 200.0;
        double kv = 40.0;
        double kpj = 0.5 * kp;
        double kvj = 0.5 * kv;

        Vector3d x; robot->position(x, link_name, pos_in_link);
        Vector3d dx; robot->linearVelocity(dx, link_name, pos_in_link);
        Vector3d xd = position;
        Vector3d dxd = (kp / kv) * (xd - x);
        double v = sat(maxVelocity / dxd.norm());
        Vector3d F = Lambda * (-kv * (dx - v * dxd));
        VectorXd postureControl = robot->_M * (-kpj * (robot->_q /* - 0 */) -kvj * robot->_dq);
        command_torques = Jv.transpose() * F + N.transpose() * postureControl + g;

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    // stop control after the task finishes
    command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");
}
