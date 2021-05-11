

#include "PDController.h"


using namespace std;
using namespace Eigen;


PDController::PDController( const string robot_file,
                            double vmax) {
	// set attributes
	maxVelocity = vmax;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load robot
	robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	initial_q = robot->_q;
	dof = robot->dof();
	robot->updateModel();
}

void PDController::gotoPosition(const Vector3d position,
                                const Vector3d rpy,
                                double targetTolerance,
                                double timeWithinTolerance,
                                const string taskName) {
    assert(timeWithinTolerance > 0);
    cout << "Task " << taskName << " started." << endl;
   	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);
	auto sat = [](double val) { return abs(val) <= 1 ? val : val / abs(val); };

	// initialize the task
	Matrix3d orientation = rpyToMatrix(rpy);
	VectorXd command_torques = VectorXd::Zero(dof);
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	double latestOutTolerance = start_time;
	bool fTimerDidSleep = true;
	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// task loop
    while (true) {
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

        Vector3d x; robot->position(x, link_name, pos_in_link);
        if((x - position).norm() > targetTolerance) latestOutTolerance = time;
        if(time - latestOutTolerance >= timeWithinTolerance) break;
        Vector3d dx; robot->linearVelocity(dx, link_name, pos_in_link);
        Vector3d xd = position;
        Vector3d dxd = (kp / kv) * (xd - x);
        double v = sat(maxVelocity / dxd.norm());
        Vector3d F = Lambda * (-kv * (dx - v * dxd));
        VectorXd postureControl = robot->_M * (-kpj * (robot->_q /* - 0 */) -kvj * robot->_dq);
        command_torques = Jv.transpose() * F + N.transpose() * postureControl + g;

        // gripper control
        command_torques.tail(2) = -gripGain * (robot->_q.tail(2) - desiredFingerPosition) -gripKv * robot->_dq.tail(2) + g.tail(2);

        // send controller results to simulation
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    // stop control after the task finishes
    command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");
    cout << "Task " << taskName << " finished." << endl;
}

void PDController::enforceGrip( const double equilibriumTolerance,
                                const double timeWithinTolerance) {
	VectorXd command_torques = VectorXd::Zero(dof);
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	double latestOutTolerance = start_time;
	bool fTimerDidSleep = true;
	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// task loop
    while (true) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        // read robot state from redis
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->updateModel();
        VectorXd g(dof); robot->gravityVector(g);

        // track target tolerance
        if((robot->_q.tail(2) - desiredFingerPosition).norm() > equilibriumTolerance) latestOutTolerance = time;
        if(time - latestOutTolerance >= timeWithinTolerance) break;

        // gripper control
        command_torques.tail(2) = -gripGain * (robot->_q.tail(2) - desiredFingerPosition) -gripKv * robot->_dq.tail(2) + g.tail(2);

        // send controller results to simulation
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    // stop control after the task finishes
    command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");
}

void PDController::holdGrip(const double desiredGripGain,
                            const double equilibriumTolerance,
                            const double timeWithinTolerance,
                            const string taskName) {
    assert(timeWithinTolerance > 0);
    cout << "Task " << taskName << " started." << endl;

	// initialize the task
	desiredFingerPosition = closedGrip;
	gripGain = desiredGripGain;
	enforceGrip(equilibriumTolerance, timeWithinTolerance);

    cout << "Task " << taskName << " finished." << endl;
}

void PDController::releaseGrip( const double desiredGripGain,
                                const double equilibriumTolerance,
                                const double timeWithinTolerance,
                                const string taskName){
    assert(timeWithinTolerance > 0);
    cout << "Task " << taskName << " started." << endl;

	// initialize the task
	desiredFingerPosition = openGrip;
	gripGain = desiredGripGain;
	enforceGrip(equilibriumTolerance, timeWithinTolerance);

    cout << "Task " << taskName << " finished." << endl;
}


Matrix3d PDController::rpyToMatrix(const Vector3d rpy) {
    Vector3d c = rpy.array().cos();
    Vector3d s = rpy.array().sin();

    Matrix3d R;
    R << c(1) * c(2), (c(1) * s(2) * s(3)) - (c(3) * s(1)), (s(1) * s(3)) + (c(1) * c(3) * s(2)),
         c(2) * s(1), (c(1) * c(3)) + (s(1) * s(2) * s(3)), (c(3) * s(1) * s(2)) - (c(1) * s(3)),
         -s(2), c(2) * s(3), c(2) * c(3);

    return R;
}