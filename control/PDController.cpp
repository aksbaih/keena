

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

void PDController::gotoPosition(const Vector3d absolute_position,
                                const Matrix3d desired_rotation,
                                double targetTolerance,
                                double timeWithinTolerance,
                                const string taskName) {
    bool grip = true;
    assert(timeWithinTolerance > 0);
    cout << "Task " << taskName << " started." << endl;
//   	// model quantities for operational space control
//	MatrixXd Jv = MatrixXd::Zero(3,dof);
//	MatrixXd Lambda = MatrixXd::Zero(3,3);
//	MatrixXd N = MatrixXd::Zero(dof,dof);
//	auto sat = [](double val) { return abs(val) <= 1 ? val : val / abs(val); };
//
//	// initialize the task
	Vector3d desired_position = absolute_position - base_position;
//	VectorXd command_torques = VectorXd::Zero(dof);
//	LoopTimer timer;
//	timer.initializeTimer();
//	timer.setLoopFrequency(1000);
//	double start_time = timer.elapsedTime(); //secs
//	double latestOutTolerance = start_time;
//	bool fTimerDidSleep = true;
//	redis_client.set(CONTROLLER_RUNING_KEY, "1");
//
//	// task loop
//    while (true) {
//        // wait for next scheduled loop
//        timer.waitForNextLoop();
//        double time = timer.elapsedTime() - start_time;
//
//        // read robot state from redis
//        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
//        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
//        robot->updateModel();
////        robot->Jv(Jv, link_name, pos_in_link);
////        robot->nullspaceMatrix(N, Jv);
////        robot->taskInertiaMatrix(Lambda, Jv);
//        MatrixXd J_0(6, dof); robot->J_0(J_0, link_name, pos_in_link);
//        MatrixXd Lambda_0(6, 6); robot->taskInertiaMatrix(Lambda_0, J_0);
//		MatrixXd N_0(dof, dof); robot->nullspaceMatrix(N_0, J_0);
//        VectorXd g(dof); robot->gravityVector(g);
//
//        Vector3d x; robot->position(x, link_name, pos_in_link);
//        if((x - position).norm() > targetTolerance) latestOutTolerance = time;
//        if(time - latestOutTolerance >= timeWithinTolerance) break;
//        Vector3d dx; robot->linearVelocity(dx, link_name, pos_in_link);
//        Vector3d xd = position;
//        Vector3d dxd = (kp / kv) * (xd - x);
//        double v = sat(maxVelocity / dxd.norm());
//        Vector3d linearControl = -kv * (dx - v * dxd);
//
//        Matrix3d R; robot->rotation(R, link_name);
//        Vector3d omega; robot->angularVelocity(omega, link_name, pos_in_link);
//        Vector3d delta_phi = Vector3d::Zero(); for(int i=0; i<3; i++) delta_phi += -0.5 * R.col(i).cross(Rd.col(i));
//        Vector3d angularControl = kp * (-delta_phi) -kv * omega;
//
//        VectorXd totalControl(6); totalControl << linearControl, angularControl;
//        VectorXd F = Lambda_0 * totalControl;
//        VectorXd postureControl = robot->_M * (-kpj * (robot->_q /* - 0 */) -kvj * robot->_dq);
//        command_torques = J_0.transpose() * F + N_0.transpose() * postureControl + g;
//
//        // gripper control
//        command_torques.tail(2) = -gripGain * (robot->_q.tail(2) - desiredFingerPosition) -gripKv * robot->_dq.tail(2) + g.tail(2);
//
//        // send controller results to simulation
//		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
//    }
//
//    // stop control after the task finishes
//    command_torques.setZero();
//	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
//	redis_client.set(CONTROLLER_RUNING_KEY, "0");
//    cout << "Task " << taskName << " finished." << endl;

    #define JOINT_CONTROLLER      0
    #define POSORI_CONTROLLER     1

    int state = JOINT_CONTROLLER;


    unsigned long long controller_counter = 0;

    const bool inertia_regularization = true;
	// load robots
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);

    #ifdef USING_OTG
        posori_task->_use_interpolation_flag = true;
    #else
        posori_task->_use_velocity_saturation_flag = true;
    #endif

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

    posori_task->_desired_position = desired_position;
    posori_task->_desired_orientation = desired_rotation ;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

    #ifdef USING_OTG
        joint_task->_use_interpolation_flag = true;
    #else
        joint_task->_use_velocity_saturation_flag = true;
    #endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired.tail(2) = grip ? closedGrip : openGrip;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (true) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_init_desired).norm() < 0.01 )
			{
			    cout << "joint space accomplished. now operational space" << endl;
				posori_task->reInitializeTask();
				posori_task->_desired_position = desired_position;
				posori_task->_desired_orientation = desired_rotation ;//* posori_task->_desired_orientation;

//				joint_task->reInitializeTask();
//				joint_task->_kp = 0;

				state = POSORI_CONTROLLER;
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
		    joint_task->reInitializeTask();
		    q_init_desired = robot->_q;
            q_init_desired.tail(2) = grip ? closedGrip : openGrip;
            joint_task->_desired_position = q_init_desired;

			// update task model and set hierarchy
			N_prec.setIdentity();
			N_prec.bottomRightCorner<2,2>().setZero();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			N_prec.bottomRightCorner<2,2>().setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

//            posori_task_torques.tail(2) *= 0;
			command_torques = posori_task_torques + joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    return q.matrix();
}