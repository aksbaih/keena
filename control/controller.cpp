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
    // Set up where the Legos are and where they need to go!
	MatrixXd legoStart = MatrixXd::Zero(3, 3);
	MatrixXd legoEnd = MatrixXd::Zero(3, 3);
	VectorXd legoEndYawOffset = VectorXd::Zero(3); 
	legoStart << -0.22, -0.396, 0.572,-0.22, -0.396, 0.552,-0.225, -0.452, 0.552;
	legoEnd   <<   0.070, -0.429, 0.546,-0.052, -0.429, 0.546,-0.088, -0.462, 0.546;
        legoEndYawOffset << M_PI/2, M_PI/2,0; 
    GenericController controller(robot_file);
    Matrix3d desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    cout << legoEndYawOffset(0);
    //cout << desired_rotation;
    Vector3d verticalOffset;
    verticalOffset(0) = 0.0;
    verticalOffset(1) = 0.0;
    verticalOffset(2) = 0.1;
        Vector3d verticalOffset_2;
    verticalOffset_2(0) = 0.0;
    verticalOffset_2(1) = 0.0;
    verticalOffset_2(2) = 0.008;
    Vector3d xd;
    xd << -0.016, -0.35, 0.763;
    
    controller.gotoPosition(xd, desired_rotation, false, 0.001, 0.1, "move");
    
    xd = legoStart.row(0).transpose() + verticalOffset ;
    
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.1, "move");
//    xd << -0.016, -0.35, 0.546;
//    controller.gotoPosition(xd, desired_rotation, false, 0.0001, 0.1, "lower");
//    xd << -0.016, -0.35, 0.763;
//    controller.gotoPosition(xd, desired_rotation, true, 0.0001, 0.1, "grab");
//    xd << 0.23, -0.355, 0.763;
 //   controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.1, "move");
//    xd << 0.23, -0.355, 0.540;
//    controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.1, "move");
//    xd << 0.23, -0.355, 0.546;
 //   controller.gotoPosition(xd, desired_rotation, false, 0.0001, 0.1, "lower");
 //   xd << 0.23, -0.355, 0.75;
  //  controller.gotoPosition(xd, desired_rotation, false, 0.0001, 0.1, "move");
   // cout << "Controller finished" << endl;
    
    //Start loop here
    
    for (int i = 0; i < 3; i++){
    
    cout << "I am here";
    
    xd = legoStart.row(i).transpose() + verticalOffset ;
    if (i ==2) {
    desired_rotation = AngleAxisd(M_PI/4 +M_PI/2 , Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    }
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.1, "move");

    
    xd = legoStart.row(i).transpose() ; //go to LEGO
    desired_rotation = AngleAxisd(M_PI/4 , Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    cout << "Lowering To LEGO";
        if (i ==2) {
    desired_rotation = AngleAxisd(M_PI/4 +M_PI/2 , Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    }
    controller.gotoPosition(xd, desired_rotation, false, 0.005, 0.15, "lower");
    
    xd = legoStart.row(i).transpose()  + verticalOffset;
    desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
     cout << "Picking Up The LEGO"; 
    controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.15, "grab");

    desired_rotation = AngleAxisd(M_PI/4 + legoEndYawOffset(i), Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd = legoEnd.row(i).transpose() + verticalOffset;
    controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.15, "move");


    desired_rotation = AngleAxisd(M_PI/4 + legoEndYawOffset(i) , Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd = legoEnd.row(i).transpose();
     cout << "Lowering The LEGO";
    controller.gotoPosition(xd, desired_rotation , true, 0.01, 0.15, "move");



    desired_rotation = AngleAxisd(M_PI/4 + legoEndYawOffset(i), Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd = legoEnd.row(i).transpose();
    controller.gotoPosition(xd, desired_rotation , false, 0.005, 0.15, "lower");



    desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd = legoEnd.row(i).transpose() + verticalOffset;
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.15, "move");
    //cout << "next piece please!";
    
//        desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
///    xd = legoEnd.row(i).transpose() + verticalOffset_2;
 //   controller.gotoPosition(xd, desired_rotation, true, 0.01, 0.15, "lower");
    //cout << "next piece please!";
    

    desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd = legoEnd.row(i).transpose() + verticalOffset;
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.15, "move");
    cout << "next piece please!";
    
   
    }
    
        desired_rotation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).matrix() * AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
    xd << -0.016, -0.35, 0.763;
    controller.gotoPosition(xd, desired_rotation, false, 0.01, 0.15, "move");
    cout << "next piece please!";
    
   
   
    //go through again and push each piece down lol. 
    cout << "Controller finished" << endl;
    return 0;
}


