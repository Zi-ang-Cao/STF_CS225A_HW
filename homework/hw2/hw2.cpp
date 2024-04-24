// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// **********************
// WRITE YOUR CODE HERE
// String formatting
#include <fstream>
#include <sstream>
#include <string>

// Customzied timmer
#include <iostream>
#include <chrono>
#include <thread>
// **********************


// sai2 main libraries includes
#include "Sai2Model.h"

// sai2 utilities from sai2-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

int main(int argc, char** argv) {
    Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER}-control {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file);

    // prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = VectorXd::Zero(dof);
    VectorXd robot_dq = VectorXd::Zero(dof);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq);

    // **********************
    // WRITE YOUR CODE HERE
    // Stream data to file
    std::ostringstream stream;
    stream << "../../homework/hw2/data_files/que_" << controller_number << ".txt";
    std::string file_name = stream.str();
    ofstream file_output;
    file_output.open(file_name);
    if (!file_output.is_open()) {cout << "Failed to open file: " << file_name << endl; exit(0);}
	else {cout << "File opened successfully" << endl;}

    // Start time point
    auto start = std::chrono::high_resolution_clock::now();
    // Set the desired loop duration as 3.0 seconds
    std::chrono::duration<double> timeout(5.0);
    // **********************

    while (runloop) {

        // **********************
        // Current time point
        auto now = std::chrono::high_resolution_clock::now();
        // Check the elapsed time
        if (now - start >= timeout) {
            std::cout << "5.0 seconds have passed. Exiting loop." << std::endl;
            break;
        }
        // **********************


        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {

            control_torques.setZero();

            file_output << time << "\t" << robot->q().transpose() << "\n";

            // For joint 1 to 6, use the following values:
            double kp = 400.0;
            double kv = 50.0;
            // For joint 7, use the following values:
            double kp7 = 50.0;
            // double kv7 = -0.250;  // modify this
            // double kv7 = -0.01;  // modify this
            // double kv7 = -0.05;  // modify this
            // double kv7 = -0.08;  // modify this



            double kv7 = -0.34;  // modify this --> 0.35 is the best value

            // Set q7d=0.1 rad
            double q7d = 0.1;  // modify this
            double m77 = 0.25;

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            // Only change the joint 7 to 0.1 rad!!!
            q_desired(6) = 0.1;  // modify this

            control_torques.setZero();
            // Use kp and kv for joint 1 to 6, kp7 and kv7 for joint 7, and m77 for the mass of the end-effector
            for (int i = 0; i < 6; i++) {
                control_torques(i) = -kp * (robot->q()(i) - q_desired(i)) - kv * robot->dq()(i) + 
                                    // robot->coriolisForce()(i) + robot->jointGravityVector()(i);
                                    robot->jointGravityVector()(i);

            }
            control_torques(6) = -kp7 * (robot->q()(6) - q7d) - kv7 * robot->dq()(6) + 
                                // robot->coriolisForce()(6) + robot->jointGravityVector()(6);
                                robot->jointGravityVector()(6);



            // control_torques(0:5) << (-kp * (robot->q()(0:5) - q_desired(0:5)) - kv * robot->dq()(0:5)) + robot->coriolisForce()(0:5) + robot->jointGravityVector()(0:5);
            // control_torques(6) = (-kp7 * (robot->q()(6) - q7d) - kv7 * robot->dq()(6)) + robot->coriolisForce()(6) + robot->jointGravityVector()(6);   
            // // control_torques.setZero();
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            control_torques.setZero();
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            control_torques.setZero();
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            control_torques.setZero();
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
