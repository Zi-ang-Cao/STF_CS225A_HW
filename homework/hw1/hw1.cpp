// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// String formatting
#include <fstream>
#include <sstream>
#include <string>

// Customzied timmer
#include <iostream>
#include <chrono>
#include <thread>

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
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm_controller.urdf";

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
        else if (controller_number < 1 || controller_number > 5) {
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
    VectorXd control_torques = VectorXd::Zero(dof);

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = VectorXd::Zero(dof);
    VectorXd robot_dq = VectorXd::Zero(dof);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);

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

    // Stream data to file
    std::ostringstream stream;
    stream << "../../homework/hw1/data_files/que_" << controller_number << ".txt";
    std::string file_name = stream.str();

    ofstream file_output;
    file_output.open(file_name);
    if (!file_output.is_open()) {cout << "Failed to open file: " << file_name << endl; exit(0);}
	else {cout << "File opened successfully" << endl;}

    // Start time point
    auto start = std::chrono::high_resolution_clock::now();
    // Set the desired loop duration as 1.2 seconds
    std::chrono::duration<double> timeout(1.2);


    while (runloop) {
        // Current time point
        auto now = std::chrono::high_resolution_clock::now();

        // Check the elapsed time
        if (now - start >= timeout) {
            std::cout << "1.2 seconds have passed. Exiting loop." << std::endl;
            break;
        }


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

            double kp = 400.0;      // chose your p gain
            double kv = 47.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, - M_PI * 125/180, 0, M_PI * 80/180, 0; // modify this

            control_torques << -kp * (robot->q() - q_desired) - kv * robot->dq();

            // cout << "debug print" << endl;
            // cout << "Init q: " << initial_q(0) << initial_q(1) << initial_q(3) << endl;
            // cout << "Current q: "<< robot->q()(0) << robot->q()(1) << robot->q()(3) << endl;
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            control_torques.setZero();
            file_output << time << "\t" << robot->q().transpose() << "\n";

            double kp = 400.0;      // chose your p gain
            double kv = 49.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, - M_PI * 125/180, 0, M_PI * 80/180, 0; // modify this

            control_torques << -kp * (robot->q() - q_desired) - kv * robot->dq() + robot->jointGravityVector();
            
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            control_torques.setZero();
            file_output << time << "\t" << robot->q().transpose() << "\n";

            double kp = 400.0;      // chose your p gain
            double kv = 43.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, - M_PI * 125/180, 0, M_PI * 80/180, 0; // modify this

            control_torques << robot->M() * (-kp * (robot->q() - q_desired) - kv * robot->dq()) + robot->jointGravityVector();
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            control_torques.setZero();
            file_output << time << "\t" << robot->q().transpose() << "\n";

            double kp = 400.0;      // chose your p gain
            // double kv = 40.0;      // chose your d gain
            double kv = 45.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, - M_PI * 125/180, 0, M_PI * 80/180, 0; // modify this

            control_torques << robot->M() * (-kp * (robot->q() - q_desired) - kv * robot->dq()) + robot->coriolisForce() + robot->jointGravityVector();
        }

        // ---------------------------  question 5 ---------------------------------------
        else if(controller_number == 5) {

            control_torques.setZero();
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.setInt("sai2::interfaces::simviz::gravity_comp_enabled", 0);
        redis_client.sendAllFromGroup();
    }

    file_output.close();

    control_torques.setZero();
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
