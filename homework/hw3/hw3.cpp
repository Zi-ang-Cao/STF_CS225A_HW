// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// sai2 main libraries includes
#include "Sai2Model.h"

// sai2 utilities from sai2-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
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
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
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
    stream << "../../homework/hw3/data_files/que_" << controller_number << ".txt";
    std::string file_name = stream.str();
    ofstream file_output;
    file_output.open(file_name);
    if (!file_output.is_open()) {cout << "Failed to open file: " << file_name << endl; exit(0);}
	else {cout << "File opened successfully" << endl;}

    // Start time point
    double duration = 10.0;
    if(controller_number == 1) {duration = 10.0;}
    else if(controller_number == 2) {duration = 1.8;}
    else if(controller_number == 3) {duration = 3.0;}
    else if(controller_number >= 4) {duration = 6.0;}

    auto start = std::chrono::high_resolution_clock::now();
    // Set the desired loop duration as 3.0 seconds
    std::chrono::duration<double> timeout(duration);
    // **********************

    while (runloop) {

        // **********************
        // Current time point
        auto now = std::chrono::high_resolution_clock::now();
        // Check the elapsed time
        if (now - start >= timeout) {
            std::cout << duration << " seconds have passed. Exiting loop." << std::endl;
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

            // operational space control
            VectorXd x = robot->position(link_name, pos_in_link);
            VectorXd x_desired = x;
            x_desired << 0.3 + 0.1 * (sin(M_PI * time)), 0.1 + 0.1 * (cos(M_PI * time)), 0.5;
            VectorXd dx = robot->linearVelocity(link_name, pos_in_link);
            // joint space control
            VectorXd q_desired = initial_q;
            q_desired << 0, 0, 0, 0, 0, 0, 0;

            // Logging data
            file_output << time << "\t" << x.transpose() << "\t" << x_desired.transpose() << "\t" << robot->q().transpose() << "\n";

            double kp = 100.0;
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;

            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);
            N = robot->nullspaceMatrix(Jv);
            VectorXd g = robot->jointGravityVector();

            // // ------ (1a) PD ------
            // VectorXd F = Lambda * (-kp * (x - x_desired) - kv * dx);

            // ------ (1c) PD + desired 1st and 2nd direvative in joint space  ------
            VectorXd x_desired_dot = VectorXd::Zero(3);
            double scale = M_PI * 0.1;
            x_desired_dot << scale * cos(M_PI * time), -scale * sin(M_PI * time), 0;
            VectorXd x_desired_ddot = VectorXd::Zero(3);
            scale = M_PI * M_PI * 0.1;
            x_desired_ddot << -scale * sin(M_PI * time), -scale * cos(M_PI * time), 0;

            VectorXd F = Lambda * (x_desired_ddot -kp * (x - x_desired) - kv * (dx - x_desired_dot));

            // ------ IN COMMON control_torques ------
            control_torques.setZero();
            control_torques = Jv.transpose() * F + N.transpose() * (- kpj * (robot_q - q_desired) - kvj * robot_dq) + g;  
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            int for_Que_de = 0; // 0 for Que f, g

            control_torques.setZero();

            double kp = 100.0;
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;
            
            // joint space control
            double k_mid = 25.0;
            double k_damp = 14.0;

            VectorXd q_low = initial_q;
            VectorXd q_high = initial_q;
            q_low << -165, -100, -165, -170, -165, 0, -165;
            q_high << 165, 100, 165, -30, 165, 210, 165;
            q_low = q_low * M_PI/180;   // change [degree] to [radians]
            q_high = q_high * M_PI/180; // change [degree] to [radians]

            VectorXd tao_mid = VectorXd::Zero(dof);
            VectorXd tao_damp = VectorXd::Zero(dof);
            tao_mid = k_mid * ((q_low + q_high) - 2 * robot_q);
            tao_damp = - k_damp * robot_dq;

            // operational space control
            VectorXd x = robot->position(link_name, pos_in_link);
            VectorXd x_desired = x;
            if (for_Que_de == 1) {x_desired << -0.1, 0.15, 0.2;}
            else {x_desired << -0.65, -0.45, 0.7;}
            VectorXd dx = robot->linearVelocity(link_name, pos_in_link);

            // ----- Save into the file -----
            file_output << time << "\t" << x.transpose() << "\t" << x_desired.transpose() << "\t" << robot->q().transpose() << "\n";

            // ----- In Common -----
            control_torques.setZero();
            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);
            N = robot->nullspaceMatrix(Jv);

            // operational space control
            VectorXd F = Lambda * (-kp * (x - x_desired) - kv * dx);
            VectorXd g = robot->jointGravityVector();

            // // ----- (2d) -----
            // control_torques = Jv.transpose() * F + N.transpose() * tao_damp + g;
            
            // // ----- (2e) (2f) -----
            // control_torques = Jv.transpose() * F + N.transpose() * tao_mid + N.transpose() * tao_damp + g;

            // // ----- (2g) -----
            control_torques = Jv.transpose() * F + tao_mid + N.transpose() * tao_damp + g;

        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            
            // Control parameters
            double kp = 100.0;
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;

            // Desired position and orientation
            Vector3d x_desired = Vector3d(0.6, 0.3, 0.5);

            MatrixXd R = robot->rotation(link_name);
            MatrixXd R_desired = MatrixXd::Identity(3,3);
            R_desired << cos(M_PI /3), 0, sin(M_PI / 3),
                         0, 1, 0,
                         -sin(M_PI / 3), 0, cos(M_PI / 3);

            Vector3d delta_phi = Vector3d::Zero();
            for (int i = 0; i < 3; i++) {
                Vector3d R_i = R(all, i);
                Vector3d R_desired_i = R_desired(all, i);
                // cout << Ri.cross(Rdi).transpose() << endl;
                delta_phi += R_i.cross(R_desired_i);
            }
            delta_phi = -0.5 * delta_phi;

            // Full Jacobian! [Jv, Jw]
            MatrixXd J_0 = robot->J(link_name, pos_in_link);
            
            // Joint Null Space Control
            N = robot->nullspaceMatrix(J_0);

            // Activate Control law
            VectorXd x = robot->position(link_name, pos_in_link);
            VectorXd dx = robot->linearVelocity(link_name, pos_in_link);
            MatrixXd Lambda_0 = robot->taskInertiaMatrix(J_0);

            VectorXd Fv = VectorXd::Zero(3);
            Fv = kp * (x_desired - x) - kv * dx;
            VectorXd Fw = VectorXd::Zero(3);
            Fw = kp * (-delta_phi) - kv * robot->angularVelocity(link_name);

            VectorXd F = VectorXd::Zero(6);
            F.head(3) = Fv;
            F.tail(3) = Fw;
            F = Lambda_0 * F;

            // Gravity Compensation
            VectorXd g = robot->jointGravityVector();

            // ----- Save into the file -----
            file_output << time << "\t" << x.transpose() << "\t" << x_desired.transpose() << "\t" << delta_phi.transpose() << "\n";

            // Common Control
            control_torques.setZero();
            control_torques = J_0.transpose() * F - N.transpose() * kvj * robot->dq() + g;
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            // Desired position and orientation
            Vector3d x_desired = Vector3d(0.6, 0.3, 0.4);
            VectorXd q_desired = initial_q;
            q_desired << 0, 0, 0, 0, 0, 0, 0;

            // Control parameters
            double kp = 200.0;  // 100.0
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;

            VectorXd x = robot->position(link_name, pos_in_link);

            VectorXd dx = robot->linearVelocity(link_name, pos_in_link);

            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);
            N = robot->nullspaceMatrix(Jv);
            MatrixXd M = robot->M();
            VectorXd g = robot->jointGravityVector();

            // // ----- (4a) -----
            // VectorXd F = Lambda * (kp * (x_desired - x) - kv * dx);

            // ----- (4b) -----
            Vector3d dx_desired = kp / kv * (x_desired - x);
             // 0.1 is the max velocity
            float v = 0.1 / dx_desired.norm();
            // saturate -- clip the value
            if (v > 1.0) {
                v = 1.0;
            }
            else if (v < -1.0) {
                v = -1.0;
            }
            VectorXd F = Lambda * (- kv * (dx - v * dx_desired));


            // ----- IN COMMON -----
            // Save into the file + Control
            file_output << time << "\t" << x.transpose() << "\t" << x_desired.transpose() << "\t" << dx.transpose() << "\t" << dx.norm() << "\n";
            control_torques.setZero();
            control_torques = Jv.transpose() * F + N.transpose() * M * (- kpj * (robot_q - q_desired) - kvj * robot_dq) + g;  
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    // **********************
    file_output.close();
    // **********************

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
