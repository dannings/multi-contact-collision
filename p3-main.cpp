/*  hw4 - p3 main.cpp
This file includes the required code to implement whole body control.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 5/10/17

Modifier: Danning Sun
Date 6/27!!!!!!
date: 7/9/17
*/
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/ChaiGraphics.h"
#include "graphics/chai_extension/CRobotBase.h"
#include "simulation/Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"
#include "geometry/CapsuleDistanceHull.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const double time_slowdown_factor = 1;

const string world_fname = "resources/hw4/world_testRRR.urdf";
const string robot_fname = "../resources/test_robot/testRRR.urdf";
const string robot_name = "testRRR";
const string camera_name = "camera_front";
//const string camera_name = "camera_top";
const string ee_link_name = "link2";

// Desired joint posture
Eigen::VectorXd q_des;

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
void simulation(Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);

    // set initial conditions
    q_des.setZero(robot->dof());
	q_des << 0/180.0*M_PI, 0/180.0*M_PI, 0/180.0*M_PI;
	robot->_q = q_des;
	sim->setJointPositions(robot_name, robot->_q);
	sim->setCollisionRestitution(0); // put it in initialization
	robot->updateModel();
	Eigen::Affine3d ee_trans;

	
	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	
	// end effector tip position in link frame
	Eigen::Vector3d ee_pos_local;
	ee_pos_local << 0.0, 0, 3.0;

	Eigen::MatrixXd Jconst_full_6(6, robot->dof());
	Eigen::Vector3d ee_pos_des;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint position, velocity
        sim->getJointPositions(robot_name, robot->_q);
        sim->getJointVelocities(robot_name, robot->_dq);
        robot->updateModel();

        // update desired ee position
        ee_pos_des << 0.5, 0.2, 0.5;
        //ee_pos_des << 0.2, 0.1 + 0.1*sin(2*M_PI*curr_time/6), 0.2;
        //ee_pos_des << 0.7, 0.2 + 0.4*sin(2*M_PI*curr_time/6), 0.5;

        //task controller
        //calcualte desired dx,ddx for control purpose
        Eigen::Vector3d ee_pos_dot_des;
        Eigen::Vector3d ee_pos_dotdot_des;
        ee_pos_dot_des << 0,0,0;
        ee_pos_dotdot_des << 0,0,0;
        //ee_pos_dot_des << 0,0.1 * cos(2*M_PI/6*curr_time)*2*M_PI/6,0;
        //ee_pos_dotdot_des << 0,-0.1 * sin(2*M_PI/6*curr_time)*2*M_PI/6*2*M_PI/6,0;
        
        //end effector position:
        Eigen::Vector3d ee_pos_curr;
        robot->position(ee_pos_curr, ee_link_name, ee_pos_local);
        //cout << ee_pos_curr << endl << endl;

        //end effector velocity
        Eigen::Vector3d ee_vel_curr;
        robot->linearVelocity(ee_vel_curr, ee_link_name, ee_pos_local);       
        //task jacobian (Jv)
        Eigen::MatrixXd Jt(3,robot->dof());
        robot->Jv(Jt, ee_link_name, ee_pos_local);
        Eigen::MatrixXd inertia_t(3,3);
        robot->taskInertiaMatrixWithPseudoInv(inertia_t,Jt);
    
        Eigen::VectorXd robot_g(robot->dof());
        robot->gravityVector(robot_g);
        //robot_g.setZero();

        //simple ee pos controller
        int kp = 400; int kv = 20;
        //controller problem, 1 ee_local_pos 2 ee_pos_des does not match ee_pos_curr
        //tau = Jt.transpose() * inertia_t *(kp * (ee_pos_curr - ee_pos_des) - kv * (ee_vel_curr - ee_pos_dot_des)) + robot_g;
        tau = robot_g * 0.9;
        //tau.setZero();
		
		/* --------------------------------------------------------- */
		sim->setJointTorques(robot_name, tau);
		
		//get contact info from sim
		std::vector<Eigen::Vector3d> num_contact;
		std::vector<Eigen::Vector3d> contact_force;
		//assume contact is at the end effector link
		sim->getContactList(num_contact,contact_force,robot_name,ee_link_name);
		//output contact point and contact force
		
		std::vector<Eigen::Vector3d>::iterator i;
		for(i = num_contact.begin(); i != num_contact.end(); ++i) {
			if (i == num_contact.begin()) cout << "contact point" << endl;
			Eigen::Vector3d current_contact_point = *i; // i instead of *i ??
			std::cout << current_contact_point << std::endl;
		}
		std::vector<Eigen::Vector3d>::iterator j;
		for(j = contact_force.begin(); j != contact_force.end(); ++j) {
			if (j == contact_force.begin()) cout << "contact force" << endl;
			Eigen::Vector3d current_contact_force = *j; // i instead of *i ??
			std::cout << current_contact_force << std::endl;
		}

		//cout << endl << "position" << endl << ee_pos_curr << endl;
		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer

	// sleep for a few moments to let graphics start
	// std::this_thread::sleep_for(std::chrono::seconds(1));	
	
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// if (timer.elapsedCycles() % 10000 == 0) {
		// 	cout << "Simulation loop frequency: " << timer.elapsedCycles()/timer.elapsedTime() << endl; 
		// }

		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);
		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW4", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    
}
