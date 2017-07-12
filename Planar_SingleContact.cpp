#include "Planar_SingleContact.h"

#include <iostream>
#include <fstream>

#include <signal.h>

#include "model/ModelInterface.h"
#include "graphics/ChaiGraphics.h"
#include "graphics/chai_extension/CRobotBase.h"
#include "simulation/Sai2Simulation.h"

#include "timer/LoopTimer.h"

static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * DemoProject::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void DemoProject::readRedisValues() {

	// read from Redis current sensor values
	//redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	//redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	// Get current simulation timestamp from Redis
	//redis_client_.getCommandIs(TIMESTAMP_KEY, redis_buf_);
	//t_curr_ = stod(redis_buf_);

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	//redis_client_.getCommandIs(KP_POSITION_KEY, redis_buf_);
	//kp_pos_ = stoi(redis_buf_);
	//redis_client_.getCommandIs(KV_POSITION_KEY, redis_buf_);
	//kv_pos_ = stoi(redis_buf_);
	//redis_client_.getCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	//kp_ori_ = stoi(redis_buf_);
	//redis_client_.getCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	//kv_ori_ = stoi(redis_buf_);
	//redis_client_.getCommandIs(KP_JOINT_KEY, redis_buf_);
	//kp_joint_ = stoi(redis_buf_);
	//redis_client_.getCommandIs(KV_JOINT_KEY, redis_buf_);
	//kv_joint_ = stoi(redis_buf_);
}

/**
 * DemoProject::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void DemoProject::writeRedisValues() {
	// Send end effector position and desired position
	//redis_client_.setEigenMatrixDerivedString(EE_POSITION_KEY, x_);
	//redis_client_.setEigenMatrixDerivedString(EE_POSITION_DESIRED_KEY, x_des_);

	// Send torques
	//redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

/**
 * DemoProject::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void DemoProject::updateModel() {
	const string world_fname = "resources/Planar_SingleContact/world_testRRR.urdf";
	const string robot_fname = "../resources/test_robot/testRRR.urdf";
	const string robot_name = "testRRR";
	//const string camera_name = "camera_front";
	//const string camera_name = "camera_top";
	const string ee_link_name = "link2";
	Eigen::Vector3d ee_local;
	ee_local.setZero();

	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x_, ee_link_name, ee_local);
	robot->linearVelocity(dx_, ee_link_name, ee_local);

	// Jacobians
	robot->Jv(Jv_, ee_link_name, ee_local);
	robot->nullspaceMatrix(N_, Jv_);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda_x_, Jv_);
	robot->gravityVector(g_);
}

/**
 * DemoProject::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
DemoProject::ControllerStatus DemoProject::computeJointSpaceControlTorques() {
	// read joint position, velocity
    sim->getJointPositions(robot_name, robot->_q);
    sim->getJointVelocities(robot_name, robot->_dq);
  	robot->updateModel();

  	int kv_joint_ = 0;
  	int kp_joint_ = 0;

	// Finish if the robot has converged to q_initial
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	if (q_err.norm() < kToleranceInitQ && dq_err.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;
	command_torques_ = robot->_M * ddq + g_;
	//command_torques_.setZero();
	return RUNNING;
}

/**
 * DemoProject::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::computeOperationalSpaceControlTorques() {
	// PD position control with velocity saturation
	Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err_;
	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

	// Nullspace posture control and damping
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	// Control torques
	Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::VectorXd F_posture = robot->_M * ddq;
	command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_posture + g_;

	return RUNNING;
}

/**
 * public DemoProject::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void DemoProject::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	//intialize conditions
	Eigen::VectorXd q_des;
	q_des.setZero(robot->dof());
	q_des << 0/180.0*M_PI, 0/180.0*M_PI, 0/180.0*M_PI;
	robot->_q = q_des;
	sim->setJointPositions(robot_name, robot->_q);
	sim->setCollisionRestitution(0); // put it in initialization
	robot->updateModel();

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	//redis_client_.serverIs(kRedisServerInfo);

	// Set gains in Redis if not initialized
	/*if (!redis_client_.getCommandIs(KP_POSITION_KEY)) {
		redis_buf_ = to_string(kp_pos_);
		redis_client_.setCommandIs(KP_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_POSITION_KEY)) {
		redis_buf_ = to_string(kv_pos_);
		redis_client_.setCommandIs(KV_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kp_ori_);
		redis_client_.setCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kv_ori_);
		redis_client_.setCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_JOINT_KEY)) {
		redis_buf_ = to_string(kp_joint_);
		redis_client_.setCommandIs(KP_JOINT_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_JOINT_KEY)) {
		redis_buf_ = to_string(kv_joint_);
		redis_client_.setCommandIs(KV_JOINT_KEY, redis_buf_);
	}*/
}

/**
 * public DemoProject::runLoop()
 * -----------------------------
 * DemoProject state machine
 */
void DemoProject::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;

		// Get latest sensor values from Redis and update robot model
		//readRedisValues();
		updateModel();

		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				
				break;
				//if (isnan(robot->_q)) continue;
				//cout << "Redis synchronized. Switching to joint space controller." << endl;
				//controller_state_ = JOINT_SPACE_INITIALIZATION;
				//break;

			// Initialize robot to default joint configuration
			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Switching to operational space controller." << endl;
					controller_state_ = DemoProject::OP_SPACE_POSITION_CONTROL;
				};
				break;

			// Control end effector to desired position
			case OP_SPACE_POSITION_CONTROL:
				computeOperationalSpaceControlTorques();
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Check command torques before sending them
		if (isnan(command_torques_)) {
			cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques_.setZero();
		}

		// Send command torques
		//writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	//redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

int main(int argc, char** argv) {
	
	// load graphics scene
	cout << "Loading graphics scene: " << world_fname << endl;
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);

	// Load robot
	const string robot_name = "testRRR";
	const string world_fname = "resources/Planar_SingleContact/world_testRRR.urdf";
	const string robot_fname = "../resources/test_robot/testRRR.urdf";

	cout << "Loading robot: " << robot_fname << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_fname, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	DemoProject app(move(robot), robot_name);
	app.initialize();
	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}