
#include <uibk_kinematics/kinematics.h>
#include <eigen_conversions/eigen_msg.h>

namespace uibk_kinematics {

Kinematics::Kinematics() : avoid_collisions_(false) {

	/* Load the robot model */
	rml_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
	/* Get a shared pointer to the model */
	// instance of our robot model loaded from URDF
	ROS_DEBUG("Loading robot model from URDF");
	robot_model_ = rml_->getModel();
	if(!robot_model_) {
		string error =  "Unable to load robot model - make sure, that the robot_description is uploaded to the server!";
		ROS_FATAL_STREAM(error);
		throw runtime_error(error);
	}

	// create instance of planning scene
	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

}

bool Kinematics::computeIK(const string &arm,
						   const geometry_msgs::Pose &goal,
						   vector<double> &solution,
						   const bool avoid_collisions,
						   const int attempts,
						   const double timeout) {

	// use empty vector as default seed state
	vector<double> seed_state;
	return computeIK(arm, goal, seed_state, solution, avoid_collisions, attempts, timeout);

}

bool Kinematics::computeIK(const string &arm,
						   const geometry_msgs::Pose &goal,
						   vector<double> &seed_state,
						   vector<double> &solution,
						   const bool avoid_collisions,
						   const int attempts,
						   const double timeout) {

	string ik_group = arm + "_arm";

	ROS_DEBUG("IK request received for group '%s'", ik_group.c_str());
	// connect to ik group
	const robot_model::JointModelGroup *jnt_model_group = robot_model_->getJointModelGroup(ik_group);
	if(jnt_model_group == NULL) {
		ROS_ERROR("Unknown IK-Group '%s' - please specify either 'left' or 'right' for arm parameter", ik_group.c_str());
		return false;
	}

	// create robot state instance and set to initial values
	robot_state::RobotState state(robot_model_);
	state.setToDefaultValues();

	// check, if seed state was provided and is valid
	if(!seed_state.empty() && seed_state.size() != jnt_model_group->getJointModelNames().size()) {
		ROS_ERROR("If a seed state is provided, it has to contain exactly one value per joint!");
	}

	// set seed state if necessary
	if(!seed_state.empty()) {
		state.setJointGroupPositions(jnt_model_group, seed_state);
	}

	avoid_collisions_ = avoid_collisions;

	// compute result
	if(state.setFromIK(jnt_model_group, goal, attempts, timeout, boost::bind(&Kinematics::collisionCheckCallback, this, _1, _2, _3))) {
		// store joint positions.
		state.copyJointGroupPositions(jnt_model_group, solution);
		ROS_DEBUG("IK calculation successful");
		return true;
	} else {
		ROS_DEBUG("IK calculation failed");
		return false;
	}
}

bool Kinematics::computeFK(const string &arm,
			   const vector<double> joint_positions,
			   geometry_msgs::Pose &solution,
			   const string &link_name) {

	string ik_group = arm + "_arm";
	ROS_DEBUG("FK request received for group '%s'", ik_group.c_str());
	// connect to ik group
	const robot_model::JointModelGroup *jnt_model_group = robot_model_->getJointModelGroup(ik_group);
	if(jnt_model_group == NULL) {
		ROS_ERROR("Unknown IK-Group '%s' - please specify either 'left' or 'right' for arm parameter", ik_group.c_str());
		return false;
	}
	// create robot state instance and set to given values
	robot_state::RobotState state(robot_model_);
	state.setJointGroupPositions(jnt_model_group, joint_positions);

	string tip_link = link_name;

	if(tip_link.empty()) {
		tip_link = arm + "_arm_7_link";
	}

	ROS_DEBUG("Computing FK solution for link '%s'", tip_link.c_str());
	const Eigen::Affine3d &eef_state = state.getGlobalLinkTransform("right_arm_7_link");
	tf::poseEigenToMsg(eef_state, solution);

	return true;
}

bool Kinematics::collisionCheckCallback(RobotState *state,
										const JointModelGroup *joint_group,
										const double *solution)
{
	if(avoid_collisions_) {
		state->setJointGroupPositions(joint_group, solution);
		state->update();
		return !planning_scene_->isStateColliding(*state, joint_group->getName());
	} else {
		return true;
	}

}

} // end uibk_kinematics
