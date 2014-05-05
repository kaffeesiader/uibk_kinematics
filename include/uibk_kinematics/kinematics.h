#ifndef KINEMATICS_H
#define KINEMATICS_H


#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>


#define ATTEMPTS 5
#define TIMEOUT 0.1

using namespace std;
using namespace ros;
using namespace robot_model;

namespace uibk_kinematics {

class Kinematics {

private:

	robot_model_loader::RobotModelLoaderPtr rml_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	bool avoid_collisions_;

	/**
	 * Callback function for the IK-solver to check wheather the given state is valid, or not
	 *
	 * @brief collisionCheckCallback
	 * @param state
	 * @param joint_group
	 * @param solution
	 * @return
	 */
	bool collisionCheckCallback(RobotState *state,
								const JointModelGroup *joint_group,
								const double *solution);

public:

	Kinematics();
	~Kinematics() {}

	/**
	 * @brief computeIK
	 *
	 * Calculate a valid IK solution for group with given name and for given pose goal
	 * Additionally can be specified, wheather the solution has to be collision free, or not.
	 * The default is set to true.
	 *
	 * Maximum number of attempts and timeout can also be specified.
	 *
	 * @param arm left or right
	 * @param goal
	 * @param solution
	 * @param avoid_collisions
	 * @param attempts
	 * @param timeout
	 * @return true on success
	 */
	bool computeIK(const string &arm,
				   const geometry_msgs::Pose &goal,
				   vector<double> &solution,
				   const bool avoid_collisions = true,
				   const int attempts = ATTEMPTS,
				   const double timeout = TIMEOUT);
	/**
	 * @brief computeIK
	 *
	 * Calculate a valid IK solution for group with given name and for given pose goal and given seed state
	 * Additionally can be specified, wheather the solution has to be collision free, or not.
	 * The default is set to true.
	 *
	 * Maximum number of attempts and timeout can also be specified.
	 * @param arm 'left' or 'right'
	 * @param goal
	 * @param seed_state A vector, holding the joint positions to be used as seed state
	 * @param solution A vector to receive the solution
	 * @param avoid_collisions Default is set to false
	 * @param attempts
	 * @param timeout
	 * @return True, if a valid solution was found
	 */
	bool computeIK(const string &arm,
				   const geometry_msgs::Pose &goal,
				   vector<double> &seed_state,
				   vector<double> &solution,
				   const bool avoid_collisions = false,
				   const int attempts = ATTEMPTS,
				   const double timeout = TIMEOUT);
	/**
	  Compute the cartesian position for link with given name, using given robot state.
	  Optionally an alternative reference frame id can be specified. Default is set to world_link

	 * @brief computeFK
	 * @param arm the given joint positions are for. Can be either 'left' or 'right'
	 * @param joint_positions A vector, containing the current joint positions of the given arm
	 * @param solution Reference to a geometry_msgs::Pose to receive the solution
	 * @param link_name Optionally the name of the link can be specified. Uses last link on given arm as default
	 * @param frame_id
	 * @return True, if a valid solution was found
	 */
	bool computeFK(const string &arm,
				   const vector<double> joint_positions,
				   geometry_msgs::Pose &solution,
				   const string &link_name = "");

};

} // end uibk_kinematics

#endif // KINEMATICS_H
