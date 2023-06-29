// Author Harry Nguyen 06/26/2023

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_path_planning");
    // Node handle
    ros::NodeHandle node;
    // Declare the asynspinner that handle multi-threaded callback
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Assign the planning group
    static const std::string group_name = "arm_group";
    moveit::planning_interface::MoveGroupInterface move_grp(group_name);
    // Add or remove collision object in Rviz
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Get the joint model group
    const robot_state::JointModelGroup* joint_model_group = move_grp.getCurrentState()->getJointModelGroup(group_name);
    // Declare rviz
    namespace rvt = rviz_visual_tools;
    // Declare the visual tool asscociate with the fisrt link of the joint group
    moveit_visual_tools::MoveItVisualTools visual_tools("arm1_link");
    // Delate all current marker in Rviz
    visual_tools.deleteAllMarkers();
    // Initiate the remote control 
    visual_tools.loadRemoteControl();
    // Dsiplay some initiate information about the robot
    ROS_INFO_NAMED("robot_5dof", "Planning frame: %s", move_grp.getPlanningFrame().c_str());
    // get planning group of the robot
    ROS_INFO_NAMED("robot_5dof", "Available planning groups: ");
    std::copy(move_grp.getJointModelGroupNames().begin(), move_grp.getJointModelGroupNames().end(), 
            std::ostream_iterator<std::string>(std::cout,", "));
    // Display instaruction
    visual_tools.prompt("Press button 'Next' in the RvizVisulaToolGui window to start the 1st path");


    // TARGET USING POSE OF THE END EFFECTOR 
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 0.707;
    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 0.707;
    target_pose1.orientation.z = 0.0;
    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.48;
    // target_pose1.orientation.w = 0.707;
    // target_pose1.position.x = 0.5;
    // target_pose1.position.y = 0.0;
    // target_pose1.position.z = 0.48;
    // Set the target to Moveit
    //move_grp.setPoseTarget(target_pose1);
    move_grp.setPoseReferenceFrame("world");
    move_grp.setApproximateJointValueTarget(target_pose1, "arm5_link");
    // Declare the moveit plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Check if the pose can be planned
    bool success = (move_grp.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_5dof", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    ROS_INFO_NAMED("robot_5dof", "Robot is moving on path 1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    move_grp.startStateMonitor();
    // TARGET USING JOINT SPACE
    // Display instaruction
    visual_tools.prompt("Press button 'Next' in the RvizVisulaToolGui window to start the 1st path");
    
    
    // Set the new start state for the robot
    robot_state::RobotState start_state(*move_grp.getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose = target_pose1;
    bool found_ik = start_state.setFromIK(joint_model_group, start_pose);
    ROS_INFO("IK calculation %s", found_ik ? "SUCCESS" : "FAILED");
    move_grp.setStartState(start_state);

    // Get the current set of joint value of the robot
    std::vector<double> joint_group_pos(5);
    // Modilfy some joint space values
    joint_group_pos[0] = -2.618;
    joint_group_pos[1] = -1.0472;
    joint_group_pos[2] = 0.7854;
    joint_group_pos[3] = -1.57;
    move_grp.setJointValueTarget(joint_group_pos);
    
    // Plan the path
    success = (move_grp.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_5dof", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");
    ROS_INFO_NAMED("robot_5dof", "Robot is moving on path 2");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();


    // // SET THE CONSTRAINT FOR THE ORIENTATION OF THE END EFFECTOR
    // Display instaruction
    visual_tools.prompt("Press button 'Next' in the RvizVisulaToolGui window to start the 1st path");
    
    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "arm5_link";
    // ocm.header.frame_id = move_grp.getPlanningFrame();
    // ocm.orientation.w = 1.0;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // // Define the importance of the constraint (the closer to 0.0 the less important the constraint)
    // ocm.weight = 1.0; 

    // // Set the constraint to the path contraint
    // moveit_msgs::Constraints path_constraint1;
    // path_constraint1.orientation_constraints.push_back(ocm);
    // move_grp.setPathConstraints(path_constraint1);
    // ROS_INFO_NAMED("robot_5dof", "Added constraint for the end-effector link");

    // Plan the robot to the new pose stateS
    // move_grp.setStartState(*move_grp.getCurrentState());

    // geometry_msgs::Pose target_pose2;
    // target_pose2.orientation.w = 1.0;
    // target_pose2.position.x = 0.5;
    // target_pose2.position.y = 0.2;
    // target_pose2.position.z = 0.52;
    // move_grp.setPoseTarget(target_pose2);

    // move_grp.setPlanningTime(10.0);
    // // Plan the path
    // success = (move_grp.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("robot_5dof", "Visualizing plan 3 (path with constraint) %s", success ? "SUCCESS" : "FAILED");
    // ROS_INFO_NAMED("robot_5dof", "Robot is moving on path 3");
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // // Clear the path constraint
    // move_grp.clearPathConstraints();

    // End the planning
    ros::shutdown();
    return 0;
}