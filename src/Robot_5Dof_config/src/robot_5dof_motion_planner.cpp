// Author Harry Nguyen (Created 06/28/2023)

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "robot_5dof_motion_planner");
    // Set up node handle
    ros::NodeHandle node("~");
    // Multi-threaded callback
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Load the robot model
    const std::string planning_grp = "arm_group";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // Load the robot state
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_grp = kinematic_state->getJointModelGroup(planning_grp);

    // Initiate the planning scene
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
    // Set robot to its default state in URDF 
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_grp,"ready");

    // Using pluginlib to load any planner we want to use in ROS
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // Get the name of the planning plugin and load the planner from the ROS parameter server
    if(!node.getParam("planning_plugin", planner_plugin_name))
    {
        ROS_FATAL_STREAM("Could not find planner plugin name");
        try
        {
            // Reset loading the planner
            planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>
                        ("moveit_core","planning_interface::PlannerManager"));

        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM("Exception while creating planner plugin loader"<< ex.what());
        }
        try
        {
            planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
            if (!planner_instance->initialize(kinematic_model, node.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
            ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
        }
        catch (pluginlib::PluginlibException& ex)
        {
            const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
            std::stringstream ss;
            for (std::size_t i = 0; i < classes.size(); ++i)
                ss << classes[i] << " ";
            ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                                << "Available plugins: " << ss.str());
        }
    }

    // Initiate moveit visual tool 
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    // Load remote control
    visual_tools.loadRemoteControl();

    // CREATE THE POSE TARGET FOR THE ROBOT
    visual_tools.prompt("Press 'next' to plan the robot for the first pose");

    // Iniate the target pose of the end effector
    geometry_msgs::PoseStamped target_pose1;
    target_pose1.header.frame_id = "world";
    target_pose1.pose.orientation.w = 0.707;
    target_pose1.pose.position.x = 0.45;
    target_pose1.pose.position.y = 0.2;
    target_pose1.pose.position.z = 0.52;
    // Position tolerance
    std::vector<double> tolerance_pose(3, 0.001);
    std::vector<double> tolerance_angle(3, 0.01);

    // Set up the service call for path planning
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    // Add the kinematic constraint
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm5_link", target_pose1, tolerance_pose, tolerance_angle);
    // Call for the motion planning service
    req.group_name = planning_grp;
    req.goal_constraints.push_back(pose_goal);
    


    ros::shutdown();
    return 0;
}