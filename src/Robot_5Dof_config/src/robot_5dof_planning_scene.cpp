// Author Harry Nguyen (Created 06/28/2023)

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// User-custom function to check for constraint of the robot
bool JointFeasibilityCheck(const robot_state::RobotState& kinematic_state, bool verbose)
{
    const double* joint_values = kinematic_state.getJointPositions("arm1_joint");
    return (joint_values[2] > 1.5708);
} 

int main(int argc, char** argv)
{
    // Initiate the node 
    ros::init(argc, argv, "robot_5dof_planning_scene");
    // Set up node handle
    ros::NodeHandle node;
    // Multi-threaded callback
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Iniate RvizVisualTollGui
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Advertise the planning scene API
    ros::Publisher planning_scene_diff_publisher = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    // Sleep until there is a subcriber for this topic
    ros::WallDuration sleep_t(0.5);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    visual_tools.prompt("Press 'next' to start inserting the object into the environment");

    // INSERT THE OBJECT INTO RVIZ
    moveit_msgs::AttachedCollisionObject new_obj;
    new_obj.link_name = "arm5_link";
    new_obj.object.header.frame_id = "arm5_link";
    new_obj.object.id = "cylinder";
    // Define the shape of the object
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.05;
    // Define the pose of the object
    geometry_msgs::Pose obj_pose;
    obj_pose.position.z = 0.13;
    obj_pose.orientation.w = 0.707;
    obj_pose.orientation.y = 0.707;
    
    // Insert the object into the environment
    new_obj.object.primitives.push_back(primitive);
    new_obj.object.primitive_poses.push_back(obj_pose);
    new_obj.object.operation = new_obj.object.ADD;
    // Ignore the collision check between the object the robot hand
    new_obj.touch_links = std::vector<std::string>{"arm5_link"};
    
    // Visualize the new object in the Rviz
    ROS_INFO("Adding the object to the environment");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(new_obj.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
  
    // INTERRACT THE PLANNING SCENE WITH THE MOVE GROUP NODE VIA SERVICE CALL
    visual_tools.prompt("Press 'next' to start inserting the object into the environment");
    ros::ServiceClient planning_scene_diff_client = 
                node.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    // Send the diff to the planning scene via service call
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

    // ATTACH THE OBJECT TO THE ROBOT DURING PICK AND PLACE TO CHECK FOR COLLISION
    /*1. Remove the current object in the space
    2. Add the new object attached to the robot arm*/
    moveit_msgs::CollisionObject rmv_obj;
    rmv_obj.id = "cylinder";
    rmv_obj.header.frame_id = "arm5_link";
    rmv_obj.operation = rmv_obj.REMOVE;
    // Carry out in the Rviz visual tool
    ROS_INFO("Removing the object from the world and attach it to the end-effector of the robot");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(rmv_obj);
    // Attach the object to the robot
    planning_scene.robot_state.attached_collision_objects.push_back(new_obj);
    planning_scene_diff_publisher.publish(planning_scene);

    // DETACH OBJECT
    visual_tools.prompt("Press 'next' to visualize the disappereance of the cylinder from the environment");
    moveit_msgs::AttachedCollisionObject detach_obj;
    detach_obj.object.id = "cylinder";
    detach_obj.link_name = "arm5_link";
    detach_obj.object.operation = detach_obj.object.REMOVE;

    // DETACH OBJECT FROM THE ROBOT
    ROS_INFO("Detaching the object from the robot");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_obj);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(new_obj.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);


    // DETACH THE OBJECT FROM THE WORLD ENVIRONMENT
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
    ROS_INFO("Deatching object from the world environment");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(rmv_obj);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
    ros::shutdown();
    return 0;
    
}