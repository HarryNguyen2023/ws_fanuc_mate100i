// Author Harry Nguyen (Created 06/28/2023)

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    // Initiate the ROS node
    ros::init(argc, argv, "robot_5dof_robot_state");
    // Configure node hanfle
    ros::NodeHandle node;
    // Setup asyncspinner for multi-threaded callback
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // LOAD THE MODEL OF THE ROBOT
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    // Get the kinematic model of the robot
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // Display the model frame
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    // LOAD THE ROBOT STATE CONTAINING THE CONFIGURATION OF THE ROBOT
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    // Set the configuration to default value set up in the URDF file
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_grp = kinematic_model->getJointModelGroup("arm_group");
    // Get joint space values and joint names
    const std::vector<std::string>& joint_names = joint_model_grp->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_grp, joint_values);
    // Display to the terminal
    for(int i = 0; i < joint_values.size(); ++i)
    {
        ROS_INFO("Joint: %s - %lf rad", joint_names[i].c_str(), joint_values[i]);
    }


    // FORWARD KINEMATICS OF THE ROBOT
    // Generate a random pose for the robot
    kinematic_state->setToRandomPositions(joint_model_grp);
    // Get the position of the final link "arm5_link" of the robot
    const Eigen::Isometry3d& ee_state = kinematic_state->getGlobalLinkTransform("arm5_link");
    // Display to the terminal
    ROS_INFO_STREAM("End effector translation: \n" << ee_state.translation() << "\n");
    ROS_INFO_STREAM("End effector rotation: \n" << ee_state.rotation() << "\n");

    // INVERSE KINEMATICS OF THE ROBOT
    double timeout = 0.1;
    // Calcualte the IK from the initial state of the robot to the new random state
    bool found_ik = kinematic_state->setFromIK(joint_model_grp, ee_state, timeout);
    // Display the solution
    if(found_ik)
    {
        // Get the new joint space values
        kinematic_state->copyJointGroupPositions(joint_model_grp, joint_values);
        for(int i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint: %s - %f rad", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("IK calcualtion FAILED");
    }

    // GET THE JACOBIAN MATRIX OF THE ROBOT 
    // The reference point for the Jocabian matrix is set to the global origin of the world link
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian_mat;
    kinematic_state->getJacobian(joint_model_grp, 
                                kinematic_state->getLinkModel(joint_model_grp->getLinkModelNames().back()),
                                reference_point, jacobian_mat);
    // Display to the screen
    ROS_INFO_STREAM("Jacobian matrix of the robot: \n" << jacobian_mat << "\n");

    // Shutdown the node
    ros::shutdown();
    return 0;
}


