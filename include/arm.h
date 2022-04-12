#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
// custom
#include "utils.h"
#include"competition.h"

namespace motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Arm {
        public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct ArmPresetLocation {
            std::vector<double> arm_preset;  //9 joints
            std::string name;
        } start, bin, agv, grasp;


        Arm(ros::NodeHandle& node_handle);



        motioncontrol::Competition competition_object;
        void set_competition_object(motioncontrol::Competition& competition){
            competition_object= competition;
        }


        bool get_quality_camera1_;
        bool get_quality_camera2_;
        bool get_quality_camera3_;
        bool get_quality_camera4_;

        void get_quality_camera1_data(){
            get_quality_camera1_ = competition_object.get_quality_camera1_data();
        }
         void get_quality_camera2_data(){
            get_quality_camera2_ = competition_object.get_quality_camera2_data();
        }
        void get_quality_camera3_data(){
            get_quality_camera3_ = competition_object.get_quality_camera3_data();
        }
         void get_quality_camera4_data(){
            get_quality_camera4_= competition_object.get_quality_camera4_data();
        }
        /**
         * @brief Initialize the object
         */
        void init();
        bool pickPart(std::string part_type, geometry_msgs::Pose part_pose);
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv);
        void testPreset(const std::vector<ArmPresetLocation>& preset_list);
        void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
        // void check_part_pose(geometry_msgs::Pose final_pose_in_world,geometry_msgs::Pose goal_in_tray_frame,std::string agv);
        void check_part_pose(geometry_msgs::Pose target_pose_in_world,std::string agv);

        void activateGripper();
        void deactivateGripper();
        
        /**
         * @brief Move the joint linear_arm_actuator_joint only
         *
         * The joint position for this joint corresponds to the y value
         * in the world frame. For instance, a value of 0 for this joint
         * moves the base of the robot to y = 0.
         *
         * @param location A preset location
         */
        void moveBaseTo(double linear_arm_actuator_joint_position);
        nist_gear::VacuumGripperState getGripperState();

        

        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(std::string location_name);

        //--preset locations;
        start home1_, home2_;
        agv agv1_, agv2_, agv3_, agv4_;
                int counter;
        private:
        
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options arm_options_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        sensor_msgs::JointState current_joint_states_;
        control_msgs::JointTrajectoryControllerState arm_controller_state_;

        

        nist_gear::VacuumGripperState gripper_state_;
        // gripper state subscriber
        ros::Subscriber gripper_state_subscriber_;
        // service client
        ros::ServiceClient gripper_control_client_;
        // publishers
        ros::Publisher arm_joint_trajectory_publisher_;
        // joint states subscribers
        ros::Subscriber arm_joint_states_subscriber_;
        // controller state subscribers
        ros::Subscriber arm_controller_state_subscriber_;
        std::string part_type_name;
      
       
        int &get_counter(){
            return counter;
        }
        auto get_part_type_name(){
            return part_type_name;
        }
 
        // const bool& get_logical_camera_1{
        //     return competition.quality_camera_1_;
        // }
        // callbacks
        void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
        void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
        void check_faulty_part(std::string part_type,geometry_msgs::Pose part_pose_in_frame,std::string agv);

    };
}//namespace
#endif
