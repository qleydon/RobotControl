#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

std_msgs::Float64 jnt_pos_start[6];

void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
	jnt_pos_start[0].data = ctr_msg->process_value;
}

void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start[1].data = ctr_msg->process_value;
}

void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start[2].data = ctr_msg->process_value;
}

void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start[3].data = ctr_msg->process_value;
}

void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start[4].data = ctr_msg->process_value;
}

void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start[5].data = ctr_msg->process_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_control");
	ros::NodeHandle n;

	//Create subscribers for all joint states
	ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
	ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
	ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
	ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
	ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
	ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);

	//Create publishers to send position commands to all joints
	ros::Publisher joint_com_pub[6]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	
	while (ros::ok()) {
        std_msgs::Float64 position[6];
        //Compute next position step for all joints
        //int count = 0;
        for(int i=0; i<6; i++) {
            position[i].data = -1;
            joint_com_pub[i].publish(position[i]);
        }
        //count++;
        ros::spinOnce();
        ros::Duration(3.0).sleep();	
	}	
	return 0;
}