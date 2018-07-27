#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>


#include <ros_myo/MyoGesture.h>

#include <boost/function.hpp>
#include <string>

#include <math.h>

static const std::string PLANNING_GROUP= "manipulator";

static bool execute_flag = false;

static int stable_count = 0;
void addObjectTable(std::vector<moveit_msgs::CollisionObject> &collision_objects, moveit_msgs::CollisionObject &table)
{
	table.id = "table";
	shape_msgs::SolidPrimitive table_shape;
	table_shape.type = table_shape.BOX;
	table_shape.dimensions.resize(3);
	table_shape.dimensions[0] = 1.5;
	table_shape.dimensions[1] = 1.5;
	table_shape.dimensions[2] = 0.2;


	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 0;
	table_pose.position.x = 0;
	table_pose.position.y = 0;
	table_pose.position.z = -0.1;

	table.primitives.push_back(table_shape);
	table.primitive_poses.push_back(table_pose);
	table.operation = table.ADD;

	collision_objects.push_back(table);

}

bool groupInit(moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{

	std::vector<double> init_pose;

	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), init_pose);
	init_pose[0] = 0.0;
	init_pose[1] = -1.57075;
	init_pose[2] = 1.57075;
	init_pose[3] = -1.57075;
	init_pose[4] = -1.57075;
	init_pose[5] = 0.0;

	group.setJointValueTarget(init_pose);
	bool success = group.plan(my_plan);

	if (success)
	{
		group.move();
		return true;
	}

	else 
	{
		ROS_ERROR("Unable to initialize robot pose! Please check the hardware!");
		return false;
	}

}

void ocm_init(moveit::planning_interface::MoveGroupInterface &group)
{
	/***
    geometry_msgs::Pose pose_temp = group.getCurrentPose().pose;

    moveit_msgs::JointConstraint jcm;
    jcm.joint_name = "shoulder_lift_joint";
    jcm.position = -1.57;
    jcm.tolerance_above = 1.1;
    jcm.tolerance_below = 1.1;

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "ee_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = pose_temp.orientation.w;
	ocm.orientation.x = pose_temp.orientation.x;
	ocm.orientation.y = pose_temp.orientation.y;
	ocm.orientation.z = pose_temp.orientation.z;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	
	moveit_msgs::Constraints path_constraint;
	path_constraint.orientation_constraints.push_back(ocm);
	//path_constraint.joint_constraints.push_back(jcm);

	group.setPathConstraints(path_constraint);
    ***/


}
void gestCallback(const ros_myo::MyoGesture &gesture)
{
	if (gesture.gesture == 2)
    {
		//execute_flag = true;
		//ROS_INFO("Ready to execute!");
        //if (gesture.gesture == )
			ros::shutdown();
    }
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_control_ur5");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	sleep(10.0);//wait to ensure Rviz has come up

	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	ros::Publisher cmd_publisher = node_handle.advertise<std_msgs::String>("/ur_driver/URScript", 1);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ros::Subscriber gesture_sub1 = node_handle.subscribe("/myo_raw/gest", 1000, &gestCallback);
	ros::Subscriber gesture_sub2 = node_handle.subscribe("/myo_raw/gest1", 1000,&gestCallback);
	tf::TransformListener listener;
	geometry_msgs::Pose target_pose1;
	int pose_count = 1;
	bool pose_change;	
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	moveit_msgs::CollisionObject table;
	table.header.frame_id = group.getPlanningFrame();
	addObjectTable(collision_objects, table);

	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
	sleep(2.0);
	//group.attachObject(table.id);
	planning_scene_interface.addCollisionObjects(collision_objects);

	while(!groupInit(group, my_plan))
	{
		sleep(1.0);
		if(!node_handle.ok())
		{
			ros::shutdown();
			return 0;
		}
	}

	sleep(5.0);
	//ocm_init(group);
    std_msgs::String cmd;
    std::stringstream ss;
    ss << "movel()"

	bool success = false;
	ROS_INFO("Initiation complete! Ready to move your arm!");
	while (node_handle.ok())
	{
		tf::StampedTransform transform;
		tf::Quaternion rot;
		tf::Vector3 trans;
		if ( execute_flag)
		{
			
		}
			
		try
		{
			listener.lookupTransform("/base_link", "/hand", ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		if(pose_count == 1)
		{
			trans = transform.getOrigin();
			rot = transform.getRotation();
			pose_count++;
		}
		else
		{
			if ( !((fabs(trans.x() - transform.getOrigin().x())>0.03) || (fabs(trans.y() - transform.getOrigin().y())>0.03) || (fabs(trans.z() - transform.getOrigin().z())>0.03)) )     
            {
                stable_count++;
                continue;
            }
			else
			{
				trans = transform.getOrigin();
				pose_change = true;
                stable_count = 0;
			}

			if ( !((fabs(rot.x() - transform.getRotation().x())>0.2) || (fabs(rot.y() - transform.getRotation().y())>0.2) || (fabs(rot.z() - transform.getRotation().z())>0.2) || (fabs(rot.w() - transform.getRotation().x())>0.2))) 
				continue;
			else
			{
				rot = transform.getRotation();
				pose_change = true;
			}

		}
		if (pose_change && stable_count >> 20)
		{
		    pose_change=false;
			robot_state::RobotState start_state(*group.getCurrentState());
			geometry_msgs::Pose start_pose2;
			//start_pose2 = target_pose1;
			const robot_state::JointModelGroup *joint_model_group =
							start_state.getJointModelGroup(group.getName());
			//start_state.setFromIK(joint_model_group, start_pose);
			group.setStartState(start_state);

		}
		target_pose1 = group.getCurrentPose().pose;
		//target_pose1.orientation.x = rot.x();
		//target_pose1.orientation.y = rot.y();
		//target_pose1.orientation.z = rot.z();
		//target_pose1.orientation.w = rot.w();
		target_pose1.position.x = trans.x()*0.6 + 0.1;
		target_pose1.position.y = - trans.y()*0.6;
		target_pose1.position.z = trans.z() * 0.7 - 0.55;
		group.setPoseTarget(target_pose1);
		
		planning_scene_interface.addCollisionObjects(collision_objects);
		success = group.plan(my_plan);
		if (success)
			group.move();


		//sleep(3.0);

		ros::spinOnce();

	  //  ROS_INFO("Visualizing plan x:%f y:%f z:%f",  target_pose1.position.x,  target_pose1.position.z,  target_pose1.position.z );    
/*****************
		if (1)
		{
	     ROS_INFO("Visualizing plan x:%f y:%f z:%f",  target_pose1.position.x,  target_pose1.position.z,  target_pose1.position.z );    
	     display_trajectory.trajectory_start = my_plan.start_state_;
	     display_trajectory.trajectory.push_back(my_plan.trajectory_);
	     display_publisher.publish(display_trajectory);
	     // Sleep to give Rviz time to visualize the plan. 
	     sleep(5.0);
		}
************/

	}	  

	ros::shutdown();
	return 0;
}
