#include "Knight.hpp"
#include <ros/ros.h>

Knight::Knight(unsigned int p_id) :
	id(p_id)
{
}

void Knight::sub_callback(const nav_msgs::Odometry::ConstPtr& _ptr)
{
	ROS_WARN("Callback called by %d", id);
	pose = _ptr->pose.pose;
}
