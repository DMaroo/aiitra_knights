#include "Knight.hpp"

Knight::Knight(unsigned int p_id) :
	id(p_id)
{
}

void Knight::sub_callback(const nav_msgs::Odometry::ConstPtr& _ptr)
{
	pose = _ptr->pose.pose;
}
