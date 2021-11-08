#ifndef KNIGHT_HPP
#define KNIGHT_HPP

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

struct Knight 
{
	unsigned int id;
	unsigned int mode;
	unsigned int step;
	bool need_path;
	std::string ns;
	int row;
	int col;
	geometry_msgs::Pose pose;

	Knight(unsigned int p_id);
	void sub_callback(const nav_msgs::Odometry::ConstPtr& _ptr);
};

#endif	// KNIGHT_HPP
