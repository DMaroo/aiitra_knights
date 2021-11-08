#include "Knight.hpp"
#include <darp_path_planner/DARPPath.h>
#include <fstream>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <thread>

#define MAP_X 50
#define MAP_Y 50

#define ANG_VEL 200
#define LIN_VEL 1000

#define ANGLE_THRESH 0.05
#define POS_THRESH	 0.1

void cell_to_map(int row, int col, int rows, int cols, double& x, double& y);

int rows = 0, cols = 0;

double to_theta(double _theta)	// convert a value to allowed values of theta: [-M_PI, M_PI]
{
	if (_theta < -M_PI)
	{
		_theta += 2 * M_PI;
	}
	else if (_theta > M_PI)
	{
		_theta -= 2 * M_PI;
	}

	return _theta;
}

double theta_difference(double _goal, double _reference)  // find the difference between two angles and return a valid difference
{
	_goal -= _reference;

	return to_theta(_goal);
}

bool close_to(double x1, double y1, double x2, double y2)  // check if two floating point numbers are "close enough"
{
	return (abs(x1 - x2) <= POS_THRESH && abs(y1 - y2) <= POS_THRESH);
}

void process_knight(Knight& knight, ros::NodeHandle& nh)
{
	int new_row = -1;
	int new_col = -1;
	double new_x, new_y;

	knight.need_path = true;

	std::mutex nh_lock;
	nh_lock.lock();
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>(knight.ns + "/knight/cmd_vel", 100);
	ros::Subscriber odom_sub = nh.subscribe("/odom", 100, &Knight::sub_callback, &knight);
	ros::ServiceClient darp_client = nh.serviceClient<darp_path_planner::DARPPath>("/darp_path");
	nh_lock.unlock();

	darp_path_planner::DARPPath path_request;

	ros::Rate loop_rate(200);

	ros::MultiThreadedSpinner spinner;

	bool rotate = true;
	double angle;
	tf2::Quaternion q;
	geometry_msgs::Twist msg;

	while (ros::ok())
	{
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0;
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;

		if (knight.need_path)
		{
			path_request.request.bot = knight.id;
			path_request.request.mode = knight.mode;
			path_request.request.step = knight.step;

			if (darp_client.call(path_request))
			{
				new_row = path_request.response.new_x;
				new_col = path_request.response.new_y;

				cell_to_map(new_row, new_col, rows, cols, new_x, new_y);
				knight.need_path = false;
			}
			else
			{
				continue;
			}
		}

		if (close_to(knight.pose.position.x, knight.pose.position.y, new_x, new_y))
		{
			knight.need_path = true;
			pub_vel.publish(msg);
			continue;
		}

		angle = atan2(new_x - knight.pose.position.x, new_y - knight.pose.position.y);
		q[0] = knight.pose.orientation.x;
		q[1] = knight.pose.orientation.y;
		q[2] = knight.pose.orientation.z;
		q[3] = knight.pose.orientation.w;

		if (theta_difference(q.getAngle(), angle) < ANGLE_THRESH)
		{
			msg.linear.x = LIN_VEL;
		}
		else
		{
			msg.angular.z = theta_difference(q.getAngle(), angle) > ANGLE_THRESH ? -ANG_VEL : ANG_VEL;
		}
		pub_vel.publish(msg);

		spinner.spin();
		loop_rate.sleep();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "knight_simulation");

	ros::NodeHandle nh;

	int bot_count = -1;
	if (!ros::param::get("/bot_count", bot_count))
	{
		ROS_ERROR("Please set the parameter \"/bot_count\" to provide the number of bots to simulate!\n");
		return -1;
	}

	int darp_mode = -1;
	if (!ros::param::get("/darp_mode", darp_mode))
	{
		ROS_ERROR("Please set the parameter \"/darp_mode\"!\n");
		return -1;
	}

	if (!ros::param::get("/darp/rows", rows))
	{
		ROS_ERROR("Please set the parameter \"/darp/rows\"!\n");
		return -1;
	}

	if (!ros::param::get("/darp/cols", cols))
	{
		ROS_ERROR("Please set the parameter \"/darp/cols\"!\n");
		return -1;
	}

	std::vector<int> initial_positions;
	if (!ros::param::get("/darp/initial_positions", initial_positions))
	{
		ROS_ERROR("Please set the parameter \"/darp/initial_positions\"!\n");
		return -1;
	}

	ROS_ASSERT_MSG(initial_positions.size() == bot_count, "Initial positions not matching the number of robots\n");

	std::vector<Knight> bots;
	for (int i = 0; i < bot_count; i++)
	{
		bots.push_back(Knight(i));
		bots[i].row = initial_positions[i] / cols;
		bots[i].col = initial_positions[i] % cols;
		cell_to_map(bots[i].row, bots[i].col, rows, cols, bots[i].pose.position.x, bots[i].pose.position.y);
		bots[i].pose.position.z = 0;
		bots[i].step = 0;
		bots[i].mode = darp_mode;
        bots[i].need_path = true;
	}

	// for (int i = 0; i < bot_count; i++)
	// {
	// 	path_request.request.bot = i;
	// 	path_request.request.mode = darp_mode;
	// 	path_request.request.step = 0;

	// 	if (darp_client.call(path_request))
	// 	{
	// 		new_row.push_back(path_request.response.new_x);
	// 		new_col.push_back(path_request.response.new_y);
	// 		tf2::Quaternion q;
	// 		if (new_row[i] > row[i])
	// 		{
	// 			q.setRPY(0, 0, M_PI);
	// 		}
	// 		else if (new_col[i] < col[i])
	// 		{
	// 			q.setRPY(0, 0, M_PI / 2);
	// 		}
	// 		else if (new_col[i] > col[i])
	// 		{
	// 			q.setRPY(0, 0, -M_PI / 2);
	// 		}
	// 		bots[i].orientation.x = q[0];
	// 		bots[i].orientation.y = q[1];
	// 		bots[i].orientation.z = q[2];
	// 		bots[i].orientation.w = q[3];
	// 	}
	// 	else
	// 	{
	// 		ROS_ERROR("Failed to find pose for %d\n", i);
	// 	}
	// }

	ros::service::waitForService("/gazebo/spawn_urdf_model");

	ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel spawn_request;
	std::string urdf_path;
	std::stringstream urdf_buf;
	std::ifstream read;

	for (int i = 0; i < bot_count; i++)
	{
		spawn_request.request.initial_pose = bots[i].pose;
		spawn_request.request.model_name = "knight" + std::to_string(i);

		urdf_path = ros::package::getPath("knight_description") + "/urdf/knight.urdf";
		read.open(urdf_path);
		urdf_buf << read.rdbuf();
		spawn_request.request.model_xml = urdf_buf.str();
		urdf_buf.clear();

		spawn_request.request.reference_frame = "/map";
		spawn_request.request.robot_namespace = spawn_request.request.model_name;
		bots[i].ns = spawn_request.request.robot_namespace;

		if (!spawn_client.call(spawn_request))
		{
			ROS_ERROR("Failed to spawn %d\n", i);
		}
	}

	// std::vector<ros::Publisher> pub_vel;
	// std::vector<ros::Subscriber> odom_sub;

	// for (auto& knight : bots)
	// {
	// 	pub_vel.push_back(nh.advertise<geometry_msgs::Twist>(knight.ns + "/knight/cmd_vel", 100));
	// 	odom_sub.push_back(nh.subscribe("/odom", 100, &Knight::sub_callback, &knight));
	// }

	ros::service::waitForService("/darp_path");
	

	// int new_row[bot_count], new_col[bot_count];
	// double new_x[bot_count], new_y[bot_count];
	// int j = 0;

	// while (ros::ok())
	// {
	// 	j = 0;
	// 	for (auto& knight : bots)
	// 	{
	// 		msg.angular.x = 0;
	// 		msg.angular.y = 0;
	// 		msg.angular.z = 0;
	// 		msg.linear.x = 0;
	// 		msg.linear.y = 0;
	// 		msg.linear.z = 0;

	// 		if (knight.need_path)
	// 		{
	// 			path_request.request.bot = knight.id;
	// 			path_request.request.mode = knight.mode;
	// 			path_request.request.step = knight.step;

	// 			if (darp_client.call(path_request))
	// 			{
	// 				new_row[j] = path_request.response.new_x;
	// 				new_col[j] = path_request.response.new_y;

	// 				cell_to_map(new_row[j], new_col[j], rows, cols, new_x[j], new_y[j]);
	// 				knight.need_path = false;
	// 			}
	// 			else
	// 			{
	// 				continue;
	// 			}
	// 		}

	// 		if (close_to(knight.pose.position.x, knight.pose.position.y, new_x[j], new_y[j]))
	// 		{
	// 			knight.need_path = true;
	// 			pub_vel[j].publish(msg);
	// 			continue;
	// 		}

	// 		angle = atan2(new_x[j] - knight.pose.position.x, new_y[j] - knight.pose.position.y);
	// 		q[0] = knight.pose.orientation.x;
	// 		q[1] = knight.pose.orientation.y;
	// 		q[2] = knight.pose.orientation.z;
	// 		q[3] = knight.pose.orientation.w;

	// 		if (theta_difference(q.getAngle(), angle) < ANGLE_THRESH)
	// 		{
	// 			msg.linear.x = LIN_VEL;
	// 		}
	// 		else
	// 		{
	// 			msg.angular.z = theta_difference(q.getAngle(), angle) > ANGLE_THRESH ? -ANG_VEL : ANG_VEL;
	// 		}
	// 		pub_vel[j].publish(msg);
	// 		j++;
	// 	}
	// 	loop_rate.sleep();
	// }
}

void cell_to_map(int row, int col, int rows, int cols, double& x, double& y)
{
	double cell_y = MAP_Y / (double) rows;
	double cell_x = MAP_X / (double) cols;

	y = MAP_Y / 2.0 - cell_y * (row + 0.5);
	x = -MAP_X / 2.0 + cell_x * (col + 0.5);
}
