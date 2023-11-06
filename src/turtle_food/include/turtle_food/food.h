#ifndef TURTLE_FOOD_H
#define TURTLE_FOOD_H

#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <turtle_food/Pose.h>
#include "std_msgs/String.h"
#include <turtle_food/SpawnFood.h>

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

// TODO namespace wont build, is this needed?
// namespace turtlesim
// {

class Food
{
	public:
		Food(const ros::NodeHandle &nh);
		void print_food_info();
		void positionCallback(const turtle_food::Pose::ConstPtr& msg);
		void spawnFood();

	private:
		std::string name_;
		int calories_;
		QPointF pos_x_;
		QPointF pos_y_;
		ros::Subscriber sub_;
		ros::NodeHandle nh_;
		int counter_;
};

#endif // TURTLE_FOOD_H

// }
