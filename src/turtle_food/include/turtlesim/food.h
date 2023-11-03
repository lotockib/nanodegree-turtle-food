#ifndef TURTLE_FOOD_H
#define TURTLE_FOOD_H

#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"

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
		Food(int& argc, char** argv);
		void print_food_info();
		void positionCallback(const std_msgs::String::ConstPtr& msg);

	private:
		std::string name_;
		int calories_;
		QPointF pos_x_;
		QPointF pos_y_;
		ros::NodeHandle n_;
};

#endif // TURTLE_FOOD_H

// }