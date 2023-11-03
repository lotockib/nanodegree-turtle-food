#ifndef TURTLE_FOOD_H
#define TURTLE_FOOD_H

#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <turtlesim/Pose.h>
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
		Food(const ros::NodeHandle &nh);
		void print_food_info();
		void positionCallback(const turtlesim::Pose::ConstPtr& msg);

	private:
		std::string name_;
		int calories_;
		QPointF pos_x_;
		QPointF pos_y_;
		ros::Subscriber sub_;
		ros::NodeHandle nh_;
};

#endif // TURTLE_FOOD_H

// }
