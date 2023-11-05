#include "turtle_food/food.h"

// namespace turtlesim
// {

// class: contains name of class
Food::Food(const ros::NodeHandle &nh) : name_("base food"), calories_(0)
{
	nh_ = nh;
	sub_ = nh_.subscribe("/turtle1/pose", 1000, &Food::positionCallback, this);
}

// print loop: just prints using ROS_INFO
void Food::print_food_info()
{
	ROS_INFO("Food name = %s, calories = %d", name_.c_str(), calories_);
}

// callback to lister to turtle position
void Food::positionCallback(const turtle_food::Pose::ConstPtr& msg)
{
	ROS_INFO("I heard: [%f] [%f]", msg->x, msg->y);
}

void nonClassCallback(const turtle_food::Pose::ConstPtr& msg)
{
	ROS_INFO("I heard: [%f] [%f]", msg->x, msg->y);
}

// main: create class -> run print loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "food");
	Food new_food = Food(ros::NodeHandle("food_handle"));

	ROS_INFO("Listenening for turtle1/pose");
	ros::spin();

	return 0;

}

// }
