#include "turtlesim/food.h"

// namespace turtlesim
// {

// class: contains name of class
Food::Food(int& argc, char** argv) : name_("base food"), calories_(0), n_(ros::NodeHandle("testname"))
{
	ros::Subscriber sub = n_.subscribe("pose", 1, &Food::positionCallback, this);
}

// print loop: just prints using ROS_INFO
void Food::print_food_info()
{
	ROS_INFO("Food name = %s, calories = %d", name_.c_str(), calories_);
}

// callback to lister to turtle position
void Food::positionCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// main: create class -> run print loop
int main(int argc, char** argv)
{
	ros::init(argc, argv, "food_node");
	Food new_food = Food(argc, argv);

	while (true)
	{
		ros::spin();
		new_food.print_food_info();
		sleep(1);
	}

	return(0);

}

// }
