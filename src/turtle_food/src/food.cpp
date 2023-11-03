#include "turtlesim/food.h"

// namespace turtlesim
// {

// class: contains name of class
Food::Food(int& argc, char** argv) : name_("base food"), calories_(0)
{
	ros::init(argc, argv, "food_node");
}

// print loop: just prints using ROS_INFO
void Food::print_food_info()
{
	ROS_INFO("Food name = %s, calories = %d", name_.c_str(), calories_);
}

// main: create class -> run print loop
int main(int argc, char** argv)
{
	Food new_food = Food(argc, argv);

	while (true)
	{
		new_food.print_food_info();
		sleep(1);
	}

	return(0);

}

// }