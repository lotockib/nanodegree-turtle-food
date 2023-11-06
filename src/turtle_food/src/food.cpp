#include "turtle_food/food.h"
#include <sstream> 

// namespace turtlesim
// {

// class: contains name of class
Food::Food(const ros::NodeHandle &nh) : name_("base food"), calories_(0), x_(0), y_(0)
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

void Food::spawnFood()
{
		// counter_++;

    // stringstream object
    std::stringstream stream;
    
    // insertion of integer variable to stream
    stream << counter_++;
    
    // variable to hold the new variable from the stream
    std::string counter;
    
    // extraction of string type of the integer variable
    stream >> counter;
    
    // cout << "The user is " + age_as_string + " years old";
    // The user is 20 years old

		std::string full_name = "food" + counter;

	/* Spawn food manually */
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtle_food::SpawnFood>("spawnFood");
	turtle_food::SpawnFood new_food_srv;
	// std::string name = "food1";

	// tried to use std::mt19937 but got compile errors I could not solve
	int min = 0;
	int max = 10;
	int output = min + (rand() % static_cast<int>(max - min + 1));

	x_ = min + (rand() % static_cast<int>(max - min + 1));;
	y_ = min + (rand() % static_cast<int>(max - min + 1));;
	new_food_srv.request.x = x_;
	new_food_srv.request.y = y_;
	new_food_srv.request.name = full_name;	
	if (client.call(new_food_srv))
  {
    ROS_INFO("Food created: %s", new_food_srv.response.name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service spawn food");
    return;
  }
	return;
}

// main: create class -> run print loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "food");
	Food new_food = Food(ros::NodeHandle("food_handle"));

	for (int i = 0; i < 5; i++)
	{
		new_food.spawnFood();
	}

	ROS_INFO("Listenening for turtle1/pose");
	ros::spin();

	return 0;

}

// }
