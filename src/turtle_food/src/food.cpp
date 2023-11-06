#include "turtle_food/food.h"

// namespace turtlesim
// {

// class: contains name of class
Food::Food(const ros::NodeHandle &nh, int number=10) : name_("base food"), calories_(0), counter_(0), number_(number)
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
	// Create name using static counter
	std::string full_name = "food" + std::to_string(counter_++);

	/* Spawn food */

	// Create service to spawn
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtle_food::SpawnFood>("spawnFood");
	turtle_food::SpawnFood new_food_srv;

	// Create random position of food
	std::random_device rd;     // Seed
	std::mt19937 rng(rd());    // Random number generator
	std::uniform_real_distribution<> dis(0.0f, 10.0f);
	float random_integer = dis(rng);
	new_food_srv.request.x = dis(rng);
	new_food_srv.request.y = dis(rng);
	new_food_srv.request.name = full_name;	

	// Send service request to spawn
	if (client.call(new_food_srv))
  {
    ROS_INFO("Food created: %s", new_food_srv.response.name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service spawn food");
    return;
  }

	std::this_thread::sleep_for(std::chrono::seconds(1));

	/* Kill food */

	// Create service to kill
	ros::ServiceClient client_kill = n.serviceClient<turtle_food::KillFood>("killFood");
	turtle_food::KillFood kill_food_srv;

	// Send service request to kill
	kill_food_srv.request.name = full_name;
	if (client_kill.call(kill_food_srv))
  {
    ROS_INFO("Food killed: %s", full_name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service kill food");
    return;
  }

	return;
}

// main: create class -> run print loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "food");
	Food new_food = Food(ros::NodeHandle("food_handle"));

	std::vector<std::future<void>> futures;

	for (int i = 0; i < 10; i++)
	{
		// threads.append(std::thread(&Food::spawnFood, &new_food));
		futures.emplace_back(std::async(std::launch::async, &Food::spawnFood, &new_food));
	}
	
	for (const std::future<void> &ftr : futures)
	{
		ftr.wait();
	}

	return 0;

}

// }
