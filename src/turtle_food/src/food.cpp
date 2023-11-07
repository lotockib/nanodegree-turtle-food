#include "turtle_food/food.h"
#include <string>

// namespace turtlesim
// {

// class: contains name of class
Food::Food(const ros::NodeHandle &nh, int number=100) 
: name_("base food"), calories_(0), counter_(0), num_food_(number), turtle_comms_running_(false), threshold_(1.0)
{
	nh_ = nh;
	sub_ = nh_.subscribe("/turtle1/pose", 1000, &Food::positionCallback, this);
	pose_ = std::make_shared<turtle_food::Pose>();
	waitForTurtle(); // Wait for turtle to be pubslishing position
	launchAsync(); // Create task for each food being added
}

// print loop: just prints using ROS_INFO
void Food::print_food_info()
{
	ROS_INFO("Food name = %s, calories = %d", name_.c_str(), calories_);
}

// callback to lister to turtle position
void Food::positionCallback(const turtle_food::Pose::ConstPtr& msg)
{
	if (!turtle_comms_running_) { turtle_comms_running_ = true; }
	// ROS_INFO("I heard: [%f] [%f]", msg->x, msg->y);
	pose_->x = msg->x;
	pose_->y = msg->y;
}

void Food::waitForTurtle()	
{
	while (!turtle_comms_running_)
	{
		ros::spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return;
}

bool Food::foodGone()
{

	for (auto & ftr : food_futures_)
	{
		if (ftr.valid())
		{
			if (ftr.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
			{
				ftr.get(); // get the future to force it invalid
			} else {
				return false; // there's at least one valid future that isn't ready to be read yet, food not gone
			}
		}
	}

	// each food future successfully erased, now it is empty
	ROS_INFO("ALL FOOD HAS BEEN EATEN!");
	return true;
}

void Food::feedingTime()
{
	while(!foodGone())
		{
			ros::spinOnce();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
}

void Food::launchAsync()
{
	for (int i = 0; i < num_food_; i++)
	{
		food_futures_.emplace_back(std::async(std::launch::async, &Food::spawnFood, this));
	}
}

std::string Food::spawnFood()
{
	// Create name using static counter
	mutex_.lock(); // counter_ is shared across tasks, so lock when modifying
	std::string full_name = "food" + std::to_string(counter_++);
	mutex_.unlock();

	/* Spawn food */

	// Create service to spawn
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtle_food::SpawnFood>("spawnFood");
	// ros::ServiceClient client = n.serviceClient<turtle_food::Spawn>("spawn");
	turtle_food::SpawnFood new_food_srv;
	// turtle_food::Spawn new_food_srv;

	// Create random position of food
	std::random_device rd;     // Seed
	std::mt19937 rng(rd());    // Random number generator
	std::uniform_real_distribution<> dis(0.0f, 10.0f);
	float food_x, food_y;
	do // Position must start away from turtle so it's not automatically eaten
	{
		food_x = dis(rng);
		food_y = dis(rng);
	}
	while (calculateDistance(food_x, food_y, pose_->x, pose_->y) < 2 * threshold_);

	// Build service request
	new_food_srv.request.x = food_x;
	new_food_srv.request.y = food_y;
	new_food_srv.request.name = full_name;	

	// Send service request to spawn
	if (!client.call(new_food_srv))
  {
    ROS_ERROR("Failed to call service spawn food");
		return full_name;
  }

	/* Kill food */
	while(calculateDistance(food_x, food_y, pose_->x, pose_->y) > threshold_)
	{
		ros::spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// Create service to kill
	ros::ServiceClient client_kill = n.serviceClient<turtle_food::KillFood>("killFood");
	turtle_food::KillFood kill_food_srv;

	// Send service request to kill
	kill_food_srv.request.name = full_name;
	if (!client_kill.call(kill_food_srv))
  {
    ROS_ERROR("Failed to call service kill food");
  }

	return full_name;
}

float Food::calculateDistance(float x1, float y1, float x2, float y2)
{
	float dx = x1-x2;
	float dy = y1-y2;
	return sqrt(pow(dx,2) + pow(dy,2));
}

// main: create class -> run print loop
int main(int argc, char **argv)
{
	// Food new_food;
	ros::init(argc, argv, "food");

	// Pass number of apples if provided
	int num_apples;
	if (argc == 2)
	{
		ROS_INFO("detected argument %s", argv[1]);
		num_apples = std::stoi(argv[1]);
	}
	else
	{
		num_apples = 10;
	}

	Food new_food(ros::NodeHandle("food_handle"), num_apples);
	new_food.feedingTime();
	return 0;
}

// }
