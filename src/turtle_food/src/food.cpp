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
	if (food_futures_.empty())
	{ 
		return true;
	}

	for (int i = 0; i < food_futures_.size(); i++)
	{
		try
		{
			food_futures_[i].wait_for(std::chrono::milliseconds(10));
			food_futures_.erase(food_futures_.begin() + i);
		}
		catch(const std::exception& e)
		{
			// do nothing, this task is still running
			return false;
		}
	}

	// each food future successfully erased, now it is empty
	ROS_INFO("All food has been eaten!");
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
		// threads.append(std::thread(&Food::spawnFood, &new_food));
		food_futures_.emplace_back(std::async(std::launch::async, &Food::spawnFood, this));
	}
}

void Food::spawnFood()
{
	// Create name using static counter
	std::string full_name = "food" + std::to_string(counter_++);

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
	float food_x = dis(rng);
	float food_y = dis(rng);
	new_food_srv.request.x = food_x;
	new_food_srv.request.y = food_y;
	// new_food_srv.request.theta = 0;
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
	while(calculateDistance(food_x, food_y, pose_->x, pose_->y) > threshold_)
	{
		// ROS_INFO("Food %s detects turtle at [%f] [%f]", full_name.c_str(), pose_->x, pose_->y);
		// ROS_INFO("Food %s is located at [%f] [%f]", full_name.c_str(), food_x, food_y);
		ros::spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// Create service to kill
	ros::ServiceClient client_kill = n.serviceClient<turtle_food::KillFood>("killFood");
	turtle_food::KillFood kill_food_srv;

	// Send service request to kill
	kill_food_srv.request.name = full_name;
	if (client_kill.call(kill_food_srv))
  {
    ROS_INFO("Food killed: %s, food remaining: %d", full_name.c_str(), (int) food_futures_.size());
  }
  else
  {
    ROS_ERROR("Failed to call service kill food");
    return;
  }

	return;
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

	// ROS_INFO("argc = %d", argc);
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

	Food new_food = Food(ros::NodeHandle("food_handle"), num_apples);

	// Food new_food = Food(ros::NodeHandle("food_handle"));
	new_food.feedingTime();
	return 0;
}

// }
