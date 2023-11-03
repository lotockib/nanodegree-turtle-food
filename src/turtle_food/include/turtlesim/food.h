#ifndef TURTLE_FOOD_H
#define TURTLE_FOOD_H

#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <iostream>

// TODO namespace wont build, is this needed?
// namespace turtlesim
// {

class Food
{
	public:
		Food(int& argc, char** argv);
		void print_food_info();

	private:
		std::string name_;
		int calories_;

};

#endif // TURTLE_FOOD_H

// }