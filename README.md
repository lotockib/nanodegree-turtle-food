# Scope

Capstone project for Udacity C++ Nanodegree course.

# Requirements

You must be running a fresh install of Ubuntu 16.04, or using a fresh Udacity virtual machine from the nanodegree course which also uses Ubuntu 16.04.

# Setup

## Udacity VM Machine
```bash
cd /home/workspace
git clone http://github.com/lotockib/nanodegree-turtle-food.git
cd nanodegree-turtle-food
./setup.sh
```

## Personal Machine
```bash
cd ~
git clone http://github.com/lotockib/nanodegree-turtle-food.git
cd nanodegree-turtle-food
./setup.sh
```

## Setup Troubleshooting

If you are having setup problems, follow these steps and then re-run setup.sh.

1. Verify python is version 3.7 with command `python --version`
2. If python is not version 3.7, download and install Miniconda3-py37_23.1.0-1-Linux-x86_64.sh from [here](https://repo.anaconda.com/miniconda/)

# Run

Open new terminal.  Make sure all other terminals are closed.

Run command to use default of 10 apples
```bash
cd <directory chosen during setup>/nanodegree-turtle-food
./run_turtle_food.sh
```
Or choose the number of apples
```bash
cd <directory chosen during setup>/nanodegree-turtle-food
./run_turtle_food.sh <num apples>
```

When the turtle GUI appears, click the terminal window again, so the terminal can read the keyboard inputs.

Use arrow keys to drive the turtle to each apple and eat them.

Close the GUI and the terminal when done.

#TODO add gif

# Project Details

## Expected Behavior

The project displays a turtle and a number of apples on the screen.  Similar to the snake game, the turtle can be controlled by the user to go to each apple and eat them.  The project is built using ROS1.  The food_node sends ROS messages to the turtlesim_node to created the apples.  The food_node is also listening to the turtle's XY position using a ROS message.  When the turtle gets close to a particular apple, it is deleted and considered eaten.

## File and Class Structure

Two ROS nodes are run for this project.

Node 1 is the [turtlesim_node](./src/turtle_food/src/turtlesim.cpp).  It initially generates the image of the turtle.  It also listens for messages to create and delete food.

Node 2 is the [food_node](src/turtle_food/src/food.cpp).  It generates food in random locations.  It communicates with the turtlesim_node using ROS messages.

This project was developed starting with the [ros turtlesim code](https://github.com/ros/ros_tutorials).  

ROS open source turtlesim code I edited
- [src/turtle_food/src/turtle_frame.cpp](src/turtle_food/src/turtle_frame.cpp)
  - Added QVector for food and food handling
  - Added TurtleFrame::spawnFoodCallback: callback function
  - Added TurtleFrame::spawnFood: creates the food
  - Added TurtleFrame::killFoodCallback: callback function to delete food
- Mis other changes.  All changes can be seen [here](https://github.com/lotockib/nanodegree-turtle-food/pull/1)  

Code I created
- [src/turtle_food/include/turtle_food/food.h](src/turtle_food/include/turtle_food/food.h) and [src/turtle_food/src/food.cpp](src/turtle_food/src/food.cpp)
- [src/turtle_food/srv/SpawnFood.srv](src/turtle_food/srv/SpawnFood.srv)
  - Created this service message so food_node could command food to be created
- [src/turtle_food/srv/KillFood.srv](src/turtle_food/srv/KillFood.srv)
  - Created this service message so food_node could command food to be deleted

# Rubric

This section describes how the Capstone project rubric is satisfied.

## README (All Rubric Points REQUIRED)

### A README with instructions is included with the project

Specification:
```
The README is included with the project and has instructions for building/running the project.
If any additional libraries are needed to run the project, these are indicated with cross-platform installation instructions.
You can submit your writeup as markdown or pdf.
```
Fulfilled:
See [Setup section of this README](#setup)

### The README indicates which project is chosen.
```
The README describes the project you have built.
The README also indicates the file and class structure, along with the expected behavior or output of the program.
```
Fulfilled:
See [Project Details section of this README](#project-details)

### The README includes information about each rubric point addressed.
```
The README indicates which rubric points are addressed. The README also indicates where in the code (i.e. files and line numbers) that the rubric points are addressed.
```
Fulfilled:
The section you are reading now fulfills this.

## Compiling and Testing (All Rubric Points REQUIRED)

### The submission must compile and run.
```
The project code must compile and run without errors.
We strongly recommend using cmake and make, as provided in the starter repos. If you choose another build system, the code must compile on any reviewer platform.
```
Fulfilled:
I have tested this on a Udacity nanodegree virtual machine.  Including a full "data reset" and then using the setup instructions above.

## 5 Of Any Remaining Rubrics Points Must Be Met

Of the listed options, these are the five that are met in this program.

### Rubric Point 1/5: The project demonstrates an understanding of C++ functions and control structures.
```
A variety of control structures are used in the project.
The project code is clearly organized into functions.
```
Fulfilled:
[src/turtle_food/include/turtle_food/food.h](src/turtle_food/include/turtle_food/food.h) and [src/turtle_food/src/food.cpp](src/turtle_food/src/food.cpp) meet this criteria. Food.cpp uses functions, class constructor, while loops, for loops, and a distance measurement function.

### Rubric Point 2/5: The project uses multithreading.
```
The project uses multiple threads in the execution.
```
Fulfilled:
[Food::launchAsync](src/turtle_food/src/food.cpp) uses async tasks to create a task for each food created.  This allows food to check if they've been eaten in parallel.

### Rubric Point 3/5: A mutex or lock is used in the project.
```
A mutex or lock (e.g. std::lock_guard or `std::unique_lock) is used to protect data that is shared across multiple threads in the project code.
```
[Food::spawnFood](src/turtle_food/src/food.cpp) uses a lock_guard when accessing static member data `counter_`.  Each parallel task needs a unique number for its identification.  The lock_guard is used to ensure that each thread has exclusive access when getting and incrementing the number.

### Rubric Point 4/5: The project uses smart pointers instead of raw pointers.
```
The project uses at least one smart pointer: unique_ptr, shared_ptr, or weak_ptr. The project does not use raw pointers.
```
[Food::Food](src/turtle_food/src/food.cpp) and Food:positionCallback use a shared pointer for storing the current position of the turtle that is being read from the ROS message.  This pointer is then shared with each food task, so each food item can efficiently observe the turtle's location, and see if it's close enough to be "eaten".

### Rubric Point 5/5: The project accepts user input and processes the input.
```
The project accepts input from a user as part of the necessary operation of the program.
```
[food.cpp int main()](src/turtle_food/src/food.cpp) accepts optional input from user to specify how many apples they want displayed.  Or if none are provided, it uses 10 apples.
