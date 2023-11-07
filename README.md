# Scope

Capstone project for Udacity C++ Nanodegree course.  This code was developed started with the [ros turtlesim code](https://github.com/ros/ros_tutorials).

# Requirements

You must be running a fresh install of Ubuntu 16.04, or using a Udacity virtual machine from the nanodegree course.

# Setup

1. Verify python is version 3.7 with command `python --version`
2. If python is not version 3.7, download and install Miniconda3-py37_23.1.0-1-Linux-x86_64.sh from [here](https://repo.anaconda.com/miniconda/)
3. Clone repo
    ```bash
    cd /home/workspace (for udacity VM) or cd ~ (for any other machine)
    git clone http://github.com/lotockib/ros-tutorials.git
    ```
4. Run setup script
    ```bash
    cd /home/workspace (for udacity VM) or cd ~ (for any other machine)
    cd ros-tutorials
    ./setup.sh
    ```

# Run

1. Close all prior terminals. In a new terminal, run roscore
    ```bash
    roscore
    ```

2. In a new terminal, run talker
    ```bash
    cd /home/workspace/ros-tutorials (for udacity VM) or cd ~/ros-tutorials (for any other machine)
    source devel/setup.bash
    rosrun beginner_tutorials talker
    ```

3. In a new terminal, run listener
    ```bash
    cd /home/workspace/ros-tutorials (for udacity VM) or cd ~/ros-tutorials (for any other machine)
    source devel/setup.bash
    rosrun beginner_tutorials listener
    ```

# Project Overview

This project was developed starting with the [ros turtlesim code](https://github.com/ros/ros_tutorials).  Some of the pre-existing code was edited:


# Rubric

This section describes how the Capstone project rubric is satisfied.

## A README with instructions is included with the project

Specification:
```
The README is included with the project and has instructions for building/running the project.
If any additional libraries are needed to run the project, these are indicated with cross-platform installation instructions.
You can submit your writeup as markdown or pdf.
```
Fulfilled:
See [Setup section of this README](#setup)

## The README indicates which project is chosen.
```
The README describes the project you have built.
The README also indicates the file and class structure, along with the expected behavior or output of the program.
```



