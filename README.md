# Maze-Solver-Turtlebot3

## Overview  

This project showcases a **Maze Solver Robot** simulated in the **Gazebo** environment using the **TurtleBot3 Burger model**. The robot is programmed to solve a maze by employing two fundamental algorithms:  

- **Right Wall-Following**  
- **Left Wall-Following**  

The implementation is done using **ROS (Robot Operating System)**.

## Installation and Execution  

Follow these steps to set up and run the project:  

1. **Create a ROS Workspace**:  
   ```bash  
   mkdir -p ~/catkin_ws/src  
   cd ~/catkin_ws  
   catkin_make  
   ```  

2. **Clone the Repository**:  
   ```bash  
   cd ~/catkin_ws/src  
   git clone https://github.com/<your-username>/maze-solver-robot.git  
   ```  

3. **Build the Workspace**:  
   ```bash  
   cd ~/catkin_ws  
   catkin_make  
   ```  

4. **Source the Workspace**:  
   ```bash  
   source devel/setup.bash  
   ```  

5. **Launch the Simulation**:  
   Run one of the provided launch files to start Gazebo with the maze environment:  
   ```bash  
   roslaunch maze_solver test1.launch  
   ```  
   (*Alternatively, you can use `test2.launch` or `test3.launch` for different maze configurations.*)  

6. **Run the Maze Solver Script**:  
   Open a new terminal, source the workspace, and run the Python script to start the robot:  
   ```bash  
   cd ~/catkin_ws  
   source devel/setup.bash  
   rosrun maze_solver main.py  
   ```  

## Features  

- **Simulation Environment**: Utilizes the Gazebo simulator to provide a realistic maze scenario.  
- **Robot Platform**: TurtleBot3 Burger, a lightweight and versatile robotic model.  
- **Wall-Following Algorithms**:  
  - **Right Wall-Following**: The robot adheres to the right wall of the maze to find its way to the exit.  
  - **Left Wall-Following**: The robot follows the left wall of the maze to navigate to the goal.  

## Demo  

Check out the robot in action! 

![test1_video](https://github.com/user-attachments/assets/6c143523-8f9d-4739-8b6b-ce02d9c9609e)



## Path Moved by the Robot

The graph below illustrates the path taken by the robot while solving the maze in the demo video.
![test1_path](https://github.com/user-attachments/assets/afc5b03b-3116-4e2e-9b61-6863e17e3bbc)

