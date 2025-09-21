ROS-PAT Autonomous Firefighting Simulation

An autonomous multi-robot system for firefighting in a simulated 12x12 grid environment. This project integrates ROS (Robot Operating System) for low-level control, Gazebo for physics simulation, and the Process Analysis Toolkit (PAT) for high-level, formally verified mission planning. A Scout robot maps the environment and identifies fire locations, while a Firefighting robot executes a PAT-generated plan to extinguish them.

(Image from the project report showing the Gazebo simulation environment)
Features

    Autonomous Mapping: A Scout robot explores a 12x12 grid world to generate a map and locate fires.

    Formal Planning & Verification: Uses the Process Analysis Toolkit (PAT) to generate a provably correct action sequence for the firefighting robot.

    Dynamic Plan Execution: A Firefighting robot follows the generated plan to navigate, extinguish all fires, and manage resources (e.g., refilling water).

    Multi-Robot System: Demonstrates coordination between two specialized robots (Scout and Firefighter).

    ROS & Gazebo Integration: Built on industry-standard tools for robotics simulation and control.

Tech Stack

    Simulation: Gazebo 11

    Robotics Middleware: ROS Noetic

    High-Level Planning: Process Analysis Toolkit (PAT) v3

    Programming Language: C++ (for ROS nodes)

    Operating System: Ubuntu 20.04 LTS

System Architecture

The system consists of two robots and several ROS nodes that handle mapping, planning, and execution.

    Scout Robot: Explores the grid, detects walls and fires using simulated sonar, and publishes a complete map.

    Translator Node (map_node):

        Receives the text map from the Scout.

        Converts the map into a PAT-readable CSP (Communicating Sequential Processes) model.

        Invokes PAT to generate a verified plan.

        Parses the PAT trace into a simple sequence of commands.

    Firefighting Robot (firebot_executor_node):

        Reads the command sequence.

        Executes the plan in Gazebo, navigating to fires, extinguishing them, and returning to a refill station when necessary.

(Image from the project report showing the data flow)
Getting Started

Follow these instructions to get a local copy of the simulation up and running.
Prerequisites

    Ubuntu 20.04 LTS

    ROS Noetic installed and configured.

    Gazebo 11 (comes with the full desktop install of ROS).

    Mono framework to run PAT (sudo apt install mono-complete).

    A configured catkin workspace.

Installation & Setup

    Clone the Repository:
    Clone this repository into the src folder of your catkin workspace.

    cd ~/catkin_ws/src
    git clone [https://github.com/your-username/your-repository-name.git](https://github.com/your-username/your-repository-name.git)

    Build the Project:
    Navigate to the root of your workspace and build the packages.

    cd ~/catkin_ws
    catkin_make

    Source the Workspace:
    Source your workspace's setup file to make the ROS nodes available in your terminal session.

    source ~/catkin_ws/devel/setup.bash

    Note: You may want to add this command to your .bashrc file.

Running the Simulation

The simulation is launched in stages. Each command should be run in a new terminal tab.

    Launch the Gazebo World:
    This starts the Gazebo simulation, loads the 12x12 house grid, and spawns the Scout and Firebot robots.

    roslaunch firebot_system house_sim.launch

    Start the Robot Sensors:
    This node simulates the sonar sensors required for the Scout to explore.

    rosrun firebot_system sonars

    Begin Mapping (Scout Robot):
    This command starts the Scout robot's exploration routine. It will navigate the entire grid to find fires. This process can take several minutes.

    rosrun firebot_system scout_node

        To Skip Mapping: If you want to save time, you can skip this step and use a pre-existing map file (full_explored_map.txt) located in the project directory.

    Generate the Plan (Translator Node):
    This node reads the map, converts it to a CSP model, runs PAT to find a solution, and saves the final commands to robot_movements.txt.

    rosrun firebot_system map_node

    Execute the Mission (Firefighting Robot):
    Finally, this command starts the Firefighting robot. It will read the plan from robot_movements.txt and execute the mission.

    rosrun firebot_system firebot_executor_node

    The robot will now navigate the grid, extinguish fires, refill its water tank if needed, and return to its starting base.

Future Work

    Scalability: Improve planning for larger grids to mitigate exponential state-space growth in PAT.

    Dynamic Fires: Extend the model to handle fires that can spread over time.

    Multiple FireBots: Adapt the planner to coordinate tasks between multiple firefighting robots.

License

Distributed under the MIT License. See LICENSE for more information.
