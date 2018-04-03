# Flying Cars - Backyard Flyer Project
In this project, a state machine using event-driven programming to autonomously flying a quadcopter drone (in a simulator) in a square (box) pattern is demonstrated. Additionally, callback programming in Python to interact with the drone over a Macvlink connection is demonstrated. 

The state transition methods for drone are:
1. arming transition, 
2. takeoff transition, 
3. waypoint transition, 
4. landing transition(), and 
5. disarming transition()

The callback methods for the drone are:
state_callback, 
local_position_callback, velocity_callback.

### Project Requirements: 
Make a simulated drone fly in a square shape!

### Project Steps
1 - By cloning the FCND-Term1-Starter-Kit directory, set up the virtual environment locally and activate it using the following command
source activate fcnd
2 - Clone the backyard flyer repository.
git clone https://github.com/manopaul/Flying-Cars-P1-Backyard-Flyer.git
3 - Download and run the Simulator and click on the Backyard Flyer icon
4 - Open a Terminal window and run the following command
python backyard_flyer.py

### Project Rubric
You should complete all sections of the BackyardFlyer class that are marked with # TODOs.

Complete the state transition methods for Drone class: arming_transition, takeoff_transition, waypoint_transition, landing_transition, and disarming_transition.
Complete the following callbacks: state_callback, local_position_callback, velocity_callback.
The callbacks check appropriate criteria dependent on the current state and transition to the appropriate next state when those criteria are met. Criteria cannot be based on time!

Mission Analysis
Running backyard_flyer.py correctly commands the vehicle to fly the box mission.

Your project will be evaluated by a Udacity reviewer according to the same Project Rubric. Your project must "meet specifications" in each category in order for your submission to pass.

Submission
When you're ready to submit, click the submit button below and upload a zip file which contains your backyard_flyer.py file.

Feedback
Please fill out the Backyard Flyer Project Feedback Form after you have completed the project.
You have not submitted the project yetSUBMIT PROJECT

CRITERIA
MEETS SPECIFICATIONS
Fill in the state transition methods for Drone class: arming_transition(), takeoff_transition(), waypoint_transition(), landing_transition(), and disarming_transition() are all filled in.

Each of the command methods are filled in with an appropriate command(s) to the vehicle and transitions to the respective state in the state machine.

Fill in the appropriate callbacks. Shell state_callback, local_position_callback, and velocity_callback are provided though they may not be required for all states.

The callbacks check appropriate criteria dependent on the current state and transition to the appropriate next state when that criteria is met. Criteria cannot be time based!

Mission Analysis

CRITERIA
MEETS SPECIFICATIONS
Running the backyard_flyer.py script correctly commands the vehicle to fly in a square.

The vehicle should fly in a square shape and land within 1m of the starting location. The size of each side of the square can be any value you choose.

Suggestions to Make Your Project Stand Out!
The student iterates on their approach to better command the vehicle to fly the box. Additional plots of the vehicle altitude, velocity, or heading to better understand the drone's behavior

