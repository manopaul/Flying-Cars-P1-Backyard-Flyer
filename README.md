# Flying Cars - Backyard Flyer Project
In this project, a state machine using event-driven programming to autonomously flying a quadcopter drone (in a simulator) in a square (box) pattern is demonstrated. Additionally, callback programming in Python to interact with the drone over a Mavlink connection is demonstrated. 

The state transition methods for drone are:
- arming transition, 
- takeoff transition, 
- waypoint transition, 
- landing transition(), and 
- disarming transition()

The callback methods for the drone are:
- state callback,
- local position callback, and
- velocity callback.

## Project Requirements: 
Make a simulated drone fly in a square shape!

## Usage
### Without visualization
1. By cloning the FCND-Term1-Starter-Kit directory, set up the virtual environment locally and activate it using the following command

  `source activate fcnd`

2. Clone the backyard flyer repository.

   `git clone https://github.com/manopaul/Flying-Cars-P1-Backyard-Flyer.git`

3. Download and run the Simulator and click on the Backyard Flyer icon

4. Open a Terminal window and run the following command

  `python backyard_flyer.py`

You should be able to observe what is shown in the video below

### Without visualization
1. By cloning the FCND-Term1-Starter-Kit directory, set up the virtual environment locally and activate it using the following command

  `source activate fcnd`

2. Clone the backyard flyer repository.

  `git clone https://github.com/manopaul/Flying-Cars-P1-Backyard-Flyer.git`

3. Download and run the Simulator and click on the Backyard Flyer icon

4. Open a Terminal window and run the following command to start the visdom.server

  `python -m visdom.server`

If the server starts successfully, You will be presented with a link. The default link is http://localhost:8097

5. Open a web browser and paste the link in the URL address bar and navigate to this link. 

6. Open another terminal window and run the following command

  `python backyard_flyer_with_visualization.py`

You should see the drone trajectory plot in the browser window

You should be able to observe what is shown in the video below

### Observations and Suggestions
Plotting the drone trajectory indicated that the target values are not absolute.
The Euler angles (Pitch, Roll and Yaw) have an impact on the path of the drone. 
The code should be modified to take into account the Euler angles and also the velocity of the drone to more accurately traverse the target waypoints.
