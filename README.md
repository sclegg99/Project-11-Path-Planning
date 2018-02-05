# Path Planning

## Objective
The objective is to develop a path planning algorithm that will successfully navigate a car through highway traffic.  Success is graded on the car completing one full lap of the simulator track (~4.3 miles) without:
1. Colliding with other traffic,
2. Driving according to the speed limit,
3. There is not excessive acceleration or jerk,
4. Stays within its lane except when changing lanes,
5. Lane changes are completed within 3 secs.
6. Demonstrated ability to select and changes lanes.

The submitted project was tested multiple times using the Term 3 simulator.  The algorithm generally met the success criteria, however, incidents were detected. The source of these incidents and potential methods for resolving them are discussed in the "Areas of Improvement" section.

## Implementation
The algorithm consists of two block; a finite state (FSM) machine and a trajectory planner.  The implemented FSM algorithm was adapted from the behavior planning lesson.  The path fitting algorithm described in the walk through video was implemented as the trajectory planner.

The FSM chooses under what conditions a lane change should be executed by observing the positions and velocities of the surrounding traffic (obtained from the sensor fusion data).  The observations are cast into a cost function that penalizes 1) driving slower than the speed limit, 2) driving too close to traffic immediately in front, 3) moving into an adjacent lane if there is traffic immediately in front or behind.  The planner ranks the costs associated with staying in the current lane (KL) or changing lanes (CL) and returns that choice to the trajectory planner.  In addition, the behavior planner observes the speed of the surrounding traffic and returns to the trajectory planner a new speed that best matches the conditions of the surrounding traffic; that is, either travel at the speed of the surrounding traffic or change speed.

The trajectory planner then takes the new speed, acceleration and lane provided by the FSM and creates a trajectory for the car to follow.  This algorithm was based almost entirely on the project work through video. The trajectory planner predicts the new trajectory in fernet coordinate using a spline fitting.  It then maps predicted trajectory into map coordinates which are sent to the simulator.  Jerk is controlled by selecting an appropriate distance between the s nodes of the spline fitting.  Given a maximum velocity of 50 mph (22.3 m/s), the best node spacing was 30 m. (Smaller node spacings were tested and were shown to result in excessive jerk during lane changes).  As an aside, a jerk minimum trajectory (JMT) class was developed and attempts were made to integrate into the path planning algorithm.  Unfortunately, these integration attempts were not successful.

## Areas of Improvement
The behavior planner generally works well traveling distances of up to 50 miles without incident.  However, the planner does fail when approaching cars that are simultaneously breaking.  Another failing of the planner is that it does not necessarily make optimal choices for which lane to move into.  The root cause of these issues are likely due to the fact that the planner does not attempt to predict the trajectory (include speed and acceleration) of the traffic vehicles.

Another area for improvement would be the implementation of the JMT class.  While the basic code exists, integration of trajectory prediction using the JMT class with the simulator proved to be difficult and hence was abandoned.
