# INTRODUCTION
In daily life, the air conditioner is an indispensable household appliance. In large buildings, the central air-conditioning refrigeration system usually requires many pipe connections for air circulation. However, because the air-conditioning ducts are usually very narrow and the structure is complicated, it is very difficult to clean and dredge directly by manpower.

# OBJECTIVE
To design a group of robotic agents that can achieve following two tasks:
1.	Go through a maze based on any given starting point and end point with knowledge of the structure of the maze.
2.	Distinguish various target using color detection and perform different behaviors based on the classification.

# TECHNICAL CONTENTS
## Design of Test Field
The arena for the whole project is consisting of two parts, which will set different tasks to test the functions of robotic insects. In the first part, a maze is designed to test the performance of pathfinding and obstacle avoidance. To fully tested the performance of robots, we stratify the difficulty by distance, obstacles, branches, blind alleys.

<img src="https://user-images.githubusercontent.com/72918178/122345987-0fec3f00-cf7b-11eb-8417-11b8ea674d55.png" alt="image" style="zoom:50%;" />

After designing the structure of the maze, we build the maze by hot melt adhesive and hardboards.

<img src="https://user-images.githubusercontent.com/72918178/122346203-4aee7280-cf7b-11eb-9a9c-99e618405905.png" alt="image" style="zoom:50%;" />

## Pose Estimation
In this project, we use ArUco markers to help us with localization.

<center><img src="https://user-images.githubusercontent.com/72918178/122346736-df58d500-cf7b-11eb-929c-e31df58eb6e5.png" alt="image-20210617143601046" style="zoom:50%;" /></center>

the position of the agent is also represented by the pose of an ArUco marker. The program will start to run after the initialization of the maze is finished and the agent is found. The sequence will be:

1. Open the external camera.

2. Scan the ArUco Markers in the maze until information of all markers are recorded.

3. Scan the ArUco Markers on the agents.

4. Start running the maze.

The result of the scanned maze is as the following picture shown:

<center><img src="https://user-images.githubusercontent.com/72918178/122346894-09aa9280-cf7c-11eb-9070-7c55d18216d7.png" alt="image-20210617143730696" style="zoom:50%;" /></center>

After convert the maze into graph, we can perform Dijkstra algorithm to find the optimal path.

## Control System

### State machine

The finite state machine (FSM), or as it is alternatively known, finite state automaton (FSA), state machine (SM), and finite automaton (FA), is a mathematical model is a mathematical model of computation. It is an abstract machine that can be in one and only one state at a given time. A finite state machine is defined by a list of its states, the initial states of the machine, and the inputs that trigger the transition, which is the terminology refers to shifting from one state to another. We will implement the state machine as our main control algorithm.



To define all the states of the agent, it is necessary to consider all the movements it needs. First, the agent needs to include basic movements (i.e., going forward, turn left, turn right) to complete the task of going through the maze. Secondly, for the convenience of later expansion, an intermediate state is needed as a state transfer station. Finally, when multiple agents meet in the maze, an avoidance movement is required. In summary, the states of each agent include:

- State 0: Start

- State M: Intermediate state

- State A: Going forward

- State B: Turning

- State C: Avoiding



The state diagram of our agent is shown below:

![image-20210617144245120](https://user-images.githubusercontent.com/72918178/122347059-36f74080-cf7c-11eb-8c86-c4f0fd943bda.png)

# RESULT

https://user-images.githubusercontent.com/72918178/122349591-f9e07d80-cf7e-11eb-85d8-aa4f03f825da.mp4

Check `demo_videos` folder for more demonstration video (including avoidance).

