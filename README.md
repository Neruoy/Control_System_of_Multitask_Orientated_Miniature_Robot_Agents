# INTRODUCTION
In daily life, the air conditioner is an indispensable household appliance. In large buildings, the central air-conditioning refrigeration system usually requires many pipe connections for air circulation. However, because the air-conditioning ducts are usually very narrow and the structure is complicated, it is very difficult to clean and dredge directly by manpower.

# OBJECTIVE
To design a group of robotic agents that can achieve following two tasks:
1.	Go through a maze based on any given starting point and end point with knowledge of the structure of the maze.
2.	Distinguish various target using color detection and perform different behaviors based on the classification.

# TECHNICAL CONTENTS
## Design of Test Field
The arena for the whole project is consisting of two parts, which will set different tasks to test the functions of robotic insects. In the first part, a maze is designed to test the performance of pathfinding and obstacle avoidance. To fully tested the performance of robots, we stratify the difficulty by distance, obstacles, branches, blind alleys.

<img src="/Users/euron/Documents/Notes/notes.assets/122343511-5ab88780-cf78-11eb-8a4a-d80cf819b52a.png" alt="image" style="zoom:50%;" />

After designing the structure of the maze, we build the maze by hot melt adhesive and hardboards.

<img src="/Users/euron/Documents/Notes/notes.assets/122343619-7a4fb000-cf78-11eb-81d4-039b66615bdd.png" alt="image" style="zoom:50%;" />

## Pose Estimation
In this project, we use ArUco markers to help us with localization.

<img src="/Users/euron/Documents/Notes/notes.assets/image-20210617143601046.png" alt="image-20210617143601046" style="zoom:50%;" />

the position of the agent is also represented by the pose of an ArUco marker. The program will start to run after the initialization of the maze is finished and the agent is found. The sequence will be:

1. Open the external camera.

2. Scan the ArUco Markers in the maze until information of all markers are recorded.

3. Scan the ArUco Markers on the agents.

4. Start running the maze.

The result of the scanned maze is as the following picture shown:

<img src="/Users/euron/Documents/Notes/notes.assets/image-20210617143730696.png" alt="image-20210617143730696" style="zoom:50%;" />

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

![image-20210617144245120](/Users/euron/Documents/Notes/notes.assets/image-20210617144245120.png)

# RESULT

Check `demo_videos` folder to see demonstration video.