# Control-System-of-Multitask-Orientated-Miniature-Robot-Agents
This project is aim to design control system of a group of robotic agents that can
1. go through the ducts with knowledge of the structure of the duct system and
2. distinguish various target, and perform different behaviors based on the classification.
## Task 1
### Design of the arena
We abstract the duct system into a maze, and an external camera and Aruco Markers are used to scan the maze. The maze is like this, it has four gates, and going between different gates will gives different degrees of difficulty.
![image](https://user-images.githubusercontent.com/72918178/119108570-e8c15100-ba52-11eb-8bed-241ffbf40bea.png)
### The control system
Due to the simplicity of our small agent, and structure of our code， we implement a mathematical model called state machine as the main control system of our agents.
![image](https://user-images.githubusercontent.com/72918178/119109252-9b91af00-ba53-11eb-9d38-96636e3dffd9.png)
### Demostration
Check `task1_demo` for demonstration videos.
