# q_learning_project
Zoa Katok, Evan Wu


## Writeup
### Objectives
The goal of this project is to use the Q-learning reinforcement learning algorithm to train a robot to be able to match colored tubes to a respective AR tag. We then programmed the necessary components to allow a robot to perform this task in a physical environment.

### High-level description
The Q-learning algorithm iteratively trains a robot to perform a specific task by reinforcing particular state-action combinations that lead the robot to the desired outcome. In this case, the task was to move colored tubes in front of AR tags in a specific order. To train the robot, we allowed it to explore a virtual version of the environment by randomly performing permissible actions. As our state space is small, over time the robot accumulates experiences of achieving the desired configuration and updates the state-action pairs in the Q-matrix to reflect the trajectory of achieved rewards. Recapitulating this path using a greedy algorithm on the trained Q-table allows for selection of the correct sequence of actions to achieve the matching tube-AR tag pattern.

### Q-learning algorithm
Lines 97-115 q_learning_algorithm()
We selected actions for the robot to take from a given state randomly based on permissible actions specified by the "action_matrix". If there are no permissible actions we reset the world. The randomly selected action is then matched to the "actions" table and we feed the details to the phantom robot to perform.

Lines 117-130 q_learning_algorithm() and lines 68-71 get_reward()
The Q-matrix is updated using the Bellman equation, where current and projected rewards for taking a certain action given a state are updated based on feedback from the environment. We get the feedback in the form of rewards from get_reward(), which subscribes to the environment node that calculates the reward from a particular robot action. To ensure correct sequencing of actions and rewards, we make sure to receive a new signal from the environment before calculating Q.

Lines 74-83 is_converged()
We determined whether the Q matrix is converged or not by determining if the Q-matrix is still changing significantly. We do this by setting a threshold value epsilon where if the difference between the past and current Q values are all less than epsilon, we consider the matrix to be not changing. Additionally, we stipulate that the matrix needs to not be changing for 1000 consecutive timesteps, mostly because our random sampling approach needs some leeway to fully explore the state space.

We will execute the path to most likely receive a reward using a greedy algorithm on the converged Q matrix. Given a state, we will choose the action with maximum Q value (or random if multiple actions share the max) until we reach a final configuration.

## Robot perception

- identifying locations and identities of colored objects:
- identifying locations and identities of tags:

## Robot manipulation and movement

- Moving to the right spot: we determined a particular ideal distance for picking up the tube. So we had the robot first move such that the correct color is in the middle of its field of vision, then move the appropriate distance away (about 0.28 meters away)
- Picking up the tube: we experimented with different arm positions to determine which are best. So when the robot is in the right position, we have the robot open its gripper all the way (for maximum margin for error), then go to the predetermined "arm down" position, then close the gripper sufficiently, then go to the predetermined "arm up" position.
- Moving to the desired destination: we made sure the arm up position does not block the camera at all because the robot has to use its camera to locate the AR tags. Then it is a matter of rotating to find the tag and moving towards it. We make sure to be a sufficient distance from the wall so the robot has enough space to put the tube down.
- Putting the tube down: Since we are in position, we just move the arm to the "arm down" position, open the gripper, and move the arm back to the "arm up" position.


## Challenges

- q-matrix setup: At first, it was a bit difficult for us to wrap our heads around the structure of the q-matrix data provided and what we were supposed to do with it. But by sitting together and going through it all step by step, we eventually figured it out.
- working with the camera: Implementing camera-based motion was difficult because we found that by the time the frame where the target was in sight was relayed back to the robot, the robot had already rotated past the object.
- working with the arm: We wanted to test out arm movement with the visualization software, but it wasn't working too well. We solved this by carefully testing on the physical robot in a separate file, modifying little by little starting from the example arm movement in the tutorial on the website.

## Future work

- We are pretty happy with our implementation. The biggest hurdle was the camera, so it would be ideal to eliminate that issue. Perhaps there is a way to guarantee a good connection and low latency for the camera. But perhaps there is another way to do this project that relies less on the camera, either by deciding that the camera is only needed in short particular bursts or by using some other method to navigate, such as mapping the area before beginning.

## Takeaways

- Throughout the project, we would make a pseudocode outline right in the python file before writing real code.
This was really helpful so that we didn't have to work from a blank slate, and it helped us organize our thoughts before we got in the weeds. Doing the outline together also helped us both understand what we were doing so that when we divided the coding labor, it wasn't too difficult for the one person to understand what the other had done.
- Our workflow was to start by looking over the project/phase we were going to start on together, then agree on what we should do before the next meeting. This worked well for us because it allowed us to have good communication without having to carve out enough time to do the entire project together.

## Implementation Plan
### Q-learning algorithm
We will get Q values for each possible state-action pair by initializing random possible trajectories and iterating through the Q-learning algorithm. We will probably have to tune parameters like the rewards for each transition, discount factor, and learning rate. We will consider the matrix to be converged when the difference for all Q-values between the current and previous timestep in the matrix is less than a small threshold, i.e. we have learned sufficiently about the task and environment. We follow the policy that maximizes the reward, in particular we're thinking of using a greedy or randomized model that follows the state path with highest next Q-value as the state space is not that large at 64 state-action pairs.

### Robot perception
We will use information from the robot camera and distance measurements from LiDAR to determine the position and location of the various pieces of state space. Color will help us to differentiate the different objects, while tags will be processed using the ArUco library. We may also want to detect whether the placement of the objects in front of the tags matches what we expect of that state.

### Robot manipulation and movement
We will use proportional control and the perception to move the robot to the necessary objects and tags. We will learn to use the manipulation arm with the MoveIt module to allow the robot to pick up and put down the objects as necessary to achieve the desired state space.

We plan to implement the Q-learning algorithm and complete training of the matrix as well as complete robot perception by next Tuesday, then focus on getting the physical robot to be able to carry out the policy plan the next week.
