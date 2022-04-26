# q_learning_project
Zoa Katok, Evan Wu

## Implementation Plan
### Q-learning algorithm
We will get Q values for each possible state-action pair by initializing random possible trajectories and iterating through the Q-learning algorithm. We will probably have to tune parameters like the rewards for each transition, discount factor, and learning rate. We will consider the matrix to be converged when the difference for all Q-values between the current and previous timestep in the matrix is less than a small threshold, i.e. we have learned sufficiently about the task and environment. We follow the policy that maximizes the reward, in particular we're thinking of using a greedy or randomized model that follows the state path with highest next Q-value as the state space is not that large at 64 state-action pairs. 

### Robot perception
We will use information from the robot camera and distance measurements from LiDAR to determine the position and location of the various pieces of state space. Color will help us to differentiate the different objects, while tags will be processed using the ArUco library. We may also want to detect whether the placement of the objects in front of the tags matches what we expect of that state. 

### Robot manipulation and movement
We will use proportional control and the perception to move the robot to the necessary objects and tags. We will learn to use the manipulation arm with the MoveIt module to allow the robot to pick up and put down the objects as necessary to achieve the desired state space. 

We plan to implement the Q-learning algorithm and complete training of the matrix as well as complete robot perception by next Tuesday, then focus on getting the physical robot to be able to carry out the policy plan the next week. 
