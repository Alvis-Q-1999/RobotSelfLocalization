# RobotSelfLocalization
Includes 2 mobile robot self-localization methods.
1 PSO-PF: Combined PSO and PF.
  PSO is used to divide the particle swarm into several similar subgroups.
  PF is applied to each subgroup to estimate the real position state of the robot.
2 PSO-QL: Combined PSO and Q-Learning
  PSO is used to divide the particle swarm into several similar subgroups.
  QL is usded to get the global map Q-table that helps robot select a best way to move to the best position to estimate the real position
