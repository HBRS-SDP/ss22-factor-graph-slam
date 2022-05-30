## PlannerSLAM_using_collected_data

#### How to run the file
1. Clone and build the GTSAM library
2. Copy the python file and paste it in "/GTSAM/gtsam/python/gtsam/examples"
3. change the file path for collected data and where final final results to be stored

#### How the factor graph is constructed

1. In this, we considered each time step as node in the graph
2. Except first node, all nodes are connected by BetweenFactor
3. The position node is connected to Landmark node with the help of BearingRangeFactor
4. So at each step, the node will be able to perceive a Aruco marker
