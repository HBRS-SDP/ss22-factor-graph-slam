# Factor graphs 
old well known algorithms can be represented as factor graphs
help in visualization of a problem 

## Bipartite graph 

This graph contains 2 nodes 
in case of factor graphs we have factor node and variable node
the edges are always connected between different nodes 

* FactorGraph

	A factor graph contains a set of variables to solve for (i.e., robot poses, landmark poses, etc.) and a set of constraints between these variables, which make up factors.

* Values:

	Values is a single object containing labeled values for all of the variables.  Currently, all variables are labeled with strings, but the type or organization of the variables can change.

* Factors

	A nonlinear factor expresses a constraint between variables, which in the SLAM example, is a measurement such as a visual reading on a landmark or odometry.



## Example elobration

easyPoint2KalmanFilter.cpp

ExtendedKalmanFilter 

classes inhertited 

-> NonlinearFactorGraph

-> NonlinearFactor