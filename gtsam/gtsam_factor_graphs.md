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

	Values is a single object containing labeled values for all of the variables. The type or organization of the variables can change.

* Factors

	A nonlinear factor expresses a constraint between variables, which in the SLAM example, is a measurement such as a visual reading on a landmark or odometry.


## Example elaboration


While creating a factor graph we always use a variable, a noise model, a factor graph container(NonlinearFactorGraph)

   * In general, creating a factor requires:
   *  - A key or set of keys labeling the variables that are acted upon
   *  - A measurement value
   *  - A measurement model with the correct dimensionality for the factor

## SimpleRotation: a simple example of optimizing a single rotation according to a single prior

- values.h -> is used to initilize a value for linearization function.(for iterative solvers). This solvers convert non-linear functions into linear function and then solve them until the variables converges to a consistent set of values

## CameraResectioning: resection camera from some known points

- Estimating the pinhole camera model parameters
- graph.emplace_shared -> Inserts a new element at the end of the vector, right after its current last element

## SFMExample: basic structure from motion

-	SFMdata.h
-	createPoints() -> gives set of ground truth values for 3D location of robot around a 10m cube  
-	createPoses() ->  gives circular pose around the cube

     *  wx Incremental roll (about X)
     *  wy Incremental pitch (about Y)
     *  wz Incremental yaw (about Z)

### easyPoint2KalmanFilter.cpp

ExtendedKalmanFilter 

classes inhertited 

-> NonlinearFactorGraph

-> NonlinearFactor


### List of optimisers used 
-	Levenberg marquard optmizer
-	Dog leg optimizer

### list of noise models used 
-	Diagonal 
-	isotropic 

### ImuFactorExample.py

#### sample data generation:

- Data generator - scenario generator {
	import gtsam
	gtsam.ScenarioRunner()
	}
- preintegrationexample --> [gtsam have header called Scenario.h]

- GTSAM have a scenario generator to test navigation examples {**Scenario.h**}

- There are two types of scenarios available in GTSAM:
	- constant twist scenario
	- Accelerating scenario

#### functions available in IMUfactor 

- add prior 
- optimize
- plot 
- run
