# Factor Graphs

Factor graphs are one of the probabilistic graphical models, well suited for inferencing complex **estimation/optimization** problems like Simultaneous Localization and Mapping (SLAM) or Structure from Motion (SFM). 

Formally, Factor graphs is a bi-partite graph with two nodes, 
 

*   Variable nodes
*   Factor nodes

The variable nodes represent unknown quantities in the problem, and the factor nodes define the relationships between variables in the graph. Edges in the factor graph connects variable nodes to factor nodes, which ensures each factor is a function of only the variables which are adjacent (or neighbors) to that particular factor. 

Note: A factor node can be connected to multiple variable nodes and vice versa but not the same (i.e) factor-factor or variable-variable relations never exists. Factor graph connects nodes of different type, thus the name bi-partite graph.

# Mathematical definition

Formally, a factor graph is a bipartite graph F = (U, V, E) with two
types of nodes: 

*   factors ($φ_{i}$ ∈ U)
*   variables ($x_{j}$ ∈ V)

Edges $e_{ij}$ ∈ E are always between factor nodes and variables nodes. The set of variable
nodes adjacent to a factor $φ_{i}$ is written in N $(φ_{i})$, and we write $X_{i}$
for an assignment to this set. With these definitions, a factor graph F
defines the **factorization of a global function φ(X)** as,
\begin{equation}
ϕ(X) = ∏\limits_{i} φ_{i}.(X_{i})
\end{equation}

where $\prod$ represents the Joint probability distribution.

In other words, the independence relationships are encoded by the edges
$e_{ij}$ of the factor graph, with each factor $φ_{i}$ a function of only the variables $X_{i}$ in its adjacency set N ($φ_{i}$).


**What does factorization of a global function means?**

Consider a global function g(X), from the definition of factor graph it states, using factor graphs we can redefine the same global function g(X) as a product of smaller functions f(x). (i.e) 
$$g(x_{1}...,x_{n}) = ∏\limits_{i} f_{i}.(S_{i}), where \space S_{i} ⊆ (x_{1}, x_{2},..x_{n})$$ 
Here, $f_{i}.(S_{i})$ are functions depends on the subset of variables what g(X) depends on. For example, g(X) = g($x_{1}, x_{2},...x_{n}$), f(x) can be f($x_{1}, x_{3}, x_{5})$. 


# Representing Factor graphs:

**In a more probabilistic way, we can say, g(X) is a joint probability distribution of all the variables, and f(x) is a conditional probability distribution of the subset of variables.** Any complex estimation or inference problems can be modelled and represented into factor graphs. Two different representations are,

*   Original Factor Graphs
*   Forney-style Factor Graphs (FFG)

In conventional (or) original factor graphs, variable nodes are represented as circles and factor nodes are represented as square boxes. A simple factor graph representation is shown below,

<center>
    <p align="center">
  <img src="Images and plots/pictures/readme_pictures/simple_example_original_fg.png" />

</p>
<caption><b> Simple (original)factor graph representation </b></caption>

Here, $x_{1}, x_{2}, x_{3}$ are variable nodes and $f_{1}, f_{2}, f_{3}$ are factor nodes 

As we dicussed earlier, Factor graph defines the factorization of a global function (i.e) it breaks down a larger global function into product of smaller functions. Above factor graph is defined as,
$$f(x_{1}, x_{2}, x_{3}) = f_{1}(x_{1}).f_{2}(x_{2},x_{1}).f_{3}(x_{3},x_{2})$$

# Some examples

Some examples for converting graphs into (original)factor graphs,

<center>

![](../Images%20and%20plots/pictures/readme_pictures/original_FG.png)

</center>

<center>

![](../Images%20and%20plots/pictures/readme_pictures/original_example_with_equation.png)

</center>

# Forney-style Factor Graph (FFG)

Forney-style Factor Graphs are undirected graphs in which variable are represented as edges and factors associated with them are represented as nodes (shown in Figure 1).
Basic assumptions to model FFG:


*   A node for each factor
*   An edge or half-edge for each variable
*   Node is connected to edge if and only if the variable is in the subset of that factor 
* **No variable should appear in more than two factors**

Forney-factor graph of the above figure can be defined as, 
<center>

![Fig 1: Simple FFG representation](../Images%20and%20plots/pictures/readme_pictures/simple_example_ffg.png)

<caption><b> Simple FFG representation</b></caption>

</center>

$$f(x_{1}, x_{2}, x_{3}) = f_{1}(x_{1}).f_{2}(x_{2},x_{1}).f_{3}(x_{3},x_{2})$$

# Equality Nodes

**What if some variables appears in more than two factors?**

- The equality node resolves this situation by constraining the information about three variables to be equal. Edges constrained by equality nodes can then effectively be regarded as a single variable that is shared among connected factors. It eliminates the use of same variable in more than two factors by introducing auxilary variables.


- Equality node is merely an branching points that allow more than two factors to share some variable. Consider the factorization, 

$$f(W, U, X, Y, Z) = f_{1}(W).f_{2}(U).f_{3}(W,U,X).f_{4}(X,Y).f_{5}(X,Z)$$ 
where W, U, X, Y, Z are variables and $f_{1}..,f_{5}$ are factors.


- Here, X appears to be in three different factors. Thus, auxilary variables X' and X'' are introduced which doesn't modify neither the values of variables nor the factors.

- With the introduction of equality node, FFG can be represented as,

$$f(W, U, X, X', X'', Y, Z) = f_{1}(W).f_{2}(U).f_{3}(W,U,X).f_{4}(X',Y).f_{5}(X'',Z).f_{=}(X,X',X'')$$

where $$f_{=}(X,X',X") = δ(X-X').δ(X-X'')$$ 

What it denotes is that, $$X=X'=X''$$

**NOTE:**  $ \space δ - $ Dirac delta, where δ(0) = 1 

- Variable X needs to be connected with three factors ($f_{3}, f_{4}, f_{5}$) but the problem is that it's not possible to draw edge connecting X with those factor nodes. Thus, an equality node which doesn't modify any assignments to the variables is introduced. The equality node provides a branching point for X to connect it with factors $f_{4}$ and $f_{5}$.

<center>

![equality_node_ffg.png](../Images%20and%20plots/pictures/readme_pictures/equality_node_ffg.png)
<caption><b>FFG with equality nodes</b></caption>

</center>

## Some examples for FFG

$$ g(X_1,X_2,X_3) = f_1(X_1) f_2(X_1,X_2) f_3(X_1,X_2) f_4(X_2,X_3) $$

<center>

![Example1](../Images%20and%20plots/pictures/readme_pictures/ffg_notebook_example1.png)

</center>

<center>

![Example2](../Images%20and%20plots/pictures/readme_pictures/ffg_notebook_example2.png)

</center>

Factorization equation for the above FFG, 

$$g(x_1,x_2,x'_2,x''_2,x_3,x_4)=fa(x_1,x_2)⋅fb(x'_2,x_3)⋅fc(x''_2,x_4)⋅f=(x_2,x′_2,x''_2)$$


## Exercise 2

**For the given factor graph equation, draw an forney-factor graph for it.**

$$g(x_1,x_2..,x_7) = f_1(x_1).f_2(x_2).f_3(x_1,x_2,x_3).f_4(x_4).f_5(x_3,x_4,x_5).f_6(x_5,x_6,x_7).f_7(x_7)$$

## **YOUR ANSWER HERE**

# Exercise 3

Now, your task is to model a forney-style factor graph for the given SLAM toy example problem (feel free to use any online drawing tool). To make the task easier, the original factor graph version is represented below. 

<center>
    
![Fig 4: A simple toy example for SLAM (Left) and its original factor graph version (Right)](../Images%20and%20plots/pictures/readme_pictures/slam.png)
<caption> <b>A simple toy example for SLAM (Left) and its original factor graph version (Right) </b></caption>

</center>

## FFG version of the given graph

**Input the FFG version of the above graph.**

## **YOUR ANSWER HERE**

# Inferencing

Inferencing is the process of finding the most probable state of the system. In the above example, the most probable state of the system is the location of the robot in the environment. Some inferencing algorithms for factor graphs are,

*  Sum-product or Belief propagation
*  Non-linear optimisation

- Inferencing in Forney-factor graphs can be done by message-passing between nodes along the edges. The most used and simple message-passing algorithm in factor graph is the **sum-product algorithm** otherwise called **belief-propagation**.


- The purpose of this algorithm is to compute marginals over a global function which is defined as a conditional joint probability mass function of given discrete variables. The **messages** are nothing but either a real value or a real valued functions (single value or distribution).


- To make the computation faster (iterations and memory), forney factor graphs are converted to tree-structure graphs and messages are passed from leaf nodes to the root node(the node which we are calculating marginal for) along the edges (i.e Variables).

## Sum-Product algorithm

- The principle behind the algorithm is to find to marginal of the desired variable. In order to compute marginals in a graph, we start with incoming messages at the terminals and half-edges, and proceed until each edge has both forward and backward messages.


- The sum-product theorem states that, if the graph is a tree, then multiplication of the forward and backward messages on an edge yields the exact marginal for the corresponding variable.

## Message passing in terminal nodes


(a) If the leaf or terminal node is a factor node alone and it has no enclosed variables, then the message out from that terminal node is the factor itself 


(b) If the leaf or terminal node has an half-edge (i.e variable alone) then the message from that terminal node 1 

<center>
    
![terminal%20condition.png](../Images%20and%20plots/pictures/readme_pictures/terminal%20condition.png)
<caption><b>Terminal conditions in FFG</b></caption>

</center>

(c) $\textbf{Message update rule:}$ For a node, outgoing message is calculated by product of all incoming messages followed by marginalizing it with respect to the variables in factor functions (except the outgoing variable). 

<center>
    
![./message%20update%20rule.png](../Images%20and%20plots/pictures/readme_pictures/message%20update%20rule.png)    
<caption><b> Sum-product update rule </b></caption>

</center>

Message update rule for the given FFG is given by,

$$\mu_{Y}(y) = \sum_{x_1,...x_n} \mu_{X_1}(x_1)...\mu_{X_n}(x_n). f(y,x_1..,x_n)$$
 
# Message passing in simple FFG

<center>

![./message_passing.png](../Images%20and%20plots/pictures/readme_pictures/message_passing.png)
<caption><b>Illustration of messsage passing in FFG</b></caption>

</center>

**NOTE:** FFG graphs are undirected graphs in general. In the above diagram, directed graphs are given for a better understanding of the underlying tree structure.

## Overview

- We need to find the marginal of $X_4$, so we will consider that variable as the root node for our tree structured graph and we complete the tree until there is a treminal node or leaf graph. 

- We use depth-first-search for traversing the tree and we will use the path for calculating the forward and backward messages to the root node.


## How to calculate forward and backward messages in our tree-structured FFG?

- In our tree-structured FFG, we always have variable as the root node. From FFG definition, we know that a variable should appear in only two factor nodes which means the root node will always have two branches in its first level. We should program in a way to find the forward and backward message paths and implement sum-product algorithm on all the nodes.

## Sum-product for given FFG (simple explanation w.r.t. the above FFG)

- In the given FFG, $\mu_{Xn}(X_n)$ are messages and the arrows represents direction of flow of messages. By the definition of sum-product algorithm, multiplying forward and backward messages to a node gives the marginal valueof that node.


- Message coming out of red box -> $\mu1_{X4}(X_4)$ is the overall forward message to the desired node $X_4$ ( which we are calculating marginal) and message coming out from blue box -> $\mu2_{X4}(X_4)$ is the backward message to the desired node $X_4$


- The forward message is computed by multiplying all factors inside the red box and marginalizing them with the variables which are included in their factor functions (except the variable which we are sending message to). Backward messages are computed in the same way too.


- Then, product of both these messages give the marginal of the desired variable

**What the forward and backward messages represents?**


Forward and backward messages represents the probability distribution or belief distribution of the variable (which it sending message to, in our case $X_4$) with respect to the enclosed factors or variables

## How sum-product works?

- Forney representation of the given graph can be represented as


$$g(X_1, X_2..,X_7) = f_1(X_1). f_2(X_2). f_3(X_3). f_4(X_1,X_2,X_3,X_4). f_5(X_4,X_5,X_6,X_7). f_6(X_5). f_7(X_6,X_8). f_8(X_7)$$


- Forney factor graph representation of probabilistic model helps to automate probabilistic inference tasks. For finding $X_4$, we need to marginalize the other variables as


$$f(X_4) = \sum_{X_1,X_2,X_3,X_5,X_6,X_7,X_8} f_1(X_1). f_2(X_2). f_3(X_3). f_4(X_1,X_2,X_3,X_4). f_5(X_4,X_5,X_6,X_7). f_6(X_5). f_7(X_6,X_8). f_8(X_7) $$


- By distributive law, above sum of products are converted to product of sums,


$$f(X_4) = \sum_{X_1,X_2,X_3}f_1(X_1). f_2(X_2). f_3(X_3). f_4(X_1,X_2,X_3,X_4).\sum_{X_5,X_6,X_7,X_8} f_5(X_4,X_5,X_6,X_7). f_6(X_5). f_7(X_6,X_8). f_8(X_7)$$


- Applying sum-product update rule, we can compute forward and backward messages to the variable,


$$ \mu1_{X4}(X_4) = \sum_{X_1,X_2,X_3}f_1(X_1). f_2(X_2). f_3(X_3). f_4(X_1,X_2,X_3,X_4),$$


$$ \mu2_{X4}(X_4) = \sum_{X_5,X_6,X_7,X_8} f_5(X_4,X_5,X_6,X_7). f_6(X_5). f_7(X_6,X_8). f_8(X_7) $$


- To compute marginal, multiply both forward and backward messages

$$f(X_4) = \mu1_{X4}(X_4). \mu2_{X4}(X_4)$$


# Factor graph with SLAM

SLAM problem solves two parts, **Localisation** (knowing where you are in the environmnet) and **Mapping** (How the environment looks). Factor graphs are one of the ways to solve the SLAM problem by modelling the unknown locations as nodes and their relationship between varibales or with environemnt objects (landmarks) as factors. Consider the simple SLAM problem below, 

<center>
    
![SLAM](../Images%20and%20plots/pictures/readme_pictures/SLAM.png)   
    
<caption><b>Simple SLAM problem modelled as factor graph</b></caption>

</center>

X1, X2 and X3 are nodes which are unknown locations 
L1 and L2 are the location of landmarks (which will be extracted from the environment using sensors)
f1, f2, f3 are factors which defines the relationship between nodes and landmarks

**What does the relationship between nodes and landmarks mean?**

- For example, Robot is at location X1, it can see the landmarks L1 and L2 (through a Camera fixed at robot frame). Thus, it can estimate the distance between X1 and L1 and X2 and L2 by using sensors (LIDAR). This is one of the parameter that defines the relationship (factors) between nodes and landmarks.


- Other parameters are orientation and pose of the landmarks (which can be estimated using IMU), distance between nodes (which can be estimated using odometry sensors)., etc.



**How it works?**

Consider the robot is at position X1 at a timestep t, and the camera of the robot could see the landmarks L1 and L2. 

- With this, we know the pose and orientation of the robot **with respect to the landmarks** L1 and L2 at this timestep. A factor node relating X1 and L1 and L2 is constructed. Now at timestep t+1, the robot displaced to a location X1 + $\Delta$ x (which is unknown initially).


- With the imu readings (acceleration and gyroscope) for this timestep, the robot knows that it has moved some distance with some acceleration. With the acceleration data, we can infer the displacement of robot from the previous timestep and also its pose and orientation. In addition to this, it constructs a 3D-map and updates it for every iteration. 


- In this way, variable nodes defining the location of robot and factor nodes defining relation between nodes and between variables-landmarks are constructed for each timestep along with the 3D-map construction. 


- With all these nodes (variable node) and relations (factor nodes), the robot keeps track and infers where it was and where it needs to be in the environment? (localisation) by inferring what was around it? (mapping). This inferencing procedure can be used for any optimization or estimation problems in robotics like loop closure, optimal path finder etc..,

**Why we are measuring the pose and orientation with respect to landmarks?**

Landmarks in a typical SLAM problems are stationary and unique because we need to localise our robot in an unknown environment. Landmarks are the only thing that are known to robot to find where exactly it is in the environment. **need to add something to make it clear or else remove it**

