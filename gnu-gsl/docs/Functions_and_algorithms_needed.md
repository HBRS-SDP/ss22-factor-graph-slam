# Functions and algorithms to be implemented

## Functions
### 1) **Factor_graph()** <br>
  - To create a custom graph data structure resembling the factor graph with nodes as variables and edges with factors (i.e) **G(V,E)**
    #### Sub-functions
    - Adding a new variable to the created graph
      - If the node to be added is a variable node, it only takes/should have parameters such as variable name, probability value (in some cases)
      - If its a factor node, then it should have factor name and factor value

    - Updating and removing the variable or factor nodes
    - Checking the status or value of a variable in the created graph

### 2) **Factor()**
  - Able to calculate joint and marginal probabilities between one or more variables and factor nodes
    #### Sub-functions
    - Instances of factor (class) should contain array of variables, their probability values (array)
    - Factor product - Function to get the product of multiple nodes (joint probability calculation)
    - Factor marginalization - finding marginal probabilities over one or more variables
    - Factor reduction

## Algorithm

### Sum-product algorithm

- For message passing over variable and factor nodes, we use sum-product algorithm
- Two kinds of message passing is done, (i.e) 
  - messages from variable node to factor node
  - messages from factor node to variable node
 ### 1) message passing from variable to factor node
  - Product of all the incoming messages to that node and send the resulting value as a message to the factor node connected to it

 ### 2) message passing from factor node to variable node
 - Product of all the messages to that factor node and marginalize over the other variables (except the node that is sending the message)
 - Multiply the resulting marginalized value with the factor value of that node and pass this resulting value as the message from that factor node to the connected variable node
