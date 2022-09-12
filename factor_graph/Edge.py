from Factor_graph import Factor_graph
import numpy as np

class Edge(Factor_graph):

    def __init__(self, name):
        '''
        name : variable name --> can be int or string but unique
        self.dist : distribution of the variable, it will be numpy array
        self.parent_factor : parent factor to which the variable is attached
        self.child_factor : child factor to which the variable is attached
        self.var_2_fac_msg : message passed from this variable to factor
        '''

        super().__init__()
        self.variable_name=name
        self.dist =None
        self.parent_factor=None
        self.child_factor=None
        self.var_2_fac_msg=1
        Factor_graph.edges.append(self)

    def add_edge(self, parent=None, child=None):
        '''
        parent : factor class object or none
        child : factor class object or none
        '''

        self.parent_factor = parent
        self.child_factor = child

        if(parent == None):
            Factor_graph.no_parent_edge.append(self)

        if(child == None):
            Factor_graph.no_child_edge.append(self)

    def add_distribution(self, distribution):
        '''
        distribution : numpy array
        '''

        self.dist = distribution
        self.var_2_fac_msg = np.copy(distribution)
