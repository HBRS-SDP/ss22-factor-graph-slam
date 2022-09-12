from Factor_graph import Factor_graph
import numpy as np

class Factor(Factor_graph):

    def __init__(self):
        '''
        self.factor_name : can be int or str but should be unique
        self.linked_edge : list of Edge class objects , it stores variables connected to the factor
        self.factor_weight : the weight by which the outgoing msg is multiplied
        self.fac_2_var_msg : message passed from factor to its outgoing variables
        self.income_var : incoming variables to the factor
        self.outgo_var : outgoing variables from the factor
        '''

        super().__init__()
        self.factor_name=None
        self.linked_edge=[]
        self.factor_weight=1
        self.fac_2_var_msg = 1
        self.income_var=[]
        self.outgo_var=[]
        Factor_graph.factors.append(self)

    def add_factor(self, name, weight):
        '''
        name: string or integer - should be unique
        weight: int or float - its the weight by which outgoing messages are multiplied
        '''
        self.factor_name=name
        self.factor_weight=weight

    def link_variables(self, variable_dict):

        '''
        variable_dict : dict --> {"incoming":[], "outgoing":[]}
        len(variable_dict["outgoing"]) or len(variable_dict["incoming"]) != 0
        if there is no incoming variables : variable_dict["incoming"] = [None]
        There should be atleast one outgoing variables
        '''

        for i in variable_dict["incoming"]:
            self.income_var.append(i)
            self.linked_edge.append(i)

        for i in variable_dict["outgoing"]:
            self.outgo_var.append(i)
            self.linked_edge.append(i)

        if(self.income_var[0]==None):
            Factor_graph.leaf_factors.append(self)

    def compute_factor_msg(self):

        '''
        compute the outgoing msg of the factor based on the incoming msgs
        '''

        fac_msg=0
        for i in range(0, len(self.linked_edge)):

            if(self.linked_edge[i] not in self.visited_edge):

                if(i==0 or fac_msg==0):
                    fac_msg =  np.copy(self.linked_edge[0].dist)

                else:
                    fac_msg = np.copy( np.outer(fac_msg, self.linked_edge[1].dist) )

                self.visited_edge.append(self.linked_edge[i])

        self.factor_msg = np.copy(fac_msg)
