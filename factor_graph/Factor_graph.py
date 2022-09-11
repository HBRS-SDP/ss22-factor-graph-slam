class Factor_graph:
    '''
    This class acts as the parent class for Factor class,
    Edge class and Inference class and it contains class variables
    which can be accessed from any instances of Factor, Edge and Inference
    classes
    '''
    no_child_edge=[]
    no_parent_edge=[]
    leaf_factors=[]
    edges =[]
    factors=[]

    def __init__(self):
        pass
