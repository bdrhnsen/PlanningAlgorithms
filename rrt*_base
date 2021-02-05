import numpy as np

class GoalBiasedGreedySteerKNeighborhoodRRTStarBase:
    node_list=[]
    goalIsReached=False
    c_goal=(0,0)
    
    def __init__(self, seed):
        '''Constructor with seed, to be supplied to the np.random.RandomState object held inside. Feel free to add things.'''
        self.random = np.random.RandomState(seed)
    def distance(self, c1, c2):
        return np.sqrt(np.sum((c1-c2)**2))
def steer(self, c0, c, step_size):
        
        '''starting from the configuration c0, gets closer to
        *
        c in discrete steps of size step_size in terms of distance(c0, c). returns the last no-collision
        configuration. If no collisions are hit during steering, returns c2. If
        steering is not possible at all, returns None.'''
        #same with collision free for a while 
        #I find the index of the colliding discrete point last no-collision config is index-1
    
        dist=self.distance(self, c0, c)
        unit_vector=(c-c0)/dist
        counter=0
        discretization_constant=dist/step_size
        trajectory=[]
        for i in range(discretization_constant+1):
            temp=c0+unit_vector*i
            trajectory.append(temp)
        for k in range(len(trajectory)):
            if not (self.valid(self,trajectory[k])):
                counter=counter+1
                index=k
                
        if (counter == 0):
            return c
        if(counter==1):
            return trajectory[index-1]
        if(counter>1):
            return None

    def allclose(self,c1, c2):
        return np.allclose(c1,c2)

     def sample(self, p):
        if not self.is_goal_reachable() and self.random.random_sample() < p:
            return (1-p)
        else:
            return self.random.uniform(low=CFREE_XY_MIN, high=CFREE_XY_MAX, size=(2,))
    
    def neighborhood(self, c, k):
        '''returns a list of k closest nodes to configuration c in terms of distance(q.value, c)'''
        q_nearest=[]
        if c==self.c_goal:
            self.goalIsReached=True
        else:
            for i in range(len(self.node_list.shape[0])):
                if self.collision_free(self,self.node_list,c,step_size) and (self.distance(self.node_list[i][0],c)<=k):#checking if there is collision on the way and the radius k
                    dist=self.distance(self.node_list[i][0],c)
                    dist=[i,dist]
                    q_nearest.append(dist)       
        return q_nearest
    def init_rrt(self, c_init, c_goal):
        '''initializes/resets rrt with the root node holding c_init and goal configuration c_goal.'''
        self.node_list=[]
        self.node_list.append([c_init,0,0])#location cost and parent
        self.c_goal=c_goal#I tried to make this variable global
        pass
    def valid(self, c):
        for obs in CIRCLE_OBSTACLES:
            if np.sqrt((c[0]-obs[0])**2+(c[1]-obs[1])**2) < obs[2]:
                return False
    def collision_free(self, c1, c2, step_size):
        '''r*eturns True if the linear trajectory between c1 and c2 are collision free.'''
        #first I discritized the continous path than checked if all discrete point are non colliding 
        # I also found the unit vector in direction c1toc2 and found the discrete points according to that vector
        dist=self.distance(self, c1, c2)
        unit_vector=(c1-c2)/dist
        counter=0
        discretization_constant=dist/step_size
        trajectory=[]
        for i in range(discretization_constant+1):
            temp=c1+unit_vector*i
            trajectory.append(temp)
        for k in range(len(trajectory)):
            if not (self.valid(self,trajectory[k])):
                counter=counter+1
        if (counter == 0):
            return True
        else:
            return False
    def add_node(self, p, k, step_size):
        '''adds a node to the rrt with goal bias probability p, near function with k closest neighbors,
        and step_size for greedy steering. returns the new configuration that is now part of the tree.'''
        #if goal is not reached first sample c then find neighbors, than if there are neighbors for all neighbours
        #I sorted them according to the new node to have minimum cost
        
        if(not self.goalIsReached):    
            c=self.sample(p)
            q_nearest=self.neighborhood(c,k)
            candidate_q=[]
            if len(q_nearest.shape[0])>0:
                for k in range(len(q_nearest.shape[0])):
                    candidate_q.append(q_nearest[k][0],self.node_list[(q_nearest[k][0])][1]+q_nearest[k][1])#candidate q involves the index of the possible parent nodes of the new node and the corresponding cost if that parent is chosen
                sorted_q=candidate_q.sort(key=lambda x:x[1])#sorted_q is the sorted form of possible parent nodes for new node according to their costs.
                parent=sorted_q[0][0]
                
                new_config_unit_vector=(c-self.node_list[parent][0])/sorted_q[0][1] #finding the unit vector in the direction of parent to new node
                c_new=self.node_list[parent][0]+new_config_unit_vector*step_size #adding the amount step size to the parent node location in the unit vector direction
                organized_q=[c_new,sorted_q[0][1]+self.node_list[parent][1],parent]
            #sorted_q[0] will have the closest node sorted_q[0]={c cost parent}
                self.node_list.append(organized_q)
                return c_new
            else:
                return None
    def get_path_to_goal(self):
        #I know c_goal and parent of c_goal and if I know the parent of c_goal I also know the parent of that node this is the approach i used
        '''returns the path to goal node as a list of tuples of configurations[(c_init, c1),(c1, c2),...,(cn,c_goal)].
        If the goal is not reachable yet, returns None.'''
        if(not self.goalIsReached):
            return None
        else:
            path=[]
            self.c_goal=self.node_list[-1][0]
            c_goal_parent=self.node_list[-1][2]
            old_c=self.c_goal
            while (1):
                new_c=self.node_list[c_goal_parent][0]
                parent=self.node_list[c_goal_parent][2]
                abc=(new_c,old_c)
                path.append(abc)
                old_c=new_c                
                c_goal_parrent=parent
                if parent==0:
                    break
            path=path.reverse()
            return path
        
    def is_goal_reachable(self):
        '''returns True if goal configuration is reachable.'''
        if (self.goalIsReached):
            return True


    def simplify_path(self, path, step_size):
        #rewiring
        #let me explain the function with and example
        #lets say i equal to 3 and k equal to 5
        #first i check the distance between 3.node and 5.node then i compare it with 3to4 and 4to5 added if it is smaller
        #choose=true and I check that on the road between 3and5 if there is an obstacle or not
        #if the road is obstacle free i choose the 5.node to be right after 3 and 4 is not part of path anymore
        
        #if the road is blocked or distance is not smaller i simply add the original child of ith node to the path
        '''greedily removes redundant edges from a configuration path represented as a list of tuples
        of configurations [(c_init,c1),(c1,c2),(c2,c3)...(cn,c_goal)], as described
        Principles of Robot Motion, Theory, Algorithms and Implementations (2005), p. 213,
        Figure 7.7.(use the default version, always try to connect to c_goal, not the other way around'''
        for i in range(len(self.node_list.shape[0])):
            for k in range (i+2,len(self.node_list.shape[0])):
                simplified_path=[]
                
                temp=0
                i_to_k=self.distance(self, self.node_list[i][0], self.node_list[k][0])
                thereIsAnObstacle=self.collision_free(self, self.node_list[i][0],self.node_list[k][0],step_size)
                for p in range(i,k):
                    p_to_i=self.distance(self, self.node_list[p][0], self.node_list[i][0])
                    temp=temp+p_to_i
                if(i_to_k)<temp:
                    choose=True
                    if((choose==True) and (not thereIsAnObstacle)): 
                        simplified_path.append(self.node_list[i][0],self.node_list[k][0])
                    else:
                        #path[i] = [ci ci+1] 
                        simplified_path.append(self.path[i][0],self.path[i][1])
        return simplified_path


    def get_all_edges(self):
        '''returns all of the edges in the tree as a list of tuples of configurations [(c1,c2),(c3,c4)...]. The
        order of the edges, The order of edges in the list and their direction is not important.'''
        edges=[]
        k=0
        for i in range(len(self.node_list.shape[0])):
            c_i=self.node_list[k][0]
            c_ii=self.node_list[k+1][0]
            edge=(c_i,c_ii)
            edges.append(edge)
            k=k+2
        return edges

    def get_goal_cost(self):
        '''returns the non-simplified goal path length in terms of distance. Returns np.Inf if goal is not reachable.'''
        # I hold information of cost in the array so it is already known
        if(self.is_goal_reachable()):
            cost=self.node_list[-1][1]
            return cost
        
        else:
            return np.Inf
        
