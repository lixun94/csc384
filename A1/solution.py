#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    count = 0
    center = int((state.width-1)/2)
    
    
    for robot in state.xanadus:
        test_center = list(robot)
        if (test_center[1] != center or test_center[0] != center):
            count += 1
    return count




def heur_manhattan_distance(state):
#OPTIONAL
    '''Manhattan distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses Manhattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the Manhattan distances between each xanadu and the escape hatch.
    
    sum = 0
    center = int((state.width-1)/2)
    
    for i in state.xanadus:
        sum+=(abs(i[0]-center) + abs(i[1]-center))
    return sum
        






def heur_L_distance(state):
    #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses mahnattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the L distances between each xanadu and the escape hatch.
    
    L = 0
    center = int((state.width-1)/2)
    
    for i in state.xanadus:
        if (i[1] != center and i[0] != center):
            L += 2
        elif(i[1] == center and i[0] == center):
            L += 0
        else:
            L+=1
        
    return L






def heur_alternate(state):
#IMPLEMENT
    '''a better lunar lockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    L = 0
    
    center = int((state.width-1)/2)
    
    for i in state.xanadus:
        
        # check if in same axis, just one step
        if ((i[1] != center and i[0] == center)or (i[0] != center and i[1] == center)):
            L+=1
            
        # need one robot to help to make L move  L += 2
        elif(robots_help(state, i)):
            L += 2
            
        # check if is in center
        elif(i[1] == center and i[0] == center):
            L += 0
        # need two robots to make turn     
        elif(d_robot(state, center, i)):
            L+=3

        else:
            L += 4
        
    return L

# 
def d_robot(state,center, xanadus):
    '''
    check if there is a robots can help xanadus to get into center
    '''
    center = int((state.width-1)/2)
    
    c= 0
    for r in state.robots:
        if xanadus[1] == r[1]:
            if xanadus[0] < r[0]:
                for j in state.robots:
                    if j[0] + 1 == r[0] and j[1] == center:
                        return True
            else:
                for j in state.robots:
                    if j[0] - 1 == r[0] and j[1] == center:
                        return True
        elif xanadus[0] == r[0]:
            if xanadus[1] < r[1]:
                for j in state.robots:
                    if j[1] + 1 == r[1] and j[0] == center:
                        return True
            else:
                for j in state.robots:
                    if j[1] - 1 == r[1] and j[0] == center:
                        return True
    return False


def robots_help(state,xanadus):
    '''
    check if there is a robots can help xanadus to get into center
    '''
    center = int((state.width-1)/2)
    
    c= 0
    for r in state.robots:
        if r[0] == xanadus[0] and (r[1] == center + 1 or r[1] == center -1):
            c += 1
        elif r[1] == xanadus[1]and (r[0] == center + 1 or r[0] == center -1):
            c += 1
            
    if c > 1:
        return True
    return False
    
    
    

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a LunarLockoutState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    
    return sN.gval + (weight * sN.hval)




def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 2):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    
    result = False
    
    
    # wrapped f-value function
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    
    # records time
    start_time = os.times()[0]
    end_time = start_time + timebound
    
    
    
    # search engine is initialized with custom
    search_engine = SearchEngine('custom', 'full')
    search_engine.init_search(initial_state, lockout_goal_state, heur_fn, wrapped_fval_function)
    
    #set initial cost bound
    cost_bound = (float("inf"), float("inf"), float("inf"))
    search_result = search_engine.search(timebound)
    
    # if still time then keep search
    while start_time < end_time:
        if search_result == False:
            return result
      
        # update time/time left
        timebound = timebound - (os.times()[0] - start_time)
        start_time = os.times()[0]
      
        # if better path found then update cost bound and keep search
        if search_result.gval <= cost_bound[0]:
            cost_bound = (search_result.gval, search_result.gval, search_result.gval)
            result = search_result
        
        search_result = search_engine.search(timebound, cost_bound)
    
    return result    




def anytime_gbfs(initial_state, heur_fn, timebound = 2):
#OPTIONAL
    '''Provides an implementation of anytime greedy best-first search.  This iteratively uses greedy best first search,'''
    '''At each iteration, however, a cost bound is enforced.  At each iteration the cost of the current "best" solution'''
    '''is used to set the cost bound for the next iteration.  Only paths within the cost bound are considered at each iteration.'''
    '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    
    return 0
    
    

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3),)),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******") 
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_alternate)
    final = se.search(timebound) 

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime Weighted A-star")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    weight = 4
    final = anytime_weighted_astar(s0, heur_alternate, weight, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime GBFS")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    final = anytime_gbfs(s0, heur_alternate, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************")   



  

