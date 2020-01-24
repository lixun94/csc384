'''
All models need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = kenken_csp_model(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the KenKen puzzle.

The grid-only models do not need to encode the cage constraints.

1. binary_ne_grid (worth 10/100 marks)
    - A model of a KenKen grid (without cage constraints) built using only 
      binary not-equal constraints for both the row and column constraints.

2. nary_ad_grid (worth 10/100 marks)
    - A model of a KenKen grid (without cage constraints) built using only n-ary 
      all-different constraints for both the row and column constraints. 

3. kenken_csp_model (worth 20/100 marks) 
    - A model built using your choice of (1) binary binary not-equal, or (2) 
      n-ary all-different constraints for the grid.
    - Together with KenKen cage constraints.

'''
from cspbase import *
import itertools
import operator

def binary_ne_grid(kenken_grid):
    # TODO! IMPLEMENT THIS!
    #print(kenken_grid)
    size = kenken_grid[0][0]
    domain = []
    for i in range(1, size + 1):
        domain.append(i)
    
    variables = []
    for j in range(1, size + 1):
        variables.append([])
        for k in range(1, size + 1):
            variables[j-1].append(Variable('V{}{}'.format(j, k), domain))

    cons = []
    for r in range(0, size):
        for c in range(0, size):
            for a in range(c + 1, size):
                con1 = Constraint("C(V{}{},V{}{})".format(r+1, c+1, r+1, a+1),\
                                 [variables[r][c], variables[r][a]])
                con2 = Constraint("C(V{}{},V{}{})".format(c+1, r+1, a+1, r+1),\
                                 [variables[c][r], variables[a][r]])
                sat_tuples = []
                for t in itertools.product(domain, domain):
                    if  t[0] != t[1]:
                        sat_tuples.append(t)
                con1.add_satisfying_tuples(sat_tuples)
                con2.add_satisfying_tuples(sat_tuples)
                cons.append(con1)
                cons.append(con2)
           
    vars_csp = []
    for i in range(0, size):
        for j in range(0, size):
            vars_csp.append(variables[i][j])
    csp = CSP("bne_grid_csp", vars_csp)
    for con in cons:
        csp.add_constraint(con)
    return csp, variables

def nary_ad_grid(kenken_grid):
    # TODO! IMPLEMENT THIS!
    size = kenken_grid[0][0]
    domain = []
    for i in range(1, size + 1):
        domain.append(i)
    
    variables = []
    for j in range(1, size + 1):
        variables.append([])
        for k in range(1, size + 1):
            variables[j-1].append(Variable('V{}{}'.format(j, k), domain))

    cons = []
    for r in range(0, size):
        con1 = Constraint("C(row{})".format(r+1),\
                         [variables[r][i] for i in range(0,size)])   
        col_var = []
        for c in range(0, size):    
            col_var.append(variables[c][r])
        con2 = Constraint("C(col{})".format(r+1),\
                         [col_var[i] for i in range(0, size)])              
        sat_tuples = []
        for t in itertools.permutations(domain, len(domain)):
            sat_tuples.append(t)
        con1.add_satisfying_tuples(sat_tuples)
        con2.add_satisfying_tuples(sat_tuples)
        cons.append(con1)
        cons.append(con2)        
    
    vars_csp = []
    for i in range(0, size):
        for j in range(0, size):
            vars_csp.append(variables[i][j])
    csp = CSP("nary_grid_csp", vars_csp)
    for con in cons:
        csp.add_constraint(con)
    
    return csp, variables  

def kenken_csp_model(kenken_grid):
    # TODO! IMPLEMENT THIS!
    ops = {0: operator.add, 1: operator.sub, 2: operator.truediv, 3: operator.mul}
    csp, board = binary_ne_grid(kenken_grid)
    #csp, board = binary_ne_grid(kenken_grid)
    cage_constraints = kenken_grid[1:]
    size = kenken_grid[0][0]
    domain = []
    for i in range(1, size + 1):
        domain.append(i)    

    for cage in cage_constraints:
        if len(cage) > 2:
            num_vars = len(cage) - 2
            result = cage[-2]
            op = cage[-1]
            vars_in_con = []
            new_con_name = "C("
            for var_pos in range(0,num_vars-1):
                pos_str = str(cage[var_pos])
                new_con_name += "V" + pos_str + ","
                
                vars_in_con.append(board[int(pos_str[0])-1][int(pos_str[1])-1])
            pos_str = str(cage[num_vars-1]) 
            
            new_con_name += "V" + pos_str + ")"
            vars_in_con.append(board[int(pos_str[0])-1][int(pos_str[1])-1])
            new_con = Constraint(new_con_name,\
                             [vars_in_con[i] for i in range(0,len(vars_in_con))])      

            sat_tuples = get_sat_tuples(result, op, domain,new_con.get_scope(),ops)
            new_con.add_satisfying_tuples(sat_tuples)
            csp.add_constraint(new_con)
        else: 
            pos_str = str(cage[0])
            var = board[int(pos_str[0])-1][int(pos_str[1])-1]
            var.dom = [cage[1]]
    return csp, board


# helper function
def get_sat_tuples(expected, operator, domain, con_scope, ops):
    full_d = []
    for i in range(len(con_scope)):
        full_d.append(domain)

    cart_prod = list(itertools.product(*full_d))
    sat_tuples = []
    
    if operator == 1 or operator == 2:
        for prod in cart_prod:         
            for val_tup in list(itertools.permutations(prod)):
                result = val_tup[0]
                for num in range(1, len(val_tup)):
                    result = ops[operator](result, val_tup[num])
                if result == expected:
                    sat_tuples.append(prod)         
             
    elif operator == 0 or operator == 3:
        for prod in cart_prod:
            result = prod[0]
            for num in prod[1:]:
                result = ops[operator](result, num)
            if result == expected:
                sat_tuples.append(prod)
                
    return sat_tuples
