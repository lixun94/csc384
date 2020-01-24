'''
This file will contain different variable ordering heuristics to be used within
bt_search.

1. ord_dh(csp)
    - Takes in a CSP object (csp).
    - Returns the next Variable to be assigned as per the DH heuristic.
2. ord_mrv(csp)
    - Takes in a CSP object (csp).
    - Returns the next Variable to be assigned as per the MRV heuristic.
3. val_lcv(csp, var)
    - Takes in a CSP object (csp), and a Variable object (var)
    - Returns a list of all of var's potential values, ordered from best value 
      choice to worst value choice according to the LCV heuristic.

The heuristics can use the csp argument (CSP object) to get access to the 
variables and constraints of the problem. The assigned variables and values can 
be accessed via methods.
'''
from cspbase import Variable
import random
from copy import deepcopy



def ord_mrv(csp):
    # TODO! IMPLEMENT THIS!
    all_vars = csp.get_all_unasgn_vars()
    min_curdom_size = 10 ** 10
    min_var = Variable(None, [])
    for var in all_vars:
        if var.cur_domain_size() < min_curdom_size:
            min_curdom_size = var.cur_domain_size()
            min_var = var
    return min_var

def val_lcv(csp, var):
    # TODO! IMPLEMENT THIS!
    cons_with_var = csp.get_cons_with_var(var)
    value_to_prune = {}
    curdom = var.cur_domain()
    pruned = []
    for d in curdom:
        value_to_prune[d] = 0
        var.assign(d)

        if cons_with_var:
            for con in cons_with_var:
                if con.get_n_unasgn() == 1:
                    unasgn_var = con.get_unasgn_vars()[0]
                    unasgn_dom = unasgn_var.cur_domain()
                    for unasgn_d in unasgn_dom:
                        if not con.has_support(unasgn_var, unasgn_d):
                            if (unasgn_var, unasgn_d) not in pruned:
                                unasgn_var.prune_value(unasgn_d)
                                pruned.append((unasgn_var, unasgn_d))
                                value_to_prune[d] += 1
        for (v, val) in pruned:
            v.unprune_value(val)
        var.unassign()
    s = [k for k in sorted(value_to_prune, key=value_to_prune.get, reverse=False)]
    return s
