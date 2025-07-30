# Problem 2 - with numeric fluents

A version of problem 2 developed using `:numeric-fluents`. It's very compact and adds very little modifications to the `p1` instance, but due to limited support of such fluents in planners the PDDL1.2 the "simpler" version was preferred. Other than weaker generizability across planners, it was also discarder due to poor extendability to HDDL modelling in `p3` instance.

You can run such version with **Expressive Numeric Heuristic Search Planner (ENHSP)** which is present in planutils and support such fluents 


```
planutils run enhsp -- -o domain.pddl -f p_numeric.pddl -sp p_numeric.plan -s WAStar -wh 0.25
```