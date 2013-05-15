

# Intelligent Agents (04/03)
<!-- Try incorporating this into OSUAR nav code -->
Percept sequence: complete history of everything agent has ever perceived
Agent function (or policy): map per seq to actions.
Review properties of environments
4 types of agents (plus learning agent) -- names are good! Identify in our nav system, since we have all five.

# Uninformed Search (04/05, 08)
Review tree search.
Use heap for priority queue.
Iterative deepening combines best of BFS and DFS.
Avoid repeated states by referring to a closed list, implemented in a hash table.
Graph search implements some search algorithm to avoid loops.

# Informed Search (04/08, 10)
An admissible heuristic never overestimates cost to reach goal, so using it will get us optimal solution.
Consistent heuristics?
Some heuristics are better than others, and sometimes they dominate (i.e., fewer relaxations so the estimate is more accurate). Better heuristics can speed up search, provided it's easily calculable and is admissible.

# Local Search in Discrete State Space (04/12, 15)
Define neighborhood -- how do we do this for continuous state space? (will be addressed in next lecture)
Use larger neighborhood for better chance at finding a better maximum.
Four types:
  * Hill Climbing (Random-restart hill climbing is a good way to get out of local maxima.)
  * Simulated Annealing -- like hill climbing, but with random downhill moves with varying probability. Sounds like guesswork. Dislike.
  * Beam Search -- like hill climbing with k threads while sharing useful information between threads. (k is beam width.)
  * Genetic Algorithms.. is a big hack. (But maybe we can use this to better evaluate biological systems, e.g., at the DRL?)

# Local Search in Continuous State Space (04/15)
  * Gradient Descent -- find gradient and move that way. Learning rate 'a' can be either set to decay over time or use the second derivative.

# Adversarial Search (04/17)
  * Alpha-Beta Pruning: If certain branches in a tree have no hope of being "better" than others, prune and do not explore that branch. This allows us to look ahead in search tree significantly further in the same amount of time.

# Evaluation Functions (04/22)
  * If we can't fit game tree in memory, rank different game states in order of "goodness."
  * Weighted linear functions -- find weights with machine learning.
  * Assume features are independent (i.e., weights don't overcount).

# Propositional Logic (04/24, 26)
  * We need to represent knowledge and reason about it.
  * Entailment, i.e., if-and-only-if.
  * Algorithms for propositional logic take exponential time and space, but some algorithms are O(n) in practice.
  * Keep tautologies out of the knowledge base.
  * Resolution.   <!-- TODO: Figure this out. -->

# Bayesian Networks
  * Directed acyclic graph (i.e., cause and effect) of conditionally
    independent nodes (why acyclic?). This can be used to break up
    a probability of many factors into one with fewer factors.
  * D-separation (two special cases) used to determine conditional independence.

Technical detail: "cost" functions need to be minimized. "utility" functions need to be maximized.

Try GA on PID tuning.


<!--
vim: syntax=markdown
-->

