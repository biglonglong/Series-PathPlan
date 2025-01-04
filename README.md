## Overview
This repository reproduces some  common path planning algorithms used in robotics, including search-based, sampling-based, group intelligence, reinforcement learning, and curves generator. Morever, it designed animations for each algorithm to demonstrate its operation.
referencing [zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning), but code was refactored.
- [Overview](#overview)
- [Search-based Planning](#search-based-planning)
  - [BFS \& DFS](#bfs--dfs)
  - [Best-First \& Dijkstra](#best-first--dijkstra)
  - [A\* and A\* Variants](#a-and-a-variants)
  - [D\* and D\* Variants](#d-and-d-variants)
  - [Hybrid A\* and other Importance](#hybrid-a-and-other-importance)
- [Sampling-based Planning](#sampling-based-planning)
  - [RRT and RRT Variants](#rrt-and-rrt-variants)
- [Group Intelligence Optimization](#group-intelligence-optimization)
  - [genetic](#genetic)
  - [ACO](#aco)
  - [particle swarm](#particle-swarm)
- [Reinforcement Learning](#reinforcement-learning)
- [Curves Generator](#curves-generator)
- [Reference](#reference)



## Search-based Planning

### BFS & DFS

<div align=right>
<table>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Bfs.py"><img src=".\map\gif\Bfs.gif" alt="BFS" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\Dfs.py"><img src=".\map\gif\Dfs.gif" alt="DFS" width="400"/></a></td>
  </tr>
</table>
</div>

### Best-First & Dijkstra

<div align=right>
<table>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Best_First.py"><img src=".\map\gif\Best_First.gif" alt="Best_First" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\Dijkstra.py"><img src=".\map\gif\Dijkstra.gif" alt="Dijkstra" width="400"/></a></td>
  </tr>
</table>
</div>

### A* and A* Variants

<div align=right>
<table>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Astar.py"><img src=".\map\gif\Astar.gif" alt="Astar" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\Bidirectional_Astar.py"><img src=".\map\gif\Bidirectional_Astar.gif" alt="Bidirectional_Astar" width="400"/></a></td>
  </tr>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Repeated_Astar.py"><img src=".\map\gif\Repeated_Astar.gif" alt="Repeated_Astar" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\ARAstar.py"><img src=".\map\gif\ARAstar.gif" alt="ARAstar" width="400"/></a></td>
  </tr>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\LRTAstar.py"><img src=".\map\gif\LRTAstar.gif" alt="LRTAstar" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\RTAAstar.py"><img src=".\map\gif\RTAAstar.gif" alt="RTAAstar" width="400"/></a></td>
  </tr>
</table>
</div>

###  D* and D* Variants

<div align=right>
<table>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Dstar.py"><img src=".\map\gif\Dstar.gif" alt="Dstar" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\LPAstar.py"><img src=".\map\gif\LPAstar.gif" alt="LPAstar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><a href=".\algorithm\Search-based Planning\Dstar_Lite.py"><img src=".\map\gif\Dstar_Lite.gif" alt="Dstar_Lite" width="400"/></a></td>
    <td><a href=".\algorithm\Search-based Planning\ADstar.py"><img src=".\map\gif\ADstar.gif" alt="ADstar" width="400"/></a></td>
  </tr>
</table>
</div>

### Hybrid A* and other Importance

…




## Sampling-based Planning

### RRT and RRT Variants

…



## Group Intelligence Optimization

### genetic

### ACO

### particle swarm

…




## Reinforcement Learning

…



## Curves Generator

…

