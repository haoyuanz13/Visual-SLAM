# Trajectory Generation for Quad rotor
The package is amied to design and generate an optimal polynomial smooth trajectory for the Quad rotor.

Polynomial Trajectory Design
----------------------------
Given a map environment (3D) with boundary information, start/goal positions and possible block position, our goal is to design a smooth polynomial trajectory without any collision.      
Here, we implement the __Dijkstra algorithm__ and __A-star__ to search for an optimal path from the start to the goal. Then according to some flag waypoints and velocity, acceleration constraints, we use __Convex optimization__ tools to design a polynomail smooth trajectory.


Trajectory Generation
----------------------
In the real Quad rotor, we use the __PID controller__ to control the flight vehicle given the designed trajectory. For the controller, we use either the __Back-Stepping assumption__ for linear control or __Geometric Nonlinear__ controller. 
