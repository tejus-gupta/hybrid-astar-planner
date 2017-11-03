### Hybrid A* Path Planner

This repository contains the code for a real-time path planner for non-holonomic vehicles using Hybrid-A* algorithm. The Hybrid-A* algorithm is described here - [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf).

This code is the result of a project at the Autonomous Ground Vehicle (AGV) research group at Indian Institute of Technology Kharagpur. We have have tested this code on Eklavya, our lab's test vehicle. We used GPS waypoints as target and the binary obstacle map was generated using LIDAR mounted in the front of the vehicle. We used ROS for sensor interfacing.

##### Algorithm Description
* A 3D discrete search space is used but unlike traditional A*, hybrid-A* associates with each grid cell a continuous 3D state of the vehicle. The resulting path is guaranteed to be drivable (standard A* can only produce piece-wise linear paths).
* The search algorithm is guided by two heuristics -
	* 'non-holonomic-without-obstacles' uses Dubin's path length ignoring obstacles
	* 'holonomic-with-obstacles' uses shortest path in 2D computed using dijkstra ignoring holonomic constraints of vehicle
* To improve search speed, the algorithm analytically expands nodes closer to goal using dubins path and checks it for collision with current obstacle map.

##### Current Maintainers
* [Tejus Gupta](https://github.com/tejus-gupta)
* [Rahul Kranti Kiran](https://github.com/KrantiKIran)

##### Images
<img src="https://imgur.com/wDC3stV.png" alt="Example1" width="400"/>             <img src="https://imgur.com/GZH6w0V.png" alt="Example2" width="400"/> 

##### Resources
* [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
* [Demo Video](https://www.youtube.com/watch?time_continue=2&v=qXZt-B7iUyw)
