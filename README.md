# CS3027 Robotics assessment

Use Robot Operating System (ROS) framework to achieve these goals:
* Visualise your robot in the world
* Write path planning algorithm of your choice for planning the path
* Enable robot localisation
* Make robot move trought the generated path
* Basic image processing

## Path planning

First of all obstacles get expanded, then using Euclidean heuristics points are sorted and paired by distance. After that A* search is started for each pair on different thread, then all threads are joined and single continuous path is generated.

When implemeting A*, I used pixel perfect navigation (1 unit is equal to 1 map pixel, more on that in the report). Also A* expands to all 8 directions.

<img src="/preview/path.gif" alt="Path planning"></img>


## Driving

There are 3 different driving modes:
* Normal driver - follows given path, from one waypoint to another
* Bug driver -  tries to avoid obstacle by following obstacle to right and after obstacle is behind changes mode back to driver
* Narrow driver - used when there are 2 close obstacles from both sides. It tries to correct the angle and drive past obstacles, after that normal driver takes responsibility.

Each of these modes alternate when required conditions are met.

<img src="/preview/drive.gif" alt="Driving"></img>
