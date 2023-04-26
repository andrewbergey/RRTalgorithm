# RRT Navigation Project Report
The files attached are labeled NavMain, NavBase, NavRobotV2, and NavRRT. Using an RRT based algorithm, these files were coded to output an RRT, final path, and a robot following this path onto a 200 x 200 pixel window representing a map with walls and with each pixel representing a centimeter (cm). 

NavBase is where the base functions, map, and input arrays were created. The Coords function converts all of the coordinates that are inputted into the pygame coordinate system, which has the origin in the top left corner of the map, whereas the desired coordinate will be in reference to an origin in the bottom left corner of the map. The wall start and end points were then created and placed in an array to later be drawn in NavMain. All of the input files were iterated over and placed into arrays to later be referenced in NavMain for different parts of the project. The robotIntersection function takes a circle's (or robot's) radius, center point, and a line start and end point as parameters. With these parameters it determines if a robot will intersect with a given line segment. Then using robotIntersection, the collision function takes a center point parameter to see if a robot will intersect with any of the walls previously created for our map.
	
NavRobot contains a robot class and the distanceTraveled function. The robot class contains the robot's current position, orientation, linear velocity, and angular velocity. Two class functions, move and draw, allow the robot to be moved over a time interval that is input into the move function, and drawn at a specific coordinate and angle using pygame's "blit" method. The distanceTraveled function takes an input from the provided "input2" file and determines the robot's final destination and how far it has moved during a 0.2 second time interval.
	
NavRRT contains the functions: rho, sigma, freePoint, visitEqn, nearestNeighbor, new_state, extend, and pathV7. Rho finds the distance between two points and two point inputs as parameters. Sigma takes a query point, current position, and current robot angle to find the distance from the robot to a query point taking into account both distance and orientation, with the distance being lengthened if the robot is not facing the query point. FreePoint finds a random, unoccupied point on the map boundaries. NearestNeighbor finds the closest point to a query point within the RRT vertices taking the vertices and a query point as parameters. New_state performs several actions, but primarily finds the closest point to the query point from nine robot control action final destinations, starting from the nearestNeighbor output point. It also creates and expands arrays storing the RRT vertices, RRT robot class current characteristics, and the path of the robot (start and end points of each edge of the RRT). Extend simply takes a query point, nearestNeighbor, and new_state and combines them to create a fully functional RRT algorithm, drawing edges in pygame as it goes. PathV7 creates the final path from the start to the goal with no parameters.
	
NavMain uses the functions and a class created in the other file(s) within loops to iterate over the provided input files and output the values required for all of the parts of the project. NavMain contains the code to show the RRT, path, and robot by importing the pygame library, meeting the requirements for Part 4. For Part 1 it takes a wall array created within NavBase and draws lines connecting each start and end point within the array to draw the map in the window limits. Then it checks each input provided in the "Input1" file and checks if an input point will collide with a wall by seeing if its distance from the wall is less than the robot radius. To check if there is a collision it utilizes the collision function, located in NavBase. It then prints the number of points that collide with the wall, divided by the total number of points, showing the percentage of the map that the robot cannot drive on, which outputs 43.4%. For Part 2 it takes the inputs from the provided "input2" file and moves the robot over a 0.2 second time interval with the given angular and linear velocity. Using the collision function, it checks to see if the start or end position of the robot is colliding with a wall, if so it discards that input. It then finds the total distance traveled from each of the qualifying inputs (non-colliding inputs). The final distance found is 791.1 meters (m). For Part 3 the inputs from the provided "input3" file were checked to see how many of the input points the robot visited while creating the RRT. To confirm if the point will be marked as "visited" the following equation was implemented: 
	
                                (1)     f(q, pos, theta) = ||pos - q|| + ||pos - q|| * (1 - cos(theta))
				
With pos, q, and theta representing the robots current position, the input point, and the angle from positive x-axis, respectively. If this equation with an output a value less than the robot radius, the point was marked visited, otherwise it was discarded/ignored. Then taking the total visited points divided by the total input points, a ratio of free space visited was created, ranging from ~0.9 - 0.97. Last, NavMain animates the robot moving on the final path from start to goal.
