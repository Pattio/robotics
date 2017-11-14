#ifndef Geometry_hpp
#define Geometry_hpp

enum FacingDirection { east, north, south, west };
// Function to get robot direction
FacingDirection getDirection(double angle);
// Convienient function for rounding values
double round(double x);
// Function to calculate angle between target and robot
double angle(double currentX, double currentY, double targetX, double targetY);

// Convienient robot sizes
#define robot_width 0.1
#define robot_height 0.1
#define robot_hypo 0.14
#define treshold 0.6

#endif