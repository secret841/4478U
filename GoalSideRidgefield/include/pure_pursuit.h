#include "vex.h"
#include <vector> 
#include "motor_config.h"
#pragma once

using namespace vex;


struct Point 
{
    double x; 
    double y; 
};

std::vector<Point> points; 

//Linear Interpolation function
double interpolate(double P0, double P1, double t)
{
    double p = (1 - t) * P0 + (t * P1);
    return p;  
}

//Bezier Curve Generator - Change back to 1.0 instead of 1.01 if needed
void bezier(Point p0, Point p1, Point p2)
{
    for (int i = 0; i < 1.01; i += 0.01)
    {
        double xa = interpolate(p0.x, p1.x, i);
        double xb = interpolate(p1.x, p2.x, i); 

        double ya = interpolate(p0.y, p1.y, i); 
        double yb = interpolate(p1.y, p2.y, i);

        double interpX = interpolate(xa, xb, i);
        double interpY = interpolate(ya, yb, i);

        //Generates our list of points using vectors
        points.push_back({interpX, interpY}); 


    }
}
