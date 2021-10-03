#ifndef BASIC_SHAPES_H
#define BASIC_SHAPES_H

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Point.h>
using namespace std;

void getpts(string path, vector<float>& lines);

void get_bormal(geometry_msgs::Point normal_end, geometry_msgs::Point pt_0, geometry_msgs::Point pt_1, geometry_msgs::Point pt_2);

#endif