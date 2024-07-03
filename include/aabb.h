
#ifndef AABB_H
#define AABB_H

#include <iostream>
#include <Eigen/Geometry>

template <typename S>
struct AABB
{
    Eigen::Matrix<S, 3, 1> min_;
    Eigen::Matrix<S, 3, 1> max_;
};

#endif