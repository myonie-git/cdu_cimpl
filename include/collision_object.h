// include/transform.h
#ifndef COLLISION_OBJECT_H
#define COLLISION_OBJECT_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "aabb.h"


// CollisionObject 类定义
template <typename S>
class CollisionObject
{
public:    

    CollisionObject(const Vector3<S>& center, S radius, const Eigen::Transform<S, 3, Eigen::Isometry>& transform)
        : aabb_center(center), aabb_radius(radius), t(transform)
    {
    }

    CollisionObject(const AABB<S>& aabb_) : aabb(aabb_)
    {
    }


    void computeAABB();

    //the position of the center of aabb
    Vector3<S> aabb_center;

    //the radius of aabb 
    S aabb_radius;

    //the transform matrix of AABB (can be zero)
    Eigen::Transform<S, 3, Eigen::Isometry> t;

    //the value of collision obj
    AABB<S> aabb;
};

#endif // TRANSFORM_H
