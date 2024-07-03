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

    void computeAABB();

    Vector3<S> aabb_center;
    S aabb_radius;
    Eigen::Transform<S, 3, Eigen::Isometry> t;
    AABB<S> aabb;
};


#endif // TRANSFORM_H
