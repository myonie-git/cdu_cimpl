// src/collision_object.cpp.cpp

#include "collision_object.h"
#include "aabb.h"
#include <iostream>
#include "types.h"


template <typename S>
void CollisionObject<S>::computeCollisionAABB(){
    //计算变化后的中心点
    Vector3<S> center = t * aabb_center;

    Vector3<S> delta = Vector3<S>::Constant(aabb_radius);

    aabb.min_ = center - delta;
    aabb.max_ = center + delta;
}

template <typename S>
void CollisionObject<S>::computeCollisionOBB(){
    
}