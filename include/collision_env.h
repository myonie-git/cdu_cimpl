#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H

#include "aabb.h"
#include "bvh_node_base.h"
#include "collision_object.h"
#include "bvh_tree.h"


//定义环境状态
template <typename BV>
class CollisionEnv{

public:
    
    CollisionEnv();

    // void InitTree(std::vector<NodeType*>& leaves); //输入一系列的碰撞物,以建树
    void InitTree(const std::vector<CollisionObject<S>*>& other_objs);
    void clear();
    void collide(CollisionObject<S>* obj, void* cdata) const;


};

#endif