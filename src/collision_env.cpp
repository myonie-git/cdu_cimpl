#include "collision_env.h"
#include <vector>

template <typename S>
CollisionEnv<S>::CollisionEnv(){

}

template <typename S>
void CollisionEnv<S>::InitTree(const std::vector<CollisionObject<S>*>& other_objs){
    if(dtree.size() > 0){
        assert(0);
    }
    else
    {
        std::vector<AABBNode*> leaves(other_objs.size());
        table.rehash(other_objs.size());
        for(size_t i = 0, size = other_objs.size(); i < size; ++i)
        {
            AABBNode* node = new AABBNode; // node will be managed by the dtree
            node->bv = other_objs[i]->getAABB();
            node->parent = nullptr;
            node->children[1] = nullptr;
            node->data = other_objs[i];
            table[other_objs[i]] = node;
            leaves[i] = node;
        }
        dtree.init(leaves);
    }
}

template <typename S>
void CollisionEnv<S>::clear(){
    dtree.clear();
    table.clear();
}


template <typename S>
bool collisionRecurse(typename CollisionEnv<S>::AABBNode* root,  CollisionObject<S>* query, void* cdata){
    if(root->isLeaf()){
        if(!root->bv.overlap(query->getAABB())) return false;
        return true;
    }

    if(!root->bv.overlap(query.aabb)) return false;
    
    if(collisionRecurse(root->children[0], query, cdata))
        return true;

    if(collisionRecurse(root->children[1], query, cdata))
        return true;

    return false;

}

template <typename S>
void CollisionEnv<S>::collide(CollisionObject<S>* obj, void* cdata) const{
    if(dtree.size() == 0) return;
    collisionRecurse(dtree.getRoot(), obj, cdata);
}

template <typename S>
size_t CollisionEnv<S>::size() const{
    return dtree.size();
}